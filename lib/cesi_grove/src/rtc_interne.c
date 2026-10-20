#include "rtc_interne.h"
#include "stm32l4xx_hal.h"
#include <string.h>
#include <stdlib.h>

// CONFIGURATION

#define RTC_MAGIC           0x32F2U  // écrit en registre de sauvegarde une fois l'horloge réglée

// Prédiviseurs : f_RTC / ((ASYNC + 1) * (SYNC + 1)) = 1 Hz
#define RTC_ASYNC_PREDIV    127U
#define RTC_SYNC_PREDIV_LSE 255U     // 32 768 Hz / (128 * 256) = 1 Hz
#define RTC_SYNC_PREDIV_LSI 249U     // 32 000 Hz / (128 * 250) = 1 Hz

static RTC_HandleTypeDef hrtc;
static bool rtc_sur_lse = false;

// FONCTIONS INTERNES

// Jour de la semaine (algorithme de Sakamoto) : 1 = lundi ... 7 = dimanche
static uint8_t calculer_jour_semaine(uint16_t annee, uint8_t mois, uint8_t jour)
{
    static const int decalage[12] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    int a = annee;

    if (mois < 3) a--;

    int j = (a + a / 4 - a / 100 + a / 400 + decalage[mois - 1] + jour) % 7; // 0 = dimanche

    return (uint8_t)((j == 0) ? 7 : j);
}

// Date et heure de compilation du fichier, utilisées comme réglage initial
static void date_compilation(RTC_DateTime *dt)
{
    static const char noms_mois[12][4] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                          "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    const char *date  = __DATE__; // "Oct 20 2026" (jour complété par un espace si < 10)
    const char *heure = __TIME__; // "14:03:27"

    dt->month = 1;
    for (uint8_t i = 0; i < 12; i++)
    {
        if (strncmp(date, noms_mois[i], 3) == 0)
        {
            dt->month = (uint8_t)(i + 1);
            break;
        }
    }

    dt->day     = (uint8_t)(((date[4] == ' ') ? 0 : (date[4] - '0')) * 10 + (date[5] - '0'));
    dt->year    = (uint16_t)atoi(&date[7]);
    dt->hours   = (uint8_t)((heure[0] - '0') * 10 + (heure[1] - '0'));
    dt->minutes = (uint8_t)((heure[3] - '0') * 10 + (heure[4] - '0'));
    dt->seconds = (uint8_t)((heure[6] - '0') * 10 + (heure[7] - '0'));
    dt->dayOfWeek = calculer_jour_semaine(dt->year, dt->month, dt->day);
}

// Choix de la source d'horloge : quartz LSE de la Nucleo, sinon oscillateur interne LSI
static bool configurer_source_horloge(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_PeriphCLKInitTypeDef pclk = {0};

    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    osc.OscillatorType = RCC_OSCILLATORTYPE_LSE;
    osc.LSEState = RCC_LSE_ON;
    osc.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&osc) == HAL_OK) // peut prendre jusqu'à 5 s si le quartz est absent
    {
        rtc_sur_lse = true;
        pclk.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    }
    else
    {
        osc.OscillatorType = RCC_OSCILLATORTYPE_LSI;
        osc.LSIState = RCC_LSI_ON;

        if (HAL_RCC_OscConfig(&osc) != HAL_OK)
            return false;

        rtc_sur_lse = false;
        pclk.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    }

    pclk.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK)
        return false;

    __HAL_RCC_RTC_ENABLE();
    return true;
}

// FONCTIONS PUBLIQUES

bool RTCInterne_Init(void)
{
    // Le RTC est dans le domaine de sauvegarde : il faut en autoriser l'accès
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    hrtc.Instance = RTC;

    /*
     * Après un simple reset, le RTC continue de compter : on ne le reconfigure pas,
     * sinon l'heure serait perdue. Il n'est configuré qu'après une coupure d'alimentation.
     */
    uint32_t source = __HAL_RCC_GET_RTC_SOURCE();
    bool deja_reglee = (READ_BIT(RCC->BDCR, RCC_BDCR_RTCEN) != 0U)
                    && (source == RCC_RTCCLKSOURCE_LSE || source == RCC_RTCCLKSOURCE_LSI)
                    && (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) == RTC_MAGIC);

    if (deja_reglee)
    {
        rtc_sur_lse = (source == RCC_RTCCLKSOURCE_LSE);

        if (!rtc_sur_lse)
        {
            // Le LSI s'arrête à chaque reset (contrairement au LSE) : il faut le relancer
            RCC_OscInitTypeDef osc = {0};
            osc.OscillatorType = RCC_OSCILLATORTYPE_LSI;
            osc.LSIState = RCC_LSI_ON;
            osc.PLL.PLLState = RCC_PLL_NONE;

            if (HAL_RCC_OscConfig(&osc) != HAL_OK)
                return false;
        }
    }
    else if (!configurer_source_horloge())
    {
        return false;
    }

    hrtc.Init.HourFormat     = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv   = RTC_ASYNC_PREDIV;
    hrtc.Init.SynchPrediv    = rtc_sur_lse ? RTC_SYNC_PREDIV_LSE : RTC_SYNC_PREDIV_LSI;
    hrtc.Init.OutPut         = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutRemap    = RTC_OUTPUT_REMAP_NONE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;

    if (HAL_RTC_Init(&hrtc) != HAL_OK)
        return false;

    if (!deja_reglee)
    {
        // Premier démarrage : réglage provisoire sur la date de compilation
        RTC_DateTime dt;
        date_compilation(&dt);

        if (!RTCInterne_SetDateTime(&dt))
            return false;
    }

    return true;
}

bool RTCInterne_GetDateTime(RTC_DateTime *dt)
{
    if (dt == 0) return false;

    RTC_TimeTypeDef heure = {0};
    RTC_DateTypeDef date = {0};

    // GetTime doit toujours être suivi de GetDate (déverrouillage des registres)
    if (HAL_RTC_GetTime(&hrtc, &heure, RTC_FORMAT_BIN) != HAL_OK) return false;
    if (HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) return false;

    dt->hours     = heure.Hours;
    dt->minutes   = heure.Minutes;
    dt->seconds   = heure.Seconds;
    dt->day       = date.Date;
    dt->month     = date.Month;
    dt->year      = (uint16_t)(2000U + date.Year);
    dt->dayOfWeek = date.WeekDay; // 1 = lundi ... 7 = dimanche

    return true;
}

bool RTCInterne_SetDateTime(const RTC_DateTime *dt)
{
    if (dt == 0) return false;

    // Contrôle des valeurs (le RTC ne stocke que les années 2000 à 2099)
    if (dt->hours > 23 || dt->minutes > 59 || dt->seconds > 59) return false;
    if (dt->month < 1 || dt->month > 12 || dt->day < 1 || dt->day > 31) return false;
    if (dt->year < 2000 || dt->year > 2099) return false;

    RTC_TimeTypeDef heure = {0};
    RTC_DateTypeDef date = {0};

    heure.Hours          = dt->hours;
    heure.Minutes        = dt->minutes;
    heure.Seconds        = dt->seconds;
    heure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    heure.StoreOperation = RTC_STOREOPERATION_RESET;

    date.Date    = dt->day;
    date.Month   = dt->month;
    date.Year    = (uint8_t)(dt->year - 2000U);
    date.WeekDay = calculer_jour_semaine(dt->year, dt->month, dt->day); // recalculé, jamais faux

    if (HAL_RTC_SetTime(&hrtc, &heure, RTC_FORMAT_BIN) != HAL_OK) return false;
    if (HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) return false;

    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, RTC_MAGIC); // horloge réglée
    return true;
}

bool RTCInterne_SurQuartz(void)
{
    return rtc_sur_lse;
}