#include "grove_gps_air530z.h"
#include "board_pins.h"
#include <string.h>
#include <stdlib.h>

#define GPS_TAILLE_TRAME  96U

static UART_HandleTypeDef gps_uart;

// Réception en cours (remplie octet par octet sous interruption)
static uint8_t  octet_recu;
static char     trame_en_cours[GPS_TAILLE_TRAME];
static uint16_t longueur_en_cours = 0U;

// Dernière trame GGA complète (écrite sous interruption, lue par le programme principal)
static char              derniere_gga[GPS_TAILLE_TRAME];
static volatile bool     gga_disponible = false;
static volatile uint32_t gga_instant = 0U; // HAL_GetTick() à la réception

// FONCTIONS INTERNES

static double nmea_to_decimal(const char *value, const char hemisphere)
{
    if (value == 0 || value[0] == '\0') return 0.0;

    double raw = atof(value);
    int degrees = (int)(raw / 100.0);
    double minutes = raw - (degrees * 100.0);
    double decimal = degrees + (minutes / 60.0);

    if (hemisphere == 'S' || hemisphere == 'W')
        decimal = -decimal;

    return decimal;
}

// Découpe et analyse une trame GGA :
// $GPGGA,time,lat,N,lon,E,fix,sats,hdop,alt,M,...
static bool analyser_gga(const char *trame, GPS_Data *gps)
{
    strncpy(gps->raw_sentence, trame, sizeof(gps->raw_sentence) - 1);
    gps->raw_sentence[sizeof(gps->raw_sentence) - 1] = '\0';

    char copy[GPS_TAILLE_TRAME];
    strncpy(copy, trame, sizeof(copy));
    copy[sizeof(copy) - 1] = '\0';

    // Découpage manuel (strtok fusionnerait les champs vides d'une trame sans fix)
    char *fields[16] = {0};
    int i = 0;
    char *p = copy;

    fields[i++] = p;
    while (*p != '\0' && i < 16)
    {
        if (*p == ',')
        {
            *p = '\0';
            fields[i++] = p + 1;
        }
        p++;
    }

    if (i < 8) return false;

    gps->latitude    = nmea_to_decimal(fields[2], fields[3][0] ? fields[3][0] : 'N');
    gps->longitude   = nmea_to_decimal(fields[4], fields[5][0] ? fields[5][0] : 'E');
    gps->fix_quality = (uint8_t)atoi(fields[6]);
    gps->satellites  = (uint8_t)atoi(fields[7]);
    gps->valid       = gps->fix_quality > 0;

    return true;
}

// Traitement d'un octet reçu (appelé sous interruption)
static void traiter_octet(uint8_t c)
{
    if (c == '\n')
    {
        trame_en_cours[longueur_en_cours] = '\0';

        if (strncmp(trame_en_cours, "$GPGGA", 6) == 0 || strncmp(trame_en_cours, "$GNGGA", 6) == 0)
        {
            memcpy(derniere_gga, trame_en_cours, (size_t)longueur_en_cours + 1U);
            gga_instant = HAL_GetTick();
            gga_disponible = true;
        }

        longueur_en_cours = 0U;
    }
    else if (c == '$')
    {
        // Début de trame : on repart de zéro (resynchronisation après un octet perdu)
        trame_en_cours[0] = '$';
        longueur_en_cours = 1U;
    }
    else if (c != '\r' && longueur_en_cours < (GPS_TAILLE_TRAME - 1U))
    {
        trame_en_cours[longueur_en_cours++] = (char)c;
    }
}

// FONCTIONS PUBLIQUES

void GroveGPS_Init(void)
{
    /*
     * Port Grove A0 : Signal1=PA0=GPS_TX, Signal2=PA1=GPS_RX.
     * Le module Air530Z émet sur Signal1 (PA0).
     * Pin swap UART4 activé : PA0=UART4_RX, PA1=UART4_TX.
     * L'init GPIO est centralisée dans HAL_UART_MspInit (setup.c).
     */
    gps_uart.Instance = UART4;
    gps_uart.Init.BaudRate = 9600;
    gps_uart.Init.WordLength = UART_WORDLENGTH_8B;
    gps_uart.Init.StopBits = UART_STOPBITS_1;
    gps_uart.Init.Parity = UART_PARITY_NONE;
    gps_uart.Init.Mode = UART_MODE_TX_RX;
    gps_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    gps_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    gps_uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    gps_uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
    gps_uart.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;

    HAL_UART_Init(&gps_uart);

    // Interruption de réception UART4
    HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);

    HAL_UART_Receive_IT(&gps_uart, &octet_recu, 1);
}

bool GroveGPS_ReadGGA(GPS_Data *gps, unsigned int age_max_ms)
{
    if (gps == 0) return false;

    char copie[GPS_TAILLE_TRAME];
    uint32_t instant;

    // Copie protégée : l'interruption ne doit pas modifier la trame pendant la lecture
    __disable_irq();
    bool disponible = gga_disponible;
    instant = gga_instant;
    memcpy(copie, derniere_gga, sizeof(copie));
    __enable_irq();

    if (!disponible || (HAL_GetTick() - instant) > age_max_ms)
        return false; // aucune trame récente : GPS débranché ou muet

    return analyser_gga(copie, gps);
}

// INTERRUPTIONS

void UART4_IRQHandler(void)
{
    HAL_UART_IRQHandler(&gps_uart);
}

/*
 * La HAL n'autorise qu'un seul HAL_UART_RxCpltCallback pour tout le programme.
 * Les autres UART (console USART2) sont transmis à UART_AutreRxCallback(), que la console redéfinit.
 */
__attribute__((weak)) void UART_AutreRxCallback(UART_HandleTypeDef *huart)    { (void)huart; }
__attribute__((weak)) void UART_AutreErrorCallback(UART_HandleTypeDef *huart) { (void)huart; }

// Appelé par la HAL à chaque octet reçu
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        traiter_octet(octet_recu);
        HAL_UART_Receive_IT(&gps_uart, &octet_recu, 1); // réarmement pour l'octet suivant
    }
    else
    {
        UART_AutreRxCallback(huart);
    }
}

// Erreur de réception (débordement, bruit au branchement...) : on relance la réception
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        longueur_en_cours = 0U;
        HAL_UART_Receive_IT(&gps_uart, &octet_recu, 1);
    }
    else
    {
        UART_AutreErrorCallback(huart);
    }
}
