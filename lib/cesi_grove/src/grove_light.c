#include "grove_light.h"
#include "board_pins.h"

ADC_HandleTypeDef hadc1;

void GroveLight_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = LIGHT_GPIO_PIN;
    gpio.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LIGHT_GPIO_PORT, &gpio);

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_10B; // 10 bits pour avoir une plage de 0 à 1023
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode = DISABLE;

    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = LIGHT_ADC_CHANNEL;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

uint16_t GroveLight_ReadRaw(void)
{
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
        return 0xFFFF;

    if (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK)
    {
        HAL_ADC_Stop(&hadc1);
        return 0xFFFF;
    }

    uint16_t value = (uint16_t)HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);

    return value;
}

// DÉTECTION DU CAPTEUR (technique de précharge)

#define LIGHT_ECART_MAX        50U   // écart max entre les 2 mesures si le capteur est branché (sur 1023)
#define LIGHT_PRECHARGE_BOUCLE 200   // environ 250 µs à 4 MHz : temps de charger la ligne

// Force la ligne à 0 V (pull-down) ou 3,3 V (pull-up), puis mesure
static uint16_t lire_apres_precharge(uint32_t tirage)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = LIGHT_GPIO_PIN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    // 1. Entrée numérique avec résistance de tirage : charge la ligne
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = tirage;
    HAL_GPIO_Init(LIGHT_GPIO_PORT, &gpio);

    for (volatile int i = 0; i < LIGHT_PRECHARGE_BOUCLE; i++);

    // 2. Retour en analogique, puis mesure immédiate
    gpio.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LIGHT_GPIO_PORT, &gpio);

    return GroveLight_ReadRaw();
}

// Branché : l'ampli du capteur impose sa tension, les 2 mesures sont proches.
// Débranché : la ligne flottante garde la précharge, les 2 mesures sont très éloignées.
bool GroveLight_EstConnecte(void)
{
    uint16_t bas  = lire_apres_precharge(GPIO_PULLDOWN);
    uint16_t haut = lire_apres_precharge(GPIO_PULLUP);

    if (bas == 0xFFFFU || haut == 0xFFFFU)
        return false;

    uint16_t ecart = (haut > bas) ? (uint16_t)(haut - bas) : (uint16_t)(bas - haut);
    return ecart < LIGHT_ECART_MAX;
}