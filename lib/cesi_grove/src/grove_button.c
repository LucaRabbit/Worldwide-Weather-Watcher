#include "grove_button.h"
#include "board_pins.h"

#include <stdint.h>

#define BOUTON_ROUGE 0U
#define BOUTON_VERT  1U
#define NB_BOUTONS   2U

// Etat d'un bouton
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;

    volatile uint8_t  front_detecte;  // écrit par l'ISR
    volatile uint32_t instant_front;  // écrit par l'ISR

    uint8_t  etat_stable;             // 1 = appuyé (après anti-rebond)
    uint32_t debut_appui;             // chronomètre de l'appui en cours
    uint8_t  ignorer_appui;           // appui présent à l'init, ignoré jusqu'au relâchement
    uint8_t  appui_long_fait;         // appui long déjà signalé pour l'appui en cours
    uint8_t  evenement_appui_long;    // appui long en attente de lecture
} bouton_t;

static bouton_t boutons[NB_BOUTONS] =
{
    [BOUTON_ROUGE] = { .port = BUTTON1_PORT, .pin = BUTTON1_PIN },
    [BOUTON_VERT]  = { .port = BUTTON2_PORT, .pin = BUTTON2_PIN },
};

// FONCTIONS INTERNES

// Lecture brute : 1 = appuyé (actif à l'état bas)
static uint8_t lire_appuye(const bouton_t *bouton)
{
    return (HAL_GPIO_ReadPin(bouton->port, bouton->pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static void anti_rebond(bouton_t *bouton)
{
    uint32_t maintenant = HAL_GetTick();
    uint8_t  nouvel_etat;

    // Aucun front, ou dernier front trop récent : attente que le signal soit stable
    if ((bouton->front_detecte == 0U) || ((maintenant - bouton->instant_front) < DEBOUNCE_MS))
    {
        return;
    }

    bouton->front_detecte = 0U;

    nouvel_etat = lire_appuye(bouton);

    if (nouvel_etat == bouton->etat_stable)
    {
        return; // rebond sans changement d'état
    }

    bouton->etat_stable = nouvel_etat;

    if (nouvel_etat != 0U)
    {
        // Début de l'appui : démarrage du chronomètre
        bouton->debut_appui     = maintenant;
        bouton->appui_long_fait = 0U;
    }
    else
    {
        // Relâchement : un appui présent à l'init n'est plus ignoré
        bouton->ignorer_appui = 0U;
    }
}

static void verifier_appui_long(bouton_t *bouton)
{
    if ((bouton->etat_stable != 0U) && (bouton->appui_long_fait == 0U) && (bouton->ignorer_appui == 0U)
        && ((HAL_GetTick() - bouton->debut_appui) >= LONG_PRESS_MS))
    {
        bouton->appui_long_fait      = 1U;
        bouton->evenement_appui_long = 1U;
    }
}

static bool consommer_appui_long(bouton_t *bouton)
{
    if (bouton->evenement_appui_long != 0U)
    {
        bouton->evenement_appui_long = 0U;
        return true;
    }
    return false;
}

// API PUBLIQUE

void GroveButton_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Mode  = GPIO_MODE_IT_RISING_FALLING; // interruption sur les deux fronts
    gpio.Pull  = GPIO_PULLUP;                 // actif à l'état bas
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    for (uint8_t i = 0U; i < NB_BOUTONS; i++)
    {
        gpio.Pin = boutons[i].pin;
        HAL_GPIO_Init(boutons[i].port, &gpio);
    }

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0); // PA10 -> EXTI10 (bouton rouge)
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);     // PB3 -> EXTI3 (bouton vert)
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);

    // Etat initial : un bouton déjà maintenu est ignoré jusqu'à son relâchement
    for (uint8_t i = 0U; i < NB_BOUTONS; i++)
    {
        boutons[i].etat_stable = lire_appuye(&boutons[i]);

        if (boutons[i].etat_stable != 0U)
        {
            boutons[i].debut_appui   = HAL_GetTick();
            boutons[i].ignorer_appui = 1U;
        }
    }
}

void GroveButton_Process(void)
{
    for (uint8_t i = 0U; i < NB_BOUTONS; i++)
    {
        anti_rebond(&boutons[i]);
        verifier_appui_long(&boutons[i]);
    }
}

bool GroveButton1_IsPressed(void)  { return boutons[BOUTON_ROUGE].etat_stable != 0U; }
bool GroveButton2_IsPressed(void)  { return boutons[BOUTON_VERT].etat_stable  != 0U; }

bool GroveButton1_LongPress(void)  { return consommer_appui_long(&boutons[BOUTON_ROUGE]); }
bool GroveButton2_LongPress(void)  { return consommer_appui_long(&boutons[BOUTON_VERT]); }

// INTERRUPTIONS

void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(BUTTON1_PIN);
}

void EXTI3_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(BUTTON2_PIN);
}

// Callback HAL : on note seulement l'instant du front, le traitement se fait dans GroveButton_Process()
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t maintenant = HAL_GetTick();

    for (uint8_t i = 0U; i < NB_BOUTONS; i++)
    {
        if (GPIO_Pin == boutons[i].pin)
        {
            boutons[i].instant_front = maintenant;
            boutons[i].front_detecte = 1U;
        }
    }
}