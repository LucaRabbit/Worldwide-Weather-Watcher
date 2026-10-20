#include "console.h"
#include "grove_gps_air530z.h" // UART_AutreRxCallback / UART_AutreErrorCallback
#include "stm32l4xx_hal.h"
#include <string.h>

#define CONSOLE_TAILLE_LIGNE  64U

extern UART_HandleTypeDef huart2; // setup.c

static uint8_t       octet_recu;
static char          ligne_en_cours[CONSOLE_TAILLE_LIGNE];
static uint16_t      longueur = 0U;
static char          ligne_prete[CONSOLE_TAILLE_LIGNE];
static volatile bool ligne_disponible = false;

// Traitement d'un caractère reçu (sous interruption)
static void traiter_octet(uint8_t c)
{
    if (c == '\r' || c == '\n')
    {
        // Fin de ligne (une ligne vide est ignorée ; une ligne non lue n'est pas écrasée)
        if (longueur > 0U && !ligne_disponible)
        {
            ligne_en_cours[longueur] = '\0';
            memcpy(ligne_prete, ligne_en_cours, (size_t)longueur + 1U);
            ligne_disponible = true;
        }
        longueur = 0U;
    }
    else if (c == 0x08U || c == 0x7FU) // retour arrière
    {
        if (longueur > 0U) longueur--;
    }
    else if (longueur < (CONSOLE_TAILLE_LIGNE - 1U))
    {
        ligne_en_cours[longueur++] = (char)c;
    }
}

void Console_Init(void)
{
    HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_UART_Receive_IT(&huart2, &octet_recu, 1);
}

bool Console_LireLigne(char *ligne, size_t taille)
{
    if (ligne == 0 || taille == 0U || !ligne_disponible)
        return false;

    __disable_irq();
    strncpy(ligne, ligne_prete, taille - 1U);
    ligne[taille - 1U] = '\0';
    ligne_disponible = false;
    __enable_irq();

    return true;
}

void Console_Vider(void)
{
    ligne_disponible = false;
}

// INTERRUPTIONS

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

void UART_AutreRxCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        traiter_octet(octet_recu);
        HAL_UART_Receive_IT(&huart2, &octet_recu, 1); // réarmement pour le caractère suivant
    }
}

void UART_AutreErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        longueur = 0U;
        HAL_UART_Receive_IT(&huart2, &octet_recu, 1);
    }
}
