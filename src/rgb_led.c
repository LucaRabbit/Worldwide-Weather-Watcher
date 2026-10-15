#include "rgb_led.h"
#include "board_pins.h"
#include "stm32l4xx_hal.h"

#include <stddef.h>   /* NULL */

/* Dernière couleur envoyée (mémorisée car le P9813 ne se relit pas) */
static volatile uint8_t couleur_r = 0U;
static volatile uint8_t couleur_g = 0U;
static volatile uint8_t couleur_b = 0U;

/* ---------- Fonctions bas niveau (privées) ---------- */

/* Courte temporisation : quelques centaines de ns, largement suffisant pour le P9813 */
static void RGB_Delay(void)
{
    for (volatile uint32_t i = 0U; i < 10U; i++)
    {
        __NOP();
    }
}

/* Une impulsion d'horloge : la donnée est échantillonnée sur le front montant */
static void RGB_ClkPulse(void)
{
    HAL_GPIO_WritePin(RGB_CLK_PORT, RGB_CLK_PIN, GPIO_PIN_RESET);
    RGB_Delay();
    HAL_GPIO_WritePin(RGB_CLK_PORT, RGB_CLK_PIN, GPIO_PIN_SET);
    RGB_Delay();
}

/* Envoi d'un octet, bit de poids fort en premier */
static void RGB_SendByte(uint8_t octet)
{
    for (uint8_t i = 0U; i < 8U; i++)
    {
        HAL_GPIO_WritePin(RGB_DATA_PORT, RGB_DATA_PIN,
                          (octet & 0x80U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        RGB_ClkPulse();
        octet <<= 1;
    }
}

/* Trame de début ou de fin : 32 bits à 0 */
static void RGB_SendZeros(void)
{
    for (uint8_t i = 0U; i < 4U; i++)
    {
        RGB_SendByte(0x00U);
    }
}

/* Octet de contrôle : "11" puis les bits 7-6 inversés de B, G, R */
static uint8_t RGB_ControlByte(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t ctrl = 0xC0U;

    if ((b & 0x80U) == 0U) ctrl |= 0x20U;
    if ((b & 0x40U) == 0U) ctrl |= 0x10U;
    if ((g & 0x80U) == 0U) ctrl |= 0x08U;
    if ((g & 0x40U) == 0U) ctrl |= 0x04U;
    if ((r & 0x80U) == 0U) ctrl |= 0x02U;
    if ((r & 0x40U) == 0U) ctrl |= 0x01U;

    return ctrl;
}

/* ---------- API publique ---------- */

void RGB_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();   /* DIN = PA8  */
    __HAL_RCC_GPIOB_CLK_ENABLE();   /* CIN = PB10 */

    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = RGB_DATA_PIN;
    HAL_GPIO_Init(RGB_DATA_PORT, &gpio);

    gpio.Pin = RGB_CLK_PIN;
    HAL_GPIO_Init(RGB_CLK_PORT, &gpio);

    HAL_GPIO_WritePin(RGB_DATA_PORT, RGB_DATA_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RGB_CLK_PORT,  RGB_CLK_PIN,  GPIO_PIN_RESET);

    RGB_Off();
}

void RGB_SetColor(uint8_t r, uint8_t g, uint8_t b)
{
    RGB_SendZeros();                        /* trame de début */
    RGB_SendByte(RGB_ControlByte(r, g, b));
    RGB_SendByte(b);
    RGB_SendByte(g);
    RGB_SendByte(r);
    RGB_SendZeros();                        /* trame de fin (verrouillage) */

    couleur_r = r;
    couleur_g = g;
    couleur_b = b;
}

void RGB_Off(void)
{
    RGB_SetColor(0U, 0U, 0U);
}

void RGB_GetColor(uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (r != NULL) *r = couleur_r;
    if (g != NULL) *g = couleur_g;
    if (b != NULL) *b = couleur_b;
}

uint8_t RGB_IsOn(void)
{
    return (couleur_r | couleur_g | couleur_b) != 0U;
}