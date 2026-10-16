#ifndef RGB_LED_H
#define RGB_LED_H

/*
 * Pilote minimal pour la Grove Chainable RGB LED (circuit P9813).
 * Protocole série 2 fils en bit-banging : CIN (horloge) + DIN (données).
 * Broches définies dans board_pins.h (RGB_CLK_*, RGB_DATA_*).
 */

#include <stdint.h>

// Couleurs prêtes à l'emploi (R, G, B)
#define RGB_OFF      0U,   0U,   0U
#define RGB_WHITE  255U, 255U, 255U
#define RGB_RED    255U,   0U,   0U
#define RGB_GREEN    0U, 255U,   0U
#define RGB_BLUE     0U,   0U, 255U
#define RGB_YELLOW 255U, 255U,   0U
#define RGB_ORANGE 255U,  94U,   0U

// Configure les GPIO et éteint la LED
void RGB_Init(void);

// Applique une couleur (0..255 par composante)
void RGB_SetColor(uint8_t r, uint8_t g, uint8_t b);

// Raccourci pour éteindre la LED
void RGB_Off(void);

// Dernière couleur envoyée (la LED ne peut pas être relue)
void RGB_GetColor(uint8_t *r, uint8_t *g, uint8_t *b);

// 1 si la dernière couleur envoyée n'est pas noire, 0 sinon
uint8_t RGB_IsOn(void);

#endif