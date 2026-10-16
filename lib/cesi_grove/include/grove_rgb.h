#ifndef GROVE_RGB_H
#define GROVE_RGB_H

/*
 * Pilote de la Grove Chainable RGB LED (circuit P9813).
 * Protocole série 2 fils en bit-banging : CIN (horloge) + DIN (données).
 * Broches définies dans board_pins.h (RGB_CLK_*, RGB_DATA_*).
 */

#include <stdint.h>

// Couleurs prédéfinies (R, G, B) : GroveRGB_SetColor(GROVE_RGB_GREEN)
#define GROVE_RGB_OFF      0U,   0U,   0U
#define GROVE_RGB_WHITE  255U, 255U, 255U
#define GROVE_RGB_RED    255U,   0U,   0U
#define GROVE_RGB_GREEN    0U, 255U,   0U
#define GROVE_RGB_BLUE     0U,   0U, 255U
#define GROVE_RGB_YELLOW 255U, 255U,   0U
#define GROVE_RGB_ORANGE 255U, 128U,   0U

// Configure les GPIO et éteint la LED
void GroveRGB_Init(void);

// Applique une couleur (0..255 par composante)
void GroveRGB_SetColor(uint8_t r, uint8_t g, uint8_t b);

// Éteint la LED
void GroveRGB_Off(void);

#endif