#ifndef GROVE_BUTTON_H
#define GROVE_BUTTON_H

/*
 * Pilote du Grove Dual Button (bouton 1 = rouge, bouton 2 = vert).
 * - Détection des fronts par interruption EXTI (EXTI15_10 et EXTI3 gérés ici)
 * - Anti-rebond et détection d'appui long traités hors interruption,
 *   dans GroveButton_Process() à appeler à chaque tour de boucle
 * - Boutons actifs à l'état bas (appuyé = GPIO_PIN_RESET), pull-up interne
 * Broches définies dans board_pins.h (BUTTON1_*, BUTTON2_*).
 */

#include <stdbool.h>

#ifndef LONG_PRESS_MS
#define LONG_PRESS_MS   5000U  // appui long = 5 secondes
#endif

#ifndef DEBOUNCE_MS
#define DEBOUNCE_MS     50U    // durée de stabilité exigée après un front
#endif

// Configure les GPIO en interruption et lit l'état initial des boutons
// Un bouton déjà maintenu à l'appel est ignoré jusqu'à son relâchement
void GroveButton_Init(void);

// Anti-rebond + détection des appuis longs
void GroveButton_Process(void);

// État stable (après anti-rebond) : true = appuyé
bool GroveButton1_IsPressed(void);
bool GroveButton2_IsPressed(void);

// true une seule fois, dès que LONG_PRESS_MS est atteint (bouton encore maintenu)
// L'événement est consommé par la lecture
bool GroveButton1_LongPress(void);
bool GroveButton2_LongPress(void);

#endif