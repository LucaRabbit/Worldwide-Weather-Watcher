#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdbool.h>
#include <stddef.h>

/*
 * Console série (USART2, moniteur série du PC) : réception par interruption.
 * Les caractères sont reçus en arrière-plan, la lecture d'une ligne ne bloque jamais.
 */

void Console_Init(void);

// true si une ligne complète a été reçue (terminée par Entrée), copiée sans "\r\n" dans ligne
bool Console_LireLigne(char *ligne, size_t taille);

// Oublie la ligne éventuellement en attente (tapée hors du mode configuration)
void Console_Vider(void);

#endif
