#ifndef GROVE_LIGHT_H
#define GROVE_LIGHT_H

#include <stdbool.h>
#include <stdint.h>

void GroveLight_Init(void);
uint16_t GroveLight_ReadRaw(void);
bool GroveLight_EstConnecte(void);   // false si le capteur est débranché (ligne flottante)

#endif
