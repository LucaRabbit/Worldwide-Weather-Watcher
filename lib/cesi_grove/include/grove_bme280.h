#ifndef GROVE_BME280_H
#define GROVE_BME280_H

#include <stdbool.h>
#include "cesi_types.h"

bool GroveBME280_Init(void);
bool GroveBME280_ReadChipId(uint8_t *chip_id);
bool GroveBME280_ReadEnvironment(Env_Data *data); // gas_resistance_ohm toujours à 0 (pas de capteur de gaz)

#endif