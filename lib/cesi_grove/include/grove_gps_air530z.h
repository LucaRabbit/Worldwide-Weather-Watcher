#ifndef GROVE_GPS_AIR530Z_H
#define GROVE_GPS_AIR530Z_H

#include <stdbool.h>
#include "cesi_types.h"
#include "stm32l4xx_hal.h"

/*
 * Réception par interruption : les trames NMEA sont reçues en arrière-plan,
 * aucune fonction de ce pilote ne bloque le programme.
 */

void GroveGPS_Init(void);

// Analyse la dernière trame GGA reçue, si elle date de moins de age_max_ms.
// Retourne false si aucune trame récente (GPS débranché ou muet). Ne bloque jamais.
bool GroveGPS_ReadGGA(GPS_Data *gps, unsigned int age_max_ms);

// Callbacks HAL des autres UART (ex. console USART2), à redéfinir par le module concerné
void UART_AutreRxCallback(UART_HandleTypeDef *huart);
void UART_AutreErrorCallback(UART_HandleTypeDef *huart);

#endif
