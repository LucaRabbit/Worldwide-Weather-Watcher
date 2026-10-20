#ifndef RTC_INTERNE_H
#define RTC_INTERNE_H

#include <stdbool.h>
#include "cesi_types.h"

/*
 * Horloge RTC interne du STM32L476 (remplace la Grove RTC DS1307 incompatible)
 * Même interface que grove_rtc_ds1307.h
 */

bool RTCInterne_Init(void);
bool RTCInterne_GetDateTime(RTC_DateTime *dt);
bool RTCInterne_SetDateTime(const RTC_DateTime *dt);
bool RTCInterne_SurQuartz(void); // true = quartz LSE 32,768 kHz (précis), false = oscillateur LSI (secours)

#endif