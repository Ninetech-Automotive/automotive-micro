/*
 * servos.h
 *
 *  Created on: 30.05.2025
 *      Author: paris
 */

#ifndef SERVOS_H_
#define SERVOS_H_

// *** Notwendige Includes ***
#include "MK22F51212.h" // für Initialisierung
#include <fsl_common.h> // für Delay funktion

// *** Defines ***
#define ServoSmall_Closed (0)
#define ServoSmall_Opened (1)
#define ServoBig_Front (0)
#define ServoBig_Middle (1)
#define ServoBig_Back (2)
#define ServoBig_Diagonal (3)

// *** Viariablen ***

// *** Funktionsprototypen ***
void ServosInit(void);
void ServoBigControl(uint8_t position);
void ServoSmallControl(uint8_t position);
void ServoBig_MoveSlowly(uint8_t target, uint32_t stepDelayUs);

#endif /* SERVOS_H_ */
