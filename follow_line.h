/*
 * follow_line.h
 *
 *  Created on: 23.05.2025
 *      Author: paris
 */

#ifndef FOLLOW_LINE_H_
#define FOLLOW_LINE_H_

// *** Notwendige Includes für dieses Headerfile ***
#include <stdbool.h>
#include <stdint.h>
#include "motor.h"
#include "linesensor.h"

// *** Defines ***
// Regelung
#define Vg (77)		// Groundspeed Linienverfolgung in % (Tastgrad Motoren)
#define r (0)		// Führungsgrösse -> 0 = Position in der Mitte v. Linesensor -> Positionsregelung
#define Kp (5)		// P-Anteil -> Verstärkung für die DeltaStellgrösse duL & duR

// *** Enum ***
typedef enum  // Rückgabetyp von FollowLine
{
	RUNNING = 0,	// Wenn Linienverfolgung noch läuft
	ON_POINT = 1,	// Wenn Endpunkt erreicht
	BARRIER = 2,	// Wenn Hinderniss im Weg
	DEADEND = 3,	// Wenn Pilone auf Weg
}FollowLine_state_t;

// *** Funktionsprototypen ***
FollowLine_state_t FollowLine(void);

#endif /* FOLLOW_LINE_H_ */
