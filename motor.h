/*
 * motor.h
 *
 *  Created on: 21.05.2025
 *      Author: paris
 */

#ifndef MOTOR_H_
#define MOTOR_H_

// *** Notwendige Includes ***
#include <stdbool.h>
#include "MK22F51212.h" // für Initialisierung
#include "linesensor.h" // für Scan

// *** Defines ***
// Motoren
/* Folgende Defines werden für die Funktion MotR_Direction & MotL_Direction als übergabeparameter verwendet
 * */
#define Mot_Setup_Forward (false)
#define Mot_Setup_Backward (true)
#define Turn_Direction_Left	(false)
#define Turn_Direction_Right (true)

// *** Viariablen ***
extern unsigned int motl_enc_pos; // Globale Variable in Main


// *** Funktionsprototypen ***
void Init_Motorsteuerung(void);
void MotR_Direction(bool direction);
void MotR_Control(uint8_t speed);
void MotL_Direction(bool direction);
void MotL_Control(uint8_t speed);
void Turn(bool direction, unsigned int degrees);
void Drive_Straith(bool direction, unsigned int distance);
void Scan(void);


#endif /* MOTOR_H_ */
