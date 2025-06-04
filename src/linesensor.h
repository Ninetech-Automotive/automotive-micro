/*
 * linesensor.h
 *
 *  Created on: 09.05.2025
 *      Author: paris
 */

#ifndef LINESENSOR_H_
#define LINESENSOR_H_

// *** Notwendige Includes ***
#include <stdint.h>
#include "fsl_common.h"
#include "fsl_i2c.h"	// for I2C


// *** Defines ***
// Linesensor
#define ADS7828_ADDR    0x48    // 8-Bit Slave-Adresse von ADC auf Linesensor Board (ADS7828
#define cmd_ch0 (0b10000000)	// Command-Byte CH0 -> sig L
#define cmd_ch1 (0b11000000)	// Command-Byte CH1 -> sig ML
#define cmd_ch2 (0b10010000)	// Command-Byte CH2 -> sig_M
#define cmd_ch3 (0b11010000)	// Command-Byte CH3 -> sig_MR
#define cmd_ch4 (0b10100000)	// Command-Byte CH4	-> sig_R

// Linesensor Linienwerte Offset
/* Da die 5-verschiedenen ADC-Werte vom Liniensensor leicht voneinander abweichen (Toleranz), wird
 * hier zusätzlich ein Offset bestimmt, damit alle Werte unter denselben Voraussetzungen die selben Werte
 * aufweisen. Dazu wurde der Liniensensor auf einen weissen Tisch gestellt,dass der Liniensensor parallel
 * zur weissen unterlage zeigt. Danach wurde der offset so angepasst, damit jeweils der Mittelwert erreicht wird
 * */
#define offset_ch0 (11)
#define offset_ch1 (-8)
#define offset_ch2 (7)
#define offset_ch3 (-13)
#define offset_ch4 (4)

// Linesensor Einstellung Schwellwerte
/*
 * Ab diesem Wert Entscheidet sich der Liniensensor daführ, dass sich der Punkt auf der weissen Linie befindet
 */
#define On_Line_Threshold (100)


// *** Funktionsprototypen ***
status_t ADS7828_ReadChannel(uint8_t command, uint16_t *result);
void Linesensor(uint16_t *ptr);


#endif /* LINESENSOR_H_ */
