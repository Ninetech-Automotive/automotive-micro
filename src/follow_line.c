/*
 * follow_line.c
 *
 *  Created on: 23.05.2025
 *      Author: paris
 */

#include "follow_line.h"

FollowLine_state_t FollowLine(void)
/* Diese Funktion folgt der Linie "nicht blockierend". Da diese Funktion nicht blockierend ist,
 *  darf im Main Loop kein delay eingefügt werden! -> beeinflusst Regelung!!
 *
 * Input: none
 * Output: none
 *
 * */
{

	 // Zwischenspeicher Werte Linesensor
	 uint16_t line_sns_values[5]; 	// L, ML, M, MR, R / 1 = auf Linie, 0 = abseits der Linie

	 // Regelparameter
	 int8_t mL = 0; 	// Messgrösse für linen Motor
	 int8_t eL = 0;		// Regeldifferenz linker Motor
	 int8_t duL = 0;	// Delta stellgrösse linker Motor -> 0%...100%
	 int8_t mR = 0; 	// Messgrösse für rechten Motor
	 int8_t eR = 0;		// Regeldifferenz rechter Motor
	 int8_t duR = 0;	// Delta stellgrösse rechter Motor -> 0%...100%

	 // Ermittlung Linesensorwerte
	 Linesensor(line_sns_values);

	 // Stop Conditions
	 if (line_sns_values[0] && line_sns_values[1] && line_sns_values[2] && line_sns_values[3] && line_sns_values[4]) // Auf Wegpunkt
	 {
		 MotR_Control(0);
		 MotR_Control(0);
		 return ON_POINT;
	 }

	 // Ermittlung Messgrösse Regelung Links
	 /*
	  * Wenn zu weit rechts -> Delta negativ! -> e = 0-(+2) = -2
	  * Wenn zu weit links -> Delta positiv! -> e = 0-(-2) = +2
	  * */
	 if (line_sns_values[0] == 1) // L
	 {
		 mL = 2;
	 }
	 else if (line_sns_values[1] == 1) // ML
	 {
		 mL= 1;
	 }
	 else if (line_sns_values[2] == 1) // M
	 {
		 mL = 0;
	 }
	 else if (line_sns_values[3] == 1) // MR
	 {
		 mL = -1;
	 }
	 else if (line_sns_values[4] == 1) // R
	 {
		 mL = -2;
	 }

	 // Ermittlung Messgrösse Regelung rechts
	 /*
	  * Wenn zu weit rechts -> Delta positiv! -> e = 0-(-2) = +2
	  * Wenn zu weit links -> Delta negativ! -> e = 0-(+2) = -2
	  * */
	 if (line_sns_values[0] == 1) // L
	 {
		 mR = -2;
	 }
	 else if (line_sns_values[1] == 1) // ML
	 {
		 mR= -1;
	 }
	 else if (line_sns_values[2] == 1) // M
	 {
		 mR = 0;
	 }
	 else if (line_sns_values[3] == 1) // MR
	 {
		 mR = 1;
	 }
	 else if (line_sns_values[4] == 1) // R
	 {
		 mR = 2;
	 }

	 // Berechnung Regeldifferenz
	 eL = r - mL;
	 eR = r - mR;

	 // Berechnung des Delta für die Stellgrösse
	 duL = eL * Kp;
	 duR = eR * Kp;

	 // Stellgrössenbegrenzung
	 if (duL > (100-Vg))	// 100% - Vg(%) = du max
	 {
		 duL = (100-Vg);
	 }
	 else if (duL < -(Vg))	// -Vg(%) = du min
	 {
		 duL = -(Vg);
	 }
	 if (duR > (100-Vg))
	 {
		 duR = (100-Vg);
	 }
	 else if (duR < -(Vg))
	 {
		 duR = -(Vg);
	 }

	 // Einstellung neue Stellgrösse
	 MotR_Control(Vg + duR); // Groundspeed Vg + delta Stellgrösse
	 MotL_Control(Vg + duL); // Groundspeed Vg + delta Stellgrösse

	 return(RUNNING);

}

