/*
 * motor.c
 *
 *  Created on: 21.05.2025
 *      Author: paris
 */

#include "motor.h" // Headerfile

#define TICKS_PER_90 348U

void Init_Motorsteuerung(void)
/*
 * Initialisiert die Motoren:
 * - FTM1 CH0 -> MOTL
 * - FTM1 CH1 -> MOTR
 * - Durch die Funktionen MotR_Direction & MotL_Direction wird die Drehrichtung eingestellt
 * */
{
    // Clock für Port A und Port B aktivieren
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;  // Aktiviert PORTA (für PTA12, PTA13)
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;  // Aktiviert PORTB (für PTB0, PTB1)
    // Clock für FTM1 aktivieren
    SIM->SCGC6 |= SIM_SCGC6_FTM1_MASK;
    // Drehrichtung Motoren
    MotL_Direction(Mot_Setup_Forward); // Default nach vorwärts drehen
    MotR_Direction(Mot_Setup_Forward); // Default nach vorwärts drehen
    // FTM1 stoppen, bevor Setup gemacht wird
    FTM1->SC = 0x00;                    // Stoppe den Timer
    // Counter und MOD-Register setzen
    // PWM-Frequenz = F_CLK / (MOD + 1)
    // F_CLK = ca. 20.97 MHz, MOD = 1049 → ca. 20 kHz PWM
    FTM1->CNT = 0;                      // Counter reset
    FTM1->MOD = 1049;                   // PWM-Periode (für 20 kHz)
    // Channel 0 & 1 konfigurieren für Edge-Aligned PWM, high-true pulses
    FTM1->CONTROLS[0].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    FTM1->CONTROLS[1].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    // MSB: PWM aktiv, ELSB: Ausgang wird bei Compare = 0, sonst 1 (high-true)

    // Duty-Cycle setzen (CnV = Compare Value)
    // 1049 = 100%, 0 = 0%
    FTM1->CONTROLS[0].CnV = 0;        // Duty Cycle MotL = 0%
    FTM1->CONTROLS[1].CnV = 0;		  // Duty Cycle MotR = 0%

    // 11. FTM1 starten mit System Clock (CLKS = 01), Prescaler = 1 (PS = 000)
    FTM1->SC = FTM_SC_CLKS(1) | FTM_SC_PS(0);  // Timer läuft jetzt
}


void MotR_Direction(bool direction)
/* Mit dieser Funktion lässt sich die Drehrichtung vom Linken Motor ändern
 *
 * Input:
 * - bool direction: false = forwärts drehen, true = rückwärts drehen
 *
 * */
{
	// Einstellung für Rückwärtsdrehen
	if (direction == true)
	{
		// PTB0 als FTM1_CH1 konfigurieren (ALT3 = MUX-Wert 3)
		PORTB->PCR[1] = PORT_PCR_MUX(3);	// Funktion: FTM1_CH1 (PWM)
		// PTA12 als GPIO konfigurieren (ALT1 = GPIO)
		PORTA->PCR[13] = PORT_PCR_MUX(1);	// Funktion: GPIO
		// PTA12 als Ausgang setzen
		GPIOA->PDDR |= (1<<13);				// PDDR = 1 -> PTA13 = output
		// PTA12 auf LOW setzen
		GPIOA->PDOR &= ~(1<<13);			// PDOR = 0 -> PTA13 auf low
	}
	// Einstellung für Vorwärtsdrehen
	else if (direction == false)
	{

		// PTA12 als FTM1_CH1 konfigurieren (ALT3 = MUX-Wert 3)
		PORTA->PCR[13] = PORT_PCR_MUX(3);    // Funktion: FTM1_CH1 (PWM)
		// PTB0 als GPIO konfigurieren (ALT1 = GPIO)
		PORTB->PCR[1] = PORT_PCR_MUX(1);     // Funktion: GPIO
		// PTB0 als Ausgang setzen
		GPIOB->PDDR |= (1 << 1);            // PDDR = 1 -> PTB1 = output
		// PTB0 auf LOW setzen
		GPIOB->PDOR &= ~(1 << 1);           // PDOR = 0 -> PTB1 auf low

	}

}


void MotL_Direction(bool direction)
/* Mit dieser Funktion lässt sich die Drehrichtung vom Linken Motor ändern
 *
 * Input:
 * - bool direction: false = forwärts drehen, true = rückwärts drehen
 *
 * */
{
	// Einstellung für Rückwärtsdrehen
	if (direction == true)
	{
		// PTA12 als FTM1_CH0 konfigurieren (ALT3 = MUX-Wert 3)
		PORTA->PCR[12] = PORT_PCR_MUX(3);    // Funktion: FTM1_CH0 (PWM)
		// PTB0 als GPIO konfigurieren (ALT1 = GPIO)
		PORTB->PCR[0] = PORT_PCR_MUX(1);     // Funktion: GPIO
		// PTB0 als Ausgang setzen
		GPIOB->PDDR |= (1 << 0);            // PDDR = 1 -> PTB0 = output
		// PTB0 auf LOW setzen
		GPIOB->PDOR &= ~(1 << 0);           // PDOR = 0 -> PTB0 auf low
	}
	// Einstellung für Vorwärtsdrehen
	else if (direction == false)
	{
		// PTB0 als FTM1_CH0 konfigurieren (ALT3 = MUX-Wert 3)
		PORTB->PCR[0] = PORT_PCR_MUX(3);	// Funktion: FTM1_CH0 (PWM)
		// PTA12 als GPIO konfigurieren (ALT1 = GPIO)
		PORTA->PCR[12] = PORT_PCR_MUX(1);	// Funktion: GPIO
		// PTA12 als Ausgang setzen
		GPIOA->PDDR |= (1<<12);				// PDDR = 1 -> PTA12 = output
		// PTA12 auf LOW setzen
		GPIOA->PDOR &= ~(1<<12);			// PDOR = 0 -> PTA12 auf low
	}

}

void MotR_Control(uint8_t speed)
/* Mit dieser Funktion lässt sich die Geschwindigkeit und Richtung des rechten Motors einstellen
 *
 * Input:
 * - char speed: 0 = 0% Duticycle
 * */
{
	if (speed >= 0 && speed <= 100) // 0% - 100%
	{
		FTM1->CONTROLS[1].CnV = (1049*speed)/100;		  // Duty Cycle MotR = speed
	}


}


void MotL_Control(uint8_t speed)
/* Mit dieser Funktion lässt sich die Geschwindigkeit und Richtung des linken Motors einstellen
 *
 * Input:
 * - char speed: 0 = 0% Duticycle
 *
 * */
{
	if (speed >= 0 && speed <= 100) // 0% - 100%
	{
		FTM1->CONTROLS[0].CnV = (1049*speed)/100;		  // Duty Cycle MotL = speed
	}

}


void Turn(bool direction, unsigned int degrees)
/* Mit dieser Funktion lässt sich das Fahrzeug in die gewünschte Richtung drehen.
 * Die Funktion ist blockierend!!
 *
 * Input:
 * - direction: Links oder Rechts -> defines
 * #define Turn_Direction_Left	(false)
   #define Turn_Direction_Right (true)
 * - degrees: in Grad
 *
 * Output:
 * -
 *
 * */
{
	motl_enc_pos = 0;
    uint32_t targetTicks = (degrees * TICKS_PER_90) / 90U;
    uint32_t threshold;

    // Motoren setzen …
    if (direction == Turn_Direction_Left) { //nach links
    	if (degrees > 315U) {
    	    threshold = (targetTicks + (140));
    	    }
    		else if (degrees > 270U && degrees <= 315U){
    	    	threshold = (targetTicks + (130));
    	    }
    		else if (degrees > 225U && degrees <= 270U){
    			threshold = (targetTicks + (118));
    		}
    		else if (degrees > 180U && degrees <= 225U){
    		    threshold = (targetTicks + (105));
    		}
    		else if (degrees > 135U && degrees <= 180U){
    		    threshold = (targetTicks + (100));
    		}
    		else if (degrees > 90U && degrees <= 135U){
    		    threshold = (targetTicks + (60));
    		}
    		else if (degrees > 45U && degrees <= 90U){
    		    threshold = (targetTicks + (30));
    		}
    	    else { //kleiner gleich 45 grad
    	        threshold = targetTicks + 4;
    	}
        MotL_Direction(Mot_Setup_Backward);
        MotR_Direction(Mot_Setup_Forward);
    } else { //nach rechts
    	if (degrees > 315U) {
			threshold = (targetTicks + (200));
			}
			else if (degrees > 270U && degrees <= 315U){
				threshold = (targetTicks + (190));
			}
			else if (degrees > 225U && degrees <= 270U){
				threshold = (targetTicks + (175));
			}
			else if (degrees > 180U && degrees <= 225U){
				threshold = (targetTicks + (155));
			}
			else if (degrees > 135U && degrees <= 180U){
				threshold = (targetTicks + (130));
			}
			else if (degrees > 90U && degrees <= 135U){
				threshold = (targetTicks + (70));
			}
			else if (degrees > 45U && degrees <= 90U){
				threshold = (targetTicks + (35));
			}
			else { //kleiner gleich 45 grad
				threshold = targetTicks;
		}
        MotL_Direction(Mot_Setup_Forward);
        MotR_Direction(Mot_Setup_Backward);
    }
    MotL_Control(71);
    MotR_Control(71);

    // Warten bis threshold erreicht
    while (motl_enc_pos < threshold) { }

    // Stoppen und Rücksetzen …
    MotL_Control(0);
    MotR_Control(0);
    MotL_Direction(Mot_Setup_Forward);
    MotR_Direction(Mot_Setup_Forward);
}

void Drive_Straith(bool direction, unsigned int distance)
/*Diese Funktion erlaubt es um eine gewisse Distanz nach vorne oder hintern zu fahren
 *Die Funktion ist blockierend!!
 *Die
 * Input:
 * - direction: Nach vorne oder hintern
 * #define Mot_Setup_Forward (false)
   #define Mot_Setup_Backward (true)
   - distance: Distanz in cm
 *
 * Output:
 * -
 * */
{
	motl_enc_pos = 0; // Globale Variable für Encoder zurücksetzen

	if (direction == Mot_Setup_Forward)
	{
		MotL_Direction(Mot_Setup_Forward);
		MotR_Direction(Mot_Setup_Forward);
		MotR_Control(70);
		MotL_Control(70);
		while(motl_enc_pos < (distance * 85)/5); // 85 = 5cm
		MotR_Control(0);
		MotL_Control(0);
	}
	else if (direction == Mot_Setup_Backward)
	{
		MotL_Direction(Mot_Setup_Backward);
		MotR_Direction(Mot_Setup_Backward);
		MotR_Control(70);
		MotL_Control(70);
		while(motl_enc_pos < (distance * 85)/5); // 85 = 5cm
		MotR_Control(0);
		MotL_Control(0);
		MotL_Direction(Mot_Setup_Forward);
		MotR_Direction(Mot_Setup_Forward);
	}

}


#include "fsl_common.h"  // für SDK_GetTickCount()


void Scan(void)
{
    uint16_t line_sns_values[5];
    motl_enc_pos = 0;

    SDK_DelayAtLeastUs(10000, SystemCoreClock);  // kurz Sensor einpendeln lassen
    Linesensor(line_sns_values);
    uint8_t prev_mid = line_sns_values[2];

    // CCW drehen
    MotL_Direction(Mot_Setup_Forward);
    MotR_Direction(Mot_Setup_Backward);
    MotR_Control(71);
    MotL_Control(71);

    while (motl_enc_pos < (4 * 348)) // 4×348 = 360°
    {
        Linesensor(line_sns_values);
        uint8_t curr_mid = line_sns_values[2];

        if (curr_mid == 1 && prev_mid == 0) {
            // 1) Stopp
            MotR_Control(0);
            MotL_Control(0);

            // 2) 2 s Pause
            SDK_DelayAtLeastUs(2000000, SystemCoreClock);

            // 3) Winkel berechnen und senden
            // TODO

            // 4) Wieder los
            MotR_Control(71);
            MotL_Control(71);

            // 5) 0.5 s ignorieren
            SDK_DelayAtLeastUs(500000, SystemCoreClock);

            // 6) Debounce: warte bis Sensor wieder abseits der Linie
            do {
                Linesensor(line_sns_values);
            } while (line_sns_values[2]);
        }

        prev_mid = curr_mid;
    }

    // Am Ende anhalten
    MotR_Control(0);
    MotL_Control(0);
}
