/*
 * motor.c
 *
 *  Created on: 21.05.2025
 *      Author: paris
 */

#include "motor.h" // Headerfile


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
	motl_enc_pos = 0; // Globale Variable für Encoder zurücksetzen

	if (direction == Turn_Direction_Left)
	{
		MotL_Direction(Mot_Setup_Backward);
		MotR_Direction(Mot_Setup_Forward);
		MotR_Control(75);
		MotL_Control(75);
		while(motl_enc_pos < ((degrees*348)/90-30)); // 348 = 90° -> -30 da Motoren unterschiedlich stark (Toleranz)
		MotR_Control(0);
		MotL_Control(0);
		MotL_Direction(Mot_Setup_Forward);
		MotR_Direction(Mot_Setup_Forward);

	}
	else if (direction == Turn_Direction_Right)
	{
		MotR_Direction(Mot_Setup_Backward);
		MotL_Direction(Mot_Setup_Forward);
		MotR_Control(75);
		MotL_Control(75);
		while(motl_enc_pos < ((degrees*348)/90));
		MotR_Control(0);
		MotL_Control(0);
		MotL_Direction(Mot_Setup_Forward);
		MotR_Direction(Mot_Setup_Forward);
	}

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


void Scan(void)
/* Diese Funktion Scant auf einem Wegpunkt im Gegenuhrzeigersin nach weiteren Wegen
 * Die Funktion ist blockierend!!
 *
 * */
{
	// Zwischenspeicher Werte Linesensor
	 uint16_t line_sns_values[5]; 	// L, ML, M, MR, R / 1 = auf Linie, 0 = abseits der Linie
	// Speicher für verschiedene Wege


	motl_enc_pos = 0; // Globale Variable für Encoder zurücksetzen
	// CCW drehen
	MotL_Direction(Mot_Setup_Backward);
	MotR_Direction(Mot_Setup_Forward);
	MotR_Control(75);
	MotL_Control(75);
	do {
		Linesensor(line_sns_values);
		if (line_sns_values[0] == 1) // Mit Flankenerkennung fortfahren
		{

		}

	}
	while(motl_enc_pos < (4 * 348)); // 348 = 90°

}
