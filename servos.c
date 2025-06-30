/*
 * servos.c
 *
 *  Created on: 30.05.2025
 *      Author: paris
 */


#include "servos.h"



void ServosInit(void)
/*
 * Initialisierung des grossen und kleinen Servos
 *
 * FTM0_ch1 -> servo small -> PTA4
 * */
{
	// Clock für Port A und Port B aktivieren
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;  // Aktiviert PORTA
	// Clock für FTM0 aktivieren
	SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK;
	// FTM0 ch1 -> PTA4
	PORTA->PCR[4] = PORT_PCR_MUX(3);
	// FTM0 ch2 -> PTA5
	PORTA->PCR[5] = PORT_PCR_MUX(3);
	// FTM0 stoppen, bevor Setup gemacht wird
	FTM0->SC = 0x00;                    // Stoppe den Timer
	// Counter und MOD-Register setzen
	// PWM-Frequenz = F_CLK / (MOD + 1)
	// F_CLK = ca. 20.97 MHz, MOD = 3276 & PS = 128 → ca. 50 Hz PWM
	FTM0->CNT = 0;                      // Counter reset
	FTM0->MOD = 3276;                   // PWM-Periode (für 50Hz)
	// FTM0 CH1 & CH2 konfigurieren für Edge-Aligned PWM, high-true pulses
	FTM0->CONTROLS[1].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0->CONTROLS[2].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	// Duty-Cycle setzen (CnV = Compare Value)
	// 3276 = 100%, 0 = 0%
	FTM0->CONTROLS[1].CnV = 393;        // Duty Cycle MotL = 0%
	FTM0->CONTROLS[2].CnV = 240;        // Duty Cycle MotL = 0%

	// Timer starten mit System Clock und Prescaler = 128
	FTM0->SC = FTM_SC_PS(7) | FTM_SC_CLKS(1);
}


void ServoBigControl(uint8_t position)
/*Zur Kontrolle des grossen Servos
 *
 * uint8_t position: Ausrichtung des grossen Servos siehe Defines
 * - #define ServoBig_Front (0)
 * - #define ServoBig_Middle (1)
 * - #define ServoBig_Back (2)
 * */
{
	if (position == ServoBig_Front)
	{
		FTM0->CONTROLS[2].CnV = 100; //100
	}

	else if (position == ServoBig_Middle)
	{
		FTM0->CONTROLS[2].CnV = 240; //240
	}

	else if (position == ServoBig_Back)
	{
		FTM0->CONTROLS[2].CnV = 365; // 365
	}

}


void ServoSmallControl(uint8_t position)
/*Zur Kontrolle des kleinen Servos
 *
 * uint8_t position: Ausrichtung des kleinen Servos siehe Defines
 *
 * #define ServoSmall_Closed (0)
 * #define ServoSmall_Opened (1)
 * */
{
	if (position == ServoSmall_Closed)
	{
		FTM0->CONTROLS[1].CnV = 287;        // Duty Cycle MotL = 0%
	}

	else if (position == ServoSmall_Opened)
	{
		FTM0->CONTROLS[1].CnV = 393;        // Duty Cycle MotL = 0%
	}


}

/**
 * Fährt den großen Servo in kleinen Schritten von der aktuellen
 * Position zur Zielposition und pausiert jeweils ein wenig.
 *
 * @param target   0 = Front, 1 = Middle, 2 = Back
 * @param stepDelayUs  Mikrosekunden Pause zwischen den Schritten.
 */
void ServoBig_MoveSlowly(uint8_t target, uint32_t stepDelayUs)
{
    // Bestimme das Ziel-CnV
    uint16_t targetCnV;
    switch (target)
    {
        case ServoBig_Front:  targetCnV = 90;  break;
        case ServoBig_Middle: targetCnV = 240;  break;
        case ServoBig_Back:   targetCnV = 370;  break;
        default: return; // ungültig
    }

    // Lese aktuelle CnV (Position)
    uint16_t currCnV = FTM0->CONTROLS[2].CnV;

    // In kleinen Schritten zum Ziel
    if (currCnV < targetCnV)
    {
        for (uint16_t v = currCnV; v <= targetCnV; ++v)
        {
            FTM0->CONTROLS[2].CnV = v;
            SDK_DelayAtLeastUs(stepDelayUs, SystemCoreClock);
        }
    }
    else
    {
        for (uint16_t v = currCnV; v >= targetCnV; --v)
        {
            FTM0->CONTROLS[2].CnV = v;
            SDK_DelayAtLeastUs(stepDelayUs, SystemCoreClock);
        }
    }
}

