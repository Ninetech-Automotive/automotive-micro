/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    MK22F51212_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK22F51212.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "fsl_uart.h"	// for UART
#include "fsl_i2c.h"	// for I2C
#include "fsl_common.h" // for Delay


// Eigene Includes
#include "linesensor.h"
#include "motor.h"
#include "follow_line.h"
#include "vl53l1_tof.h"
#include "servos.h"


/* TODO: insert other definitions and declarations here. */


// *** Funktionsprototypen ***
void Send_Word(char *message);

// *** Defines ***
// Switch
#define IDLE_START (0)
#define START (1)
#define IDLE (2)
#define STOPP (3)


// Schalterposition Ziel



// *** Globale Variablen ***
uint8_t state = IDLE_START;		// Startzustand Statemachine
unsigned int motl_enc_pos = 0;	// Zähler Position MotL



/* PORTD_IRQn interrupt handler */
void PORTD_IRQHandler(void) {
  /* Get pin flags */
  uint32_t pin_flags = GPIO_PortGetInterruptFlags(GPIOD);
  /* Place your interrupt code here */
  if (pin_flags & (1<<4))	// Check btn_stopp for rising_edge
  {
	  DbgConsole_Printf("btn_stop pushed!\n\r");
	  state = STOPP;

  }
  else if (pin_flags & (1<<3)) // Check btn_start for rising edge
  {
	  DbgConsole_Printf("btn_start pushed!\n\r");
	  if (state == IDLE_START)
	  {
		  state = START; // Auslösung -> Start Condition
	  }

  }
  /* Clear pin flags */
  GPIO_PortClearInterruptFlags(GPIOD, pin_flags);
}

/* PORTD_IRQn interrupt handler */
void PORTB_IRQHandler(void) {
  /* Get pin flags */
  uint32_t pin_flags = GPIO_PortGetInterruptFlags(GPIOB);
  /* Place your interrupt code here */
  if (pin_flags & (1<<2))	// MotL_Enc_A
  {
	  motl_enc_pos++;

  }
  /* Clear pin flags */
  GPIO_PortClearInterruptFlags(GPIOB, pin_flags);
}



/*
 * @brief   Application entry point.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    //BOARD_InitDebugConsole(); -> für printf manuell initialisiert
    DbgConsole_Init(1, 115200, kSerialPort_Uart, CLOCK_GetFreq(kCLOCK_BusClk)); // Um printf auf Console über UART1 -> PTC3/4 (COM3) anzeigen zu lassen
    // Initiolisierung der Motoren -> siehe motor.h
	Init_Motorsteuerung();
	// Initialisierung der Servos -> siehe servos.h
	ServosInit();
    /* vl53l1x tof senso*/
	//Für Zeitfunktion
	SysTick_Config(CLOCK_GetFreq(kCLOCK_CoreSysClk) / 1000U);
	// vl5311x init
	ToF_Init();
	Tof_Select(SELECT_LOWER_TOF);





    // *** Lokale Variablen Definieren ***
    // Test Linesensor
    //uint16_t line_sns_values[5]; // L, ML, M, MR, R / 1 = auf Linie, 0 = abseits der Linie
    // UART0

	#define MAX_WORD_LENGTH (30)
    uint8_t ch;
    uint8_t buffer[MAX_WORD_LENGTH];
    int i = 0;

    // Test Tof
	//uint16_t dist;



    DbgConsole_Printf("\n\nApplication start:\n\r");




    /* Enter an infinite loop*/
    while(1) {

    	/* Test UART0 Receive and Send Words
    	if (UART_ReadBlocking(UART0, &ch, 1) == kStatus_Success)  // Lies 1 Zeichen
    	{
    		if (ch == '\n' || ch == ' ' || ch == '\r')  // Wortende?
				{
					buffer[i] = '\0';  // Nullterminierung
					DbgConsole_Printf("Empfangenes Wort: %s\r\n", buffer);
					UART_WriteBlocking(UART0, buffer, i);
					UART_WriteBlocking(UART0, (uint8_t*)"\r\n", 2);  // Neue Zeile
					i = 0;  // Für das nächste Wort zurücksetzen



				}
			else if (i < MAX_WORD_LENGTH - 1)
			{
				buffer[i++] = ch;
			}
    	}
		*/



    	/* Test Liniensensor*
    	Linesensor(line_sns_values);
    	DbgConsole_Printf("%i \t %i \t %i \t %i \t %i\n\r", line_sns_values[0], line_sns_values[1], line_sns_values[2], line_sns_values[3], line_sns_values[4]);
		*/

    	/* Test Tof
    	ToF_PerformSingleMeasurement(&dist);
		DbgConsole_Printf("Distanz: %d mm\r\n", dist);
		*/




    	/* MainSwitch
    	switch(state)
    	{
    	case IDLE_START: // Warten bis Start ausgelösst

    		break;

    	case START:		// Start ausgelöst
    		Send_Word("set_target:{A}\n");
    		state = IDLE;

    		break;

    	case IDLE:		// Warten auf Befehl
    		if (UART_ReadBlocking(UART0, &ch, 1) == kStatus_Success)  // Lies 1 Zeichen
				{
					if (ch == '\n')  // Wortende?
						{
							buffer[i] = '\0';  // Nullterminierung

						}
				}


    	case FOLLOW_LINE:
			if(FollowLine() == ON_POINT)
			{
				Drive_Straith(Mot_Setup_Forward, 10);
				state = STOPP;
			}

    		break;

    	case TURN:
    		Turn(Turn_Direction_Left, 90); // Blockierend!
    		state = STOPP;

    		break;

    	case STRAITH:
    		Drive_Straith(Mot_Setup_Forward, 5); // Blockierend!
    		state = STOPP;

    	case STOPP:

    		MotL_Control(0);
    		MotR_Control(0);
    		Send_Word("stop\n");
    		state = IDLE_START;

    		break;
    	}
*/



    	// Delay von 1 Sekunde
		//SDK_DelayAtLeastUs(1000000U, CLOCK_GetFreq(kCLOCK_CoreSysClk));


    }

}



void Send_Word(char *message)
{
	UART_WriteBlocking(UART0, (uint8_t *)message, strlen(message));
}

