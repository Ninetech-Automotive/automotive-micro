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

//Schalter
#define SW_DIP_A (GPIOD->PDIR & (1<<1))
#define SW_DIP_B (GPIOD->PDIR & (1<<2))

// Switch
#define IDLE_START (0)
#define START (1)
#define IDLE (2)
#define STOPP (3)
#define PONG          (4)
#define FOLLOW_LINE   (5)
#define TURN          (6)
#define SCAN_POINT    (7)
#define REMOVE_OBSTACLE	(8)


// Schalterposition Ziel



// *** Globale Variablen ***
uint8_t state = IDLE_START;		// Startzustand Statemachine
unsigned int motl_enc_pos = 0;	// Zähler Position MotL
int turnDirection = 0;
int targetAngle = 0;


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

    // Test Turn funktion
    //uint8_t x = 0;



    DbgConsole_Printf("\n\nApplication start:\n\r");




    /* Enter an infinite loop*/
    while(1) {

    	  // Test Schalter
    	        	if (SW_DIP_A)
    	        	{
    	        		DbgConsole_Printf("A\n\r");
    	        	}

    	        	if (SW_DIP_B)
    	        	{
    	        		DbgConsole_Printf("B\n\r");
    	        	}

    	/*
    	Test UART0 Receive and Send Words
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

    	//Test Tof
    	//ToF_PerformSingleMeasurement(&dist);
		//DbgConsole_Printf("Distanz: %d mm\r\n", dist);





    	// MainSwitch
    	switch(state)
    	{
    	case IDLE_START: // Warten bis Start ausgelösst

    		break;

    	case START:		// Start ausgelöst

    		if (SW_DIP_A && !SW_DIP_B)
			{
    			Send_Word("set_target:A\n");
    			DbgConsole_Printf("Target: A\n\r");
			}

    		else if (SW_DIP_B && !SW_DIP_A)
			{
    			Send_Word("set_target:B\n");
    			DbgConsole_Printf("Target: B\n\r");
			}
    		else {
    			Send_Word("set_target:C\n");
    			DbgConsole_Printf("Target: C\n\r");
    		}
    		DbgConsole_Printf("\n\nStart ausgeloest\n\r");
    		SDK_DelayAtLeastUs(100000, SystemCoreClock);
    		state = IDLE;
    		break;

    	case IDLE:
		{
			// Versuch, 1 Byte zu lesen (blockierend)
			if (UART_ReadBlocking(UART0, &ch, 1) == kStatus_Success)
			{
				if (ch != '\n' && ch != '\r' && ch != ' ')
				{
					// Zeichen zum Puffer hinzufügen, solange Platz ist
					if (i < MAX_WORD_LENGTH - 1)
					{
						buffer[i++] = ch;
					}
				}
				else
				{
					// Wortende erreicht
					buffer[i] = '\0';  // Null-Terminator
					i = 0;             // Für das nächste Wort zurücksetzen

					// Befehle auswerten
					if (strcmp((char*)buffer, "ping") == 0)
					{
						state = PONG;
					}
					else if (strcmp((char*)buffer, "follow_line") == 0)
					{
						state = FOLLOW_LINE;
					}
					else if (strncmp((char*)buffer, "target_line_angle:", 18) == 0)
					{
						int angle = 0;
						if (sscanf((char*)buffer + 18, "%d", &angle) == 1)
						{
							if (angle >= 0)
							{
								turnDirection = 1;      // rechts
								targetAngle = angle;
							}
							else
							{
								turnDirection = 2;      // links
								targetAngle = -angle;
							}
							state = TURN;
						}
					}
					else if (strcmp((char*)buffer, "target_reached") == 0)
					{
						state = STOPP;
					}
					// sonst bleibt state = IDLE
				}
			}
		}
		break;


		case PONG:
			Send_Word("pong\n");
			state = IDLE;

			break;


    	case FOLLOW_LINE:
			uint16_t dist_upper = 0;
			uint16_t dist_lower = 0;
			const uint16_t OBSTACLE_DIST_THRESHOLD = 83; // Beispielwert in mm
			const uint16_t CONE_DIST_THRESHOLD = 100; // Beispielwert in mm


			// Unteren ToF abfragen
			ToF_PerformSingleMeasurement(&dist_lower);
			//DbgConsole_Printf("Distanz: %d mm\r\n", dist_lower);


			if(dist_lower < OBSTACLE_DIST_THRESHOLD) {
				MotL_Control(0);
				MotR_Control(0);
				Tof_Select(SELECT_UPPER_TOF);
				SDK_DelayAtLeastUs(1000000U, SystemCoreClock); //1s delay um TOF einzupendeln
				ToF_PerformSingleMeasurement(&dist_upper);
				Tof_Select(SELECT_LOWER_TOF);
				if (dist_upper < CONE_DIST_THRESHOLD) {
					Send_Word("cone_detected\n");
					Drive_Straith(Mot_Setup_Backward, 12); // Ein wenig Rückwärts fahren
					Turn(Turn_Direction_Left, 180); // Umdrehen
					state = FOLLOW_LINE; // Zurück fahren
					break;
				} else {
					Send_Word("obstacle_detected\n");
					state = REMOVE_OBSTACLE;
					break;
				}
			}


			if(FollowLine() == ON_POINT )
			{
				Drive_Straith(Mot_Setup_Forward, 13);
				Send_Word("on_waypoint\n");
				state = IDLE;
			}

			break;

		case TURN:
			DbgConsole_Printf("\n\nin Turn State\n\r");
			if (turnDirection == 1) // right
			{
				Turn(Turn_Direction_Right, targetAngle); // Blockierend!
				SDK_DelayAtLeastUs(1000000U, SystemCoreClock);
				DbgConsole_Printf("\n\nTurned Right\n\r");
			}
			else if (turnDirection == 2) // left
			{
				Turn(Turn_Direction_Left, targetAngle); // Blockierend!
				SDK_DelayAtLeastUs(1000000U, SystemCoreClock);
				DbgConsole_Printf("\n\nTurned Left\n\r");
			}

			// Nach dem Drehen: Prüfen, ob eine Linie erkannt wird
			
			uint16_t line_sns_values[5];
			Linesensor(line_sns_values);
			bool line_found = false;
			for (int j = 0; j < 5; j++)
			{
				if (line_sns_values[j])
				{
					line_found = true;
					break;
				}
			}
			if (line_found)
			{
				Send_Word("turned_to_target_line\n");
				state = IDLE;
			}
			else
			{
				// Versuche, die Linie durch kleine Korrekturen zu finden
				const int correction_angle = 30;
				line_found = false;

				// Erst kleine Korrektur in Drehrichtung
				if (turnDirection == 1)
				{
					Turn(Turn_Direction_Right, correction_angle);
				}
				else if (turnDirection == 2)
				{
					Turn(Turn_Direction_Left, correction_angle);
				}
				Linesensor(line_sns_values);
				for (int j = 0; j < 5; j++)
				{
					if (line_sns_values[j])
					{
						line_found = true;
						break;
					}
				}

				// Falls nicht gefunden, Korrektur in Gegenrichtung (doppelt so groß)
				if (!line_found)
				{
					if (turnDirection == 1)
					{
						Turn(Turn_Direction_Left, correction_angle * 2);
					}
					else if (turnDirection == 2)
					{
						Turn(Turn_Direction_Right, correction_angle * 2);
					}
					Linesensor(line_sns_values);
					for (int j = 0; j < 5; j++)
					{
						if (line_sns_values[j])
						{
							line_found = true;
							break;
						}
					}
					// Falls immer noch nicht gefunden, zurück zur Ausgangsposition
					if (!line_found)
					{
						if (turnDirection == 1)
						{
							Turn(Turn_Direction_Right, correction_angle);
						}
						else if (turnDirection == 2)
						{
							Turn(Turn_Direction_Left, correction_angle);
						}
						Send_Word("line_missing\n");
						state = IDLE;
					}
					else
					{
						Send_Word("turned_to_target_line\n");
						state = IDLE;
					}
				}
				else
				{
					Send_Word("turned_to_target_line\n");
					state = IDLE;
				}
			}
			

			break;



    	case STOPP:
    		MotL_Control(0);
    		MotR_Control(0);
    		Send_Word("stop\n");
    		state = IDLE_START;

    		break;

		case REMOVE_OBSTACLE:
			// Entfernen des Hindernisses
			ServoBig_MoveSlowly(ServoBig_Front, 13000);
			SDK_DelayAtLeastUs(1000000, SystemCoreClock);
			ServoSmallControl(ServoSmall_Closed);
			SDK_DelayAtLeastUs(500000, SystemCoreClock);
			ServoBig_MoveSlowly(ServoBig_Middle, 13000);
			SDK_DelayAtLeastUs(1000000, SystemCoreClock);

			// Für ca. 1 Sekunde der Linie folgen
			for (int i = 0; i < 550; i++) {
				FollowLine();
				SDK_DelayAtLeastUs(1000, SystemCoreClock); // 1 ms warten
			}
			MotL_Control(0);
			MotR_Control(0);

			ServoBig_MoveSlowly(ServoBig_Back, 13000);
			SDK_DelayAtLeastUs(1000000, SystemCoreClock);
			ServoSmallControl(ServoSmall_Opened);
			SDK_DelayAtLeastUs(500000, SystemCoreClock);
			ServoBig_MoveSlowly(ServoBig_Middle, 13000);

			state = FOLLOW_LINE;
			break;


		case SCAN_POINT:
			Scan();
			state = IDLE;
			break;
    	}

    	// Delay von 1 Sekunde
		//SDK_DelayAtLeastUs(1000000U, CLOCK_GetFreq(kCLOCK_CoreSysClk))



    }

}



void Send_Word(char *message)
{
	UART_WriteBlocking(UART0, (uint8_t *)message, strlen(message));
}

