/*
 * linesensor.c
 *
 *  Created on: 09.05.2025
 *      Author: paris
 */

#include "linesensor.h" // Headerfile


status_t ADS7828_ReadChannel(uint8_t command, uint16_t *result)
/* Liesst den jeweiligen Kanal über I2C vom ADC des Linesensors
 * Input:
 * - uint8_t command: Command Byte für den jeweiligen Channel
 * - uint16_t *result: Zeiger auf die Variable, in welcher das Resultat gespeichert werden soll
 * Output:
 * - I2C_Status von I2C_fsl ob Kommunikation erfolgreich war "kStatus_Success" oder nicht
 * */
{
	i2c_master_transfer_t xfer;
	    status_t status;
	    uint8_t rxBuf[2];

	    // Schritt 1: Kommando-Byte senden (Write ohne Stop)
	    memset(&xfer, 0, sizeof(xfer));
	    xfer.slaveAddress = ADS7828_ADDR;
	    xfer.direction = kI2C_Write;
	    xfer.data = &command;
	    xfer.dataSize = 1;
	    xfer.subaddressSize = 0;
	    xfer.flags = kI2C_TransferNoStopFlag;

	    status = I2C_MasterTransferBlocking(I2C1, &xfer);

	    if (status != kStatus_Success)
	        return status;

	    SDK_DelayAtLeastUs(10, CLOCK_GetFreq(kCLOCK_CoreSysClk)); // 12us für Wandlungszeit abwarten

	    // Schritt 2: 2 Byte lesen (mit Stop)
	    memset(&xfer, 0, sizeof(xfer));
	    xfer.slaveAddress = ADS7828_ADDR;
	    xfer.direction = kI2C_Read;
	    xfer.data = rxBuf;
	    xfer.dataSize = 2;
	    xfer.subaddressSize = 0;
	    xfer.flags = kI2C_TransferRepeatedStartFlag;

	    status = I2C_MasterTransferBlocking(I2C1, &xfer);
	    if (status != kStatus_Success)
	        return status;

	    // 12-Bit-Wert extrahieren (linksbündig)
	    *result = ((uint16_t)rxBuf[0] << 8 | rxBuf[1]) >> 4;

	    return kStatus_Success;
}


void Linesensor(uint16_t *ptr)
/*
 * Form des Arrays, wessen Adresse mitgegeben wird:
 * uint16_t linevalue[5]; // 0 = L, 1 = ML, 2 = M, 3 = MR, 4 = R
 *
 * Input:
 * Adresse des Arrays, in welchem die Linesensor Werte gespeichert werden sollen
 * uint16_t linevalue[5];
 * Linesensor(&linevalue[0]); -> ptr zeigt auf erstes Feld des Arrays
 *
 * Output:
 * Schreibt in Zielarray ob jeweiliger Punkt auf der Linie = 1 oder abseits der Linie = 0
 *
 * */
{
	status_t status; // Variable für i2c_status
	uint16_t result; // Zwischenresultat, bevor der offset dazugerechnet wird und der Wert im Zielarray angepasst

	// ptr zeigt auf Feld 0 des Inputarrays = L
	status = ADS7828_ReadChannel(cmd_ch0, &result);
	if (status != kStatus_Success)
	{
		DbgConsole_Printf("Fehler beim Lesen von CH0 des Linesensor! Code: %d\n\r", status);
	}


	if ((result + offset_ch0) >= On_Line_Threshold) // Entscheidung ob auf der Linie oder nicht
	{
		*ptr = 1;
	}
	else
	{
		*ptr = 0;
	}


	ptr++;	// ptr zeigt auf Feld 1 des Inputarrays = ML

	status = ADS7828_ReadChannel(cmd_ch1, &result);
	if (status != kStatus_Success)
	{
		DbgConsole_Printf("Fehler beim Lesen von CH1 des Linesensor! Code: %d\n\r", status);
	}

	if ((result + offset_ch1) >= On_Line_Threshold) // Entscheidung ob auf der Linie oder nicht
	{
		*ptr = 1;
	}
	else
	{
		*ptr = 0;
	}



	ptr++;	// ptr zeigt auf Feld 2 des Inputarrays = M

	status = ADS7828_ReadChannel(cmd_ch2, &result);
	if (status != kStatus_Success)
	{
		DbgConsole_Printf("Fehler beim Lesen von CH2 des Linesensor! Code: %d\n\r", status);
	}


	if ((result + offset_ch2) >= On_Line_Threshold) // Entscheidung ob auf der Linie oder nicht
	{
		*ptr = 1;
	}
	else
	{
		*ptr = 0;
	}


	ptr++; // ptr zeigt auf Feld 3 des Inputarrays = MR

	status = ADS7828_ReadChannel(cmd_ch3, &result);
	if (status != kStatus_Success)
	{
		DbgConsole_Printf("Fehler beim Lesen von CH3 des Linesensor! Code: %d\n\r", status);
	}

	if ((result + offset_ch3) >= On_Line_Threshold) // Entscheidung ob auf der Linie oder nicht
	{
		*ptr = 1;
	}
	else
	{
		*ptr = 0;
	}

	ptr++; // ptr zeigt auf Feld 4 des Inputarrays = R

	status = ADS7828_ReadChannel(cmd_ch4, &result);
	if (status != kStatus_Success)
	{
		DbgConsole_Printf("Fehler beim Lesen von CH4 des Linesensor! Code: %d\n\r", status);
	}


	if ((result + offset_ch4) >= On_Line_Threshold) // Entscheidung ob auf der Linie oder nicht
	{
		*ptr = 1;
	}
	else
	{
		*ptr = 0;
	}


}

