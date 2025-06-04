/*
 * vl53l1_tof.c
 *
 *  Created on: 29.05.2025
 *      Author: paris
 */


#include "vl53l1_tof.h"
#include "fsl_debug_console.h"
#include "peripherals.h"


// Globale Geräteinstanz
VL53L1_Dev_t tof_dev;
VL53L1_DEV tof_pDev = &tof_dev;
I2C_Type *i2c_instance = I2C1_PERIPHERAL;



// Initialisierung (einmal aufrufen)
VL53L1_Error ToF_Init(void)
{
    VL53L1_Error status;

    // I2C-Adresse setzen (0x29 = 0x52 >> 1)
    tof_dev.i2c_slave_address = 0x29;

    status = VL53L1_WaitDeviceBooted(tof_pDev);
    if (status) return status;

    status = VL53L1_DataInit(tof_pDev);
    if (status) return status;

    status = VL53L1_StaticInit(tof_pDev);
    if (status) return status;

    status = VL53L1_SetDistanceMode(tof_pDev, VL53L1_DISTANCEMODE_LONG);
    if (status) return status;

    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(tof_pDev, 50000);
    return status;
}

// Einzelmessung (blocking)
VL53L1_Error ToF_PerformSingleMeasurement(uint16_t *distance_mm)
{
    VL53L1_Error status;
    VL53L1_RangingMeasurementData_t measurementData;
    uint8_t isDataReady = 0;

    status = VL53L1_StartMeasurement(tof_pDev);
    if (status) return status;

    do {
        status = VL53L1_GetMeasurementDataReady(tof_pDev, &isDataReady);
        if (status) return status;
    } while (!isDataReady);

    status = VL53L1_GetRangingMeasurementData(tof_pDev, &measurementData);
    if (status) return status;

    *distance_mm = measurementData.RangeMilliMeter;

    return VL53L1_StopMeasurement(tof_pDev);
}

void Tof_Select(bool tof)
/*Mit dieser Funktion kann man einstellen, ob man den oberen oder unteren TOF ansprechen will
 *
 * Input:
 * - bool tof: mit folgenden Defines:
 * #define SELECT_UPPER_TOF (0)
   #define SELECT_LOWER_TOF (1)
 *
 * */
{
	if(tof == true) // Unterer Tof aktivieren (Erkennung Hinderniss)
	{
		// PTD6 = 1 , PTD7 = low
		GPIOD->PDOR &= ~(1 << 7);
		GPIOD->PDOR |= (1 << 6);
	}
	else if (tof == false) // Oberer Tof aktivieren (Erkennung Pilone)
	{
		// PTD6 = 0 , PTD7 = 1
		GPIOD->PDOR &= ~(1 << 6);
		GPIOD->PDOR |= (1 << 7);
	}

	// 100us warten für Setup
	SDK_DelayAtLeastUs(100, CLOCK_GetFreq(kCLOCK_CoreSysClk));
}



void test_i2c_vll53l1(void)
/*Testet die API-I2C Schnittstelle vom vl53l1x*/
{
	VL53L1_Dev_t dev;
	VL53L1_Error status;
	uint8_t model_id = 0;
	dev.i2c_slave_address = 0x52 >> 1;  // 7-Bit Adresse (0x29)
	status = VL53L1_RdByte(&dev, 0x010F, &model_id);  // 0x010F = IDENTIFICATION__MODEL_ID

	if (status == VL53L1_ERROR_NONE)
	{
		DbgConsole_Printf("VL53L1X Model ID: 0x%02X\r\n", model_id);
	}
	else
	{
		DbgConsole_Printf("VL53L1X Read failed: error code = %d\r\n", status);
	   }

}


