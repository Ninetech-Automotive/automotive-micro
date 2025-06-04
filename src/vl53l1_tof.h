/*
 * vl53l1_tof.h
 *
 *  Created on: 29.05.2025
 *      Author: paris
 */

#ifndef VL53L1_TOF_H_
#define VL53L1_TOF_H_

// *** Notwendige Includes ***
#include "vl53l1_api.h"             // Haupt-API
#include "vl53l1_platform.h"        // I2C/Treiberanbindung
#include "fsl_i2c.h"
#include "fsl_common.h"
#include "MK22F51212.h"

// *** Defines ***
#define SELECT_UPPER_TOF (0)
#define SELECT_LOWER_TOF (1)

// *** Globale Variablen & Instanzen ***
// Globale Instanz (optional extern)
extern VL53L1_Dev_t tof_dev;
extern VL53L1_DEV tof_pDev;

// *** Funktionsprototypen ***
VL53L1_Error ToF_Init(void);
VL53L1_Error ToF_PerformSingleMeasurement(uint16_t *distance_mm);
void test_i2c_vll53l1(void);
void Tof_Select(bool tof);


#endif /* VL53L1_TOF_H_ */
