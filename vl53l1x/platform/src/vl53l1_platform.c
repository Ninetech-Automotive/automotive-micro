/*******************************************************************************
 Copyright (C) 2016, STMicroelectronics International N.V.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of STMicroelectronics nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
 IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/**
 * @file   vl53l1_platform.c
 * @brief  Code function definitions for EwokPlus25 Platform Layer
 *         RANGING SENSOR VERSION
 *
 */
//#include <windows.h>

#include <stdio.h>      // sprintf(), vsnprintf(), printf()
#include <stdint.h>
#include <string.h>     // strncpy(), strnlen()

// Anpassungen ***
#include "fsl_common.h"
#include "fsl_clock.h"
#include "fsl_i2c.h"

extern I2C_Type *i2c_instance;  // deine globale I2C-Instanz (z. B. I2C0 oder I2C1)

// Für VL53L1_GetTickCount
volatile uint32_t s_ms_ticks = 0;

void SysTick_Handler(void)
{
    s_ms_ticks++;
}
// ***

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

#include "vl53l1_platform.h"
#include "vl53l1_platform_log.h"

#ifdef PAL_EXTENDED
	#include "vl53l1_register_strings.h"
#else
	#define VL53L1_get_register_name(a,b)
#endif

//#include "ranging_sensor_comms.h"
//#include "power_board_defs.h"

// flag to indicate if power board has been accessed
const uint32_t _power_board_in_use = 0;

// flag to indicate if we can use the extended voltage ranges (not laser safe!)
uint32_t _power_board_extended = 0;

// cache of the comms type flag
uint8_t global_comms_type = 0;

#define  VL53L1_COMMS_CHUNK_SIZE  56
#define  VL53L1_COMMS_BUFFER_SIZE 64

//#define GPIO_INTERRUPT          RS_GPIO62
//#define GPIO_POWER_ENABLE       RS_GPIO60
//#define GPIO_XSHUTDOWN          RS_GPIO61
//#define GPIO_SPI_CHIP_SELECT    RS_GPIO51

/*!
 *  The intent of this Abstraction layer is to provide the same API
 *  to the underlying SystemVerilog tasks as the C driver will have
 *  to ST Comms DLL's for the talking to Ewok via the USB + STM32
 *  or if the C-driver is implemented directly on the STM32
 */

#define trace_print(level, ...) \
	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_PLATFORM, \
	level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

#define trace_i2c(...) \
	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_NONE, \
	VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)


VL53L1_Error VL53L1_CommsInitialise(
	VL53L1_Dev_t *pdev,
	uint8_t       comms_type,
	uint16_t      comms_speed_khz)
{
	(void)pdev;
	(void)comms_type;
	(void)comms_speed_khz;
	global_comms_type = VL53L1_I2C;  // Du nutzt I2C, fix setzen
	return VL53L1_ERROR_NONE;
}


VL53L1_Error VL53L1_CommsClose(
	VL53L1_Dev_t *pdev)
{
	(void)pdev;
	return VL53L1_ERROR_NONE;
}

/*
 * ----------------- COMMS FUNCTIONS -----------------
 */

VL53L1_Error VL53L1_WriteMulti(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	    uint8_t buffer[256];

	    // Prüfe auf zulässige Paketgröße
	    if (count + 2 > sizeof(buffer))
	        return VL53L1_ERROR_INVALID_PARAMS;

	    // 16-bit Registeradresse als Big-Endian voranstellen
	    buffer[0] = (index >> 8) & 0xFF;
	    buffer[1] = index & 0xFF;

	    memcpy(&buffer[2], pdata, count);

	    i2c_master_transfer_t xfer;
	    memset(&xfer, 0, sizeof(xfer));

	    xfer.slaveAddress = pdev->i2c_slave_address;
	    xfer.direction = kI2C_Write;
	    xfer.subaddressSize = 0;
	    xfer.data = buffer;
	    xfer.dataSize = count + 2;
	    xfer.flags = kI2C_TransferDefaultFlag;

	    if (I2C_MasterTransferBlocking(i2c_instance, &xfer) != kStatus_Success)
	        status = VL53L1_ERROR_CONTROL_INTERFACE;

	    return status;
}


VL53L1_Error VL53L1_ReadMulti(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	    uint8_t index_buffer[2];

	    // Registeradresse im Big-Endian-Format
	    index_buffer[0] = (index >> 8) & 0xFF;
	    index_buffer[1] = index & 0xFF;

	    i2c_master_transfer_t xfer_write = {
	        .slaveAddress = pdev->i2c_slave_address,
	        .direction = kI2C_Write,
	        .subaddress = 0,
	        .subaddressSize = 0,
	        .data = index_buffer,
	        .dataSize = sizeof(index_buffer),
	        .flags = kI2C_TransferNoStopFlag // Wichtig: Kein Stop, da gleich ein Read folgt
	    };

	    if (I2C_MasterTransferBlocking(i2c_instance, &xfer_write) != kStatus_Success)
	        return VL53L1_ERROR_CONTROL_INTERFACE;

	    i2c_master_transfer_t xfer_read = {
	        .slaveAddress = pdev->i2c_slave_address,
	        .direction = kI2C_Read,
	        .subaddress = 0,
	        .subaddressSize = 0,
	        .data = pdata,
	        .dataSize = count,
	        .flags = kI2C_TransferRepeatedStartFlag // Repeated Start zwischen Write und Read
	    };

	    if (I2C_MasterTransferBlocking(i2c_instance, &xfer_read) != kStatus_Success)
	        status = VL53L1_ERROR_CONTROL_INTERFACE;

	    return status;
}


VL53L1_Error VL53L1_WrByte(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t       data)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;
	uint8_t  buffer[1];

	// Split 16-bit word into MS and LS uint8_t
	buffer[0] = (uint8_t)(data);

	status = VL53L1_WriteMulti(pdev, index, buffer, 1);

	return status;
}


VL53L1_Error VL53L1_WrWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint16_t      data)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;
	uint8_t  buffer[2];

	// Split 16-bit word into MS and LS uint8_t
	buffer[0] = (uint8_t)(data >> 8);
	buffer[1] = (uint8_t)(data &  0x00FF);

	status = VL53L1_WriteMulti(pdev, index, buffer, VL53L1_BYTES_PER_WORD);

	return status;
}


VL53L1_Error VL53L1_WrDWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint32_t      data)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;
	uint8_t  buffer[4];

	// Split 32-bit word into MS ... LS bytes
	buffer[0] = (uint8_t) (data >> 24);
	buffer[1] = (uint8_t)((data &  0x00FF0000) >> 16);
	buffer[2] = (uint8_t)((data &  0x0000FF00) >> 8);
	buffer[3] = (uint8_t) (data &  0x000000FF);

	status = VL53L1_WriteMulti(pdev, index, buffer, VL53L1_BYTES_PER_DWORD);

	return status;
}


VL53L1_Error VL53L1_RdByte(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;
	uint8_t  buffer[1];

	status = VL53L1_ReadMulti(pdev, index, buffer, 1);

	*pdata = buffer[0];

	return status;
}


VL53L1_Error VL53L1_RdWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint16_t     *pdata)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;
	uint8_t  buffer[2];

	status = VL53L1_ReadMulti(
					pdev,
					index,
					buffer,
					VL53L1_BYTES_PER_WORD);

	*pdata = (uint16_t)(((uint16_t)(buffer[0])<<8) + (uint16_t)buffer[1]);

	return status;
}


VL53L1_Error VL53L1_RdDWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint32_t     *pdata)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t  buffer[4];

	status = VL53L1_ReadMulti(
					pdev,
					index,
					buffer,
					VL53L1_BYTES_PER_DWORD);

	*pdata = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) + ((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];

	return status;
}

/*
 * ----------------- HOST TIMING FUNCTIONS -----------------
 */

VL53L1_Error VL53L1_WaitUs(
	VL53L1_Dev_t *pdev,
	int32_t       wait_us)
{
	(void)pdev;  // pdev wird nicht verwendet
	SDK_DelayAtLeastUs((uint32_t)wait_us, CLOCK_GetFreq(kCLOCK_CoreSysClk));
	return VL53L1_ERROR_NONE;
}


VL53L1_Error VL53L1_WaitMs(
	VL53L1_Dev_t *pdev,
	int32_t       wait_ms)
{
	return VL53L1_WaitUs(pdev, wait_ms * 1000);
}

/*
 * ----------------- DEVICE TIMING FUNCTIONS -----------------
 */

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
	 *ptimer_freq_hz = CLOCK_GetFreq(kCLOCK_CoreSysClk);  // MCU-spezifisch
	 return VL53L1_ERROR_NONE;
}


VL53L1_Error VL53L1_GetTimerValue(int32_t *ptimer_count)
{
	*ptimer_count = (int32_t)SysTick->VAL;
	return VL53L1_ERROR_NONE;
}


/*
 * ----------------- GPIO FUNCTIONS -----------------
 */

// ---> Alle durch Dummy-Funktionen ersetzt

VL53L1_Error VL53L1_GpioSetMode(uint8_t pin, uint8_t mode)
{
	(void)pin;
	(void)mode;
	return VL53L1_ERROR_NONE;
}


VL53L1_Error  VL53L1_GpioSetValue(uint8_t pin, uint8_t value)
{
	(void)pin;
	(void)value;
	return VL53L1_ERROR_NONE;
}


VL53L1_Error  VL53L1_GpioGetValue(uint8_t pin, uint8_t *pvalue)
{
	(void)pin;
	if (pvalue) *pvalue = 0;
	return VL53L1_ERROR_NONE;
}

/*
 * ----------------- HARDWARE STATE FUNCTIONS -----------------
 */

VL53L1_Error  VL53L1_GpioXshutdown(uint8_t value)
{
	 (void)value;
	 return VL53L1_ERROR_NONE;
}


VL53L1_Error  VL53L1_GpioCommsSelect(uint8_t value)
{
	(void)value;
	return VL53L1_ERROR_NONE;
}


VL53L1_Error  VL53L1_GpioPowerEnable(uint8_t value)
{
	(void)value;
	return VL53L1_ERROR_NONE;
}


VL53L1_Error  VL53L1_GpioInterruptEnable(void (*function)(void), uint8_t edge_type)
{
	(void)function;
	(void)edge_type;
	return VL53L1_ERROR_NONE;
}


VL53L1_Error  VL53L1_GpioInterruptDisable(void)
{
	 return VL53L1_ERROR_NONE;
}


VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptick_count_ms)
{

	 *ptick_count_ms = s_ms_ticks;

	#ifdef VL53L1_LOG_ENABLE
	    trace_print(
	        VL53L1_TRACE_LEVEL_DEBUG,
	        "VL53L1_GetTickCount() = %5u ms;\n",
	        *ptick_count_ms);
	#endif

	return VL53L1_ERROR_NONE;

}


VL53L1_Error VL53L1_WaitValueMaskEx(
	VL53L1_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{
	/*
	 * Platform implementation of WaitValueMaskEx V2WReg script command
	 *
	 * WaitValueMaskEx(
	 *          duration_ms,
	 *          index,
	 *          value,
	 *          mask,
	 *          poll_delay_ms);
	 */

	VL53L1_Error status         = VL53L1_ERROR_NONE;
	uint32_t     start_time_ms   = 0;
	uint32_t     current_time_ms = 0;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;
#ifdef VL53L1_LOG_ENABLE
	uint32_t     trace_functions = 0;
#endif

	_LOG_STRING_BUFFER(register_name);

	SUPPRESS_UNUSED_WARNING(poll_delay_ms);

#ifdef VL53L1_LOG_ENABLE
	/* look up register name */
	VL53L1_get_register_name(
			index,
			register_name);

	/* Output to I2C logger for FMT/DFT  */
	trace_i2c("WaitValueMaskEx(%5d, %s, 0x%02X, 0x%02X, %5d);\n",
		timeout_ms, register_name, value, mask, poll_delay_ms);
#endif // VL53L1_LOG_ENABLE

	/* calculate time limit in absolute time */

	VL53L1_GetTickCount(&start_time_ms);
	pdev->new_data_ready_poll_duration_ms = 0;

	/* remember current trace functions and temporarily disable
	 * function logging
	 */

#ifdef VL53L1_LOG_ENABLE
	trace_functions = _LOG_GET_TRACE_FUNCTIONS();
#endif
	_LOG_SET_TRACE_FUNCTIONS(VL53L1_TRACE_FUNCTION_NONE);

	/* wait until value is found, timeout reached on error occurred */

	while ((status == VL53L1_ERROR_NONE) &&
		   (pdev->new_data_ready_poll_duration_ms < timeout_ms) &&
		   (found == 0))
	{
		status = VL53L1_RdByte(
						pdev,
						index,
						&byte_value);

		if ((byte_value & mask) == value)
		{
			found = 1;
		}

		/*if (status == VL53L1_ERROR_NONE  &&
			found == 0 &&
			poll_delay_ms > 0)
			status = VL53L1_WaitMs(
							pdev,
							poll_delay_ms);
		*/

		/* Update polling time (Compare difference rather than absolute to
		negate 32bit wrap around issue) */
		VL53L1_GetTickCount(&current_time_ms);
		pdev->new_data_ready_poll_duration_ms = current_time_ms - start_time_ms;
	}

	/* Restore function logging */
	_LOG_SET_TRACE_FUNCTIONS(trace_functions);

	if (found == 0 && status == VL53L1_ERROR_NONE)
		status = VL53L1_ERROR_TIME_OUT;

	return status;
}

