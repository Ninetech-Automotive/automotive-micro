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


#ifndef _VL53L1_PLATFORM_USER_DEFINES_H_
#define _VL53L1_PLATFORM_USER_DEFINES_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @file   vl53l1_platform_user_defines.h
 *
 * @brief  All end user OS/platform/application definitions
 */
#define VL53L1_LOG_ENABLE             0
#define VL53L1_LOG_ENABLE_I2C         0
#define VL53L1_LOG_DISABLED
#define VL53L1_USE_EMPTY_CLOCK_FUNCTIONS

#define VL53L1_USE_INTERRUPTS         0
#define VL53L1_USE_GPIO_FUNCTIONS     0

// GPIO Dummy-Zuweisungen (werden sonst in platform.c verwendet)
#define GPIO_XSHUTDOWN                0
#define GPIO_POWER_ENABLE             0
#define GPIO_INTERRUPT                0
#define GPIO_SPI_CHIP_SELECT          0



/**
 * @def do_division_u
 * @brief customer supplied division operation - 64-bit unsigned
 *
 * @param dividend      unsigned 64-bit numerator
 * @param divisor       unsigned 64-bit denominator
 */
#define do_division_u(dividend, divisor) (dividend / divisor)


/**
 * @def do_division_s
 * @brief customer supplied division operation - 64-bit signed
 *
 * @param dividend      signed 64-bit numerator
 * @param divisor       signed 64-bit denominator
 */
#define do_division_s(dividend, divisor) (dividend / divisor)


/**
 * @def WARN_OVERRIDE_STATUS
 * @brief customer supplied macro to optionally output info when a specific
	  error has been overridden with success within the EwokPlus driver
 *
 * @param __X__      the macro which enabled the suppression
 */
#define WARN_OVERRIDE_STATUS(__X__)\
	trace_print (VL53L1_TRACE_LEVEL_WARNING, #__X__);


#ifdef _MSC_VER
#define DISABLE_WARNINGS() { \
	__pragma (warning (push)); \
	__pragma (warning (disable:4127)); \
	}
#define ENABLE_WARNINGS() { \
	__pragma (warning (pop)); \
	}
#else
	#define DISABLE_WARNINGS()
	#define ENABLE_WARNINGS()
#endif


#ifdef __cplusplus
}
#endif

#endif // _VL53L1_PLATFORM_USER_DEFINES_H_

