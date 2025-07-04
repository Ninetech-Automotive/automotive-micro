/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _USB_DEVICE_CONFIG_H_
#define _USB_DEVICE_CONFIG_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @addtogroup usb_device_configuration
 * @{
 */

/*!
 * @name Hardware instance define
 * @{
 */

/* @TEST_ANCHOR */

/*! @brief KHCI instance count */
#ifndef USB_DEVICE_CONFIG_KHCI
#define USB_DEVICE_CONFIG_KHCI (1U)
#endif

/*! @brief EHCI instance count */
#ifndef USB_DEVICE_CONFIG_EHCI
#define USB_DEVICE_CONFIG_EHCI (0U)
#endif

/*! @brief LPC USB IP3511 FS instance count */
#ifndef USB_DEVICE_CONFIG_LPCIP3511FS
#define USB_DEVICE_CONFIG_LPCIP3511FS (0U)
#endif

/*! @brief LPC USB IP3511 HS instance count */
#ifndef USB_DEVICE_CONFIG_LPCIP3511HS
#define USB_DEVICE_CONFIG_LPCIP3511HS (0U)
#endif

/*! @brief Device instance count, the sum of KHCI and EHCI instance counts*/
#define USB_DEVICE_CONFIG_NUM \
    (USB_DEVICE_CONFIG_KHCI + USB_DEVICE_CONFIG_EHCI + USB_DEVICE_CONFIG_LPCIP3511FS + USB_DEVICE_CONFIG_LPCIP3511HS)

/* @} */

/*!
 * @name class instance define
 * @{
 */

/*! @brief HID instance count */
#ifndef USB_DEVICE_CONFIG_HID
#define USB_DEVICE_CONFIG_HID (0U)
#endif

/*! @brief CDC ACM instance count */
#ifndef USB_DEVICE_CONFIG_CDC_ACM
#define USB_DEVICE_CONFIG_CDC_ACM (0U)
#endif
#ifndef USB_DEVICE_CONFIG_CDC_RNDIS
#define USB_DEVICE_CONFIG_CDC_RNDIS (0U)
#endif

/*! @brief MSC instance count */
#ifndef USB_DEVICE_CONFIG_MSC
#define USB_DEVICE_CONFIG_MSC (0U)
#endif

/*! @brief Audio instance count */
#ifndef USB_DEVICE_CONFIG_AUDIO
#define USB_DEVICE_CONFIG_AUDIO (0U)
#endif

/*! @brief PHDC instance count */
#ifndef USB_DEVICE_CONFIG_PHDC
#define USB_DEVICE_CONFIG_PHDC (0U)
#endif

/*! @brief Video instance count */
#ifndef USB_DEVICE_CONFIG_VIDEO
#define USB_DEVICE_CONFIG_VIDEO (0U)
#endif

/*! @brief CCID instance count */
#ifndef USB_DEVICE_CONFIG_CCID
#define USB_DEVICE_CONFIG_CCID (0U)
#endif

/*! @brief Printer instance count */
#ifndef USB_DEVICE_CONFIG_PRINTER
#define USB_DEVICE_CONFIG_PRINTER (0U)
#endif

/*! @brief DFU instance count */
#ifndef USB_DEVICE_CONFIG_DFU
#define USB_DEVICE_CONFIG_DFU (0U)
#endif

/* @} */

/*! @brief Whether device is self power. 1U supported, 0U not supported */
#define USB_DEVICE_CONFIG_SELF_POWER (1U)

/*! @brief How many endpoints are supported in the stack. */
#define USB_DEVICE_CONFIG_ENDPOINTS (4U)

/*! @brief Whether the device task is enabled. */
#define USB_DEVICE_CONFIG_USE_TASK (0U)

/*! @brief How many the notification message are supported when the device task is enabled. */
#define USB_DEVICE_CONFIG_MAX_MESSAGES (8U)

/*! @brief Whether test mode enabled. */
#define USB_DEVICE_CONFIG_USB20_TEST_MODE (0U)

/*! @brief Whether device CV test is enabled. */
#define USB_DEVICE_CONFIG_CV_TEST (0U)

/*! @brief Whether device compliance test is enabled. If the macro is enabled,
    the test mode and CV test macroes will be set.*/
#ifndef USB_DEVICE_CONFIG_COMPLIANCE_TEST
#define USB_DEVICE_CONFIG_COMPLIANCE_TEST (0U)
#endif

#if ((defined(USB_DEVICE_CONFIG_COMPLIANCE_TEST)) && (USB_DEVICE_CONFIG_COMPLIANCE_TEST > 0U))

/*! @brief Undefine the macro USB_DEVICE_CONFIG_USB20_TEST_MODE. */
#undef USB_DEVICE_CONFIG_USB20_TEST_MODE
/*! @brief Undefine the macro USB_DEVICE_CONFIG_CV_TEST. */
#undef USB_DEVICE_CONFIG_CV_TEST

/*! @brief enable the test mode. */
#define USB_DEVICE_CONFIG_USB20_TEST_MODE (1U)

/*! @brief enable the CV test */
#define USB_DEVICE_CONFIG_CV_TEST (1U)

#endif

#if ((defined(USB_DEVICE_CONFIG_KHCI)) && (USB_DEVICE_CONFIG_KHCI > 0U))

/*! @brief The MAX buffer length for the KHCI DMA workaround.*/
#define USB_DEVICE_CONFIG_KHCI_DMA_ALIGN_BUFFER_LENGTH (64U)
#endif

#if ((defined(USB_DEVICE_CONFIG_EHCI)) && (USB_DEVICE_CONFIG_EHCI > 0U))
/*! @brief How many the DTD are supported. */
#define USB_DEVICE_CONFIG_EHCI_MAX_DTD (16U)

/*! @brief Whether the EHCI ID pin detect feature enabled. */
#define USB_DEVICE_CONFIG_EHCI_ID_PIN_DETECT (0U)
#endif

/*! @brief Whether the keep alive feature enabled. */
#define USB_DEVICE_CONFIG_KEEP_ALIVE_MODE (0U)

/*! @brief Whether the transfer buffer is cache-enabled or not. */
#ifndef USB_DEVICE_CONFIG_BUFFER_PROPERTY_CACHEABLE
#define USB_DEVICE_CONFIG_BUFFER_PROPERTY_CACHEABLE (0U)
#endif
/*! @brief Whether the low power mode is enabled or not. */
#define USB_DEVICE_CONFIG_LOW_POWER_MODE (0U)

#if ((defined(USB_DEVICE_CONFIG_LOW_POWER_MODE)) && (USB_DEVICE_CONFIG_LOW_POWER_MODE > 0U))
/*! @brief Whether device remote wakeup supported. 1U supported, 0U not supported */
#define USB_DEVICE_CONFIG_REMOTE_WAKEUP (0U)

/*! @brief Whether LPM is supported. 1U supported, 0U not supported */
#define USB_DEVICE_CONFIG_LPM_L1 (0U)
#else
/*! @brief The device remote wakeup is unsupported. */
#define USB_DEVICE_CONFIG_REMOTE_WAKEUP (0U)
#endif

/*! @brief Whether the device detached feature is enabled or not. */
#define USB_DEVICE_CONFIG_DETACH_ENABLE (0U)

/*! @brief Whether handle the USB bus error. */
#define USB_DEVICE_CONFIG_ERROR_HANDLING (0U)

/* @} */

#endif /* _USB_DEVICE_CONFIG_H_ */
