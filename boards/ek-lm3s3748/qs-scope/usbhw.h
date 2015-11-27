//*****************************************************************************
//
// usbhw.h - Definitions related to the USB hardware connections on the
//           EK board.
//
// Copyright (c) 2008-2009 Luminary Micro, Inc.  All rights reserved.
// Software License Agreement
// 
// Luminary Micro, Inc. (LMI) is supplying this software for use solely and
// exclusively on LMI's microcontroller products.
// 
// The software is owned by LMI and/or its suppliers, and is protected under
// applicable copyright laws.  All rights are reserved.  You may not combine
// this software with "viral" open-source software in order to form a larger
// program.  Any use in violation of the foregoing restrictions may subject
// the user to criminal sanctions under applicable laws, as well as to civil
// liability for the breach of the terms and conditions of this license.
// 
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
// OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
// LMI SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
// CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 4781 of the EK-LM3S3748 Firmware Package.
//
//*****************************************************************************

#ifndef __USBHW_H__
#define __USBHW_H__

//*****************************************************************************
//
// GPIO definitions related to USB operation.
//
//*****************************************************************************
#define USB_MUX_GPIO_PERIPH     SYSCTL_PERIPH_GPIOH
#define USB_MUX_GPIO_BASE       GPIO_PORTH_BASE
#define USB_MUX_GPIO_PIN        GPIO_PIN_2
#define USB_MUX_SEL_DEVICE      USB_MUX_GPIO_PIN
#define USB_MUX_SEL_HOST        0

#define USB_PWR_GPIO_PERIPH     SYSCTL_PERIPH_GPIOH
#define USB_PWR_GPIO_BASE       GPIO_PORTH_BASE
#define USB_PWR_GPIO_PINS       (GPIO_PIN_3 | GPIO_PIN_4)

#define USB_HOST_GPIO_PERIPH    SYSCTL_PERIPH_GPIOB
#define USB_HOST_GPIO_BASE      GPIO_PORTB_BASE
#define USB_HOST_GPIO_PINS      (GPIO_PIN_0 | GPIO_PIN_1)

#endif // __USBHW_H__
