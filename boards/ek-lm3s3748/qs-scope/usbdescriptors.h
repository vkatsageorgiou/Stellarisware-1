//*****************************************************************************
//
// usbdescriptors.h - The usb descriptor information for the oscilloscope
// device.
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

#ifndef __USBDESCRIPTORS_H__
#define __USBDESCRIPTORS_H__

//*****************************************************************************
//
// Prototypes for USB event handlers
//
//*****************************************************************************
extern void HandleEndpoints(unsigned long ulIndex, unsigned long ulStatus);
extern void HandleConfigChange(unsigned long ulIndex, unsigned long ulInfo);
extern void HandleDisconnect(unsigned long ulIndex);
extern void HandleReset(unsigned long ulIndex);

//*****************************************************************************
//
// Endpoints to use for each of the required endpoints in the driver.
//
//*****************************************************************************
#define DATA_IN_ENDPOINT        USB_EP_1
#define DATA_OUT_ENDPOINT       USB_EP_2

//*****************************************************************************
//
// Maximum packet size for the bulk endpoints used for serial data
// transmission and reception and the associated FIFO sizes to set aside
// for each endpoint.
//
//*****************************************************************************
#define DATA_IN_EP_FIFO_SIZE    USB_FIFO_SZ_64
#define DATA_OUT_EP_FIFO_SIZE   USB_FIFO_SZ_64

#define DATA_IN_EP_MAX_SIZE     USB_FIFO_SZ_TO_BYTES(DATA_IN_EP_FIFO_SIZE)
#define DATA_OUT_EP_MAX_SIZE    USB_FIFO_SZ_TO_BYTES(DATA_IN_EP_FIFO_SIZE)

//*****************************************************************************
//
// The device information structure containing callbacks and descriptors.
//
//*****************************************************************************
extern tDeviceInfo g_sScopeDeviceInfo;

#endif // __USBDESCRIPTORS_H__
