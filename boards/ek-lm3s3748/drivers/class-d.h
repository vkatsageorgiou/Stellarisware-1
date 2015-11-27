//*****************************************************************************
//
// class-d.h - Prototypes for the Class-D amplifier driver.
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

#ifndef __CLASS_D_H__
#define __CLASS_D_H__

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void ClassDPWMHandler(void);
extern void ClassDInit(unsigned long ulPWMClock);
extern void ClassDPlayPCM(const unsigned char *pucBuffer,
                          unsigned long ulLength);
extern void ClassDPlayADPCM(const unsigned char *pucBuffer,
                            unsigned long ulLength);
extern tBoolean ClassDBusy(void);
extern void ClassDStop(void);
extern void ClassDVolumeSet(unsigned long ulVolume);
extern void ClassDVolumeUp(unsigned long ulVolume);
extern void ClassDVolumeDown(unsigned long ulVolume);

#endif // __CLASS_D_H__
