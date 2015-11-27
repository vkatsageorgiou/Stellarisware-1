//*****************************************************************************
//
// file.h - Public function prototypes and globals related to file handling
//!         in the Quickstart Oscilloscope application.
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
#ifndef _FILE_H_
#define _FILE_H_

extern void FileTickHandler(void);
extern tBoolean FileInit(void);
extern tBoolean FileWriteCSV(tDataAcqCaptureStatus *pCapData,
                             tBoolean bSDCard);
extern tBoolean FileWriteBitmap(tDataAcqCaptureStatus *pCapData,
                                tBoolean bSDCard);
extern tBoolean FileFindNextFilename(char *pcFilename,
                                     unsigned char ucDriveNum,
                                     unsigned long ulLen,
                                     const char *pcExt);
extern tBoolean FileCatToUART(char *pcFilename);
extern tBoolean FileLsToUART(char *pcDir);
extern tBoolean FileIsDrivePresent(unsigned char ucDriveNum);
extern tBoolean FileMountUSB(tBoolean bMount);

#endif
