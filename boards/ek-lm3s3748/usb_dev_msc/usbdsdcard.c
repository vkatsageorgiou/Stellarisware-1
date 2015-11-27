//*****************************************************************************
//
// usbdsdcard.c - Routines supplied for use by the mass storage class device
// class.
//
// Copyright (c) 2009 Luminary Micro, Inc.  All rights reserved.
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

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/systick.h"
#include "grlib/grlib.h"
#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdmsc.h"
#include "drivers/formike128x128x16.h"
#include "usb_msc_structs.h"
#include "third_party/fatfs/src/diskio.h"


#define SDCARD_PRESENT          0x00000001
#define SDCARD_IN_USE           0x00000002
struct
{
    unsigned long ulFlags;
}
g_sDriveInformation;

//*****************************************************************************
//
// This function opens the drive number and prepares it for use by the Mass
// storage class device.
//
// /param ulDrive is the driver number to open.
//
// This function is used to initialize and open the physical drive number
// associated with the parameter /e ulDrive.  The function will return zero if
// the drive could not be opened for some reason.  In the case of removable
// device like an SD card this function should return zero if the SD card is
// not present.
//
// /return Returns a pointer to data that should be passed to other APIs or it
// will return 0 if no drive was found.
//
//*****************************************************************************
void *
USBDMSCStorageOpen(unsigned long ulDrive)
{
    unsigned char ucPower;
    unsigned long ulTemp;

    ASSERT(ulDrive == 0);

    //
    // Return if already in use.
    //
    if(g_sDriveInformation.ulFlags & SDCARD_IN_USE)
    {
        return(0);
    }

    //
    // Initialize the drive if it is present.
    //
    ulTemp = disk_initialize(0);

    if(ulTemp == RES_OK)
    {
        //
        // Power up the card.
        //
        ucPower = 1;
        disk_ioctl(0, CTRL_POWER, &ucPower);

        //
        // Card is present and in use.
        //
        g_sDriveInformation.ulFlags = SDCARD_PRESENT | SDCARD_IN_USE;
    }
    else if(ulTemp == STA_NODISK)
    {
        //
        // Allocate the card but it is not present.
        //
        g_sDriveInformation.ulFlags = SDCARD_IN_USE;
    }
    else
    {
        return(0);
    }

    return((void *)&g_sDriveInformation);
}

//*****************************************************************************
//
// This function close the drive number in use by the mass storage class device.
//
// /param pvDrive is the pointer that was returned from a call to
// USBDMSCStorageOpen().
//
// This function is used to close the physical drive number associated with the
// parameter /e pvDrive.  This function will return 0 if the drive was closed
// successfully and any other value will indicate a failure.
//
// /return Returns 0 if the drive was successfully closed or non-zero for a
// failure.
//
//*****************************************************************************
void
USBDMSCStorageClose(void * pvDrive)
{
    unsigned char ucPower;

    ASSERT(pvDrive != 0);

    //
    // Clear all flags.
    //
    g_sDriveInformation.ulFlags = 0;

    //
    // Power up the card.
    //
    ucPower = 0;

    //
    // Turn off the power to the card.
    //
    disk_ioctl(0, CTRL_POWER, &ucPower);
}

//*****************************************************************************
//
// This function will read a block from a device opened by the
// USBDMSCStorageOpen() call.
//
// /param pvDrive is the pointer that was returned from a call to
// USBDMSCStorageOpen().
// /param pucData is the buffer that data will be written into.
// /param ulNumBlocks is the number of blocks to read.
//
// This function is use to read blocks from a physical device and return them
// in the /e pucData buffer.  The data area pointed to by /e pucData should be
// at least /e ulNumBlocks * Block Size bytes to prevent overwriting data.
//
// /return Returns the number of bytes that were read from the device.
//
//*****************************************************************************
unsigned long USBDMSCStorageRead(void * pvDrive,
                                 unsigned char *pucData,
                                 unsigned long ulSector,
                                 unsigned long ulNumBlocks)
{
    ASSERT(pvDrive != 0);

    if(disk_read (0, pucData, ulSector, ulNumBlocks) == RES_OK)
    {
        // TODO remove fixed 512
        return(ulNumBlocks * 512);
    }
    return(0);
}

//*****************************************************************************
//
// This function will write a block to a device opened by the
// USBDMSCStorageOpen() call.
//
// /param pvDrive is the pointer that was returned from a call to
// USBDMSCStorageOpen().
// /param pucData is the buffer that data will be used for writing.
// /param ulNumBlocks is the number of blocks to write.
//
// This function is use to write blocks to a physical device from the buffer
// pointed to by the /e pucData buffer.  If the number of blocks is greater than
// one then the block address will increment and write to the next block until
// /e ulNumBlocks * Block Size bytes have been written.
//
// /return Returns the number of bytes that were written to the device.
//
//*****************************************************************************
unsigned long USBDMSCStorageWrite(void * pvDrive,
                                  unsigned char *pucData,
                                  unsigned long ulSector,
                                  unsigned long ulNumBlocks)
{
    ASSERT(pvDrive != 0);

    if(disk_write(0, pucData, ulSector, ulNumBlocks) == RES_OK)
    {
        return(ulNumBlocks * 512);
    }
    return(0);
}

//*****************************************************************************
//
// This function will return the number of blocks present on a device.
//
// /param pvDrive is the pointer that was returned from a call to
// USBDMSCStorageOpen().
//
// This function is used to return the total number of blocks on a physical
// device based on the /e pvDrive parameter.
//
// /return Returns the number of blocks that are present in a device.
//
//*****************************************************************************
unsigned long
USBDMSCStorageNumBlocks(void * pvDrive)
{
    unsigned long ulSectorCount;

    //
    // Read the number of sectors.
    //
    disk_ioctl(0, GET_SECTOR_COUNT, &ulSectorCount);

    return(ulSectorCount);
}

//*****************************************************************************
//
// This function will return the current status of a device.
//
// /param pvDrive is the pointer that was returned from a call to
// USBDMSCStorageOpen().
//
// This function is used to return the current status of the device indicated by
// the /e pvDrive parameter.  This can be used to see if the device is busy,
// or if it is present.
//
// /return Returns the size in bytes of blocks that in a device.
//
//*****************************************************************************
#define USBDMSC_IDLE            0x00000000
#define USBDMSC_NOT_PRESENT     0x00000001
unsigned long USBDMSCStorageStatus(void * pvDrive);
