//*****************************************************************************
//
// usbdmsc.h - USB mass storage device class driver.
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
// This is part of revision 4781 of the Stellaris USB Library.
//
//*****************************************************************************

#ifndef __USBDMSC_H__
#define __USBDMSC_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \addtogroup msc_device_class_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// USBDMSCMediaChange() ulStatus values.
//
//*****************************************************************************
typedef enum
{
    USBDMSC_MEDIA_PRESENT,
    USBDMSC_MEDIA_NOTPRESENT,
    USBDMSC_MEDIA_UNKNOWN
}
tUSBDMSCMediaStatus;

//*****************************************************************************
//
//! Media Access functions.
//
//*****************************************************************************
typedef struct
{
    //
    //! This function is used to initialize and open the physical drive number
    //! associated with the parameter ulDrive.  The function will return zero if
    //! the drive could not be opened for some reason.  In the case of removable
    //! device like an SD card this function should return zero if the SD card
    //! is not present.
    //! The function returns a pointer to data that should be passed to other
    //! APIs or it will return 0 if no drive was found.
    //
    void *(* Open)(unsigned long ulDrive);

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
    void (* Close)(void * pvDrive);

    //*****************************************************************************
    //
    // This function will read a block from a device opened by the
    // USBDMSCStorageOpen() call.
    //
    // /param pvDrive is the pointer that was returned from a call to
    // USBDMSCStorageOpen().
    // /param pucData is the buffer that data will be written into.
    // /param ulSector is the block address to read.
    // /param ulNumBlocks is the number of blocks to read.
    //
    // This function is use to read blocks from a physical device and return them
    // in the /e pucData buffer.  The data area pointed to by /e pucData should be
    // at least /e ulNumBlocks * Block Size bytes to prevent overwriting data.
    //
    // /return Returns the number of bytes that were read from the device.
    //
    //*****************************************************************************
    unsigned long (* BlockRead)(void * pvDrive, unsigned char *pucData,
                                unsigned long ulSector,
                                unsigned long ulNumBlocks);

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
    unsigned long (* BlockWrite)(void * pvDrive, unsigned char *pucData,
                                 unsigned long ulSector,
                                 unsigned long ulNumBlocks);

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
    unsigned long (* NumBlocks)(void * pvDrive);

}
tMSCDMedia;

//*****************************************************************************
//
// PRIVATE
//
// This structure defines the private instance data and state variables for the
// mass storage class.  The memory for this structure is pointed to by
// the psPrivateData field in the tUSBDMSCDevice structure passed on
// USBDMSCInit() and should not be modified by any code outside of the mass
// storage device code.
//
//*****************************************************************************
typedef struct
{
    unsigned long ulUSBBase;
    tDeviceInfo *psDevInfo;
    tConfigDescriptor *psConfDescriptor;

    //
    // These three values are used to return the current sense data for an
    // instance of the mass storage class.
    //
    unsigned char ucErrorCode;
    unsigned char ucSenseKey;
    unsigned short usAddSenseCode;

    //
    // The pointer to the instance returned from the Open call to the media.
    //
    void *pvMedia;

    volatile tBoolean bConnected;

    //
    // Holds the flag settings for this instance.
    //
    unsigned long ulFlags;

    tUSBDMSCMediaStatus eMediaStatus;
}
tMSCInstance;

//*****************************************************************************
//
//! This is the size in bytes of the private data for the mass storage class.
//
//*****************************************************************************
#define USB_MSC_WORKSPACE_SIZE  sizeof(tMSCInstance);

//*****************************************************************************
//
//! The structure used by the application to define operating parameters for
//! the mass storage device.
//
//*****************************************************************************
typedef struct
{
    //
    //! The vendor ID that this device is to present in the device descriptor.
    //
    unsigned short usVID;

    //
    //! The product ID that this device is to present in the device descriptor.
    //
    unsigned short usPID;

    //
    //! 8 byte vendor string.
    //
    unsigned char pucVendor[8];

    //
    //! 16 byte vendor string.
    //
    unsigned char pucProduct[16];

    //
    //! 4 byte vendor string.
    //
    unsigned char pucVersion[4];

    //
    //! The maximum power consumption of the device, expressed in milliamps.
    //
    unsigned short usMaxPowermA;

    //
    //! Indicates whether the device is self or bus-powered and whether or not
    //! it supports remote wakeup.  Valid values are USB_CONF_ATTR_SELF_PWR or
    //! USB_CONF_ATTR_BUS_PWR, optionally ORed with USB_CONF_ATTR_RWAKE.
    //
    unsigned char ucPwrAttributes;

    //
    //! A pointer to the string descriptor array for this device.  This array
    //! must contain the following string descriptor pointers in this order.
    //! Language descriptor, Manufacturer name string (language 1), Product
    //! name string (language 1), Serial number string (language 1), MSC
    //! Interface description string (language 1), Configuration description
    //! string (language 1).
    //!
    //! If supporting more than 1 language, the descriptor block (except for
    //! string descriptor 0) must be repeated for each language defined in the
    //! language descriptor.
    //!
    //
    const unsigned char * const *ppStringDescriptors;

    //
    //! The number of descriptors provided in the ppStringDescriptors
    //! array.  This must be 1 + ((5 + (num HID strings)) * (num languages)).
    //
    unsigned long ulNumStringDescriptors;

    //
    //! This structure holds the access functions for the media used by this
    //! instance of the mass storage class device.  All of the functions in this
    //! structure are required to be filled out with valid functions.
    //
    tMSCDMedia sMediaFunctions;

    //
    //! This is the callback function for various events that occur during
    //! mass storage class operation.
    //
    tUSBCallback pfnEventCallback;

    //
    //! A pointer to RAM workspace for this device instance.  The client
    //! must fill in this field with a pointer to at least
    //! USB_MSC_WORKSPACE_SIZE bytes of read/write storage that the library
    //! can use for driver workspace.  This memory must remain accessible for
    //! as long as the MSC device is in use and must not be modified by any
    //! code outside the MSC class driver.
    //
    tMSCInstance *psPrivateData;
}
tUSBDMSCDevice;

//*****************************************************************************
//
// MSC-specific device class driver events
//
//*****************************************************************************

//*****************************************************************************
//
//! This event indicates that the host has completed other operations and is
//! no longer accessing the device.
//
//*****************************************************************************
#define USBD_MSC_EVENT_IDLE     (USBD_MSC_EVENT_BASE + 0)

//*****************************************************************************
//
//! This event indicates that the host is reading the storage media.
//
//*****************************************************************************
#define USBD_MSC_EVENT_READING  (USBD_MSC_EVENT_BASE + 1)

//*****************************************************************************
//
//! This event indicates that the host is writing to the storage media.
//
//*****************************************************************************
#define USBD_MSC_EVENT_WRITING  (USBD_MSC_EVENT_BASE + 2)


//*****************************************************************************
//
// API Function Prototypes
//
//*****************************************************************************
extern void *USBDMSCInit(unsigned long ulIndex,
                         const tUSBDMSCDevice *psMSCDevice);
extern void USBDMSCTerm(void *psDevice);
extern void USBDMSCMediaChange(void *psDevice,
                               tUSBDMSCMediaStatus eMediaStatus);

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
