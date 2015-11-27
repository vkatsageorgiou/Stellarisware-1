//*****************************************************************************
//
// usbhost.h - Host specific definitions for the USB host library.
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
// This is part of revision 4781 of the Stellaris USB Library.
//
//*****************************************************************************

#ifndef __USBHOST_H__
#define __USBHOST_H__

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
//! \addtogroup usblib_hcd
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// This is the type used to identify what the pipe is currently in use for.
//
//*****************************************************************************
#define USBHCD_PIPE_UNUSED          0x00100000
#define USBHCD_PIPE_CONTROL         0x00130000
#define USBHCD_PIPE_BULK_OUT        0x00210000
#define USBHCD_PIPE_BULK_IN         0x00220000
#define USBHCD_PIPE_INTR_OUT        0x00410000
#define USBHCD_PIPE_INTR_IN         0x00420000
#define USBHCD_PIPE_ISOC_OUT        0x00810000
#define USBHCD_PIPE_ISOC_IN         0x00820000
#define USBHCD_PIPE_BULK_OUT_DMA    0x01210000
#define USBHCD_PIPE_BULK_IN_DMA     0x01220000

//*****************************************************************************
//
//! This macro is used to declare an instance of an Event driver for the USB
//! library.
//!
//! \param VarName is the name of the variable.
//! \param pfnOpen is the callback for the Open call to this driver.  This
//! value is currently reserved and should be set to 0.
//! \param pfnClose is the callback for the Close call to this driver.  This
//! value is currently reserved and should be set to 0.
//! \param pfnEvent is the callback that will be called for various USB events.
//!
//! The first parameter is the actual name of the variable that will
//! be declared by this macro.  The second and third parameter are reserved
//! for future functionality and are unused and should be set to zero.  The
//! last parameter is the actual callback function and is specified as
//! a function pointer of the type:
//!
//! void (*pfnEvent)(void *pvData);
//!
//! When the \e pfnEvent function is called the void pointer that is passed in
//! as a parameter should be cast to a pointer to a structure of type
//! tEventInfo.  This will contain the event that caused the pfnEvent function
//! to be called.
//
//*****************************************************************************
#define DECLARE_EVENT_DRIVER(VarName, pfnOpen, pfnClose, pfnEvent)          \
void IntFn(void *pvData);                                                   \
const tUSBHostClassDriver VarName =                                         \
{                                                                           \
    USB_CLASS_EVENTS,                                                       \
    0,                                                                      \
    0,                                                                      \
    pfnEvent                                                                \
}

//*****************************************************************************
//
//! This is the structure that holds all of the information for devices
//! that are enumerated in the system.   It is passed in to Open function of
//! USB host class drivers so that they can allocate any endpoints and parse
//! out other information that the device class needs to complete enumeration.
//
//*****************************************************************************
typedef struct
{
    //
    //! The current device address for this device.
    //
    unsigned long ulAddress;

    //
    //! The current interface for this device.
    //
    unsigned long ulInterface;

    //
    //! A pointer to the device descriptor for this device.
    //
    tDeviceDescriptor DeviceDescriptor;

    //
    //! A pointer to the configuration descriptor for this device.
    //
    tConfigDescriptor *pConfigDescriptor;

    //
    //! The size of the buffer allocated to pConfigDescriptor.
    //
    unsigned long ulConfigDescriptorSize;
}
tUSBHostDevice;

//*****************************************************************************
//
//! This structure defines a USB host class driver interface, it is parsed to
//! find a USB class driver once a USB device is enumerated.
//
//*****************************************************************************
typedef struct
{
    //
    //! The interface class that this device class driver supports.
    //
    unsigned long ulInterfaceClass;

    //
    //! The function is called when this class of device has been detected.
    //
    void * (*pfnOpen)(tUSBHostDevice *pDevice);

    //
    //! The function is called when the device, originally opened with a call
    //! to the pfnOpen function, is disconnected.
    //
    void (*pfnClose)(void *pvInstance);

    //
    //! This is the optional interrupt handler that will be called when an
    //! endpoint associated with this device instance generates an interrupt.
    //
    void (*pfnIntHandler)(void *pvInstance);
}
tUSBHostClassDriver;

//*****************************************************************************
//
//! This structure is used to return generic event based information to an
//! application.  The following events are currently supported:
//! USB_EVENT_CONNECTED, USB_EVENT_DISCONNECTED, and USB_EVENT_POWER_FAULT.
//
//*****************************************************************************
typedef struct
{
    unsigned long ulEvent;

    unsigned long ulInstance;
}
tEventInfo;

//*****************************************************************************
//
// This is the type definition a callback for events on USB Pipes allocated
// by USBHCDPipeAlloc().
//
// \param ulPipe is well the pipe
// \param ulEvent is well the event
//
// longer def thand may need more text in order to be recoginzed what should
// this really say about ourselves.
//
// \return None.
//
//*****************************************************************************
typedef void (* tHCDPipeCallback)(unsigned long ulPipe,
                                  unsigned long ulEvent);

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// If the g_USBEventDriver is included in the host controller driver list then
// this funtion must be provided by the application.
//
//*****************************************************************************
void USBHCDEvents(void *pvData);

//*****************************************************************************
//
// Prototypes for the USB Host controller APIs.
//
//*****************************************************************************
extern void USBHCDMain(void);
extern void USBHCDInit(unsigned long ulIndex, void *pData,
                       unsigned long ulSize);
extern void USBHCDPowerConfigInit(unsigned long ulIndex,
                                  unsigned long ulFlags);
extern void
       USBHCDRegisterDrivers(unsigned long ulIndex,
                             const tUSBHostClassDriver * const *ppHClassDrvrs,
                             unsigned long ulNumDrivers);
extern void USBHCDTerm(unsigned long ulIndex);
extern void USBHCDSetConfig(unsigned long ulIndex, unsigned long ulDevice,
                            unsigned long ulConfiguration);
extern void USBHCDSuspend(unsigned long ulIndex);
extern void USBHCDResume(unsigned long ulIndex);
extern void USBHCDReset(unsigned long ulIndex);
extern void USBHCDPipeFree(unsigned long ulPipe);
extern unsigned long USBHCDPipeAlloc(unsigned long ulIndex,
                                     unsigned long ulEndpointType,
                                     unsigned long ulDevAddr,
                                     tHCDPipeCallback pCallback);
extern unsigned long USBHCDPipeConfig(unsigned long ulPipe,
                                      unsigned long ulMaxPayload,
                                      unsigned long ulInterval,
                                      unsigned long ulTargetEndpoint);
extern unsigned long USBHCDPipeStatus(unsigned long ulPipe);
extern unsigned long USBHCDPipeWrite(unsigned long ulPipe,
                                     unsigned char *pData,
                                     unsigned long ulSize);
extern unsigned long USBHCDPipeRead(unsigned long ulPipe, unsigned char *pData,
                                    unsigned long ulSize);
extern unsigned long USBHCDPipeSchedule(unsigned long ulPipe,
                                        unsigned char *pucData,
                                        unsigned long ulSize);
extern unsigned long USBHCDPipeReadNonBlocking(unsigned long ulPipe,
                                               unsigned char *pucData,
                                               unsigned long ulSize);
extern unsigned long USBHCDControlTransfer(unsigned long ulIndex,
                                           tUSBRequest *pSetupPacket,
                                           unsigned long ulAddress,
                                           unsigned char *pData,
                                           unsigned long ulSize,
                                           unsigned long ulMaxPacketSize);
extern void USB0HostIntHandler(void);

//*****************************************************************************
//
// The host class driver definition for the USB MSC host driver.
//
//*****************************************************************************
extern const tUSBHostClassDriver g_USBHostMSCClassDriver;
extern const tUSBHostClassDriver g_USBHIDClassDriver;

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __USBHOST_H__
