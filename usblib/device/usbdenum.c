//*****************************************************************************
//
// usbenum.c - Enumeration code to handle all endpoint zero traffic.
//
// Copyright (c) 2007-2009 Luminary Micro, Inc.  All rights reserved.
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

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/usb.h"
#include "usblib/usblib.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdevicepriv.h"
#include "usblib/usblibpriv.h"

//*****************************************************************************
//
// External prototypes.
//
//*****************************************************************************
extern tUSBMode g_eUSBMode;

//*****************************************************************************
//
// Local functions prototypes.
//
//*****************************************************************************
static void USBDGetStatus(unsigned long ulIndex, tUSBRequest *pUSBRequest);
static void USBDClearFeature(unsigned long ulIndex, tUSBRequest *pUSBRequest);
static void USBDSetFeature(unsigned long ulIndex, tUSBRequest *pUSBRequest);
static void USBDSetAddress(unsigned long ulIndex, tUSBRequest *pUSBRequest);
static void USBDGetDescriptor(unsigned long ulIndex, tUSBRequest *pUSBRequest);
static void USBDSetDescriptor(unsigned long ulIndex, tUSBRequest *pUSBRequest);
static void USBDGetConfiguration(unsigned long ulIndex,
                                 tUSBRequest *pUSBRequest);
static void USBDSetConfiguration(unsigned long ulIndex,
                                 tUSBRequest *pUSBRequest);
static void USBDGetInterface(unsigned long ulIndex, tUSBRequest *pUSBRequest);
static void USBDSetInterface(unsigned long ulIndex, tUSBRequest *pUSBRequest);
static void USBDSyncFrame(unsigned long ulIndex, tUSBRequest *pUSBRequest);
static void USBDEP0StateTx(unsigned long ulIndex);
static void USBDEP0StateTxConfig(unsigned long ulIndex);
static long USBDStringIndexFromRequest(unsigned short usLang,
                                       unsigned short usIndex);

//*****************************************************************************
//
//! \addtogroup device_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! The default USB endpoint FIFO configuration structure.  This structure
//! contains definitions to set all USB FIFOs into single buffered mode with
//! no DMA use.  Each endpoint's FIFO is sized to hold the largest maximum
//! packet size for any interface alternate setting in the current config
//! descriptor.  A pointer to this structure may be passed in the psFIFOConfig
//! field of the tDeviceInfo structure passed to USBCDCInit if the application
//! does not require any special handling of the USB controller FIFO.
//
//*****************************************************************************
const tFIFOConfig g_sUSBDefaultFIFOConfig =
{
    {
        { 1, false, 0 },
        { 1, false, 0 },
        { 1, false, 0 }
    },
    {
        { 1, false, 0 },
        { 1, false, 0 },
        { 1, false, 0 }
    },
};

//*****************************************************************************
//
// This structure holds the full state for the device enumeration.
//
//*****************************************************************************
typedef struct
{
    //
    // The devices current address, this also has a change pending bit in the
    // MSB of this value specified by DEV_ADDR_PENDING.
    //
    volatile unsigned long ulDevAddress;

    //
    // This holds the current active configuration for this device.
    //
    unsigned long ulConfiguration;

    //
    // This holds the configuration id that will take effect after a reset.
    //
    unsigned long ulDefaultConfiguration;

    //
    // This holds the current alternate interface for this device.
    //
    unsigned char pucAltSetting[USB_MAX_INTERFACES_PER_DEVICE];

    //
    // This is the pointer to the current data being sent out or received
    // on endpoint zero.
    //
    unsigned char *pEP0Data;

    //
    // This is the number of bytes that remain to be sent from or received
    // into the g_sUSBDeviceState.pEP0Data data buffer.
    //
    volatile unsigned long ulEP0DataRemain;

    //
    // The amount of data being sent/received due to a custom request.
    //
    unsigned long ulOUTDataSize;

    //
    // Holds the current device status.
    //
    unsigned char ucStatus;

    //
    // Holds the endpoint status for the HALT condition.  This array is sized
    // to hold halt status for all IN and OUT endpoints.
    //
    unsigned char ucHalt[2][NUM_USB_EP - 1];

    //
    // Holds the config descriptor section number currently being sent
    // to the host.
    //
    unsigned char ucConfigSection;

    //
    // Holds the offset within the config descriptor section currently being
    // sent to the host.
    //
    unsigned char ucSectionOffset;

    //
    // Holds the index of the configuration that we are currently sending back
    // to the host.
    //
    unsigned char ucConfigIndex;

    //
    // This flag is set to true if the client has called USBDPowerStatusSet
    // and tells the USB library not to try to determine the current power
    // status from the config descriptor.
    //
    tBoolean bPwrSrcSet;

    //
    // This flag indicates whether or not remote wakeup signaling is in
    // progress.
    //
    tBoolean bRemoteWakeup;

    //
    // During remote wakeup signaling, this counter is used to track the
    // number of milliseconds since the signaling was initiated.
    //
    unsigned char ucRemoteWakeupCount;
}
tDeviceState;

//*****************************************************************************
//
// Indices into the ucHalt array to select the IN or OUT endpoint group.
//
//*****************************************************************************
#define HALT_EP_IN              0
#define HALT_EP_OUT             1

//*****************************************************************************
//
// The states for endpoint zero during enumeration.
//
//*****************************************************************************
typedef enum
{
    //
    // The USB device is waiting on a request from the host controller on
    // endpoint zero.
    //
    USB_STATE_IDLE,

    //
    // The USB device is sending data back to the host due to an IN request.
    //
    USB_STATE_TX,

    //
    // The USB device is sending the config descriptor back to the host due
    // to an IN request.
    //
    USB_STATE_TX_CONFIG,

    //
    // The USB device is receiving data from the host due to an OUT
    // request from the host.
    //
    USB_STATE_RX,

    //
    // The USB device has completed the IN or OUT request and is now waiting
    // for the host to acknowledge the end of the IN/OUT transaction.  This
    // is the status phase for a USB control transaction.
    //
    USB_STATE_STATUS,

    //
    // This endpoint has signaled a stall condition and is waiting for the
    // stall to be acknowledged by the host controller.
    //
    USB_STATE_STALL
}
tEP0State;

//*****************************************************************************
//
// Define the max packet size for endpoint zero.
//
//*****************************************************************************
#define EP0_MAX_PACKET_SIZE     64

//*****************************************************************************
//
// This is a flag used with g_sUSBDeviceState.ulDevAddress to indicate that a
// device address change is pending.
//
//*****************************************************************************
#define DEV_ADDR_PENDING        0x80000000

//*****************************************************************************
//
// This label defines the default configuration number to use after a bus
// reset.  This may be overridden by calling USBDCDSetDefaultConfiguration()
// during processing of the device reset handler if required.
//
//*****************************************************************************
#define DEFAULT_CONFIG_ID       1

//*****************************************************************************
//
// This label defines the number of milliseconds that the remote wakeup signal
// must remain asserted before removing it. Section 7.1.7.7 of the USB 2.0 spec
// states that "the remote wakeup device must hold the resume signaling for at
// least 1ms but for no more than 15ms" so 10mS seems a reasonable choice.
//
//*****************************************************************************
#define REMOTE_WAKEUP_PULSE_MS 10

//*****************************************************************************
//
// This label defines the number of milliseconds between the point where we
// assert the remote wakeup signal and calling the client back to tell it that
// bus operation has been resumed.  This value is based on the timings provided
// in section 7.1.7.7 of the USB 2.0 specification which indicates that the host
// (which takes over resume signaling when the device's initial signal is
// detected) must hold the resume signaling for at least 20mS.
//
//*****************************************************************************
#define REMOTE_WAKEUP_READY_MS 20

//*****************************************************************************
//
// The buffer for reading data coming into EP0
//
//*****************************************************************************
static unsigned char g_pucDataBufferIn[EP0_MAX_PACKET_SIZE];

//*****************************************************************************
//
// The pointer to the device's information structure as passed on a call to
// USBDCDInit().
//
//*****************************************************************************
tDeviceInfo *g_psUSBDeviceInfo;

//*****************************************************************************
//
// This global holds the current state information for the USB device.
//
//*****************************************************************************
static volatile tDeviceState g_sUSBDeviceState;

//*****************************************************************************
//
// This global holds the current state of endpoint zero.
//
//*****************************************************************************
static volatile tEP0State g_eUSBDEP0State = USB_STATE_IDLE;

//*****************************************************************************
//
// Function table to handle standard requests.
//
//*****************************************************************************
static const tStdRequest g_psUSBDStdRequests[] =
{
    USBDGetStatus,
    USBDClearFeature,
    0,
    USBDSetFeature,
    0,
    USBDSetAddress,
    USBDGetDescriptor,
    USBDSetDescriptor,
    USBDGetConfiguration,
    USBDSetConfiguration,
    USBDGetInterface,
    USBDSetInterface,
    USBDSyncFrame
};

//*****************************************************************************
//
// Functions accessible by USBLIB clients.
//
//*****************************************************************************

//*****************************************************************************
//
//! Initialize the USB library device control driver for a given hardware
//! controller.
//!
//! \param ulIndex is the index of the USB controller which is to be
//! initialized.
//! \param psDevice is a pointer to a structure containing information that
//! the USB library requires to support operation of this application's
//! device.  The structure contains event handler callbacks and pointers to the
//! various standard descriptors that the device wishes to publish to the
//! host.
//!
//! This function must be called by any application which wishes to operate
//! as a USB device.  It initializes the USB device control driver for the
//! given controller and saves the device information for future use.  Prior to
//! returning from this function, the device is connected to the USB bus.
//! Following return, the caller can expect to receive a callback to the
//! supplied <tt>pfnResetHandler</tt> function when a host connects to the
//! device.
//!
//! The device information structure passed in \e psDevice must remain
//! unchanged between this call and any matching call to USBDCDTerm() since
//! it is not copied by the USB library.
//!
//! \return None.
//
//*****************************************************************************
void
USBDCDInit(unsigned long ulIndex, tDeviceInfo *psDevice)
{
    const tConfigHeader *psHdr;
    const tConfigDescriptor *psDesc;

    //
    // Check the arguments.
    //
    ASSERT(ulIndex == 0);
    ASSERT(psDevice != 0);

    //
    // Make sure someone else has not already claimed this controller.
    //
    ASSERT(g_psUSBDeviceInfo == 0);

    //
    // Should not call this if the stack is in host mode.
    //
    ASSERT(g_eUSBMode != USB_MODE_HOST)

    //
    // Initialize a couple of fields in the device state structure.
    //
    g_sUSBDeviceState.ulConfiguration = DEFAULT_CONFIG_ID;
    g_sUSBDeviceState.ulDefaultConfiguration = DEFAULT_CONFIG_ID;

    //
    // Remember the device information pointer.
    //
    g_psUSBDeviceInfo = psDevice;

    //
    // If no mode is set then make the mode become device mode.
    //
    if(g_eUSBMode == USB_MODE_NONE)
    {
        g_eUSBMode = USB_MODE_DEVICE;
    }

    //
    // Only do hardware update if the stack is in Device mode, do not touch the
    // hardware for OTG mode operation.
    //
    if(g_eUSBMode == USB_MODE_DEVICE)
    {
        //
        // Enable Clocking to the USB controller.
        //
        SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);

        //
        // Turn on USB Phy clock.
        //
        SysCtlUSBPLLEnable();
    }

    //
    // Initialize the USB tick module.
    //
    InternalUSBTickInit();

    //
    // Only do hardware update if the stack is in Device mode, do not touch the
    // hardware for OTG mode operation.
    //
    if(g_eUSBMode == USB_MODE_DEVICE)
    {
        //
        // Ask for the interrupt status.  As a side effect, this clears all
        // pending USB interrupts.
        //
        USBIntStatusControl(USB0_BASE);
        USBIntStatusEndpoint(USB0_BASE);

        //
        // Enable USB Interrupts.
        //
        USBIntEnableControl(USB0_BASE, USB_INTCTRL_RESET |
                                       USB_INTCTRL_DISCONNECT |
                                       USB_INTCTRL_RESUME |
                                       USB_INTCTRL_SUSPEND |
                                       USB_INTCTRL_SOF);
        USBIntEnableEndpoint(USB0_BASE, USB_INTEP_ALL);
    }

    //
    // Get a pointer to the default config descriptor.
    //
    psHdr = g_psUSBDeviceInfo->ppConfigDescriptors[
                                 g_sUSBDeviceState.ulDefaultConfiguration - 1];
    psDesc = (const tConfigDescriptor *)(psHdr->psSections[0]->pucData);

    //
    // Default to the state where remote wakeup is disabled.
    //
    g_sUSBDeviceState.ucStatus = 0;
    g_sUSBDeviceState.bRemoteWakeup = false;

    //
    // Determine the self- or bus-powered state based on the flags the
    // user provided.
    //
    g_sUSBDeviceState.bPwrSrcSet = false;
    if((psDesc->bmAttributes & USB_CONF_ATTR_PWR_M) == USB_CONF_ATTR_SELF_PWR)
    {
        g_sUSBDeviceState.ucStatus |= USB_STATUS_SELF_PWR;
    }
    else
    {
        g_sUSBDeviceState.ucStatus &= ~USB_STATUS_SELF_PWR;
    }

    //
    // Only do hardware update if the stack is in Device mode, do not touch the
    // hardware for OTG mode operation.
    //
    if(g_eUSBMode == USB_MODE_DEVICE)
    {
        //
        // Make sure we disconnect from the host for a while.  This ensures
        // that the host will reenumerate us even if we were previously
        // connected to the bus.
        //
        USBDevDisconnect(USB0_BASE);

        //
        // Wait about 100mS.
        //
        SysCtlDelay(SysCtlClockGet() / 30);

        //
        // Attach the device using the soft connect.
        //
        USBDevConnect(USB0_BASE);

        //
        // Enable the USB interrupt.
        //
        IntEnable(INT_USB0);
    }
}

//*****************************************************************************
//
//! Free the USB library device control driver for a given hardware controller.
//!
//! \param ulIndex is the index of the USB controller which is to be
//! freed.
//!
//! This function should be called by an application if it no longer requires
//! the use of a given USB controller to support its operation as a USB device.
//! It frees the controller for use by another client.
//!
//! It is the caller's responsibility to remove its device from the USB bus
//! prior to calling this function.
//!
//! \return None.
//
//*****************************************************************************
void
USBDCDTerm(unsigned long ulIndex)
{
    //
    // Check the arguments.
    //
    ASSERT(ulIndex == 0);

    g_psUSBDeviceInfo = (tDeviceInfo *)0;

    //
    // Disable the USB interrupts.
    //
    IntDisable(INT_USB0);

    USBIntDisableControl(USB0_BASE, USB_INTCTRL_ALL);
    USBIntDisableEndpoint(USB0_BASE, USB_INTEP_ALL);

    //
    // Detach the device using the soft connect.
    //
    USBDevDisconnect(USB0_BASE);

    //
    // Clear any pending interrupts.
    //
    USBIntStatusControl(USB0_BASE);
    USBIntStatusEndpoint(USB0_BASE);

    //
    // Turn off USB Phy clock.
    //
    SysCtlUSBPLLDisable();

    //
    // Disable the USB peripheral
    //
    SysCtlPeripheralDisable(SYSCTL_PERIPH_USB0);
}

//*****************************************************************************
//
//! This function starts the request for data from the host on endpoint zero.
//!
//! \param ulIndex is the index of the USB controller from which the data
//! is being requested.
//! \param pucData is a pointer to the buffer to fill with data from the USB
//! host.
//! \param ulSize is the size of the buffer or data to return from the USB
//! host.
//!
//! This function handles retrieving data from the host when a custom command
//! has been issued on endpoint zero.  If the application needs notification
//! when the data has been received,
//! <tt>tDeviceInfo.sCallbacks.pfnDataReceived</tt> should contain valid
//! function pointer.  In nearly all cases this is necessary because the caller
//! of this function would likely need to know that the data requested was
//! received.
//!
//! \return None.
//
//*****************************************************************************
void
USBDCDRequestDataEP0(unsigned long ulIndex, unsigned char *pucData,
                     unsigned long ulSize)
{
    ASSERT(ulIndex == 0);

    //
    // Enter the RX state on end point 0.
    //
    g_eUSBDEP0State = USB_STATE_RX;

    //
    // Save the pointer to the data.
    //
    g_sUSBDeviceState.pEP0Data = pucData;

    //
    // Location to save the current number of bytes received.
    //
    g_sUSBDeviceState.ulOUTDataSize = ulSize;

    //
    // Bytes remaining to be received.
    //
    g_sUSBDeviceState.ulEP0DataRemain = ulSize;
}

//*****************************************************************************
//
//! This function requests transfer of data to the host on endpoint zero.
//!
//! \param ulIndex is the index of the USB controller which is to be used to
//! send the data.
//! \param pucData is a pointer to the buffer to send via endpoint zero.
//! \param ulSize is the amount of data to send in bytes.
//!
//! This function handles sending data to the host when a custom command is
//! issued or non-standard descriptor has been requested on endpoint zero.  If
//! the application needs notification when this is complete,
//! <tt>tDeviceInfo.sCallbacks.pfnDataSent</tt> should contain a valid function
//! pointer.  This callback could be used to free up the buffer passed into
//! this function in the \e pucData parameter.  The contents of the \e pucData
//! buffer must remain unchanged until the <tt>pfnDataSent</tt> callback is
//! received.
//!
//! \return None.
//
//*****************************************************************************
void
USBDCDSendDataEP0(unsigned long ulIndex, unsigned char *pucData,
                  unsigned long ulSize)
{
    ASSERT(ulIndex == 0);

    //
    // Return the externally provided device descriptor.
    //
    g_sUSBDeviceState.pEP0Data = pucData;

    //
    // The size of the device descriptor is in the first byte.
    //
    g_sUSBDeviceState.ulEP0DataRemain = ulSize;

    //
    // Save the total size of the data sent.
    //
    g_sUSBDeviceState.ulOUTDataSize = ulSize;

    //
    // Now in the transmit data state.
    //
    USBDEP0StateTx(0);
}

//*****************************************************************************
//
//! This function sets the default configuration for the device.
//!
//! \param ulIndex is the index of the USB controller whose default
//! configuration is to be set.
//! \param ulDefaultConfig is the configuration identifier (byte 6 of the
//! standard configuration descriptor) which is to be presented to the host
//! as the default configuration in cases where the config descriptor is
//! queried prior to any specific configuration being set.
//!
//! This function allows a device to override the default configuration
//! descriptor that will be returned to a host whenever it is queried prior
//! to a specific configuration having been set.  The parameter passed must
//! equal one of the configuration identifiers found in the
//! <tt>ppConfigDescriptors</tt> array for the device.
//!
//! If this function is not called, the USB library will return the first
//! configuration in the <tt>ppConfigDescriptors</tt> array as the default
//! configuration.
//!
//! \note The USB device stack assumes that the configuration IDs (byte 6 of
//! the config descriptor, <tt>bConfigurationValue</tt>) stored within the
//! configuration descriptor array, <tt>ppConfigDescriptors</tt>,
//! are equal to the array index + 1.  In other words, the first entry in the
//! array must contain a descriptor with <tt>bConfigurationValue</tt> 1, the
//! second must have <tt>bConfigurationValue</tt> 2 and so on.
//!
//! \return None.
//
//*****************************************************************************
void
USBDCDSetDefaultConfiguration(unsigned long ulIndex,
                              unsigned long ulDefaultConfig)
{
    ASSERT(ulIndex == 0);

    g_sUSBDeviceState.ulDefaultConfiguration = ulDefaultConfig;
}

//*****************************************************************************
//
//! This function generates a stall condition on endpoint zero.
//!
//! \param ulIndex is the index of the USB controller whose endpoint zero is to
//! be stalled.
//!
//! This function is typically called to signal an error condition to the host
//! when an unsupported request is received by the device.  It should be
//! called from within the callback itself (in interrupt context) and not
//! deferred until later since it affects the operation of the endpoint zero
//! state machine in the USB library.
//!
//! \return None.
//
//*****************************************************************************
void
USBDCDStallEP0(unsigned long ulIndex)
{
    ASSERT(ulIndex == 0);

    //
    // Stall the endpoint in question.
    //
    USBDevEndpointStall(USB0_BASE, USB_EP_0, USB_EP_DEV_OUT);

    //
    // Enter the stalled state.
    //
    g_eUSBDEP0State = USB_STATE_STALL;
}

//*****************************************************************************
//
//! Reports the device power status (bus- or self-powered) to the library.
//!
//! \param ulIndex is the index of the USB controller whose device power
//! status is being reported.
//! \param ucPower indicates the current power status, either \b
//! USB_STATUS_SELF_PWR or \b USB_STATUS_BUS_PWR.
//!
//! Applications which support switching between bus- or self-powered
//! operation should call this function whenever the power source changes
//! to indicate the current power status to the USB library.  This information
//! is required by the library to allow correct responses to be provided when
//! the host requests status from the device.
//!
//! \return None.
//
//*****************************************************************************
void
USBDCDPowerStatusSet(unsigned long ulIndex, unsigned char ucPower)
{
    //
    // Check for valid parameters.
    //
    ASSERT((ucPower == USB_STATUS_BUS_PWR) ||
           (ucPower == USB_STATUS_SELF_PWR));
    ASSERT(ulIndex == 0);

    //
    // Update the device status with the new power status flag.
    //
    g_sUSBDeviceState.bPwrSrcSet = true;
    g_sUSBDeviceState.ucStatus &= ~USB_STATUS_PWR_M;
    g_sUSBDeviceState.ucStatus |= ucPower;
}

//*****************************************************************************
//
//! Requests a remote wakeup to resume communication when in suspended state.
//!
//! \param ulIndex is the index of the USB controller that will request
//! a bus wakeup.
//!
//! When the bus is suspended, an application which supports remote wakeup
//! (advertised to the host via the config descriptor) may call this function
//! to initiate remote wakeup signaling to the host.  If the remote wakeup
//! feature has not been disabled by the host, this will cause the bus to
//! resume operation within 20mS.  If the host has disabled remote wakeup,
//! \b false will be returned to indicate that the wakeup request was not
//! successful.
//!
//! \return Returns \b true if the remote wakeup is not disabled and the
//! signaling was started or \b false if remote wakeup is disabled or if
//! signaling is currently ongoing following a previous call to this function.
//
//*****************************************************************************
tBoolean
USBDCDRemoteWakeupRequest(unsigned long ulIndex)
{
    //
    // Check for parameter validity.
    //
    ASSERT(ulIndex == 0);

    //
    // Is remote wakeup signaling currently enabled?
    //
    if(g_sUSBDeviceState.ucStatus & USB_STATUS_REMOTE_WAKE)
    {
        //
        // The host has not disabled remote wakeup. Are we still in the
        // middle of a previous wakeup sequence?
        //
        if(!g_sUSBDeviceState.bRemoteWakeup)
        {
            //
            // No - we are not in the middle of a wakeup sequence so start
            // one here.
            //
            g_sUSBDeviceState.ucRemoteWakeupCount = 0;
            g_sUSBDeviceState.bRemoteWakeup = true;
            USBHostResume(USB0_BASE, true);
            return(true);
        }
    }

    //
    // If we drop through to here, signaling was not initiated so return
    // false.
    return(false);
}

//*****************************************************************************
//
// Internal Functions, not to be called by applications
//
//*****************************************************************************

//*****************************************************************************
//
// This internal function is called on the SOF interrupt to process any
// outstanding remote wakeup requests.
//
// \return None.
//
//*****************************************************************************
void USBDeviceResumeTickHandler(unsigned long ulIndex)
{
    if(g_sUSBDeviceState.bRemoteWakeup)
    {
        //
        // Increment the millisecond counter we use to time the resume
        // signaling.
        //
        g_sUSBDeviceState.ucRemoteWakeupCount++;

        //
        // Have we reached the 10mS mark? If so, we need to turn the signaling
        // off again.
        //
        if(g_sUSBDeviceState.ucRemoteWakeupCount == REMOTE_WAKEUP_PULSE_MS)
        {
            USBHostResume(USB0_BASE, false);
        }

        //
        // Have we reached the point at which we can tell the client that the
        // bus has resumed? The controller doesn't give us an interrupt if we
        // initiated the wakeup signaling so we just wait until 20mS have
        // passed then tell the client all is well.
        //
        if(g_sUSBDeviceState.ucRemoteWakeupCount == REMOTE_WAKEUP_READY_MS)
        {
            //
            // We are now finished with the remote wakeup signaling.
            //
            g_sUSBDeviceState.bRemoteWakeup = false;

            //
            // If the client has registered a resume callback, call it.  In the
            // case of a remote wakeup request, we do not get a resume interrupt
            // from the controller so we need to fake it here.
            //
            if(g_psUSBDeviceInfo->sCallbacks.pfnResumeHandler)
            {
                g_psUSBDeviceInfo->sCallbacks.pfnResumeHandler(0);
            }
        }
    }
}

//*****************************************************************************
//
// This internal function reads a request data packet and dispatches it to
// either a standard request handler or the registered device request
// callback depending upon the request type.
//
// \return None.
//
//*****************************************************************************
static void
USBDReadAndDispatchRequest(void)
{
    unsigned long ulSize;
    tUSBRequest *pRequest;

    //
    // Cast the buffer to a request structure.
    //
    pRequest = (tUSBRequest *)g_pucDataBufferIn;

    //
    // Set the buffer size.
    //
    ulSize = EP0_MAX_PACKET_SIZE;

    //
    // Get the data from the USB controller end point 0.
    //
    USBEndpointDataGet(USB0_BASE,
                       USB_EP_0,
                       g_pucDataBufferIn,
                       &ulSize);

    if(!ulSize)
    {
        return;
    }

    //
    // See if this is a standard request or not.
    //
    if((pRequest->bmRequestType & USB_RTYPE_TYPE_M) != USB_RTYPE_STANDARD)
    {
        //
        // Since this is not a standard request, see if there is
        // an external handler present.
        //
        if(g_psUSBDeviceInfo->sCallbacks.pfnRequestHandler)
        {
            g_psUSBDeviceInfo->sCallbacks.pfnRequestHandler(0, pRequest);
        }
        else
        {
            //
            // If there is no handler then stall this request.
            //
            USBDCDStallEP0(0);
        }
    }
    else
    {
        //
        // Assure that the jump table is not out of bounds.
        //
        if((pRequest->bRequest <
           (sizeof(g_psUSBDStdRequests) / sizeof(tStdRequest))) &&
           (g_psUSBDStdRequests[pRequest->bRequest] != 0))
        {
            //
            // Jump table to the appropriate handler.
            //
            g_psUSBDStdRequests[pRequest->bRequest](0, pRequest);
        }
        else
        {
            //
            // If there is no handler then stall this request.
            //
            USBDCDStallEP0(0);
        }
    }
}

//*****************************************************************************
//
// This is interrupt handler for endpoint zero.
//
// This function handles all interrupts on endpoint zero in order to maintain
// the state needed for the control endpoint on endpoint zero.  In order to
// successfully enumerate and handle all USB standard requests, all requests
// on endpoint zero must pass through this function.  The endpoint has the
// following states: \b USB_STATE_IDLE, \b USB_STATE_TX, \b USB_STATE_RX,
// \b USB_STATE_STALL, and \b USB_STATE_STATUS.  In the \b USB_STATE_IDLE
// state the USB controller has not received the start of a request, and once
// it does receive the data for the request it will either enter the
// \b USB_STATE_TX, \b USB_STATE_RX, or \b USB_STATE_STALL depending on the
// command.  If the controller enters the \b USB_STATE_TX or \b USB_STATE_RX
// then once all data has been sent or received, it must pass through the
// \b USB_STATE_STATUS state to allow the host to acknowledge completion of
// the request.  The \b USB_STATE_STALL is entered from \b USB_STATE_IDLE in
// the event that the USB request was not valid.  Both the \b USB_STATE_STALL
// and \b USB_STATE_STATUS are transitional states that return to the
// \b USB_STATE_IDLE state.
//
// \return None.
//
// USB_STATE_IDLE -*--> USB_STATE_TX -*-> USB_STATE_STATUS -*->USB_STATE_IDLE
//                 |                  |                     |
//                 |--> USB_STATE_RX -                      |
//                 |                                        |
//                 |--> USB_STATE_STALL ---------->---------
//
//  ----------------------------------------------------------------
// | Current State       | State 0           | State 1              |
// | --------------------|-------------------|----------------------
// | USB_STATE_IDLE      | USB_STATE_TX/RX   | USB_STATE_STALL      |
// | USB_STATE_TX        | USB_STATE_STATUS  |                      |
// | USB_STATE_RX        | USB_STATE_STATUS  |                      |
// | USB_STATE_STATUS    | USB_STATE_IDLE    |                      |
// | USB_STATE_STALL     | USB_STATE_IDLE    |                      |
//  ----------------------------------------------------------------
//
//*****************************************************************************
void
USBDeviceEnumHandler(void)
{
    unsigned long ulEPStatus;

    //
    // Get the end point 0 status.
    //
    ulEPStatus = USBEndpointStatus(USB0_BASE, USB_EP_0);

    switch(g_eUSBDEP0State)
    {
        //
        // Handle the status state, this is a transitory state from
        // USB_STATE_TX or USB_STATE_RX back to USB_STATE_IDLE.
        //
        case USB_STATE_STATUS:
        {
            //
            // Just go back to the idle state.
            //
            g_eUSBDEP0State = USB_STATE_IDLE;

            //
            // If there is a pending address change then set the address.
            //
            if(g_sUSBDeviceState.ulDevAddress & DEV_ADDR_PENDING)
            {
                //
                // Clear the pending address change and set the address.
                //
                g_sUSBDeviceState.ulDevAddress &= ~DEV_ADDR_PENDING;
                USBDevAddrSet(USB0_BASE, g_sUSBDeviceState.ulDevAddress);
            }

            //
            // If a new packet is already pending, we need to read it
            // and handle whatever request it contains.
            //
            if(ulEPStatus & USB_DEV_EP0_OUT_PKTRDY)
            {
                //
                // Process the newly arrived packet.
                //
                USBDReadAndDispatchRequest();
            }
            break;
        }

        //
        // In the IDLE state the code is waiting to receive data from the host.
        //
        case USB_STATE_IDLE:
        {
            //
            // Is there a packet waiting for us?
            //
            if(ulEPStatus & USB_DEV_EP0_OUT_PKTRDY)
            {
                //
                // Yes - process it.
                //
                USBDReadAndDispatchRequest();
            }
            break;
        }

        //
        // Data is still being sent to the host so handle this in the
        // EP0StateTx() function.
        //
        case USB_STATE_TX:
        {
            USBDEP0StateTx(0);
            break;
        }

        //
        // We are still in the middle of sending the config descriptor so
        // handle this in the EP0StateTxConfig() function.
        //
        case USB_STATE_TX_CONFIG:
        {
            USBDEP0StateTxConfig(0);
            break;
        }

        //
        // Handle the receive state for commands that are receiving data on
        // endpoint zero.
        //
        case USB_STATE_RX:
        {
            unsigned long ulDataSize;

            //
            // Set the number of bytes to get out of this next packet.
            //
            if(g_sUSBDeviceState.ulEP0DataRemain > EP0_MAX_PACKET_SIZE)
            {
                //
                // Don't send more than EP0_MAX_PACKET_SIZE bytes.
                //
                ulDataSize = EP0_MAX_PACKET_SIZE;
            }
            else
            {
                //
                // There was space so send the remaining bytes.
                //
                ulDataSize = g_sUSBDeviceState.ulEP0DataRemain;
            }

            //
            // Get the data from the USB controller end point 0.
            //
            USBEndpointDataGet(USB0_BASE, USB_EP_0, g_sUSBDeviceState.pEP0Data,
                               &ulDataSize);

            //
            // If there we not more that EP0_MAX_PACKET_SIZE or more bytes
            // remaining then this transfer is complete.  If there were exactly
            // EP0_MAX_PACKET_SIZE remaining then there still needs to be
            // null packet sent before this is complete.
            //
            if(g_sUSBDeviceState.ulEP0DataRemain < EP0_MAX_PACKET_SIZE)
            {
                //
                // Need to ack the data on end point 0 in this case
                // without setting data end.
                //
                USBDevEndpointDataAck(USB0_BASE, USB_EP_0, true);

                //
                // Return to the idle state.
                //
                g_eUSBDEP0State =  USB_STATE_IDLE;

                //
                // If there is a receive callback then call it.
                //
                if((g_psUSBDeviceInfo->sCallbacks.pfnDataReceived) &&
                   (g_sUSBDeviceState.ulOUTDataSize != 0))
                {
                    //
                    // Call the custom receive handler to handle the data
                    // that was received.
                    //
                    g_psUSBDeviceInfo->sCallbacks.pfnDataReceived(0,
                        g_sUSBDeviceState.ulOUTDataSize);

                    //
                    // Indicate that there is no longer any data being waited
                    // on.
                    //
                    g_sUSBDeviceState.ulOUTDataSize = 0;
                }
            }
            else
            {
                //
                // Need to ack the data on end point 0 in this case
                // without setting data end.
                //
                USBDevEndpointDataAck(USB0_BASE, USB_EP_0, false);
            }

            //
            // Advance the pointer.
            //
            g_sUSBDeviceState.pEP0Data += ulDataSize;

            //
            // Decrement the number of bytes that are being waited on.
            //
            g_sUSBDeviceState.ulEP0DataRemain -= ulDataSize;

            break;
        }
        //
        // The device stalled endpoint zero so check if the stall needs to be
        // cleared once it has been successfully sent.
        //
        case USB_STATE_STALL:
        {
            //
            // If we sent a stall then acknowledge this interrupt.
            //
            if(ulEPStatus & USB_DEV_EP0_SENT_STALL)
            {
                //
                // Clear the Setup End condition.
                //
                USBDevEndpointStatusClear(USB0_BASE, USB_EP_0,
                                          USB_DEV_EP0_SENT_STALL);

                //
                // Reset the global end point 0 state to IDLE.
                //
                g_eUSBDEP0State = USB_STATE_IDLE;

            }
            break;
        }
        //
        // Halt on an unknown state, but only in DEBUG mode builds.
        //
        default:
        {
            ASSERT(0);
        }
    }
}

//*****************************************************************************
//
// This function handles bus reset notifications.
//
// This function is called from the low level USB interrupt handler whenever
// a bus reset is detected.  It performs tidy-up as required and resets the
// configuration back to defaults in preparation for descriptor queries from
// the host.
//
// \return None.
//
//*****************************************************************************
void
USBDeviceEnumResetHandler(void)
{
    unsigned long ulLoop;

    //
    // Disable remote wakeup signaling (as per USB 2.0 spec 9.1.1.6).
    //
    g_sUSBDeviceState.ucStatus &= ~USB_STATUS_REMOTE_WAKE;
    g_sUSBDeviceState.bRemoteWakeup = false;

    //
    // Call the device dependent code to indicate a bus reset has occurred.
    //
    if(g_psUSBDeviceInfo->sCallbacks.pfnResetHandler)
    {
        g_psUSBDeviceInfo->sCallbacks.pfnResetHandler(0);
    }

    //
    // Reset the default configuration identifier and alternate function
    // selections.
    //
    g_sUSBDeviceState.ulConfiguration =
        g_sUSBDeviceState.ulDefaultConfiguration;
    for(ulLoop = 0; ulLoop < USB_MAX_INTERFACES_PER_DEVICE; ulLoop++)
    {
        g_sUSBDeviceState.pucAltSetting[ulLoop] = (unsigned char)0;
    }
}

//*****************************************************************************
//
// This function handles the GET_STATUS standard USB request.
//
// \param ulIndex is the index of the USB controller which is to be
// initialized.
// \param pUSBRequest holds the request type and endpoint number if endpoint
// status is requested.
//
// This function handles responses to a Get Status request from the host
// controller.  A status request can be for the device, an interface or an
// endpoint.  If any other type of request is made this function will cause
// a stall condition to indicate that the command is not supported.  The
// \e pUSBRequest structure holds the type of the request in the
// bmRequestType field.  If the type indicates that this is a request for an
// endpoint's status, then the wIndex field holds the endpoint number.
//
// \return None.
//
//*****************************************************************************
static void
USBDGetStatus(unsigned long ulIndex, tUSBRequest *pUSBRequest)
{
    unsigned short usData;

    ASSERT(ulIndex == 0);

    //
    // Determine what type of status was requested.
    //
    switch(pUSBRequest->bmRequestType & USB_RTYPE_RECIPIENT_M)
    {
        //
        // This was a Device Status request.
        //
        case USB_RTYPE_DEVICE:
        {
            //
            // Return the current status for the device.
            //
            usData = (unsigned short)g_sUSBDeviceState.ucStatus;

            break;
        }

        //
        // This was a Interface status request.
        //
        case USB_RTYPE_INTERFACE:
        {
            //
            // Interface status always returns 0.
            //
            usData = (unsigned short)0;

            break;
        }

        //
        // This was an endpoint status request.
        //
        case USB_RTYPE_ENDPOINT:
        {
            unsigned short usIndex;
            unsigned long ulDir;

            //
            // Which endpoint are we dealing with?
            //
            usIndex = pUSBRequest->wIndex & USB_REQ_EP_NUM_M;

            //
            // Check if this was a valid endpoint request.
            //
            if((usIndex == 0) || (usIndex >= NUM_USB_EP))
            {
                USBDCDStallEP0(0);
                return;
            }
            else
            {
                //
                // Are we dealing with an IN or OUT endpoint?
                //
                ulDir = ((pUSBRequest->wIndex & USB_REQ_EP_DIR_M) ==
                         USB_REQ_EP_DIR_IN) ? HALT_EP_IN : HALT_EP_OUT;

                //
                // Get the current halt status for this endpoint.
                //
                usData = (unsigned short)
                             g_sUSBDeviceState.ucHalt[ulDir][usIndex - 1];
            }
            break;
        }

        //
        // This was an unknown request.
        //
        default:
        {
            //
            // Anything else causes a stall condition to indicate that the
            // command was not supported.
            //
            USBDCDStallEP0(0);
            return;
        }
    }

    //
    // Send the two byte status response.
    //
    g_sUSBDeviceState.ulEP0DataRemain = 2;
    g_sUSBDeviceState.pEP0Data = (unsigned char *)&usData;

    //
    // Send the response.
    //
    USBDEP0StateTx(0);
}

//*****************************************************************************
//
// This function handles the CLEAR_FEATURE standard USB request.
//
// \param ulIndex is the index of the USB controller which is to be
// initialized.
// \param pUSBRequest holds the options for the Clear Feature USB request.
//
// This function handles device or endpoint clear feature requests.  The
// \e pUSBRequest structure holds the type of the request in the bmRequestType
// field and the feature is held in the wValue field.  For device, the only
// clearable feature is the Remote Wake feature.  This device request
// should only be made if the descriptor indicates that Remote Wake is
// implemented by the device.  For endpoint requests the only clearable
// feature is the ability to clear a halt on a given endpoint.  If any other
// requests are made, then the device will stall the request to indicate to
// the host that the command was not supported.
//
// \return None.
//
//*****************************************************************************
static void
USBDClearFeature(unsigned long ulIndex, tUSBRequest *pUSBRequest)
{
    ASSERT(ulIndex == 0);

    //
    // Determine what type of status was requested.
    //
    switch(pUSBRequest->bmRequestType & USB_RTYPE_RECIPIENT_M)
    {
        //
        // This is a clear feature request at the device level.
        //
        case USB_RTYPE_DEVICE:
        {
            //
            // Only remote wake is clearable by this function.
            //
            if(USB_FEATURE_REMOTE_WAKE & pUSBRequest->wValue)
            {
                //
                // Clear the remote wake up state.
                //
                g_sUSBDeviceState.ucStatus &= ~USB_STATUS_REMOTE_WAKE;

                //
                // Need to ack the data on end point 0.
                //
                USBDevEndpointDataAck(USB0_BASE, USB_EP_0, true);
            }
            else
            {
                USBDCDStallEP0(0);
            }
            break;
        }

        //
        // This is a clear feature request at the endpoint level.
        //
        case USB_RTYPE_ENDPOINT:
        {
            unsigned long ulDir;
            unsigned short usIndex;

            //
            // Which endpoint are we dealing with?
            //
            usIndex = pUSBRequest->wIndex & USB_REQ_EP_NUM_M;

            //
            // Not a valid endpoint.
            //
            if((usIndex == 0) || (usIndex > NUM_USB_EP))
            {
                USBDCDStallEP0(0);
            }
            else
            {
                //
                // Only the halt feature is supported.
                //
                if(USB_FEATURE_EP_HALT == pUSBRequest->wValue)
                {
                    //
                    // Are we dealing with an IN or OUT endpoint?
                    //
                    ulDir = ((pUSBRequest->wIndex & USB_REQ_EP_DIR_M) ==
                             USB_REQ_EP_DIR_IN) ? HALT_EP_IN : HALT_EP_OUT;

                    //
                    // Clear the halt condition on this endpoint.
                    //
                    g_sUSBDeviceState.ucHalt[ulDir][usIndex - 1] = 0;

                    if(ulDir == HALT_EP_IN)
                    {
                        USBDevEndpointStallClear(USB0_BASE,
                                                 INDEX_TO_USB_EP(usIndex),
                                                 USB_EP_DEV_IN);
                    }
                    else
                    {
                        USBDevEndpointStallClear(USB0_BASE,
                                                 INDEX_TO_USB_EP(usIndex),
                                                 USB_EP_DEV_OUT);
                    }
                }
                else
                {
                    //
                    // If any other feature is requested, this is an error.
                    //
                    USBDCDStallEP0(0);
                    return;
                }

                //
                // Need to ack the data on end point 0.
                //
                USBDevEndpointDataAck(USB0_BASE, USB_EP_0, true);
            }
            break;
        }

        //
        // This is an unknown request.
        //
        default:
        {
            USBDCDStallEP0(0);
            return;
        }
    }
}

//*****************************************************************************
//
// This function handles the SET_FEATURE standard USB request.
//
// \param ulIndex is the index of the USB controller which is to be
// initialized.
// \param pUSBRequest holds the feature in the wValue field of the USB
// request.
//
// This function handles device or endpoint set feature requests.  The
// \e pUSBRequest structure holds the type of the request in the bmRequestType
// field and the feature is held in the wValue field.  For device, the only
// settable feature is the Remote Wake feature.  This device request
// should only be made if the descriptor indicates that Remote Wake is
// implemented by the device.  For endpoint requests the only settable feature
// is the ability to issue a halt on a given endpoint.  If any other requests
// are made, then the device will stall the request to indicate to the host
// that the command was not supported.
//
// \return None.
//
//*****************************************************************************
static void
USBDSetFeature(unsigned long ulIndex, tUSBRequest *pUSBRequest)
{
    ASSERT(ulIndex == 0);

    //
    // Determine what type of status was requested.
    //
    switch(pUSBRequest->bmRequestType & USB_RTYPE_RECIPIENT_M)
    {
        //
        // This is a set feature request at the device level.
        //
        case USB_RTYPE_DEVICE:
        {
            //
            // Only remote wake is setable by this function.
            //
            if(USB_FEATURE_REMOTE_WAKE & pUSBRequest->wValue)
            {
                //
                // Set the remote wakeup state.
                //
                g_sUSBDeviceState.ucStatus |= USB_STATUS_REMOTE_WAKE;

                //
                // Need to ack the data on end point 0.
                //
                USBDevEndpointDataAck(USB0_BASE, USB_EP_0, true);
            }
            else
            {
                USBDCDStallEP0(0);
            }
            break;
        }

        //
        // This is a set feature request at the endpoint level.
        //
        case USB_RTYPE_ENDPOINT:
        {
            unsigned short usIndex;
            unsigned long ulDir;

            //
            // Which endpoint are we dealing with?
            //
            usIndex = pUSBRequest->wIndex & USB_REQ_EP_NUM_M;

            //
            // Not a valid endpoint?
            //
            if((usIndex == 0) || (usIndex >= NUM_USB_EP))
            {
                USBDCDStallEP0(0);
            }
            else
            {
                //
                // Only the Halt feature is settable.
                //
                if(USB_FEATURE_EP_HALT == pUSBRequest->wValue)
                {
                    //
                    // Are we dealing with an IN or OUT endpoint?
                    //
                    ulDir = ((pUSBRequest->wIndex & USB_REQ_EP_DIR_M) ==
                             USB_REQ_EP_DIR_IN) ? HALT_EP_IN : HALT_EP_OUT;

                    //
                    // Clear the halt condition on this endpoint.
                    //
                    g_sUSBDeviceState.ucHalt[ulDir][usIndex - 1] = 1;
                }
                else
                {
                    //
                    // No other requests are supported.
                    //
                    USBDCDStallEP0(0);
                    return;
                }

                //
                // Need to ack the data on end point 0.
                //
                USBDevEndpointDataAck(USB0_BASE, USB_EP_0, true);
            }
            break;
        }

        //
        // This is an unknown request.
        //
        default:
        {
            USBDCDStallEP0(0);
            return;
        }
    }
}

//*****************************************************************************
//
// This function handles the SET_ADDRESS standard USB request.
//
// \param ulIndex is the index of the USB controller which is to be
// initialized.
// \param pUSBRequest holds the new address to use in the wValue field of the
// USB request.
//
// This function is called to handle the change of address request from the
// host controller.  This can only start the sequence as the host must
// acknowledge that the device has changed address.  Thus this function sets
// the address change as pending until the status phase of the request has
// been completed successfully.  This prevents the devices address from
// changing and not properly responding to the status phase.
//
// \return None.
//
//*****************************************************************************
static void
USBDSetAddress(unsigned long ulIndex, tUSBRequest *pUSBRequest)
{
    ASSERT(ulIndex == 0);

    //
    // The data needs to be acknowledged on end point 0 without setting data
    // end because there is no data coming.
    //
    USBDevEndpointDataAck(USB0_BASE, USB_EP_0, true);

    //
    // Save the device address as we cannot change address until the status
    // phase is complete.
    //
    g_sUSBDeviceState.ulDevAddress = pUSBRequest->wValue | DEV_ADDR_PENDING;

    //
    // Transition directly to the status state since there is no data phase
    // for this request.
    //
    g_eUSBDEP0State = USB_STATE_STATUS;
}

//*****************************************************************************
//
// This function handles the GET_DESCRIPTOR standard USB request.
//
// \param ulIndex is the index of the USB controller which is to be
// initialized.
// \param pUSBRequest holds the data for this request.
//
// This function will return most of the descriptors requested by the host
// controller.  The descriptor specified by \e
// g_psUSBDeviceInfo->pDeviceDescriptor will be returned when the device
// descriptor is requested.  If a request for a specific configuration
// descriptor is made, then the appropriate descriptor from the \e
// g_pConfigDescriptors will be returned.  When a request for a string
// descriptor is made, the appropriate string from the
// \e g_psUSBDeviceInfo->pStringDescriptors will be returned.  If the \e
// g_psUSBDeviceInfo->sCallbacks.GetDescriptor is specified it will be called
// to handle the request.  In this case it must call the USBDCDSendDataEP0()
// function to send the data to the host controller.  If the callback is not
// specified, and the descriptor request is not for a device, configuration,
// or string descriptor then this function will stall the request to indicate
// that the request was not supported by the device.
//
// \return None.
//
//*****************************************************************************
static void
USBDGetDescriptor(unsigned long ulIndex, tUSBRequest *pUSBRequest)
{
    tBoolean bConfig;

    ASSERT(ulIndex == 0);

    //
    // Need to ack the data on end point 0 in this case without
    // setting data end.
    //
    USBDevEndpointDataAck(USB0_BASE, USB_EP_0, false);

    //
    // Assume we are not sending the config descriptor until we determine
    // otherwise.
    //
    bConfig = false;

    //
    // Which descriptor are we being asked for?
    //
    switch(pUSBRequest->wValue >> 8)
    {
        //
        // This request was for a device descriptor.
        //
        case USB_DTYPE_DEVICE:
        {
            //
            // Return the externally provided device descriptor.
            //
            g_sUSBDeviceState.pEP0Data =
                (unsigned char *)g_psUSBDeviceInfo->pDeviceDescriptor;

            //
            // The size of the device descriptor is in the first byte.
            //
            g_sUSBDeviceState.ulEP0DataRemain =
                g_psUSBDeviceInfo->pDeviceDescriptor[0];

            break;
        }

        //
        // This request was for a configuration descriptor.
        //
        case USB_DTYPE_CONFIGURATION:
        {
            const tConfigHeader *psConfig;
            const tDeviceDescriptor *psDevice;
            unsigned char ucIndex;

            //
            // Which configuration are we being asked for?
            //
            ucIndex = (unsigned char)(pUSBRequest->wValue & 0xFF);

            //
            // Is this valid?
            //
            psDevice = (const tDeviceDescriptor *)
                            (g_psUSBDeviceInfo->pDeviceDescriptor);
            if(ucIndex >= psDevice->bNumConfigurations)
            {
                //
                // This is an invalid configuration index.  Stall EP0 to
                // indicate a request error.
                //
                USBDCDStallEP0(0);
                g_sUSBDeviceState.pEP0Data = 0;
                g_sUSBDeviceState.ulEP0DataRemain = 0;
            }
            else
            {
                //
                // Return the externally specified configuration descriptor.
                //
                psConfig = g_psUSBDeviceInfo->ppConfigDescriptors[ucIndex];

                //
                // Start by sending data from the beginning of the first
                // descriptor.
                //
                g_sUSBDeviceState.ucConfigSection = 0;
                g_sUSBDeviceState.ucSectionOffset = 0;
                g_sUSBDeviceState.pEP0Data = (unsigned char *)
                                            psConfig->psSections[0]->pucData;

                //
                // Determine the total size of the configuration descriptor
                // by counting the sizes of the sections comprising it.
                //
                g_sUSBDeviceState.ulEP0DataRemain =
                                            USBDCDConfigDescGetSize(psConfig);

                //
                // Remember that we need to send the config descriptor and
                // which descriptor we need to send.
                //
                g_sUSBDeviceState.ucConfigIndex = ucIndex;
                bConfig = true;
            }
            break;
        }

        //
        // This request was for a string descriptor.
        //
        case USB_DTYPE_STRING:
        {
            long lIndex;

            //
            // Determine the correct descriptor index based on the requested
            // language ID and index.
            //
            lIndex = USBDStringIndexFromRequest(pUSBRequest->wIndex,
                                                pUSBRequest->wValue & 0xFF);

            //
            // If the mapping function returned -1 then stall the request to
            // indicate that the request was not valid.
            //
            if(lIndex == -1)
            {
                USBDCDStallEP0(0);
                break;
            }

            //
            // Return the externally specified configuration descriptor.
            //
            g_sUSBDeviceState.pEP0Data =
              (unsigned char *)g_psUSBDeviceInfo->ppStringDescriptors[lIndex];

            //
            // The total size of a string descriptor is in byte 0.
            //
            g_sUSBDeviceState.ulEP0DataRemain =
                g_psUSBDeviceInfo->ppStringDescriptors[lIndex][0];

            break;
        }

        //
        // Any other request is not handled by the default enumeration handler
        // so see if it needs to be passed on to another handler.
        //
        default:
        {
            //
            // If there is a handler for requests that are not handled then
            // call it.
            //
            if(g_psUSBDeviceInfo->sCallbacks.pfnGetDescriptor)
            {
                g_psUSBDeviceInfo->sCallbacks.pfnGetDescriptor(0, pUSBRequest);
                return;
            }
            else
            {
                //
                // Whatever this was this handler does not understand it so
                // just stall the request.
                //
                USBDCDStallEP0(0);
            }
            break;
        }
    }

    //
    // If this request has data to send, then send it.
    //
    if(g_sUSBDeviceState.pEP0Data)
    {
        //
        // If there is more data to send than is requested then just
        // send the requested amount of data.
        //
        if(g_sUSBDeviceState.ulEP0DataRemain > pUSBRequest->wLength)
        {
            g_sUSBDeviceState.ulEP0DataRemain = pUSBRequest->wLength;
        }

        //
        // Now in the transmit data state.  Be careful to call the correct
        // function since we need to handle the config descriptor differently
        // from the others.
        //
        if(!bConfig)
        {
            USBDEP0StateTx(0);
        }
        else
        {
            USBDEP0StateTxConfig(0);
        }
    }
}

//*****************************************************************************
//
// This function determines which string descriptor to send to satisfy a
// request for a given index and language.
//
// \param usLang is the requested string language ID.
// \param usIndex is the requested string descriptor index.
//
// When a string descriptor is requested, the host provides a language ID and
// index to identify the string ("give me string number 5 in French").  This
// function maps these two parameters to an index within our device's string
// descriptor array which is arranged as multiple groups of strings with
// one group for each language advertised via string descriptor 0.
//
// We assume that there are an equal number of strings per language and
// that the first descriptor is the language descriptor and use this fact to
// perform the mapping.
//
// \return The index of the string descriptor to return or -1 if the string
// could not be found.
//
//*****************************************************************************
static long
USBDStringIndexFromRequest(unsigned short usLang, unsigned short usIndex)
{
    tString0Descriptor *pLang;
    unsigned long ulNumLangs;
    unsigned long ulNumStringsPerLang;
    unsigned long ulLoop;

    //
    // Make sure we have a string table at all.
    //
    if(!g_psUSBDeviceInfo || !g_psUSBDeviceInfo->ppStringDescriptors)
    {
        return(-1);
    }

    //
    // First look for the trivial case where descriptor 0 is being
    // requested.  This is the special case since descriptor 0 contains the
    // language codes supported by the device.
    //
    if(usIndex == 0)
    {
        return(0);
    }

    //
    // How many languages does this device support?  This is determined by
    // looking at the length of the first descriptor in the string table,
    // subtracting 2 for the header and dividing by two (the size of each
    // language code).
    //
    ulNumLangs = (g_psUSBDeviceInfo->ppStringDescriptors[0][0] - 2) / 2;

    //
    // We assume that the table includes the same number of strings for each
    // supported language.  We know the number of entries in the string table,
    // so how many are there for each language?  This may seem an odd way to
    // do this (why not just have the application tell us in the device info
    // structure?) but it's needed since we didn't want to change the API
    // after the first release which did not support multiple languages.
    //
    ulNumStringsPerLang = ((g_psUSBDeviceInfo->ulNumStringDescriptors - 1) /
                           ulNumLangs);

    //
    // Just to be sure, make sure that the calculation indicates an equal
    // number of strings per language.  We expect the string table to contain
    // (1 + (strings_per_language * languages)) entries.
    //
    if((1 + (ulNumStringsPerLang * ulNumLangs)) !=
       g_psUSBDeviceInfo->ulNumStringDescriptors)
    {
        return(-1);
    }

    //
    // Now determine which language we are looking for.  It is assumed that
    // the order of the groups of strings per language in the table is the
    // same as the order of the language IDs listed in the first descriptor.
    //
    pLang = (tString0Descriptor *)(g_psUSBDeviceInfo->ppStringDescriptors[0]);

    //
    // Look through the supported languages looking for the one we were asked
    // for.
    //
    for(ulLoop = 0; ulLoop < ulNumLangs; ulLoop++)
    {
        //
        // Have we found the requested language?
        //
        if(pLang->wLANGID[ulLoop] == usLang)
        {
            //
            // Yes - calculate the index of the descriptor to send.
            //
            return((ulNumStringsPerLang * ulLoop) + usIndex);
        }
    }

    //
    // If we drop out of the loop, the requested language was not found so
    // return -1 to indicate the error.
    //
    return(-1);
}

//*****************************************************************************
//
// This function handles the SET_DESCRIPTOR standard USB request.
//
// \param ulIndex is the index of the USB controller which is to be
// initialized.
// \param pUSBRequest holds the data for this request.
//
// This function currently is not supported and will respond with a Stall
// to indicate that this command is not supported by the device.
//
// \return None.
//
//*****************************************************************************
static void
USBDSetDescriptor(unsigned long ulIndex, tUSBRequest *pUSBRequest)
{
    ASSERT(ulIndex == 0);

    //
    // This function is not handled by default.
    //
    USBDCDStallEP0(0);
}

//*****************************************************************************
//
// This function handles the GET_CONFIGURATION standard USB request.
//
// \param ulIndex is the index of the USB controller which is to be
// initialized.
// \param pUSBRequest holds the data for this request.
//
// This function responds to a host request to return the current
// configuration of the USB device.  The function will send the configuration
// response to the host and return.  This value will either be 0 or the last
// value received from a call to SetConfiguration().
//
// \return None.
//
//*****************************************************************************
static void
USBDGetConfiguration(unsigned long ulIndex, tUSBRequest *pUSBRequest)
{
    unsigned char ucValue;

    ASSERT(ulIndex == 0);

    //
    // If we still have an address pending then the device is still not
    // configured.
    //
    if(g_sUSBDeviceState.ulDevAddress & DEV_ADDR_PENDING)
    {
        ucValue = 0;
    }
    else
    {
        ucValue = (unsigned char)g_sUSBDeviceState.ulConfiguration;
    }

    g_sUSBDeviceState.ulEP0DataRemain = 1;
    g_sUSBDeviceState.pEP0Data = &ucValue;

    //
    // Send the single byte response.
    //
    USBDEP0StateTx(0);
}

//*****************************************************************************
//
// This function handles the SET_CONFIGURATION standard USB request.
//
// \param ulIndex is the index of the USB controller which is to be
// initialized.
// \param pUSBRequest holds the data for this request.
//
// This function responds to a host request to change the current
// configuration of the USB device.  The actual configuration number is taken
// from the structure passed in via \e pUSBRequest.  This number should be one
// of the configurations that was specified in the descriptors.  If the
// \e ConfigChange callback is specified in \e g_psUSBDeviceInfo->sCallbacks,
// it will be called so that the application can respond to a change in
// configuration.
//
// \return None.
//
//*****************************************************************************
static void
USBDSetConfiguration(unsigned long ulIndex, tUSBRequest *pUSBRequest)
{
    ASSERT(ulIndex == 0);

    //
    // Cannot set the configuration to one that does not exist so check the
    // enumeration structure to see how many valid configurations are present.
    //
    if(pUSBRequest->wValue > g_psUSBDeviceInfo->pDeviceDescriptor[17])
    {
        //
        // The passed configuration number is not valid.  Stall the endpoint to
        // signal the error to the host.
        //
        USBDCDStallEP0(0);
    }
    else
    {
        //
        // Need to ack the data on end point 0.
        //
        USBDevEndpointDataAck(USB0_BASE, USB_EP_0, true);

        //
        // Save the configuration.
        //
        g_sUSBDeviceState.ulConfiguration = pUSBRequest->wValue;

        //
        // If passed a configuration other than 0 (which tells us that we are
        // not currently configured), configure the endpoints (other than EP0)
        // appropriately.
        //
        if(g_sUSBDeviceState.ulConfiguration)
        {
            const tConfigHeader *psHdr;
            const tConfigDescriptor *psDesc;

            //
            // Get a pointer to the config descriptor.  This will always be the
            // first section in the current configuration.
            //
            psHdr =
               g_psUSBDeviceInfo->ppConfigDescriptors[pUSBRequest->wValue - 1];
            psDesc =
                (const tConfigDescriptor *)(psHdr->psSections[0]->pucData);

            //
            // Remember the new self- or bus-powered state if the user has not
            // already called us to tell us the state to report.
            //
            if(!g_sUSBDeviceState.bPwrSrcSet)
            {
                if((psDesc->bmAttributes & USB_CONF_ATTR_PWR_M) ==
                    USB_CONF_ATTR_SELF_PWR)
                {
                    g_sUSBDeviceState.ucStatus |= USB_STATUS_SELF_PWR;
                }
                else
                {
                    g_sUSBDeviceState.ucStatus &= ~USB_STATUS_SELF_PWR;
                }
            }

            //
            // Configure endpoints for the new configuration.
            //
            USBDeviceConfig(0,
               g_psUSBDeviceInfo->ppConfigDescriptors[pUSBRequest->wValue - 1],
               g_psUSBDeviceInfo->psFIFOConfig);
        }

        //
        // If there is a configuration change callback then call it.
        //
        if(g_psUSBDeviceInfo->sCallbacks.pfnConfigChange)
        {
            g_psUSBDeviceInfo->sCallbacks.pfnConfigChange(0,
                                            g_sUSBDeviceState.ulConfiguration);
        }
    }
}

//*****************************************************************************
//
// This function handles the GET_INTERFACE standard USB request.
//
// \param ulIndex is the index of the USB controller which is to be
// initialized.
// \param pUSBRequest holds the data for this request.
//
// This function is called when the host controller request the current
// interface that is in use by the device.  This simply returns the value set
// by the last call to SetInterface().
//
// \return None.
//
//*****************************************************************************
static void
USBDGetInterface(unsigned long ulIndex, tUSBRequest *pUSBRequest)
{
    unsigned char ucValue;

    ASSERT(ulIndex == 0);
    ASSERT(pUSBRequest);

    //
    // If we still have an address pending then the device is still not
    // configured.
    //
    if(g_sUSBDeviceState.ulDevAddress & DEV_ADDR_PENDING)
    {
        ucValue = (unsigned char)0;
    }
    else
    {
        //
        // Is the interface number valid?
        //
        if(pUSBRequest->wIndex < USB_MAX_INTERFACES_PER_DEVICE)
        {
            //
            // Read the current alternate setting for the required interface.
            //
            ucValue = g_sUSBDeviceState.pucAltSetting[pUSBRequest->wIndex];
        }
        else
        {
            //
            // An invalid interface number was specified.
            //
            USBDCDStallEP0(0);
            return;
        }
    }

    //
    // Send the single byte response.
    //
    g_sUSBDeviceState.ulEP0DataRemain = 1;
    g_sUSBDeviceState.pEP0Data = &ucValue;

    //
    // Send the single byte response.
    //
    USBDEP0StateTx(0);
}

//*****************************************************************************
//
// This function handles the SET_INTERFACE standard USB request.
//
// \param ulIndex is the index of the USB controller which is to be
// initialized.
// \param pUSBRequest holds the data for this request.
//
// This function is called when a standard request for changing the interface
// is received from the host controller.  If this is a valid request the
// function will call the function specified by the InterfaceChange in the
// \e g_psUSBDeviceInfo->sCallbacks variable to notify the application that the
// interface has changed and will pass it the new alternate interface number.
//
// \return None.
//
//*****************************************************************************
static void
USBDSetInterface(unsigned long ulIndex, tUSBRequest *pUSBRequest)
{
    const tConfigHeader *psConfig;
    tInterfaceDescriptor *psInterface;
    unsigned long ulLoop;
    unsigned long ulSection;
    unsigned long ulNumInterfaces;
    unsigned char ucInterface;
    tBoolean bRetcode;

    ASSERT(ulIndex == 0);
    ASSERT(pUSBRequest);

    //
    // Use the current configuration.
    //
    psConfig = g_psUSBDeviceInfo->ppConfigDescriptors[
                    (g_sUSBDeviceState.ulConfiguration - 1)];

    //
    // How many interfaces are included in the descriptor?
    //
    ulNumInterfaces = USBDCDConfigDescGetNum(psConfig,
                                             USB_DTYPE_INTERFACE);

    //
    // Find the interface descriptor for the supplied interface and alternate
    // setting numbers.
    //
    for(ulLoop = 0; ulLoop < ulNumInterfaces; ulLoop++)
    {
        //
        // Get the next interface descriptor in the config descriptor.
        //
        psInterface = USBDCDConfigGetInterface(psConfig, ulLoop, USB_DESC_ANY,
                                               &ulSection);

        //
        // Is this the required interface with the correct alternate setting?
        //
        if(psInterface &&
           (psInterface->bInterfaceNumber == pUSBRequest->wIndex) &&
           (psInterface->bAlternateSetting == pUSBRequest->wValue))
        {
            ucInterface = psInterface->bInterfaceNumber;
            //
            // Make sure we don't write outside the bounds of the pucAltSetting
            // array (in a debug build, anyway, since this indicates an error
            // in the device descriptor).
            //
            ASSERT(ucInterface < USB_MAX_INTERFACES_PER_DEVICE);

            //
            // Need to ack the data on end point 0.
            //
            USBDevEndpointDataAck(USB0_BASE, USB_EP_0, true);

            //
            // If anything changed, reconfigure the endpoints for the new
            // alternate setting.
            //
            if(g_sUSBDeviceState.pucAltSetting[ucInterface] !=
                   psInterface->bAlternateSetting)
            {
                //
                // This is the correct interface descriptor so save the
                // setting.
                //
                g_sUSBDeviceState.pucAltSetting[ucInterface] =
                       psInterface->bAlternateSetting;

                //
                // Reconfigure the endpoints to match the requirements of the
                // new alternate setting for the interface.
                //
                bRetcode = USBDeviceConfigAlternate(0, psConfig, ucInterface,
                                               psInterface->bAlternateSetting);

                //
                // If there is a callback then notify the application of the
                // change to the alternate interface.
                //
                if(bRetcode &&
                   g_psUSBDeviceInfo->sCallbacks.pfnInterfaceChange)
                {
                    g_psUSBDeviceInfo->sCallbacks.pfnInterfaceChange(0,
                        pUSBRequest->wIndex, pUSBRequest->wValue);
                }
            }

            //
            // All done.
            //
            return;
        }
    }

    //
    // If we drop out of the loop, we didn't find an interface descriptor
    // matching the requested number and alternate setting or there was an
    // error while trying to set up for the new alternate setting.
    //
    USBDCDStallEP0(0);
}

//*****************************************************************************
//
// This function handles the SYNC_FRAME standard USB request.
//
// \param ulIndex is the index of the USB controller which is to be
// initialized.
// \param pUSBRequest holds the data for this request.
//
// This is currently a stub function that will stall indicating that the
// command is not supported.
//
// \return None.
//
//*****************************************************************************
static void
USBDSyncFrame(unsigned long ulIndex, tUSBRequest *pUSBRequest)
{
    ASSERT(ulIndex == 0);

    //
    // Not handled yet so stall this request.
    //
    USBDCDStallEP0(0);
}

//*****************************************************************************
//
// This internal function handles sending data on endpoint zero.
//
// \param ulIndex is the index of the USB controller which is to be
// initialized.
//
// \return None.
//
//*****************************************************************************
static void
USBDEP0StateTx(unsigned long ulIndex)
{
    unsigned long ulNumBytes;
    unsigned char *pData;

    ASSERT(ulIndex == 0);

    //
    // In the TX state on endpoint zero.
    //
    g_eUSBDEP0State = USB_STATE_TX;

    //
    // Set the number of bytes to send this iteration.
    //
    ulNumBytes = g_sUSBDeviceState.ulEP0DataRemain;

    //
    // Limit individual transfers to 64 bytes.
    //
    if(ulNumBytes > EP0_MAX_PACKET_SIZE)
    {
        ulNumBytes = EP0_MAX_PACKET_SIZE;
    }

    //
    // Save the pointer so that it can be passed to the USBEndpointDataPut()
    // function.
    //
    pData = (unsigned char *)g_sUSBDeviceState.pEP0Data;

    //
    // Advance the data pointer and counter to the next data to be sent.
    //
    g_sUSBDeviceState.ulEP0DataRemain -= ulNumBytes;
    g_sUSBDeviceState.pEP0Data += ulNumBytes;

    //
    // Put the data in the correct FIFO.
    //
    USBEndpointDataPut(USB0_BASE, USB_EP_0, pData, ulNumBytes);

    //
    // If this is exactly 64 then don't set the last packet yet.
    //
    if(ulNumBytes == EP0_MAX_PACKET_SIZE)
    {
        //
        // There is more data to send or exactly 64 bytes were sent, this
        // means that there is either more data coming or a null packet needs
        // to be sent to complete the transaction.
        //
        USBEndpointDataSend(USB0_BASE, USB_EP_0, USB_TRANS_IN);
    }
    else
    {
        //
        // Now go to the status state and wait for the transmit to complete.
        //
        g_eUSBDEP0State = USB_STATE_STATUS;

        //
        // Send the last bit of data.
        //
        USBEndpointDataSend(USB0_BASE, USB_EP_0, USB_TRANS_IN_LAST);

        //
        // If there is a sent callback then call it.
        //
        if((g_psUSBDeviceInfo->sCallbacks.pfnDataSent) &&
           (g_sUSBDeviceState.ulOUTDataSize != 0))
        {
            //
            // Call the custom handler.
            //
            g_psUSBDeviceInfo->sCallbacks.pfnDataSent(0,
                                              g_sUSBDeviceState.ulOUTDataSize);

            //
            // There is no longer any data pending to be sent.
            //
            g_sUSBDeviceState.ulOUTDataSize = 0;
        }
    }
}

//*****************************************************************************
//
// This internal function handles sending the config descriptor on endpoint
// zero.
//
// \param ulIndex is the index of the USB controller which is to be used.
//
//
// \return None.
//
//*****************************************************************************
static void
USBDEP0StateTxConfig(unsigned long ulIndex)
{
    unsigned long ulNumBytes;
    unsigned long ulSecBytes;
    unsigned long ulToSend;
    unsigned char *pData;
    tConfigDescriptor sConfDesc;
    const tConfigHeader *psConfig;
    const tConfigSection *psSection;

    ASSERT(ulIndex == 0);

    //
    // In the TX state on endpoint zero.
    //
    g_eUSBDEP0State = USB_STATE_TX_CONFIG;

    //
    // Find the current config descriptor definition.
    //
    psConfig = g_psUSBDeviceInfo->ppConfigDescriptors[
                                              g_sUSBDeviceState.ucConfigIndex];

    //
    // Set the number of bytes to send this iteration.
    //
    ulNumBytes = g_sUSBDeviceState.ulEP0DataRemain;

    //
    // Limit individual transfers to 64 bytes.
    //
    if(ulNumBytes > EP0_MAX_PACKET_SIZE)
    {
        ulNumBytes = EP0_MAX_PACKET_SIZE;
    }

    //
    // If this is the first call, we need to fix up the total length of the
    // config descriptor.  This has already been determined and set in
    // g_sUSBDeviceState.ulEP0DataRemain.
    //
    if((g_sUSBDeviceState.ucSectionOffset == 0) &&
       (g_sUSBDeviceState.ucConfigSection == 0))
    {
        //
        // Copy the USB config descriptor from the beginning of the first
        // section of the current config.
        //
        sConfDesc = *(tConfigDescriptor *)g_sUSBDeviceState.pEP0Data;

        //
        // Update the total size.
        //
        sConfDesc.wTotalLength = (unsigned short)USBDCDConfigDescGetSize(
                                                                   psConfig);

        //
        // Write the descriptor to the USB FIFO.
        //
        ulToSend = (ulNumBytes < sizeof(tConfigDescriptor)) ? ulNumBytes :
                        sizeof(tConfigDescriptor);
        USBEndpointDataPut(USB0_BASE, USB_EP_0, (unsigned char *)&sConfDesc,
                           ulToSend);

        //
        // Did we reach the end of the first section?
        //
        if(psConfig->psSections[0]->ucSize == ulToSend)
        {
            //
            // Update our tracking indices to point to the start of the next
            // section.
            //
            g_sUSBDeviceState.ucSectionOffset = 0;
            g_sUSBDeviceState.ucConfigSection = 1;
        }
        else
        {
            //
            // Note that we have sent the first few bytes of the descriptor.
            //
            g_sUSBDeviceState.ucSectionOffset = (unsigned char)ulToSend;
        }

        //
        // How many bytes do we have remaining to send on this iteration?
        //
        ulToSend = ulNumBytes - ulToSend;
    }
    else
    {
        //
        // Set the number of bytes we still have to send on this call.
        //
        ulToSend = ulNumBytes;
    }

    //
    // Add the relevant number of bytes to the USB FIFO
    //
    while(ulToSend)
    {
        //
        // Get a pointer to the current config section.
        //
        psSection = psConfig->psSections[g_sUSBDeviceState.ucConfigSection];

        //
        // How many bytes are available in the current config section?
        //
        ulSecBytes = (unsigned long)(psSection->ucSize -
                                     g_sUSBDeviceState.ucSectionOffset);

        //
        // Save the pointer so that it can be passed to the
        // USBEndpointDataPut() function.
        //
        pData = (unsigned char *)psSection->pucData +
                g_sUSBDeviceState.ucSectionOffset;

        //
        // Are there more bytes in this section that we still have to send?
        //
        if(ulSecBytes > ulToSend)
        {
            //
            // Yes - send only the remaining bytes in the transfer.
            //
            ulSecBytes = ulToSend;
        }

        //
        // Put the data in the correct FIFO.
        //
        USBEndpointDataPut(USB0_BASE, USB_EP_0, pData, ulSecBytes);

        //
        // Fix up our pointers for the next iteration.
        //
        ulToSend -= ulSecBytes;
        g_sUSBDeviceState.ucSectionOffset += (unsigned char)ulSecBytes;

        //
        // Have we reached the end of a section?
        //
        if(g_sUSBDeviceState.ucSectionOffset == psSection->ucSize)
        {
            //
            // Yes - move to the next one.
            //
            g_sUSBDeviceState.ucConfigSection++;
            g_sUSBDeviceState.ucSectionOffset = 0;
        }
    }

    //
    // Fix up the number of bytes remaining to be sent and the start pointer.
    //
    g_sUSBDeviceState.ulEP0DataRemain -= ulNumBytes;

    //
    // If we ran out of bytes in the configuration section, bail and just
    // send out what we have.
    //
    if(psConfig->ucNumSections <= g_sUSBDeviceState.ucConfigSection)
    {
        g_sUSBDeviceState.ulEP0DataRemain = 0;
    }

    //
    // If there is no more data don't keep looking or ucConfigSection might
    // overrun the available space.
    //
    if(g_sUSBDeviceState.ulEP0DataRemain != 0)
    {
        pData =(unsigned char *)
            psConfig->psSections[g_sUSBDeviceState.ucConfigSection]->pucData;
        ulToSend = g_sUSBDeviceState.ucSectionOffset;
        g_sUSBDeviceState.pEP0Data = (pData + ulToSend);
    }

    //
    // If this is exactly 64 then don't set the last packet yet.
    //
    if(ulNumBytes == EP0_MAX_PACKET_SIZE)
    {
        //
        // There is more data to send or exactly 64 bytes were sent, this
        // means that there is either more data coming or a null packet needs
        // to be sent to complete the transaction.
        //
        USBEndpointDataSend(USB0_BASE, USB_EP_0, USB_TRANS_IN);
    }
    else
    {
        //
        // Send the last bit of data.
        //
        USBEndpointDataSend(USB0_BASE, USB_EP_0, USB_TRANS_IN_LAST);

        //
        // If there is a sent callback then call it.
        //
        if((g_psUSBDeviceInfo->sCallbacks.pfnDataSent) &&
           (g_sUSBDeviceState.ulOUTDataSize != 0))
        {
            //
            // Call the custom handler.
            //
            g_psUSBDeviceInfo->sCallbacks.pfnDataSent(0,
                                              g_sUSBDeviceState.ulOUTDataSize);

            //
            // There is no longer any data pending to be sent.
            //
            g_sUSBDeviceState.ulOUTDataSize = 0;
        }

        //
        // Now go to the status state and wait for the transmit to complete.
        //
        g_eUSBDEP0State = USB_STATE_STATUS;
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
