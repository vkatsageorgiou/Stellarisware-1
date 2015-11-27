//*****************************************************************************
//
// usbhostenum.c - Device enumeration code for the USB host library.
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

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/udma.h"
#include "driverlib/usb.h"
#include "usblib/usblib.h"
#include "usblib/usblibpriv.h"
#include "usblib/host/usbhost.h"

//*****************************************************************************
//
//! \addtogroup usblib_hcd
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// External prototypes.
//
//*****************************************************************************
extern tUSBMode g_eUSBMode;

extern void OTGDeviceDisconnect(unsigned long ulIndex);

//*****************************************************************************
//
// Internal function prototypes.
//
//*****************************************************************************
static void USBHCDEP0StateTx(void);
static void USBHCDEnumHandler(void);
static void USBHCDClearFeature(unsigned long ulDevAddress,
                               unsigned long ulEndpoint,
                               unsigned long ulFeature);

//*****************************************************************************
//
// Flags used to signal between the interrupt handler and USBHCDMain().
//
//*****************************************************************************
#define INT_EVENT_VBUS_ERR      0x01
#define INT_EVENT_CONNECT       0x02
#define INT_EVENT_DISCONNECT    0x04
#define INT_EVENT_POWER_FAULT   0x08

volatile unsigned long g_ulUSBHIntEvents;

//*****************************************************************************
//
// Flags used to indicate that a uDMA transfer is pending on a pipe.
//
//*****************************************************************************
#define DMA_PEND_TRANSMIT_FLAG  0x10000
#define DMA_PEND_RECEIVE_FLAG   0x1

volatile unsigned long g_ulDMAPending = 0;

//*****************************************************************************
//
// Flag used to indicate that a workaround should be applied when using
// uDMA with USB.  The uDMA transfers must match the USB FIFO size when
// with Rev A0 silicon.
//
//*****************************************************************************
static unsigned int g_bUseDMAWA = 0;

//*****************************************************************************
//
// This holds the current power configuration that is used when USBHCDInit()
// is called.
//
//*****************************************************************************
static unsigned long g_ulPowerConfig = USB_HOST_PWREN_HIGH;

//*****************************************************************************
//
// The states for endpoint 0 during enumeration.
//
//*****************************************************************************
typedef enum
{
    //
    // The USB device is waiting on a request from the host controller on
    // endpoint 0.
    //
    EP0_STATE_IDLE,

    //
    // Setup packet is expecting data IN.
    //
    EP0_STATE_SETUP_IN,

    //
    // Setup packet is sending data OUT.
    //
    EP0_STATE_SETUP_OUT,

    //
    // The USB device is receiving data from the device due to an SETUP IN
    // request.
    //
    EP0_STATE_RX,

    //
    // The USB device has completed the IN or OUT request and is now waiting
    // for the host to acknowledge the end of the IN/OUT transaction.  This
    // is the status phase for a USB control transaction.
    //
    EP0_STATE_STATUS,

    //
    // This state is for when a response only has a status phase and no
    // data phase.
    //
    EP0_STATE_STATUS_IN,

    //
    // This endpoint has signaled a stall condition and is waiting for the
    // stall to be acknowledged by the host controller.
    //
    EP0_STATE_STALL
}
tEP0State;

//*****************************************************************************
//
// This structure holds the full state for the device enumeration.
//
//*****************************************************************************
typedef struct
{
    //
    // This is the pointer to the current data being sent out or received
    // on endpoint 0.
    //
    unsigned char *pData;

    //
    // This is the number of bytes that remain to be sent from or received
    // into the g_DeviceState.pEP0Data data buffer.
    //
    volatile unsigned long ulBytesRemaining;

    //
    // The amount of data being sent/received due to a request.
    //
    unsigned long ulDataSize;

    //
    // This is the current device address in use by endpoint 0.
    //
    unsigned long ulDevAddress;

    //
    // The maximum packet size for the device responding to the setup packet.
    //
    unsigned long ulMaxPacketSize;

    //
    // The host controller's state.
    //
    tEP0State eState;
}
tHostState;

//*****************************************************************************
//
// This variable holds the current state of endpoint 0.
//
//*****************************************************************************
static volatile tHostState g_sUSBHEP0State =
{
    0,                          // pData
    0,                          // ulBytesRemaining
    0,                          // ulDataSize
    0,                          // ulDevAddress
    0,                          // ulMaxPacketSize
    EP0_STATE_IDLE              // eState
};

//*****************************************************************************
//
// The global delay time for use by SysCtlDelay() function.  This is
// initialized to an appropriate value for a 50MHz clock.  The correct value
// will be set in USBHCDInit().
//
//*****************************************************************************
static unsigned long g_ulTickms = (50000000 / 3000);
static volatile unsigned long g_ulCurrentTick = 0;

//*****************************************************************************
//
// The current active driver.
//
//*****************************************************************************
static int g_iUSBHActiveDriver = -1;
static void *g_pvDriverInstance = 0;

//*****************************************************************************
//
// This is the structure used to hold the information for a given USB pipe
// that is attached to a device.
//
//*****************************************************************************
typedef struct
{
    //
    // The current address for this pipe.
    //
    unsigned long ulDevAddr;

    //
    // The current address for this pipe.
    //
    unsigned char ucEPNumber;

    //
    // The DMA channel assigned to this endpoint.
    //
    unsigned char ucDMAChannel;

    //
    // The current type for this pipe.
    //
    unsigned long ulType;

    //
    // The millisecond interval for this pipe.
    //
    unsigned long ulInterval;

    //
    // The next tick value to trigger and event on this pipe.
    //
    unsigned long ulNextEventTick;

    //
    // The current callback for this pipe.
    //
    tHCDPipeCallback pfnCallback;

    //
    // The state of a given USB pipe.
    //
    volatile enum
    {
        PIPE_READING,
        PIPE_DATA_READY,
        PIPE_DATA_SENT,
        PIPE_WRITING,
        PIPE_STALLED,
        PIPE_ERROR,
        PIPE_IDLE,
        PIPE_DISABLED
    }
    eState;
}
tUSBHCDPipe;

//*****************************************************************************
//
// The internal state of the device.
//
//*****************************************************************************
typedef enum
{
    HCD_DEV_DISCONNECTED,
    HCD_DEV_CONNECTED,
    HCD_DEV_REQUEST,
    HCD_DEV_RESET,
    HCD_DEV_ADDRESSED,
    HCD_DEV_CONFIGURED,
    HCD_DEV_GETSTRINGS,
    HCD_VBUS_ERROR,
    HCD_POWER_FAULT,
    HCD_IDLE
}
tUSBHDeviceState;

//*****************************************************************************
//
// This is a fixed number as it relates to the maximum number of USB pipes
// available on any USB controller.  The actual number on a given device may
// be less than this number.
//
//*****************************************************************************
#define MAX_NUM_PIPES           15

//*****************************************************************************
//
// This is a fixed number as it relates to the number of USB pipes available
// in the USB controller.
//
//*****************************************************************************
#define MAX_NUM_DMA_CHANNELS    6

//*****************************************************************************
//
// Marker for an unused DMA channel slot.
//
//*****************************************************************************
#define USBHCD_DMA_UNUSED       0xff

//*****************************************************************************
//
// These definitions are used to manipulate the values returned as allocated
// USB pipes.
//
//*****************************************************************************
#define EP_PIPE_USE_UDMA        0x01000000
#define EP_PIPE_TYPE_ISOC       0x00800000
#define EP_PIPE_TYPE_INTR       0x00400000
#define EP_PIPE_TYPE_BULK       0x00200000
#define EP_PIPE_TYPE_CONTROL    0x00100000
#define EP_PIPE_TYPE_IN         0x00020000
#define EP_PIPE_TYPE_OUT        0x00010000
#define EP_PIPE_IDX_M           0x0000ffff

//*****************************************************************************
//
// This creates a USB pipe handle from an index.
//
//*****************************************************************************
#define OUT_PIPE_HANDLE(ulIdx)  (g_sUSBHCD.USBOUTPipes[ulIdx].ulType | ulIdx)
#define IN_PIPE_HANDLE(ulIdx)   (g_sUSBHCD.USBINPipes[ulIdx].ulType | ulIdx)

//*****************************************************************************
//
// Converts from an endpoint specifier to the offset of the endpoint's
// control/status registers.
//
//*****************************************************************************
#define EP_OFFSET(Endpoint)     (Endpoint - 0x10)

//*****************************************************************************
//
// This structure holds the state information for a given host controller.
//
//*****************************************************************************
typedef struct
{
    unsigned long ulUSBBase;

    tUSBHCDPipe USBControlPipe;
    tUSBHCDPipe USBOUTPipes[MAX_NUM_PIPES];
    tUSBHCDPipe USBINPipes[MAX_NUM_PIPES];
    unsigned char ucDMAChannels[MAX_NUM_DMA_CHANNELS];

    //
    // Each devices state.
    //
    tUSBHostDevice USBDevice[1];

    //
    // Holds the current state of the device.
    //
    volatile tUSBHDeviceState eDeviceState[1];

    //
    // Pointer to the memory pool for this controller.
    //
    void *pvPool;

    //
    // The pool size for this controller.
    //
    unsigned long ulPoolSize;

    //
    // The class drivers for this controller.
    //
    const tUSBHostClassDriver * const *pClassDrivers;

    //
    // The number of class drivers.
    //
    unsigned long ulNumClassDrivers;

    //
    // This is the index in the driver list of the event driver.
    //
    int iEventDriver;

    //
    // This is the generic event information used by the event driver.
    //
    tEventInfo EventInfo;

    unsigned long ulClass;
}
tUSBHCD;

//*****************************************************************************
//
// The global to hold all of the state information for a given host controller.
//
//*****************************************************************************
static tUSBHCD g_sUSBHCD;

//*****************************************************************************
//
//! This function is used to allocate a USB HCD pipe.
//!
//! \param ulIndex specifies which USB controller to use.
//! \param ulEndpointType is the type of endpoint that this pipe will be
//! communicating with.
//! \param ulDevAddr is the device address to use for this endpoint.
//! \param pfnCallback is the function that will be called when events occur on
//! this USB Pipe.
//!
//! Since there are a limited number of USB HCD pipes that can be used in the
//! host controller, this function is used to temporarily or permanently
//! acquire one of the endpoints.  It also provides a method to register a
//! callback for status changes on this endpoint.  If no callbacks are desired
//! then the \e pfnCallback function should be set to 0.
//!
//! \return This function returns a value indicating which pipe was reserved.
//! If the value is 0 then there were no pipes currently available.  This value
//! should be passed to any USBHCDPipe APIs to indicate which pipe is being
//! accessed.
//
//*****************************************************************************
unsigned long
USBHCDPipeAlloc(unsigned long ulIndex, unsigned long ulEndpointType,
                unsigned long ulDevAddr, tHCDPipeCallback pfnCallback)
{
    int iIdx;
    int iDMAIdx;

    ASSERT(ulIndex == 0);

    //
    // Find a USB pipe that is free.
    //
    for(iIdx = 0; iIdx < MAX_NUM_PIPES; iIdx++)
    {
        //
        // Handle OUT Pipes.
        //
        if(ulEndpointType & EP_PIPE_TYPE_OUT)
        {
            //
            // A zero address indicates free.
            //
            if(g_sUSBHCD.USBOUTPipes[iIdx].ulDevAddr == 0)
            {
                //
                // Set up uDMA for the pipe.
                //
                if(ulEndpointType & EP_PIPE_USE_UDMA)
                {
                    //
                    // First three endpoints have fixed channels on some
                    // parts so bias the pipes to match this as best is
                    // possible.
                    //
                    if(iIdx < 3)
                    {
                        //
                        // Check if the fixed channel is available for this
                        // USB pipe.
                        //
                        if((g_sUSBHCD.ucDMAChannels[1 + (iIdx * 2)] ==
                                      USBHCD_DMA_UNUSED))
                        {
                            //
                            // The default channel was available so use it.
                            //
                            g_sUSBHCD.ucDMAChannels[1 + (iIdx * 2)] = iIdx;
                            g_sUSBHCD.USBOUTPipes[iIdx].ucDMAChannel =
                                1 + (iIdx * 2);
                        }
                        else
                        {
                            //
                            // Go to the next USB pipe if the fixed one was not
                            // available.
                            //
                            continue;
                        }
                    }
                    else
                    {
                        //
                        // Either the fixed channel was not available or the
                        // pipe index was more than the first 3 pipes that are
                        // available on all parts.
                        //
                        for(iDMAIdx = 1; iDMAIdx < MAX_NUM_DMA_CHANNELS;
                            iDMAIdx += 2)
                        {
                            //
                            // Find any available channel.
                            //
                            if(g_sUSBHCD.ucDMAChannels[iDMAIdx] ==
                               USBHCD_DMA_UNUSED)
                            {
                                //
                                // Save the index and the DMA channel
                                // information.
                                //
                                g_sUSBHCD.ucDMAChannels[iDMAIdx] = iIdx;
                                g_sUSBHCD.USBOUTPipes[iIdx].ucDMAChannel =
                                    iDMAIdx;
                            }
                        }
                    }

                    //
                    // If no DMA channel was available then just disable DMA
                    // on this pipe.
                    //
                    if(g_sUSBHCD.USBOUTPipes[iIdx].ucDMAChannel ==
                       USBHCD_DMA_UNUSED)
                    {
                        ulEndpointType &= ~EP_PIPE_USE_UDMA;
                    }
                    else
                    {
                        //
                        // Set the DMA channel for this endpoint, this has no
                        // effect on parts without configurable DMA.
                        //
                        USBEndpointDMAChannel(
                            USB0_BASE, INDEX_TO_USB_EP(iIdx + 1),
                            g_sUSBHCD.USBOUTPipes[iIdx].ucDMAChannel);

                        //
                        // Clear all the attributes for the channel
                        //
                        uDMAChannelAttributeDisable(
                            g_sUSBHCD.USBOUTPipes[iIdx].ucDMAChannel,
                            UDMA_ATTR_ALL);

                        //
                        // Configure the uDMA channel for the pipe
                        //
                        uDMAChannelControlSet(
                            g_sUSBHCD.USBOUTPipes[iIdx].ucDMAChannel,
                            (UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
                             UDMA_ARB_64));
                    }
                }

                //
                // Save the endpoint type and device address and callback
                // function.
                //
                g_sUSBHCD.USBOUTPipes[iIdx].ulType = ulEndpointType;
                g_sUSBHCD.USBOUTPipes[iIdx].ulDevAddr = ulDevAddr;
                g_sUSBHCD.USBOUTPipes[iIdx].pfnCallback = pfnCallback;

                //
                // Initialize the endpoint as idle.
                //
                g_sUSBHCD.USBOUTPipes[iIdx].eState = PIPE_IDLE;

                //
                // This is the fixed allocation FIFO.
                //
                USBFIFOConfigSet(USB0_BASE, INDEX_TO_USB_EP(iIdx + 1),
                                 64 * ((iIdx * 2) + 1), USB_FIFO_SZ_64,
                                 USB_EP_HOST_OUT);

                //
                // Set the function address for this endpoint.
                //
                USBHostAddrSet(USB0_BASE, INDEX_TO_USB_EP(iIdx + 1), ulDevAddr,
                               USB_EP_HOST_OUT);

                break;
            }
        }
        //
        // Handle IN Pipes.
        //
        else if(ulEndpointType & EP_PIPE_TYPE_IN)
        {
            //
            // A zero address indicates free.
            //
            if(g_sUSBHCD.USBINPipes[iIdx].ulDevAddr == 0)
            {
                //
                // Set up uDMA for the pipe.
                //
                if(ulEndpointType & EP_PIPE_USE_UDMA)
                {
                    //
                    // First three endpoints have fixed channels on some
                    // parts so bias the pipes to match this as best is
                    // possible.
                    //
                    if(iIdx < 3)
                    {
                        //
                        // Check if the fixed channel is available for this
                        // USB pipe.
                        //
                        if(g_sUSBHCD.ucDMAChannels[iIdx * 2] ==
                           USBHCD_DMA_UNUSED)
                        {
                            //
                            // The default channel was available so use it.
                            //
                            g_sUSBHCD.ucDMAChannels[iIdx * 2] = iIdx;
                            g_sUSBHCD.USBINPipes[iIdx].ucDMAChannel = iIdx * 2;
                        }
                        else
                        {
                            //
                            // Go to the next USB pipe if the fixed one was not
                            // available.
                            //
                            continue;
                        }
                    }
                    else
                    {
                        //
                        // Either the fixed channel was not available or the
                        // pipe index was more than the first 3 pipes that are
                        // available on all parts.
                        //
                        for(iDMAIdx = 0; iDMAIdx < MAX_NUM_DMA_CHANNELS;
                            iDMAIdx += 2)
                        {
                            //
                            // Find any available channel.
                            //
                            if(g_sUSBHCD.ucDMAChannels[iDMAIdx] ==
                               USBHCD_DMA_UNUSED)
                            {
                                //
                                // Save the index and the DMA channel
                                // information.
                                //
                                g_sUSBHCD.ucDMAChannels[iDMAIdx] = iIdx;
                                g_sUSBHCD.USBINPipes[iIdx].ucDMAChannel =
                                    iDMAIdx;
                            }
                        }
                    }

                    //
                    // If no DMA channel was available then just disable DMA
                    // on this pipe.
                    //
                    if(g_sUSBHCD.USBINPipes[iIdx].ucDMAChannel ==
                       USBHCD_DMA_UNUSED)
                    {
                        ulEndpointType &= ~EP_PIPE_USE_UDMA;
                    }
                    else
                    {
                        //
                        // Set the DMA channel for this endpoint, this has no
                        // effect on parts without configurable DMA.
                        //
                        //
                        USBEndpointDMAChannel(
                            USB0_BASE, INDEX_TO_USB_EP(iIdx + 1),
                            g_sUSBHCD.USBINPipes[iIdx].ucDMAChannel);

                        //
                        // Clear all the attributes for the channel
                        //
                        uDMAChannelAttributeDisable(
                            g_sUSBHCD.USBINPipes[iIdx].ucDMAChannel,
                            UDMA_ATTR_ALL);

                        //
                        // Configure the uDMA channel for the pipe
                        //
                        uDMAChannelControlSet(
                            g_sUSBHCD.USBINPipes[iIdx].ucDMAChannel,
                            (UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
                             UDMA_ARB_64));
                    }
                }

                //
                // Save the endpoint type and device address and callback
                // function.
                //
                g_sUSBHCD.USBINPipes[iIdx].ulType = ulEndpointType;
                g_sUSBHCD.USBINPipes[iIdx].ulDevAddr = ulDevAddr;
                g_sUSBHCD.USBINPipes[iIdx].pfnCallback = pfnCallback;

                //
                // This is the fixed allocation FIFO, this needs to be better.
                //
                USBFIFOConfigSet(USB0_BASE, INDEX_TO_USB_EP(iIdx + 1),
                                 64 * ((iIdx + 1)* 2), USB_FIFO_SZ_64,
                                 USB_EP_HOST_IN);

                //
                // Set the function address for this endpoint.
                //
                USBHostAddrSet(USB0_BASE, INDEX_TO_USB_EP(iIdx + 1), ulDevAddr,
                               USB_EP_HOST_IN);

                //
                // Reset the state of the pipe to idle.
                //
                g_sUSBHCD.USBINPipes[iIdx].eState = PIPE_IDLE;

                break;
            }
        }
    }

    //
    // Did not find a free pipe.
    //
    if(iIdx == MAX_NUM_PIPES)
    {
        return(0);
    }

    //
    // Return the pipe index and type that was allocated.
    //
    return(ulEndpointType | iIdx);
}

//*****************************************************************************
//
//! This function is used to configure a USB HCD pipe.
//!
//! This should be called after allocating a USB pipe with a call to
//! USBHCDPipeAlloc().  It is used to set the configuration associated with an
//! endpoint like the max payload and target endpoint.
//!
//! \param ulPipe is the allocated endpoint to modify.
//! \param ulMaxPayload is maximum data that can be handled per transaction.
//! \param ulInterval is the polling interval for data transfers expressed in
//! frames.
//! \param ulTargetEndpoint is the target endpoint on the device to communicate
//! with.
//!
//! \return If the call was successful, this function returns zero any other
//! value indicates an error.
//
//*****************************************************************************
unsigned long
USBHCDPipeConfig(unsigned long ulPipe, unsigned long ulMaxPayload,
                 unsigned long ulInterval, unsigned long ulTargetEndpoint)
{
    unsigned long ulFlags;
    unsigned long ulIndex;

    //
    // Get the index number from the allocated pipe.
    //
    ulIndex = (ulPipe & EP_PIPE_IDX_M);

    //
    // Set the direction.
    //
    if(ulPipe & EP_PIPE_TYPE_OUT)
    {
        //
        // Set the mode for this endpoint.
        //
        if(g_sUSBHCD.USBOUTPipes[ulIndex].ulType & EP_PIPE_TYPE_BULK)
        {
            ulFlags = USB_EP_MODE_BULK;
        }
        else if(g_sUSBHCD.USBOUTPipes[ulIndex].ulType & EP_PIPE_TYPE_INTR)
        {
            ulFlags = USB_EP_MODE_INT;
        }
        else if(g_sUSBHCD.USBOUTPipes[ulIndex].ulType & EP_PIPE_TYPE_ISOC)
        {
            ulFlags = USB_EP_MODE_ISOC;
        }
        else
        {
            ulFlags = USB_EP_MODE_CTRL;
        }

        ulFlags |= USB_EP_HOST_OUT;

        g_sUSBHCD.USBOUTPipes[ulIndex].ucEPNumber =
            (unsigned char)ulTargetEndpoint;

        //
        // Save the interval and the next tick to trigger a scheduler event.
        //
        g_sUSBHCD.USBOUTPipes[ulIndex].ulInterval = ulInterval;
        g_sUSBHCD.USBOUTPipes[ulIndex].ulNextEventTick =
            ulInterval + g_ulCurrentTick;
    }
    else
    {
        //
        // Set the mode for this endpoint.
        //
        if(g_sUSBHCD.USBINPipes[ulIndex].ulType & EP_PIPE_TYPE_BULK)
        {
            ulFlags = USB_EP_MODE_BULK;
        }
        else if(g_sUSBHCD.USBINPipes[ulIndex].ulType & EP_PIPE_TYPE_INTR)
        {
            ulFlags = USB_EP_MODE_INT;
        }
        else if(g_sUSBHCD.USBINPipes[ulIndex].ulType & EP_PIPE_TYPE_ISOC)
        {
            ulFlags = USB_EP_MODE_ISOC;
        }
        else
        {
            ulFlags = USB_EP_MODE_CTRL;
        }
        ulFlags |= USB_EP_HOST_IN;

        g_sUSBHCD.USBINPipes[ulIndex].ucEPNumber =
            (unsigned char)ulTargetEndpoint;

        //
        // Save the interval and the next tick to trigger a scheduler event.
        //
        g_sUSBHCD.USBINPipes[ulIndex].ulInterval = ulInterval;
        g_sUSBHCD.USBINPipes[ulIndex].ulNextEventTick =
            ulInterval + g_ulCurrentTick;
    }

    //
    // Full speed by default but low speed will be selected if the device is
    // low speed.
    //
    ulFlags |= USB_EP_SPEED_FULL;

    //
    // Set up the appropriate flags if uDMA is used.
    //
    if(ulPipe & EP_PIPE_USE_UDMA)
    {
        ulFlags |= USB_EP_DMA_MODE_0;
    }

    //
    // Configure the endpoint according to the flags determined above.
    //
    USBHostEndpointConfig(USB0_BASE,
                          INDEX_TO_USB_EP((ulPipe & EP_PIPE_IDX_M) + 1),
                          ulMaxPayload, DISABLE_NAK_LIMIT, ulTargetEndpoint,
                          ulFlags);

    return(0);
}

//*****************************************************************************
//
//! This function is used to return the current status of a USB HCD pipe.
//!
//! This function will return the current status for a given USB pipe.  If
//! there is no status to report this call will simply return
//! \b USBHCD_PIPE_NO_CHANGE.
//!
//! \param ulPipe is the USB pipe for this status request.
//!
//! \return This function returns the current status for the given endpoint.
//! This will be one of the \b USBHCD_PIPE_* values.
//
//*****************************************************************************
unsigned long
USBHCDPipeStatus(unsigned long ulPipe)
{
    return(0);
}

//*****************************************************************************
//
//! This function is used to write data to a USB HCD pipe.
//!
//! \param ulPipe is the USB pipe to put data into.
//! \param pucData is a pointer to the data to send.
//! \param ulSize is the amount of data to send.
//!
//! This function will block until it has sent as much data as was
//! requested using the USB pipe's FIFO.  The caller should have registered a
//! callback with the USBHCDPipeAlloc() call in order to be informed when the
//! data has been transmitted.  The value returned by this function can be less
//! than the \e ulSize requested if the USB pipe has less space available than
//! this request is making.
//!
//! \return This function returns the number of bytes that were scheduled to
//! be sent on the given USB pipe.
//
//*****************************************************************************
unsigned long
USBHCDPipeWrite(unsigned long ulPipe, unsigned char *pucData,
                unsigned long ulSize)
{
    unsigned long ulEndpoint;
    unsigned long ulRemainingBytes;
    unsigned long ulByteToSend;
    unsigned long ulPipeIdx;

    //
    // Determine which endpoint interface that this pipe is using.
    //
    ulEndpoint = INDEX_TO_USB_EP((EP_PIPE_IDX_M & ulPipe) + 1);

    //
    // Get index used for looking up pipe data
    //
    ulPipeIdx = ulPipe & EP_PIPE_IDX_M;

    //
    // Set the total number of bytes to send out.
    //
    ulRemainingBytes = ulSize;

    if(ulSize > 64)
    {
        //
        // Only send 64 bytes at a time.
        //
        ulByteToSend = 64;
    }
    else
    {
        //
        // Send the requested number of bytes.
        //
        ulByteToSend = ulSize;
    }

    //
    // Send all of the requested data.
    //
    while(ulRemainingBytes != 0)
    {
        //
        // Start a write request.
        //
        g_sUSBHCD.USBOUTPipes[ulPipeIdx].eState = PIPE_WRITING;

        //
        // If uDMA is not enabled for this pipe, or if the uDMA workaround
        // is applied, then dont use uDMA for this transfer.
        //
        if(!(ulPipe & EP_PIPE_USE_UDMA) ||
           (g_bUseDMAWA && (ulByteToSend != 64)))
        {
            //
            // Disable uDMA on the USB endpoint
            //
            USBEndpointDMADisable(USB0_BASE, ulEndpoint, USB_EP_HOST_OUT);

            //
            // Put the data in the buffer.
            //
            USBEndpointDataPut(USB0_BASE, ulEndpoint, pucData, ulByteToSend);

            //
            // Schedule the data to be sent.
            //
            USBEndpointDataSend(USB0_BASE, ulEndpoint, USB_TRANS_OUT);
        }

        //
        // Otherwise, uDMA should be used for this transfer
        //
        else
        {
            //
            // Set up the uDMA transfer.
            //
            uDMAChannelTransferSet(UDMA_CHANNEL_USBEP1TX + (ulPipeIdx * 2),
                                   UDMA_MODE_AUTO, pucData,
                                   (void *)USBFIFOAddrGet(USB0_BASE,
                                                          ulEndpoint),
                                   ulByteToSend);

            //
            // Enable uDMA on the USB endpoint
            //
            USBEndpointDMAEnable(USB0_BASE, ulEndpoint, USB_EP_HOST_OUT);

            //
            // Set pending transmit DMA flag
            //
            g_ulDMAPending |= DMA_PEND_TRANSMIT_FLAG << ulPipeIdx;

            //
            // Enable the uDMA channel to start the transfer
            //
            uDMAChannelEnable(UDMA_CHANNEL_USBEP1TX + (ulPipeIdx * 2));
        }

        //
        // Wait for a status change.
        //
        while(g_sUSBHCD.USBOUTPipes[ulPipeIdx].eState == PIPE_WRITING)
        {
        }

        //
        // If the data was successfully sent then decrement the count and
        // continue.
        //
        if(g_sUSBHCD.USBOUTPipes[ulPipeIdx].eState == PIPE_DATA_SENT)
        {
            //
            // Decrement the remaining data and advance the pointer.
            //
            ulRemainingBytes -= ulByteToSend;
            pucData += ulByteToSend;
        }

        //
        // If there are less than 64 bytes to send then this is the last
        // of the data to go out.
        //
        if(ulRemainingBytes < 64)
        {
            ulByteToSend = ulRemainingBytes;
        }
    }

    return(ulSize);
}

//*****************************************************************************
//
//! This function is used to schedule and IN transaction on a USB HCD pipe.
//!
//! \param ulPipe is the USB pipe to read data from.
//! \param pucData is a pointer to store the data that is received.
//! \param ulSize is the size in bytes of the buffer pointed to by pucData.
//!
//! This function will not block depending on the type of pipe passed in will
//! schedule either a send of data to the device or a read of data from the
//! device.  In either case the amount of data will be limited to what will
//! fit in the FIFO for a given endpoint.
//!
//! \return This function returns the number of bytes that sent in the case
//! of a transfer of data or it will return 0 for a request on a USB IN pipe.
//
//*****************************************************************************
unsigned long
USBHCDPipeSchedule(unsigned long ulPipe, unsigned char *pucData,
                   unsigned long ulSize)
{
    unsigned long ulEndpoint;

    //
    // Determine which endpoint interface that this pipe is using.
    //
    ulEndpoint = INDEX_TO_USB_EP((EP_PIPE_IDX_M & ulPipe) + 1);

    if(ulPipe & EP_PIPE_TYPE_OUT)
    {
        //
        // Start a write request.
        //
        g_sUSBHCD.USBOUTPipes[EP_PIPE_IDX_M & ulPipe].eState = PIPE_WRITING;

        //
        // Put the data in the buffer.
        //
        USBEndpointDataPut(USB0_BASE, ulEndpoint, pucData, ulSize);

        //
        // Schedule the data to be sent.
        //
        USBEndpointDataSend(USB0_BASE, ulEndpoint, USB_TRANS_OUT);

    }
    else
    {
        //
        // Start a read request.
        //
        g_sUSBHCD.USBINPipes[EP_PIPE_IDX_M & ulPipe].eState = PIPE_READING;

        //
        // Trigger a request for data from the device.
        //
        USBHostRequestIN(USB0_BASE, ulEndpoint);

        //
        // No data was put into or read from the buffer.
        //
        ulSize = 0;
    }
    return(ulSize);
}

//*****************************************************************************
//
//! This function is used to read data from a USB HCD pipe.
//!
//! \param ulPipe is the USB pipe to read data from.
//! \param pucData is a pointer to store the data that is received.
//! \param ulSize is the size in bytes of the buffer pointed to by pucData.
//!
//! This function will not block and will only read as much data as requested
//! or as much data is currently available from the USB pipe.  The caller
//! should have registered a callback with the USBHCDPipeAlloc() call in order
//! to be informed when the data has been received.  The value returned by this
//! function can be less than the \e ulSize requested if the USB pipe has less
//! data available than was requested.
//!
//! \return This function returns the number of bytes that were returned in the
//! \e pucData buffer.
//
//*****************************************************************************
unsigned long
USBHCDPipeReadNonBlocking(unsigned long ulPipe, unsigned char *pucData,
                          unsigned long ulSize)
{
    unsigned long ulEndpoint;

    //
    // Determine which endpoint interface that this pipe is using.
    //
    ulEndpoint = INDEX_TO_USB_EP((EP_PIPE_IDX_M & ulPipe) + 1);

    //
    // Read the data out of the USB endpoint interface.
    //
    USBEndpointDataGet(USB0_BASE, ulEndpoint, pucData, &ulSize);

    //
    // Acknowledge that the data was read from the endpoint.
    //
    USBHostEndpointDataAck(USB0_BASE, ulEndpoint);

    //
    // Go Idle once this state has been reached.
    //
    g_sUSBHCD.USBINPipes[EP_PIPE_IDX_M & ulPipe].eState = PIPE_IDLE;

    return(ulSize);
}

//*****************************************************************************
//
//! This function is used to read data from a USB HCD pipe.
//!
//! \param ulPipe is the USB pipe to read data from.
//! \param pucData is a pointer to store the data that is received.
//! \param ulSize is the size in bytes of the buffer pointed to by pucData.
//!
//! This function will block and will only return when it has read as much data
//! as requested from the USB pipe.  The caller should have registered a
//! callback with the USBHCDPipeAlloc() call in order to be informed when the
//! data has been received.  The value returned by this function can be less
//! than the \e ulSize requested if the USB pipe has less data available than
//! was requested.
//!
//! \return This function returns the number of bytes that were returned in the
//! \e pucData buffer.
//
//*****************************************************************************
unsigned long
USBHCDPipeRead(unsigned long ulPipe, unsigned char *pucData,
               unsigned long ulSize)
{
    unsigned long ulEndpoint;
    unsigned long ulRemainingBytes;
    unsigned long ulBytesRead;
    unsigned long ulPipeIdx;

    //
    // Get index used for looking up pipe data
    //
    ulPipeIdx = ulPipe & EP_PIPE_IDX_M;

    //
    // Initialized the number of bytes read.
    //
    ulBytesRead = 0;

    //
    // Determine which endpoint interface that this pipe is using.
    //
    ulEndpoint = INDEX_TO_USB_EP(ulPipeIdx + 1);

    //
    // Set the remaining bytes to received.
    //
    ulRemainingBytes = ulSize;

    //
    // Continue until all data requested has been received.
    //
    while(ulRemainingBytes != 0)
    {
        //
        // Start a read request.
        //
        g_sUSBHCD.USBINPipes[ulPipeIdx].eState = PIPE_READING;

        //
        // If uDMA is not enabled for this pipe, or if the uDMA workaround
        // is applied, then dont use uDMA for this transfer.
        //
        if(!(ulPipe & EP_PIPE_USE_UDMA) ||
            (g_bUseDMAWA && (ulRemainingBytes < 64)))
        {
            //
            // Disable uDMA on the endpoint
            //
            USBEndpointDMADisable(USB0_BASE, ulEndpoint, USB_EP_HOST_IN);
        }

        //
        // Otherwise, uDMA should be used for this transfer, so set up
        // the uDMA channel in advance of triggering the IN request.
        //
        else
        {
            //
            // Compute bytes to transfer and set up transfer
            //
            ulBytesRead = ulRemainingBytes > 64 ? 64 : ulRemainingBytes;
            uDMAChannelTransferSet(UDMA_CHANNEL_USBEP1RX + (ulPipeIdx * 2),
                                   UDMA_MODE_AUTO,
                                   (void *)USBFIFOAddrGet(USB0_BASE,
                                                          ulEndpoint),
                                   pucData, ulBytesRead);

            //
            // Enable uDMA on the endpoint
            //
            USBEndpointDMAEnable(USB0_BASE, ulEndpoint, USB_EP_HOST_IN);

            //
            // Set pending DMA flag
            //
            g_ulDMAPending |= DMA_PEND_RECEIVE_FLAG << ulPipeIdx;

            //
            // Enable the uDMA channel to start the transfer
            //
            uDMAChannelEnable(UDMA_CHANNEL_USBEP1RX + (ulPipeIdx * 2));
        }

        //
        // Trigger a request for data from the device.
        //
        USBHostRequestIN(USB0_BASE, ulEndpoint);

        //
        // Wait for a status change.
        //
        while(g_sUSBHCD.USBINPipes[ulPipeIdx].eState == PIPE_READING)
        {
        }

        //
        // If data is ready then return it.
        //
        if(g_sUSBHCD.USBINPipes[ulPipeIdx].eState == PIPE_DATA_READY)
        {
            //
            // If not using uDMA then read the data from the USB.  Otherwise
            // the data will already be in the buffer.
            //
            if(!(ulPipe & EP_PIPE_USE_UDMA) ||
                (g_bUseDMAWA && (ulRemainingBytes < 64)))
            {
                //
                // Request all of the remaining bytes.
                //
                ulBytesRead = ulRemainingBytes;

                //
                // Read the data out of the USB endpoint interface.
                //
                USBEndpointDataGet(USB0_BASE, ulEndpoint, pucData,
                                   &ulBytesRead);

                //
                // Acknowledge that the data was read from the endpoint.
                //
                USBHostEndpointDataAck(USB0_BASE, ulEndpoint);
            }

            //
            // If there were less than 64 bytes read, then this was a short
            // packet and no more data will be returned.
            //
            if(ulBytesRead < 64)
            {
                //
                // Subtract off the bytes that were not received and exit the
                // loop.
                //
                ulSize = ulSize - ulRemainingBytes;
                break;
            }
            else
            {
                //
                // There are more bytes to read so remove the bytes that were
                // read and continue receiveing data.
                //
                ulRemainingBytes -= ulBytesRead;
                pucData += 64;
            }
        }
        else if(g_sUSBHCD.USBINPipes[ulPipeIdx].eState ==
                PIPE_STALLED)
        {
            //
            // This is the actual endpoint number.
            //
            USBHCDClearFeature(1, ulPipe, USB_FEATURE_EP_HALT);

            //
            // If uDMA is being used, then disable the channel.
            //
            if(ulPipe & EP_PIPE_USE_UDMA)
            {
                uDMAChannelDisable(UDMA_CHANNEL_USBEP1RX + (ulPipeIdx * 2));
            }

            //
            // If there was a stall, then no more data is coming so break out.
            //
            break;
        }
    }

    //
    // Go Idle once this state has been reached.
    //
    g_sUSBHCD.USBINPipes[ulPipeIdx].eState = PIPE_IDLE;

    return(ulSize);
}

//*****************************************************************************
//
//! This function is used to release a USB pipe.
//!
//! \param ulPipe is the allocated USB pipe to release.
//!
//! This function is used to release a USB pipe that was allocated by a call to
//! USBHCDPipeAlloc() for use by some other device endpoint in the system.
//! Freeing an unallocated or invalid pipe will not generate an error and will
//! instead simply return.
//!
//! \return None.
//
//*****************************************************************************
void
USBHCDPipeFree(unsigned long ulPipe)
{
    int iDMAIdx;

    if(ulPipe & EP_PIPE_TYPE_OUT)
    {
        //
        // Clear the address and type for this endpoint to free it up.
        //
        g_sUSBHCD.USBOUTPipes[ulPipe & EP_PIPE_IDX_M].ulDevAddr = 0;
        g_sUSBHCD.USBOUTPipes[ulPipe & EP_PIPE_IDX_M].ulType = 0;
        g_sUSBHCD.USBOUTPipes[ulPipe & EP_PIPE_IDX_M].pfnCallback = 0;

        //
        // Get the dma channel used by this pipe.
        //
        iDMAIdx = g_sUSBHCD.USBOUTPipes[ulPipe & EP_PIPE_IDX_M].ucDMAChannel;

        //
        // Mark the channel as free for use.
        //
        g_sUSBHCD.ucDMAChannels[iDMAIdx] = USBHCD_DMA_UNUSED;

        //
        // Clear out the current channel in use by this pipe.
        //
        g_sUSBHCD.USBOUTPipes[ulPipe & EP_PIPE_IDX_M].ucDMAChannel =
            USBHCD_DMA_UNUSED;
    }
    else if(ulPipe & EP_PIPE_TYPE_IN)
    {
        //
        // Clear the address and type for this endpoint to free it up.
        //
        g_sUSBHCD.USBINPipes[ulPipe & EP_PIPE_IDX_M].ulDevAddr = 0;
        g_sUSBHCD.USBINPipes[ulPipe & EP_PIPE_IDX_M].ulType = 0;
        g_sUSBHCD.USBINPipes[ulPipe & EP_PIPE_IDX_M].pfnCallback = 0;

        //
        // Get the dma channel used by this pipe.
        //
        iDMAIdx = g_sUSBHCD.USBINPipes[ulPipe & EP_PIPE_IDX_M].ucDMAChannel;

        //
        // Mark the channel as free for use.
        //
        g_sUSBHCD.ucDMAChannels[iDMAIdx] = USBHCD_DMA_UNUSED;

        //
        // Clear out the current channel in use by this pipe.
        //
        g_sUSBHCD.USBINPipes[ulPipe & EP_PIPE_IDX_M].ucDMAChannel =
            USBHCD_DMA_UNUSED;
    }
}

//*****************************************************************************
//
// This internal function initializes the HCD code.
//
// \param ulIndex specifies which USB controller to use.
// \param pvPool is a pointer to the data to use as a memory pool for this
// controller.
// \param ulPoolSize is the size in bytes of the buffer passed in as pvPool.
//
// This function will perform all the necessary operations to allow the USB
// host controller to begin enumeration and communication with a device.  This
// function should typically be called once at the start of an application
// before any other calls are made to the host controller.
//
// \return None.
//
//*****************************************************************************
static void
USBHCDInitInternal(unsigned long ulIndex, void *pvPool,
                   unsigned long ulPoolSize)
{
    int iIdx;
    ASSERT(ulIndex == 0);

    //
    // Save the base address for this controller.
    //
    g_sUSBHCD.ulUSBBase = USB0_BASE;

    //
    // All Pipes are unused at start.
    //
    for(iIdx = 0; iIdx < MAX_NUM_PIPES; iIdx++)
    {
        g_sUSBHCD.USBINPipes[iIdx].ulDevAddr = 0;
        g_sUSBHCD.USBINPipes[iIdx].ulType = USBHCD_PIPE_UNUSED;
        g_sUSBHCD.USBINPipes[iIdx].ucDMAChannel = USBHCD_DMA_UNUSED;
        g_sUSBHCD.USBOUTPipes[iIdx].ulDevAddr = 0;
        g_sUSBHCD.USBOUTPipes[iIdx].ulType = USBHCD_PIPE_UNUSED;
        g_sUSBHCD.USBOUTPipes[iIdx].ucDMAChannel = USBHCD_DMA_UNUSED;
    }

    //
    // All DMA channels are unused at start.
    //
    for(iIdx = 0; iIdx < MAX_NUM_DMA_CHANNELS; iIdx++)
    {
        g_sUSBHCD.ucDMAChannels[iIdx] = USBHCD_DMA_UNUSED;
    }

    //
    // Initialized the device structure.
    //
    g_sUSBHCD.eDeviceState[0] = HCD_IDLE;
    g_sUSBHCD.USBDevice[0].pConfigDescriptor = 0;

    //
    // Initialize the device descriptor.
    //
    g_sUSBHCD.USBDevice[0].DeviceDescriptor.bLength = 0;
    g_sUSBHCD.USBDevice[0].DeviceDescriptor.bMaxPacketSize0 = 0;

    //
    // Initialize the device address.
    //
    g_sUSBHCD.USBDevice[0].ulAddress = 0;

    //
    // Set the current interface to 0.
    //
    g_sUSBHCD.USBDevice[0].ulInterface = 0;

    //
    // Allocate the memory needed for reading descriptors.
    //
    g_sUSBHCD.pvPool = pvPool;
    g_sUSBHCD.ulPoolSize = ulPoolSize;

    //
    // Initialize the device class.
    //
    g_sUSBHCD.ulClass = USB_CLASS_EVENTS;

    //
    // Initialize the USB tick module.
    //
    InternalUSBTickInit();

    //
    // Only do hardware update if the stack is in Host mode, do not touch the
    // hardware for OTG mode operation.
    //
    if(g_eUSBMode == USB_MODE_HOST)
    {
        //
        // Configure the End point 0.
        //
        USBHostEndpointConfig(USB0_BASE, USB_EP_0, 64, 0, 0,
                              (USB_EP_MODE_CTRL | USB_EP_SPEED_FULL |
                               USB_EP_HOST_OUT));

        //
        // Enable USB Interrupts.
        //
        USBIntEnableControl(USB0_BASE, USB_INTCTRL_RESET |
                                       USB_INTCTRL_DISCONNECT |
                                       USB_INTCTRL_SOF |
                                       USB_INTCTRL_SESSION |
                                       USB_INTCTRL_BABBLE |
                                       USB_INTCTRL_CONNECT |
                                       USB_INTCTRL_RESUME |
                                       USB_INTCTRL_SUSPEND |
                                       USB_INTCTRL_VBUS_ERR |
                                       USB_INTCTRL_POWER_FAULT);
        USBIntEnableEndpoint(USB0_BASE, USB_INTEP_ALL);

        //
        // Enable the USB interrupt.
        //
        IntEnable(INT_USB0);

        //
        // Initialize the power configuration.  This sets the power enable
        // signal to be active high and does not enable the power fault.
        //
        USBHostPwrFaultConfig(USB0_BASE, g_ulPowerConfig);

        //
        // Power the USB bus.
        //
        USBHostPwrEnable(USB0_BASE);
    }

    //
    // This only has effect on non-OTG parts.
    //
    USBHostMode(USB0_BASE);

    //
    // Only do hardware update if the stack is in Host mode, do not touch the
    // hardware for OTG mode operation.
    //
    if(g_eUSBMode == USB_MODE_HOST)
    {
        //
        // Start the session.
        //
        USBOTGSessionRequest(USB0_BASE, true);
    }
}

//*****************************************************************************
//
//! This function is used to set the power pin and power fault configuration.
//!
//! \param ulIndex specifies which USB controller to use.
//! \param ulPwrConfig is the power configuration to use for the application.
//!
//! This function must be called before HCDInit() is called so that the power
//! pin configuration can be set before power is enabled.  The \e ulPwrConfig
//! flags specify the power fault level sensitivity, the power fault action,
//! and the power enable pin level and source.
//!
//! One of the following can be selected as the power fault level sensitivity:
//!
//! - \b USB_HOST_PWRFLT_LOW - Power fault is indicated by the pin being driven
//!   low.
//! - \b USB_HOST_PWRFLT_HIGH - Power fault is indicated by the pin being
//!   driven high.
//!
//! One of the following can be selected as the power fault action:
//!
//! - \b USB_HOST_PWRFLT_EP_NONE - No automatic action when power fault
//!   detected.
//! - \b USB_HOST_PWRFLT_EP_TRI - Automatically Tri-state the power enable pin
//!   on a power fault.
//! - \b USB_HOST_PWRFLT_EP_LOW - Automatically drive power enable pin low on a
//!   power fault.
//! - \b USB_HOST_PWRFLT_EP_HIGH - Automatically drive power enable pin high on
//!   a power fault.
//!
//! One of the following can be selected as the power enable level and source:
//!
//! - \b USB_HOST_PWREN_LOW - power enable pin is driven low when power is
//!   enabled.
//! - \b USB_HOST_PWREN_HIGH - power enable pin is driven high when power is
//!   enabled.
//! - \b USB_HOST_PWREN_VBLOW - power enable pin is driven high when VBUS is
//!   low.
//! - \b USB_HOST_PWREN_VBHIGH - power enable pin is driven high when VBUS is
//!   high.
//!
//! \return None.
//
//*****************************************************************************
void
USBHCDPowerConfigInit(unsigned long ulIndex, unsigned long ulPwrConfig)
{
    ASSERT(ulIndex == 0);

    //
    // Save the value as it will be used later.
    //
    g_ulPowerConfig = ulPwrConfig;
}

//*****************************************************************************
//
//! This function is used to initialize the HCD code.
//!
//! \param ulIndex specifies which USB controller to use.
//! \param pvPool is a pointer to the data to use as a memory pool for this
//! controller.
//! \param ulPoolSize is the size in bytes of the buffer passed in as pvPool.
//!
//! This function will perform all the necessary operations to allow the USB
//! host controller to begin enumeration and communication with devices.  This
//! function should typically be called once at the start of an application
//! once all of the device and class drivers are ready for normal operation.
//! This call will start up the USB host controller and any connected device
//! will immediately start the enumeration sequence.
//!
//! \return None.
//
//*****************************************************************************
void
USBHCDInit(unsigned long ulIndex, void *pvPool, unsigned long ulPoolSize)
{
    int iDriver;

    //
    // Check the arguments.
    //
    ASSERT(ulIndex == 0);

    //
    // Make sure there is at least enough to read the configuration descriptor.
    //
    ASSERT(ulPoolSize >= sizeof(tConfigDescriptor));

    //
    // Should not call this if the stack is in device mode.
    //
    ASSERT(g_eUSBMode != USB_MODE_DEVICE)

    //
    // If no mode is set then make the mode become host mode.
    //
    if(g_eUSBMode == USB_MODE_NONE)
    {
        g_eUSBMode = USB_MODE_HOST;
    }

    //
    // Only do hardware update if the stack is in Host mode, do not touch the
    // hardware for OTG mode operation.
    //
    if(g_eUSBMode == USB_MODE_HOST)
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
    // Call our internal function to perform the initialization.
    //
    USBHCDInitInternal(ulIndex, pvPool, ulPoolSize);

    //
    // No event driver is present by default.
    //
    g_sUSBHCD.iEventDriver = -1;

    //
    // Search through the Host Class driver list for the devices class.
    //
    for(iDriver = 0; iDriver < g_sUSBHCD.ulNumClassDrivers; iDriver++)
    {
        if(g_sUSBHCD.pClassDrivers[iDriver]->ulInterfaceClass ==
           USB_CLASS_EVENTS)
        {
            //
            // Event driver was found so remember it.
            //
            g_sUSBHCD.iEventDriver = iDriver;
        }
    }

    //
    // Get the number of ticks per millisecond, this is only used by blocking
    // delays using the SysCtlDelay() function.
    //
    g_ulTickms = SysCtlClockGet() / 3000;

    //
    // Check to see if uDMA workaround is needed.
    //
    if(CLASS_IS_DUSTDEVIL && REVISION_IS_A0)
    {
        g_bUseDMAWA = 1;
    }
}

//*****************************************************************************
//
//! This function is used to initialize the HCD class driver list.
//!
//! \param ulIndex specifies which USB controller to use.
//! \param ppHClassDrvs is an array of host class drivers that are
//! supported on this controller.
//! \param ulNumDrivers is the number of entries in the \e pHostClassDrivers
//! array.
//!
//! This function will set the host classes supported by the host controller
//! specified by the \e ulIndex parameter.  This function should be called
//! before enabling the host controller driver with the USBHCDInit() function.
//!
//! \return None.
//
//*****************************************************************************
void
USBHCDRegisterDrivers(unsigned long ulIndex,
                      const tUSBHostClassDriver * const *ppHClassDrvs,
                      unsigned long ulNumDrivers)
{
    ASSERT(ulIndex == 0);

    //
    // Save the class drivers.
    //
    g_sUSBHCD.pClassDrivers = ppHClassDrvs;

    //
    // Save the number of class drivers.
    //
    g_sUSBHCD.ulNumClassDrivers = ulNumDrivers;
}

//*****************************************************************************
//
//! This function is used to terminate the HCD code.
//!
//! \param ulIndex specifies which USB controller to release.
//!
//! This function will clean up the USB host controller and disable it in
//! preparation for shutdown or a switch to USB device mode.  Once this call is
//! made, \e USBHCDInit() may be called to reinitialize the controller and
//! prepare for host mode operation.
//!
//! \return None.
//
//*****************************************************************************
void
USBHCDTerm(unsigned long ulIndex)
{
    ASSERT(ulIndex == 0);

    //
    // End the session.
    //
    USBOTGSessionRequest(USB0_BASE, false);

    //
    // Remove power from the USB bus.
    //
    USBHostPwrDisable(USB0_BASE);

    //
    // Disable USB interrupts.
    //
    IntDisable(INT_USB0);

    USBIntDisableControl(USB0_BASE, USB_INTCTRL_ALL);

    USBIntDisableEndpoint(USB0_BASE, USB_INTEP_ALL);

    //
    // Set the host controller state back to it's initial values.
    //
    g_sUSBHCD.USBINPipes[0].ulType = USBHCD_PIPE_UNUSED;
    g_sUSBHCD.USBINPipes[1].ulType = USBHCD_PIPE_UNUSED;
    g_sUSBHCD.USBINPipes[2].ulType = USBHCD_PIPE_UNUSED;
    g_sUSBHCD.USBOUTPipes[0].ulType = USBHCD_PIPE_UNUSED;
    g_sUSBHCD.USBOUTPipes[1].ulType = USBHCD_PIPE_UNUSED;
    g_sUSBHCD.USBOUTPipes[2].ulType = USBHCD_PIPE_UNUSED;
    g_sUSBHCD.eDeviceState[0] = HCD_IDLE;
    g_sUSBHCD.USBDevice[0].pConfigDescriptor = 0;
    g_sUSBHCD.USBDevice[0].DeviceDescriptor.bLength = 0;
    g_sUSBHCD.USBDevice[0].DeviceDescriptor.bMaxPacketSize0 = 0;
    g_sUSBHCD.USBDevice[0].ulAddress = 0;
    g_sUSBHCD.USBDevice[0].ulInterface = 0;
    g_sUSBHCD.pvPool = 0;
    g_sUSBHCD.ulPoolSize = 0;
}

//*****************************************************************************
//
//! This function generates reset signaling on the USB bus.
//!
//! \param ulIndex specifies which USB controller to use.
//!
//! This function handles sending out reset signaling on the USB bus.  After
//! returning from this function, any attached device on the USB bus should
//! have returned to it's reset state.
//!
//! \return None.
//
//*****************************************************************************
void
USBHCDReset(unsigned long ulIndex)
{
    ASSERT(ulIndex == 0);

    //
    // Start the reset signaling.
    //
    USBHostReset(USB0_BASE, 1);

    //
    // Wait 20ms
    //
    SysCtlDelay(g_ulTickms * 20);

    //
    // End reset signaling on the bus.
    //
    USBHostReset(USB0_BASE, 0);

    //
    // Need to wait at least 10ms to let the device recover from
    // the reset.  This is the delay specified in the USB 2.0 spec.
    // We will hold the reset for 20ms.
    //
    SysCtlDelay(g_ulTickms * 20);
}

//*****************************************************************************
//
//! This function will generate suspend signaling on the USB bus.
//!
//! \param ulIndex specifies which USB controller to use.
//!
//! This function is used to generate suspend signaling on the USB bus.  In
//! order to leave the suspended state, the application should call
//! USBHCDResume().
//!
//! \return None.
//
//*****************************************************************************
void
USBHCDSuspend(unsigned long ulIndex)
{
    ASSERT(ulIndex == 0);

    //
    // Start the suspend signaling.
    //
    USBHostSuspend(USB0_BASE);
}

//*****************************************************************************
//
//! This function will generate resume signaling on the USB bus.
//!
//! \param ulIndex specifies which USB controller to use.
//!
//! This function is used to generate resume signaling on the USB bus in order
//! to cause  USB devices to leave their suspended state.  This call should
//! not be made unless a preceding call to USBHCDSuspend() has been made.
//!
//! \return None.
//
//*****************************************************************************
void
USBHCDResume(unsigned long ulIndex)
{
    ASSERT(ulIndex == 0);

    //
    // Start the resume signaling.
    //
    USBHostResume(USB0_BASE, 1);

    //
    // Wait 20ms
    //
    SysCtlDelay(g_ulTickms * 100);

    //
    // End reset signaling on the bus.
    //
    USBHostResume(USB0_BASE, 0);
}

//*****************************************************************************
//
//! This function issues a request for the current configuration descriptor
//! from a device.
//!
//! \param ulIndex specifies which USB controller to use.
//! \param pDevice is a pointer to the device structure that holds the buffer
//! to store the configuration descriptor.
//!
//! This function will request the configuration descriptor from the device.
//! The \e pDevice->ConfigDescriptor member variable is used to hold the data
//! for this request.  This buffer will be allocated from the pool provided by
//! the HCDInit() function.  \e pDevice->DeviceDescriptor.bMaxPacketSize0
//! should be valid prior to this call in order to correctly receive the
//! configuration descriptor.  If this variable is not valid then this call
//! will not return accurate data.
//!
//! \return The number of bytes returned due to the request.  This value can be
//! zero if the device did not respond.
//
//*****************************************************************************
static unsigned long
USBHCDGetConfigDescriptor(unsigned long ulIndex, tUSBHostDevice *pDevice)
{
    tUSBRequest SetupPacket;
    unsigned long ulBytes;

    ASSERT(ulIndex == 0);

    ulBytes = 0;

    //
    // This is a Standard Device IN request.
    //
    SetupPacket.bmRequestType =
        USB_RTYPE_DIR_IN | USB_RTYPE_STANDARD | USB_RTYPE_DEVICE;

    //
    // Request a Device Descriptor.
    //
    SetupPacket.bRequest = USBREQ_GET_DESCRIPTOR;
    SetupPacket.wValue = USB_DTYPE_CONFIGURATION << 8;

    //
    // Index is always 0 for device configurations requests.
    //
    SetupPacket.wIndex = 0;

    //
    // Only ask for the configuration header first to see how big the
    // whole thing is.
    //
    if (g_sUSBHCD.USBDevice[0].pConfigDescriptor == 0)
    {
        //
        // Only request the space available.
        //
        SetupPacket.wLength = sizeof(tConfigDescriptor);

        //
        // Set the memory to use for the config descriptor and save the size.
        //
        g_sUSBHCD.USBDevice[0].pConfigDescriptor = g_sUSBHCD.pvPool;
        g_sUSBHCD.USBDevice[0].ulConfigDescriptorSize = g_sUSBHCD.ulPoolSize;

        //
        // Put the setup packet in the buffer.
        //
        ulBytes =
            USBHCDControlTransfer(0, &SetupPacket, pDevice->ulAddress,
                                  (unsigned char *)pDevice->pConfigDescriptor,
                                  sizeof(tConfigDescriptor),
                                  pDevice->DeviceDescriptor.bMaxPacketSize0);
    }

    //
    // If the Configuration header was successfully returned then get the
    // full configuration descriptor.
    //
    if(ulBytes == sizeof(tConfigDescriptor))
    {
        //
        // Save the total size and request the full configuration descriptor.
        //
        SetupPacket.wLength =
            g_sUSBHCD.USBDevice[0].pConfigDescriptor->wTotalLength;

        //
        // Don't allow the buffer to be larger than was allocated.
        //
        if(SetupPacket.wLength > g_sUSBHCD.ulPoolSize)
        {
            SetupPacket.wLength = g_sUSBHCD.ulPoolSize;
        }

        //
        // Put the setup packet in the buffer.
        //
        ulBytes =
            USBHCDControlTransfer(0, &SetupPacket, pDevice->ulAddress,
                                  (unsigned char *)pDevice->pConfigDescriptor,
                                  SetupPacket.wLength,
                                  pDevice->DeviceDescriptor.bMaxPacketSize0);
    }

    return(ulBytes);
}

//*****************************************************************************
//
//! This function issues a request for a device descriptor from a device.
//!
//! \param ulIndex specifies which USB controller to use.
//! \param pDevice is a pointer to the device structure that holds the buffer
//! to store the device descriptor into.
//!
//! This function will request the device descriptor from the device.  The
//! \e pDevice->DeviceDescriptor descriptor is used to hold the data for this
//! request.  \e pDevice->DeviceDescriptor.bMaxPacketSize0 should be
//! initialized to zero or to the valid maximum packet size if it is known.  If
//! this variable is not set to zero, then this call will determine the maximum
//! packet size for endpoint 0 and save it in the structure member
//! bMaxPacketSize0.
//!
//! \return The number of bytes returned due to the request.  This value can be
//! zero if the device did not respond.
//
//*****************************************************************************
static unsigned long
USBHCDGetDeviceDescriptor(unsigned long ulIndex, tUSBHostDevice *pDevice)
{
    tUSBRequest SetupPacket;
    unsigned long ulBytes;

    ASSERT(ulIndex == 0);

    //
    // This is a Standard Device IN request.
    //
    SetupPacket.bmRequestType =
        USB_RTYPE_DIR_IN | USB_RTYPE_STANDARD | USB_RTYPE_DEVICE;

    //
    // Request a Device Descriptor.
    //
    SetupPacket.bRequest = USBREQ_GET_DESCRIPTOR;
    SetupPacket.wValue = USB_DTYPE_DEVICE << 8;

    //
    // Index is always 0 for device requests.
    //
    SetupPacket.wIndex = 0;

    //
    // All devices must have at least an 8 byte max packet size so just ask
    // for 8 bytes to start with.
    //
    SetupPacket.wLength = 8;

    ulBytes = 0;

    //
    // Discover the max packet size for endpoint 0.
    //
    if(pDevice->DeviceDescriptor.bMaxPacketSize0 == 0)
    {
        //
        // Put the setup packet in the buffer.
        //
        ulBytes =
            USBHCDControlTransfer(ulIndex, &SetupPacket, pDevice->ulAddress,
                                  (unsigned char *)&(pDevice->DeviceDescriptor),
                                  sizeof(tDeviceDescriptor),
                                  MAX_PACKET_SIZE_EP0);
    }

    //
    // Now get the full descriptor now that the actual maximum packet size
    // is known.
    //
    if(ulBytes < sizeof(tDeviceDescriptor))
    {
        SetupPacket.wLength = (unsigned short)sizeof(tDeviceDescriptor);

        ulBytes =
            USBHCDControlTransfer(ulIndex, &SetupPacket, pDevice->ulAddress,
                                  (unsigned char *)&(pDevice->DeviceDescriptor),
                                  sizeof(tDeviceDescriptor),
                                  pDevice->DeviceDescriptor.bMaxPacketSize0);
    }

    return(ulBytes);
}

//*****************************************************************************
//
//! This function is used to send the set address command to a device.
//!
//! \param ulDevAddress is the new device address to use for a device.
//!
//! The USBHCDSetAddress() function is used to set the USB device address, once
//! a device has been discovered on the bus.  This is typically issued
//! following a USB reset which is triggered by a call the USBHCDReset().  The
//! address passed into this function via the \e ulDevAddress parameter should
//! be used for all further communications with the device once this function
//! returns.
//!
//! \return None.
//
//*****************************************************************************
void
USBHCDSetAddress(unsigned long ulDevAddress)
{
    tUSBRequest SetupPacket;

    //
    // This is a Standard Device OUT request.
    //
    SetupPacket.bmRequestType =
        USB_RTYPE_DIR_OUT | USB_RTYPE_STANDARD | USB_RTYPE_DEVICE;

    //
    // Request a Device Descriptor.
    //
    SetupPacket.bRequest = USBREQ_SET_ADDRESS;
    SetupPacket.wValue = ulDevAddress;

    //
    // Index is always 0 for device requests.
    //
    SetupPacket.wIndex = 0;

    //
    // Only request the space available.
    //
    SetupPacket.wLength = 0;

    //
    // Put the setup packet in the buffer.
    //
    USBHCDControlTransfer(0, &SetupPacket, 0, 0, 0, MAX_PACKET_SIZE_EP0);

    //
    // Must delay 2ms after setting the address.
    //
    SysCtlDelay(g_ulTickms * 2);
}

//*****************************************************************************
//
//! This function is used to send a Clear Feature request to a device.
//!
//! \param ulDevAddress is the USB bus address of the device that will receive
//! this request.
//! \param ulPipe is the pipe that will be used to send the request.
//! \param ulFeature is one of the USB_FEATURE_* definitions.
//!
//! This function will issue a Clear Feature request to the device indicated
//! by the \e ulDevAddress parameter.  The \e ulPipe parameter is the USB pipe
//! that should be used to send this request.  The \e ulFeature parameter
//! should be one of the following values:
//!
//! * \b USB_FEATURE_EP_HALT is used to end a HALT condition on a devices
//!   endpoint.
//! * \b USB_FEATURE_REMOTE_WAKE is used to disable a device's remote wake
//!   feature.
//! * \b USB_FEATURE_TEST_MODE is used take the USB device out of test mode.
//!
//! \return None.
//
//*****************************************************************************
void
USBHCDClearFeature(unsigned long ulDevAddress, unsigned long ulPipe,
                   unsigned long ulFeature)
{
    tUSBRequest SetupPacket;
    unsigned long ulIndex;

    //
    // Get the index number from the allocated pipe.
    //
    ulIndex = (ulPipe & EP_PIPE_IDX_M);

    //
    // This is a Standard Device OUT request.
    //
    SetupPacket.bmRequestType =
        USB_RTYPE_DIR_OUT | USB_RTYPE_STANDARD | USB_RTYPE_ENDPOINT;

    //
    // Request a Device Descriptor.
    //
    SetupPacket.bRequest = USBREQ_CLEAR_FEATURE;
    SetupPacket.wValue = ulFeature;

    //
    // Set the endpoint to access.
    //
    if(ulPipe & EP_PIPE_TYPE_IN)
    {
        SetupPacket.wIndex = g_sUSBHCD.USBINPipes[ulIndex].ucEPNumber | 0x80;
    }
    else
    {
        SetupPacket.wIndex = g_sUSBHCD.USBOUTPipes[ulIndex].ucEPNumber;
    }

    //
    // This is always 0.
    //
    SetupPacket.wLength = 0;

    //
    // Put the setup packet in the buffer.
    //
    USBHCDControlTransfer(0, &SetupPacket, ulDevAddress, 0, 0,
                          MAX_PACKET_SIZE_EP0);

    //
    // Must delay 2ms after clearing the feature.
    //
    SysCtlDelay(g_ulTickms * 2);
}

//*****************************************************************************
//
//! This function is used to set the current configuration for a device.
//!
//! \param ulIndex specifies which USB controller to use.
//! \param ulDevice is the USB device for this function.
//! \param ulConfiguration is one of the devices valid configurations.
//!
//! This function is used to set the current device configuration for a USB
//! device.  The \e ulConfiguration value must be one of the configuration
//! indexes that was returned in the configuration descriptor from the device,
//! or a value of 0.  If 0 is passed in, the device will return to it's
//! addressed state and no longer be in a configured state.  If the value is
//! non-zero then the device will change to the requested configuration.
//!
//! \return None.
//
//*****************************************************************************
void
USBHCDSetConfig(unsigned long ulIndex, unsigned long ulDevice,
                unsigned long ulConfiguration)
{
    tUSBRequest SetupPacket;
    tUSBHostDevice *pDevice;

    ASSERT(ulIndex == 0);

    pDevice = (tUSBHostDevice *)ulDevice;

    //
    // This is a Standard Device OUT request.
    //
    SetupPacket.bmRequestType =
        USB_RTYPE_DIR_OUT | USB_RTYPE_STANDARD | USB_RTYPE_DEVICE;

    //
    // Request a Device Descriptor.
    //
    SetupPacket.bRequest = USBREQ_SET_CONFIG;
    SetupPacket.wValue = ulConfiguration;

    //
    // Index is always 0 for device requests.
    //
    SetupPacket.wIndex = 0;

    //
    // Only request the space available.
    //
    SetupPacket.wLength = 0;

    //
    // Put the setup packet in the buffer.
    //
    USBHCDControlTransfer(0, &SetupPacket, pDevice->ulAddress, 0, 0,
                          MAX_PACKET_SIZE_EP0);
}

//*****************************************************************************
//
// The internal function to see if a new schedule event should occur.
//
// This function is called by the main interrupt handler due to start of frame
// interrupts to determine if a new scheduler event should be sent to the USB
// pipe.
//
// \return None.
//
//*****************************************************************************
void
USBHostCheckPipes(void)
{
    int iIdx;

    for(iIdx = 0; iIdx < 3; iIdx++)
    {
        //
        // Skip unused pipes.
        //
        if(g_sUSBHCD.USBINPipes[iIdx].ulType == USBHCD_PIPE_UNUSED)
        {
            continue;
        }

        //
        // If the tick has expired and it has an interval then update it.
        //
        if((g_sUSBHCD.USBINPipes[iIdx].ulInterval != 0) &&
           (g_sUSBHCD.USBINPipes[iIdx].ulNextEventTick == g_ulCurrentTick))
        {
            //
            // Schedule the next event.
            //
            g_sUSBHCD.USBINPipes[iIdx].ulNextEventTick +=
                g_sUSBHCD.USBINPipes[iIdx].ulInterval;

            //
            // If the pipe is IDLE and there is a callback, let the higher
            // level drivers know that a new transfer can be scheduled.
            //
            if((g_sUSBHCD.USBINPipes[iIdx].eState == PIPE_IDLE) &&
               (g_sUSBHCD.USBINPipes[iIdx].pfnCallback))
            {
                g_sUSBHCD.USBINPipes[iIdx].pfnCallback(IN_PIPE_HANDLE(iIdx),
                                                       USB_EVENT_SCHEDULER);
            }
        }
    }
}

//*****************************************************************************
//
// The internal USB host mode interrupt handler.
//
// \param ulStatus is the current interrupt status as read via a call to
// \e USBIntStatusControl().
//
// This the main USB interrupt handler called when operating in host mode.
// This handler will branch the interrupt off to the appropriate handlers
// depending on the current status of the USB controller.
//
// The two-tiered structure for the interrupt handler ensures that it is
// possible to use the same handler code in both host and OTG modes and
// means that device code can be excluded from applications that only require
// support for USB host mode operation.
//
// \return None.
//
//*****************************************************************************
void
USBHostIntHandlerInternal(unsigned long ulStatus)
{
    unsigned long ulEPStatus;
    static unsigned long ulSOFDivide = 0;
    unsigned long ulEvent;
    unsigned long ulIdx;

    if(ulStatus & USB_INTCTRL_SOF)
    {
        g_ulCurrentTick++;

        USBHostCheckPipes();
    }

    //
    // A power fault has occurred so notify the application.
    //
    if(ulStatus & USB_INTCTRL_POWER_FAULT)
    {
        //
        // Indicate that a power fault has occurred.
        //
        g_ulUSBHIntEvents |= INT_EVENT_POWER_FAULT;

        //
        // Turn off power to the bus.
        //
        USBHostPwrDisable(USB0_BASE);

        //
        // Disable USB interrupts.
        //
        IntDisable(INT_USB0);

        return;
    }

    //
    // In the event of a USB VBUS error, end the session and remove power to
    // the device.
    //
    if(ulStatus & USB_INTCTRL_VBUS_ERR)
    {
        //
        // Set the VBUS error event.  We deliberately clear all other events
        // since this one means anything else that is outstanding is
        // irrelevant.
        //
        g_ulUSBHIntEvents = INT_EVENT_VBUS_ERR;
        return;
    }

    //
    // Received a reset from the host.
    //
    if(ulStatus & USB_INTCTRL_RESET)
    {
    }

    //
    // Suspend was signaled on the bus.
    //
    if(ulStatus & USB_INTCTRL_SUSPEND)
    {
    }

    //
    // Start the session.
    //
    if(ulStatus & USB_INTCTRL_SESSION)
    {
        //
        // Power the USB bus.
        //
        USBHostPwrEnable(USB0_BASE);

        USBOTGSessionRequest(USB0_BASE, true);
    }

    //
    // Resume was signaled on the bus.
    //
    if(ulStatus & USB_INTCTRL_RESUME)
    {
    }

    //
    // Device connected so tell the main routine to issue a reset.
    //
    if(ulStatus & USB_INTCTRL_CONNECT)
    {
        //
        // Set the connect flag and clear disconnect if it happens to be set.
        //
        g_ulUSBHIntEvents |= INT_EVENT_CONNECT;
        g_ulUSBHIntEvents &= ~INT_EVENT_DISCONNECT;

        //
        // Power the USB bus.
        //
        USBHostPwrEnable(USB0_BASE);
    }

    //
    // Device was unplugged.
    //
    if(ulStatus & USB_INTCTRL_DISCONNECT)
    {
        //
        // Set the disconnect flag and clear connect if it happens to be set.
        //
        g_ulUSBHIntEvents |= INT_EVENT_DISCONNECT;
        g_ulUSBHIntEvents &= ~INT_EVENT_CONNECT;
    }

    //
    // Start of Frame was received.
    //
    if(ulStatus & USB_INTCTRL_SOF)
    {
        //
        // Increment the global Start of Frame counter.
        //
        g_ulUSBSOFCount++;

        //
        // Increment our SOF divider.
        //
        ulSOFDivide++;

        //
        // Have we counted enough SOFs to allow us to call the tick function?
        //
        if(ulSOFDivide == USB_SOF_TICK_DIVIDE)
        {
            //
            // Yes - reset the divider and call the SOF tick handler.
            //
            ulSOFDivide = 0;
            InternalUSBStartOfFrameTick(USB_SOF_TICK_DIVIDE);
        }
    }

    //
    // Get the current endpoint interrupt status.
    //
    ulStatus = USBIntStatusEndpoint(USB0_BASE);

    //
    // Handle end point 0 interrupts.
    //
    if(ulStatus & USB_INTEP_0)
    {
        USBHCDEnumHandler();
    }

    //
    // Check to see if any uDMA transfers are pending
    //
    for(ulIdx = 0; ulIdx < MAX_NUM_PIPES; ulIdx++)
    {
        if((g_ulDMAPending == 0) && (ulStatus == 0))
        {
            break;
        }

        //
        // Check each pipe to see if uDMA is pending
        //
        if(g_ulDMAPending & (DMA_PEND_RECEIVE_FLAG << ulIdx))
        {
            //
            // Handle the case where the pipe is reading
            //
            if(g_sUSBHCD.USBINPipes[ulIdx].eState == PIPE_READING)
            {
                //
                // If the uDMA channel transfer is complete, send an ack.
                //
                if(uDMAChannelModeGet(UDMA_CHANNEL_USBEP1RX + (ulIdx * 2))
                        == UDMA_MODE_STOP)
                {
                    USBHostEndpointDataAck(USB0_BASE,
                                           INDEX_TO_USB_EP(ulIdx + 1));
                    g_ulDMAPending &= ~(DMA_PEND_RECEIVE_FLAG << ulIdx);

                    //
                    // If using uDMA then the endpoint status int will not
                    // occur.  So process the data ready event here.
                    //
                    g_sUSBHCD.USBINPipes[ulIdx].eState = PIPE_DATA_READY;
                    ulEvent = USB_EVENT_RX_AVAILABLE;

                    //
                    // Only call a handler if one is present.
                    //
                    if(g_sUSBHCD.USBINPipes[ulIdx].pfnCallback)
                    {
                        g_sUSBHCD.USBINPipes[ulIdx].pfnCallback(
                            IN_PIPE_HANDLE(ulIdx), ulEvent);
                    }
                }
            }
        }
        if(g_ulDMAPending & (DMA_PEND_TRANSMIT_FLAG << ulIdx))
        {
            //
            // Handle the case where the pipe is writing
            //
            if(g_sUSBHCD.USBOUTPipes[ulIdx].eState == PIPE_WRITING)
            {
                //
                // If the uDMA channel transfer is complete, then tell
                // the USB controller to go ahead and send the data
                //
                if(uDMAChannelModeGet(UDMA_CHANNEL_USBEP1TX + (ulIdx * 2))
                        == UDMA_MODE_STOP)
                {
                    USBEndpointDataSend(USB0_BASE,
                                        INDEX_TO_USB_EP(ulIdx + 1),
                                        USB_TRANS_OUT);
                    g_ulDMAPending &= ~(DMA_PEND_TRANSMIT_FLAG << ulIdx);
                }
            }
        }

        //
        // Check the next pipe, the first time through this will clear out
        // any interrupts dealing with endpoint zero since it was handled above.
        //
        ulStatus >>= 1;

        //
        // Check the status of the transmit(OUT) pipes.
        //
        if(ulStatus & 1)
        {
            //
            // Read the status of the endpoint connected to this pipe.
            //
            ulEPStatus = USBEndpointStatus(USB0_BASE,
                                           INDEX_TO_USB_EP(ulIdx + 1));

            //
            // Check if the device stalled the request.
            //
            if(ulEPStatus & USB_HOST_OUT_STALL)
            {
                //
                // Clear the stall condition on this endpoint pipe.
                //
                USBHostEndpointStatusClear(USB0_BASE,
                                           INDEX_TO_USB_EP(ulIdx + 1),
                                           USB_HOST_OUT_STALL);

                //
                // Save the STALLED state.
                //
                g_sUSBHCD.USBOUTPipes[ulIdx].eState = PIPE_STALLED;

                //
                // Notify the pipe that it was stalled.
                //
                ulEvent = USB_EVENT_STALL;
            }
            else
            {
                //
                // Data was transmitted successfully.
                //
                g_sUSBHCD.USBOUTPipes[ulIdx].eState = PIPE_DATA_SENT;

                //
                // Notify the pipe that its last transaction was completed.
                //
                ulEvent = USB_EVENT_TX_COMPLETE;
            }

            //
            // Only call a handler if one is present.
            //
            if(g_sUSBHCD.USBOUTPipes[ulIdx].pfnCallback)
            {
                g_sUSBHCD.USBOUTPipes[ulIdx].pfnCallback(OUT_PIPE_HANDLE(ulIdx),
                                                         ulEvent);
            }
        }

        //
        // Check the status of the receive(IN) pipes.
        //
        if(ulStatus & 0x10000)
        {
            //
            // Clear the status flag for the IN Pipe.
            //
            ulStatus &= ~0x10000;

            //
            // Read the status of the endpoint connected to this pipe.
            //
            ulEPStatus = USBEndpointStatus(USB0_BASE,
                                           INDEX_TO_USB_EP(ulIdx + 1));

            //
            // Check if the device stalled the request.
            //
            if(ulEPStatus & USB_HOST_IN_STALL)
            {
                //
                // Clear the stall condition on this endpoint pipe.
                //
                USBHostEndpointStatusClear(USB0_BASE,
                                           INDEX_TO_USB_EP(ulIdx + 1),
                                           USB_HOST_IN_STALL);

                //
                // Save the STALLED state.
                //
                g_sUSBHCD.USBINPipes[ulIdx].eState = PIPE_STALLED;

                //
                // Notify the pipe that it was stalled.
                //
                ulEvent = USB_EVENT_STALL;
            }
            else
            {
                //
                // Data is avaliable.
                //
                g_sUSBHCD.USBINPipes[ulIdx].eState = PIPE_DATA_READY;

                //
                // Notify the pipe that its last transaction was completed.
                //
                ulEvent = USB_EVENT_RX_AVAILABLE;
            }

            //
            // Only call a handler if one is present.
            //
            if(g_sUSBHCD.USBINPipes[ulIdx].pfnCallback)
            {
                g_sUSBHCD.USBINPipes[ulIdx].pfnCallback(IN_PIPE_HANDLE(ulIdx),
                                                        ulEvent);
            }
        }
    }

    //
    // If there is an active driver and it has a callback then call it.
    //
    if((g_iUSBHActiveDriver >= 0) &&
       (g_sUSBHCD.pClassDrivers[g_iUSBHActiveDriver]->pfnIntHandler))
    {
        g_sUSBHCD.pClassDrivers[g_iUSBHActiveDriver]->
            pfnIntHandler(g_pvDriverInstance);
    }
}

//*****************************************************************************
//
//! The USB host mode interrupt handler for controller index 0.
//!
//! This the main USB interrupt handler entry point.  This handler will branch
//! the interrupt off to the appropriate handlers depending on the current
//! status of the USB controller.   This function must be placed in the
//! interrupt table in order for the USB Library host stack to function.
//!
//! \return None.
//
//*****************************************************************************
void
USB0HostIntHandler(void)
{
    unsigned long ulStatus;

    //
    // Get the control interrupt status.
    //
    ulStatus = USBIntStatusControl(USB0_BASE);

    //
    // Call the internal handler to process the interrupts.
    //
    USBHostIntHandlerInternal(ulStatus);
}

//*****************************************************************************
//
//! This function opens the class driver.
//!
//! \param ulIndex specifies which USB controller to use.
//! \param ulDeviceNum is the device number for the driver to load.
//!
//! This function opens the driver needed based on the class value found in
//! the device's interface descriptor.
//!
//! \return This function returns -1 if no driver is found, or it returns the
//! index of the driver found in the list of host class drivers.
//
//*****************************************************************************
static int
USBHCDOpenDriver(unsigned long ulIndex, unsigned long ulDeviceNum)
{
    int iDriver;
    unsigned long ulClass;
    tInterfaceDescriptor *pInterface;

    ASSERT(ulIndex == 0);

    //
    // Get the interface descriptor.
    //
    pInterface = USBDescGetInterface(g_sUSBHCD.USBDevice[0].pConfigDescriptor,
                                     g_sUSBHCD.USBDevice[0].ulInterface,
                                     USB_DESC_ANY);

    //
    // Read the interface class.
    //
    ulClass = pInterface->bInterfaceClass;

    //
    // Search through the Host Class driver list for the devices class.
    //
    for(iDriver = 0; iDriver < g_sUSBHCD.ulNumClassDrivers; iDriver++)
    {
        //
        // If a driver was found call the open for this driver and save which
        // driver is in use.
        //
        if(g_sUSBHCD.pClassDrivers[iDriver]->ulInterfaceClass == ulClass)
        {
            //
            // Call the open function for the class driver.
            //
            g_pvDriverInstance = g_sUSBHCD.pClassDrivers[iDriver]->pfnOpen(
                &g_sUSBHCD.USBDevice[0]);

            //
            // If the driver was successfully loaded then break out of the
            // loop.
            //
            if(g_pvDriverInstance != 0)
            {
                break;
            }
        }
    }

    //
    // If no drivers were found then return -1 to indicate an invalid
    // driver instance.
    //
    if(iDriver == g_sUSBHCD.ulNumClassDrivers)
    {
        if((g_sUSBHCD.iEventDriver != -1) &&
           (g_sUSBHCD.pClassDrivers[g_sUSBHCD.iEventDriver]->pfnIntHandler))
        {
            //
            // Send the generic connected event.
            //
            g_sUSBHCD.EventInfo.ulEvent = USB_EVENT_CONNECTED;

            //
            // Save the class for later incase and application needs it.
            //
            g_sUSBHCD.ulClass = ulClass;
            g_sUSBHCD.EventInfo.ulInstance = (unsigned long)0x1;

            g_sUSBHCD.pClassDrivers[g_sUSBHCD.iEventDriver]->pfnIntHandler(
               &g_sUSBHCD.EventInfo);
        }

        //
        // Indicate that no driver was found.
        //
        iDriver = -1;
    }

    return(iDriver);
}

//*****************************************************************************
//
// This function handles the necessary clean up for device disconnect.
//
// \param ulIndex is the device number for the device that was disconnected.
//
// This function handles all of the necessary clean up after a device
// disconnect has been detected by the stack.  This includes calling back the
// appropriate driver if necessary.
//
// \return None.
//
//*****************************************************************************
static void
USBHCDDeviceDisconnected(unsigned long ulIndex)
{
    ASSERT(ulIndex == 0);

    if(g_sUSBHCD.USBDevice[0].pConfigDescriptor)
    {
        //
        // Invalidate the configuration descriptor.
        //
        g_sUSBHCD.USBDevice[0].pConfigDescriptor = 0;
    }

    //
    // Reset the max packet size so that this will be re-read from new devices.
    //
    g_sUSBHCD.USBDevice[0].DeviceDescriptor.bMaxPacketSize0 = 0;

    //
    // No longer have a device descriptor.
    //
    g_sUSBHCD.USBDevice[0].DeviceDescriptor.bLength = 0;

    //
    // No longer addressed.
    //
    g_sUSBHCD.USBDevice[0].ulAddress = 0;

    //
    // If this was an active driver then close it out.
    //
    if(g_iUSBHActiveDriver >= 0)
    {
        //
        // Call the driver Close entry point.
        //
        g_sUSBHCD.pClassDrivers[g_iUSBHActiveDriver]->
            pfnClose(&g_sUSBHCD.USBDevice[0]);

        //
        // No active driver now present.
        //
        g_iUSBHActiveDriver = -1;
        g_pvDriverInstance = 0;
    }
    else
    {
        if((g_sUSBHCD.iEventDriver != -1) &&
           (g_sUSBHCD.pClassDrivers[g_sUSBHCD.iEventDriver]->pfnIntHandler))
        {
            //
            // Send the generic disconnect event.
            //
            g_sUSBHCD.EventInfo.ulEvent = USB_EVENT_DISCONNECTED;

            g_sUSBHCD.pClassDrivers[g_sUSBHCD.iEventDriver]->pfnIntHandler(
               &g_sUSBHCD.EventInfo);

            //
            // Reset the class and the instance.
            //
            g_sUSBHCD.ulClass = USB_CLASS_EVENTS;
            g_sUSBHCD.EventInfo.ulInstance = 0;
        }
    }

    //
    // This call is necessary for OTG controllers to know that the host
    // stack has completed handling the disconnect of the device before
    // removing power and returning to a state that can allow OTG
    // negotiaions once again.
    //
    if(g_eUSBMode == USB_MODE_OTG)
    {
        OTGDeviceDisconnect(0);
    }
}

//*****************************************************************************
//
//! This function is the main routine for the Host Controller Driver.
//!
//! This function is the main routine for the host controller driver, and must
//! be called periodically by the main application outside of a callback
//! context.  This allows for a simple cooperative system to access the the
//! host controller driver interface without the need for an RTOS.  All time
//! critical operations are handled in interrupt context but all blocking
//! operations are run from the this function to allow them to block and wait
//! for completion without holding off other interrupts.
//!
//! \return None.
//
//*****************************************************************************
void
USBHCDMain(void)
{
    unsigned long ulIntState;
    tUSBHDeviceState eOldState;

    //
    // Save the old state to detect changes properly.
    //
    eOldState = g_sUSBHCD.eDeviceState[0];

    //
    // Fix up the state if any important interrupt events occurred.
    //
    if(g_ulUSBHIntEvents)
    {
        //
        // Perform this fixup with interrupts disabled to prevent race
        // conditions related to g_ulUSBHIntEvents.
        //
        ulIntState = IntMasterDisable();

        if(g_ulUSBHIntEvents & INT_EVENT_POWER_FAULT)
        {
            //
            // A power fault has occurred so notify the application.
            //
            if((g_sUSBHCD.iEventDriver != -1) &&
               (g_sUSBHCD.pClassDrivers[g_sUSBHCD.iEventDriver]->pfnIntHandler))
            {
                //
                // Send the generic power fault event.
                //
                g_sUSBHCD.EventInfo.ulEvent = USB_EVENT_POWER_FAULT;

                g_sUSBHCD.pClassDrivers[g_sUSBHCD.iEventDriver]->pfnIntHandler(
                   &g_sUSBHCD.EventInfo);
            }

            g_sUSBHCD.eDeviceState[0] = HCD_POWER_FAULT;
        }
        else if(g_ulUSBHIntEvents & INT_EVENT_VBUS_ERR)
        {
            //
            // A VBUS error has occurred.  This event trumps connect and
            // disconnect since it will cause a controller reset.
            //
            g_sUSBHCD.eDeviceState[0] = HCD_VBUS_ERROR;
        }
        else
        {
            //
            // Has a device connected?
            //
            if(g_ulUSBHIntEvents & INT_EVENT_CONNECT)
            {
                g_sUSBHCD.eDeviceState[0] = HCD_DEV_RESET;
            }
            else
            {
                //
                // Has a device disconnected?
                //
                if(g_ulUSBHIntEvents & INT_EVENT_DISCONNECT)
                {
                    g_sUSBHCD.eDeviceState[0] = HCD_DEV_DISCONNECTED;
                }
            }
        }

        //
        // Clear the flags.
        //
        g_ulUSBHIntEvents = 0;

        //
        // Turn interrupts back on if they were on when we were called.
        //
        if(!ulIntState)
        {
            IntMasterEnable();
        }
    }

    switch(g_sUSBHCD.eDeviceState[0])
    {
        //
        // There was a power fault condition so shut down and wait for the
        // application to re-initialized the system.
        //
        case HCD_POWER_FAULT:
        {
            break;
        }

        //
        // There was a VBUS error so handle it.
        //
        case HCD_VBUS_ERROR:
        {
            //
            // Disable USB interrupts.
            //
            IntDisable(INT_USB0);

            //
            // If there was a device in any state of connection then indicate
            // that it has been disconnected.
            //
            if((eOldState != HCD_IDLE) && (eOldState != HCD_POWER_FAULT))
            {
                //
                // Handle device disconnect.
                //
                USBHCDDeviceDisconnected(0);
            }

            //
            // Reset the controller.
            //
            SysCtlPeripheralReset(SYSCTL_PERIPH_USB0);

            //
            // Wait for 100ms before trying to re-power the device.
            //
            SysCtlDelay(g_ulTickms * 100);

            //
            // Re-initialize the HCD.
            //
            USBHCDInitInternal(0, g_sUSBHCD.pvPool, g_sUSBHCD.ulPoolSize);

            break;
        }
        //
        // Trigger a reset to the connected device.
        //
        case HCD_DEV_RESET:
        {
            //
            // Trigger a Reset.
            //
            USBHCDReset(0);

            //
            // The state moves to connected but not configured.
            //
            g_sUSBHCD.eDeviceState[0] = HCD_DEV_CONNECTED;

            break;
        }
        //
        // Device conncetion has been established now start enumerating
        // the device.
        //
        case HCD_DEV_CONNECTED:
        {
            //
            // First check if we have read the device descriptor at all
            // before proceding.
            //
            if(g_sUSBHCD.USBDevice[0].DeviceDescriptor.bLength == 0)
            {
                //
                // Change the state to issuing a device request.
                //
                g_sUSBHCD.eDeviceState[0] = HCD_DEV_REQUEST;

                //
                // Initialize a request for the device descriptor.
                //
                USBHCDGetDeviceDescriptor(0, &g_sUSBHCD.USBDevice[0]);

                //
                // Now reset again.
                //
                g_sUSBHCD.eDeviceState[0] = HCD_DEV_RESET;
            }
            //
            // If we have the device descriptor then move on to setting
            // the address of the device.
            //
            else if(g_sUSBHCD.USBDevice[0].ulAddress == 0)
            {
                //
                // Send the set address command.
                //
                USBHCDSetAddress(1);

                //
                // Save the address.
                //
                g_sUSBHCD.USBDevice[0].ulAddress = 1;

                //
                // Move on to the addressed state.
                //
                g_sUSBHCD.eDeviceState[0] = HCD_DEV_ADDRESSED;
            }
            break;
        }
        case HCD_DEV_ADDRESSED:
        {
            //
            // First check if we have read the configuration descriptor.
            //
            if (g_sUSBHCD.USBDevice[0].pConfigDescriptor == 0)
            {
                //
                // Initialize a request for the device descriptor.
                //
                USBHCDGetConfigDescriptor(0, &g_sUSBHCD.USBDevice[0]);
            }
            //
            // Now have addressed and received the device configuration,
            // so get ready to set the device configuration.
            //
            else
            {
                //
                // Use the first configuration to set the device
                // configuration.
                //
                USBHCDSetConfig(0, (unsigned long)&g_sUSBHCD.USBDevice[0], 1);

                //
                // Move on to the configured state.
                //
                g_sUSBHCD.eDeviceState[0] = HCD_DEV_CONFIGURED;

                //
                // Open the driver for device 0.
                //
                g_iUSBHActiveDriver = USBHCDOpenDriver(0, 0);
            }
            break;
        }
        //
        // The device was making a request and is now complete.
        //
        case HCD_DEV_REQUEST:
        {
            g_sUSBHCD.eDeviceState[0] = HCD_DEV_CONNECTED;
            break;
        }
        //
        // The strings are currently not accessed.
        //
        case HCD_DEV_GETSTRINGS:
        {
            break;
        }
        //
        // Basically Idle at this point.
        //
        case HCD_DEV_DISCONNECTED:
        {
            //
            // Handle device disconnect.
            //
            USBHCDDeviceDisconnected(0);

            //
            // Return to the Idle state.
            //
            g_sUSBHCD.eDeviceState[0] = HCD_IDLE;
            break;
        }

        //
        // Connection and enumeration is complete so allow this function
        // to exit.
        //
        case HCD_DEV_CONFIGURED:
        {
            break;
        }
        default:
        {
            break;
        }
    }
}

//*****************************************************************************
//
//! This function completes a control transaction to a device.
//!
//! \param ulIndex is the controller index to use for this transfer.
//! \param pSetupPacket is the setup request to be sent.
//! \param ulDevAddress is the address of the device for this request.
//! \param pData is the data to send for OUT requests or the receive buffer
//! for IN requests.
//! \param ulSize is the size of the buffer in pData.
//! \param ulMaxPacketSize is the maximum packet size for the device for this
//! request.
//!
//! This function handles the state changes necessary to send a control
//! transaction to a device.  This function should not be called from within
//! an interrupt callback as it is a blocking function.
//!
//! \return The number of bytes of data that were sent or received as a result
//! of this request.
//
//*****************************************************************************
unsigned long
USBHCDControlTransfer(unsigned long ulIndex, tUSBRequest *pSetupPacket,
                      unsigned long ulDevAddress, unsigned char *pData,
                      unsigned long ulSize, unsigned long ulMaxPacketSize)
{
    unsigned long ulRemaining;
    unsigned long ulDataSize;

    ASSERT(g_sUSBHEP0State.eState == EP0_STATE_IDLE);
    ASSERT(ulIndex == 0);

    //
    // Initialize the state of the data for this request.
    //
    g_sUSBHEP0State.pData = pData;
    g_sUSBHEP0State.ulBytesRemaining = ulSize;
    g_sUSBHEP0State.ulDataSize = ulSize;

    //
    // Set the maximum packet size.
    //
    g_sUSBHEP0State.ulMaxPacketSize = ulMaxPacketSize;

    //
    // Save the current address.
    //
    g_sUSBHEP0State.ulDevAddress = ulDevAddress;

    //
    // Set the address the host will used to communicate with the device.
    //
    USBHostAddrSet(USB0_BASE, USB_EP_0, g_sUSBHEP0State.ulDevAddress,
                   USB_EP_HOST_EP0);

    //
    // Put the data in the correct FIFO.
    //
    USBEndpointDataPut(USB0_BASE, USB_EP_0, (unsigned char *)pSetupPacket,
                       sizeof(tUSBRequest));

    //
    // If this is an IN request, change to that state.
    //
    if(pSetupPacket->bmRequestType & USB_RTYPE_DIR_IN)
    {
        g_sUSBHEP0State.eState = EP0_STATE_SETUP_IN;
    }
    else
    {
        //
        // If there is no data then this is not an OUT request.
        //
        if(ulSize != 0)
        {
            //
            // Since there is data, this is an OUT request.
            //
            g_sUSBHEP0State.eState = EP0_STATE_SETUP_OUT;
        }
        else
        {
            //
            // Otherwise this request has no data and just a status phase.
            //
            g_sUSBHEP0State.eState = EP0_STATE_STATUS_IN;
        }
    }

    //
    // Send the Setup packet.
    //
    USBEndpointDataSend(USB0_BASE, USB_EP_0, USB_TRANS_SETUP);

    //
    // Block until endpoint 0 returns to the IDLE state.
    //
    while((g_sUSBHEP0State.eState != EP0_STATE_IDLE) &&
          !(g_ulUSBHIntEvents & (INT_EVENT_VBUS_ERR | INT_EVENT_DISCONNECT)))
    {
    }

    //
    // If we aborted the transfer due to an error, tell the caller
    // that no bytes were transferred.
    //
    if(g_ulUSBHIntEvents & (INT_EVENT_VBUS_ERR | INT_EVENT_DISCONNECT))
    {
        return(0);
    }

    //
    // Calculate and return the number of bytes that were sent or received.
    // The extra copy into local variables is required to prevent some
    // compilers from warning about undefined order of volatile access.
    //
    ulDataSize = g_sUSBHEP0State.ulDataSize;
    ulRemaining = g_sUSBHEP0State.ulBytesRemaining;

    return(ulDataSize - ulRemaining);
}

//*****************************************************************************
//
// This is the endpoint 0 interrupt handler.
//
// \return None.
//
//*****************************************************************************
static void
USBHCDEnumHandler(void)
{
    unsigned long ulEPStatus;
    unsigned long ulDataSize;

    //
    // Get the end point 0 status.
    //
    ulEPStatus = USBEndpointStatus(USB0_BASE, USB_EP_0);

    switch(g_sUSBHEP0State.eState)
    {
        //
        // Handle the status state, this is a transitory state from
        // USB_STATE_TX or USB_STATE_RX back to USB_STATE_IDLE.
        //
        case EP0_STATE_STATUS:
        {
            //
            // Handle the case of a received status packet.
            //
            if(ulEPStatus & (USB_HOST_EP0_RXPKTRDY | USB_HOST_EP0_STATUS))
            {
                //
                // Clear this status indicating that the status packet was
                // received.
                //
                USBHostEndpointStatusClear(USB0_BASE, USB_EP_0,
                                           (USB_HOST_EP0_RXPKTRDY |
                                            USB_HOST_EP0_STATUS));
            }

            //
            // Just go back to the idle state.
            //
            g_sUSBHEP0State.eState = EP0_STATE_IDLE;

            break;
        }

        //
        // This state triggers a STATUS IN request from the device.
        //
        case EP0_STATE_STATUS_IN:
        {
            //
            // Generate an IN request from the device.
            //
            USBHostRequestStatus(USB0_BASE);

            //
            // Change to the status phase and wait for the response.
            //
            g_sUSBHEP0State.eState =  EP0_STATE_STATUS;

            break;
        }

        //
        // In the IDLE state the code is waiting to receive data from the host.
        //
        case EP0_STATE_IDLE:
        {
            break;
        }

        //
        // Data is still being sent to the host so handle this in the
        // EP0StateTx() function.
        //
        case EP0_STATE_SETUP_OUT:
        {
            //
            // Send remaining data if necessary.
            //
            USBHCDEP0StateTx();

            break;
        }

        //
        // Handle the receive state for commands that are receiving data on
        // endpoint 0.
        //
        case EP0_STATE_SETUP_IN:
        {
            //
            // Generate a new IN request to the device.
            //
            USBHostRequestIN(USB0_BASE, USB_EP_0);

            //
            // Proceed to the RX state to receive the requested data.
            //
            g_sUSBHEP0State.eState =  EP0_STATE_RX;

            break;
        }

        //
        // The endponit remains in this state until all requested data has
        // been received.
        //
        case EP0_STATE_RX:
        {
            //
            // There was a stall on endpoint 0 so go back to the idle state
            // as this command has been terminated.
            //
            if(ulEPStatus & USB_HOST_EP0_RX_STALL)
            {
                g_sUSBHEP0State.eState = EP0_STATE_IDLE;

                //
                // Clear the stalled state on endpoint 0.
                //
                USBHostEndpointStatusClear(USB0_BASE, USB_EP_0, ulEPStatus);
                break;
            }
            //
            // Set the number of bytes to get out of this next packet.
            //
            if(g_sUSBHEP0State.ulBytesRemaining > MAX_PACKET_SIZE_EP0)
            {
                //
                // Don't send more than EP0_MAX_PACKET_SIZE bytes.
                //
                ulDataSize = MAX_PACKET_SIZE_EP0;
            }
            else
            {
                //
                // There was space so send the remaining bytes.
                //
                ulDataSize = g_sUSBHEP0State.ulBytesRemaining;
            }

            if(ulDataSize != 0)
            {
                //
                // Get the data from the USB controller end point 0.
                //
                USBEndpointDataGet(USB0_BASE, USB_EP_0, g_sUSBHEP0State.pData,
                                   &ulDataSize);
            }

            //
            // Advance the pointer.
            //
            g_sUSBHEP0State.pData += ulDataSize;

            //
            // Decrement the number of bytes that are being waited on.
            //
            g_sUSBHEP0State.ulBytesRemaining -= ulDataSize;

            //
            // Need to ack the data on end point 0 in this case
            // without setting data end.
            //
            USBDevEndpointDataAck(USB0_BASE, USB_EP_0, false);

            //
            // If there was not more than the maximum packet size bytes of data
            // the this was a short packet and indicates that this transfer is
            // complete.  If there were exactly g_sUSBHEP0State.ulMaxPacketSize
            // remaining then there still needs to be null packet sent before
            // this transfer is complete.
            //
            if(ulDataSize < g_sUSBHEP0State.ulMaxPacketSize)
            {
                //
                // Return to the idle state.
                //
                g_sUSBHEP0State.eState =  EP0_STATE_STATUS;

                //
                // No more data.
                //
                g_sUSBHEP0State.pData = 0;

                //
                // Send a null packet to acknoledge that all data was received.
                //
                USBEndpointDataSend(USB0_BASE, USB_EP_0, USB_TRANS_STATUS);
            }
            else
            {
                //
                // Request more data.
                //
                USBHostRequestIN(USB0_BASE, USB_EP_0);
            }
            break;
        }

        //
        // The device stalled endpoint zero so check if the stall needs to be
        // cleared once it has been successfully sent.
        //
        case EP0_STATE_STALL:
        {
            //
            // Reset the global end point 0 state to IDLE.
            //
            g_sUSBHEP0State.eState = EP0_STATE_IDLE;

            break;
        }

        //
        // Halt on an unknown state, but only in DEBUG builds.
        //
        default:
        {
            ASSERT(0);
            break;
        }
    }
}

//*****************************************************************************
//
// This internal function handles sending data on endpoint 0.
//
// \return None.
//
//*****************************************************************************
static void
USBHCDEP0StateTx(void)
{
    unsigned long ulNumBytes;
    unsigned char *pData;

    //
    // In the TX state on endpoint 0.
    //
    g_sUSBHEP0State.eState = EP0_STATE_SETUP_OUT;

    //
    // Set the number of bytes to send this iteration.
    //
    ulNumBytes = g_sUSBHEP0State.ulBytesRemaining;

    //
    // Limit individual transfers to 64 bytes.
    //
    if(ulNumBytes > 64)
    {
        ulNumBytes = 64;
    }

    //
    // Save the pointer so that it can be passed to the USBEndpointDataPut()
    // function.
    //
    pData = (unsigned char *)g_sUSBHEP0State.pData;

    //
    // Advance the data pointer and counter to the next data to be sent.
    //
    g_sUSBHEP0State.ulBytesRemaining -= ulNumBytes;
    g_sUSBHEP0State.pData += ulNumBytes;

    //
    // Put the data in the correct FIFO.
    //
    USBEndpointDataPut(USB0_BASE, USB_EP_0, pData, ulNumBytes);

    //
    // If this is exactly 64 then don't set the last packet yet.
    //
    if(ulNumBytes == 64)
    {
        //
        // There is more data to send or exactly 64 bytes were sent, this
        // means that there is either more data coming or a null packet needs
        // to be sent to complete the transaction.
        //
        USBEndpointDataSend(USB0_BASE, USB_EP_0, USB_TRANS_OUT);
    }
    else
    {
        //
        // Send the last bit of data.
        //
        USBEndpointDataSend(USB0_BASE, USB_EP_0, USB_TRANS_OUT);

        //
        // Now go to the status state and wait for the transmit to complete.
        //
        g_sUSBHEP0State.eState = EP0_STATE_STATUS_IN;
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
