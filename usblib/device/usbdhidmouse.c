//*****************************************************************************
//
// usbdhidmouse.c - USB HID Mouse device class driver.
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

#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/usb.h"
#include "usblib/usblib.h"
#include "usblib/device/usbdevice.h"
#include "usblib/usbhid.h"
#include "usblib/device/usbdhid.h"
#include "usblib/device/usbdhidmouse.h"

//*****************************************************************************
//
//! \addtogroup hid_mouse_device_class_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// The report descriptor for the mouse class device.
//
//*****************************************************************************
static const unsigned char g_pucMouseReportDescriptor[]=
{
    UsagePage(USB_HID_GENERIC_DESKTOP),
    Usage(USB_HID_MOUSE),
    Collection(USB_HID_APPLICATION),
        Usage(USB_HID_POINTER),
        Collection(USB_HID_PHYSICAL),

            //
            // The buttons.
            //
            UsagePage(USB_HID_BUTTONS),
            UsageMinimum(1),
            UsageMaximum(3),
            LogicalMinimum(0),
            LogicalMaximum(1),

            //
            // 3 - 1 bit values for the buttons.
            //
            ReportSize(1),
            ReportCount(3),
            Input(USB_HID_INPUT_DATA | USB_HID_INPUT_VARIABLE |
                  USB_HID_INPUT_ABS),

            //
            // 1 - 5 bit unused constant value to fill the 8 bits.
            //
            ReportSize(5),
            ReportCount(1),
            Input(USB_HID_INPUT_CONSTANT | USB_HID_INPUT_ARRAY |
                  USB_HID_INPUT_ABS),

            //
            // The X and Y axis.
            //
            UsagePage(USB_HID_GENERIC_DESKTOP),
            Usage(USB_HID_X),
            Usage(USB_HID_Y),
            LogicalMinimum(-127),
            LogicalMaximum(127),

            //
            // 2 - 8 bit Values for x and y.
            //
            ReportSize(8),
            ReportCount(2),
            Input(USB_HID_INPUT_DATA | USB_HID_INPUT_VARIABLE |
                  USB_HID_INPUT_RELATIVE),

            //
            // 2 - 8 bit Values for x and y.  // Padding
            //
            ReportSize(8),
            ReportCount(MOUSE_REPORT_SIZE - 3),
            Input(USB_HID_INPUT_CONSTANT | USB_HID_INPUT_ARRAY |
                  USB_HID_INPUT_ABS),

        EndCollection,
    EndCollection,
};

//*****************************************************************************
//
// The HID class descriptor table.  For the mouse class, we have only a single
// report descriptor.
//
//*****************************************************************************
static const unsigned char * const g_pMouseClassDescriptors[] =
{
    g_pucMouseReportDescriptor
};

//*****************************************************************************
//
// The HID descriptor for the mouse device.
//
//*****************************************************************************
static const tHIDDescriptor g_sMouseHIDDescriptor =
{
    9,                                 // bLength
    USB_HID_DTYPE_HID,                 // bDescriptorType
    0x111,                             // bcdHID (version 1.11 compliant)
    0,                                 // bCountryCode (not localized)
    1,                                 // bNumDescriptors
    {
        {
            USB_HID_DTYPE_REPORT,                  // Report descriptor
            sizeof(g_pucMouseReportDescriptor)     // Size of report descriptor
        }
    }
};

//*****************************************************************************
//
// Forward references for mouse device callback functions.
//
//*****************************************************************************
static unsigned long HIDMouseRxHandler(void *pvCBData,
                                          unsigned long ulEvent,
                                          unsigned long ulMsgData,
                                          void *pvMsgData);
static unsigned long HIDMouseTxHandler(void *pvCBData,
                                          unsigned long ulEvent,
                                          unsigned long ulMsgData,
                                          void *pvMsgData);

//*****************************************************************************
//
// The HID device initialization and customization structures.
//
//*****************************************************************************
static tUSBDHIDDevice g_sHIDMouseDevice =
{
    0,                          // Will be filled in during USBDHIDMouseInit
    0,                          // Will be filled in during USBDHIDMouseInit
    0,                          // Will be filled in during USBDHIDMouseInit
    0,                          // Will be filled in during USBDHIDMouseInit
    USB_HID_SCLASS_BOOT,
    USB_HID_PROTOCOL_MOUSE,
    1,
    0,                          // Will be filled in during USBDHIDMouseInit
    HIDMouseRxHandler,
    0,                          // Will be filled in during USBDHIDMouseInit
    HIDMouseTxHandler,
    0,                          // Will be filled in during USBDHIDMouseInit
    false,
    &g_sMouseHIDDescriptor,
    g_pMouseClassDescriptors,
    0,                          // Will be filled in during USBDHIDMouseInit
    0,                          // Will be filled in during USBDHIDMouseInit
    0                           // Will be filled in during USBDHIDMouseInit
};

//*****************************************************************************
//
// The HID mouse report offsets for this mouse application.
//
//*****************************************************************************
#define HID_REPORT_BUTTONS      0
#define HID_REPORT_X            1
#define HID_REPORT_Y            2

//*****************************************************************************
//
// Main HID device class event handler function.
//
// \param pvCBData is the event callback pointer provided during USBDHIDInit().
// This is a pointer to our HID device structure (&g_sHIDMouseDevice).
// \param ulEvent identifies the event we are being called back for.
// \param ulMsgData is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the HID device class driver to inform the
// application of particular asynchronous events related to operation of the
// mouse HID device.
//
// \return Returns a value which is event-specific.
//
//*****************************************************************************
static unsigned long
HIDMouseRxHandler(void *pvCBData, unsigned long ulEvent,
                  unsigned long ulMsgData, void *pvMsgData)
{
    tHIDMouseInstance *psInst;
    tUSBDHIDMouseDevice *psDevice;

    //
    // Make sure we didn't get a NULL pointer.
    //
    ASSERT(pvCBData);

    //
    // Get a pointer to our instance data
    //
    psDevice = (tUSBDHIDMouseDevice *)pvCBData;
    psInst = psDevice->psPrivateHIDMouseData;

    //
    // Which event were we sent?
    //
    switch (ulEvent)
    {
        //
        // The host has connected to us and configured the device.
        //
        case USB_EVENT_CONNECTED:
        {
            psInst->ucUSBConfigured = true;

            //
            // Pass the information on to the client.
            //
            psDevice->pfnCallback(psDevice->pvCBData, USB_EVENT_CONNECTED,
                                  0, (void *)0);

            break;
        }

        //
        // The host has disconnected from us.
        //
        case USB_EVENT_DISCONNECTED:
        {
            psInst->ucUSBConfigured = false;

            //
            // Pass the information on to the client.
            //
            psDevice->pfnCallback(psDevice->pvCBData, USB_EVENT_DISCONNECTED,
                                  0, (void *)0);

            break;
        }

        //
        // The host is polling us for a particular report and the HID driver
        // is asking for the latest version to transmit.
        //
        case USBD_HID_EVENT_IDLE_TIMEOUT:
        case USBD_HID_EVENT_GET_REPORT:
        {
            //
            // We only support a single input report so we don't need to check
            // the ulMsgValue parameter in this case.  Set the report pointer
            // in *pvMsgData and return the length of the report in bytes.
            //
            *(unsigned char **)pvMsgData = psInst->pucReport;
            return (8);
        }

        //
        // The device class driver has completed sending a report to the
        // host in response to a Get_Report request.
        //
        case USBD_HID_EVENT_REPORT_SENT:
        {
            //
            // We have nothing to do here.
            //
            break;
        }

        //
        // This event is sent in response to a host Set_Report request.  The
        // mouse device has no output reports so we return a NULL pointer and
        // zero length to cause this request to be stalled.
        //
        case USBD_HID_EVENT_GET_REPORT_BUFFER:
        {
            //
            // We are being asked for a report that does not exist for
            // this device.
            //
            *(unsigned char **)pvMsgData = (void *)0;
            return (0);
        }

        //
        // The host is asking us to set either boot or report protocol (not
        // that it makes any difference to this particular mouse).
        //
        case USBD_HID_EVENT_SET_PROTOCOL:
        {
            psInst->ucProtocol = ulMsgData;
            break;
        }

        //
        // The host is asking us to tell it which protocol we are currently
        // using, boot or request.
        //
        case USBD_HID_EVENT_GET_PROTOCOL:
        {
            return (psInst->ucProtocol);
        }

        //
        // Pass ERROR, SUSPEND and RESUME to the client unchanged.
        //
        case USB_EVENT_ERROR:
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
        {
            return(psDevice->pfnCallback(psDevice->pvCBData, ulEvent,
                                         ulMsgData, pvMsgData));
        }

        //
        // We ignore all other events.
        //
        default:
        {
            break;
        }
    }
    return (0);
}

//*****************************************************************************
//
// HID device class transmit channel event handler function.
//
// \param pvCBData is the event callback pointer provided during USBDHIDInit().
// This is a pointer to our HID device structure (&g_sHIDMouseDevice).
// \param ulEvent identifies the event we are being called back for.
// \param ulMsgData is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the HID device class driver to inform the
// application of particular asynchronous events related to report
// transmissions made using the interrupt IN endpoint.
//
// \return Returns a value which is event-specific.
//
//*****************************************************************************
static unsigned long
HIDMouseTxHandler(void *pvCBData, unsigned long ulEvent,
                  unsigned long ulMsgData, void *pvMsgData)
{
    tHIDMouseInstance *psInst;
    tUSBDHIDMouseDevice *psDevice;

    //
    // Make sure we didn't get a NULL pointer.
    //
    ASSERT(pvCBData);

    //
    // Get a pointer to our instance data
    //
    psDevice = (tUSBDHIDMouseDevice *)pvCBData;
    psInst = psDevice->psPrivateHIDMouseData;

    //
    // Which event were we sent?
    //
    switch (ulEvent)
    {
        //
        // A report transmitted via the interrupt IN endpoint was acknowledged
        // by the host.
        //
        case USB_EVENT_TX_COMPLETE:
        {
            //
            // Our last transmission is complete.
            //
            psInst->eMouseState = HID_MOUSE_STATE_IDLE;

            //
            // Pass the event on to the client.
            //
            psDevice->pfnCallback(psDevice->pvCBData, USB_EVENT_TX_COMPLETE,
                                  ulMsgData, (void *)0);

            break;
        }

        //
        // We ignore all other events related to transmission of reports via
        // the interrupt IN endpoint.
        //
        default:
        {
            break;
        }
    }

    return (0);
}

//*****************************************************************************
//
//! Initializes HID mouse device operation for a given USB controller.
//!
//! \param ulIndex is the index of the USB controller which is to be
//! initialized for HID mouse device operation.
//! \param psDevice points to a structure containing parameters customizing
//! the operation of the HID mouse device.
//!
//! An application wishing to offer a USB HID mouse interface to a USB host
//! must call this function to initialize the USB controller and attach the
//! mouse device to the USB bus.  This function performs all required USB
//! initialization.
//!
//! On successful completion, this function will return the \e psDevice pointer
//! passed to it.  This must be passed on all future calls to the HID mouse
//! device driver.
//!
//! When a host connects and configures the device, the application callback
//! will receive \b USB_EVENT_CONNECTED after which calls can be made to
//! USBDHIDMouseStateChange() to report pointer movement and button presses
//! to the host.
//!
//! \note The application must not make any calls to the lower level USB device
//! interfaces if interacting with USB via the USB HID mouse device API.
//! Doing so will cause unpredictable (though almost certainly unpleasant)
//! behavior.
//!
//! \return Returns NULL on failure or the psDevice pointer on success.
//
//*****************************************************************************
void *
USBDHIDMouseInit(unsigned long ulIndex, const tUSBDHIDMouseDevice *psDevice)
{
    tHIDMouseInstance *psInst;
    void *pvRetcode;

    //
    // Check parameter validity.
    //
    ASSERT(psDevice);
    ASSERT(psDevice->ppStringDescriptors);
    ASSERT(psDevice->psPrivateHIDMouseData);
    ASSERT(psDevice->pfnCallback);

    //
    // Get a pointer to our instance data
    //
    psInst = psDevice->psPrivateHIDMouseData;

    //
    // Initialize the various fields in our instance structure.
    //
    psInst->ucUSBConfigured = 0;
    psInst->ucProtocol = USB_HID_PROTOCOL_REPORT;
    psInst->sReportIdle.ucDuration4mS = 0;
    psInst->sReportIdle.ucReportID = 0;
    psInst->sReportIdle.ulTimeSinceReportmS = 0;
    psInst->sReportIdle.usTimeTillNextmS = 0;
    psInst->eMouseState = HID_MOUSE_STATE_UNCONFIGURED;

    //
    // Initialize the HID device class instance structure based on input from
    // the caller.
    //
    g_sHIDMouseDevice.usPID = psDevice->usPID;
    g_sHIDMouseDevice.usVID = psDevice->usVID;
    g_sHIDMouseDevice.usMaxPowermA = psDevice->usMaxPowermA;
    g_sHIDMouseDevice.ucPwrAttributes = psDevice->ucPwrAttributes;
    g_sHIDMouseDevice.ppStringDescriptors = psDevice->ppStringDescriptors;
    g_sHIDMouseDevice.ulNumStringDescriptors =
        psDevice->ulNumStringDescriptors;
    g_sHIDMouseDevice.psPrivateHIDData = &psInst->sHIDInstance;
    g_sHIDMouseDevice.psReportIdle = &psInst->sReportIdle;
    g_sHIDMouseDevice.pvRxCBData = (void *)psDevice;
    g_sHIDMouseDevice.pvTxCBData = (void *)psDevice;

    //
    // Initialize the lower layer HID driver and pass it the various structures
    // and descriptors necessary to declare that we are a mouse.
    //
    pvRetcode = USBDHIDInit(ulIndex, &g_sHIDMouseDevice);

    //
    // If we initialized the HID layer successfully, pass our device pointer
    // back as the return code, otherwise return NULL to indicate an error.
    //
    if(pvRetcode)
    {
        return((void *)psDevice);
    }
    else
    {
        return((void *)0);
    }
}

//*****************************************************************************
//
//! Shuts down the HID mouse device.
//!
//! \param psDevice is the pointer to the device instance structure.
//!
//! This function terminates HID mouse operation for the instance supplied
//! and removes the device from the USB bus.  Following this call, the \e
//! psDevice instance may not me used in any other call to the HID mouse
//! device other than USBDHIDMouseInit().
//!
//! \return None.
//
//*****************************************************************************
void
USBDHIDMouseTerm(void *psDevice)
{
    tHIDMouseInstance *psInst;

    ASSERT(psDevice);

    //
    // Get a pointer to our instance data
    //
    psInst = (((tUSBDHIDMouseDevice *)psDevice)->psPrivateHIDMouseData);

    //
    // Mark our device as unconfigured.
    //
    psInst->ucUSBConfigured = 0;

    //
    // Terminate the low level HID driver.
    //
    USBDHIDTerm(&g_sHIDMouseDevice);
}

//*****************************************************************************
//
//! Sets the client-specific pointer parameter for the mouse callback.
//!
//! \param psDevice is the pointer to the mouse device instance structure.
//! \param pvCBData is the pointer that client wishes to be provided on each
//! event sent to the mouse callback function.
//!
//! The client uses this function to change the callback pointer passed in
//! the first parameter on all callbacks to the \e pfnCallback function
//! passed on USBDHIDMouseInit().
//!
//! If a client wants to make runtime changes in the callback pointer, it must
//! ensure that the psDevice structure passed to USBDHIDMouseInit() resides
//! in RAM.  If this structure is in flash, callback data changes will not be
//! possible.
//!
//! \return Returns the previous callback pointer that was set for this
//! instance.
//
//*****************************************************************************
void *
USBDHIDMouseSetCBData(void *psDevice, void *pvCBData)
{
    void *pvOldCBData;
    tUSBDHIDMouseDevice *psMouse;

    //
    // Check for a NULL pointer in the device parameter.
    //
    ASSERT(psDevice);

    //
    // Get a pointer to our mouse device.
    //
    psMouse = (tUSBDHIDMouseDevice *)psDevice;

    //
    // Save the old callback pointer and replace it with the new value.
    //
    pvOldCBData = psMouse->pvCBData;
    psMouse->pvCBData = pvCBData;

    //
    // Pass the old callback pointer back to the caller.
    //
    return(pvOldCBData);
}

//*****************************************************************************
//
//! Reports a mouse state change, pointer movement or button press, to the USB
//! host.
//!
//! \param psDevice is the pointer to the mouse device instance structure.
//! \param cDeltaX is the relative horizontal pointer movement that the
//! application wishes to report.  Valid values are in the range [-127, 127]
//! with positive values indicating movement to the right.
//! \param cDeltaY is the relative vertical pointer movement that the
//! application wishes to report.  Valid values are in the range [-127, 127]
//! with positive values indicating downward movement.
//! \param ucButtons is a bit mask indicating which (if any) of the three
//! mouse buttons is pressed.  Valid values are logical OR combinations of
//! \e MOUSE_REPORT_BUTTON_1, \e MOUSE_REPORT_BUTTON_2 and \e
//! MOUSE_REPORT_BUTTON_3.
//!
//! This function is called to report changes in the mouse state to the USB
//! host.  These changes can be movement of the pointer, reported relative to
//! its previous position, or changes in the states of up to 3 buttons that
//! the mouse may support.  The return code indicates whether or not the
//! mouse report could be sent to the host.  In cases where a previous
//! report is still being transmitted, \b MOUSE_ERR_TX_ERROR will be returned
//! and the state change will be ignored.
//!
//! \return Returns \b MOUSE_SUCCESS on success, \b MOUSE_ERR_TX_ERROR if an
//! error occurred while attempting to schedule transmission of the mouse
//! report to the host (typically due to a previous report which has not yet
//! completed transmission or due to disconnection of the host) or \b
//! MOUSE_ERR_NOT_CONFIGURED if called before a host has connected to and
//! configured the device.
//
//*****************************************************************************
unsigned long
USBDHIDMouseStateChange(void *psDevice, char cDeltaX, char cDeltaY,
                        unsigned char ucButtons)
{
    unsigned long ulRetcode;
    unsigned long ulCount;
    tHIDMouseInstance *psInst;

    //
    // Get a pointer to our instance data
    //
    psInst = (((tUSBDHIDMouseDevice *)psDevice)->psPrivateHIDMouseData);

    //
    // Update the global mouse report with the information passed.
    //
    psInst->pucReport[HID_REPORT_BUTTONS] = ucButtons;
    psInst->pucReport[HID_REPORT_X] = (unsigned char)cDeltaX;
    psInst->pucReport[HID_REPORT_Y] = (unsigned char)cDeltaY;

    //
    // If we are not configured, return an error here before trying to send
    // anything.
    //
    if(!psInst->ucUSBConfigured)
    {
        return(MOUSE_ERR_NOT_CONFIGURED);
    }

    //
    // Only send a report if the transmitter is currently free.
    //
    if(USBDHIDTxPacketAvailable((void *)&g_sHIDMouseDevice))
    {
        //
        // Send the report to the host.
        //
        psInst->eMouseState = HID_MOUSE_STATE_SEND;
        ulCount = USBDHIDReportWrite((void *)&g_sHIDMouseDevice,
                                     psInst->pucReport, MOUSE_REPORT_SIZE,
                                     true);

        //
        // Did we schedule a packet for transmission correctly?
        //
        if(!ulCount)
        {
            //
            // No - report the error to the caller.
            //
            ulRetcode = MOUSE_ERR_TX_ERROR;
        }
        else
        {
            ulRetcode = MOUSE_SUCCESS;
        }
    }
    else
    {
        ulRetcode = MOUSE_ERR_TX_ERROR;
    }
    //
    // Return the relevant error code to the caller.
    //
    return(ulRetcode);
}

//*****************************************************************************
//
//! Reports the device power status (bus- or self-powered) to the USB library.
//!
//! \param psDevice is the pointer to the mouse device instance structure.
//! \param ucPower indicates the current power status, either \b
//! USB_STATUS_SELF_PWR or \b USB_STATUS_BUS_PWR.
//!
//! Applications which support switching between bus- or self-powered
//! operation should call this function whenever the power source changes
//! to indicate the current power status to the USB library.  This information
//! is required by the USB library to allow correct responses to be provided
//! when the host requests status from the device.
//!
//! \return None.
//
//*****************************************************************************
void
USBDHIDMousePowerStatusSet(void *psDevice, unsigned char ucPower)
{
    ASSERT(psDevice);

    //
    // Pass the request through to the lower layer.
    //
    USBDHIDPowerStatusSet((void *)&g_sHIDMouseDevice, ucPower);
}

//*****************************************************************************
//
//! Requests a remote wakeup to resume communication when in suspended state.
//!
//! \param psDevice is the pointer to the mouse device instance structure.
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
USBDHIDMouseRemoteWakeupRequest(void *psDevice)
{
    ASSERT(psDevice);

    //
    // Pass the request through to the lower layer.
    //
    return(USBDHIDRemoteWakeupRequest((void *)&g_sHIDMouseDevice));
}
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
