//*****************************************************************************
//
// usb_dev_serial.c - Main routines for the USB CDC serial example.
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

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/usb.h"
#include "grlib/grlib.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#ifdef DEBUG
#include "utils/uartstdio.h"
#endif
#include "utils/ustdlib.h"
#include "drivers/formike128x128x16.h"
#include "usb_serial_structs.h"
#include "usb_serial_structs.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>USB Serial Device (usb_dev_serial)</h1>
//!
//! This example application turns the evaluation kit into a virtual serial
//! port when connected to the USB host system.  The application supports the
//! USB Communication Device Class, Abstract Control Model to redirect UART0
//! traffic to and from the USB host system.  File usb_dev_serial_win2k.inf may
//! be used to install the example as a virtual COM port on a Windows2000
//! system.  For WindowsXP or Vista, usb_dev_serial.inf should be used.
//
//*****************************************************************************

//*****************************************************************************
//
// Note:
//
// This example is intended to run on Luminary Micro evaluation kit hardware
// where the UARTs are wired solely for TX and RX, and do not have GPIOs
// connected to act as handshake signals.  As a result, this example mimics
// the case where communication is always possible.  It reports DSR, DCD
// and CTS as high to ensure that the USB host recognizes that data can be
// sent and merely ignores the host's requested DTR and RTS states.  "TODO"
// comments in the code indicate where code would be required to add support
// for real handshakes.
//
//*****************************************************************************

//*****************************************************************************
//
// Configuration and tuning parameters.
//
//*****************************************************************************

//*****************************************************************************
//
// The system tick rate expressed both as ticks per second and a millisecond
// period.
//
//*****************************************************************************
#define SYSTICKS_PER_SECOND 100
#define SYSTICK_PERIOD_MS (1000 / SYSTICKS_PER_SECOND)

//*****************************************************************************
//
// If you have a board configured such that UART1 is available, you can define
// the following to redirect UART1 to USB and free up UART0 for debug output.
// On an unmodified evaluation board, however, this is not possible.
//
//*****************************************************************************
//#define REDIRECT_UART1

//*****************************************************************************
//
// USB mux GPIO definitions.
//
//*****************************************************************************
#define USB_MUX_GPIO_PERIPH     SYSCTL_PERIPH_GPIOH
#define USB_MUX_GPIO_BASE       GPIO_PORTH_BASE
#define USB_MUX_GPIO_PIN        GPIO_PIN_2
#define USB_MUX_SEL_DEVICE      USB_MUX_GPIO_PIN


//*****************************************************************************
//
// Variables tracking transmit and receive counts.
//
//*****************************************************************************
volatile unsigned long g_ulUARTTxCount = 0;
volatile unsigned long g_ulUARTRxCount = 0;
#ifdef DEBUG
unsigned long g_ulUARTRxErrors = 0;
#endif

//*****************************************************************************
//
// The base address, peripheral ID and interrupt ID of the UART that is to
// be redirected.
//
//*****************************************************************************
#ifdef REDIRECT_UART1
//*****************************************************************************
//
// Defines required to redirect UART1 via USB.  This frees UART0 for debug
// information but would require some board additions to get U1TX and U1RX to
// a suitable serial connector.
//
//*****************************************************************************
#define USB_UART_BASE           UART1_BASE
#define USB_UART_PERIPH         SYSCTL_PERIPH_UART1
#define USB_UART_INT            INT_UART1
#else
//*****************************************************************************
//
// Defines required to redirect UART0 via USB.
//
//*****************************************************************************
#define USB_UART_BASE           UART0_BASE
#define USB_UART_PERIPH         SYSCTL_PERIPH_UART0
#define USB_UART_INT            INT_UART0
#endif

//*****************************************************************************
//
// Default line coding settings for the redirected UART.
//
//*****************************************************************************
#define DEFAULT_BIT_RATE        115200
#define DEFAULT_UART_CONFIG     (UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | \
                                 UART_CONFIG_STOP_ONE)

//*****************************************************************************
//
// GPIO peripherals and pins muxed with the redirected UART.  These will depend
// upon the IC in use and the UART selected in USB_UART_BASE.  Be careful that
// these settings all agree with the hardware you are using.
//
//*****************************************************************************
#ifdef REDIRECT_UART1
//*****************************************************************************
//
// Defines required to redirect UART1 via USB.
//
//*****************************************************************************
#define TX_GPIO_BASE            GPIO_PORTD_BASE
#define TX_GPIO_PERIPH          SYSCTL_PERIPH_GPIOD
#define TX_GPIO_PIN             GPIO_PIN_3

#define RX_GPIO_BASE            GPIO_PORTD_BASE
#define RX_GPIO_PERIPH          SYSCTL_PERIPH_GPIOD
#define RX_GPIO_PIN             GPIO_PIN_2
#else
//*****************************************************************************
//
// Defines required to redirect UART0 via USB.
//
//*****************************************************************************
#define TX_GPIO_BASE            GPIO_PORTA_BASE
#define TX_GPIO_PERIPH          SYSCTL_PERIPH_GPIOA
#define TX_GPIO_PIN             GPIO_PIN_1

#define RX_GPIO_BASE            GPIO_PORTA_BASE
#define RX_GPIO_PERIPH          SYSCTL_PERIPH_GPIOA
#define RX_GPIO_PIN             GPIO_PIN_0
#endif

//*****************************************************************************
//
// Debug-related definitions and declarations.
//
// Debug output is available via UART0 if flag DEBUG is defined in the build
// and we are not using UART0 as the UART that is being redirected via USB.
//
//*****************************************************************************
#if (defined DEBUG) && (USB_UART_BASE != UART0_BASE)
//*****************************************************************************
//
// Map all debug print calls to UARTprintf in debug builds.
//
//*****************************************************************************
#define DEBUG_PRINT             UARTprintf

//*****************************************************************************
//
// Global file handle for the debug printing.
//
//*****************************************************************************
int g_iHandle;

//*****************************************************************************
//
// Define the following if you want to echo characters passed through the
// USB-redirected UART to the debug port too.  This is not recommended if you
// intend to pass lots of data through the device! Use only for debug purposes.
//
//*****************************************************************************
//#define ECHO_TO_DEBUG

#else

//*****************************************************************************
//
// Compile out all debug print calls in release builds.
//
//*****************************************************************************
#define DEBUG_PRINT while(0) ((int (*)(char *, ...))0)
#define g_iHandle 0
#endif

//*****************************************************************************
//
// Flag indicating whether or not we are currently sending a Break condition.
//
//*****************************************************************************
static tBoolean g_bSendingBreak = false;

//*****************************************************************************
//
// Global system tick counter
//
//*****************************************************************************
volatile unsigned long g_ulSysTickCount = 0;

//*****************************************************************************
//
// Graphics context used to show text on the color STN display.
//
//*****************************************************************************
tContext g_sContext;
#define TEXT_FONT               &g_sFontFixed6x8
#define TEXT_HEIGHT             8
#define BUFFER_METER_HEIGHT     12
#define BUFFER_METER_WIDTH      52

//*****************************************************************************
//
// Flags used to pass commands from interrupt context to the main loop.
//
//*****************************************************************************
#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002

volatile unsigned long g_ulFlags = 0;
char *g_pcStatus;

//*****************************************************************************
//
// Global flag indicating that a USB configuration has been set.
//
//*****************************************************************************
static volatile tBoolean g_bUSBConfigured = false;

//*****************************************************************************
//
// Internal function prototypes.
//
//*****************************************************************************
static void USBUARTPrimeTransmit(unsigned long ulBase);
static void CheckForSerialStateChange(const tUSBDCDCDevice *psDevice, long lErrors);
static void SetControlLineState(unsigned short usState);
static tBoolean SetLineCoding(tLineCoding *psLineCoding);
static void GetLineCoding(tLineCoding *psLineCoding);
static void SendBreak(tBoolean bSend);

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
    while(1)
    {
    }
}
#endif

//*****************************************************************************
//
// This function is called whenever serial data is received from the UART.
// It is passed the accumulated error flags from each character received in
// this interrupt and determines from them whether or not an interrupt
// notification to the host is required.
//
// If a notification is required and the control interrupt endpoint is idle,
// we send the notification immediately.  If the endpoint is not idle, we
// accumulate the errors in a global variable which will be checked on
// completion of the previous notification and used to send a second one
// if necessary.
//
//*****************************************************************************
static void
CheckForSerialStateChange(const tUSBDCDCDevice *psDevice, long lErrors)
{
    unsigned short usSerialState;

    //
    // Clear our USB serial state.  Since we are faking the handshakes, always
    // set the TXCARRIER (DSR) and RXCARRIER (DCD) bits.
    //
    usSerialState = USB_CDC_SERIAL_STATE_TXCARRIER |
                    USB_CDC_SERIAL_STATE_RXCARRIER;

    //
    // Are any error bits set?
    //
    if(lErrors)
    {
        //
        // At least one error is being notified so translate from our hardware
        // error bits into the correct state markers for the USB notification.
        //
        if(lErrors & UART_DR_OE)
        {
            usSerialState |= USB_CDC_SERIAL_STATE_OVERRUN;
        }

        if(lErrors & UART_DR_PE)
        {
            usSerialState |= USB_CDC_SERIAL_STATE_PARITY;
        }

        if(lErrors & UART_DR_FE)
        {
            usSerialState |= USB_CDC_SERIAL_STATE_FRAMING;
        }

        if(lErrors & UART_DR_BE)
        {
            usSerialState |= USB_CDC_SERIAL_STATE_BREAK;
        }

        // Call the CDC driver to notify the state change.
        USBDCDCSerialStateChange((void *)psDevice, usSerialState);
    }
}

//*****************************************************************************
//
// Read as many characters from the UART FIFO as we can and move them into
// the CDC transmit buffer.
//
// \return Returns UART error flags read during data reception.
//
//*****************************************************************************
static long
ReadUARTData(void)
{
    long lChar, lErrors;
    unsigned char ucChar;
    unsigned long ulSpace;

    //
    // Clear our error indicator.
    //
    lErrors = 0;

    //
    // How much space do we have in the buffer?
    //
    ulSpace = USBBufferSpaceAvailable((tUSBBuffer *)&g_sTxBuffer);

    //
    // Read data from the UART FIFO until there is none left or we run
    // out of space in our receive buffer.
    //
    while(ulSpace && UARTCharsAvail(USB_UART_BASE))
    {
        //
        // Read a character from the UART FIFO into the ring buffer if no
        // errors are reported.
        //
        lChar = UARTCharGetNonBlocking(USB_UART_BASE);

        //
        // If the character did not contain any error notifications,
        // copy it to the output buffer.
        //
        if(!(lChar & ~0xFF))
        {
            ucChar = (unsigned char)(lChar & 0xFF);
            USBBufferWrite((tUSBBuffer *)&g_sTxBuffer,
                           (unsigned char *)&ucChar, 1);

            //
            // Decrement the number of bytes we know the buffer can accept.
            //
            ulSpace--;

#ifdef ECHO_TO_DEBUG
            //
            // If configured to do this, echo the received character to the
            // debug port.
            //
            DEBUG_PRINT("RX: %c\n",
                        g_pcUSBTxBuffer[g_ulUARTRxWriteIndex]);
#endif
        }
        else
        {
#ifdef DEBUG
            //
            // Increment our receive error counter.
            //
            g_ulUARTRxErrors++;
#endif
            //
            // Update our error accumulator.
            //
            lErrors |= lChar;

#ifdef ECHO_TO_DEBUG
            //
            // If configured to do this, echo the received character to the
            // debug port.
            //
            DEBUG_PRINT("RX: %c ERRORs 0x%x\n",
                        (char)lChar, (lChar & 0xF00));
#endif
        }

        //
        // Update our count of bytes received via the UART.
        //
        g_ulUARTRxCount++;
    }

    //
    // Pass back the accumulated error indicators.
    //
    return(lErrors);
}

//*****************************************************************************
//
// Take as many bytes from the transmit buffer as we have space for and move
// them into the USB UART's transmit FIFO.
//
//*****************************************************************************
static void
USBUARTPrimeTransmit(unsigned long ulBase)
{
    unsigned long ulRead;
    unsigned char ucChar;

    //
    // If we are currently sending a break condition, don't receive any
    // more data. We will resume transmission once the break is turned off.
    //
    if(g_bSendingBreak)
    {
        return;
    }

    //
    // If there is space in the UART FIFO, try to read some characters
    // from the receive buffer to fill it again.
    //
    while(UARTSpaceAvail(ulBase))
    {
        //
        // Get a character from the buffer.
        //
        ulRead = USBBufferRead((tUSBBuffer *)&g_sRxBuffer, &ucChar, 1);

        //
        // Did we get a character?
        //
        if(ulRead)
        {
            //
            // Place the character in the UART transmit FIFO.
            //
            UARTCharPutNonBlocking(ulBase, ucChar);

            //
            // If configured to do this, echo the character to the
            // debug port.
            //
#ifdef ECHO_TO_DEBUG
            DEBUG_PRINT("TX: %c\n", ucChar);
#endif

            //
            // Update our count of bytes transmitted via the UART.
            //
            g_ulUARTTxCount++;
        }
        else
        {
            //
            // We ran out of characters so exit the function.
            //
            return;
        }
    }
}

//*****************************************************************************
//
// Interrupt handler for the system tick counter.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Update our system time.
    //
    g_ulSysTickCount++;
}

//*****************************************************************************
//
// Interrupt handler for the UART which we are redirecting via USB.
//
//*****************************************************************************
void
USBUARTIntHandler(void)
{
    unsigned long ulInts;
    long lErrors;

    //
    // Get and clear the current interrupt source(s)
    //
    ulInts = UARTIntStatus(USB_UART_BASE, true);
    UARTIntClear(USB_UART_BASE, ulInts);

    //
    // Are we being interrupted because the TX FIFO has space available?
    //
    if(ulInts & UART_INT_TX)
    {
        //
        // Move as many bytes as we can into the transmit FIFO.
        //
        USBUARTPrimeTransmit(USB_UART_BASE);

        //
        // If the output buffer is empty, turn off the transmit interrupt.
        //
        if(!USBBufferDataAvailable(&g_sRxBuffer))
        {
            UARTIntDisable(USB_UART_BASE, UART_INT_TX);
        }
    }

    //
    // Handle receive interrupts.
    //
    if(ulInts & (UART_INT_RX | UART_INT_RT))
    {
        //
        // Read the UART's characters into the buffer.
        //
        lErrors = ReadUARTData();

        //
        // Check to see if we need to notify the host of any errors we just
        // detected.
        //
        CheckForSerialStateChange(&g_sCDCDevice, lErrors);
    }
}

//*****************************************************************************
//
// Set the state of the RS232 RTS and DTR signals.
//
//*****************************************************************************
static void
SetControlLineState(unsigned short usState)
{
    //
    // TODO: If configured with GPIOs controlling the handshake lines,
    // set them appropriately depending upon the flags passed in the wValue
    // field of the request structure passed.
    //
    // Since we currently don't support handshaking, merely dump a debug
    // message and ignore this request.
    //
    DEBUG_PRINT("RTS %s, DTR %s\n",
                (usState & USB_CDC_ACTIVATE_CARRIER) ?
                "set" : "clear",
                (usState & USB_CDC_DTE_PRESENT) ?
                "set" : "clear");
}

//*****************************************************************************
//
// Set the communication parameters to use on the UART.
//
//*****************************************************************************
static tBoolean
SetLineCoding(tLineCoding *psLineCoding)
{
    unsigned long ulConfig;
    tBoolean bRetcode;

    //
    // Assume everything is OK until we detect any problem.
    //
    bRetcode = true;

    DEBUG_PRINT("Setting line coding %dbps, %d\\",
                psLineCoding->ulRate, psLineCoding->ucDatabits);

    //
    // Word length.  For invalid values, the default is to set 8 bits per
    // character and return an error.
    //
    switch(psLineCoding->ucDatabits)
    {
        case 5:
        {
            ulConfig = UART_CONFIG_WLEN_5;
            break;
        }

        case 6:
        {
            ulConfig = UART_CONFIG_WLEN_6;
            break;
        }

        case 7:
        {
            ulConfig = UART_CONFIG_WLEN_7;
            break;
        }

        case 8:
        {
            ulConfig = UART_CONFIG_WLEN_8;
            break;
        }

        default:
        {
            ulConfig = UART_CONFIG_WLEN_8;
            bRetcode = false;
            break;
        }
    }

    //
    // Parity.  For any invalid values, we set no parity and return an error.
    //
    switch(psLineCoding->ucParity)
    {
        case USB_CDC_PARITY_NONE:
        {
            DEBUG_PRINT("N\\");
            ulConfig |= UART_CONFIG_PAR_NONE;
            break;
        }

        case USB_CDC_PARITY_ODD:
        {
            DEBUG_PRINT("O\\");
            ulConfig |= UART_CONFIG_PAR_ODD;
            break;
        }

        case USB_CDC_PARITY_EVEN:
        {
            DEBUG_PRINT("E\\");
            ulConfig |= UART_CONFIG_PAR_EVEN;
            break;
        }

        case USB_CDC_PARITY_MARK:
        {
            DEBUG_PRINT("M\\");
            ulConfig |= UART_CONFIG_PAR_ONE;
            break;
        }

        case USB_CDC_PARITY_SPACE:
        {
            DEBUG_PRINT("S\\");
            ulConfig |= UART_CONFIG_PAR_ZERO;
            break;
        }

        default:
        {
            DEBUG_PRINT("N(%d)\\", psLineCoding->ucParity);
            ulConfig |= UART_CONFIG_PAR_NONE;
            bRetcode = false;
            break;
        }
    }

    //
    // Stop bits.  Our hardware only supports 1 or 2 stop bits whereas CDC
    // allows the host to select 1.5 stop bits.  If passed 1.5 (or any other
    // invalid or unsupported value of ucStop, we set up for 1 stop bit but
    // return an error in case the caller needs to Stall or otherwise report
    // this back to the host.
    //
    switch(psLineCoding->ucStop)
    {
        //
        // One stop bit requested.
        //
        case USB_CDC_STOP_BITS_1:
        {
            DEBUG_PRINT("1 ");
            ulConfig |= UART_CONFIG_STOP_ONE;
            break;
        }

        //
        // Two stop bits requested.
        //
        case USB_CDC_STOP_BITS_2:
        {
            DEBUG_PRINT("2");
            ulConfig |= UART_CONFIG_STOP_TWO;
            break;
        }

        //
        // Other cases are either invalid values of ucStop or values that we
        // cannot support so set 1 stop bit but return an error.
        //
        default:
        {
            DEBUG_PRINT("1(%d)", psLineCoding->ucStop);
            ulConfig = UART_CONFIG_STOP_ONE;
            bRetcode |= false;
            break;
        }
    }

    DEBUG_PRINT(" %s\n", bRetcode ? "" : "(Error)");

    //
    // Set the UART mode appropriately.
    //
    UARTConfigSetExpClk(USB_UART_BASE, SysCtlClockGet(), psLineCoding->ulRate,
                        ulConfig);

    //
    // Let the caller know if we had a problem or not.
    //
    return(bRetcode);
}

//*****************************************************************************
//
// Get the communication parameters in use on the UART.
//
//*****************************************************************************
static void
GetLineCoding(tLineCoding *psLineCoding)
{
    unsigned long ulConfig;
    unsigned long ulRate;

    //
    // Get the current line coding set in the UART.
    //
    UARTConfigGetExpClk(USB_UART_BASE, SysCtlClockGet(), &ulRate,
                        &ulConfig);
    psLineCoding->ulRate = ulRate;

    DEBUG_PRINT("Current line coding %d, ", psLineCoding->ulRate);

    //
    // Translate the configuration word length field into the format expected
    // by the host.
    //
    switch(ulConfig & UART_CONFIG_WLEN_MASK)
    {
        case UART_CONFIG_WLEN_8:
        {
            DEBUG_PRINT("8\\");
            psLineCoding->ucDatabits = 8;
            break;
        }

        case UART_CONFIG_WLEN_7:
        {
            DEBUG_PRINT("7\\");
            psLineCoding->ucDatabits = 7;
            break;
        }

        case UART_CONFIG_WLEN_6:
        {
            DEBUG_PRINT("6\\");
            psLineCoding->ucDatabits = 6;
            break;
        }

        case UART_CONFIG_WLEN_5:
        {
            DEBUG_PRINT("5\\");
            psLineCoding->ucDatabits = 5;
            break;
        }
    }

    //
    // Translate the configuration parity field into the format expected
    // by the host.
    //
    switch(ulConfig & UART_CONFIG_PAR_MASK)
    {
        case UART_CONFIG_PAR_NONE:
        {
            DEBUG_PRINT("N\\");
            psLineCoding->ucParity = USB_CDC_PARITY_NONE;
            break;
        }

        case UART_CONFIG_PAR_ODD:
        {
            DEBUG_PRINT("O\\");
            psLineCoding->ucParity = USB_CDC_PARITY_ODD;
            break;
        }

        case UART_CONFIG_PAR_EVEN:
        {
            DEBUG_PRINT("E\\");
            psLineCoding->ucParity = USB_CDC_PARITY_EVEN;
            break;
        }

        case UART_CONFIG_PAR_ONE:
        {
            DEBUG_PRINT("M\\");
            psLineCoding->ucParity = USB_CDC_PARITY_MARK;
            break;
        }

        case UART_CONFIG_PAR_ZERO:
        {
            DEBUG_PRINT("S\\");
            psLineCoding->ucParity = USB_CDC_PARITY_SPACE;
            break;
        }
    }

    //
    // Translate the configuration stop bits field into the format expected
    // by the host.
    //
    switch(ulConfig & UART_CONFIG_STOP_MASK)
    {
        case UART_CONFIG_STOP_ONE:
        {
            DEBUG_PRINT("1\n");
            psLineCoding->ucStop = USB_CDC_STOP_BITS_1;
            break;
        }

        case UART_CONFIG_STOP_TWO:
        {
            DEBUG_PRINT("2\n");
            psLineCoding->ucStop = USB_CDC_STOP_BITS_2;
            break;
        }
    }

    //
    // Send the information back to the host.
    //
    USBDCDSendDataEP0(0, (unsigned char *)psLineCoding, sizeof(tLineCoding));
}

//*****************************************************************************
//
// This function sets or clears a break condition on the redirected UART RX
// line.  A break is started when the function is called with \e bSend set to
// \b true and persists until the function is called again with \e bSend set
// to \b false.
//
//*****************************************************************************
static void
SendBreak(tBoolean bSend)
{
    //
    // Are we being asked to start or stop the break condition?
    //
    if(!bSend)
    {
        //
        // Remove the break condition on the line.
        //
        UARTBreakCtl(USB_UART_BASE, false);
        g_bSendingBreak = false;
    }
    else
    {
        //
        // Start sending a break condition on the line.
        //
        UARTBreakCtl(USB_UART_BASE, true);
        g_bSendingBreak = true;
    }
}

//*****************************************************************************
//
// Shows the status string on the color STN display.
//
// \param psContext is a pointer to the graphics context representing the
// display.
// \param pcStatus is a pointer to the string to be shown.
//
//*****************************************************************************
void
DisplayStatus(tContext *psContext, char *pcStatus)
{
    tRectangle rectLine;
    long lY;

    //
    // Calculate the Y coordinate of the top left of the character cell
    // for our line of text.
    //
    lY = (GrContextDpyHeightGet(psContext) / 4) -
         (GrFontHeightGet(TEXT_FONT) / 2);

    //
    // Determine the bounding rectangle for this line of text.
    //
    rectLine.sXMin = 0;
    rectLine.sXMax = GrContextDpyWidthGet(psContext) - 1;
    rectLine.sYMin = lY;
    rectLine.sYMax = lY + GrFontHeightGet(TEXT_FONT) - 1;

    //
    // Clear the line with black.
    //
    GrContextForegroundSet(&g_sContext, ClrBlack);
    GrRectFill(psContext, &rectLine);

    //
    // Draw the new status string
    //
    DEBUG_PRINT("%s\n", pcStatus);
    GrContextForegroundSet(&g_sContext, ClrWhite);
    GrStringDrawCentered(psContext, pcStatus, -1,
                         GrContextDpyWidthGet(psContext) / 2,
                         GrContextDpyHeightGet(psContext) / 4 , false);
}

//*****************************************************************************
//
// Draw a horizontal meter at a given position on the display and fill it
// with green.
//
//*****************************************************************************
void
DrawBufferMeter(tContext *psContext, long lX, long lY)
{
    tRectangle sRect;
    long lCorrectedY;

    //
    // Correct the Y coordinate so that the meter is centered on the same line
    // as the text caption to its left.
    //
    lCorrectedY = lY - ((BUFFER_METER_HEIGHT - TEXT_HEIGHT) / 2);

    //
    // Determine the bounding rectangle of the meter.
    //
    sRect.sXMin = lX;
    sRect.sXMax = lX + BUFFER_METER_WIDTH - 1;
    sRect.sYMin = lCorrectedY;
    sRect.sYMax = lCorrectedY + BUFFER_METER_HEIGHT - 1;

    //
    // Fill the meter with green to indicate empty
    //
    GrContextForegroundSet(psContext, ClrGreen);
    GrRectFill(psContext, &sRect);

    //
    // Put a white box around the meter.
    //
    GrContextForegroundSet(psContext, ClrWhite);
    GrRectDraw(psContext, &sRect);
}

//*****************************************************************************
//
// Draw green and red blocks within a graphical meter on the display to
// indicate percentage fullness of some quantity (transmit and receive buffers
// in this case).
//
//*****************************************************************************
void
UpdateBufferMeter(tContext *psContext, unsigned long ulFullPercent, long lX,
                  long lY)
{
    tRectangle sRect;
    long lCorrectedY;
    long lXBreak;

    //
    // Correct the Y coordinate so that the meter is centered on the same line
    // as the text caption to its left and so that we avoid the meter's 1 pixel
    // white border.
    //
    lCorrectedY = lY - ((BUFFER_METER_HEIGHT - TEXT_HEIGHT) / 2) + 1;

    //
    // Determine where the break point between full (red) and empty (green)
    // sections occurs.
    //
    lXBreak = (lX + 1) + (ulFullPercent * (BUFFER_METER_WIDTH - 2)) / 100;

    //
    // Determine the bounding rectangle of the full section.
    //
    sRect.sXMin = lX + 1;
    sRect.sXMax = lXBreak;
    sRect.sYMin = lCorrectedY;
    sRect.sYMax = lCorrectedY + BUFFER_METER_HEIGHT - 3;

    //
    // Fill the full section with red (if there is anything to draw)
    //
    if(ulFullPercent)
    {
        GrContextForegroundSet(psContext, ClrRed);
        GrRectFill(psContext, &sRect);
    }

    //
    // Fill the empty section with green.
    //
    sRect.sXMin = lXBreak;
    sRect.sXMax = lX + BUFFER_METER_WIDTH - 2;
    if(sRect.sXMax > sRect.sXMin)
    {
        GrContextForegroundSet(psContext, ClrGreen);
        GrRectFill(psContext, &sRect);
    }

    //
    // Revert to white for text drawing which may occur later.
    //
    GrContextForegroundSet(psContext, ClrWhite);

}

//*****************************************************************************
//
// Handles CDC driver notifications related to control and setup of the device.
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to perform control-related
// operations on behalf of the USB host.  These functions include setting
// and querying the serial communication parameters, setting handshake line
// states and sending break conditions.
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long
ControlHandler(void *pvCBData, unsigned long ulEvent,
               unsigned long ulMsgValue, void *pvMsgData)
{
    unsigned long ulIntsOff;

    //
    // Which event are we being asked to process?
    //
    switch(ulEvent)
    {
        //
        // We are connected to a host and communication is now possible.
        //
        case USB_EVENT_CONNECTED:
            g_bUSBConfigured = true;

            //
            // Flush our buffers.
            //
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);

            //
            // Tell the main loop to update the display.
            //
            ulIntsOff = IntMasterDisable();
            g_pcStatus = "Host connected.";
            g_ulFlags |= COMMAND_STATUS_UPDATE;
            if(!ulIntsOff)
            {
                IntMasterEnable();
            }
            break;

        //
        // The host has disconnected.
        //
        case USB_EVENT_DISCONNECTED:
            g_bUSBConfigured = false;
            ulIntsOff = IntMasterDisable();
            g_pcStatus = "Host disconnected.";
            g_ulFlags |= COMMAND_STATUS_UPDATE;
            if(!ulIntsOff)
            {
                IntMasterEnable();
            }
            break;

        //
        // Return the current serial communication parameters.
        //
        case USBD_CDC_EVENT_GET_LINE_CODING:
            GetLineCoding(pvMsgData);
            break;

        //
        // Set the current serial communication parameters.
        //
        case USBD_CDC_EVENT_SET_LINE_CODING:
            SetLineCoding(pvMsgData);
            break;

        //
        // Set the current serial communication parameters.
        //
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
            SetControlLineState((unsigned short)ulMsgValue);
            break;

        //
        // Send a break condition on the serial line.
        //
        case USBD_CDC_EVENT_SEND_BREAK:
            SendBreak(true);
            break;

        //
        // Clear the break condition on the serial line.
        //
        case USBD_CDC_EVENT_CLEAR_BREAK:
            SendBreak(false);
            break;

        //
        // Ignore SUSPEND and RESUME for now.
        //
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif

    }

    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the transmit channel (data to
// the USB host).
//
// \param ulCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long
TxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
          void *pvMsgData)
{
    //
    // Which event have we been sent?
    //
    switch(ulEvent)
    {
        case USB_EVENT_TX_COMPLETE:
            //
            // Since we are using the USBBuffer, we don't need to do anything
            // here.
            //
            break;

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif

    }
    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the receive channel (data from
// the USB host).
//
// \param ulCBData is the client-supplied callback data value for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long
RxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
          void *pvMsgData)
{
    unsigned long ulCount;

    //
    // Which event are we being sent?
    //
    switch(ulEvent)
    {
        //
        // A new packet has been received.
        //
        case USB_EVENT_RX_AVAILABLE:
        {
            //
            // Feed some characters into the UART TX FIFO and enable the
            // interrupt so we are told when there is more space.
            //
            USBUARTPrimeTransmit(USB_UART_BASE);
            UARTIntEnable(USB_UART_BASE, UART_INT_TX);
            break;
        }

        //
        // We are being asked how much unprocessed data we have still to
        // process. We return 0 if the UART is currently idle or 1 if it is
        // in the process of transmitting something. The actual number of
        // bytes in the UART FIFO is not important here, merely whether or
        // not everything previously sent to us has been transmitted.
        //
        case USB_EVENT_DATA_REMAINING:
        {
            //
            // Get the number of bytes in the buffer and add 1 if some data
            // still has to clear the transmitter.
            //
            ulCount = UARTBusy(USB_UART_BASE) ? 1 : 0;
            return(ulCount);
        }

        //
        // We are being asked to provide a buffer into which the next packet
        // can be read. We do not support this mode of receiving data so let
        // the driver know by returning 0. The CDC driver should not be sending
        // this message but this is included just for illustration and
        // completeness.
        //
        case USB_EVENT_REQUEST_BUFFER:
        {
            return(0);
        }

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif
    }

    return(0);
}

//*****************************************************************************
//
// This is the main application entry function.
//
//*****************************************************************************
int
main(void)
{
    unsigned long ulTxCount;
    unsigned long ulRxCount;
    tRectangle sRect;
    char pcBuffer[16];
    unsigned long ulFullness;

    //
    // Set the clocking to run from the PLL at 50MHz
    //
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_8MHZ);

#if defined(DEBUG) && (USB_UART_BASE != UART0_BASE)
    if(g_iHandle == 0)
    {
        g_iHandle = DiagOpenStdio();
    }
#endif

    //
    // Not configured initially.
    //
    g_bUSBConfigured = false;

    //
    // Initialize the display driver.
    //
    Formike128x128x16Init();

    //
    // Turn on the backlight.
    //
    Formike128x128x16BacklightOn();

    //
    // Initialize the graphics context.
    //
    GrContextInit(&g_sContext, &g_sFormike128x128x16);

    //
    // Fill the top 15 rows of the screen with blue to create the banner.
    //
    sRect.sXMin = 0;
    sRect.sYMin = 0;
    sRect.sXMax = GrContextDpyWidthGet(&g_sContext) - 1;
    sRect.sYMax = 14;
    GrContextForegroundSet(&g_sContext, ClrDarkBlue);
    GrRectFill(&g_sContext, &sRect);

    //
    // Put a white box around the banner.
    //
    GrContextForegroundSet(&g_sContext, ClrWhite);
    GrRectDraw(&g_sContext, &sRect);

    //
    // Put the application name in the middle of the banner.
    //
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    GrStringDrawCentered(&g_sContext, "usb_dev_serial", -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 7, 0);

    //
    // Show the various static text elements on the color STN display.
    //
    GrContextFontSet(&g_sContext, TEXT_FONT);
    GrStringDraw(&g_sContext, "Tx bytes:", -1, 8, 50, false);
    GrStringDraw(&g_sContext, "Tx buffer:", -1, 8, 65, false);
    GrStringDraw(&g_sContext, "Rx bytes:", -1, 8, 90, false);
    GrStringDraw(&g_sContext, "Rx buffer:", -1, 8, 105, false);
    DrawBufferMeter(&g_sContext, 70, 65);
    DrawBufferMeter(&g_sContext, 70, 105);

    //
    // Enable the UART that we will be redirecting.
    //
    SysCtlPeripheralEnable(USB_UART_PERIPH);

    //
    // Enable and configure the UART RX and TX pins
    //
    SysCtlPeripheralEnable(TX_GPIO_PERIPH);
    SysCtlPeripheralEnable(RX_GPIO_PERIPH);
    GPIOPinTypeUART(TX_GPIO_BASE, TX_GPIO_PIN);
    GPIOPinTypeUART(RX_GPIO_BASE, RX_GPIO_PIN);

    //
    // TODO: Add code to configure handshake GPIOs if required.
    //

    //
    // Set the default UART configuration.
    //
    UARTConfigSetExpClk(USB_UART_BASE, SysCtlClockGet(), DEFAULT_BIT_RATE,
                        DEFAULT_UART_CONFIG);
    UARTFIFOLevelSet(USB_UART_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

    //
    // Configure and enable UART interrupts.
    //
    UARTIntClear(USB_UART_BASE, UARTIntStatus(USB_UART_BASE, false));
    UARTIntEnable(USB_UART_BASE, (UART_INT_OE | UART_INT_BE | UART_INT_PE |
                  UART_INT_FE | UART_INT_RT | UART_INT_TX | UART_INT_RX));

    //
    // Configure the USB mux on the board to put us in device mode.  We pull
    // the relevant pin high to do this.
    //
    SysCtlPeripheralEnable(USB_MUX_GPIO_PERIPH);
    GPIOPinTypeGPIOOutput(USB_MUX_GPIO_BASE, USB_MUX_GPIO_PIN);
    GPIOPinWrite(USB_MUX_GPIO_BASE, USB_MUX_GPIO_PIN, USB_MUX_SEL_DEVICE);

    //
    // Enable the system tick.
    //
    SysTickPeriodSet(SysCtlClockGet() / SYSTICKS_PER_SECOND);
    SysTickIntEnable();
    SysTickEnable();

    //
    // Show the application name on the display and UART output.
    //
    DEBUG_PRINT("\nLuminary Micro Serial CDC device example\n");
    DEBUG_PRINT("----------------------------------------\n\n");

    //
    // Tell the user what we are up to.
    //
    DisplayStatus(&g_sContext, "Configuring USB...");

    //
    // Initialize the transmit and receive buffers.
    //
    USBBufferInit((tUSBBuffer *)&g_sTxBuffer);
    USBBufferInit((tUSBBuffer *)&g_sRxBuffer);

    //
    // Pass our device information to the USB library and place the device
    // on the bus.
    //
    USBDCDCInit(0, (tUSBDCDCDevice *)&g_sCDCDevice);

    //
    // Wait for initial configuration to complete.
    //
    DisplayStatus(&g_sContext, "Waiting for host...");

    //
    // Clear our local byte counters.
    //
    ulRxCount = 0;
    ulTxCount = 0;

    //
    // Enable interrupts now that the application is ready to start.
    //
    IntEnable(USB_UART_INT);

    //
    // Main application loop.
    //
    while(1)
    {

        //
        // Have we been asked to update the status display?
        //
        if(g_ulFlags & COMMAND_STATUS_UPDATE)
        {
            //
            // Clear the command flag
            //
            IntMasterDisable();
            g_ulFlags &= ~COMMAND_STATUS_UPDATE;
            IntMasterEnable();

            DisplayStatus(&g_sContext, g_pcStatus);
        }

        //
        // Has there been any transmit traffic since we last checked?
        //
        if(ulTxCount != g_ulUARTTxCount)
        {
            //
            // Take a snapshot of the latest transmit count.
            //
            ulTxCount = g_ulUARTTxCount;

            //
            // Update the display of bytes transmitted by the UART.
            //
            usnprintf(pcBuffer, 16, "%d", ulTxCount);
            GrStringDraw(&g_sContext, pcBuffer, -1, 70, 50, true);

            //
            // Update the RX buffer fullness. Remember that the buffers are
            // named relative to the USB whereas the status display is from
            // the UART's perspective. The USB's receive buffer is the UART's
            // transmit buffer.
            //
            ulFullness = ((USBBufferDataAvailable(&g_sRxBuffer) * 100) /
                          UART_BUFFER_SIZE);

            UpdateBufferMeter(&g_sContext, ulFullness, 70, 65);
        }

        //
        // Has there been any receive traffic since we last checked?
        //
        if(ulRxCount != g_ulUARTRxCount)
        {
            //
            // Take a snapshot of the latest receive count.
            //
            ulRxCount = g_ulUARTRxCount;

            //
            // Update the display of bytes received by the UART.
            //
            usnprintf(pcBuffer, 16, "%d", ulRxCount);
            GrStringDraw(&g_sContext, pcBuffer, -1, 70, 90, true);

            //
            // Update the TX buffer fullness. Remember that the buffers are
            // named relative to the USB whereas the status display is from
            // the UART's perspective. The USB's transmit buffer is the UART's
            // receive buffer.
            //
            ulFullness = ((USBBufferDataAvailable(&g_sTxBuffer) * 100) /
                          UART_BUFFER_SIZE);

            UpdateBufferMeter(&g_sContext, ulFullness, 70, 105);
        }
    }
}
