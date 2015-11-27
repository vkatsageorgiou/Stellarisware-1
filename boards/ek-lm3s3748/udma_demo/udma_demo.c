//*****************************************************************************
//
// udma_demo.c - uDMA example.
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
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "grlib/grlib.h"
#include "utils/cpu_usage.h"
#include "utils/ustdlib.h"
#include "drivers/formike128x128x16.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>uDMA (udma_demo)</h1>
//!
//! This example application demonstrates the use of the uDMA controller to
//! transfer data between memory buffers, and to transfer data to and from a
//! UART.
//
//*****************************************************************************

//*****************************************************************************
//
// The GPIO port and bit for the select button.
//
//*****************************************************************************
#define BUTTONS_GPIO_PERIPH     SYSCTL_PERIPH_GPIOB
#define BUTTONS_GPIO_BASE       GPIO_PORTB_BASE
#define SELECT_BUTTON           GPIO_PIN_7

//*****************************************************************************
//
// The number of SysTick ticks per second used for the SysTick interrupt.
//
//*****************************************************************************
#define SYSTICKS_PER_SECOND     100

//*****************************************************************************
//
// The size of the memory transfer source and destination buffers (in words).
//
//*****************************************************************************
#define MEM_BUFFER_SIZE         1024

//*****************************************************************************
//
// The size of the UART transmit and receive buffers.  They do not need to be
// the same size.
//
//*****************************************************************************
#define UART_TXBUF_SIZE         256
#define UART_RXBUF_SIZE         256

//*****************************************************************************
//
// Graphics context used to show text on the CSTN display.
//
//*****************************************************************************
static tContext g_sContext;

//*****************************************************************************
//
// The source and destination buffers used for memory transfers.
//
//*****************************************************************************
static unsigned long g_ulSrcBuf[MEM_BUFFER_SIZE];
static unsigned long g_ulDstBuf[MEM_BUFFER_SIZE];

//*****************************************************************************
//
// The transmit and receive buffers used for the UART transfers.  There is one
// transmit buffer and a pair of recieve ping-pong buffers.
//
//*****************************************************************************
static unsigned char g_ucTxBuf[UART_TXBUF_SIZE];
static unsigned char g_ucRxBufA[UART_RXBUF_SIZE];
static unsigned char g_ucRxBufB[UART_RXBUF_SIZE];

//*****************************************************************************
//
// The count of uDMA errors.  This value is incremented by the uDMA error
// handler.
//
//*****************************************************************************
static unsigned long g_uluDMAErrCount = 0;

//*****************************************************************************
//
// The count of times the uDMA interrupt occurred but the uDMA transfer was not
// complete.  This should remain 0.
//
//*****************************************************************************
static unsigned long g_ulBadISR = 0;

//*****************************************************************************
//
// The count of UART buffers filled, one for each ping-pong buffer.
//
//*****************************************************************************
static unsigned long g_ulRxBufACount = 0;
static unsigned long g_ulRxBufBCount = 0;

//*****************************************************************************
//
// The count of memory uDMA transfer blocks.  This value is incremented by the
// uDMA interrupt handler whenever a memory block transfer is completed.
//
//*****************************************************************************
static unsigned long g_ulMemXferCount = 0;

//*****************************************************************************
//
// The CPU usage in percent, in 16.16 fixed point format.
//
//*****************************************************************************
static unsigned long g_ulCPUUsage;

//*****************************************************************************
//
// The number of seconds elapsed since the start of the program.  This value is
// maintained by the SysTick interrupt handler.
//
//*****************************************************************************
static unsigned long g_ulSeconds = 0;

//*****************************************************************************
//
// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.
//
//*****************************************************************************
#if defined(ewarm)
#pragma data_alignment=1024
unsigned char ucControlTable[1024];
#else
unsigned char ucControlTable[1024] __attribute__ ((aligned(1024)));
#endif

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

//*****************************************************************************
//
// The interrupt handler for the SysTick timer.  This handler will increment a
// seconds counter whenever the appropriate number of ticks has occurred.  It
// will also call the CPU usage tick function to find the CPU usage percent.
//
//*****************************************************************************
void
SysTickHandler(void)
{
    static unsigned long ulTickCount = 0;

    //
    // Increment the tick counter.
    //
    ulTickCount++;

    //
    // If the number of ticks per second has occurred, then increment the
    // seconds counter.
    //
    if(!(ulTickCount % SYSTICKS_PER_SECOND))
    {
        g_ulSeconds++;
    }

    //
    // Call the CPU usage tick function.  This function will compute the amount
    // of cycles used by the CPU since the last call and return the result in
    // percent in fixed point 16.16 format.
    //
    g_ulCPUUsage = CPUUsageTick();
}

//*****************************************************************************
//
// The interrupt handler for uDMA errors.  This interrupt will occur if the
// uDMA encounters a bus error while trying to perform a transfer.  This
// handler just increments a counter if an error occurs.
//
//*****************************************************************************
void
uDMAErrorHandler(void)
{
    unsigned long ulStatus;

    //
    // Check for uDMA error bit
    //
    ulStatus = uDMAErrorStatusGet();

    //
    // If there is a uDMA error, then clear the error and increment
    // the error counter.
    //
    if(ulStatus)
    {
        uDMAErrorStatusClear();
        g_uluDMAErrCount++;
    }
}

//*****************************************************************************
//
// The interrupt handler for uDMA interrupts from the memory channel.  This
// interrupt will increment a counter, and then restart another memory
// transfer.
//
//*****************************************************************************
void
uDMAIntHandler(void)
{
    unsigned long ulMode;

    //
    // Check for the primary control structure to indicate complete.
    //
    ulMode = uDMAChannelModeGet(UDMA_CHANNEL_SW);
    if(ulMode == UDMA_MODE_STOP)
    {
        //
        // Increment the count of completed transfers.
        //
        g_ulMemXferCount++;

        //
        // Configure it for another transfer.
        //
        uDMAChannelTransferSet(UDMA_CHANNEL_SW, UDMA_MODE_AUTO,
                               g_ulSrcBuf, g_ulDstBuf, MEM_BUFFER_SIZE);

        //
        // Initiate another transfer.
        //
        uDMAChannelEnable(UDMA_CHANNEL_SW);
        uDMAChannelRequest(UDMA_CHANNEL_SW);
    }

    //
    // If the channel is not stopped, then something is wrong.
    //
    else
    {
        g_ulBadISR++;
    }
}

//*****************************************************************************
//
// The interrupt handler for UART1.  This interrupt will occur when a DMA
// transfer is complete using the UART1 uDMA channel.  It will also be
// triggered if the peripheral signals an error.  This interrupt handler will
// switch between receive ping-pong buffers A and B.  It will also restart a TX
// uDMA transfer if the prior transfer is complete.  This will keep the UART
// running continuously (looping TX data back to RX).
//
//*****************************************************************************
void
UART1IntHandler(void)
{
    unsigned long ulStatus;
    unsigned long ulMode;

    //
    // Read the interrupt status of the UART.
    //
    ulStatus = ROM_UARTIntStatus(UART1_BASE, 1);

    //
    // Clear any pending status, even though there should be none since no UART
    // interrupts were enabled.  If UART error interrupts were enabled, then
    // those interrupts could occur here and should be handled.  Since uDMA is
    // used for both the RX and TX, then neither of those interrupts should be
    // enabled.
    //
    ROM_UARTIntClear(UART1_BASE, ulStatus);

    //
    // Check the DMA control table to see if the ping-pong "A" transfer is
    // complete.  The "A" transfer uses receive buffer "A", and the primary
    // control structure.
    //
    ulMode = uDMAChannelModeGet(UDMA_CHANNEL_UART1RX | UDMA_PRI_SELECT);

    //
    // If the primary control structure indicates stop, that means the "A"
    // receive buffer is done.  The uDMA controller should still be receiving
    // data into the "B" buffer.
    //
    if(ulMode == UDMA_MODE_STOP)
    {
        //
        // Increment a counter to indicate data was received into buffer A.  In
        // a real application this would be used to signal the main thread that
        // data was received so the main thread can process the data.
        //
        g_ulRxBufACount++;

        //
        // Set up the next transfer for the "A" buffer, using the primary
        // control structure.  When the ongoing receive into the "B" buffer is
        // done, the uDMA controller will switch back to this one.  This
        // example re-uses buffer A, but a more sophisticated application could
        // use a rotating set of buffers to increase the amount of time that
        // the main thread has to process the data in the buffer before it is
        // reused.
        //
        uDMAChannelTransferSet(UDMA_CHANNEL_UART1RX | UDMA_PRI_SELECT,
                               UDMA_MODE_PINGPONG,
                               (void *)(UART1_BASE + UART_O_DR),
                               g_ucRxBufA, sizeof(g_ucRxBufA));
    }

    //
    // Check the DMA control table to see if the ping-pong "B" transfer is
    // complete.  The "B" transfer uses receive buffer "B", and the alternate
    // control structure.
    //
    ulMode = uDMAChannelModeGet(UDMA_CHANNEL_UART1RX | UDMA_ALT_SELECT);

    //
    // If the alternate control structure indicates stop, that means the "B"
    // receive buffer is done.  The uDMA controller should still be receiving
    // data into the "A" buffer.
    //
    if(ulMode == UDMA_MODE_STOP)
    {
        //
        // Increment a counter to indicate data was received into buffer A.  In
        // a real application this would be used to signal the main thread that
        // data was received so the main thread can process the data.
        //
        g_ulRxBufBCount++;

        //
        // Set up the next transfer for the "B" buffer, using the alternate
        // control structure.  When the ongoing receive into the "A" buffer is
        // done, the uDMA controller will switch back to this one.  This
        // example re-uses buffer B, but a more sophisticated application could
        // use a rotating set of buffers to increase the amount of time that
        // the main thread has to process the data in the buffer before it is
        // reused.
        //
        uDMAChannelTransferSet(UDMA_CHANNEL_UART1RX | UDMA_ALT_SELECT,
                               UDMA_MODE_PINGPONG,
                               (void *)(UART1_BASE + UART_O_DR),
                               g_ucRxBufB, sizeof(g_ucRxBufB));
    }

    //
    // If the UART1 DMA TX channel is disabled, that means the TX DMA transfer
    // is done.
    //
    if(!uDMAChannelIsEnabled(UDMA_CHANNEL_UART1TX))
    {
        //
        // Start another DMA transfer to UART1 TX.
        //
        uDMAChannelTransferSet(UDMA_CHANNEL_UART1TX | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC, g_ucTxBuf,
                               (void *)(UART1_BASE + UART_O_DR),
                               sizeof(g_ucTxBuf));

        //
        // The uDMA TX channel must be re-enabled.
        //
        uDMAChannelEnable(UDMA_CHANNEL_UART1TX);
    }
}

//*****************************************************************************
//
// Initializes the UART1 peripheral and sets up the TX and RX uDMA channels.
// The UART is configured for loopback mode so that any data sent on TX will be
// received on RX.  The uDMA channels are configured so that the TX channel
// will copy data from a buffer to the UART TX output.  And the uDMA RX channel
// will receive any incoming data into a pair of buffers in ping-pong mode.
//
//*****************************************************************************
void
InitUART1Transfer(void)
{
    unsigned int uIdx;

    //
    // Fill the TX buffer with a simple data pattern.
    //
    for(uIdx = 0; uIdx < UART_TXBUF_SIZE; uIdx++)
    {
        g_ucTxBuf[uIdx] = uIdx;
    }

    //
    // Enable the UART peripheral, and configure it to operate even if the CPU
    // is in sleep.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART1);

    //
    // Configure the UART communication parameters.
    //
    ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 115200,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE);

    //
    // Set both the TX and RX trigger thresholds to 4.  This will be used by
    // the uDMA controller to signal when more data should be transferred.  The
    // uDMA TX and RX channels will be configured so that it can transfer 4
    // bytes in a burst when the UART is ready to transfer more data.
    //
    ROM_UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

    //
    // Enable the UART for operation, and enable the uDMA interface for both TX
    // and RX channels.
    //
    ROM_UARTEnable(UART1_BASE);
    UARTDMAEnable(UART1_BASE, UART_DMA_RX | UART_DMA_TX);

    //
    // This register write will set the UART to operate in loopback mode.  Any
    // data sent on the TX output will be received on the RX input.
    //
    HWREG(UART1_BASE + UART_O_CTL) |= UART_CTL_LBE;

    //
    // Enable the UART peripheral interrupts.  Note that no UART interrupts
    // were enabled, but the uDMA controller will cause an interrupt on the
    // UART interrupt signal when a uDMA transfer is complete.
    //
    ROM_IntEnable(INT_UART1);

    //
    // Put the attributes in a known state for the uDMA UART1RX channel.  These
    // should already be disabled by default.
    //
    uDMAChannelAttributeDisable(UDMA_CHANNEL_UART1RX,
                                UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);

    //
    // Configure the control parameters for the primary control structure for
    // the UART RX channel.  The primary contol structure is used for the "A"
    // part of the ping-pong receive.  The transfer data size is 8 bits, the
    // source address does not increment since it will be reading from a
    // register.  The destination address increment is byte 8-bit bytes.  The
    // arbitration size is set to 4 to match the RX FIFO trigger threshold.
    // The uDMA controller will use a 4 byte burst transfer if possible.  This
    // will be somewhat more effecient that single byte transfers.
    //
    uDMAChannelControlSet(UDMA_CHANNEL_UART1RX | UDMA_PRI_SELECT,
                          UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
                          UDMA_ARB_4);

    //
    // Configure the control parameters for the alternate control structure for
    // the UART RX channel.  The alternate contol structure is used for the "B"
    // part of the ping-pong receive.  The configuration is identical to the
    // primary/A control structure.
    //
    uDMAChannelControlSet(UDMA_CHANNEL_UART1RX | UDMA_ALT_SELECT,
                          UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
                          UDMA_ARB_4);

    //
    // Set up the transfer parameters for the UART RX primary control
    // structure.  The mode is set to ping-pong, the transfer source is the
    // UART data register, and the destination is the receive "A" buffer.  The
    // transfer size is set to match the size of the buffer.
    //
    uDMAChannelTransferSet(UDMA_CHANNEL_UART1RX | UDMA_PRI_SELECT,
                           UDMA_MODE_PINGPONG,
                           (void *)(UART1_BASE + UART_O_DR),
                           g_ucRxBufA, sizeof(g_ucRxBufA));

    //
    // Set up the transfer parameters for the UART RX alternate control
    // structure.  The mode is set to ping-pong, the transfer source is the
    // UART data register, and the destination is the receive "B" buffer.  The
    // transfer size is set to match the size of the buffer.
    //
    uDMAChannelTransferSet(UDMA_CHANNEL_UART1RX | UDMA_ALT_SELECT,
                           UDMA_MODE_PINGPONG,
                           (void *)(UART1_BASE + UART_O_DR),
                           g_ucRxBufB, sizeof(g_ucRxBufB));

    //
    // Put the attributes in a known state for the uDMA UART1TX channel.  These
    // should already be disabled by default.
    //
    uDMAChannelAttributeDisable(UDMA_CHANNEL_UART1TX,
                                UDMA_ATTR_ALTSELECT |
                                UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);

    //
    // Set the USEBURST attribute for the uDMA UART TX channel.  This will
    // force the controller to always use a burst when transferring data from
    // the TX buffer to the UART.  This is somewhat more effecient bus usage
    // than the default which allows single or burst transfers.
    //
    uDMAChannelAttributeEnable(UDMA_CHANNEL_UART1TX, UDMA_ATTR_USEBURST);

    //
    // Configure the control parameters for the UART TX.  The uDMA UART TX
    // channel is used to transfer a block of data from a buffer to the UART.
    // The data size is 8 bits.  The source address increment is 8-bit bytes
    // since the data is coming from a buffer.  The destination increment is
    // none since the data is to be written to the UART data register.  The
    // arbitration size is set to 4, which matches the UART TX FIFO trigger
    // threshold.
    //
    uDMAChannelControlSet(UDMA_CHANNEL_UART1TX | UDMA_PRI_SELECT,
                          UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
                          UDMA_ARB_4);

    //
    // Set up the transfer parameters for the uDMA UART TX channel.  This will
    // configure the transfer source and destination and the transfer size.
    // Basic mode is used because the peripheral is making the uDMA transfer
    // request.  The source is the TX buffer and the destination is the UART
    // data register.
    //
    uDMAChannelTransferSet(UDMA_CHANNEL_UART1TX | UDMA_PRI_SELECT,
                           UDMA_MODE_BASIC, g_ucTxBuf,
                           (void *)(UART1_BASE + UART_O_DR),
                           sizeof(g_ucTxBuf));

    //
    // Now both the uDMA UART TX and RX channels are primed to start a
    // transfer.  As soon as the channels are enabled, the peripheral will
    // issue a transfer request and the data transfers will begin.
    //
    uDMAChannelEnable(UDMA_CHANNEL_UART1RX);
    uDMAChannelEnable(UDMA_CHANNEL_UART1TX);
}

//*****************************************************************************
//
// Initializes the uDMA software channel to perform a memory to memory uDMA
// transfer.
//
//*****************************************************************************
void
InitSWTransfer(void)
{
    unsigned int uIdx;

    //
    // Fill the source memory buffer with a simple incrementing pattern.
    //
    for(uIdx = 0; uIdx < MEM_BUFFER_SIZE; uIdx++)
    {
        g_ulDstBuf[uIdx] = uIdx;
    }

    //
    // Enable interrupts from the uDMA software channel.
    //
    ROM_IntEnable(INT_UDMA);

    //
    // Put the attributes in a known state for the uDMA software channel.
    // These should already be disabled by default.
    //
    uDMAChannelAttributeDisable(UDMA_CHANNEL_SW,
                                UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT |
                                UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);

    //
    // Configure the control parameters for the SW channel.  The SW channel
    // will be used to transfer between two memory buffers, 32 bits at a time.
    // Therefore the data size is 32 bits, and the address increment is 32 bits
    // for both source and destination.  The arbitration size will be set to 8,
    // which causes the uDMA controller to rearbitrate after 8 items are
    // transferred.  This keeps this channel from hogging the uDMA controller
    // once the transfer is started, and allows other channels cycles if they
    // are higher priority.
    //
    uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
                          UDMA_SIZE_32 | UDMA_SRC_INC_32 | UDMA_DST_INC_32 |
                          UDMA_ARB_8);

    //
    // Set up the transfer parameters for the software channel.  This will
    // configure the transfer buffers and the transfer size.  Auto mode must be
    // used for software transfers.
    //
    uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_MODE_AUTO,
                           g_ulSrcBuf, g_ulDstBuf, MEM_BUFFER_SIZE);

    //
    // Now the software channel is primed to start a transfer.  The channel
    // must be enabled.  For software based transfers, a request must be
    // issued.  After this, the uDMA memory transfer begins.
    //
    uDMAChannelEnable(UDMA_CHANNEL_SW);
    uDMAChannelRequest(UDMA_CHANNEL_SW);
}

//*****************************************************************************
//
// This example demonstrates how to use the uDMA controller to transfer data
// between memory buffers and to and from a peripheral, in this case a UART.
// The uDMA controller is configured to repeatedly transfer a block of data
// from one memory buffer to another.  It is also set up to repeatedly copy a
// block of data from a buffer to the UART output.  The UART data is looped
// back so the same data is received, and the uDMA controlled is configured to
// continuously receive the UART data using ping-pong buffers.
//
// The processor is put to sleep when it is not doing anything, and this allows
// collection of CPU usage data to see how much CPU is being used while the
// data transfers are ongoing.
//
//*****************************************************************************
int
main(void)
{
    static unsigned long ulPrevSeconds;
    static unsigned long ulPrevXferCount;
    static unsigned long ulPrevUARTCount = 0;
    static char cStrBuf[40];
    tRectangle sRect;
    unsigned long ulCenterX;
    unsigned long ulXfersCompleted;
    unsigned long ulBytesTransferred;
    unsigned long ulButton;

    //
    // Set the clocking to run from the PLL at 50 MHz.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_8MHZ);

    //
    // Enable peripherals to operate when CPU is in sleep.
    //
    ROM_SysCtlPeripheralClockGating(true);

    //
    // Set the select button as an input with a pull-up.
    //
    ROM_SysCtlPeripheralEnable(BUTTONS_GPIO_PERIPH);
    ROM_GPIODirModeSet(BUTTONS_GPIO_BASE, SELECT_BUTTON, GPIO_DIR_MODE_IN);
    ROM_GPIOPadConfigSet(BUTTONS_GPIO_BASE, SELECT_BUTTON,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

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
    // Get the center X coordinate of the screen, since it is used a lot.
    //
    ulCenterX = GrContextDpyWidthGet(&g_sContext) / 2;

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
    GrStringDrawCentered(&g_sContext, "udma_demo", -1, ulCenterX, 7, 0);

    //
    // Show the clock frequency on the display.
    //
    usnprintf(cStrBuf, sizeof(cStrBuf), "Stellaris @ %u MHz",
              ROM_SysCtlClockGet() / 1000000);
    GrStringDrawCentered(&g_sContext, cStrBuf, -1, ulCenterX, 32, 0);

    //
    // Show static text and field labels on the display.
    //
    GrStringDrawCentered(&g_sContext, "uDMA Mem Transfers", -1,
                         ulCenterX, 56, 0);
    GrStringDrawCentered(&g_sContext, "uDMA UART Transfers", -1,
                         ulCenterX, 80, 0);
    GrStringDrawCentered(&g_sContext, "Press button", -1,
                         ulCenterX, 112, 0);
    GrStringDrawCentered(&g_sContext, "to use debugger", -1,
                         ulCenterX, 120, 0);

    //
    // Configure SysTick to occur 100 times per second, to use as a time
    // reference.  Enable SysTick to generate interrupts.
    //
    ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / SYSTICKS_PER_SECOND);
    ROM_SysTickIntEnable();
    ROM_SysTickEnable();

    //
    // Initialize the CPU usage measurement routine.
    //
    CPUUsageInit(ROM_SysCtlClockGet(), SYSTICKS_PER_SECOND, 2);

    //
    // Enable the uDMA controller at the system level.  Enable it to continue
    // to run while the processor is in sleep.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);

    //
    // Enable the uDMA controller error interrupt.  This interrupt will occur
    // if there is a bus error during a transfer.
    //
    ROM_IntEnable(INT_UDMAERR);

    //
    // Enable the uDMA controller.
    //
    uDMAEnable();

    //
    // Point at the control table to use for channel control structures.
    //
    uDMAControlBaseSet(ucControlTable);

    //
    // Initialize the uDMA memory to memory transfers.
    //
    InitSWTransfer();

    //
    // Initialize the uDMA UART transfers.
    //
    InitUART1Transfer();

    //
    // Remember the current SysTick seconds count.
    //
    ulPrevSeconds = g_ulSeconds;

    //
    // Remember the current count of memory buffer transfers.
    //
    ulPrevXferCount = g_ulMemXferCount;

    //
    // Loop until the button is pressed.  The processor is put to sleep
    // in this loop so that CPU utilization can be measured.  When the
    // processor is sleeping a lot, it can be hard to connect to the target
    // with the debugger.  Pressing the button will cause this loop to exit
    // and the processor will no longer sleep.
    //
    while(1)
    {
        //
        // Check for the select button press.  If the button is pressed,
        // then exit this loop.
        //
        ulButton = ROM_GPIOPinRead(BUTTONS_GPIO_BASE, SELECT_BUTTON);
        if(!ulButton)
        {
            break;
        }

        //
        // Check to see if one second has elapsed.  If so, the make some
        // updates.
        //
        if(g_ulSeconds != ulPrevSeconds)
        {
            //
            // Print a message to the display showing the CPU usage percent.
            // The fractional part of the percent value is ignored.
            //
            usnprintf(cStrBuf, sizeof(cStrBuf), "CPU util %2u%%",
                      g_ulCPUUsage >> 16);
            GrStringDrawCentered(&g_sContext, cStrBuf, -1, ulCenterX, 40, 1);

            //
            // Remember the new seconds count.
            //
            ulPrevSeconds = g_ulSeconds;

            //
            // Calculate how many memory transfers have occurred since the last
            // second.
            //
            ulXfersCompleted = g_ulMemXferCount - ulPrevXferCount;

            //
            // Remember the new transfer count.
            //
            ulPrevXferCount = g_ulMemXferCount;

            //
            // Compute how many bytes were transferred in the memory transfer
            // since the last second.
            //
            ulBytesTransferred = ulXfersCompleted * MEM_BUFFER_SIZE * 4;

            //
            // Print a message to the display showing the memory transfer rate.
            //
            usnprintf(cStrBuf, sizeof(cStrBuf), "%8u Bytes/Sec",
                      ulBytesTransferred);
            GrStringDrawCentered(&g_sContext, cStrBuf, -1, ulCenterX, 65, 1);

            //
            // Calculate how many UART transfers have occurred since the last
            // second.
            //
            ulXfersCompleted = (g_ulRxBufACount + g_ulRxBufBCount -
                                ulPrevUARTCount);

            //
            // Remember the new UART transfer count.
            //
            ulPrevUARTCount = g_ulRxBufACount + g_ulRxBufBCount;

            //
            // Compute how many bytes were transferred by the UART.  The number
            // of bytes received is multiplied by 2 so that the TX bytes
            // transferred are also accounted for.
            //
            ulBytesTransferred = ulXfersCompleted * UART_RXBUF_SIZE * 2;

            //
            // Print a message to the display showing the UART transfer rate.
            //
            usnprintf(cStrBuf, sizeof(cStrBuf), "%8u Bytes/Sec",
                      ulBytesTransferred);
            GrStringDrawCentered(&g_sContext, cStrBuf, -1, ulCenterX, 89, 1);
        }

        //
        // Put the processor to sleep if there is nothing to do.  This allows
        // the CPU usage routine to measure the number of free CPU cycles.
        // If the processor is sleeping a lot, it can be hard to connect to
        // the target with the debugger.
        //
        ROM_SysCtlSleep();
    }

    //
    // Indicate on the display that the example is stopped.
    //
    GrStringDrawCentered(&g_sContext, "            ", -1,
                         ulCenterX, 112, 1);
    GrStringDrawCentered(&g_sContext, "    Stopped    ", -1,
                         ulCenterX, 120, 1);

    //
    // Loop forever with the CPU not sleeping, so the debugger can connect.
    //
    while(1)
    {
    }
}
