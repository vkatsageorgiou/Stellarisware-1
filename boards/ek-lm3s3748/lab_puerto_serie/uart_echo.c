//*****************************************************************************
//
// uart_echo.c - Example for reading data from and writing data to the UART in
//               an interrupt driven fashion.
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
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "grlib/grlib.h"
#include "drivers/formike128x128x16.h"

// Graphics context used to show text on the CSTN display.
tContext g_sContext;

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART (uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the FTDI virtual serial port on the evaluation board) will be
//! configured in 115,200 baud, 8-n-1 mode.  All characters received on the
//! UART are transmitted back to the UART.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

void itoa(int i, char b[])
{
    char const digit[] = "0123456789";
    char* p = b;
    if(i<0){
        *p++ = '-';
        i *= -1;
    }
    int shifter = i;
    do{ //Move to where representation ends
        ++p;
        shifter = shifter/10;
    }while(shifter);
    *p = '\0';
    do{ //Move back, inserting digits as u go
        *--p = digit[i%10];
        i = i/10;
    }while(i);
}



//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
int Eco = 1;
int update = 0;
char UART_Respuesta;

void UARTIntHandler(void)
{
    update = 1;
    //
    // Get the interrrupt status.
    //
    unsigned long ulStatus = ROM_UARTIntStatus(UART0_BASE, true);;

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART0_BASE, ulStatus);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART0_BASE))
    {
        UART_Respuesta = ROM_UARTCharGetNonBlocking(UART0_BASE);
        //
        // Read the next character from the UART and write it back to the UART.
        //
        //if (UART_Respuesta == '\r') ROM_UARTCharPutNonBlocking(UART0_BASE,'\n');
        if (Eco)
        {
            ROM_UARTCharPutNonBlocking(UART0_BASE,UART_Respuesta);
        }
    }
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void UARTSend(const unsigned char *pucBuffer)
{
    //
    // Loop while there are more characters to send.
    //
    while(*pucBuffer != '\0')
    {
        //
        // Write the next character to the UART.
        //
        UARTCharPut(UART0_BASE, *pucBuffer++);
    }
}

void screen_config()
{
    // Set the clocking to run directly from the crystal.
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ);

    // Initialize the display driver.
    Formike128x128x16Init();

    // Turn on the backlight.
    Formike128x128x16BacklightOn();

    // Initialize the graphics context.
    GrContextInit(&g_sContext, &g_sFormike128x128x16);
}

void uart_config()
{
    // Enable the peripherals used by this example.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable processor interrupts.
    IntMasterEnable();

    // Set GPIO A0 and A1 as UART pins.
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the UART for 115,200, 8-N-1 operation.
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Enable the UART interrupt.
    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

int main(void)
{
    screen_config();

    tRectangle sRect;

    // Fill the top 15 rows of the screen with blue to create the banner.
    sRect.sXMin = 0;
    sRect.sYMin = 0;
    sRect.sXMax = GrContextDpyWidthGet(&g_sContext) - 1;
    sRect.sYMax = 14;
    GrContextForegroundSet(&g_sContext, ClrLime);
    GrRectFill(&g_sContext, &sRect);

    // Put a white box around the banner.
    GrContextForegroundSet(&g_sContext, ClrWhite);
    GrRectDraw(&g_sContext, &sRect);

    // Put the application name in the middle of the banner.
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    GrStringDrawCentered(&g_sContext, "Lab Puerto Serie", -1,
                                   GrContextDpyWidthGet(&g_sContext) / 2, 7, 0);

    // Initialize the CSTN display and write status.
    GrStringDraw(&g_sContext, "Port:   Uart 0",       -1, 12, 24, 0);
    GrStringDraw(&g_sContext, "Baud:   115,200 bps",  -1, 12, 32, 0);
    GrStringDraw(&g_sContext, "Data:   8 Bit",        -1, 12, 40, 0);
    GrStringDraw(&g_sContext, "Parity: None",         -1, 12, 48, 0);
    GrStringDraw(&g_sContext, "Stop:   1 Bit",        -1, 12, 56, 0);

    uart_config();
    // Prompt for text to be entered.
    UARTSend((unsigned char *)"**********Lab 1: Puerto serie**********\r\n");
    UARTSend((unsigned char *)"1: On Eco\r\n2: Off Eco\r\n3: On mostrar tecla");
    UARTSend((unsigned char *)"\r\n4: Off mostrar tecla\r\n");

    tRectangle pRect;
    pRect.sXMin = 0;
    pRect.sYMin = 80;
    pRect.sXMax = GrContextDpyWidthGet(&g_sContext) - 1;
    pRect.sYMax = 100;
    int mostrar_respuesta = 1;
    // Loop forever echoing data through the UART.
    while(1)
    {
        switch (UART_Respuesta)
        {
            case '1':
              Eco = 1;
              break;
            case '2':
              Eco = 0;
              break;
            case '3':
              mostrar_respuesta = 1;
              break;
            case '4':
              mostrar_respuesta = 0;
              break;
            case '\r':
              ROM_UARTCharPutNonBlocking(UART0_BASE,'\n');
              break;
        }
        if (mostrar_respuesta == 1)
        {
            if (update)
            {
                GrContextForegroundSet(&g_sContext, ClrBlack);
                GrRectFill(&g_sContext, &pRect);
                update = 0;
                //GrFlush(&g_sContext);
            }
            GrContextFontSet(&g_sContext, &g_sFontCm20);
            GrContextForegroundSet(&g_sContext, ClrWhite);
            GrStringDraw(&g_sContext,&UART_Respuesta,-1, 12, 80, 1);
            //GrFlush(&g_sContext);
        }
    }
}
