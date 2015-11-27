//*****************************************************************************
//
// pwmgen.c - PWM signal generation example.
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

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "grlib/grlib.h"
#include "drivers/formike128x128x16.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>PWM (pwmgen)</h1>
//!
//! This example application utilizes the PWM peripheral to output a 20% duty
//! cycle PWM signal and a 80% duty cycle PWM signal, both at 8000 Hz.  Once
//! configured, the application enters an infinite loop, doing nothing while
//! the PWM peripheral continues to output its signals.
//
//*****************************************************************************

//*****************************************************************************
//
// Graphics context used to show text on the CSTN display.
//
//*****************************************************************************
tContext g_sContext;

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
// This example demonstrates how to setup the PWM block to generate signals.
//
//*****************************************************************************
int
main(void)
{
    tRectangle sRect;
    unsigned long ulPeriod;

    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_8MHZ);
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

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
    GrStringDrawCentered(&g_sContext, "pwmgen", -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 7, 0);

    //
    // Tell the user what is happening.
    //
    GrStringDrawCentered(&g_sContext, "Generating PWM on", -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 56, 0);
    GrStringDrawCentered(&g_sContext, "pins PWM0 and PWM1", -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 68, 0);

    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Set GPIO F0 and F1 as PWM pins.  They are used to output the PWM0 and
    // PWM1 signals.
    //
    ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1,
                         GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

    //
    // Compute the PWM period based on the system clock.
    //
    ulPeriod = ROM_SysCtlClockGet() / 8000;

    //
    // Set the PWM period to 440 (A) Hz.
    //
    ROM_PWMGenConfigure(PWM_BASE, PWM_GEN_0,
                        PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    ROM_PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, ulPeriod);

    //
    // Set PWM0 to a duty cycle of 50% and PWM1 to a duty cycle of 80%.
    //
    ROM_PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, ((ulPeriod * 8) / 10));
    ROM_PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, ((ulPeriod * 2) / 10));

    //
    // Enable the PWM0 and PWM1 output signals.
    //
    ROM_PWMOutputState(PWM_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);

    //
    // Enable the PWM generator.
    //
    ROM_PWMGenEnable(PWM_BASE, PWM_GEN_0);

    //
    // Loop forever while the PWM signals are generated.
    //
    while(1)
    {
    }
}
