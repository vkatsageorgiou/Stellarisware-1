USB HID Keyboard Host

This example application demonstrates how to support a USB keyboard
attached to the evaluation kit board.  The display will show if a keyboard
is currently connected and the current state of the Caps Lock key on the
keyboard that is connected on the bottom status area of the screen.
Pressing any keys on the keyboard will cause them to be printed on the
screen and to be sent out the UART at 115200 baud with no parity, 8 bits
and 1 stop bit.  Any keyboard that supports the USB HID bios protocol
should work with this demo application.

-------------------------------------------------------------------------------

Copyright (c) 2008-2009 Luminary Micro, Inc.  All rights reserved.
Software License Agreement

Luminary Micro, Inc. (LMI) is supplying this software for use solely and
exclusively on LMI's microcontroller products.

The software is owned by LMI and/or its suppliers, and is protected under
applicable copyright laws.  All rights are reserved.  You may not combine
this software with "viral" open-source software in order to form a larger
program.  Any use in violation of the foregoing restrictions may subject
the user to criminal sanctions under applicable laws, as well as to civil
liability for the breach of the terms and conditions of this license.

THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
LMI SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

This is part of revision 4781 of the EK-LM3S3748 Firmware Package.
