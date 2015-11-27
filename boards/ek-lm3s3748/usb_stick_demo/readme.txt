USB Stick Update Demo

An example to demonstrate the use of the flash-based USB stick update
program.  This example is meant to be loaded into flash memory from a USB
memory stick, using the USB stick update program (usb_stick_update),
running on the microcontroller.

After this program is built, the binary file (usb_stick_demo.bin), should
be renamed to the filename expected by usb_stick_update ("FIRMWARE.BIN" by
default) and copied to the root directory of a USB memory stick.  Then,
when the memory stick is plugged into the eval board that is running the
usb_stick_update program, this example program will be loaded into flash
and then run on the microcontroller.

This program simply displays a message on the screen and prompts the user
to press the select button.  Once the button is pressed, control is passed
back to the usb_stick_update program which is still is flash, and it will
attempt to load another program from the memory stick.  This shows how
a user application can force a new firmware update from the memory stick.

-------------------------------------------------------------------------------

Copyright (c) 2009 Luminary Micro, Inc.  All rights reserved.
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
