USB Boot Loader Demo 2

An example to demonstrate the use of the flash-based USB boot loader.  At
startup, the application displays a message then waits for the user to press
the select button before branching to the USB boot loader to await the
start of an update.  The boot loader presents a Device Firmware Upgrade
interface to the host allowing new applications to be downloaded to flash
via USB.

The usb_boot_demo1 application can be used along with this application to
easily demonstrate that the boot loader is actually updating the on-chip
flash.

The application dfuwrap, found in the boards directory, can be used to
prepare binary images for download to a particular position in device
flash.  This application adds a Luminary-specific prefix and a DFU standard
suffix to the binary. A sample Windows command line application, dfuprog,
is also provided which allows either binary images or DFU-wrapped files to
be downloaded to the board or uploaded from it.

The usb_boot_demo1 and usb_boot_demo2 applications are essentially
identical to boot_demo1 and boot_demo2 with the exception that they are
linked to run at address 0x1800 rather than 0x0.  This is due to the fact
that the USB boot loader is not currently included in the Stellaris ROM
and therefore has to be stored in the bottom few KB of flash with the
main application stored above it.

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
