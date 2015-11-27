USB Memory Stick Updater

This example application behaves the same way as a boot loader.  It resides
at the beginning of flash, and will read a binary file from a USB memory
stick and program it into another location in flash.  Once the user
application has been programmed into flash, this program will always start
the user application until requested to load a new application.

When this application starts, if there is a user application already in
flash (at \b APP_START_ADDRESS), then it will just run the user application.
It will attempt to load a new application from a USB memory stick under
the following conditions:

- no user application is present at \b APP_START_ADDRESS
- the user application has requested an update by transferring control
to the updater
- the user holds down the eval board push button when the board is reset

When this application is attempting to perform an update, it will wait
forever for a USB memory stick to be plugged in.  Once a USB memory stick
is found, it will search the root directory for a specific file name, which
is \e FIRMWARE.BIN by default.  This file must be a binary image of the
program you want to load (the .bin file), linked to run from the correct
address, at \b APP_START_ADDRESS.

The USB memory stick must be formatted as a FAT16 or FAT32 file system
(the normal case), and the binary file must be located in the root
directory.  Other files can exist on the memory stick but they will be
ignored.

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
