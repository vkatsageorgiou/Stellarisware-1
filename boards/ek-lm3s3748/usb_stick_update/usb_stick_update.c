//*****************************************************************************
//
// usb_stick_update.c - Example to update flash from a USB memory stick.
//
// Copyright (c) 2009 Luminary Micro, Inc.  All rights reserved.
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

#include <string.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_flash.h"
#include "inc/hw_gpio.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_usb.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/udma.h"
#include "driverlib/gpio.h"
#include "driverlib/usb.h"
#include "usblib/usblib.h"
#include "usblib/usbmsc.h"
#include "usblib/host/usbhost.h"
#include "usblib/host/usbhmsc.h"
#include "simple_fs.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>USB Memory Stick Updater (usb_stick_update)</h1>
//!
//! This example application behaves the same way as a boot loader.  It resides
//! at the beginning of flash, and will read a binary file from a USB memory
//! stick and program it into another location in flash.  Once the user
//! application has been programmed into flash, this program will always start
//! the user application until requested to load a new application.
//!
//! When this application starts, if there is a user application already in
//! flash (at \b APP_START_ADDRESS), then it will just run the user application.
//! It will attempt to load a new application from a USB memory stick under
//! the following conditions:
//!
//! - no user application is present at \b APP_START_ADDRESS
//! - the user application has requested an update by transferring control
//! to the updater
//! - the user holds down the eval board push button when the board is reset
//!
//! When this application is attempting to perform an update, it will wait
//! forever for a USB memory stick to be plugged in.  Once a USB memory stick
//! is found, it will search the root directory for a specific file name, which
//! is \e FIRMWARE.BIN by default.  This file must be a binary image of the
//! program you want to load (the .bin file), linked to run from the correct
//! address, at \b APP_START_ADDRESS.
//!
//! The USB memory stick must be formatted as a FAT16 or FAT32 file system
//! (the normal case), and the binary file must be located in the root
//! directory.  Other files can exist on the memory stick but they will be
//! ignored.
//
//*****************************************************************************

//*****************************************************************************
//
// The name of the binary firmware file on the USB stick.  This is the user
// application that will be searched for and loaded into flash if it is found.
// Note that the name of the file must be 11 characters total, 8 for the base
// name and 3 for the extension.  If the actual file name has fewer characters
// then it must be padded with spaces.  This macro should not contain the dot
// "." for the extension.
//
// Examples: firmware.bin --> "FIRMWAREBIN"
//           myfile.bn    --> "MYFILE  BN "
//
//*****************************************************************************
#define USB_UPDATE_FILENAME "FIRMWAREBIN"

//*****************************************************************************
//
// The size of the flash for this microcontroller.
//
//*****************************************************************************
#define FLASH_SIZE (128 * 1024)

//*****************************************************************************
//
// The starting address for the application that will be loaded into flash
// memory from the USB stick.  This address must be high enough to be above
// the USB stick updater, and must be on a 1K boundary.
// Note that the application that will be loaded must also be linked to run
// from this address.
//
//*****************************************************************************
#define APP_START_ADDRESS 0x3000

//*****************************************************************************
//
// A memory location and value that is used to indicate that the application
// wants to force an update.
//
//*****************************************************************************
#define FORCE_UPDATE_ADDR 0x20004000
#define FORCE_UPDATE_VALUE 0x1234cdef

//*****************************************************************************
//
// The parameters to control the USB mux on the LM3S3748 board.
//
//*****************************************************************************
#define USB_MUX_GPIO_PERIPH     SYSCTL_PERIPH_GPIOH
#define USB_MUX_GPIO_BASE       GPIO_PORTH_BASE
#define USB_MUX_GPIO_PIN        GPIO_PIN_2
#define USB_MUX_SEL_DEVICE      USB_MUX_GPIO_PIN
#define USB_MUX_SEL_HOST        0

//*****************************************************************************
//
// The prototype for the function that is used to call the user application.
//
//*****************************************************************************
static void CallApplication(unsigned long ulStartAddr);

//*****************************************************************************
//
// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.  In this application uDMA is only used for USB,
// so only the first 6 channels are needed.
//
//*****************************************************************************
#if defined(ewarm)
#pragma data_alignment=1024
tDMAControlTable g_sDMAControlTable[6];
#else
tDMAControlTable g_sDMAControlTable[6] __attribute__ ((aligned(1024)));
#endif

//*****************************************************************************
//
// The global that holds all of the host drivers in use in the application.
// In this case, only the MSC class is loaded.
//
//*****************************************************************************
static tUSBHostClassDriver const * const g_ppHostClassDrivers[] =
{
    &g_USBHostMSCClassDriver
};

//*****************************************************************************
//
// This global holds the number of class drivers in the g_ppHostClassDrivers
// list.
//
//*****************************************************************************
#define NUM_CLASS_DRIVERS       (sizeof(g_ppHostClassDrivers) /               \
                                 sizeof(g_ppHostClassDrivers[0]))

//*****************************************************************************
//
// Hold the current state for the application.
//
//*****************************************************************************
volatile enum
{
    //
    // No device is present.
    //
    STATE_NO_DEVICE,

    //
    // Mass storage device is being enumerated.
    //
    STATE_DEVICE_ENUM,
}
g_eState;

//*****************************************************************************
//
// The instance data for the MSC driver.
//
//*****************************************************************************
unsigned long g_ulMSCInstance = 0;

//*****************************************************************************
//
// The size of the host controller's memory pool in bytes.
//
//*****************************************************************************
#define HCD_MEMORY_SIZE         128

//*****************************************************************************
//
// The memory pool to provide to the Host controller driver.
//
//*****************************************************************************
unsigned char g_pHCDPool[HCD_MEMORY_SIZE];

//*****************************************************************************
//
// A buffer for holding sectors read from the storage device
//
//*****************************************************************************
static unsigned char g_ucSectorBuf[512];

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
    for(;;)
    {
    }
}
#endif

//*****************************************************************************
//
// Read a sector from the USB mass storage device.
//
// \param ulSector is the sector to read from the connected USB mass storage
// device (memory stick)
// \param pucBuf is a pointer to the buffer where the sector data should be
// stored
//
// This is the application-specific implementation of a function to read
// sectors from a storage device, in this case a USB mass storage device.
// This function is called from the \e simple_fs.c file when it needs to read
// data from the storage device.
//
// \return Non-zero if data was read from the device, 0 if no data was read.
//
//*****************************************************************************
unsigned long
SimpleFsReadMediaSector(unsigned long ulSector, unsigned char *pucBuf)
{
    //
    // Return the requested sector from the connected USB mass storage
    // device.
    //
    return(USBHMSCBlockRead(g_ulMSCInstance, ulSector, pucBuf, 1));
}

//*****************************************************************************
//
// This is the callback from the MSC driver.
//
// \param ulInstance is the driver instance which is needed when communicating
// with the driver.
// \param ulEvent is one of the events defined by the driver.
// \param pvData is a pointer to data passed into the initial call to register
// the callback.
//
// This function handles callback events from the MSC driver.  The only events
// currently handled are the \b MSC_EVENT_OPEN and \b MSC_EVENT_CLOSE.  This
// allows the main routine to know when an MSC device has been detected and
// enumerated and when an MSC device has been removed from the system.
//
// \return Returns \e true on success or \e false on failure.
//
//*****************************************************************************
static void
MSCCallback(unsigned long ulInstance, unsigned long ulEvent, void *pvData)
{
    //
    // Determine the event.
    //
    switch(ulEvent)
    {
        //
        // Called when the device driver has successfully enumerated an MSC
        // device.
        //
        case MSC_EVENT_OPEN:
        {
            //
            // Proceed to the enumeration state.
            //
            g_eState = STATE_DEVICE_ENUM;
            break;
        }

        //
        // Called when the device driver has been unloaded due to error or
        // the device is no longer present.
        //
        case MSC_EVENT_CLOSE:
        {
            //
            // Go back to the "no device" state and wait for a new connection.
            //
            g_eState = STATE_NO_DEVICE;
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
// Read the application image from the file system and program it into flash.
//
// This function will attempt to open and read the firmware image file from
// the mass storage device.  If the file is found it will be programmed into
// flash.  The name of the file to be read is configured by the macro
// \b USB_UPDATE_FILENAME.  It will be programmed into flash starting at the
// address specified by APP_START_ADDRESS.
//
// \return Zero if successful or non-zero if the file cannot be read or
// programmed.
//
//*****************************************************************************
unsigned long
ReadAppAndProgram(void)
{
    unsigned long ulFlashEnd;
    unsigned long ulFileSize;
    unsigned long ulDataSize;
    unsigned long ulRemaining;
    unsigned long ulProgAddr;
    unsigned long ulSavedRegs[2];
    unsigned long ulRetry;
    volatile unsigned long ulIdx;

    //
    // Check to see if the mass storage device is ready. Return error
    // indication if not ready.  Some large drives take a while to be
    // ready, so retry a few times before giving up.
    //
    ulRetry = 5;
    while(USBHMSCDriveReady(g_ulMSCInstance))
    {
        //
        // Kill some time before retrying
        //
        for(ulIdx = 0; ulIdx < 50000; ulIdx++)
        {
        }

        //
        // If number of retries is done then just give up and return error
        //
        ulRetry--;
        if(ulRetry == 0)
        {
            return(1);
        }
    }

    //
    // Initialize the file system and return if error.
    //
    if(SimpleFsInit(g_ucSectorBuf))
    {
        return(1);
    }

    //
    // Attempt to open the firmware file, retrieving the file/image size.
    // A file size of error means the file was not there, or there was an
    // error.
    //
    ulFileSize = SimpleFsOpen(USB_UPDATE_FILENAME);
    if(ulFileSize == 0)
    {
        return(1);
    }

    //
    // Get the size of flash.  This is the ending address of the flash.
    // If reserved space is configured, then the ending address is reduced
    // by the amount of the reserved block.
    //
    ulFlashEnd = FLASH_SIZE;
#ifdef FLASH_RSVD_SPACE
    ulFlashEnd -= FLASH_RSVD_SPACE;
#endif

    //
    // If flash code protection is not used, then change the ending address
    // to be the ending of the application image.  This will be the highest
    // flash page that will be erased and so only the necessary pages will
    // be erased.  If flash code protection is enabled, then all of the
    // application area pages will be erased.
    //
#ifndef FLASH_CODE_PROTECTION
    ulFlashEnd = ulFileSize + APP_START_ADDRESS;
#endif

    //
    // Check to make sure the file size is not too large to fit in the flash.
    // If it is too large, then return an error.
    //
    if((ulFileSize + APP_START_ADDRESS) > ulFlashEnd)
    {
        return(1);
    }

    //
    // Enter a loop to erase all the requested flash pages beginning at the
    // application start address (above the USB stick updater).
    //
    for(ulIdx = APP_START_ADDRESS; ulIdx < ulFlashEnd; ulIdx += 1024)
    {
        ROM_FlashErase(ulIdx);
    }

    //
    // Enter a loop to read sectors from the application image file and
    // program into flash.  Start at the user app start address (above the USB
    // stick updater).
    //
    ulProgAddr = APP_START_ADDRESS;
    ulRemaining = ulFileSize;
    while(SimpleFsReadFileSector())
    {
        //
        // Compute how much data was read from this sector and adjust the
        // remaining amount.
        //
        ulDataSize = ulRemaining >= 512 ? 512 : ulRemaining;
        ulRemaining -= ulDataSize;

        //
        // Special handling for the first block of the application.
        // This block contains as the first two location the applications's
        // initial stack pointer and instruction pointer.  The USB updater
        // relied on the values in these locations to determine if a valid
        // application is present.  When there is a valid application the
        // updater runs the user application.  Otherwise the updater attempts
        // to load a new application.
        // In order to prevent a partially programmed imaged (due to some
        // error occurring while programming), the first two locations are
        // not programmed until all of the rest of the image has been
        // successfully loaded into the flash.  This way if there is some error,
        // the updater will detect that a user application is not present and
        // will not attempt to run it.
        //
        // Save the first two words of the image and replace them with all Fs.
        // These two words will be programmed later, after everything else.
        //
        if(ulProgAddr == APP_START_ADDRESS)
        {
            ulSavedRegs[0] = ((unsigned long *)g_ucSectorBuf)[0];
            ulSavedRegs[1] = ((unsigned long *)g_ucSectorBuf)[1];
            ((unsigned long *)g_ucSectorBuf)[0] = 0xFFFFFFFF;
            ((unsigned long *)g_ucSectorBuf)[1] = 0xFFFFFFFF;
        }

        //
        // Call the function to program a block of flash.  The length of the
        // block passed to the flash function must be divisible by 4.
        //
        ROM_FlashProgram((unsigned long *)g_ucSectorBuf, ulProgAddr,
                         (ulDataSize + 3) & ~3);

        //
        // If there is more image to program, then update the programming
        // address.  Progress will continue to the next iteration of
        // the while loop.
        //
        if(ulRemaining)
        {
            ulProgAddr += ulDataSize;
        }

        //
        // Otherwise we are done programming so perform final steps.
        // Program the first two words of the image that were saved earlier,
        // and return a success indication.
        //
        else
        {
            ROM_FlashProgram((unsigned long *)ulSavedRegs, APP_START_ADDRESS,
                              8);

            return(0);
        }
    }

    //
    // If we make it here, that means that an attempt to read a sector of
    // data from the device was not successful.  That means that the complete
    // user app has not been programmed into flash, so just return an error
    // indication.
    //
    return(1);
}

//*****************************************************************************
//
// This is the main routine for performing an update from a mass storage
// device.
//
// This function forms the main loop of the USB stick updater.  It polls for
// a USB mass storage device to be connected,  Once a device is connected
// it will attempt to read a firmware image from the device and load it into
// flash.
//
// \return None.
//
//*****************************************************************************
void
UpdaterUSB(void)
{
    //
    // Loop forever, running the USB host driver.
    //
    while(1)
    {
        USBHCDMain();

        //
        // Check for a state change from the USB driver.
        //
        switch(g_eState)
        {
            //
            // This state means that a mass storage device has been
            // plugged in and enumerated.
            //
            case STATE_DEVICE_ENUM:
            {
                //
                // Attempt to read the application image from the storage
                // device and load it into flash memory.
                //
                if(ReadAppAndProgram())
                {
                    //
                    // There was some error reading or programming the app,
                    // so reset the state to no device which will cause a
                    // wait for a new device to be plugged in.
                    //
                    g_eState = STATE_NO_DEVICE;
                }
                else
                {
                    //
                    // User app load and programming was successful, so reboot
                    // the micro.  Perform a software reset request.  This
                    // will cause the microcontroller to reset; no further
                    // code will be executed.
                    //
                    HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY |
                                        NVIC_APINT_SYSRESETREQ;

                    //
                    // The microcontroller should have reset, so this should
                    // never be reached.  Just in case, loop forever.
                    //
                    while(1)
                    {
                    }
                }
                break;
            }

            //
            // This state means that there is no device present, so just
            // do nothing until something is plugged in.
            //
            case STATE_NO_DEVICE:
            {
                break;
            }
        }
    }
}

//*****************************************************************************
//
// Configure the USB controller and power the bus.
//
// This function configures the USB controller for host operation.
// It is assumed that the main system clock has been configured at this point.
//
// \return None.
//
//*****************************************************************************
void
ConfigureUSBInterface(void)
{
    //
    // Enable the uDMA controller and set up the control table base.
    // This is required by usblib.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    uDMAEnable();
    uDMAControlBaseSet(g_sDMAControlTable);

    //
    // Enable the USB controller.
    //
    HWREG(SYSCTL_RCGC2) |= 0x10000;

    //
    // Set the USB pins to be controlled by the USB controller.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    HWREG(SYSCTL_RCGC2) |= 0x10000;     // kill a few cycles
    HWREG(GPIO_PORTH_BASE + GPIO_O_AFSEL) |= GPIO_PIN_3 | GPIO_PIN_4;
    HWREG(GPIO_PORTH_BASE + GPIO_O_DEN) |= GPIO_PIN_3 | GPIO_PIN_4;

    //
    // The LM3S3748 board uses a USB mux that must be switched to use the
    // host connector and not the device connector.
    //
    ROM_GPIOPinTypeGPIOOutput(USB_MUX_GPIO_BASE, USB_MUX_GPIO_PIN);
    ROM_GPIOPinWrite(USB_MUX_GPIO_BASE, USB_MUX_GPIO_PIN, USB_MUX_SEL_HOST);

    //
    // Register the host class driver
    //
    USBHCDRegisterDrivers(0, g_ppHostClassDrivers, NUM_CLASS_DRIVERS);

    //
    // Open an instance of the mass storage class driver.
    //
    g_ulMSCInstance = USBHMSCDriveOpen(0, MSCCallback);

    //
    // Initialize the power configuration. This sets the power enable signal
    // to be active high and does not enable the power fault.
    //
    USBHCDPowerConfigInit(0, USB_HOST_PWREN_HIGH);

    //
    // Initialize the host controller.
    //
    USBHCDInit(0, g_pHCDPool, HCD_MEMORY_SIZE);
}

//*****************************************************************************
//
// Generic configuration is handled in this function.
//
// This function is called by the start up code to perform any configuration
// necessary before calling the update routine.  It is responsible for setting
// the system clock to the expected rate and setting flash programming
// parameters prior to calling ConfigureUSBInterface() to set up the USB
// hardware.
//
// \return None.
//
//*****************************************************************************
void
UpdaterMain(void)
{
    //
    // Make sure NVIC points at the correct vector table.
    //
    HWREG(NVIC_VTABLE) = 0;

    //
    // Set up system clock to use PLL and run at 25 MHz
    //
    ROM_SysCtlClockSet(SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_8MHZ | SYSCTL_SYSDIV_8);

    //
    // Set the flash programming based on the 25MHz system clock.
    //
    HWREG(FLASH_USECRL) = (25 - 1);

    //
    // Configure the USB interface and power the bus.
    //
    ConfigureUSBInterface();

    //
    // Call the updater function.  This will attempt to load a new image
    // into flash from a USB memory stick.
    //
    UpdaterUSB();
}

//*****************************************************************************
//
// Main entry point for the USB stick update example.
//
// This function will check to see if a flash update should be performed from
// the USB memory stick, or if the user application should just be run without
// any update.
//
// The following checks are made, any of which mean that an update should be
// performed:
// - the PC and SP for the user app do not appear to be valid
// - a memory location contains a certain value, meaning the user app wants
//   to force an update
// - the user button on the eval board is being pressed, meaning the user wants
//   to force an update even if there is a valid user app in memory
//
// If any of the above checks are true, then that means that an update should
// be attempted.  The USB stick updater will then wait for a USB stick to be
// plugged in, and once it is look for a firmware update file.
//
// If none of the above checks are true, then the user application that is
// already in flash is run and no update is performed.
//
// \return None.
//
//*****************************************************************************
int
main(void)
{
    unsigned long *pulApp;

    //
    // See if the first location is 0xfffffffff or something that does not
    // look like a stack pointer, or if the second location is 0xffffffff or
    // something that does not look like a reset vector.
    //
    pulApp = (unsigned long *)APP_START_ADDRESS;
    if((pulApp[0] == 0xffffffff) || ((pulApp[0] & 0xfff00000) != 0x20000000) ||
       (pulApp[1] == 0xffffffff) || ((pulApp[1] & 0xfff00001) != 0x00000001))
    {
        //
        // App starting stack pointer or PC is not valid, so force an update.
        //
        UpdaterMain();
    }

    //
    // Check to see if the application has requested an update
    //
    if(HWREG(FORCE_UPDATE_ADDR) == FORCE_UPDATE_VALUE)
    {
        HWREG(FORCE_UPDATE_ADDR) = 0;
        UpdaterMain();
    }

    //
    // Enable the GPIO input for the user button.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_DIR_MODE_IN);
    ROM_GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_7,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //
    // Check if the button is pressed, if so then force an update.
    //
    if(GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_7) == 0)
    {
        UpdaterMain();
    }

    //
    // If we get to here that means that none of the conditions that should
    // cause an update are true.  Therefore, call the application.
    //
    CallApplication(APP_START_ADDRESS);
}

//*****************************************************************************
//
// This function is called from the application to request an update.  The
// address of this function is stored in the SVC vector at offset 0x2C, so
// the user application can call it by using a statement like this:
//
//      (*((void (*)(void))(*(unsigned long *)0x2c)))();
//
//*****************************************************************************
void
AppForceUpdate(void)
{
    //
    // Set a value in a memory location to indicate that the app requests
    // an update.  Then cause the processor to reset.
    //
    HWREG(FORCE_UPDATE_ADDR) = FORCE_UPDATE_VALUE;
    HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
}

//*****************************************************************************
//
// This function is used to call the user application.  It will set the NVIC
// to point at the user app's vector table, load up the user app's stack
// pointer, and then jump to the application.
//
// This function must be programmed in assembly since it needs to directly
// manipulate the value in the stack pointer, and because it needs to perform
// a direct branch to the user app and not a function call (bl).
//
//*****************************************************************************
#if defined(codered) || defined(gcc) || defined(sourcerygxx)
void __attribute__((naked))
CallApplication(unsigned long ulStartAddr)
{
    //
    // Set the vector table to the beginning of the app in flash.
    //
    HWREG(NVIC_VTABLE) = ulStartAddr;

    //
    // Load the stack pointer from the application's vector table.
    //
    __asm("    ldr     r1, [r0]\n"
          "    mov     sp, r1");

    //
    // Load the initial PC from the application's vector table and branch to
    // the application's entry point.
    //
    __asm("    ldr     r0, [r0, #4]\n"
          "    bx      r0\n");
}
#endif
#if defined(ewarm)
void
CallApplication(unsigned long ulStartAddr)
{
    //
    // Set the vector table to the beginning of the app in flash.
    //
    HWREG(NVIC_VTABLE) = ulStartAddr;

    //
    // Load the stack pointer from the application's vector table.
    //
    __asm("    ldr     r1, [r0]\n"
          "    mov     sp, r1");

    //
    // Load the initial PC from the application's vector table and branch to
    // the application's entry point.
    //
    __asm("    ldr     r0, [r0, #4]\n"
          "    bx      r0\n");
}
#endif
#if defined(rvmdk) || defined(__ARMCC_VERSION)
__asm void
CallApplication(unsigned long ulStartAddr)
{
    //
    // Set the vector table address to the beginning of the application.
    //
    ldr     r1, =0xe000ed08
    str     r0, [r1]

    //
    // Load the stack pointer from the application's vector table.
    //
    ldr     r1, [r0]
    mov     sp, r1

    //
    // Load the initial PC from the application's vector table and branch to
    // the application's entry point.
    //
    ldr     r0, [r0, #4]
    bx      r0
}
#endif
