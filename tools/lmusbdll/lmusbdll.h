// usbdll.h : main header file for the usbdll DLL
//

#ifdef __cplusplus

#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

//
// Functions exported by this DLL.
//
extern "C" {
#endif

typedef void *LMUSB_HANDLE;

LMUSB_HANDLE __stdcall InitializeDevice(unsigned short usVID,
                                        unsigned short usPID,
                                        LPGUID lpGUID,
                                        BOOL *pbDriverInstalled);
BOOL __stdcall TerminateDevice(LMUSB_HANDLE hHandle);
BOOL __stdcall WriteUSBPacket(LMUSB_HANDLE hHandle,
                              unsigned char *pcBuffer,
                              unsigned long ulSize,
                              unsigned long *pulWritten);
DWORD __stdcall ReadUSBPacket(LMUSB_HANDLE hHandle,
                             unsigned char *pcBuffer,
                             unsigned long ulSize,
                             unsigned long *pulRead,
                             unsigned long ulTimeoutMs,
                             HANDLE hBreak);

#ifdef __cplusplus
}
#endif

