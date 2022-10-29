/***********************************************************************
*                    SEGGER Microcontroller GmbH                       *
*                        The Embedded Experts                          *
************************************************************************
*                                                                      *
*                  (c) SEGGER Microcontroller GmbH                     *
*                        All rights reserved                           *
*                          www.segger.com                              *
*                                                                      *
************************************************************************
*                                                                      *
************************************************************************
*                                                                      *
*                                                                      *
*  Licensing terms                                                     *
*                                                                      *
* Redistribution and use in source and binary forms, with or without   *
* modification, are permitted provided that the following conditions   *
* are met:                                                             *
*                                                                      *
* 1. Redistributions of source code must retain the above copyright    *
* notice, this list of conditions and the following disclaimer.        *
*                                                                      *
* 2. Redistributions in binary form must reproduce the above           *
* copyright notice, this list of conditions and the following          *
* disclaimer in the documentation and/or other materials provided      *
* with the distribution.                                               *
*                                                                      *
*                                                                      *
* THIS SOFTWARE IS PROVIDED BY COPYRIGHT HOLDER "AS IS" AND ANY        *
* EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE    *
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR   *
* PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE        *
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,     *
* OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,             *
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR   *
* PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY  *
* OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT         *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE    *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH.    *
* DAMAGE.                                                              *
*                                                                      *
************************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : FlashOS.h
Purpose : Contains all defines and prototypes of public functions.
*/

#define U8  unsigned char
#define U16 unsigned short
#define U32 unsigned long

#define I8  signed char
#define I16 signed short
#define I32 signed long

struct SEGGER_OPEN_CMD_INFO;  // Forward declaration of OFL lib private struct

typedef struct {
  //
  // Optional functions may be be NULL
  //
  void (*pfFeedWatchdog)   (void);                                            // Optional
  int  (*pfInit)           (U32 Addr, U32 Freq, U32 Func);                    // Mandatory
  int  (*pfUnInit)         (U32 Func);                                        // Mandatory
  int  (*pfEraseSector)    (U32 Addr);                                        // Mandatory
  int  (*pfProgramPage)    (U32 Addr, U32 NumBytes, U8 *pSrcBuff);            // Mandatory
  int  (*pfBlankCheck)     (U32 Addr, U32 NumBytes, U8 BlankData);            // Optional
  int  (*pfEraseChip)      (void);                                            // Optional
  U32  (*pfVerify)         (U32 Addr, U32 NumBytes, U8 *pSrcBuff);            // Optional
  U32  (*pfSEGGERCalcCRC)  (U32 CRC, U32 Addr, U32 NumBytes, U32 Polynom);    // Optional
  int  (*pfSEGGERRead)     (U32 Addr, U32 NumBytes, U8 *pDestBuff);           // Optional
  int  (*pfSEGGERProgram)  (U32 DestAddr, U32 NumBytes, U8 *pSrcBuff);        // Optional
  int  (*pfSEGGERErase)    (U32 SectorAddr, U32 SectorIndex, U32 NumSectors); // Optional
  void (*pfSEGGERStart)    (volatile struct SEGGER_OPEN_CMD_INFO* pInfo);     // Optional
} SEGGER_OFL_API;

//
// Keil / CMSIS API
//
int  Init                 (U32 Addr, U32 Freq, U32 Func);                    // Mandatory
int  UnInit               (U32 Func);                                        // Mandatory
int  EraseSector          (U32 Addr);                                        // Mandatory
int  ProgramPage          (U32 Addr, U32 NumBytes, U8 *pSrcBuff);            // Mandatory
int  BlankCheck           (U32 Addr, U32 NumBytes, U8 BlankData);            // Optional
int  EraseChip            (void);                                            // Optional
U32  Verify               (U32 Addr, U32 NumBytes, U8 *pSrcBuff);            // Optional
//
// SEGGER extensions
//
U32  SEGGER_OPEN_CalcCRC  (U32 CRC, U32 Addr, U32 NumBytes, U32 Polynom);    // Optional
int  SEGGER_OPEN_Read     (U32 Addr, U32 NumBytes, U8 *pDestBuff);           // Optional
int  SEGGER_OPEN_Program  (U32 DestAddr, U32 NumBytes, U8 *pSrcBuff);        // Optional
int  SEGGER_OPEN_Erase    (U32 SectorAddr, U32 SectorIndex, U32 NumSectors); // Optional
void SEGGER_OPEN_Start    (volatile struct SEGGER_OPEN_CMD_INFO* pInfo);     // Optional
//
// SEGGER OFL lib helper functions that may be called by specific algo part
//
U32  SEGGER_OFL_Lib_CalcCRC     (const SEGGER_OFL_API* pAPI, U32 CRC, U32 Addr, U32 NumBytes, U32 Polynom);
void SEGGER_OFL_Lib_StartTurbo  (const SEGGER_OFL_API* pAPI, volatile struct SEGGER_OPEN_CMD_INFO* pInfo);

/**************************** End of file ***************************/
