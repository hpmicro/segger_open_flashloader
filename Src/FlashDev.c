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

File    : FlashDev.c
Purpose : Flash device description Template
*/

#include "FlashOS.h"

struct SECTOR_INFO  {
  U32 SectorSize;       // Sector Size in bytes
  U32 SectorStartAddr;  // Start address of the sector area (relative to the "BaseAddr" of the flash)
};

struct FlashDevice {
   U16 AlgoVer;       // Algo version number
   U8  Name[128];     // Flash device name. NEVER change the size of this array!
   U16 Type;          // Flash device type
   U32 BaseAddr;      // Flash base address
   U32 TotalSize;     // Total flash device size in Bytes (256 KB)
   U32 PageSize;      // Page Size (number of bytes that will be passed to ProgramPage(). MinAlig is 8 byte
   U32 Reserved;      // Reserved, should be 0
   U8  ErasedVal;     // Flash erased value
   U32 TimeoutProg;   // Program page timeout in ms
   U32 TimeoutErase;  // Erase sector timeout in ms
   struct SECTOR_INFO SectorInfo[4]; // Flash sector layout definition. May be adapted up to 512 entries
};

struct FlashDevice const FlashDevice __attribute__ ((section ("DevDscr"))) = {
  0x0101,                         // Algo version. Must be == 0x0101
  { "HPM XPI Nor Flash" },  // Flash device name
  1,                              // Flash device type. Must be == 1
  0x80000000,                     // Flash base address
  XPI_FLASH_SIZE,                     // Total flash device size in Bytes
  256,                            // Page Size (Will be passed as <NumBytes> to ProgramPage(). A multiple of this is passed as <NumBytes> to SEGGER_OPEN_Program() to program moer than 1 page in 1 RAMCode call, speeding up programming).
  0,                              // Reserved, should be 0
  0xFF,                           // Flash erased value
  100,                            // Program page timeout in ms
  6000,                           // Erase sector timeout in ms
  //
  // Flash sector layout definition
  // Keil / CMSIS does not do a great job in explaining these...
  // The logic is as follows:
  // For flashes with uniform sectors, you need exactly 1 entry here: <SectorSize>, <SectorStartOff> (rel. to <FlashBaseAddr>)
  // For a flash with 512 sectors, the entry would be: 0x200, 0x0
  //
  // When having a flash with 3 different  sector sizes, like: 4 * 16 KB, 1 * 64 KB, 1 * 128 KB
  // you will have 3 entries here:
  // 0x4000, 0x0            4 *  16 KB =  64 KB
  // 0x10000, 0x10000       1 *  64 KB =  64 KB
  // 0x20000, 0x20000       1 * 128 KB = 128 KB
  //
  {
    { 0x00001000, 0x00000000 },   // 4 *  16 KB =  64 KB
    { 0xFFFFFFFF, 0xFFFFFFFF }    // Indicates the end of the flash sector layout. Must be present.
  }
};