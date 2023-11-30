//*****************************************************************************
//
//! @file am_hal_mram.h
//!
//! @brief BootROM Helper Function Table
//!
//! @addtogroup mram4_4b MRAM Functionality
//! @ingroup apollo4b_hal
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2023, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision release_sdk_4_4_1-7498c7b770 of the AmbiqSuite Development Package.
//
//*****************************************************************************
#ifndef AM_HAL_MRAM_H
#define AM_HAL_MRAM_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
//
// MRAM Program keys.
//
//*****************************************************************************
#define AM_HAL_MRAM_PROGRAM_KEY            0x12344321
#define AM_HAL_MRAM_INFO_KEY               0xD894E09E

//*****************************************************************************
//
// Some helpful SRAM values and macros.
//
//*****************************************************************************
#define AM_HAL_MRAM_SRAM_ADDR                  SRAM_BASEADDR
#define AM_HAL_MRAM_SRAM_SIZE                  (1024 * 1024)
#define AM_HAL_MRAM_SRAM_LARGEST_VALID_ADDR    (AM_HAL_MRAM_SRAM_ADDR + AM_HAL_MRAM_SRAM_SIZE - 1)
#define AM_HAL_MRAM_DTCM_START                 AM_HAL_MRAM_SRAM_ADDR
#define AM_HAL_MRAM_DTCM_END                   (AM_HAL_MRAM_SRAM_ADDR + (384 * 1024) - 1)

//*****************************************************************************
//
// Some helpful mram values and macros.
//
//*****************************************************************************
#define AM_HAL_MRAM_ADDR                   MRAM_BASEADDR
#define AM_HAL_MRAM_INSTANCE_SIZE          ( 1 * 1024 * 1024 )
#define AM_HAL_MRAM_NUM_INSTANCES          2
#define AM_HAL_MRAM_TOTAL_SIZE             ( AM_HAL_MRAM_INSTANCE_SIZE * AM_HAL_MRAM_NUM_INSTANCES )
#define AM_HAL_MRAM_LARGEST_VALID_ADDR     ( AM_HAL_MRAM_ADDR + AM_HAL_MRAM_TOTAL_SIZE - 1 )

#define AM_HAL_INFO0_SIZE_BYTES            (2 * 1024)
#define AM_HAL_INFO1_SIZE_BYTES            (6 * 1024)
#define AM_HAL_INFO1_VISIBLE_OFFSET        (0x1200)

//*****************************************************************************
//
//! @brief This programs up to N words of the Main MRAM
//!
//! @param ui32ProgramKey - The programming key, AM_HAL_MRAM_PROGRAM_KEY.
//! @param pui32Src - Pointer to word aligned array of data to program into
//! the MRAM.
//! @param pui32Dst - Pointer to the 4 words (16 byte) aligned MRAM location where
//! programming of the MRAM is to begin.
//! @param ui32NumWords - The number of words to be programmed. This MUST be
//! a multiple of 4 (16 byte multiple)
//!
//! This function will program multiple 4 words (16 byte) tuples in main MRAM.
//!
//! @note THIS FUNCTION ONLY OPERATES ON 16 BYTE BLOCKS OF MAIN MRAM. The pDst
//! MUST be 4 words (16 byte) aligned, and ui32NumWords MUST be multiple of 4.
//!
//! @return 0 for success, non-zero for failure.
//!     Failing return code indicates:
//!    -1   invalid alignment for pui32Dst or ui32NumWords not 16 byte multiple
//!     1   ui32ProgramKey is invalid.
//!     2   pui32Dst is invalid.
//!     3   Flash addressing range would be exceeded.  That is, (pui32Dst +
//!         (ui32NumWords * 4)) is greater than the last valid address.
//!     4   pui32Src is invalid.
//!     5   pui32Src is invalid.
//!     6   Flash controller hardware timeout.
//
//*****************************************************************************
extern int am_hal_mram_main_program(uint32_t ui32ProgramKey, uint32_t *pui32Src,
                            uint32_t *pui32Dst, uint32_t ui32NumWords);
//*****************************************************************************
//
//! @brief This Fills up to N words of the Main MRAM
//!
//! @param ui32ProgramKey - The programming key, AM_HAL_MRAM_PROGRAM_KEY.
//! @param ui32Value - 32-bit data value to fill into the MRAM
//! @param pui32Dst - Pointer to the 4 words (16 byte) aligned MRAM location where
//! programming of the MRAM is to begin.
//! @param ui32NumWords - The number of words to be programmed. This MUST be
//! a multiple of 4 (16 byte multiple)
//!
//! This function will fill multiple 16 byte tuples in main MRAM with specified pattern.
//!
//! @note THIS FUNCTION ONLY OPERATES ON 16 BYTE BLOCKS OF MAIN MRAM. The pDst
//! MUST be 4 words (16 byte) aligned, and ui32NumWords MUST be multiple of 4.
//!
//! @return 0 for success, non-zero for failure.
//!    -1   invalid alignment for pui32Dst or ui32NumWords not 16 byte multiple
//!     1   ui32InfoKey is invalid.
//!     2   ui32Offset is invalid.
//!     3   addressing range would be exceeded.  That is, (ui32Offset +
//!         (ui32NumWords * 4)) is greater than the last valid address.
//!     4   pui32Src is invalid.
//!     5   pui32Src is invalid.
//!     6   Flash controller hardware timeout.
//
//*****************************************************************************
extern int am_hal_mram_main_fill(uint32_t ui32ProgramKey, uint32_t ui32Value,
                            uint32_t *pui32Dst, uint32_t ui32NumWords);

//*****************************************************************************
//
//! @brief This programs up to N words of the Main array on one MRAM.
//!
//! @param ui32InfoKey - The programming key, AM_HAL_MRAM_INFO_KEY.
//! @param pui32Src - Pointer to word aligned array of data to program into
//! INFO0
//! @param ui32Offset - Pointer to the word aligned INFO0 offset where
//! programming of the INFO0 is to begin.
//! @param ui32NumWords - The number of words to be programmed.
//!
//! This function will program multiple words in INFO0
//!
//! @return 0 for success, non-zero for failure.
//!     1   ui32InfoKey is invalid.
//!     2   ui32Offset is invalid.
//!     3   addressing range would be exceeded.  That is, (ui32Offset +
//!         (ui32NumWords * 4)) is greater than the last valid address.
//!     4   pui32Src is invalid.
//!     5   pui32Src is invalid.
//!     6   Flash controller hardware timeout.
//
//*****************************************************************************
extern int am_hal_mram_info_program(uint32_t ui32InfoKey, uint32_t *pui32Src,
                            uint32_t ui32Offset, uint32_t ui32NumWords);

//*****************************************************************************
//
//! @brief This function erases main MRAM + customer INFO space
//!
//! @param ui32BrickKey - The Brick key.
//!
//! This function erases main MRAM and customer INFOinstance
//! even if the customer INFO space is programmed to not be erasable. This
//! function completely erases the MRAM main and info instance and wipes the
//! SRAM. Upon completion of the erasure operations, it does a POI (power on
//! initialization) reset.
//!
//! @note This function enforces 128 bit customer key lock. The caller needs to assert
//! the Recovery Lock using am_hal_security_set_key() providing appropriate key.
//! Otherwise, the function will fail.  Therefore, always check for a return code.
//! If the function returns, a failure has occured.
//!
//! @return Does not return if successful.  Returns failure code otherwise.
//!     Failing return code indicates:
//!     0x00000001  ui32BrickKey is invalid.
//!     0x00000002  Recovery key lock not set.
//!     Other values indicate Internal error.
//
//*****************************************************************************
extern int am_hal_mram_recovery(uint32_t ui32BrickKey);

//*****************************************************************************
//
//! @brief This programs up to N words of the Main MRAM
//!
//! @param ui32ProgramKey - The programming key, AM_HAL_MRAM_PROGRAM_KEY.
//! @param pui32Src - Pointer to word aligned array of data to program into
//! the MRAM.
//! @param pui32Dst - Pointer to the words (4 byte) aligned MRAM location where
//! programming of the MRAM is to begin.
//! @param ui32NumWords - The number of words to be programmed.
//!
//! This function will program multiple words (4 byte) tuples in main MRAM.
//!
//! @note This function is provided only for convenience. It is most efficient
//! to operate on MRAM in be 4 words (16 byte) aligned multiples. Doing word
//! access will be very inefficient and should be avoided.
//!
//! @return 0 for success, non-zero for failure.
//!     Failing return code indicates:
//!    -1   invalid alignment for pui32Dst or ui32NumWords not 16 bye multiple
//!     1   ui32ProgramKey is invalid.
//!     2   pui32Dst is invalid.
//!     3   Flash addressing range would be exceeded.  That is, (pui32Dst +
//!         (ui32NumWords * 4)) is greater than the last valid address.
//!     4   pui32Src is invalid.
//!     5   pui32Src is invalid.
//!     6   Flash controller hardware timeout.
//
//*****************************************************************************
extern int am_hal_mram_main_words_program(uint32_t ui32ProgramKey, uint32_t *pui32Src,
                            uint32_t *pui32Dst, uint32_t ui32NumWords);

//*****************************************************************************
//
//! @brief Read INFO data.
//!
//! This function implements a workaround required for Apollo4 B0 parts in
//! order to accurately read INFO space.
//!
//! @param ui32InfoSpace - 0 = Read INFO0.
//!                        1 = Only valid to read the customer visible area
//!                            of INFO1. If INFO1, the ui32WordOffset argument
//!                            must be 0x480 or greater (byte offset 0x1200).
//! @param ui32WordOffset - Desired word offset into INFO space.
//! @param ui32NumWords  - The number of words to be retrieved.
//! @param pui32Dst      - Pointer to the location where the INFO data
//!                        is to be copied to.
//!
//! @return 0 for success, non-zero for failure.                                                6
//
//*****************************************************************************
extern int am_hal_mram_info_read(uint32_t ui32InfoSpace, uint32_t ui32Offset,
                                 uint32_t ui32NumWords, uint32_t *pui32Dst);

//*****************************************************************************
//
//! @brief Initialize MRAM for DeepSleep.
//!
//! This function implements a workaround required for Apollo4 B0 parts in
//! order to fix the MRAM DeepSleep config params.
//!
//! @return 0 for success, non-zero for failure.                                                6
//
//*****************************************************************************
extern int am_hal_mram_ds_init(void);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_MRAM_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

