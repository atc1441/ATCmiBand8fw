//*****************************************************************************
//
//! @file am_hal_mram.c
//!
//! @brief BootROM Helper Function Table
//!
//! @addtogroup mram4_4p MRAM Functionality
//! @ingroup apollo4p_hal
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

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_hal_bootrom_helper.h"


#define HLPRFNC_MAXSRC_ADDR     0x101D7FFC // Maximum src size for Apollo4 Plus

//#define INFO_ACCESS_WA


#define MRAM_OVERRIDE()
#define MRAM_REVERT()
#define MRAM_RECOVER()


//*****************************************************************************
//
// This programs up to N words of the Main MRAM
//
//*****************************************************************************
int
am_hal_mram_main_words_program(uint32_t ui32ProgramKey, uint32_t *pui32Src,
                               uint32_t *pui32Dst, uint32_t ui32NumWords)
{
    //
    // Use the new helper function to efficiently program the data in MRAM.
    //

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the Source to ensure proper location.
    //
    if ( ((uint32_t)pui32Src & 0xFF000000) == SRAM_BASEADDR )
    {
        //
        // Check that the source buffer is within bounds (see errata ERR122)
        //
        if ( ((uint32_t)pui32Src + (ui32NumWords * 4)) > HLPRFNC_MAXSRC_ADDR )
        {
            return AM_HAL_STATUS_OUT_OF_RANGE;
        }
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    if ( (uint32_t)pui32Dst <= AM_HAL_MRAM_LARGEST_VALID_ADDR )
    {
        //
        // This helper function requires a word offset rather than an actual
        // address. Since MRAM addresses start at 0x0, we can convert the addr
        // into a word offset by simply dividing the destination address by 4.
        //
        pui32Dst = (uint32_t*)((uint32_t)pui32Dst / 4);
    }

    return g_am_hal_bootrom_helper.nv_program_main2(ui32ProgramKey, 1,
                                                    (uint32_t)pui32Src,
                                                    (uint32_t)pui32Dst,
                                                    ui32NumWords);
}

//*****************************************************************************
//
// This programs up to N words of the Main MRAM
//
//*****************************************************************************
int
am_hal_mram_main_program(uint32_t ui32ProgramKey, uint32_t *pui32Src,
                         uint32_t *pui32Dst, uint32_t ui32NumWords)
{
    //
    // Check for pui32Dst & ui32NumWords
    //
    if (((uint32_t)pui32Dst & 0xf) || (ui32NumWords & 0x3))
    {
        return -1;
    }

    return am_hal_mram_main_words_program(ui32ProgramKey, pui32Src,
                                          pui32Dst, ui32NumWords);
}

//*****************************************************************************
//
// This Fills up to N words of the Main MRAM
//
//*****************************************************************************
int
am_hal_mram_main_fill(uint32_t ui32ProgramKey, uint32_t ui32Value,
                      uint32_t *pui32Dst, uint32_t ui32NumWords)
{
    //
    // Check for pui32Dst & ui32NumWords
    //
    if (((uint32_t)pui32Dst & 0xf) || (ui32NumWords & 0x3))
    {
        return -1;
    }

    //
    // Use the new helper function to efficiently fill MRAM with a value.
    //

    if ( (uint32_t)pui32Dst <= AM_HAL_MRAM_LARGEST_VALID_ADDR )
    {
        //
        // This helper function requires a word offset rather than an actual
        // address. Since MRAM addresses start at 0x0, we can convert the addr
        // into a word offset by simply dividing the destination address by 4.
        //
        pui32Dst = (uint32_t*)((uint32_t)pui32Dst / 4);
    }

    return g_am_hal_bootrom_helper.nv_program_main2(ui32ProgramKey, 0,
                                                    (uint32_t)ui32Value,
                                                    (uint32_t)pui32Dst,
                                                    ui32NumWords);
}

//*****************************************************************************
//
// This programs up to N words of the Main array on one MRAM.
//
//*****************************************************************************
int
am_hal_mram_info_program(uint32_t ui32InfoKey, uint32_t *pui32Src,
                         uint32_t ui32Offset, uint32_t ui32NumWords)
{
    int retval;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the Source to ensure proper location.
    //
    if ( ((uint32_t)pui32Src & 0xFF000000) == SRAM_BASEADDR )
    {
        //
        // Check that the source buffer is within bounds (see errata ERR122)
        //
        if ( ((uint32_t)pui32Src + (ui32NumWords * 4)) > HLPRFNC_MAXSRC_ADDR )
        {
            return AM_HAL_STATUS_OUT_OF_RANGE;
        }
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    MRAM_OVERRIDE();

    retval = g_am_hal_bootrom_helper.nv_program_info_area(ui32InfoKey,
                                pui32Src, ui32Offset, ui32NumWords);

    MRAM_REVERT();

    return retval;
}

#ifdef INFO_ACCESS_WA

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
int
am_hal_mram_recovery(uint32_t ui32BrickKey)
{
    return g_am_hal_bootrom_helper.nv_recovery(ui32BrickKey);
}
#endif

//*****************************************************************************
//
// Read INFO data.
//
//*****************************************************************************
int
am_hal_mram_info_read(uint32_t ui32InfoSpace,
                      uint32_t ui32WordOffset,
                      uint32_t ui32NumWords,
                      uint32_t *pui32Dst)
{
    int retval = 0;
    uint32_t ux;
    uint32_t *pui32Info;

    if ( ui32InfoSpace == 0 )
    {
        if ( (ui32WordOffset >= (AM_HAL_INFO0_SIZE_BYTES / 4)) || ((ui32WordOffset + ui32NumWords) > (AM_HAL_INFO0_SIZE_BYTES / 4))  )
        {
            return 2;
        }
        pui32Info = (uint32_t*)(AM_REG_INFO0_BASEADDR + (ui32WordOffset * 4));
    }
    else if ( ui32InfoSpace == 1 )
    {
        if ( (ui32WordOffset < (AM_HAL_INFO1_VISIBLE_OFFSET / 4)) || (ui32WordOffset >= (AM_HAL_INFO1_SIZE_BYTES / 4)) || ((ui32WordOffset + ui32NumWords) > (AM_HAL_INFO1_SIZE_BYTES / 4))  )
        {
            return 2;
        }
        pui32Info = (uint32_t*)(AM_REG_INFO1_BASEADDR + (ui32WordOffset * 4));
    }
    else
    {
        return 1;
    }

#ifdef INFO_ACCESS_WA
    MRAM_OVERRIDE();

    //
    // Start a critical section.
    //
    AM_CRITICAL_BEGIN

    for (ux = 0; ux < ui32NumWords; ux++ )
    {
        // INFO read should use the util function - to ensure the code is not running from MRAM
        *pui32Dst++ = am_hal_load_ui32(pui32Info++);
    }

    //
    // End the critical section.
    //
    AM_CRITICAL_END

    MRAM_REVERT();
#else
    for (ux = 0; ux < ui32NumWords; ux++ )
    {
        *pui32Dst++ = AM_REGVAL(pui32Info++);
    }
#endif

    return retval;
}

//*****************************************************************************
//
// Initialize MRAM for DeepSleep.
//
//*****************************************************************************
int
am_hal_mram_ds_init(void)
{
    MRAM_RECOVER();
    return 0;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
