//*****************************************************************************
//
//! @file am_hal_utils.c
//!
//! @brief HAL Utility Functions
//!
//! @addtogroup utils4_4b Utils - HAL Utility Functions
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

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_hal_bootrom_helper.h"

//*****************************************************************************
//
//! Mode - the result of a change request.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_BURST_MODE,
    AM_HAL_NORMAL_MODE,
} am_hal_burst_mode_e;

//*****************************************************************************
//
//! @brief Return current burst mode state
//!
//! Implemented as a macro, this function returns the current burst mode state.
//!     AM_HAL_BURST_MODE
//!     AM_HAL_NORMAL_MODE
//
//*****************************************************************************
#define am_hal_burst_mode_status()  AM_HAL_NORMAL_MODE

//*****************************************************************************
//
//! @brief Given an integer number of microseconds, convert to a value representing
//! the number of cycles that will provide that amount
//! of delay.  This macro is designed to take into account some of the call
//! overhead and latencies.
//!
//! The BOOTROM_CYCLES_US macro assumes:
//!  - Burst or normal mode operation.
//!  - If cache is not enabled, use BOOTROM_CYCLES_US_NOCACHE() instead.
//! @name BootROM_CYCLES
//! @{
//
//*****************************************************************************
#define CYCLESPERITER                 (AM_HAL_CLKGEN_FREQ_MAX_MHZ / 3)
#define BOOTROM_CYCLES_US(n)          (((n) * CYCLESPERITER) + 0)
#define BOOTROM_CYCLES_US_NOCACHE(n)  ( (n == 0) ? 0 : (n * CYCLESPERITER) - 5)
//! @}

//*****************************************************************************
//
//! @brief Use the bootrom to implement a spin loop.
//!
//! @param ui32us - Number of microseconds to delay.  Must be >=1; the
//! value of 0 will result in an extremely long delay.
//!
//! Use this function to implement a CPU busy waiting spin loop without cache
//! or delay uncertainties.
//!
//! Notes for Apollo3:
//! - The ROM-based function executes at 3 cycles per iteration plus the normal
//!   function call, entry, and exit overhead and latencies.
//! - Cache settings affect call overhead.  However, the cache does not affect
//!   the time while inside the BOOTROM function.
//! - The function accounts for burst vs normal mode, along with some of the
//!   overhead encountered with executing the function itself (such as the
//!   check for burst mode).
//! - Use of the FLASH_CYCLES_US() or FLASH_CYCLES_US_NOCACHE() macros for the
//!   ui32Iterations parameter will result in approximate microsecond timing.
//! - The parameter ui32Iterations==0 is allowed but is still incurs a delay.
//!
//! Example:
//! - MCU operating at 48MHz -> 20.83 ns / cycle
//! - Therefore each iteration (once inside the bootrom function) will consume
//!   62.5ns (non-burst-mode).
//!
//! @note Interrupts are not disabled during execution of this function.
//!       Therefore, any interrupt taken will affect the delay timing.
//!
//
//*****************************************************************************
void
am_hal_delay_us(uint32_t ui32us)
{

    register uint32_t ui32Iterations = BOOTROM_CYCLES_US(ui32us);
    register uint32_t ui32CycleCntAdj;

    //
    // Check for LP (96MHz) vs. HP (192MHz) mode and create the adjustment accordingly.
    //
    if (PWRCTRL->MCUPERFREQ_b.MCUPERFSTATUS == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE)
    {
        ui32Iterations <<= 1;

        //
        // There's an additional shift to account for.
        //
        ui32CycleCntAdj = ((13 * 2) + 37) / 3;
    }
    else
    {
        ui32CycleCntAdj = ((13 * 1) + 32) / 3;
    }

    //
    // Allow for the overhead of the burst-mode check and these comparisons
    // by eliminating an appropriate number of iterations.
    //
    if ( ui32Iterations > ui32CycleCntAdj )
    {
        ui32Iterations -= ui32CycleCntAdj;

        g_am_hal_bootrom_helper.bootrom_delay_cycles(ui32Iterations);
    }

} // am_hal_delay_us()

//*****************************************************************************
//
//! @brief Delays for a desired amount of cycles while also waiting for a
//! status to change a value.
//!
//! @param ui32usMaxDelay - Maximum number of ~1uS delay loops.
//! @param ui32Address    - Address of the register for the status change.
//! @param ui32Mask       - Mask for the status change.
//! @param ui32Value      - Target value for the status change.
//!
//! This function will delay for approximately the given number of microseconds
//! while checking for a status change, exiting when either the given time has
//! expired or the status change is detected.
//!
//! @returns AM_HAL_STATUS_SUCCESS = status change detected.
//!          AM_HAL_STATUS_TIMEOUT = timeout.
//
//*****************************************************************************
uint32_t
am_hal_delay_us_status_change(uint32_t ui32usMaxDelay, uint32_t ui32Address,
                              uint32_t ui32Mask, uint32_t ui32Value)
{
    while ( 1 )
    {
        //
        // Check the status
        //
        if ( ( AM_REGVAL(ui32Address) & ui32Mask ) == ui32Value )
        {
            return AM_HAL_STATUS_SUCCESS;
        }

        if (ui32usMaxDelay--)
        {
            //
            // Call the BOOTROM cycle function to delay for about 1 microsecond.
            //
            am_hal_delay_us(1);
        }
        else
        {
            break;
        }
    }

    return AM_HAL_STATUS_TIMEOUT;

} // am_hal_delay_us_status_change()

//*****************************************************************************
//
//! @brief Delays for a desired amount of cycles while also waiting for a
//! status to equal OR not-equal to a value.
//!
//! @param ui32usMaxDelay - Maximum number of ~1uS delay loops.
//! @param ui32Address    - Address of the register for the status change.
//! @param ui32Mask       - Mask for the status change.
//! @param ui32Value      - Target value for the status change.
//! @param bIsEqual       - Check for equal if true; not-equal if false.
//!
//! This function will delay for approximately the given number of microseconds
//! while checking for a status change, exiting when either the given time has
//! expired or the status change is detected.
//!
//! @returns 0 = timeout.
//!          1 = status change detected.
//
//*****************************************************************************
uint32_t
am_hal_delay_us_status_check(uint32_t ui32usMaxDelay, uint32_t ui32Address,
                             uint32_t ui32Mask, uint32_t ui32Value,
                             bool bIsEqual)
{
    while ( 1 )
    {
        //
        // Check the status
        //
        if ( bIsEqual )
        {
            if ( ( AM_REGVAL(ui32Address) & ui32Mask ) == ui32Value )
            {
                return AM_HAL_STATUS_SUCCESS;
            }
        }
        else
        {
            if ( ( AM_REGVAL(ui32Address) & ui32Mask ) != ui32Value )
            {
                return AM_HAL_STATUS_SUCCESS;
            }
        }

        if ( ui32usMaxDelay-- )
        {
            //
            // Call the BOOTROM cycle function to delay for about 1 microsecond.
            //
            am_hal_delay_us(1);
        }
        else
        {
            break;
        }
    }

    return AM_HAL_STATUS_TIMEOUT;

} // am_hal_delay_us_status_check()

//*****************************************************************************
//
//! @brief Read a uint32 value from a valid memory or peripheral location.
//!
//! @param pui32Address - The location to be read.
//!
//! Use this function to safely read a value from peripheral or memory locations.
//!
//! This function calls a function that resides BOOTROM or SRAM to do the actual
//! read, thus completely avoiding any conflict with flash or INFO space.
//!
//! @return The value read from the given address.
//
//*****************************************************************************
uint32_t
am_hal_load_ui32(uint32_t *pui32Address)
{
    return g_am_hal_bootrom_helper.bootrom_util_read_word(pui32Address);
} // am_hal_load_ui32()

//*****************************************************************************
//
//! @brief Use the bootrom to write to a location in SRAM or the system bus.
//!
//! @param pui32Address - Store the data value corresponding to this location.
//! @param ui32Data     - 32-bit Data to be stored.
//!
//! Use this function to store a value to various peripheral or SRAM locations
//! that can not be touched from code running in SRAM or FLASH.  There is no
//! known need for this function in Apollo3 at this time.
//!
//
//*****************************************************************************
void
am_hal_store_ui32(uint32_t *pui32Address, uint32_t ui32Data)
{
    g_am_hal_bootrom_helper.bootrom_util_write_word(pui32Address, ui32Data);
} // am_hal_store_ui32()

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
