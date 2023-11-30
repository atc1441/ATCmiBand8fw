//*****************************************************************************
//
//! @file am_hal_utils.h
//!
//! @brief HAL Utility Functions
//!
//! @addtogroup utils4_4p Utils - HAL Utility Functions
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
#ifndef AM_HAL_UTILS_H
#define AM_HAL_UTILS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

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
//
//*****************************************************************************
extern void am_hal_delay_us(uint32_t ui32us);

//*****************************************************************************
//
//! @brief Delays for a desired amount of cycles while also waiting for a
//! status to change a value.
//!
//! @param ui32usMaxDelay - Maximum number of ~1uS delay loops.
//! @param ui32Address    - Address of the register for the status change.
//! @param ui32Mask   - Mask for the status change.
//! @param ui32Value  - Target value for the status change.
//!
//! This function will delay for approximately the given number of microseconds
//! while checking for a status change, exiting when either the given time has
//! expired or the status change is detected.
//!
//! @returns AM_HAL_STATUS_SUCCESS = status change detected.
//!          AM_HAL_STATUS_TIMEOUT = timeout.
//
//*****************************************************************************
extern uint32_t am_hal_delay_us_status_change(uint32_t ui32usMaxDelay, uint32_t ui32Address,
                                 uint32_t ui32Mask, uint32_t ui32Value);

//*****************************************************************************
//
//! @brief Delays for a desired amount of cycles while also waiting for a
//! status to equal OR not-equal to a value.
//!
//! @param ui32usMaxDelay - Maximum number of ~1uS delay loops.
//! @param ui32Address    - Address of the register for the status change.
//! @param ui32Mask   - Mask for the status change.
//! @param ui32Value  - Target value for the status change.
//! @param bIsEqual   - Check for equal if true; not-equal if false.
//!
//! This function will delay for approximately the given number of microseconds
//! while checking for a status change, exiting when either the given time has
//! expired or the status change is detected.
//!
//! @returns 0 = timeout.
//!          1 = status change detected.
//
//*****************************************************************************
extern uint32_t am_hal_delay_us_status_check(uint32_t ui32usMaxDelay, uint32_t ui32Address,
                                uint32_t ui32Mask, uint32_t ui32Value,
                                bool bIsEqual);

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
extern uint32_t am_hal_load_ui32(uint32_t *pui32Address);

//*****************************************************************************
//
//! @brief Use the bootrom to write to a location in SRAM or the system bus.
//!
//! @param pui32Address - Store the data value corresponding to this location.
//! @param ui32Data - 32-bit Data to be stored.
//!
//! Use this function to store a value to various peripheral or SRAM locations
//! that can not be touched from code running in SRAM or FLASH.  There is no
//! known need for this function in Apollo3 at this time.
//
//*****************************************************************************
extern void am_hal_store_ui32(uint32_t *pui32Address, uint32_t ui32Data);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_UTILS_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

