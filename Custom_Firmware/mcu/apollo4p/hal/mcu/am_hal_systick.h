//*****************************************************************************
//
//! @file am_hal_systick.h
//!
//! @brief Functions for interfacing with the SYSTICK
//!
//! @addtogroup systick4_4p SYSTICK - System Tick Timer
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
#ifndef AM_HAL_SYSTICK_H
#define AM_HAL_SYSTICK_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief Start the SYSTICK.
//!
//! This function starts the systick timer.
//!
//! @note This timer does not run in deep-sleep mode as it runs from the core
//! clock, which is gated in deep-sleep. If a timer is needed in deep-sleep use
//! one of the ctimers instead. Also to note is this timer will consume higher
//! power than the ctimers.
//
//*****************************************************************************
extern void am_hal_systick_start(void);

//*****************************************************************************
//
//! @brief Stop the SYSTICK.
//!
//! This function stops the systick timer.
//!
//! @note This timer does not run in deep-sleep mode as it runs from the core
//! clock, which is gated in deep-sleep. If a timer is needed in deep-sleep use
//! one of the ctimers instead. Also to note is this timer will consume higher
//! power than the ctimers.
//
//*****************************************************************************
extern void am_hal_systick_stop(void);

//*****************************************************************************
//
//! @brief Enable the interrupt in the SYSTICK.
//!
//! This function enables the interupt in the systick timer.
//
//*****************************************************************************
extern void am_hal_systick_int_enable(void);

//*****************************************************************************
//
//! @brief Disable the interrupt in the SYSTICK.
//!
//! This function disables the interupt in the systick timer.
//
//*****************************************************************************
extern void am_hal_systick_int_disable(void);

//*****************************************************************************
//
//! @brief Reads the interrupt status.
//!
//! This function reads the interrupt status in the systick timer.
//!
//! @return the interrupt status.
//
//*****************************************************************************
extern uint32_t am_hal_systick_int_status_get(void);

//*****************************************************************************
//
//! @brief Reset the interrupt in the SYSTICK.
//!
//! This function resets the systick timer by clearing out the configuration
//! register.
//
//*****************************************************************************
extern void am_hal_systick_reset(void);

//*****************************************************************************
//
//! @brief Load the value into the SYSTICK.
//!
//! @param ui32LoadVal the desired load value for the systick. Maximum value is
//! 0x00FF.FFFF.
//!
//! This function loads the desired value into the systick timer.
//
//*****************************************************************************
extern void am_hal_systick_load(uint32_t ui32LoadVal);

//*****************************************************************************
//
//! @brief Get the current count value in the SYSTICK.
//!
//! This function gets the current count value in the systick timer.
//!
//! @return Current count value.
//
//*****************************************************************************
extern uint32_t am_hal_systick_count(void);

//*****************************************************************************
//
//! @brief Wait the specified number of ticks.
//!
//! This function delays for the given number of SysTick ticks.
//!
//! @param ui32Ticks Number of Ticks to Wait
//!
//! @note If the SysTick timer is being used elsewhere, it will be corrupted
//! by calling this function.
//!
//! @return 0 if successful.
//
//*****************************************************************************
extern uint32_t am_hal_systick_wait_ticks(uint32_t ui32Ticks);

//*****************************************************************************
//
//! @brief Delay the specified number of microseconds.
//!
//! This function will use the SysTick timer to delay until the specified
//!  number of microseconds have elapsed.  It uses the processor clocks and
//!  takes into account the current CORESEL setting.
//!
//! @param ui32NumUs Number of usecs to Wait
//!
//! @note If the SysTick timer is being used elsewhere, it will be corrupted
//! by calling this function.
//!
//! @return Total number of SysTick ticks delayed.
//
//*****************************************************************************
extern uint32_t am_hal_systick_delay_us(uint32_t ui32NumUs);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_SYSTICK_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

