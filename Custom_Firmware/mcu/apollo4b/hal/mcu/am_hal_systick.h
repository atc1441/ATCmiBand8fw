//*****************************************************************************
//
//! @file am_hal_systick.h
//!
//! @brief Functions for interfacing with the SYSTICK
//!
//! @addtogroup systick4_4b SYSTICK - System Tick Timer
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
  extern void am_hal_systick_start(void);
  extern void am_hal_systick_stop(void);
  extern void am_hal_systick_int_enable(void);
  extern void am_hal_systick_int_disable(void);
  extern uint32_t am_hal_systick_int_status_get(void);
  extern void am_hal_systick_reset(void);
  extern void am_hal_systick_load(uint32_t ui32LoadVal);
  extern uint32_t am_hal_systick_count(void);
  extern uint32_t am_hal_systick_wait_ticks(uint32_t ui32Ticks);
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

