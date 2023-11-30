//*****************************************************************************
//
//! @file am_hal_bootrom_helper.h
//!
//! @brief BootROM Helper Function Table
//!
//! @addtogroup bootrom4_4b Bootrom Functionality
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
#ifndef AM_HAL_BOOTROM_HELPER_H
#define AM_HAL_BOOTROM_HELPER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
//
//! Structure of pointers to helper functions invoking flash operations.
//
//! The functions we are pointing to here are in the Apollo 4
//! integrated BOOTROM.
//
//*****************************************************************************
typedef struct am_hal_bootrom_helper_struct
{
    //
    // Basics functions required by most toolchains.
    //
    int  (*nv_mass_erase)(uint32_t, uint32_t);
    int  (*nv_page_erase)(uint32_t, uint32_t, uint32_t);
    int  (*nv_program_main)(uint32_t, uint32_t *, uint32_t *, uint32_t);

    //
    // Infospace programming function.
    //
    int  (*nv_program_info_area)(uint32_t, uint32_t *, uint32_t, uint32_t);

    //
    // Helpful utilities.
    //
    int  (*nv_program_main2)(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);

    uint32_t (*bootrom_util_read_word)( uint32_t *);
    void (*bootrom_util_write_word)( uint32_t *, uint32_t);

    //
    // Infospace erase functions.
    //
    int  (*nv_info_erase)( uint32_t);

    //
    // Non-Volatile Recovery function.
    //
    int  (*nv_recovery)( uint32_t value);

    //
    // Cycle accurate delay function.
    //
    void (*bootrom_delay_cycles)(uint32_t ui32Cycles);

} am_hal_bootrom_helper_t;

extern const am_hal_bootrom_helper_t g_am_hal_bootrom_helper;

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_BOOTROM_HELPER_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

