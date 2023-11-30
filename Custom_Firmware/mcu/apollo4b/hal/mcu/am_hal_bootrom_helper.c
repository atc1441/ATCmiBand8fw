//*****************************************************************************
//
//! @file am_hal_bootrom_helper.c
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

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_hal_bootrom_helper.h"

//
// Look-up table
//
const am_hal_bootrom_helper_t g_am_hal_bootrom_helper =
{
         ((int  (*)(uint32_t, uint32_t))                                    0x0800004D),    // nv_mass_erase
         ((int  (*)(uint32_t, uint32_t, uint32_t))                          0x08000051),    // nv_page_erase
         ((int  (*)(uint32_t, uint32_t *, uint32_t *, uint32_t))            0x08000055),    // nv_program_main
         ((int  (*)(uint32_t, uint32_t *, uint32_t, uint32_t))              0x08000059),    // nv_program_info_area
         ((int  (*)(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t))      0x0800006D),    // nv_program_main2
         ((uint32_t (*)(uint32_t *))                                        0x08000075),    // br_util_read_word
         ((void (*)( uint32_t *, uint32_t))                                 0x08000079),    // br_util_write_word
         ((int  (*)( uint32_t))                                             0x08000081),    // nv_info_erase
         ((int  (*)( uint32_t ))                                            0x08000099),    // nv_recovery
         ((void (*)(uint32_t ))                                             0x0800009D),    // br_util_delay_cycles
};

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
