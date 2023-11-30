//*****************************************************************************
//
//! @file am_hal_system.h
//!
//! @brief Apollo4B system-wide definitions.
//!
//! @addtogroup system_4b System Wide Definitions
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

#ifndef AM_HAL_SYSTEM_H
#define AM_HAL_SYSTEM_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
//
//! @name Some helpful macros defining addresses and sizes.
//! @{
//
//*****************************************************************************
#define AM_HAL_SYSTEM_NVRAM_ADDR                (0x00000000 + AM_HAL_SYSTEM_NVRAM_SBL_OFFSET)
#define AM_HAL_SYSTEM_NVRAM_SIZE                ((2 * 1024 * 1024) - AM_HAL_SYSTEM_NVRAM_SBL_OFFSET)
#define AM_HAL_SYSTEM_NVRAM_NWDS                (AM_HAL_SYSTEM_NVRAM_SIZE / 4)

#define AM_HAL_SYSTEM_MCU_DTCM_ADDR             0x10000000
#define AM_HAL_SYSTEM_MCU_DTCM_SIZE             (128 * 1024)
#define AM_HAL_SYSTEM_MCU_DTCM_NWDS             (AM_HAL_SYSTEM_MCU_DTCM_SIZE / 4)

#define AM_HAL_SYSTEM_SSRAM_ADDR                (0x10000000 + (384 * 1024))
#define AM_HAL_SYSTEM_SSRAM_SIZE                (1024 * 1024)
#define AM_HAL_SYSTEM_SSRAM_NWDS                (AM_HAL_SYSTEM_SSRAM_SIZE / 4)
//! @}

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_SYSTEM_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

