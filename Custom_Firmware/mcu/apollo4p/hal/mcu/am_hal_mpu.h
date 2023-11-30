//*****************************************************************************
//
//! @file am_hal_mpu.h
//!
//! @brief Hardware abstraction for the Memory Protection Unit.
//!
//! @addtogroup mpu_4p MPU - Memory Protection Unit
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

#ifndef AM_HAL_MPU_H
#define AM_HAL_MPU_H

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

#ifdef __cplusplus
extern "C"
{
#endif

//
//! Default value to use for TEX,S,C,B
//
#define MPU_DEFAULT_TEXSCB      ((MPU_RASR_B_Msk | MPU_RASR_S_Msk))  // 0x00050000

//*****************************************************************************
//
//! @name Macro definitions for system control block registers
//! @{
//
//*****************************************************************************
#define AM_REG_SYSCTRL_MMFSR    0xE000ED28
#define AM_REG_SYSCTRL_MMFAR    0xE000ED34
//! @}


//*****************************************************************************
//
// External variable definitions
//
//*****************************************************************************

//*****************************************************************************
//
// Structure definitions.
//
//*****************************************************************************

//
//! @brief Enum type for specifying memory access privileges for an MPU region.
//
typedef enum
{
    NO_ACCESS       = ARM_MPU_AP_NONE,
    PRIV_RW         = ARM_MPU_AP_PRIV,
    PRIV_RW_PUB_RO  = ARM_MPU_AP_URO,
    PRIV_RW_PUB_RW  = ARM_MPU_AP_FULL,
    PRIV_RO         = ARM_MPU_AP_PRO,
    PRIV_RO_PUB_RO  = ARM_MPU_AP_RO,
    PRIV_FORCE_X32  = 0x7FFFFFFF,
}
tAccessPermission;

//
//! @brief Configuration structure for MPU regions.
//
typedef struct
{
    uint32_t ui32BaseAddress;
    tAccessPermission eAccessPermission;
    uint16_t ui16SubRegionDisable;
    uint8_t ui8RegionNumber;
    uint8_t ui8Size;
    bool bExecuteNever;
}
tMPURegion;

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Returns the contents of the MPU_TYPE register
//!
//! @param pui32Type pointer to 32-bit unsigned integer representing
//! the contents of MPU_TYPE
//!
//! This function accesses the ARM MPU_TYPE register. It can be used to check
//! for the presence of an MPU, and to obtain basic information about the
//! implementation of the MPU.
//!
//! @return standard hal status
//
//*****************************************************************************
extern uint32_t am_hal_mpu_type_get(uint32_t *pui32Type);
//#define mpu_type_get am_hal_mpu_type_get

//*****************************************************************************
//
//! @brief Sets the global configuration of the MPU
//!
//! @param bMPUEnable         - Enable the MPU
//! @param bPrivilegedDefault - Enable the default priveleged memory map
//! @param bFaultNMIProtect   - Enable the MPU during fault handlers
//!
//! This function is a wrapper for the MPU_CTRL register, which controls the
//! global configuration of the MPU. This function can enable or disable the
//! MPU overall with the \e bMPUEnable parameter, and also controls how fault
//! handlers, NMI service routines, and privileged-mode execution is handled by
//! the MPU.
//!
//! Setting \e bPrivilegedDefault will enable the default memory map for
//! privileged accesses. If the MPU is enabled with this value set, only
//! privileged code can execute from the system address map
//!
//! Setting \e bFaultNMIProtect leaves the MPU active during the execution of
//! NMI and Hard Fault handlers. Clearing this value will disable the MPU
//! during these procedures.
//!
//! @return standard hal status
//
//*****************************************************************************
extern uint32_t am_hal_mpu_global_configure(bool bMPUEnable,
                                            bool bPrivilegedDefault,
                                            bool bFaultNMIProtect);
#define mpu_global_configure am_hal_mpu_global_configure

//*****************************************************************************
//
//! @brief Configures an MPU region.
//!
//! @param psConfig
//! @param bEnableNow
//!
//! @details This function performs the necessary configuration for the MPU region
//! described by the \e psConfig structure, and will also enable the region if
//! the \e bEnableNow option is true.
//!
//! @return standard hal status
//
//*****************************************************************************
extern uint32_t am_hal_mpu_region_configure(tMPURegion *psConfig, bool bEnableNow);
#define mpu_region_configure am_hal_mpu_region_configure

//*****************************************************************************
//
//! @brief Enable an MPU region.
//!
//! @param ui8RegionNumber
//!
//! @details Enable the MPU region referred to by \e ui8RegionNumber.
//!
//! @note This function should only be called after the desired region has
//! already been configured.
//!
//! @return standard hal status
//
//*****************************************************************************
extern uint32_t am_hal_mpu_region_enable(uint8_t ui8RegionNumber);
#define mpu_region_enable am_hal_mpu_region_enable

//*****************************************************************************
//
//! @brief Disable an MPU region.
//!
//! @param ui8RegionNumber
//!
//! @details Disable the MPU region referred to by \e ui8RegionNumber.
//!
//!
//! @return standard hal status
//
//*****************************************************************************
extern uint32_t am_hal_mpu_region_disable(uint8_t ui8RegionNumber);
#define mpu_region_disable am_hal_mpu_region_disable

//*****************************************************************************
//
//! @brief Get the MPU region number.
//!
//! @param ui32pMpuRNR - pointer to 32-bit unsigned int where region number
//! will be returned
//!
//! @details Get the MPU region number from MPU_RNR register.
//!
//! @return standard hal status
//
//*****************************************************************************
extern uint32_t am_hal_mpu_get_region_number(uint32_t *ui32pMpuRNR);
//#define mpu_get_region_number am_hal_mpu_get_region_number

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_MPU_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

