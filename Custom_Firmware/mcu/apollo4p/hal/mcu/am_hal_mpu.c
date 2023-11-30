//*****************************************************************************
//
//! @file am_hal_mpu.c
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

#include "am_hal_mpu.h"

//*****************************************************************************
//
// Returns the contents of the MPU_TYPE register
//
//*****************************************************************************
uint32_t
am_hal_mpu_type_get(uint32_t *pui32Type)
{
    *pui32Type = MPU->TYPE ;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Sets the global configuration of the MPU
//
//*****************************************************************************
uint32_t
am_hal_mpu_global_configure(bool bMPUEnable,
                            bool bPrivilegedDefault,
                            bool bFaultNMIProtect)
{
    __DMB();

    MPU->CTRL =  _VAL2FLD(MPU_CTRL_ENABLE,   (uint32_t)bMPUEnable ) |
                 _VAL2FLD(MPU_CTRL_HFNMIENA, (uint32_t)bFaultNMIProtect ) |
                 _VAL2FLD(MPU_CTRL_PRIVDEFENA,  (uint32_t)bPrivilegedDefault ) ;


    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;

    __DSB();
    __ISB();

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Configures an MPU region.
//
//*****************************************************************************
uint32_t
am_hal_mpu_region_configure(tMPURegion *psConfig, bool bEnableNow)
{

#ifndef AM_HAL_DISABLE_API_VALIDATION

    if (psConfig->ui8RegionNumber > _FLD2VAL( MPU_RBAR_REGION , 0xFFFFFFFF ) ) //(MPU_RBAR_REGION_Msk >>MPU_RBAR_REGION_Pos) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    if (psConfig->eAccessPermission > _FLD2VAL( MPU_RASR_AP, 0xFFFFFFFF)) // (MPU_RASR_AP_Msk >> MPU_RASR_AP_Pos))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    if (psConfig->ui16SubRegionDisable > _FLD2VAL(MPU_RASR_SRD, 0xFFFFFFFF)) // (MPU_RASR_SRD_Msk >> MPU_RASR_SRD_Pos))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    if (psConfig->ui8Size > _FLD2VAL(MPU_RASR_SIZE, 0xFFFFFFFF)) // (MPU_RASR_SIZE_Msk >> MPU_RASR_SIZE_Pos))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Set the new base address for the specified region.
    //
    MPU->RBAR = (psConfig->ui32BaseAddress & MPU_RBAR_ADDR_Msk)             |
                 _VAL2FLD(MPU_RBAR_REGION, psConfig->ui8RegionNumber )      |
                 _VAL2FLD(MPU_RBAR_VALID, 1 ) ;

    //
    // Set the attributes for this region based on the input structure.
    //
    MPU->RASR =  _VAL2FLD(MPU_RASR_XN, (uint32_t) psConfig->bExecuteNever ) |
                 _VAL2FLD(MPU_RASR_AP, psConfig->eAccessPermission)         |
                 _VAL2FLD(MPU_RASR_SRD, psConfig->ui16SubRegionDisable)     |
                 _VAL2FLD(MPU_RASR_SIZE, psConfig->ui8Size )                |
                 _VAL2FLD(MPU_RASR_ENABLE, (uint32_t) bEnableNow )          |
                 MPU_DEFAULT_TEXSCB;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Enable an MPU region.
//
//*****************************************************************************
uint32_t
am_hal_mpu_region_enable(uint8_t ui8RegionNumber)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (ui8RegionNumber > _FLD2VAL(MPU_RBAR_REGION, 0xFFFFFFFF))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif
    //
    // Set the region number in the MPU_RNR register, and set the enable bit.
    //

    MPU->RNR = _VAL2FLD(MPU_RBAR_REGION, ui8RegionNumber);
    MPU->RASR |= MPU_RASR_ENABLE_Msk;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Disable an MPU region.
//
//*****************************************************************************
uint32_t
am_hal_mpu_region_disable(uint8_t ui8RegionNumber)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (ui8RegionNumber > _FLD2VAL(MPU_RBAR_REGION, 0xFFFFFFFF))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif
    //
    // Set the region number in the MPU_RNR register, and clear the enable bit.
    //
    MPU->RNR  = _VAL2FLD(MPU_RBAR_REGION, ui8RegionNumber);
    MPU->RASR &= ~(MPU_RASR_ENABLE_Msk);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Get the MPU region number.
//
//*****************************************************************************
uint32_t
am_hal_mpu_get_region_number(uint32_t *ui32pMpuRNR)
{
    *ui32pMpuRNR = _FLD2VAL( MPU_RBAR_REGION, MPU->RNR);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

