//*****************************************************************************
//
//! @file am_hal_access.h
//!
//! @brief This file controls peripheral access in Apollo4.
//!
//! @addtogroup access_4b Access - Peripheral Access
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
#ifndef AM_HAL_ACCESS_H
#define AM_HAL_ACCESS_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Global definitions
//
//*****************************************************************************
#define AM_HAL_ACCESS_STRUCT_SIZE   5

//*****************************************************************************
//
//! @brief Error codes.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_ACCESS_NOT_ALLOWED = AM_HAL_STATUS_MODULE_SPECIFIC_START
}
am_hal_access_error_e;

//*****************************************************************************
//
//! @brief Enumerated list of peripherals with controlled access.
//
//*****************************************************************************
typedef struct
{
    const uint32_t *pui32Shared;
    const uint32_t *pui32MCUAllowed;
    const uint32_t *pui32DSP0Allowed;
    const uint32_t *pui32DSP1Allowed;
    uint32_t *pui32MCUClaimed;
    uint32_t *pui32DSP0Claimed;
    uint32_t *pui32DSP1Claimed;
}
am_hal_access_t;

//*****************************************************************************
//
//! @brief Enumerated list of peripherals with controlled access.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_ACCESS_GPIO_0,
    AM_HAL_ACCESS_GPIO_1,
    AM_HAL_ACCESS_GPIO_2,
    AM_HAL_ACCESS_GPIO_3,
    AM_HAL_ACCESS_GPIO_4,
    AM_HAL_ACCESS_GPIO_5,
    AM_HAL_ACCESS_GPIO_6,
    AM_HAL_ACCESS_GPIO_7,
    AM_HAL_ACCESS_GPIO_8,
    AM_HAL_ACCESS_GPIO_9,
    AM_HAL_ACCESS_GPIO_10,
    AM_HAL_ACCESS_GPIO_11,
    AM_HAL_ACCESS_GPIO_12,
    AM_HAL_ACCESS_GPIO_13,
    AM_HAL_ACCESS_GPIO_14,
    AM_HAL_ACCESS_GPIO_15,
    AM_HAL_ACCESS_GPIO_16,
    AM_HAL_ACCESS_GPIO_17,
    AM_HAL_ACCESS_GPIO_18,
    AM_HAL_ACCESS_GPIO_19,
    AM_HAL_ACCESS_GPIO_20,
    AM_HAL_ACCESS_GPIO_21,
    AM_HAL_ACCESS_GPIO_22,
    AM_HAL_ACCESS_GPIO_23,
    AM_HAL_ACCESS_GPIO_24,
    AM_HAL_ACCESS_GPIO_25,
    AM_HAL_ACCESS_GPIO_26,
    AM_HAL_ACCESS_GPIO_27,
    AM_HAL_ACCESS_GPIO_28,
    AM_HAL_ACCESS_GPIO_29,
    AM_HAL_ACCESS_GPIO_30,
    AM_HAL_ACCESS_GPIO_31,
    AM_HAL_ACCESS_GPIO_32,
    AM_HAL_ACCESS_GPIO_33,
    AM_HAL_ACCESS_GPIO_34,
    AM_HAL_ACCESS_GPIO_35,
    AM_HAL_ACCESS_GPIO_36,
    AM_HAL_ACCESS_GPIO_37,
    AM_HAL_ACCESS_GPIO_38,
    AM_HAL_ACCESS_GPIO_39,
    AM_HAL_ACCESS_GPIO_40,
    AM_HAL_ACCESS_GPIO_41,
    AM_HAL_ACCESS_GPIO_42,
    AM_HAL_ACCESS_GPIO_43,
    AM_HAL_ACCESS_GPIO_44,
    AM_HAL_ACCESS_GPIO_45,
    AM_HAL_ACCESS_GPIO_46,
    AM_HAL_ACCESS_GPIO_47,
    AM_HAL_ACCESS_GPIO_48,
    AM_HAL_ACCESS_GPIO_49,
    AM_HAL_ACCESS_GPIO_50,
    AM_HAL_ACCESS_GPIO_51,
    AM_HAL_ACCESS_GPIO_52,
    AM_HAL_ACCESS_GPIO_53,
    AM_HAL_ACCESS_GPIO_54,
    AM_HAL_ACCESS_GPIO_55,
    AM_HAL_ACCESS_GPIO_56,
    AM_HAL_ACCESS_GPIO_57,
    AM_HAL_ACCESS_GPIO_58,
    AM_HAL_ACCESS_GPIO_59,
    AM_HAL_ACCESS_GPIO_60,
    AM_HAL_ACCESS_GPIO_61,
    AM_HAL_ACCESS_GPIO_62,
    AM_HAL_ACCESS_GPIO_63,
}
am_hal_access_periph_e;

#define AM_HAL_ACCESS_GPIO_BASE         AM_HAL_ACCESS_GPIO_0

//*****************************************************************************
//
// External functions.
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Initialize the central access structure.
//!
//! @param pvHandle is the handle for this structure.
//!
//! This initializes a local set of pointers to the global peripheral access
//! structure.
//!
//! @return Status code.
//
//*****************************************************************************
extern uint32_t am_hal_access_initialize(void **pvHandle);

//*****************************************************************************
//
//! @brief De-initialize the central access structure.
//!
//! @param pvHandle is the handle for this structure.
//!
//! This function de-initializes the local set of pointers for the global
//! peripheral access structure.
//!
//! @return Status code.
//
//*****************************************************************************
extern uint32_t am_hal_access_deinitialize(void *pvHandle);

//*****************************************************************************
//
//! @brief De-initialize the central access structure.
//!
//! @param pvHandle is the handle for this access structure.
//! @param psGlobalAccess is the structure describing access permissions.
//!
//! This function sets up a series of pointers that the HAL can use to check
//! and aquire permission to access specific peripherals.
//!
//! @return Status code.
//
//*****************************************************************************
extern uint32_t am_hal_access_config(void *pvHandle,
                                     am_hal_access_t *psGlobalAccess);

//*****************************************************************************
//
//! @brief Obtain access to a peripheral
//!
//! @param pvHandle is the handle for this access structure.
//! @param ePeripheral is the peripheral to obtain access for.
//! @param ui32TimeoutUS timeout in microseconds
//!
//! This function attempts to gain access to a particular peripheral. It
//! ensures that no other caller can successfully make the same request until
//! control is released.
//!
//! @return Status code.
//
//*****************************************************************************
extern uint32_t am_hal_access_get(void *pvHandle,
                                  am_hal_access_periph_e ePeripheral,
                                  uint32_t ui32TimeoutUS);

//*****************************************************************************
//
//! @brief Release control of a peripheral.
//!
//! @param pvHandle is the handle for this access structure.
//! @param ePeripheral is the peripheral to obtain access for.
//! @param ui32TimeoutUS timeout in microseconds
//!
//! This function releases control of a peripheral, allowing it to be used by
//! other callers.
//!
//! @return Status code.
//
//*****************************************************************************
extern uint32_t am_hal_access_release(void *pvHandle,
                                      am_hal_access_periph_e ePeripheral,
                                      uint32_t ui32TimeoutUS);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_ACCESS_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

