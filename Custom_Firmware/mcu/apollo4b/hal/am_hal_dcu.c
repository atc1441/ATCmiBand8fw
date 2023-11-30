//*****************************************************************************
//
//! @file am_hal_dcu.c
//!
//! @brief Implementation for Debug Control Unit functionality
//!
//! @addtogroup dcu_4b DCU - Debug Control Unit
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
#include "am_mcu_apollo.h"

#define CRYPTO_CC_IS_IDLE()     while (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 0)

// Raw offset for 3b value corresponding to DCU value 1
uint32_t gStartOff    = 0; // DCU value 1 corresponds to b[2:0]
uint64_t gDcuMask     = AM_HAL_DCURAW_MASK;
uint64_t gDcuEnable   = AM_HAL_DCURAW_ENABLE;
uint64_t gDcuDisable  = AM_HAL_DCURAW_DISABLE;
volatile uint32_t *gpDcuEnable = &CRYPTO->HOSTDCUEN2;
volatile uint32_t *gpDcuLock   = &CRYPTO->HOSTDCULOCK2;

//*****************************************************************************
//
//! @brief Get the Current RAW DCU Mask
//
//*****************************************************************************
static uint64_t
get_raw_dcu_mask(uint32_t ui32DcuMask, uint8_t threeBitVal)
{
    uint32_t i = AM_HAL_DCU_NUMDCU;
    uint64_t ui64Mask = 0;
    uint32_t offset = gStartOff;
    ui32DcuMask >>= 1; // DCU value 0 is not defined
    while ( --i )
    {
        if (ui32DcuMask & 0x1)
        {
            ui64Mask |= (uint64_t)threeBitVal << offset;
        }
        offset += 3;
        ui32DcuMask >>= 1;
    }
    return ui64Mask;
}

//*****************************************************************************
//
//! @brief Get the Current DCU Mask
//
//*****************************************************************************
static uint32_t
get_ui32_dcu_mask(uint64_t ui64DcuMask, uint8_t threeBitVal)
{
    uint32_t i = AM_HAL_DCU_NUMDCU;
    uint32_t ui32Mask = 0;
    ui64DcuMask >>= gStartOff;
    while ( --i )
    {
        if ((ui64DcuMask & AM_HAL_DCURAWVAL_MASK) == threeBitVal)
        {
            ui32Mask |= (1 << (AM_HAL_DCU_NUMDCU - i - 1));
        }
        ui64DcuMask >>= 3;
    }
    return ui32Mask;
}

//*****************************************************************************
//
//! @brief Copy words from Register to Register
//
//*****************************************************************************
static void
copy_words(uint32_t *pDst, uint32_t *pSrc, uint32_t numWords)
{
    while (numWords--)
    {
        AM_REGVAL((pDst + numWords)) = AM_REGVAL((pSrc + numWords));
    }
}

//*****************************************************************************
//
//! @brief  Read DCU Lock
//!
//! @param  pui64Val -  Pointer to double word for returned data
//!
//! This will retrieve the DCU Lock information
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
static
uint32_t am_hal_dcu_raw_lock_status_get(uint64_t *pui64Val)
{
    copy_words((uint32_t *)pui64Val, (uint32_t *)gpDcuLock, sizeof(uint64_t) / 4);
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief  Read DCU Lock
//!
//! @param  pui32Val -  Pointer to word for returned data (Qualified DCU Mask)
//!
//! This will retrieve the DCU Lock information
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
uint32_t am_hal_dcu_lock_status_get(uint32_t *pui32Val)
{
    uint64_t ui64Lock;
    uint32_t ui32Status;
    if ((PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO == 0) || (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 0))
    {
        // Crypto is not accessible
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    ui32Status = am_hal_dcu_raw_lock_status_get(&ui64Lock);
    *pui32Val = get_ui32_dcu_mask(ui64Lock, AM_HAL_DCURAWVAL_MASK);
    return ui32Status;
}

//*****************************************************************************
//
//! @brief  Write DCU Lock
//!
//! @param  val -  double word for lock values
//!
//! This will lock the DCU from further changes
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
static
uint32_t am_hal_dcu_raw_lock(uint64_t ui64Mask)
{
    //
    // copy_words((uint32_t *)gpDcuLock, (uint32_t *)&ui64Mask, sizeof(uint64_t) / 4);
    //
    // In order to avoid a GCC compiler warning, we'll explicitly handle the
    // call to copy_words() by replacing it with a couple of direct writes.
    // Note that the writes are done in the same reverse order as copy_words()
    // would do it.
    //
    gpDcuLock[1] = (uint32_t)(ui64Mask >> 32);
    gpDcuLock[0] = (uint32_t)(ui64Mask >> 0);
    CRYPTO_CC_IS_IDLE();
    return AM_HAL_STATUS_SUCCESS;
} // am_hal_dcu_raw_lock()

//*****************************************************************************
//
//! @brief  Write DCU Lock (Qualified Values)
//!
//! @param  ui32Mask -  Mask for lock values
//!
//! This will lock the DCU from further changes
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
uint32_t am_hal_dcu_lock(uint32_t ui32Mask)
{
    uint64_t ui64Lock;
    if ((PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO == 0) || (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 0))
    {
        // Crypto is not accessible
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    ui64Lock = get_raw_dcu_mask(ui32Mask, AM_HAL_DCURAWVAL_MASK);
    return am_hal_dcu_raw_lock(ui64Lock);
}

//*****************************************************************************
//
//! @brief  Read DCU Enables
//!
//! @param  pui64Val -  Pointer to double word for returned data
//!
//! This will get the current DCU Enable settings
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
static
uint32_t am_hal_dcu_raw_get(uint64_t *pui64Val)
{
    copy_words((uint32_t *)pui64Val, (uint32_t *)gpDcuEnable, sizeof(uint64_t) / 4);
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief  Read DCU Enables (Qualified Values)
//!
//! @param  pui32Val -  Pointer to Mask for returned data
//!
//! This will get the current DCU Enable settings
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
uint32_t am_hal_dcu_get(uint32_t *pui32Val)
{
    uint64_t ui64Enable;
    uint32_t ui32Status;
    if ((PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO == 0) || (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 0))
    {
        // Crypto is not accessible
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    ui32Status = am_hal_dcu_raw_get(&ui64Enable);
    *pui32Val = get_ui32_dcu_mask(ui64Enable, AM_HAL_DCURAWVAL_ENABLE);
    return ui32Status;
}

//*****************************************************************************
//
//! @brief  Update DCU Enable
//!
//! @param  ui64Mask -  DCU controls to be modified
//! @param  bEnable - Whether to enable or disable
//!
//! This will update the DCU Enable settings, if not locked
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
static
uint32_t am_hal_dcu_raw_update(bool bEnable, uint64_t ui64Mask)
{
    uint64_t dcuVal;
    uint64_t dcuLock;
    copy_words((uint32_t *)&dcuLock, (uint32_t *)gpDcuLock, sizeof(uint64_t) / 4);
    if (ui64Mask & dcuLock)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    copy_words((uint32_t *)&dcuVal, (uint32_t *)gpDcuEnable, sizeof(uint64_t) / 4);
    if (bEnable)
    {
        dcuVal = (dcuVal & ~ui64Mask) | (gDcuEnable & ui64Mask);
    }
    else
    {
        dcuVal = (dcuVal & ~ui64Mask) | (gDcuDisable & ui64Mask);
    }
    copy_words((uint32_t *)gpDcuEnable, (uint32_t *)&dcuVal, sizeof(uint64_t) / 4);
    CRYPTO_CC_IS_IDLE();
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Update DCU Enable (Qualified Values)
//
// This will update the DCU Enable settings, if not locked
//
//*****************************************************************************
uint32_t am_hal_dcu_update(bool bEnable, uint32_t ui32Mask)
{
    uint64_t ui64Mask;
    if ((PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO == 0) || (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 0))
    {
        // Crypto is not accessible
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    ui64Mask = get_raw_dcu_mask(ui32Mask, AM_HAL_DCURAWVAL_MASK);
    return am_hal_dcu_raw_update(bEnable, ui64Mask);
}

//*****************************************************************************
//
// DCU Disable - Using MCUCTRL Override
//
// This will update the MCUCTRL DCU Disable Override settings
// This can only further lock things if the corresponding DCU Enable was open
//
//*****************************************************************************
uint32_t am_hal_dcu_mcuctrl_override(uint32_t ui32Mask)
{
    MCUCTRL->DEBUGGER = ui32Mask;
    return AM_HAL_STATUS_SUCCESS;
}


//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
