//*****************************************************************************
//
//! @file am_hal_security.c
//!
//! @brief Functions for on-chip security features
//!
//! @addtogroup security_4b Security Functionality
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

//*****************************************************************************
//  Local defines.
//*****************************************************************************
//
//! Maximum iterations for hardware CRC to finish
//
#define MAX_CRC_WAIT        100000

#define AM_HAL_SECURITY_LOCKSTAT_CUSTOTP_PROG   0x00000001
#define AM_HAL_SECURITY_LOCKSTAT_CUSTOTP_READ   0x00000002

//*****************************************************************************
//
// Globals
//
//*****************************************************************************

//*****************************************************************************
//
//  Get Device Security Info
//
//*****************************************************************************
uint32_t am_hal_security_get_info(am_hal_security_info_t *pSecInfo)
{
    bool     bSbl;

    if ( !pSecInfo )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    am_hal_mram_info_read(0, AM_REG_INFO0_SECURITY_VERSION_O / 4, 1, &pSecInfo->info0Version);
    pSecInfo->bInfo0Valid = MCUCTRL->SHADOWVALID_b.INFO0VALID;
    bSbl = (MCUCTRL->BOOTLOADER_b.SECBOOTFEATURE == MCUCTRL_BOOTLOADER_SECBOOTFEATURE_ENABLED);
    if ((PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO == 0) || (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 0))
    {
        // Crypto is not accessible
        pSecInfo->lcs = AM_HAL_SECURITY_LCS_UNDEFINED;
    }
    else
    {
        pSecInfo->lcs = (am_hal_security_device_lcs_e)CRYPTO->LCSREG_b.LCSREG;
    }
    if ( bSbl )
    {
        // Use the field in INFO1 - Readable space
        am_hal_mram_info_read(1, AM_REG_INFO1_SBR_VERSION_0_O / 4, 1, &pSecInfo->sbrVersion);
        am_hal_mram_info_read(1, AM_REG_INFO1_SBL_VERSION_0_O / 4, 1, &pSecInfo->sblVersion);
        am_hal_mram_info_read(1, AM_REG_INFO1_SBL_VERSION_1_O / 4, 1, &pSecInfo->sblVersionAddInfo);
        am_hal_mram_info_read(1, AM_REG_INFO1_SBLOTA_O / 4, 1, &pSecInfo->sblStagingAddr);
    }
    else
    {
        pSecInfo->sbrVersion = 0xFFFFFFFF;
        pSecInfo->sblVersion = 0xFFFFFFFF;
        pSecInfo->sblVersionAddInfo = 0xFFFFFFFF;
        pSecInfo->sblStagingAddr = 0xFFFFFFFF;
    }

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_security_get_info()

//*****************************************************************************
//
//  Get Device Security SOCID
//
//*****************************************************************************
uint32_t am_hal_security_get_socid(am_hal_security_socid_t *pSocId)
{
    return am_hal_mram_info_read(1, AM_REG_INFO1_SOCID0_O / 4, 8, pSocId->socid);
}

//*****************************************************************************
//
//  Set the key for specified lock
//
//*****************************************************************************
uint32_t am_hal_security_set_key(am_hal_security_locktype_t lockType, am_hal_security_128bkey_t *pKey)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( pKey == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    switch (lockType)
    {
        case AM_HAL_SECURITY_LOCKTYPE_CUSTOTP_PROG:
        case AM_HAL_SECURITY_LOCKTYPE_CUSTOTP_READ:
            break;
        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    SECURITY->LOCKCTRL = lockType;
    SECURITY->KEY0 = pKey->keys.key0;
    SECURITY->KEY1 = pKey->keys.key1;
    SECURITY->KEY2 = pKey->keys.key2;
    SECURITY->KEY3 = pKey->keys.key3;

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_security_set_key()

//*****************************************************************************
//
// Get the current status of the specified lock
//
//*****************************************************************************
uint32_t am_hal_security_get_lock_status(am_hal_security_locktype_t lockType, bool *pbUnlockStatus)
{
    uint32_t unlockMask;
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (pbUnlockStatus == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    switch ( lockType )
    {
        case AM_HAL_SECURITY_LOCKTYPE_CUSTOTP_PROG:
            unlockMask = AM_HAL_SECURITY_LOCKSTAT_CUSTOTP_PROG;
            break;
        case AM_HAL_SECURITY_LOCKTYPE_CUSTOTP_READ:
            unlockMask = AM_HAL_SECURITY_LOCKSTAT_CUSTOTP_READ;
            break;
        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }
    *pbUnlockStatus = SECURITY->LOCKSTAT & unlockMask;

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_security_get_lock_status()

//*****************************************************************************
//
//  Compute CRC32 for a specified payload
//
//*****************************************************************************
uint32_t
am_hal_crc32(uint32_t ui32StartAddr, uint32_t ui32SizeBytes, uint32_t *pui32Crc)
{
    uint32_t status, ui32CRC32;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (pui32Crc == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Make sure size is multiple of 4 bytes
    //
    if (ui32SizeBytes & 0x3)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Program the CRC engine to compute the crc
    //
    ui32CRC32                 = 0xFFFFFFFF;
    SECURITY->RESULT          = ui32CRC32;
    SECURITY->SRCADDR         = ui32StartAddr;
    SECURITY->LEN             = ui32SizeBytes;
    SECURITY->CTRL_b.FUNCTION = SECURITY_CTRL_FUNCTION_CRC32;

    if ((ui32StartAddr + ui32SizeBytes) >= (SRAM_BASEADDR + TCM_MAX_SIZE))
    {
        // Need to ensure memory writes have been flushed before starting hardware
        am_hal_sysctrl_bus_write_flush();
    }
    //
    // Start the CRC
    //
    SECURITY->CTRL_b.ENABLE = 1;

    //
    // Wait for CRC to finish
    //
    status = am_hal_delay_us_status_change(MAX_CRC_WAIT,
        (uint32_t)&SECURITY->CTRL, SECURITY_CTRL_ENABLE_Msk, 0);

    if (status == AM_HAL_STATUS_SUCCESS)
    {
        *pui32Crc = SECURITY->RESULT;
    }

    return status;

} // am_hal_crc32()

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
