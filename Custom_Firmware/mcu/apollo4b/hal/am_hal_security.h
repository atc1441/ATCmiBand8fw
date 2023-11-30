//*****************************************************************************
//
//! @file am_hal_security.h
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

#ifndef AM_HAL_SECURITY_H
#define AM_HAL_SECURITY_H

#define AM_HAL_SECURITY_SOCID_NUMWORDS  8

//
//! Security Device LCS
//
typedef enum
{
    AM_HAL_SECURITY_LCS_CM = 0,
    AM_HAL_SECURITY_LCS_DM = 1,
    AM_HAL_SECURITY_LCS_SECURE = 5,
    AM_HAL_SECURITY_LCS_RMA = 7,
    AM_HAL_SECURITY_LCS_UNDEFINED = 0xff
} am_hal_security_device_lcs_e;

//
//! Security Info
//
typedef struct
{
    bool                            bInfo0Valid;
    am_hal_security_device_lcs_e    lcs;
    uint32_t                        sbrVersion;
    uint32_t                        info0Version;
    uint32_t                        sblVersion;
    uint32_t                        sblVersionAddInfo;
    uint32_t                        sblStagingAddr;
} am_hal_security_info_t;

//
//! Security Lock Type Definitions
//
typedef enum
{
    AM_HAL_SECURITY_LOCKTYPE_CUSTOTP_PROG   = 0x1,
    AM_HAL_SECURITY_LOCKTYPE_CUSTOTP_READ   = 0x2,
} am_hal_security_locktype_t;

//
//! Security 128b Key
//
typedef union
{
    uint32_t keyword[4];
    struct
    {
        uint32_t key0;
        uint32_t key1;
        uint32_t key2;
        uint32_t key3;
    } keys;
} am_hal_security_128bkey_t;

//
//! Security SOC Id
//
typedef struct
{
    uint32_t socid[AM_HAL_SECURITY_SOCID_NUMWORDS];
} am_hal_security_socid_t;

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @brief  Get Device Security Info
//!
//! @param  pSecInfo -  Pointer to structure for returned security info
//!
//! This will retrieve the security information for the device
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
uint32_t am_hal_security_get_info(am_hal_security_info_t *pSecInfo);

//*****************************************************************************
//
//! @brief  Get Device Security SOCID
//!
//! @param  pSocId -  Pointer to structure for returned SOCID info
//!
//! This will retrieve the SOCID information for the device
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
uint32_t am_hal_security_get_socid(am_hal_security_socid_t *pSocId);

//*****************************************************************************
//
//! @brief  Set the key for specified lock
//!
//! @param  lockType - The lock type to be operated upon
//! @param  pKey -  Pointer to 128b key value
//!
//! This will program the lock registers for the specified lock and key
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
uint32_t am_hal_security_set_key(am_hal_security_locktype_t lockType, am_hal_security_128bkey_t *pKey);

//*****************************************************************************
//
//! @brief  Get the current status of the specified lock
//!
//! @param lockType - The lock type to be operated upon
//! @param pbUnlockStatus -  Pointer to return variable with lock status
//!
//! This will get the lock status for specified lock - true implies unlocked
//! Note that except for customer lock, other locks are self-locking on status read
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
uint32_t am_hal_security_get_lock_status(am_hal_security_locktype_t lockType, bool *pbUnlockStatus);

//*****************************************************************************
//
//! @brief  Compute CRC32 for a specified payload
//!
//! @param  ui32StartAddr - The start address of the payload.
//! @param  ui32SizeBytes - The length of payload in bytes.
//! @param  pui32Crc      - Pointer to variable to return the computed CRC.
//!
//! This function uses the hardware engine to compute CRC32 on an arbitrary data
//! payload.  The payload can reside in any contiguous memory including external
//! memory.
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
uint32_t am_hal_crc32(uint32_t startAddr, uint32_t sizeBytes, uint32_t *pCrc);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_SECURITY_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

