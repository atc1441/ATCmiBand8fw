//*****************************************************************************
//
//! @file am_devices_emmc_rpmb.h
//!
//! @brief Driver for eMMC RPMB(Replay Protected Memory Block) feature.
//! This driver contains functions for accessing eMMC PRMB partition.
//!
//! @addtogroup emmc_rpmb emmc driver
//! @ingroup devices
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
#ifndef AM_DEVICES_EMMC_RPMB_H
#define AM_DEVICES_EMMC_RPMB_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name Global definitions for emmc rpmb request type
//! @{
//
//*****************************************************************************
#define AM_DEVICES_EMMC_RPMB_REQ_KEY            1
#define AM_DEVICES_EMMC_RPMB_REQ_WCOUNTER       2
#define AM_DEVICES_EMMC_RPMB_REQ_WRITE_DATA     3
#define AM_DEVICES_EMMC_RPMB_REQ_READ_DATA      4
#define AM_DEVICES_EMMC_RPMB_REQ_STATUS         5
//! @}

//*****************************************************************************
//
//! @name Global definitions for emmc rpmb reponse type
//! @{
//
//*****************************************************************************
#define AM_DEVICES_EMMC_RPMB_RESP_KEY           0x0100
#define AM_DEVICES_EMMC_RPMB_RESP_WCOUNTER      0x0200
#define AM_DEVICES_EMMC_RPMB_RESP_WRITE_DATA    0x0300
#define AM_DEVICES_EMMC_RPMB_RESP_READ_DATA     0x0400
//! @}

//*****************************************************************************
//
//! @name Global definitions for size of rpmb data frame
//! @{
//
//*****************************************************************************
#define AM_DEVICES_EMMC_RPMB_STUFF_SIZE        196
#define AM_DEVICES_EMMC_RPMB_MAC_SIZE          32
#define AM_DEVICES_EMMC_RPMB_DATA_SIZE         256
#define AM_DEVICES_EMMC_RPMB_NONCE_SIZE        16
#define AM_DEVICES_EMMC_RPMB_SHA256_SIZE       64
#define AM_DEVICES_EMMC_RPMB_SHA256_HMAC_SIZE  (256 + 16 + 12)
//! @}

//*****************************************************************************
//
//! @name Global definitions for the emmc rpmb error mask
//! @{
//
//*****************************************************************************
#define AM_DEVICES_EMMC_RPMB_ERROR_MASK        0x07
//! @}

//*****************************************************************************
//
//! @name Global definitions for the emmc rpmb function
//! @{
//
//*****************************************************************************
#define AM_DEVICE_RPMB_DEBUG(fmt, ...)          am_util_debug_printf("[RPMB] line %04d - "fmt, __LINE__, ##__VA_ARGS__)
#define AM_DEVICES_EMMC_MBEDTLS_HEAP_SIZE       (1024)

#define USWAP_16(x)    ((((x) & 0xff00) >> 8) | (((x) & 0x00ff) << 8))

#define USWAP_32(x)    ((((x) & 0xff000000) >> 24) | (((x) & 0x00ff0000) >>  8) | \
                        (((x) & 0x0000ff00) <<  8) | (((x) & 0x000000ff) << 24))
//! @}

//*****************************************************************************
//
// Global type definitions.
//
//*****************************************************************************

typedef enum
{
    AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS,
    AM_DEVICES_EMMC_RPMB_STATUS_GENERAL_ERROR,
    AM_DEVICES_EMMC_RPMB_STATUS_AUTHENTICATION_ERROR,
    AM_DEVICES_EMMC_RPMB_STATUS_COUNTER_ERROR,
    AM_DEVICES_EMMC_RPMB_STATUS_ADDRESS_ERROR,
    AM_DEVICES_EMMC_RPMB_STATUS_WRITE_ERROR,
    AM_DEVICES_EMMC_RPMB_STATUS_READ_ERROR,
    AM_DEVICES_EMMC_RPMB_STATUS_KEY_NOT_PROGRAMMED_ERROR,
    AM_DEVICES_EMMC_RPMB_STATUS_SWITCH_PARTITION_ERROR,
    AM_DEVICES_EMMC_RPMB_STATUS_RESPONSE_ERROR,
    AM_DEVICES_EMMC_RPMB_STATUS_CMD_ERROR,
    AM_DEVICES_EMMC_RPMB_STATUS_INVALID_ARG,
    AM_DEVICES_EMMC_RPMB_STATUS_HMAC_ERROR,
    AM_DEVICES_EMMC_RPMB_STATUS_INIT_ERROR,
    AM_DEVICES_EMMC_RPMB_STATUS_CNT_EXPIRED_ERROR = 0x80,
} am_devices_emmc_rpmb_status_e;


typedef enum
{
    AM_DEVICES_EMMC_NO_BOOT_ACCESS,
    AM_DEVICES_EMMC_BOOT1_ACCESS,
    AM_DEVICES_EMMC_BOOT2_ACCESS,
    AM_DEVICES_EMMC_RPMB_ACCESS,
} am_devices_emmc_partiton_access_e;

// eMMC rpmb error messages
static const char * const am_devices_emmc_rpmb_err_msg[] = {
    "",
    "General failure",
    "Authentication failure",
    "Counter failure",
    "Address failure",
    "Write failure",
    "Read failure",
    "Authentication key not yet programmed",
};

typedef struct
{
    uint8_t ui8Stuff[AM_DEVICES_EMMC_RPMB_STUFF_SIZE];
    uint8_t ui8Mac[AM_DEVICES_EMMC_RPMB_MAC_SIZE];
    uint8_t ui8Data[AM_DEVICES_EMMC_RPMB_DATA_SIZE];
    uint8_t ui8Nonce[AM_DEVICES_EMMC_RPMB_NONCE_SIZE];
    uint32_t ui32WriteCnt;
    uint16_t ui16Address;
    uint16_t ui16BlockCnt;
    uint16_t ui16Result;
    uint16_t ui16Request;
}  am_devices_emmc_rpmb_t;

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief Initialize the mebedtls hardware crypto module.
//!
//! This function is used to initialize crypto hardware module.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_devices_emmc_mebedtls_init(void);

//*****************************************************************************
//
//! @brief deinitialize the mebedtls hardware crypto module.
//!
//! This function is used to deinitialize crypto hardware module.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_devices_emmc_mebedtls_deinit(void);

//*****************************************************************************
//
//! @brief Switch eMMC card partiton access
//! @param pCard               - pointer to the card instance.
//! @param ePartionMode        - eMMC partition access mode
//!
//! This function is used to switch eMMC card partiton access according to the ePartionMode
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_devices_emmc_rpmb_partition_switch(am_hal_card_t *pCard, am_devices_emmc_partiton_access_e ePartionMode);

//*****************************************************************************
//
//! @brief Get counter in RPMB write counter.
//!
//! @param pCard        - pointer to the card instance.
//! @param pui32Counter - pointer to write counter in RPMB
//! @param pui8Key      - pointer to secure key in eMMC RPMB
//!
//! This function is get write counter from eMMC, which is read only.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_devices_emmc_rpmb_get_counter(am_hal_card_t *pCard, uint32_t *pui32Counter, uint8_t *pui8Key);

//*****************************************************************************
//
//! @brief Set secure key to eMMC.
//!
//! @param pCard       - pointer to the card instance.
//! @param pui8Key     - pointer to secure key
//!
//! This function is used to write secure key to eMMC's OTP area.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_devices_emmc_rpmb_set_key(am_hal_card_t *pCard,  uint8_t *pui8Key);

//*****************************************************************************
//
//! @brief eMMC RPMB read function
//!
//! @param pCard         - pointer to the card instance.
//! @param pui8RxBuffer  - Buffer to store the received data from the eMMC RPMB
//! @param ui32Blk       - start block number
//! @param ui32BlkCnt    - read block count
//! @param pui8Key       - pointer to secure key
//!
//! This function reads the 'ui32BlkCnt' blocks starting from 'ui32Blk' block and
//! saves the data in the 'pui8RxBuffer' read buffer. The caller will be blocked until all data has been sent
//! out or failed.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_devices_emmc_rpmb_read(am_hal_card_t *pCard, uint8_t *pui8RxBuffer, uint32_t ui32Blk, uint32_t ui32BlkCnt, uint8_t *pui8Key);

//*****************************************************************************
//
//! @brief eMMC RPMB write function
//!
//! @param pCard         - pointer to the card instance.
//! @param pui8TxBuffer  - Buffer to write the eMMC RPMB
//! @param ui32Blk       - start block number
//! @param ui32BlkCnt    - write block count
//! @param pui8Key        -pointer to secure key
//!
//! This function writes 'ui32BlkCnt' blocks in the 'pui8TxBuffer' write buffer to the card blocks
//! starting from 'ui32Blk' block. The caller will be blocked until all data has been sent
//! out or failed.
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_devices_emmc_rpmb_write(am_hal_card_t *pCard, uint8_t *pui8TxBuffer, uint32_t ui32Blk, uint32_t ui32BlkCnt, uint8_t *pui8Key);

#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_EMMC_RPMB_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

