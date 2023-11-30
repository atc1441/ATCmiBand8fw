//*****************************************************************************
//
//! @file am_devices_emmc_rpmb.c
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
#include <string.h>
#include "am_mcu_apollo.h"
#include "am_util.h"
#include "am_devices_emmc_rpmb.h"

#define MBEDTLS_CONFIG_FILE <config-cc312-apollo4-no-os.h>

#include "cc_lib.h"
#include "mbedtls/sha256.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "mbedtls/memory_buffer_alloc.h"

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************

//
// Context variables for mbedTLS operations.
//
CCRndContext_t g_RpmbRndContext;
CCRndWorkBuff_t g_RpmbRndWorkBuff;
mbedtls_ctr_drbg_context g_RpmbRndState;
mbedtls_entropy_context g_RpmbMbedtlsEntropy;

//
// Dynamic memory for mbedTLS stack.
//
uint32_t g_ui32RpmbMbedTLSHeap[AM_DEVICES_EMMC_MBEDTLS_HEAP_SIZE];

//
// Buffer of RPMB HMAC
//
uint8_t g_ui8RpmbHmac[AM_DEVICES_EMMC_RPMB_MAC_SIZE];

//
// Structure of RPMB Data Frame
//
am_devices_emmc_rpmb_t g_RpmbFrame;

//*****************************************************************************
//
// Initialize crypto hardware module.
//
//*****************************************************************************
uint32_t
am_devices_emmc_mebedtls_init(void)
{
    uint32_t ui32Status = 0;

    //
    // init Rnd context's inner member
    //
    g_RpmbRndContext.rndState = &g_RpmbRndState;
    g_RpmbRndContext.entropyCtx = &g_RpmbMbedtlsEntropy;

    //
    // Enable the Crypto module
    //
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);

    AM_DEVICE_RPMB_DEBUG("Powering On Crypto and mbedtls init\n");

    //
    // Initiailize MbedTLS
    //
    mbedtls_memory_buffer_alloc_init((uint8_t*)g_ui32RpmbMbedTLSHeap, AM_DEVICES_EMMC_MBEDTLS_HEAP_SIZE);
    ui32Status = CC_LibInit(&g_RpmbRndContext, &g_RpmbRndWorkBuff);
    if ( ui32Status )
    {
        AM_DEVICE_RPMB_DEBUG("CC_LibInit error = %x\n", ui32Status);
        return AM_DEVICES_EMMC_RPMB_STATUS_INIT_ERROR;
    }

    return ui32Status;
}

//*****************************************************************************
//
// Deinitialize crypto hardware module.
//
//*****************************************************************************
uint32_t
am_devices_emmc_mebedtls_deinit(void)
{
    uint32_t ui32Status = 0;

    //
    // Deinitiailize MbedTLS
    //
    mbedtls_memory_buffer_alloc_free();
    ui32Status = CC_LibFini(&g_RpmbRndContext);
    if ( ui32Status )
    {
        AM_DEVICE_RPMB_DEBUG("CC_LibDeinit error = %x\n", ui32Status);
        return AM_DEVICES_EMMC_RPMB_STATUS_INIT_ERROR;
    }

    //
    // disable the Crypto module
    //
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);

    return ui32Status;
}

//*****************************************************************************
//
// Calculate rpmb mac by random number
//
//*****************************************************************************
static uint32_t
am_devices_emmc_rpmb_hmac(uint8_t *pui8Key, uint8_t *pui8RandBuff, uint32_t ui32Len, uint8_t *pui8Output)
{
    mbedtls_sha256_context ctx;
    uint32_t i;
    uint32_t ui32Status = 0;
    uint8_t ui8IpadBuf[AM_DEVICES_EMMC_RPMB_SHA256_SIZE];
    uint8_t ui8OpadBuf[AM_DEVICES_EMMC_RPMB_SHA256_SIZE];

    memset(&ctx , 0, sizeof(ctx));

    ui32Status = mbedtls_sha256_starts_ret(&ctx, 0);
    if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
    {
        AM_DEVICE_RPMB_DEBUG("mbedtls sha256 starts failed%x\n", ui32Status);
        return AM_DEVICES_EMMC_RPMB_STATUS_HMAC_ERROR;
    }

    //
    // ui8IpadBuf is the byte 0x36 repeated blocksize times
    // ui8OpadBuf is the byte 0x5c repeated blocksize times
    //
    for (i = 0; i < AM_DEVICES_EMMC_RPMB_MAC_SIZE; i++)
    {
        ui8IpadBuf[i] = pui8Key[i] ^ 0x36;
        ui8OpadBuf[i] = pui8Key[i] ^ 0x5c;
    }

    for (i = AM_DEVICES_EMMC_RPMB_MAC_SIZE; i < AM_DEVICES_EMMC_RPMB_SHA256_SIZE; i++)
    {
        ui8IpadBuf[i] = 0x36;
        ui8OpadBuf[i] = 0x5c;
    }

    ui32Status = mbedtls_sha256_update_ret(&ctx, ui8IpadBuf, AM_DEVICES_EMMC_RPMB_SHA256_SIZE);
    if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
    {
        AM_DEVICE_RPMB_DEBUG("mbedtls sha256 update failed%x\n", ui32Status);
        return AM_DEVICES_EMMC_RPMB_STATUS_HMAC_ERROR;
    }

    ui32Status = mbedtls_sha256_update_ret(&ctx, pui8RandBuff, ui32Len);
    if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
    {
        AM_DEVICE_RPMB_DEBUG("mbedtls sha256 update failed%x\n", ui32Status);
        return AM_DEVICES_EMMC_RPMB_STATUS_HMAC_ERROR;
    }

    ui32Status = mbedtls_sha256_finish_ret(&ctx, pui8Output);
    if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
    {
        AM_DEVICE_RPMB_DEBUG("mbedtls sha256 finish failed%x\n", ui32Status);
        return AM_DEVICES_EMMC_RPMB_STATUS_HMAC_ERROR;
    }

    mbedtls_sha256_free(&ctx);

    //
    // Init context for second caculate
    //
    memset(&ctx , 0, sizeof(ctx));

    ui32Status = mbedtls_sha256_starts_ret(&ctx, 0);
    if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
    {
        AM_DEVICE_RPMB_DEBUG("mbedtls sha256 starts failed%x\n", ui32Status);
        return AM_DEVICES_EMMC_RPMB_STATUS_HMAC_ERROR;
    }

    ui32Status = mbedtls_sha256_update_ret(&ctx, ui8OpadBuf, AM_DEVICES_EMMC_RPMB_SHA256_SIZE);
    if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
    {
        AM_DEVICE_RPMB_DEBUG("mbedtls sha256 update failed%x\n", ui32Status);
        return AM_DEVICES_EMMC_RPMB_STATUS_HMAC_ERROR;
    }

    ui32Status = mbedtls_sha256_update_ret(&ctx, pui8Output, AM_DEVICES_EMMC_RPMB_MAC_SIZE);
    if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
    {
        AM_DEVICE_RPMB_DEBUG("mbedtls sha256 update failed%x\n", ui32Status);
        return AM_DEVICES_EMMC_RPMB_STATUS_HMAC_ERROR;
    }

    ui32Status = mbedtls_sha256_finish_ret(&ctx, pui8Output);
    if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
    {
        AM_DEVICE_RPMB_DEBUG("mbedtls sha256 finish failed%x\n", ui32Status);
        return AM_DEVICES_EMMC_RPMB_STATUS_HMAC_ERROR;
    }

    mbedtls_sha256_free(&ctx);

    return AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Send rpmb request to emmc
//
//*****************************************************************************
static uint32_t
am_devices_emmc_rpmb_request(am_hal_card_t *pCard, am_devices_emmc_rpmb_t *pRpmb, bool bRelWrite)
{
    uint32_t ui32Status;

    ui32Status = am_hal_card_block_rpmb_rw(pCard, (uint8_t *)pRpmb, false, bRelWrite);
    if ( (ui32Status & 0xffff) != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
    {
        AM_DEVICE_RPMB_DEBUG("Request Execute Cmd Error:%x\n", ui32Status);
        return AM_DEVICES_EMMC_RPMB_STATUS_CMD_ERROR;
    }

    return AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Get rpmb response from emmc
//
//*****************************************************************************
static uint32_t
am_devices_emmc_rpmb_response(am_hal_card_t *pCard, am_devices_emmc_rpmb_t *pRpmb, uint32_t ui32Expected)
{
    uint32_t ui32Status;

    ui32Status = am_hal_card_block_rpmb_rw(pCard, (uint8_t *)pRpmb, true, false);
    if ( (ui32Status & 0xffff) != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
    {
        AM_DEVICE_RPMB_DEBUG("Response Execute Cmd Error:%x\n", ui32Status);
        return AM_DEVICES_EMMC_RPMB_STATUS_CMD_ERROR;
    }

    //
    //  Check the response and the status
    //
    if ( USWAP_16(pRpmb->ui16Request) != ui32Expected )
    {
        AM_DEVICE_RPMB_DEBUG("Response:%x not match expected:%x\n", pRpmb->ui16Request, ui32Expected);
        return AM_DEVICES_EMMC_RPMB_STATUS_RESPONSE_ERROR;
    }

    ui32Status = USWAP_16(pRpmb->ui16Result);
    if (ui32Status)
    {
        AM_DEVICE_RPMB_DEBUG("%s \n", am_devices_emmc_rpmb_err_msg[ui32Status & AM_DEVICES_EMMC_RPMB_ERROR_MASK]);
        return ui32Status;
    }
    return AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Get response from emmc
//
//*****************************************************************************
static uint32_t
am_devices_emmc_rpmb_status(am_hal_card_t *pCard, uint32_t ui32Expected)
{
    uint32_t ui32Status;
    memset(&g_RpmbFrame, 0x0, sizeof(g_RpmbFrame));

    g_RpmbFrame.ui16Request = USWAP_16(AM_DEVICES_EMMC_RPMB_REQ_STATUS);

    ui32Status = am_devices_emmc_rpmb_request(pCard, &g_RpmbFrame, false);
    if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
    {
        AM_DEVICE_RPMB_DEBUG("Failed to request when get status\n");
        return ui32Status;
    }

    //
    // Read the ui16Result
    //
    ui32Status = am_devices_emmc_rpmb_response(pCard, &g_RpmbFrame, ui32Expected);
    if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
    {
        AM_DEVICE_RPMB_DEBUG("Failed to response when get status\n");
        return ui32Status;
    }

    return AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Switch partition mode
//
//*****************************************************************************
uint32_t
am_devices_emmc_rpmb_partition_switch(am_hal_card_t *pCard, am_devices_emmc_partiton_access_e ePartionMode)
{
    uint32_t ui32Mode;
    uint32_t ui32Status = 0;

    ui32Mode = MMC_EXT_MODE_WRITE_BYTE | MMC_EXT_REGS_BOOT_CONFIG | (ePartionMode << 8);
    ui32Status = am_hal_card_mode_switch(pCard, ui32Mode, DEFAULT_CMD6_TIMEOUT_MS);
    if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
    {
        AM_DEVICE_RPMB_DEBUG("Failed to switch partition config\n");
        return AM_DEVICES_EMMC_RPMB_STATUS_SWITCH_PARTITION_ERROR;
    }

    //
    // read back the ext_csd and check the partition access mode
    //
    pCard->bExtCsdValid = false;
    ui32Mode = am_hal_card_get_ext_csd_field(pCard, MMC_EXT_REGS_PARTITON_CONFIG, 1);
    if ( ui32Mode != ePartionMode )
    {
        AM_DEVICE_RPMB_DEBUG("%d - Failed to switch partition\n", __FUNCTION__);
        return AM_DEVICES_EMMC_RPMB_STATUS_SWITCH_PARTITION_ERROR;
    }

    return AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Get write counter in RPMB
//
//*****************************************************************************
uint32_t
am_devices_emmc_rpmb_get_counter(am_hal_card_t *pCard, uint32_t *pCounter, uint8_t *pui8Key)
{
    uint32_t ui32Status = 0;

    if ( !pui8Key )
    {
        return AM_DEVICES_EMMC_RPMB_STATUS_INVALID_ARG;
    }

    memset(&g_RpmbFrame, 0x0, sizeof(g_RpmbFrame));

    g_RpmbFrame.ui16Request = USWAP_16(AM_DEVICES_EMMC_RPMB_REQ_WCOUNTER);

    mbedtls_ctr_drbg_random(g_RpmbRndContext.rndState, (uint8_t *)g_RpmbFrame.ui8Nonce, AM_DEVICES_EMMC_RPMB_NONCE_SIZE);

    ui32Status = am_devices_emmc_rpmb_request(pCard, &g_RpmbFrame, false);
    if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
    {
        AM_DEVICE_RPMB_DEBUG("Failed to request when get counter\n");
        return ui32Status;
    }

    //
    // Read the Result
    //
    ui32Status = am_devices_emmc_rpmb_response(pCard, &g_RpmbFrame, AM_DEVICES_EMMC_RPMB_RESP_WCOUNTER);
    if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
    {
        AM_DEVICE_RPMB_DEBUG("Failed to response when get counter\n");
        return ui32Status;
    }

    if ( am_devices_emmc_rpmb_hmac(pui8Key, (uint8_t *)g_RpmbFrame.ui8Data, AM_DEVICES_EMMC_RPMB_SHA256_HMAC_SIZE, g_ui8RpmbHmac) != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
    {
        AM_DEVICE_RPMB_DEBUG("Failed to check key get counter\n");
        return AM_DEVICES_EMMC_RPMB_STATUS_HMAC_ERROR;
    }

    if (memcmp(g_ui8RpmbHmac, &g_RpmbFrame.ui8Mac, AM_DEVICES_EMMC_RPMB_MAC_SIZE))
    {
        AM_DEVICE_RPMB_DEBUG("MAC error on block get counter\n");
        return AM_DEVICES_EMMC_RPMB_STATUS_AUTHENTICATION_ERROR;
    }

    *pCounter = USWAP_32(g_RpmbFrame.ui32WriteCnt);

    return AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Write secure key to eMMC's OTP area
//
//*****************************************************************************
uint32_t
am_devices_emmc_rpmb_set_key(am_hal_card_t *pCard, uint8_t *pui8Key)
{
    uint32_t ui32Status = 0;

    if ( !pui8Key )
    {
        return AM_DEVICES_EMMC_RPMB_STATUS_INVALID_ARG;
    }

    memset(&g_RpmbFrame, 0x0, sizeof(g_RpmbFrame));
    g_RpmbFrame.ui16Request = USWAP_16(AM_DEVICES_EMMC_RPMB_REQ_KEY);

    memcpy(&g_RpmbFrame.ui8Mac, pui8Key, AM_DEVICES_EMMC_RPMB_MAC_SIZE);

    ui32Status = am_devices_emmc_rpmb_request(pCard, &g_RpmbFrame, true);
    if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
    {
        AM_DEVICE_RPMB_DEBUG("Failed to request set key\n");
        return ui32Status;
    }

    //
    // read the operation status
    //
    ui32Status = am_devices_emmc_rpmb_status(pCard, AM_DEVICES_EMMC_RPMB_RESP_KEY);
    if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
    {
        AM_DEVICE_RPMB_DEBUG("Failed to read status\n");
        return ui32Status;
    }

    return AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Read data from RPMB area
//
//*****************************************************************************
uint32_t
am_devices_emmc_rpmb_read(am_hal_card_t *pCard, uint8_t *pui8RxBuffer, uint32_t ui32Blk, uint32_t ui32BlkCnt, uint8_t *pui8Key)
{
    uint32_t ui32Cnt = 0 ;
    uint32_t ui32Status = 0;

    if ( !pui8Key )
    {
        return AM_DEVICES_EMMC_RPMB_STATUS_INVALID_ARG;
    }

    for (ui32Cnt = 0; ui32Cnt < ui32BlkCnt; ui32Cnt++)
    {
        memset(&g_RpmbFrame, 0x0, sizeof(g_RpmbFrame));
        g_RpmbFrame.ui16Address = USWAP_16(ui32Blk + ui32Cnt);
        g_RpmbFrame.ui16Request = USWAP_16(AM_DEVICES_EMMC_RPMB_REQ_READ_DATA);

        mbedtls_ctr_drbg_random(g_RpmbRndContext.rndState, (uint8_t *)g_RpmbFrame.ui8Nonce, AM_DEVICES_EMMC_RPMB_NONCE_SIZE);

        ui32Status = am_devices_emmc_rpmb_request(pCard, &g_RpmbFrame, false);
        if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
        {
            AM_DEVICE_RPMB_DEBUG("Failed to request in rpmb read\n");
            return ui32Status;
        }

        ui32Status = am_devices_emmc_rpmb_response(pCard, &g_RpmbFrame, AM_DEVICES_EMMC_RPMB_RESP_READ_DATA);
        if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
        {
            AM_DEVICE_RPMB_DEBUG("Failed to response in rpmb read\n");
            return ui32Status;
        }

        //
        // Check the HMAC if key is provided
        //
        ui32Status = am_devices_emmc_rpmb_hmac(pui8Key, (uint8_t *)g_RpmbFrame.ui8Data, AM_DEVICES_EMMC_RPMB_SHA256_HMAC_SIZE, g_ui8RpmbHmac);
        if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
        {
            AM_DEVICE_RPMB_DEBUG("Failed to caculate mac in read\n");
            return AM_DEVICES_EMMC_RPMB_STATUS_HMAC_ERROR;
        }

        if (memcmp(g_ui8RpmbHmac, &g_RpmbFrame.ui8Mac, AM_DEVICES_EMMC_RPMB_MAC_SIZE))
        {
            AM_DEVICE_RPMB_DEBUG("MAC error on block #%d\n", ui32Cnt);
            return AM_DEVICES_EMMC_RPMB_STATUS_AUTHENTICATION_ERROR;
        }

        //
        // Copy data
        //
        memcpy((void *)(pui8RxBuffer + ui32Cnt * AM_DEVICES_EMMC_RPMB_DATA_SIZE), &g_RpmbFrame.ui8Data, AM_DEVICES_EMMC_RPMB_DATA_SIZE);
    }

    return AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS;
}


//*****************************************************************************
//
// Write data to RPMB area
//
//*****************************************************************************
uint32_t
am_devices_emmc_rpmb_write(am_hal_card_t *pCard, uint8_t *pui8TxBuffer, uint32_t ui32Blk, uint32_t ui32BlkCnt, uint8_t *pui8Key)
{
    uint32_t ui32Count = 0;
    uint32_t ui32Status = 0;

    if ( !pui8Key )
    {
        return AM_DEVICES_EMMC_RPMB_STATUS_INVALID_ARG;
    }

    for (uint32_t i = 0; i < ui32BlkCnt; i++)
    {
        ui32Status = am_devices_emmc_rpmb_get_counter(pCard, &ui32Count, pui8Key);
        if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
        {
            AM_DEVICE_RPMB_DEBUG("Failed to get counter in rpmb write\n");
            return ui32Status;
        }

        memset(&g_RpmbFrame, 0x0, sizeof(g_RpmbFrame));
        memcpy(&g_RpmbFrame.ui8Data, (void *)(pui8TxBuffer + i * AM_DEVICES_EMMC_RPMB_DATA_SIZE), AM_DEVICES_EMMC_RPMB_DATA_SIZE);

        g_RpmbFrame.ui16BlockCnt = USWAP_16(1);
        g_RpmbFrame.ui32WriteCnt = USWAP_32(ui32Count);
        g_RpmbFrame.ui16Address = USWAP_16(ui32Blk + i);
        g_RpmbFrame.ui16Request = USWAP_16(AM_DEVICES_EMMC_RPMB_REQ_WRITE_DATA);

        //
        // Computes HMAC
        //
        ui32Status = am_devices_emmc_rpmb_hmac(pui8Key, (uint8_t *)g_RpmbFrame.ui8Data, AM_DEVICES_EMMC_RPMB_SHA256_HMAC_SIZE, (uint8_t *)g_RpmbFrame.ui8Mac);
        if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
        {
            AM_DEVICE_RPMB_DEBUG("Failed to caculate hmac in rpmb write\n");
            return ui32Status;
        }

        ui32Status = am_devices_emmc_rpmb_request(pCard, &g_RpmbFrame, true);
        if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
        {
            AM_DEVICE_RPMB_DEBUG("Failed to request in rpmb write\n");
            return ui32Status;
        }

        //
        // read the operation status
        //
        ui32Status = am_devices_emmc_rpmb_status(pCard, AM_DEVICES_EMMC_RPMB_RESP_WRITE_DATA);
        if ( ui32Status != AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS )
        {
            AM_DEVICE_RPMB_DEBUG("Failed to read status in rpmb write\n");
            return ui32Status;
        }
    }

    return AM_DEVICES_EMMC_RPMB_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************