//*****************************************************************************
//
//! @file am_util_ble_cooper.c
//!
//! @brief Cooper BLE functions not covered by the HAL.
//!
//! This file contains functions for interacting with the Apollo4 BLE hardware
//! that are not already covered by the HAL. Most of these commands either
//! adjust RF settings or facilitate RF testing operations.
//!
//! @addtogroup ble_cooper Cooper - BLE Functions
//! @ingroup utils
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
#include <string.h>
#include "am_util_delay.h"
#include "am_mcu_apollo.h"
#include "am_devices_cooper.h"

#define COOPER_INFO0_BASE                 (0x60000000)
#define COOPER_INFO0_TRIM_VER_OFFSET      (0x60)
#define COOPER_INOF0_RETV_TRIM_VER_OFFSET (0X64)

//*****************************************************************************
//
// Statics
//
//*****************************************************************************

//*****************************************************************************
//
// Globals
//
//*****************************************************************************
//*****************************************************************************
//
// Read a register value from the BLE core.
//
//*****************************************************************************
uint32_t
am_util_ble_plf_reg_read(void* pHandle, uint32_t ui32Address, uint32_t* pui32Value)
{
    //
    // Fill the buffer with the specific command we want to write, and send it.
    //
    uint8_t write_cmd[HCI_VSC_CMD_LENGTH(HCI_VSC_REG_RD_CMD_LENGTH)] =
                                               HCI_VSC_CMD(HCI_VSC_REG_RD,
                                               UINT32_TO_BYTE0(ui32Address), UINT32_TO_BYTE1(ui32Address), UINT32_TO_BYTE2(ui32Address), UINT32_TO_BYTE3(ui32Address));
    //
    // Make a buffer big enough to hold the register write command, and a
    // second one big enough to hold the response.
    //
    uint32_t ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_SUCCESS;
    uint32_t ui32BytesNum = 0;
    am_devices_cooper_buffer(32) sResponse = {0};
    ui32ErrorStatus = am_devices_cooper_command_write(pHandle, (uint32_t*)write_cmd, sizeof(write_cmd), sResponse.words, &ui32BytesNum);
    *pui32Value = (((sResponse.words[2] & 0xFF000000) >> 24) |
                   ((sResponse.words[3] & 0x00FFFFFF) << 8));
    return ui32ErrorStatus;
}

//*****************************************************************************
//
// Write a register value to the BLE core.
//
//*****************************************************************************
uint32_t
am_util_ble_plf_reg_write(void* pHandle, uint32_t ui32Address, uint32_t ui32Value)
{
    //
    // Fill the buffer with the specific command we want to write, and send it.
    //
    uint8_t write_cmd[HCI_VSC_CMD_LENGTH(HCI_VSC_REG_WR_CMD_LENGTH)] =
                                               HCI_VSC_CMD(HCI_VSC_REG_WR,
                                               UINT32_TO_BYTE0(ui32Address), UINT32_TO_BYTE1(ui32Address), UINT32_TO_BYTE2(ui32Address), UINT32_TO_BYTE3(ui32Address),
                                               UINT32_TO_BYTE0(ui32Value), UINT32_TO_BYTE1(ui32Value), UINT32_TO_BYTE2(ui32Value), UINT32_TO_BYTE3(ui32Value));
    //
    // Make a buffer big enough to hold the register write command, and a
    // second one big enough to hold the response.
    //
    uint32_t ui32BytesNum = 0;
    am_devices_cooper_buffer(16) sResponse = {0};
    return (am_devices_cooper_command_write(pHandle, (uint32_t*)write_cmd, sizeof(write_cmd), sResponse.words, &ui32BytesNum));
}

//*****************************************************************************
//
// Manually enable/disable transmitter to output carrier signal
// set ui8TxChannel as 0 to 0x27 for each transmit channel, 0xFF back to normal modulate mode
//
//*****************************************************************************
uint32_t
am_util_ble_hci_reset(void *pHandle)
{
    //
    // Fill the buffer with the specific command we want to write, and send it.
    //
    uint8_t write_cmd[HCI_VSC_CMD_LENGTH(0)] = HCI_RAW_CMD(0x0C03, 0);

    //
    // Make a buffer big enough to hold the register write command, and a
    // second one big enough to hold the response.
    //
    uint32_t ui32BytesNum = 0;
    am_devices_cooper_buffer(16) sResponse = {0};

    return (am_devices_cooper_command_write(pHandle, (uint32_t*)write_cmd, sizeof(write_cmd), sResponse.words, &ui32BytesNum));
}

//*****************************************************************************
//
// Set BLE sleep enable/disable for the BLE core.
// enable = 'true' set sleep enable, enable = 'false' set sleep disable
//
//*****************************************************************************
uint32_t
am_util_ble_sleep_set(void* pHandle, bool enable)
{
    uint32_t ui32BytesNum = 0;
    am_devices_cooper_buffer(16) sResponse = {0};
    uint32_t ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_SUCCESS;

    if ( enable )
    {
        uint8_t write_cmd[HCI_VSC_CMD_LENGTH(HCI_VSC_UPDATE_NVDS_CFG_CMD_LENGTH)] = HCI_VSC_CMD(HCI_VSC_UPDATE_NVDS_CFG, NVDS_PARAMETER_MAGIC_NUMBER, NVDS_PARAMETER_SLEEP_ENABLE);
        ui32ErrorStatus = am_devices_cooper_command_write(pHandle, (uint32_t*)write_cmd, sizeof(write_cmd), sResponse.words, &ui32BytesNum);
    }
    else
    {
        uint8_t write_cmd[HCI_VSC_CMD_LENGTH(HCI_VSC_UPDATE_NVDS_CFG_CMD_LENGTH)] = HCI_VSC_CMD(HCI_VSC_UPDATE_NVDS_CFG, NVDS_PARAMETER_MAGIC_NUMBER, NVDS_PARAMETER_SLEEP_DISABLE);
        ui32ErrorStatus = am_devices_cooper_command_write(pHandle, (uint32_t*)write_cmd, sizeof(write_cmd), sResponse.words, &ui32BytesNum);
    }

    if ( ui32ErrorStatus != AM_DEVICES_COOPER_STATUS_SUCCESS )
    {
        return ui32ErrorStatus;
    }

    am_util_delay_ms(1);
    am_util_ble_hci_reset(pHandle);

    return ui32ErrorStatus;
}

//*****************************************************************************
//
// set the tx power of BLE
// values.
// ui32TxPower: enum txPowerLevel_t defined in hci_drv_cooper.h
//
//*****************************************************************************
uint32_t
am_util_ble_tx_power_set(void* pHandle, uint8_t ui32TxPower)
{
    //
    // Fill the buffer with the specific command we want to write, and send it.
    //
    uint8_t write_cmd[HCI_VSC_CMD_LENGTH(HCI_VSC_SET_TX_POWER_LEVEL_CFG_CMD_LENGTH)] = HCI_VSC_CMD(HCI_VSC_SET_TX_POWER_LEVEL_CFG, ui32TxPower);

    //
    // Make a buffer big enough to hold the register write command, and a
    // second one big enough to hold the response.
    //
    uint32_t ui32BytesNum = 0;
    am_devices_cooper_buffer(16) sResponse = {0};
    return (am_devices_cooper_command_write(pHandle, (uint32_t*)write_cmd, sizeof(write_cmd), sResponse.words, &ui32BytesNum));
}

//*****************************************************************************
//
// Write NVDS parameters to the BLE core.
//
//*****************************************************************************
uint32_t
am_util_ble_nvds_set(void *pHandle, uint8_t* pui8NVDS, uint8_t ui8Length)
{
    uint32_t ui32BytesNum = 0;
    am_devices_cooper_buffer(16) sResponse = {0};
    uint32_t ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_SUCCESS;
    uint8_t write_cmd[HCI_VSC_CMD_LENGTH(HCI_VSC_UPDATE_NVDS_CFG_CMD_LENGTH)] = HCI_VSC_CMD(HCI_VSC_UPDATE_NVDS_CFG, NVDS_PARAMETER_MAGIC_NUMBER);
    memcpy(&write_cmd[HCI_VSC_UPDATE_NVDS_CFG_CMD_OFFSET], pui8NVDS, ui8Length);

    ui32ErrorStatus = am_devices_cooper_command_write(pHandle, (uint32_t*)write_cmd, sizeof(write_cmd), sResponse.words, &ui32BytesNum);
    if ( ui32ErrorStatus != AM_DEVICES_COOPER_STATUS_SUCCESS )
    {
        return ui32ErrorStatus;
    }

    am_util_delay_ms(1);
    am_util_ble_hci_reset(pHandle);

    return ui32ErrorStatus;
}

//*****************************************************************************
//
// Write update signature to the BLE core.
//
//*****************************************************************************
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
uint32_t
am_util_ble_update_sign_set(void *pHandle, uint32_t ui32Sign)
{
    am_devices_cooper_t *pBle = (am_devices_cooper_t *)pHandle;
    uint32_t ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_SUCCESS;
    uint32_t ui32BytesNum = 0;
    am_devices_cooper_buffer(16) sResponse = {0};

    //
    // Fill the buffer with the specific command we want to write, and send it.
    //
    uint8_t write_cmd[HCI_VSC_CMD_LENGTH(HCI_VSC_UPDATE_FW_CFG_CMD_LENGTH)] = \
                                            HCI_VSC_CMD(HCI_VSC_UPDATE_FW_CFG,
                                            UINT32_TO_BYTE0(ui32Sign),
                                            UINT32_TO_BYTE1(ui32Sign),
                                            UINT32_TO_BYTE2(ui32Sign),
                                            UINT32_TO_BYTE3(ui32Sign));

    ui32ErrorStatus = am_devices_cooper_command_write(pHandle, (uint32_t*)write_cmd, sizeof(write_cmd), sResponse.words, &ui32BytesNum);
    if ( ui32ErrorStatus != AM_DEVICES_COOPER_STATUS_SUCCESS )
    {
        return ui32ErrorStatus;
    }

    //
    // There's different signature writing process for versions lower than 1.14, need to wait flash writing finishes
    //
    if ( pBle->ui32Firmver < 0x10E )
    {
        am_util_delay_ms(2000);
    }
    return (sResponse.bytes[ui32BytesNum - 1]);
}
#endif

//*****************************************************************************
//
// to do directly output modulation signal. change channel ranges from 0 to 0x27, pattern from 0 to 7.
//
//*****************************************************************************
uint32_t
am_util_ble_trasmitter_test_ex(void *pHandle, uint8_t channel, uint8_t pattern)
{
    //
    // Fill the buffer with the specific command we want to write, and send it.
    //
    uint8_t write_cmd[HCI_VSC_CMD_LENGTH(3)] = HCI_RAW_CMD(0x201E, 3, channel, 0x25, pattern);

    //
    // Make a buffer big enough to hold the register write command, and a
    // second one big enough to hold the response.
    //
    uint32_t ui32BytesNum = 0;
    am_devices_cooper_buffer(16) sResponse = {0};

    return (am_devices_cooper_command_write(pHandle, (uint32_t*)write_cmd, sizeof(write_cmd), sResponse.words, &ui32BytesNum));
}

//*****************************************************************************
//
// to do directly receiver test. change channel ranges from 0 to 0x27, return received packets in 100ms.
//
//*****************************************************************************
uint32_t
am_util_ble_receiver_test_ex(void *pHandle, uint8_t channel, uint32_t *recvpackets)
{
    //
    // Fill the buffer with the specific command we want to write, and send it.
    //
    uint32_t ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_SUCCESS;
    uint8_t start_cmd[HCI_VSC_CMD_LENGTH(1)] = HCI_RAW_CMD(0x201D, 1, channel);
    uint8_t end_cmd[HCI_VSC_CMD_LENGTH(0)] = HCI_RAW_CMD(0x201F, 0);

    //
    // Make a buffer big enough to hold the register write command, and a
    // second one big enough to hold the response.
    //
    uint32_t ui32BytesNum = 0;
    am_devices_cooper_buffer(16) sResponse = {0};

    //
    // issue the HCI command with to init for the channel 1
    //
    ui32ErrorStatus = am_devices_cooper_command_write(pHandle, (uint32_t*)start_cmd, sizeof(start_cmd), sResponse.words, &ui32BytesNum);
    if ( ui32ErrorStatus != AM_DEVICES_COOPER_STATUS_SUCCESS )
    {
        return ui32ErrorStatus;
    }

    am_util_delay_ms(100);

    //
    // issue the HCI command with to stop test for the channel 1
    //
    ui32ErrorStatus = am_devices_cooper_command_write(pHandle, (uint32_t*)end_cmd, sizeof(end_cmd), sResponse.words, &ui32BytesNum);
    *recvpackets = (sResponse.bytes[8] << 8) + sResponse.bytes[7];

    return ui32ErrorStatus;
}

//*****************************************************************************
//
// Dump info0 of BLE controller for debug use.
//
//*****************************************************************************
uint32_t
am_util_ble_info0_dump(void *pHandle)
{
    uint32_t i = 0;
    uint32_t start_addr = 0x60000000;

    for ( i = 0; i < 66; i++ )
    {
        uint32_t reg_value;
        am_util_ble_plf_reg_read(pHandle, start_addr, &reg_value);
        am_util_stdio_printf("0x%04x = 0x%08x\r\n", start_addr, reg_value);
        start_addr += 4;
    }

    return AM_DEVICES_COOPER_STATUS_SUCCESS;
}

//*****************************************************************************
//
// get cooper TRIM version.
//
// return Status code.
//
// pui32TrimVer: The uint32_t that will receive the trim version number.
//
//*****************************************************************************
uint32_t
am_util_ble_trim_version_get(void* pHandle, uint32_t *pui32TrimVer, uint32_t *pui32RetvTrimVer)
{
    uint32_t ui32ErrorStatus, reg_value;
    uint32_t start_addr = COOPER_INFO0_BASE + COOPER_INFO0_TRIM_VER_OFFSET;

    ui32ErrorStatus = am_util_ble_plf_reg_read(pHandle, start_addr, &reg_value);
    am_util_stdio_printf("Cooper trim version: 0x%04x = 0x%08x\r\n", start_addr, reg_value);

    if ( ui32ErrorStatus == AM_DEVICES_COOPER_STATUS_SUCCESS )
    {
        if ( pui32TrimVer )
        {
            *pui32TrimVer = reg_value;
        }
    }
    else
    {
        return ui32ErrorStatus;
    }

    start_addr = COOPER_INFO0_BASE + COOPER_INOF0_RETV_TRIM_VER_OFFSET;
    ui32ErrorStatus = am_util_ble_plf_reg_read(pHandle, start_addr, &reg_value);
    am_util_stdio_printf("Cooper retention voltage trim version: 0x%04x = 0x%08x\r\n", start_addr, reg_value);

    if ( ui32ErrorStatus == AM_DEVICES_COOPER_STATUS_SUCCESS )
    {
        if ( pui32RetvTrimVer )
        {
            *pui32RetvTrimVer = reg_value;
        }
    }

    return ui32ErrorStatus;
}

//*****************************************************************************
//
// API to disable the BLE controller's firmware rollback version (Enabled in default)
// Should be called as the very last step during manufacturing, after it done,
// the BLE controller will reset.
//
//*****************************************************************************
uint32_t
am_util_ble_disable_rollback(void* pHandle, void* pDevConfig)
{
    const unsigned char info_1_enc_patch_disable_ver_rollBack_bin[] =
    {
        0x80, 0x00, 0x30, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x94, 0xfd, 0x5c, 0x6c,
        0x3b, 0x83, 0x36, 0x18, 0x73, 0x08, 0xa0, 0x95, 0xf5, 0x0b, 0xb4, 0xcd,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x35, 0x4c, 0x47, 0x0f, 0x16, 0xa3, 0x4d, 0xb7, 0x71, 0xf7, 0x6c, 0x8d,
        0x92, 0x63, 0x20, 0xac, 0xf5, 0x3e, 0xfd, 0xd6, 0xea, 0x3f, 0xcc, 0x4d,
        0x8a, 0xf6, 0x4e, 0xd6, 0x7f, 0x01, 0x74, 0xce, 0x0d, 0x95, 0x9a, 0x35,
        0xcc, 0xf2, 0x41, 0x3c, 0x42, 0x46, 0x78, 0x2d, 0x61, 0xaa, 0x87, 0xcb,
        0x1e, 0x5e, 0x22, 0x54, 0x5a, 0xcd, 0xee, 0x49, 0xab, 0xf6, 0xb9, 0x9e,
        0x16, 0xb4, 0x09, 0x08, 0x93, 0xe6, 0x41, 0x05, 0x41, 0x7a, 0xd4, 0x1d,
        0xf1, 0x9c, 0x72, 0xb2, 0x57, 0xfc, 0x25, 0xb7
    };

    am_devices_cooper_sbl_update_data_t     sBLEInfo1Image =
    {
        (uint8_t*)& info_1_enc_patch_disable_ver_rollBack_bin,
        sizeof(info_1_enc_patch_disable_ver_rollBack_bin),
        AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_1,
        0
    };

    uint32_t ui32ErrorStatus;

    //
    // Update info1 image information to SBL
    //
    am_devices_cooper_get_info1_patch(&sBLEInfo1Image);

    //
    // For info 1 patching
    // write HCI command to trigger Cooper to reboot for SBL to do download.
    //
    ui32ErrorStatus = am_util_ble_update_sign_set(pHandle, COOPER_INFO1_UPDATE_SIGN);
    if ( ui32ErrorStatus != AM_DEVICES_COOPER_STATUS_SUCCESS )
    {
        am_util_stdio_printf("Write signature to BLE Controller failed\n");
    }
    else
    {
        //
        // reset Cooper to get SBL to update info1
        //
        am_util_stdio_printf("Reset Cooper for info1 updating\r\n");
        ui32ErrorStatus = am_devices_cooper_reset_with_sbl_check(pHandle, (am_devices_cooper_config_t*)pDevConfig);
    }

    return ui32ErrorStatus;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

