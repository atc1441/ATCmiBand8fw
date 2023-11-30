//*****************************************************************************
//
//! @file am_util_ble_cooper.h
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

#ifndef AM_UTIL_BLE_COOPER_H
#define AM_UTIL_BLE_COOPER_H

//*****************************************************************************
//
// External function declarations.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @brief Read a register value from the BLE core.
//!
//! @param pHandle - pointer to BLE Handle
//! @param ui32Address - address to read
//! @param pui32Value - pointer to read value
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_plf_reg_read(void* pHandle,
                                         uint32_t ui32Address,
                                         uint32_t* pui32Value);

//*****************************************************************************
//
//! @brief Write a register value to the BLE core.
//!
//! @param pHandle - pointer to BLE Handle
//! @param ui32Address - address to read
//! @param ui32Value - write value
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_plf_reg_write(void* pHandle,
                                          uint32_t ui32Address,
                                          uint32_t ui32Value);

//*****************************************************************************
//
//! @brief Manually enable/disable transmitter to output carrier signal
//! set ui8TxChannel as 0 to 0x27 for each transmit channel, 0xFF back to normal
//! modulate mode
//!
//! @param pHandle - pointer to BLE Handle
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_hci_reset(void *pHandle);

//*****************************************************************************
//
//! @brief Set BLE sleep enable/disable for the BLE core.
//! enable = 'true' set sleep enable, enable = 'false' set sleep disable
//!
//! @param pHandle - pointer to BLE Handle
//! @param enable - enable/disable
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_sleep_set(void* pHandle, bool enable);

//*****************************************************************************
//
//! @brief set the tx power of BLE
//!
//! @param pHandle - pointer to BLE Handle
//! @param ui32TxPower - enum txPowerLevel_t defined in hci_drv_cooper.h
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_tx_power_set(void* pHandle, uint8_t ui32TxPower);

//*****************************************************************************
//
//! @brief Write NVDS parameters to the BLE core.
//!
//! @param pHandle - pointer to BLE Handle
//! @param pui8NVDS - pointer to NVDS parameters
//! @param ui8Length - length of parameters
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_nvds_set(void *pHandle,
                                     uint8_t* pui8NVDS,
                                     uint8_t ui8Length);

#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
//*****************************************************************************
//
//! @brief Write update signature to the BLE core.
//!
//! @param pHandle - pointer to BLE Handle
//! @param ui32Sign - signature to update
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_update_sign_set(void *pHandle, uint32_t ui32Sign);
#endif

//*****************************************************************************
//
//! @brief to do directly output modulation signal.
//! channel ranges from 0 to 0x27, pattern from 0 to 7.
//!
//! @param pHandle - pointer to BLE Handle
//! @param channel - channel number
//! @param pattern - pattern for TX test
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_trasmitter_test_ex(void *pHandle,
                                               uint8_t channel,
                                               uint8_t pattern);

//*****************************************************************************
//
//! @brief Receiver test.
//! change channel ranges from 0 to 0x27, return received packets in 100ms.
//!
//! @param pHandle - pointer to BLE Handle
//! @param channel - channel number
//! @param recvpackets - pointer for RX Packets
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_receiver_test_ex(void *pHandle,
                                             uint8_t channel,
                                             uint32_t *recvpackets);

//*****************************************************************************
//
//! @brief Dump info0 of BLE controller for debug use.
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_info0_dump(void *pHandle);

//*****************************************************************************
//
//! @brief get cooper TRIM version.
//!
//! @param pHandle - pointer to BLE Handle
//! @param pui32TrimVer - pointer to Trim Version
//! @param pui32RetvTrimVer - pointer to Return Value for Trim Version
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_trim_version_get(void* pHandle,
                                             uint32_t *pui32TrimVer,
                                             uint32_t *pui32RetvTrimVer);

//*****************************************************************************
//
//! @brief API to disable the BLE controller's firmware rollback version
//! (Enabled in default)
//! Should be called as the very last step during manufacturing, after it done,
//! the BLE controller will reset.
//!
//! @param pHandle - pointer to BLE Handle
//! @param pDevConfig - pointer to Device Configuration
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_disable_rollback(void* pHandle, void* pDevConfig);

#ifdef __cplusplus
}
#endif

#endif // AM_UTIL_BLE_COOPER_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

