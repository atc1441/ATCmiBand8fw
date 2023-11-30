//*****************************************************************************
//
//! @file am_util_ble.h
//!
//! @brief Useful BLE functions not covered by the HAL.
//!
//! This file contains functions for interacting with the BLE hardware
//! that are not already covered by the HAL. Most of these commands either
//! adjust RF settings or facilitate RF testing operations.
//!
//! @addtogroup ble BLE
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

#ifndef AM_UTIL_BLE_H
#define AM_UTIL_BLE_H

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
//! @brief In DTM mode, set TX to constant trans mode for SRRC/FCC/CE
//! set enable as 'true' to constant trans mode, 'false' back to normal
//!
//! @param pHandle - pointer to BLE Handle
//! @param enable - enable or disable
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_set_constant_transmission(void *pHandle, bool enable);

//*****************************************************************************
//
//! @brief Manually enable/disable transmitter
//! set ui8TxCtrl as 1 to manually enable transmitter, 0 back to default
//!
//! @param pHandle - pointer to BLE Handle
//! @param ui8TxCtrl - enable or disable
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_transmitter_control(void *pHandle, uint8_t ui8TxCtrl);

//*****************************************************************************
//
//! @brief to fix the channel 1 bug in DTM mode
//!
//! @param pHandle - pointer to BLE Handle
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_init_rf_channel(void *pHandle);

//*****************************************************************************
//
//! @brief BLE init for BQB test
//! set enable as 'true' to init as BQB test mode, 'false' back to default
//!
//! @param pHandle - pointer to BLE Handle
//! @param enable - enable or disable
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_BQB_test_init(void *pHandle, bool enable);

//*****************************************************************************
//
//! @brief Set the 32M crystal frequency based on the tested values at customer
//! side. Set trim value smaller in case of negative frequency offset
//! ui32TrimValue: default is 0x400
//!
//! @param pHandle - pointer to BLE Handle
//! @param ui32TrimValue - Trim Value for Crystal
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_crystal_trim_set(void *pHandle, uint32_t ui32TrimValue);

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
//! @brief to do directly output modulation signal. change channel ranges from 0 to 0x27,
//! pattern from 0 to 7.
//!
//! @param pHandle - pointer to BLE Handle
//! @param channel - channel for transmitter
//! @param pattern - pattern for output modulation
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_trasmitter_test_ex(void *pHandle,
                                               uint8_t channel,
                                               uint8_t pattern);

//*****************************************************************************
//
//! @brief to do directly receiver test. change channel ranges from 0 to 0x27, return
//! received packets in 100ms.
//!
//! @param pHandle - pointer to BLE Handle
//! @param channel - channel for transmitter
//! @param recvpackets - pointer to rx packet
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_receiver_test_ex(void *pHandle,
                                             uint8_t channel,
                                             uint32_t *recvpackets);

//*****************************************************************************
//
//! @brief to directly output carrier wave. change channel ranges from 0 to 0x27.
//!
//! @param pHandle - pointer to BLE Handle
//! @param channel - channel for transmitter
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_set_carrier_wave_ex(void *pHandle, uint8_t channel);

//*****************************************************************************
//
//! @brief Manually enable/disable transmitter to output carrier wave signal
//! set ui8TxChannel as 0 to 0x27 for each transmit channel, 0xFF back to normal
//! modulate mode
//!
//! @param pHandle - pointer to BLE Handle
//! @param ui8TxChannel - channel for transmitter
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_transmitter_control_ex(void *pHandle,
                                                   uint8_t ui8TxChannel);

//*****************************************************************************
//
//! @brief to directly output constant modulation signal. change channel from 0 to 0x27.
//!
//! @param pHandle - pointer to BLE Handle
//! @param channel - channel for transmitter
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_set_constant_transmission_ex(void *pHandle,
                                                         uint8_t channel);

//*****************************************************************************
//
//! @brief read current modex value from BLEIP
//!
//! @param pHandle - pointer to BLE Handle
//!
//! @return Status Code
//
//*****************************************************************************
extern uint32_t am_util_ble_read_modex_value(void *pHandle);

#ifdef __cplusplus
}
#endif

#endif // AM_UTIL_BLE_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

