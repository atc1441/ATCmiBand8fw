//*****************************************************************************
//
//! @file dtm_api.h
//!
//! @brief Global includes for the dtm_main app.
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

#ifndef DTM_API_H
#define DTM_API_H

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************

/*****************************************************************************/
/* ENABLE_DTM_MODE_FEATURE is used to enable or disable the DTM mode in general
 *  BLE example. 1: enable, 0: disable.
 * Add the judgement of this marco in your code when using the variable/function
 * of DTM mode, like:
 *
 * #if defined(ENABLE_DTM_MODE_FEATURE) && (ENABLE_DTM_MODE_FEATURE == 1)
 *
 * **codes**
 *
 * #endif
 */
/*****************************************************************************/
#define ENABLE_DTM_MODE_FEATURE       1

//*****************************************************************************
//
// External variable Declarations
//
//*****************************************************************************
extern bool g_bDtmModeRunning;

//*****************************************************************************
//
// Function Declarations
//
//*****************************************************************************

#ifndef BLE_BRIDGE_SINGLE_MODE
void ui_switch_to_dtm(void);
void ui_exit_from_dtm(void);
void reset_ble_and_enter_dtm(void);
#endif

void dtm_init(void);
void dtm_process(void);
void serial_interface_init(void);
void serial_interface_deinit(void);
void serial_data_read(uint8_t* pui8Data, uint32_t* ui32Length);
void serial_data_write(uint8_t* pui8Data, uint32_t ui32Length);
void serial_irq_enable(void);
void serial_irq_disable(void);
void serial_task(void);

#endif
