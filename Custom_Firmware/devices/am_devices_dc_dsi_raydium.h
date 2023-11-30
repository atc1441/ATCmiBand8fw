//*****************************************************************************
//
//! @file am_devices_dc_dsi_raydium.h
//!
//! @brief Generic raydium DSI driver.
//!
//! @addtogroup dc_dsi Raydium DC DSI Driver
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


#ifndef AM_DEVICES_DC_DSI_RAYDIUM_H
#define AM_DEVICES_DC_DSI_RAYDIUM_H

#include "stdint.h"
#include "nema_dc.h"
#include "nema_dc_hal.h"
#include "nema_dc_mipi.h"
#include "nema_dc_regs.h"
#include "nema_dc_intern.h"
#include "nema_dc_dsi.h"
#include "nema_sys_defs.h"
#include "am_devices_display_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! Global type definitions.
//
//*****************************************************************************
typedef enum
{
    FMT_RGB565 = 0,
    FMT_RGB888,
    FMT_NUM
} pixel_format_t;

typedef struct
{
    uint32_t ui32PixelFormat;
    uint16_t ui16ResX;
    uint16_t ui16ResY;
    uint16_t ui16MinX;
    uint16_t ui16MinY;
    bool bTEEnable;
    bool bFlip;
} am_devices_dc_dsi_raydium_config_t;

//*****************************************************************************
//
//! External function definitions.
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Reset the display panel
//!
//! This function resets the display panel by toggling reset pin.
//
//*****************************************************************************
extern void am_devices_dc_dsi_raydium_hardware_reset(void);

//*****************************************************************************
//
//! @brief Initialize raydium's IC RM67162, RM69090 or RM69330 with DSI interface.
//!
//! @param psDisplayPanelConfig     - Display panel configuration structure.
//!
//! @note This function configures display panel with the configurations in
//! sDisplayPanelConfig. It should be called after NemaDC and DSI are
//! initialized and configured.
//!
//! @return status.
//
//*****************************************************************************
extern uint32_t am_devices_dc_dsi_raydium_init(am_devices_dc_dsi_raydium_config_t *psDisplayPanelConfig);

//*****************************************************************************
//
//! @brief Set scanline
//!
//! @param ui16ScanLine - scanline index
//! @param ui16ResY     - Y resolution of display panel
//!
//! @note This function is used to set scanline index. It will return error when given
//! scanline is greater than or equal to display maximum resolution.
//!
//! @return status.
//
//*****************************************************************************
extern uint32_t am_devices_dc_dsi_set_scanline(uint16_t ui16ScanLine, uint16_t ui16ResY);

//*****************************************************************************
//
//! @brief Set scanline with recommended parameter
//!
//! @param TETimesPerFrame    - How many TE interrupts during transfering one frame
//! @param ui16ResY           - Y resolution of display panel
//!
//! @note This function is used to set scanline with recommended parameter, it's valid
//! when ui8TEIntTimesPerFrame equals to 1 or 2.
//!
//! @return status.
//
//*****************************************************************************
extern uint32_t am_devices_dc_dsi_set_scanline_with_recommended_parameter(uint8_t TETimesPerFrame,
                                                                          uint16_t ui16ResY);

//*****************************************************************************
//
//! @brief Set the region to be updated.
//!
//! @param ui16ResX    - X resolution of display region
//! @param ui16ResY    - Y resolution of display region
//! @param ui16MinX    - X-axis starting point of display region
//! @param ui16MinY    - Y-axis starting point of display region
//!
//! This function can be used to set display region of display driver IC.
//!
//! @return status.
//
//*****************************************************************************
extern uint32_t am_devices_dc_dsi_raydium_set_region(uint16_t ui16ResX,
                                                     uint16_t ui16ResY,
                                                     uint16_t ui16MinX,
                                                     uint16_t ui16MinY);


#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_DC_DSI_RAYDIUM_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
