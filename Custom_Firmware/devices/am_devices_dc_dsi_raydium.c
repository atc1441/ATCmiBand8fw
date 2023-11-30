//*****************************************************************************
//
//! @file am_devices_dc_dsi_raydium.c
//!
//! @brief Generic Raydium display driver with DSI interfaces. This
//! driver supports the display panels with driver IC rm67162, rm69330 or
//! rm69090, etc.
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

#include "am_devices_dc_dsi_raydium.h"
#include "am_util_delay.h"
#include "am_bsp.h"
#include "am_util.h"

#ifndef SIMULATION
#define DELAY am_util_delay_ms
#else
#define DELAY(...)
#endif

//*****************************************************************************
//
// Reset the display panel
//
//*****************************************************************************
void
am_devices_dc_dsi_raydium_hardware_reset(void)
{
    am_bsp_disp_reset_pins_set();
    DELAY(5);
    am_bsp_disp_reset_pins_clear();
    DELAY(20);
    am_bsp_disp_reset_pins_set();
    DELAY(150);
}

//*****************************************************************************
//
// Initialize raydium's IC RM67162, RM69090 or RM69330 with DSI interface.
//
//*****************************************************************************
uint32_t
am_devices_dc_dsi_raydium_init(am_devices_dc_dsi_raydium_config_t *psDisplayPanelConfig)
{
    uint8_t ui8CmdBuf[4];
    const int MIPI_set_cmd_page = 0xFE;

    //
    // Sending brightness command.
    //
    const int MIPI_set_display_brightness = 0x51;
    ui8CmdBuf[0] = MIPI_set_display_brightness;
    ui8CmdBuf[1] = 0xff;
    nemadc_mipi_cmd_write(0, ui8CmdBuf, 2, false, false);
    DELAY(10);

    //
    // Set MIPI Panel Pixel Format
    //
    ui8CmdBuf[0] = (uint8_t)(psDisplayPanelConfig->ui32PixelFormat & 0x3f);
    nemadc_mipi_cmd_write(MIPI_set_pixel_format, ui8CmdBuf, 1, true, false);
    DELAY(10);

    //
    // When using RM69330 DSI, set N565 reg to 1 to solve color issue caused by endianess.
    //
    const int MIPI_set_hsifopctr = 0x0A;
    ui8CmdBuf[0] = MIPI_set_cmd_page;
    ui8CmdBuf[1] = 0x01; // MCS
    nemadc_mipi_cmd_write(0, ui8CmdBuf, 2, false, false);
    DELAY(10);
    ui8CmdBuf[0] = MIPI_set_hsifopctr;
    ui8CmdBuf[1] = 0xF8; // set N565 to 1
    nemadc_mipi_cmd_write(0, ui8CmdBuf, 2, false, false);
    DELAY(10);
    ui8CmdBuf[0] = MIPI_set_cmd_page;
    ui8CmdBuf[1] = 0x00; // UCS
    nemadc_mipi_cmd_write(0, ui8CmdBuf, 2, false, false);
    DELAY(10);
    //
    // Need to flip display when drive IC is RM67162
    //
    if ( psDisplayPanelConfig->bFlip )
    {
        ui8CmdBuf[0] = 0x02;
    }
    else
    {
        ui8CmdBuf[0] = 0x00;
    }

    nemadc_mipi_cmd_write(MIPI_set_address_mode, ui8CmdBuf, 1, true, false);
    DELAY(10);

    //
    // Enable MIPI Panel
    //
    nemadc_mipi_cmd_write(MIPI_exit_sleep_mode, NULL, 0, true, false);
    DELAY(130);

    nemadc_mipi_cmd_write(MIPI_set_display_on, NULL, 0, true, false);
    DELAY(200);

    //
    // Set display panel region to be updated
    //
    am_devices_dc_dsi_raydium_set_region(psDisplayPanelConfig->ui16ResX,
                                         psDisplayPanelConfig->ui16ResY,
                                         psDisplayPanelConfig->ui16MinX,
                                         psDisplayPanelConfig->ui16MinY);

    DELAY(200);

    //
    // Enable/disable tearing
    //
    if ( psDisplayPanelConfig->bTEEnable )
    {
        ui8CmdBuf[0] = 0x02; // TE output active at refresh frame
        nemadc_mipi_cmd_write(MIPI_set_tear_on, ui8CmdBuf, 1, true, false);
    }
    else
    {
        nemadc_mipi_cmd_write(MIPI_set_tear_off, NULL, 0, true, false);
    }
    DELAY(10);

    return AM_DEVICES_DISPLAY_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Set scanline
//
//*****************************************************************************
uint32_t
am_devices_dc_dsi_set_scanline(uint16_t ui16ScanLine, uint16_t ui16ResY)
{
    uint8_t ui8CmdBuf[2];

    if ( ui16ScanLine > ui16ResY )
    {
        return AM_DEVICES_DISPLAY_STATUS_OUT_OF_RANGE;
    }

    ui8CmdBuf[0] = ui16ScanLine >> 8;
    ui8CmdBuf[1] = ui16ScanLine & 0x00FF;
    nemadc_mipi_cmd_write(MIPI_set_tear_scanline, ui8CmdBuf, 2, true, false);

    return AM_DEVICES_DISPLAY_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Set scanline with recommended parameter
//
//*****************************************************************************
uint32_t
am_devices_dc_dsi_set_scanline_with_recommended_parameter(uint8_t TETimesPerFrame,
                                                          uint16_t ui16ResY)
{
    uint16_t ui16ScanLine = 0;
    //
    // setting scanline.
    //
    if ( TETimesPerFrame == 2 )
    {
        //
        // setting scanline to start line plus 10 lines when frame transfer time is longer than TE signals interval.
        //
        ui16ScanLine = 10;
    }
    else if ( TETimesPerFrame == 1 )
    {
        //
        // setting scanline to start line minus 10 lines when frame transfer time is shorter than TE signals interval.
        //
        ui16ScanLine = ui16ResY - 10;
    }
    else
    {
        return AM_DEVICES_DISPLAY_STATUS_INVALID_ARG;
    }
    return am_devices_dc_dsi_set_scanline(ui16ScanLine, ui16ResY);
}

//*****************************************************************************
//
// Set the region to be updated.
//
//*****************************************************************************
uint32_t
am_devices_dc_dsi_raydium_set_region(uint16_t ui16ResX,
                                     uint16_t ui16ResY,
                                     uint16_t ui16MinX,
                                     uint16_t ui16MinY)
{
    uint8_t ui8CmdBuf[4];
    uint16_t ui16MaxX, ui16MaxY;

    ui16MaxX = ui16MinX + ui16ResX - 1;
    ui16MaxY = ui16MinY + ui16ResY - 1;

    //
    // Set MIPI Panel region to be updated
    //
    ui8CmdBuf[0] = (uint8_t)(ui16MinX >> 8U);
    ui8CmdBuf[1] = (uint8_t)(ui16MinX & 0xFFU);
    ui8CmdBuf[2] = (uint8_t)(ui16MaxX >> 8U);
    ui8CmdBuf[3] = (uint8_t)(ui16MaxX & 0xFFU);
    nemadc_mipi_cmd_write(MIPI_set_column_address, ui8CmdBuf, 4, true, false);

    ui8CmdBuf[0] = (uint8_t)(ui16MinY >> 8U);
    ui8CmdBuf[1] = (uint8_t)(ui16MinY & 0xFFU);
    ui8CmdBuf[2] = (uint8_t)(ui16MaxY >> 8U);
    ui8CmdBuf[3] = (uint8_t)(ui16MaxY & 0xFFU);
    nemadc_mipi_cmd_write(MIPI_set_page_address, ui8CmdBuf, 4, true, false);

    return AM_DEVICES_DISPLAY_STATUS_SUCCESS;
}


//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
