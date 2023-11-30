//*****************************************************************************
//
//! @file am_hal_dsi.c
//!
//! @brief Hardware abstraction for the Display Serial Interface
//!
//! @addtogroup dsi_4p DSI - Display Serial Interface
//! @ingroup apollo4p_hal
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
//
//! VDD18 control callback function
//
//*****************************************************************************
am_hal_dsi_external_vdd18_callback external_vdd18_callback;

//*****************************************************************************
//
// Register callback function
//
//*****************************************************************************
uint32_t
am_hal_dsi_register_external_vdd18_callback(const am_hal_dsi_external_vdd18_callback cb)
{
    if (cb != NULL)
    {
        external_vdd18_callback = cb;
    }
    else
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Configure DSI frequency and timing
//
//*****************************************************************************
uint32_t
am_hal_dsi_timing(uint32_t ui32FreqTrim)
{
    uint32_t ui32FreqTrimLsb6;
    ui32FreqTrimLsb6 = ui32FreqTrim & 0x3F;
    //
    // DPHY CLK/DATA timing parameters
    //
    if ((ui32FreqTrimLsb6 >= 0x0B) && (ui32FreqTrimLsb6 <= 0x0F))
    {
        DSI->DPHYPARAM_b.HSPREP = 0x04;
        DSI->DPHYPARAM_b.HSZERO = 0x08;
        DSI->DPHYPARAM_b.HSTRAIL = 0x05;
        DSI->DPHYPARAM_b.HSEXIT = 0x14;
        DSI->CLKLANETIMPARM_b.HSPREP = 0x04;
        DSI->CLKLANETIMPARM_b.HSZERO = 0x1C;
        DSI->CLKLANETIMPARM_b.HSTRAIL = 0x05;
        DSI->CLKLANETIMPARM_b.HSEXIT = 0x0F;
    }
    else if ((ui32FreqTrimLsb6 >= 0x08) && (ui32FreqTrimLsb6 <= 0x0A))
    {
        DSI->DPHYPARAM_b.HSPREP = 0x03;
        DSI->DPHYPARAM_b.HSZERO = 0x06;
        DSI->DPHYPARAM_b.HSTRAIL = 0x04;
        DSI->DPHYPARAM_b.HSEXIT = 0x0E;
        DSI->CLKLANETIMPARM_b.HSPREP = 0x03;
        DSI->CLKLANETIMPARM_b.HSZERO = 0x11;
        DSI->CLKLANETIMPARM_b.HSTRAIL = 0x04;
        DSI->CLKLANETIMPARM_b.HSEXIT = 0x0F;
    }
    else if ((ui32FreqTrimLsb6 >= 0x05) && (ui32FreqTrimLsb6 <= 0x07))
    {
        DSI->DPHYPARAM_b.HSPREP = 0x02;
        DSI->DPHYPARAM_b.HSZERO = 0x04;
        DSI->DPHYPARAM_b.HSTRAIL = 0x03;
        DSI->DPHYPARAM_b.HSEXIT = 0x0A;
        DSI->CLKLANETIMPARM_b.HSPREP = 0x02;
        DSI->CLKLANETIMPARM_b.HSZERO = 0x0B;
        DSI->CLKLANETIMPARM_b.HSTRAIL = 0x03;
        DSI->CLKLANETIMPARM_b.HSEXIT = 0x0F;
    }
    else
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }
    //
    // PLL settings
    //
    DSI->AFETRIM1 &= _VAL2FLD(DSI_AFETRIM1_AFETRIM1, ~0x0000007F);
    DSI->AFETRIM1 |= _VAL2FLD(DSI_AFETRIM1_AFETRIM1, ui32FreqTrim);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Initialize the DSI
//
//*****************************************************************************
uint32_t
am_hal_dsi_para_config(uint8_t ui8LanesNum, uint8_t ui8DBIBusWidth, uint32_t ui32FreqTrim)
{

    //
    // ui32DSIFuncPrg_REG (DATA_WIDTH, RESERVED, FMT_VIDEO, CH_NO_CM, CH_NO_VM, DATA_LANE_CNT)
    //                   [15:13]     [12:10]   [9:7]      [6:5]     [4:3]     [2:0]
    //
    uint32_t ui32DSIFuncPrg = 0;

    //
    // check number of lanes parameters
    //
    switch (ui8LanesNum)
    {
        case 1:
          ui32DSIFuncPrg |= _VAL2FLD(DSI_DSIFUNCPRG_DATALANES, 0x1);
          break;

        case 2:
          ui32DSIFuncPrg |= _VAL2FLD(DSI_DSIFUNCPRG_DATALANES, 0x2);
          break;

        default:
          return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    //
    // check DBI bus width parameter
    //
    switch (ui8DBIBusWidth)
    {
        case 8:
          ui32DSIFuncPrg |= _VAL2FLD(DSI_DSIFUNCPRG_REGNAME, 3);
          break;

        case 9:
          ui32DSIFuncPrg |= _VAL2FLD(DSI_DSIFUNCPRG_REGNAME, 2);
          break;

        case 16:
          ui32DSIFuncPrg |= _VAL2FLD(DSI_DSIFUNCPRG_REGNAME, 4); // opt1
          break;

        default:
          return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    DSI->RSTENBDFE = _VAL2FLD(DSI_RSTENBDFE_ENABLE, 0);

    //
    // Add 1ms delay to avoid low contention.
    //
    am_hal_delay_us(1000);

    //
    // write into DSI functional programming register
    //
    DSI->DSIFUNCPRG = ui32DSIFuncPrg;

    //
    // write into HIGH SPEED RECEIVE TIMEOUT REGISTER
    //
    DSI->HSTXTIMEOUT = _VAL2FLD(DSI_HSTXTIMEOUT_MAXDURTOCNT, 0x00FFFFFF);

    //
    //write into LOW POWER RECEIVE TIMEOUT REGISTER
    //
    DSI->LPRXTO = _VAL2FLD(DSI_LPRXTO_TOCHKRVS, 0xFF);

    //
    // write into TURN AROUND TIMEOUT REGISTER
    //
    DSI->TURNARNDTO = _VAL2FLD(DSI_TURNARNDTO_TIMOUT, 0x1F);

    //
    // write into DEVICE RESET TIMER REGISTER
    //
    DSI->DEVICERESETTIMER = _VAL2FLD(DSI_DEVICERESETTIMER_TIMOUT, 0xFF);

    //
    // write into HIGH TO LOW SWITCH COUNT REGISTER
    //
    DSI->DATALANEHILOSWCNT = _VAL2FLD(DSI_DATALANEHILOSWCNT_DATALHLSWCNT, 0xFFFF);
    DSI->INITCNT = _VAL2FLD(DSI_INITCNT_MSTR, 0x7d0);
    DSI->LPBYTECLK = _VAL2FLD(DSI_LPBYTECLK_VALBYTECLK, 0x3);
    DSI->CLKEOT = _VAL2FLD(DSI_CLKEOT_CLOCK, 1);
    DSI->CLKEOT |= _VAL2FLD(DSI_CLKEOT_EOT, 1);
    am_hal_dsi_timing(ui32FreqTrim);
    DSI->AFETRIM2 = _VAL2FLD(DSI_AFETRIM2_AFETRIM2, 0x10000000);
    DSI->AFETRIM2 |= _VAL2FLD(DSI_AFETRIM2_AFETRIM2, 0x00480000); // trim_2<22> and trim_2<19> need to be set for DSI TX in 1-lane configuration.
    DSI->AFETRIM1 |= _VAL2FLD(DSI_AFETRIM1_AFETRIM1, 0x00002000); // trim_1<13> needs to be set

    //
    // enable DSI TX and DPHY
    //
    DSI->AFETRIM3 |= _VAL2FLD(DSI_AFETRIM3_AFETRIM3, 0x00030000);
    DSI->RSTENBDFE = _VAL2FLD(DSI_RSTENBDFE_ENABLE, 1);
    DSI->DEVICEREADY |= _VAL2FLD(DSI_DEVICEREADY_READY, 1);

    //
    // Wait for DPHY init
    //
    am_hal_delay_us(400);

    //
    // ULPS Exit sequence
    //
    DSI->DEVICEREADY_b.ULPS = DSI_DEVICEREADY_ULPS_LOW_POWER;
    am_hal_delay_us(10);
    DSI->DEVICEREADY_b.ULPS = DSI_DEVICEREADY_ULPS_EXIT;
    DSI->AFETRIM3 &= _VAL2FLD(DSI_AFETRIM3_AFETRIM3, ~0x00030000);
    am_hal_delay_us(1010);
    DSI->DEVICEREADY_b.ULPS = DSI_DEVICEREADY_ULPS_This;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Initialize power and clock of DSI
//
//*****************************************************************************
uint32_t
am_hal_dsi_init(void)
{
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_DISPPHY);
    DSI->RSTENBDFE = _VAL2FLD(DSI_RSTENBDFE_ENABLE, 0);
    //DSI->DEVICEREADY = _VAL2FLD(DSI_DEVICEREADY_READY, 0);

    //
    // vdd18 enable
    //
    if (external_vdd18_callback)
    {
        external_vdd18_callback(true);
    }

    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DISPCLKSEL_DPHYPLL, NULL);
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DCCLK_ENABLE, NULL);
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_PLLCLKSEL_HFRC12, NULL);
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_PLLCLK_ENABLE, NULL);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Turn off power and clock of DSI
//
//*****************************************************************************
uint32_t
am_hal_dsi_deinit(void)
{
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DISPCLKSEL_OFF, NULL);
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DCCLK_DISABLE, NULL);
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_PLLCLKSEL_OFF, NULL);
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_PLLCLK_DISABLE, NULL);

    //
    // vdd18 disable
    //
    if (external_vdd18_callback)
    {
        external_vdd18_callback(false);
    }

    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_DISPPHY);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Enter ULPS mode
//
//*****************************************************************************
uint32_t
am_hal_dsi_ulps_entry(void)
{
    DSI->DEVICEREADY_b.ULPS = DSI_DEVICEREADY_ULPS_LOW_POWER;
    DSI->AFETRIM0 |= _VAL2FLD(DSI_AFETRIM0_AFETRIM0, 0x00000800); //  trim_0<11> needs to be set - contention detector disabled
    am_hal_delay_us(10);
    DSI->AFETRIM1 |= _VAL2FLD(DSI_AFETRIM1_AFETRIM1, 0x00000200); //  trim_1<9> needs to be set
    DSI->AFETRIM2 |= _VAL2FLD(DSI_AFETRIM2_AFETRIM2, 0x0000001C); //  trim_2<2>, trim_2<3> & trim_2<4> need to be set
    DSI->AFETRIM3 |= _VAL2FLD(DSI_AFETRIM3_AFETRIM3, 0x00038000); //  trim_3<15>, trim_3<16>, and trim_3<17> need to be set

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Exit ULPS mode
//
//*****************************************************************************
uint32_t
am_hal_dsi_ulps_exit(void)
{
    DSI->AFETRIM3 &= _VAL2FLD(DSI_AFETRIM3_AFETRIM3, ~0x00038000); //  trim_3<15>, trim_3<16>, and trim_3<17> need to be cleared
    DSI->AFETRIM2 &= _VAL2FLD(DSI_AFETRIM2_AFETRIM2, ~0x0000001C); //  trim_2<2>, trim_2<3> & trim_2<4> need to be cleared
    DSI->AFETRIM1 &= _VAL2FLD(DSI_AFETRIM1_AFETRIM1, ~0x00000200); //  trim_1<9> needs to be cleared
    DSI->DEVICEREADY_b.ULPS = DSI_DEVICEREADY_ULPS_EXIT;
    am_hal_delay_us(1010);
    DSI->DEVICEREADY_b.ULPS = DSI_DEVICEREADY_ULPS_This;
    DSI->AFETRIM0 &= _VAL2FLD(DSI_AFETRIM0_AFETRIM0, ~0x00000800); //  trim_0<11> needs to be cleared - contention detector enable

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// DSI napping
//
//*****************************************************************************
uint32_t
am_hal_dsi_napping(void)
{
    am_hal_dsi_ulps_entry();
    am_hal_dsi_deinit();

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// DSI wakeup
//
//*****************************************************************************
uint32_t
am_hal_dsi_wakeup(uint8_t ui8LanesNum, uint8_t ui8DBIBusWidth, uint32_t ui32FreqTrim)
{
    am_hal_dsi_init();

    if ( am_hal_dsi_para_config(ui8LanesNum, ui8DBIBusWidth, ui32FreqTrim) != 0 )
    {
        return AM_HAL_STATUS_FAIL;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// DSI set return packet size (bytes)
//
//*****************************************************************************
uint32_t
am_hal_dsi_set_return_size(uint8_t ui8DataLen, bool bHS)
{
    DSI->MAXRETPACSZE_b.COUNTVAL = ui8DataLen;
    DSI->MAXRETPACSZE_b.HSLP = (uint32_t) (!bHS);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
