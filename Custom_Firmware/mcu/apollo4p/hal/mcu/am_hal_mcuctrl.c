//*****************************************************************************
//
//! @file am_hal_mcuctrl.c
//!
//! @brief Functions for interfacing with the MCUCTRL.
//!
//! @addtogroup mcuctrl4_4p MCUCTRL - MCU Control
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

#include "am_hal_mcuctrl.h"

//*****************************************************************************
//
// Global Variables.
//
//*****************************************************************************
//
//! Lookup table for memory sizes as derived from the SKU register.
//!  0: MRAM    size in KB
//!  1: TCM     size in KB
//!  2: SSRAM   size in KB
//!  3: Ext RAM size in KB
//
static const uint16_t
g_am_hal_mcuctrl_sku_ssram_size[AM_HAL_MCUCTRL_SKU_SSRAM_SIZE_N][2] =
{
    {1024, 384},    //! 0x0:  1024KB SRAM + 384KB Ext
    {2048, 384},    //! 0x1:  2948KB SRAM + 384KB Ext
    {1024, 384},    //! 0x2:  1024KB SRAM + 384KB Ext
    {2048, 384}     //! 0x3:  2948KB SRAM + 384KB Ext
};

//
//! Lookup table for MRAM sizes as derived from the SKU register.
//
static const uint16_t
g_am_hal_mcuctrl_sku_mram_size[AM_HAL_MCUCTRL_SKU_MRAM_SIZE_N] =
{
     512,
     1024,
     1536,
     2048
};

// ****************************************************************************
// MCUCTRL XTALHSCAP Globals for Cooper Device
// Refer to App Note Apollo4 Blue 32MHz Crystal Calibration
// ****************************************************************************
uint32_t g_ui32xtalhscap2trim = XTALHSCAP2TRIM_DEFAULT;
uint32_t g_ui32xtalhscaptrim = XTALHSCAPTRIM_DEFAULT;

//
//! These data keeps track of number of users of the HF XTAL CLOCK
//! and the HFXTAL padout (HFXTAL clockout)
//! when these values are non-zero the respective settings
//! (HFXTAL clock, and HFXTAL padout, cannot be disabled)
//! the bits in these are set for each user module according to enum
//! am_hal_mcuctrl_hfxtal_users_e
//
typedef struct
{
    uint32_t  g_ui32HfXtalEnFlags;
    uint32_t  g_ui32HfXtalClockOutlags;
    bool      b_HfXtalKickStartEnabled;
}
am_hal_xtal_users_t;

am_hal_xtal_users_t am_hal_xtal_users  =
{
    .g_ui32HfXtalEnFlags = 0,
    .b_HfXtalKickStartEnabled = 0
};

//
//! @brief default stuct used track modifications of XTAL clock behavior
//! default structure passed as arg (pointer) into am_hal_mcuctrl_control
//! is's assumed the user will change the value of AM_HAL_HCXTAL_DEFAULT_EN
//! to module name the user needs the XTAL clock
//
const am_hal_mcuctrl_control_arg_t g_amHalMcuctrlArgDefault =
{
    .ui32_arg_hfxtal_user_mask = 1 << AM_HAL_HCXTAL_DEFAULT_EN,
    .b_arg_hfxtal_in_use       = true,
    .b_arg_apply_ext_source    = false,
    .b_arg_force_update        = false,
    .b_arg_enable_HfXtalClockout = false,
};
//
//! default stuct used track modifications of XTAL for BLE
//! this was needed in so many places a separate entry has been created
//! for this. Additionally BLE will need the 32Mhz (HFXTAL) clockout enabled
//
const am_hal_mcuctrl_control_arg_t g_amHalMcuctrlArgBLEDefault =
{
    .ui32_arg_hfxtal_user_mask = 1 << AM_HAL_HFXTAL_BLE_CONTROLLER_EN,
    .b_arg_hfxtal_in_use       = true,
    .b_arg_apply_ext_source    = false,
    .b_arg_force_update        = false,
    .b_arg_enable_HfXtalClockout = true,
};


static uint32_t
mcuctrl_HFXTAL_clockOutPad_mask_modify(bool bClockOutEnable, uint32_t ui32ClockEnableMask );
static uint32_t
mcuctrl_ctrl_HFXTAL_normal(const am_hal_mcuctrl_control_arg_t *peCtrlArg );
static uint32_t
mcuctrl_ctrl_HFXTAL_kickstart(const am_hal_mcuctrl_control_arg_t *peCtrlArg );
static uint32_t
mcuctrl_ctrl_HFXTAL_disable(const am_hal_mcuctrl_control_arg_t *peCtrlArg );
static uint32_t
mcuctrl_HFXTAL_set_mask(bool bSet, uint32_t ui32Mask );

// ****************************************************************************
//
//  device_info_get()
//  Gets all relevant device information.
//
// ****************************************************************************
static void
device_info_get(am_hal_mcuctrl_device_t *psDevice)
{
    //
    // Read the Part Number.
    //
    psDevice->ui32ChipPN = MCUCTRL->CHIPPN;

    //
    // Read the Chip ID0.
    //
    psDevice->ui32ChipID0 = MCUCTRL->CHIPID0;

    //
    // Read the Chip ID1.
    //
    psDevice->ui32ChipID1 = MCUCTRL->CHIPID1;

    //
    // Read the Chip Revision.
    //
    psDevice->ui32ChipRev = MCUCTRL->CHIPREV;

    //
    // Read the Chip VENDOR ID.
    //
    psDevice->ui32VendorID = MCUCTRL->VENDORID;

    //
    // Read the SKU.
    //
    psDevice->ui32SKU = MCUCTRL->SKU;

    //
    // Qualified from Part Number.
    //
    psDevice->ui32Qualified = 1;

    //
    // MRAM size as derived from the SKU register.
    //
    psDevice->ui32MRAMSize = g_am_hal_mcuctrl_sku_mram_size[MCUCTRL->SKU_b.SKUMRAMSIZE] * 1024;

    //
    // TCM size.
    //
    psDevice->ui32DTCMSize = TCM_MAX_SIZE;

    //
    // Shared SRAM size as derived from the SKU register.
    //
    psDevice->ui32SSRAMSize = (g_am_hal_mcuctrl_sku_ssram_size[MCUCTRL->SKU_b.SKUSRAMSIZE][0] +
                               g_am_hal_mcuctrl_sku_ssram_size[MCUCTRL->SKU_b.SKUSRAMSIZE][1]) * 1024;

    //
    // Now, let's look at the JEDEC info.
    // The full partnumber is 12 bits total, but is scattered across 2 registers.
    // Bits [11:8] are 0xE.
    // Bits [7:4] are 0xE for Apollo, 0xD for Apollo2.
    // Bits [3:0] are defined differently for Apollo and Apollo2.
    //   For Apollo, the low nibble is 0x0.
    //   For Apollo2, the low nibble indicates flash and SRAM size.
    //
    psDevice->ui32JedecPN  = JEDEC->PID0_b.PNL8 << 0;
    psDevice->ui32JedecPN |= JEDEC->PID1_b.PNH4 << 8;

    //
    // JEPID is the JEP-106 Manufacturer ID Code, which is assigned to Ambiq as
    //  0x1B, with parity bit is 0x9B.  It is 8 bits located across 2 registers.
    //
    psDevice->ui32JedecJEPID  = JEDEC->PID1_b.JEPIDL << 0;
    psDevice->ui32JedecJEPID |= JEDEC->PID2_b.JEPIDH << 4;

    //
    // CHIPREV is 8 bits located across 2 registers.
    //
    psDevice->ui32JedecCHIPREV  = JEDEC->PID2_b.CHIPREVH4 << 4;
    psDevice->ui32JedecCHIPREV |= JEDEC->PID3_b.CHIPREVL4 << 0;

    //
    // Let's get the Coresight ID (32-bits across 4 registers)
    // For Apollo and Apollo2, it's expected to be 0xB105100D.
    //
    psDevice->ui32JedecCID  = JEDEC->CID3_b.CID << 24;
    psDevice->ui32JedecCID |= JEDEC->CID2_b.CID << 16;
    psDevice->ui32JedecCID |= JEDEC->CID1_b.CID <<  8;
    psDevice->ui32JedecCID |= JEDEC->CID0_b.CID <<  0;

} // device_info_get()

// ****************************************************************************
//
//  am_hal_mcuctrl_EXTCLK_active()
//  return true if any peripheral is using the HSXTAL clock.
//
// ****************************************************************************
/******************************************************************************
//   get usage status of HF clock
//
// return true if one or more users (modules) are using the HF Xtal clock
// return false  if the xtal clock is disabled and there are no users
 *****************************************************************************/
bool
am_hal_mcuctrl_EXTCLK_active(void )
{
    return am_hal_xtal_users.g_ui32HfXtalEnFlags != 0;
}

//******************************************************************************
//
//! @brief set or clear bits the xtal users variable
//!
//! @param bSet  when true will set bits, when false will clear them
//! @param ui32Mask  contains bits to set or clear
//!
//! @return always success
//
// *****************************************************************************
static uint32_t
mcuctrl_HFXTAL_set_mask(bool bSet, uint32_t ui32Mask )
{
    if ( bSet )
    {
        am_hal_xtal_users.g_ui32HfXtalEnFlags |= ui32Mask;
    }
    else
    {
        am_hal_xtal_users.g_ui32HfXtalEnFlags &= ~ui32Mask;
    }
    return AM_HAL_STATUS_SUCCESS;
}

//****************************************************************************
//
//! @brief enables HF Xtal
//!
//! @param peCtrlArg  pointer to struct that controls HF Xtal setups
//!
//! @return standard hal status
//
//**************************************************************************
static uint32_t
mcuctrl_ctrl_HFXTAL_normal(const am_hal_mcuctrl_control_arg_t *peCtrlArg )
{
    if (peCtrlArg == 0)
    {
        //
        // this is the legacy call and is no longer supported.
        //
        return AM_HAL_STATUS_INVALID_ARG;
    }


    //
    // Begin critical section.
    //
    AM_CRITICAL_BEGIN

    //
    // remember if the hs xtal clock is enabled on entry
    //
    bool bHSXTAL_entry_enabled = am_hal_mcuctrl_EXTCLK_active();

    if (peCtrlArg->b_arg_hfxtal_in_use)
    {
        //
        // set bit(s) in RAM variable that manage HFXTAL clock enablement
        //
        mcuctrl_HFXTAL_set_mask(true, peCtrlArg->ui32_arg_hfxtal_user_mask);

        //
        // clear or set clock pad-out bit for this peripheral
        //
        mcuctrl_HFXTAL_clockOutPad_mask_modify(peCtrlArg->b_arg_enable_HfXtalClockout,
                                               peCtrlArg->ui32_arg_hfxtal_user_mask);
    }

    bool bHSXTAL_enabled  = am_hal_mcuctrl_EXTCLK_active();

    //
    // is the HFXTAL transitioning to enabled with this call
    //
    bool bEnableHFXTAL    = !bHSXTAL_entry_enabled && bHSXTAL_enabled;

    //
    // if the HFXTAL needs to be enabled, and if it isn't already enabled,
    // the code below will enable the HFXTAL
    //
    if (bEnableHFXTAL || peCtrlArg->b_arg_force_update)
    {

        uint32_t ui32TrimReg = _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSCAP2TRIM, g_ui32xtalhscap2trim) |
                  _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSCAPTRIM, g_ui32xtalhscaptrim) |
                  _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSDRIVETRIM, 3) |
                  _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSDRIVERSTRENGTH, 0) |
                  _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMP2TRIM, 3) |
                  _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMPTRIM, 15) |
                  _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASTRIM, 127) |
                  _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSRSTRIM, 0) |
                  _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSSPARE, 0);
        MCUCTRL->XTALHSTRIMS = ui32TrimReg;  // ok

        uint32_t ui32Reg = MCUCTRL->XTALHSCTRL;
        ui32Reg &= ~(MCUCTRL_XTALHSCTRL_XTALHSCOMPPDNB_Msk |
                     MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE_Msk);
        ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSCOMPPDNB, 1) |
                   _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE, 1);
        MCUCTRL->XTALHSCTRL = ui32Reg;

        //
        // Turn on xtalhs_pdnb
        //
        ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNB, 1);
        MCUCTRL->XTALHSCTRL = ui32Reg;

        //
        // Apply external source
        //
        if (peCtrlArg->b_arg_apply_ext_source)
        {
            ui32Reg &= ~(MCUCTRL_XTALHSCTRL_XTALHSPDNB_Msk |
                         MCUCTRL_XTALHSCTRL_XTALHSEXTERNALCLOCK_Msk);
            ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNB, 0) |
                       _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSEXTERNALCLOCK, 1);
        }
        else
        {
            ui32Reg &= ~(MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE_Msk |
                         MCUCTRL_XTALHSCTRL_XTALHSIBSTENABLE_Msk);
            ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE, 0) |
                       _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSIBSTENABLE, 1);
        }
        MCUCTRL->XTALHSCTRL = ui32Reg;

    } // not enabled

    //
    // End critical section.
    //
    AM_CRITICAL_END

    return AM_HAL_STATUS_SUCCESS;

}

//******************************************************************************
//! @brief enables HF Xtal with kickstart
//!
//! @param peCtrlArg  pointer to struct that controls HF Xtal setups
//!
//! @return standard hal status
//
//*****************************************************************************
static uint32_t
mcuctrl_ctrl_HFXTAL_kickstart(const am_hal_mcuctrl_control_arg_t *peCtrlArg )
{

    if (peCtrlArg == 0)
    {
        //
        // this is the legacy call and is no longer supported.
        //
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Begin critical section.
    //
    AM_CRITICAL_BEGIN

    //
    // remember if the hs xtal clock is enabled on entry
    //
    bool bHSXTAL_entry_enabled = am_hal_mcuctrl_EXTCLK_active();

    if (peCtrlArg->b_arg_hfxtal_in_use)
    {

        //
        // set bit(s) in RAM variable that monitor HFXTAL clock enablement
        //
        mcuctrl_HFXTAL_set_mask(true, peCtrlArg->ui32_arg_hfxtal_user_mask);

        //
        // clear or set clock pad-out bit for this peripheral
        //
        mcuctrl_HFXTAL_clockOutPad_mask_modify(peCtrlArg->b_arg_enable_HfXtalClockout,
                                               peCtrlArg->ui32_arg_hfxtal_user_mask);
    }

    bool bHSXTAL_enabled  = am_hal_mcuctrl_EXTCLK_active();

    //
    // is the HFXTAL transitioning to enabled with this call
    //
    bool bEnableHFXTAL    = !bHSXTAL_entry_enabled && bHSXTAL_enabled;

    //
    // if the HFXTAL needs to be enabled, and if it isn't already enabled,
    // the code below will enable the HFXTAL
    //
    if ( bEnableHFXTAL || peCtrlArg->b_arg_force_update )
    {
        am_hal_xtal_users.b_HfXtalKickStartEnabled = true;

        // Set the default trim code for CAP1/CAP2, it impacts frequency accuracy and should be retrimmed
        uint32_t ui32TrimReg =
            _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSCAP2TRIM, g_ui32xtalhscap2trim) |
            _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSCAPTRIM, g_ui32xtalhscaptrim) |
            // Set the transconductance of crystal to maximum, it accelerates the startup sequence
            _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSDRIVETRIM, 3) |
            // Choose the power of clock driver to be the cleanest one
            _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSDRIVERSTRENGTH, 0) |
            // Tune the bias generator
            _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMP2TRIM, 3) |
            _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMPTRIM, 15) |
            // Set the bias of crystal to maximum
            _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASTRIM, 127) |
            _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSRSTRIM, 0) |
            _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSSPARE, 0);

        MCUCTRL->XTALHSTRIMS = ui32TrimReg;

        uint32_t ui32Reg = MCUCTRL->XTALHSCTRL;
        ui32Reg &= ~(MCUCTRL_XTALHSCTRL_XTALHSCOMPPDNB_Msk |
                     MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE_Msk);
        ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSCOMPPDNB, 1) |
                   _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE, 0);
        MCUCTRL->XTALHSCTRL = ui32Reg;

        //
        // turn on crystal oscillator
        //
        ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNB, 1);
        MCUCTRL->XTALHSCTRL = ui32Reg;

        //
        // inject HFRC clock to accelerate the startup sequence
        //
        ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSINJECTIONENABLE, 1);
        MCUCTRL->XTALHSCTRL = ui32Reg;

        //
        // Turn on xtalhs_ibst_enable
        //  Maximize the bias current to accelerate the startup sequence
        //
        ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSIBSTENABLE, 1);
        MCUCTRL->XTALHSCTRL = ui32Reg;

        //
        // Wait 5us to make the setting above effective
        // Turn off xtalhs_injection_enable (clock injection)
        //
        am_hal_delay_us(5);


        ui32Reg = MCUCTRL->XTALHSCTRL ;
        ui32Reg &= ~MCUCTRL_XTALHSCTRL_XTALHSINJECTIONENABLE_Msk;

        if (peCtrlArg->b_arg_apply_ext_source)
        {
            ui32Reg &= ~(MCUCTRL_XTALHSCTRL_XTALHSPDNB_Msk |
                         MCUCTRL_XTALHSCTRL_XTALHSIBSTENABLE_Msk |
                         MCUCTRL_XTALHSCTRL_XTALHSEXTERNALCLOCK_Msk);
            ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNB, 0) |
                       _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSIBSTENABLE, 0) |
                       _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSEXTERNALCLOCK, 1);
        }

        MCUCTRL->XTALHSCTRL = ui32Reg;


    } // !bEnableHFXTAL || peCtrlArg->b_arg_force_update

    //
    // End critical section.
    //
    AM_CRITICAL_END

    return AM_HAL_STATUS_SUCCESS;
}

/******************************************************************************
//! @brief sets or clears the clockoutput pin
//!
//! @note if using preemptive os, this should be called from a critical section
//!
//! @details clockout is disabled once no no user is requesting it
//!
//! @param bClockOutEnable  when true clock out is enabled, false it is disabled
//! @param ui32ClockEnableMask indicates which user is making this request
//!
//! @return always success
 *****************************************************************************/
static uint32_t
mcuctrl_HFXTAL_clockOutPad_mask_modify(bool bClockOutEnable, uint32_t ui32ClockEnableMask )
{
    if (bClockOutEnable)
    {
        //
        // Enable the option to output clock on PAD GPIO46
        // connecting to Cooper crystal input
        //
        am_hal_xtal_users.g_ui32HfXtalClockOutlags |= ui32ClockEnableMask;
    }
    else
    {
        //
        // Disable clockout on PAD GPIO46
        //
        am_hal_xtal_users.g_ui32HfXtalClockOutlags &= ~ui32ClockEnableMask;
    }

    //
    // below the actual register bit is set or cleared
    //
    if ( am_hal_xtal_users.g_ui32HfXtalClockOutlags )
    {
        MCUCTRL->XTALHSCTRL_b.XTALHSPADOUTEN = 1;
    }
    else
    {
        MCUCTRL->XTALHSCTRL_b.XTALHSPADOUTEN = 0;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//****************************************************************************
//
//! @brief disable the HF Xtal clock when it isn't being used
//!
//! @param peCtrlArg  pointer to struct that controls HF Xtal setups
//!
//! @return always success
//
//*****************************************************************************
static uint32_t
mcuctrl_ctrl_HFXTAL_disable(const am_hal_mcuctrl_control_arg_t *peCtrlArg )
{
    if (peCtrlArg == 0)
    {
        //
        // this is the legacy call and is no longer supported.
        //
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Begin critical section.
    //
    AM_CRITICAL_BEGIN

    if (peCtrlArg->b_arg_hfxtal_in_use)
    {

        //
        // clear bit for this peripheral
        //
        mcuctrl_HFXTAL_set_mask(false, peCtrlArg->ui32_arg_hfxtal_user_mask);

        //
        // clear or set clock pad-out bit for this peripheral
        //
        mcuctrl_HFXTAL_clockOutPad_mask_modify(peCtrlArg->b_arg_enable_HfXtalClockout,
                                               peCtrlArg->ui32_arg_hfxtal_user_mask);
    }

    //
    // don't disable clock until all peripherals have released it
    // or the action is forced
    //
    bool HSXTAL_enabled = am_hal_mcuctrl_EXTCLK_active();

    if (!HSXTAL_enabled || peCtrlArg->b_arg_force_update)
    {
        am_hal_xtal_users.b_HfXtalKickStartEnabled = false;

        //
        // clear all bits here, in case b_arg_force_update was used
        //
        mcuctrl_HFXTAL_set_mask(false, 0xFFFFFFFF);

        uint32_t ui32TrimReg  =
            _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSCAP2TRIM, g_ui32xtalhscap2trim)    |
            _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSCAPTRIM, g_ui32xtalhscaptrim)      |
            _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSDRIVETRIM, 0)                      |
            _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSDRIVERSTRENGTH, 7)                 |
            _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMP2TRIM, 3)                 |
            _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMPTRIM, 8)                  |
            _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASTRIM, 24)                     |
            _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSRSTRIM, 0)                         |
            _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSSPARE, 0);
        MCUCTRL->XTALHSTRIMS = ui32TrimReg;

        uint32_t ui32Reg = MCUCTRL->XTALHSCTRL;
        ui32Reg &= ~(MCUCTRL_XTALHSCTRL_XTALHSPDNB_Msk |
                     MCUCTRL_XTALHSCTRL_XTALHSEXTERNALCLOCK_Msk |
                     MCUCTRL_XTALHSCTRL_XTALHSCOMPPDNB_Msk |
                     MCUCTRL_XTALHSCTRL_XTALHSIBSTENABLE_Msk |
                     MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE_Msk);

        ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNB, 0) |
                   _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSEXTERNALCLOCK, 0) |
                   _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSCOMPPDNB, 1) |
                   _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSIBSTENABLE, 0) |
                   _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE, 0);
        MCUCTRL->XTALHSCTRL = ui32Reg;
    }

    //
    // End critical section.
    //
    AM_CRITICAL_END

    return AM_HAL_STATUS_SUCCESS;
}


// ****************************************************************************
//
//  am_hal_mcuctrl_control()
//  Apply various specific commands/controls on the MCUCTRL module.
//
// ****************************************************************************
uint32_t
am_hal_mcuctrl_control(am_hal_mcuctrl_control_e eControl, void *pArgs)
{
    volatile uint32_t ui32Reg;

    switch ( eControl )
    {
        case AM_HAL_MCUCTRL_CONTROL_EXTCLK32K_ENABLE:
            //
            // Configure the bits in XTALCTRL that enable external 32KHz clock.
            //

            //
            // Begin critical section.
            //
            AM_CRITICAL_BEGIN

            ui32Reg  = MCUCTRL->XTALCTRL;
            ui32Reg &= ~(MCUCTRL_XTALCTRL_XTALPDNB_Msk                      |
                         MCUCTRL_XTALCTRL_XTALCOMPPDNB_Msk                  |
                         MCUCTRL_XTALCTRL_XTALCOMPBYPASS_Msk                |
                         MCUCTRL_XTALCTRL_XTALCOREDISFB_Msk                 |
                         MCUCTRL_XTALCTRL_XTALSWE_Msk);
            ui32Reg |= _VAL2FLD(MCUCTRL_XTALCTRL_XTALPDNB,       MCUCTRL_XTALCTRL_XTALPDNB_PWRUPCORE)       |
                       _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOMPPDNB,   MCUCTRL_XTALCTRL_XTALCOMPPDNB_PWRUPCOMP)   |
                       _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOMPBYPASS, MCUCTRL_XTALCTRL_XTALCOMPBYPASS_USECOMP)   |
                       _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOREDISFB,  MCUCTRL_XTALCTRL_XTALCOREDISFB_EN)         |
                       _VAL2FLD(MCUCTRL_XTALCTRL_XTALSWE,        MCUCTRL_XTALCTRL_XTALSWE_OVERRIDE_EN);
            MCUCTRL->XTALCTRL = ui32Reg;

            //
            // End critical section.
            //
            AM_CRITICAL_END
            break;

        case AM_HAL_MCUCTRL_CONTROL_EXTCLK32K_DISABLE:
            //
            // Configure the bits in XTALCTRL that disable external 32KHz
            // clock, thus re-configuring for the crystal.

            //
            // Begin critical section.
            //
            AM_CRITICAL_BEGIN

            ui32Reg  = MCUCTRL->XTALCTRL;
            ui32Reg &= ~(MCUCTRL_XTALCTRL_XTALPDNB_Msk                      |
                         MCUCTRL_XTALCTRL_XTALCOMPPDNB_Msk                  |
                         MCUCTRL_XTALCTRL_XTALCOMPBYPASS_Msk                |
                         MCUCTRL_XTALCTRL_XTALCOREDISFB_Msk                 |
                         MCUCTRL_XTALCTRL_XTALSWE_Msk);
            ui32Reg |= _VAL2FLD(MCUCTRL_XTALCTRL_XTALPDNB,       MCUCTRL_XTALCTRL_XTALPDNB_PWRUPCORE)       |
                       _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOMPPDNB,   MCUCTRL_XTALCTRL_XTALCOMPPDNB_PWRUPCOMP)   |
                       _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOMPBYPASS, MCUCTRL_XTALCTRL_XTALCOMPBYPASS_USECOMP)   |
                       _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOREDISFB,  MCUCTRL_XTALCTRL_XTALCOREDISFB_EN)         |
                       _VAL2FLD(MCUCTRL_XTALCTRL_XTALSWE,        MCUCTRL_XTALCTRL_XTALSWE_OVERRIDE_DIS);
            MCUCTRL->XTALCTRL = ui32Reg;

            //
            // End critical section.
            //
            AM_CRITICAL_END
            break;
        case AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_CLOCKOUT:

            if ( 0 == pArgs )
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            return mcuctrl_HFXTAL_clockOutPad_mask_modify(*((bool *) pArgs), (1 << AM_HAL_HCXTAL_DEFAULT_EN));

        case AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_KICK_START:
            return mcuctrl_ctrl_HFXTAL_kickstart((am_hal_mcuctrl_control_arg_t *) pArgs);

        case AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_NORMAL:
            return mcuctrl_ctrl_HFXTAL_normal((am_hal_mcuctrl_control_arg_t *) pArgs);

        case AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_DISABLE:
            return mcuctrl_ctrl_HFXTAL_disable((am_hal_mcuctrl_control_arg_t *) pArgs);

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_mcuctrl_control()

// ****************************************************************************
//
//  am_hal_mcuctrl_status_get()
// This function returns  current status of the MCUCTRL as obtained from
// various registers of the MCUCTRL block.
//
// ****************************************************************************
uint32_t
am_hal_mcuctrl_status_get(am_hal_mcuctrl_status_t *psStatus)
{
    uint32_t ui32Status;

    if ( psStatus == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    psStatus->bDebuggerLockout =
        _FLD2VAL(MCUCTRL_DEBUGGER_LOCKOUT, MCUCTRL->DEBUGGER);

    psStatus->bADCcalibrated =
        _FLD2VAL(MCUCTRL_ADCCAL_ADCCALIBRATED, MCUCTRL->ADCCAL);

    psStatus->bBattLoadEnabled =
        _FLD2VAL(MCUCTRL_ADCBATTLOAD_BATTLOAD, MCUCTRL->ADCBATTLOAD);

    ui32Status = MCUCTRL->BOOTLOADER;

    psStatus->bSecBootOnColdRst =
        (_FLD2VAL(MCUCTRL_BOOTLOADER_SECBOOT, ui32Status) != MCUCTRL_BOOTLOADER_SECBOOT_ERROR);
    psStatus->bSecBootOnWarmRst =
        (_FLD2VAL(MCUCTRL_BOOTLOADER_SECBOOTONRST, ui32Status) != MCUCTRL_BOOTLOADER_SECBOOTONRST_ERROR);

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_mcuctrl_status_get()

// ****************************************************************************
//
//  am_hal_mcuctrl_info_get()
//  Get information of the given MCUCTRL item.
//
// ****************************************************************************
uint32_t
am_hal_mcuctrl_info_get(am_hal_mcuctrl_infoget_e eInfoGet, void *pInfo)
{
    am_hal_mcuctrl_feature_t *psFeature;

    if ( pInfo == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    switch ( eInfoGet )
    {
        case AM_HAL_MCUCTRL_INFO_FEATURES_AVAIL:
            psFeature = (am_hal_mcuctrl_feature_t*)pInfo;
            psFeature->eDTCMSize = AM_HAL_MCUCTRL_DTCM_384K;
            psFeature->eSharedSRAMSize = (am_hal_mcuctrl_ssram_e)MCUCTRL->SKU_b.SKUSRAMSIZE;
            psFeature->eMRAMSize = (am_hal_mcuctrl_mram_e)MCUCTRL->SKU_b.SKUMRAMSIZE;
            psFeature->bTurboSpot = (MCUCTRL->SKU_b.SKUTURBOSPOT > 0);
            psFeature->bDisplayCtrl = (MCUCTRL->SKU_b.SKUMIPIDSI > 0);
            psFeature->bGPU = (MCUCTRL->SKU_b.SKUGFX > 0);
            psFeature->bUSB = (MCUCTRL->SKU_b.SKUUSB > 0);
            psFeature->bSecBootFeature = (MCUCTRL->SKU_b.SKUSECURESPOT > 0);
            break;

        case AM_HAL_MCUCTRL_INFO_DEVICEID:
            device_info_get((am_hal_mcuctrl_device_t *)pInfo);
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_mcuctrl_info_get()

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
