//*****************************************************************************
//
//! @file am_hal_pwrctrl.c
//!
//! @brief Functions for enabling and disabling power domains.
//!
//! @addtogroup pwrctrl4_4p PWRCTRL - Power Control
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
// Local defines
//
//*****************************************************************************

//
//! Maximum number of checks to memory power status before declaring error
// (5 x 1usec = 5usec).
//
#define AM_HAL_PWRCTRL_MAX_WAIT_US      5


#define AM_HAL_PWRCTRL_MEMPWREN_MASK    ( PWRCTRL_MEMPWREN_PWRENDTCM_Msk        |   \
                                          PWRCTRL_MEMPWREN_PWRENNVM0_Msk        |   \
                                          PWRCTRL_MEMPWREN_PWRENCACHEB0_Msk     |   \
                                          PWRCTRL_MEMPWREN_PWRENCACHEB2_Msk )

#define AM_HAL_PWRCTRL_DSPMEMPWRST_MASK ( PWRCTRL_DSP0MEMPWRST_PWRSTDSP0RAM_Msk |   \
                                          PWRCTRL_DSP0MEMPWRST_PWRSTDSP0ICACHE_Msk )

//
//! "PCM" is simply a trim level version of product test.
//
#define TRIMREV_PCM             3   // Trim revision number for PCM

//
//! Trim revision 6 is required for the following pwrctrl adjustments:
//!  - Sourcing MCUH from the VDDC_LV rail
//!  - Reliable application of TempCo
//!  - Applying Crypto boosts
//
#define TRIMREV_PWRCTRL         6

//*****************************************************************************
//
//! @name Define max values of some useful fields
//! @{
//
// ****************************************************************************
#define MAX_ACTTRIMVDDF         _FLD2VAL(MCUCTRL_SIMOBUCK12_ACTTRIMVDDF, 0xFFFFFFFF)    // Buck VDDF
#define MAX_MEMLDOACTIVETRIM    _FLD2VAL(MCUCTRL_LDOREG2_MEMLDOACTIVETRIM, 0xFFFFFFFF)  // LDO VDDF
#define MAX_LPTRIMVDDF          _FLD2VAL(MCUCTRL_SIMOBUCK12_LPTRIMVDDF, 0xFFFFFFFF)     // VDDF LP
#define MAX_MEMLPLDOTRIM        _FLD2VAL(MCUCTRL_LDOREG2_MEMLPLDOTRIM, 0xFFFFFFFF)      // MEM LP LDO
#define MAX_TVRGVREFTRIM        _FLD2VAL(MCUCTRL_VREFGEN2_TVRGVREFTRIM, 0xFFFFFFFF)     // Buck VDDC
#define MAX_CORELDOACTIVETRIM   _FLD2VAL(MCUCTRL_LDOREG1_CORELDOACTIVETRIM, 0xFFFFFFFF) // Buck VDDC
//! @}

//
// Internal non-published function, since Timer13 is reserved for a workaround.
//
extern uint32_t internal_timer_config(uint32_t ui32TimerNumber,
                                      am_hal_timer_config_t *psTimerConfig);

// ****************************************************************************
//
// Global variables.
//
// ****************************************************************************

//*****************************************************************************
//
//! @name Global State Variables for the VDDF and VDDC boosting
//! @{
//
// ****************************************************************************
am_hal_pwrctrl_mcu_mode_e g_eCurrPwrMode    = AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER;
uint32_t g_ui32TrimVer                      = 0xFFFFFFFF;
uint32_t g_ui32origSimobuckVDDStrim         = 0xFFFFFFFF;
//! @}

//*****************************************************************************
//
//! @name Save factory trim values.
//! @{
//
// ****************************************************************************
static bool     g_bOrigTrimsStored          = false;
static uint32_t g_orig_ACTTRIMVDDF          = 0;
static uint32_t g_orig_MEMLDOACTIVETRIM     = 0;
static uint32_t g_orig_LPTRIMVDDF           = 0;
static uint32_t g_orig_MEMLPLDOTRIM         = 0;
static uint32_t g_orig_TVRGVREFTRIM         = 0;
static uint32_t g_orig_CORELDOACTIVETRIM    = 0;
//! @}

#if AM_HAL_PWRCTL_OPTIMIZE_ACTIVE_TRIMS_CRYPTO
//*****************************************************************************
//
//! @name Trim state variables
//! @{
//
// ****************************************************************************
static uint32_t g_ui32VDDFAdjustCodes       = 0;
static  int32_t g_i32LatestVddfActTrim      = 0;
static  int32_t g_i32LatestLDOActTrim       = 0;
static bool     g_bBoostForCryptoApplied    = false;
//
//! @}
//
#endif // AM_HAL_PWRCTL_OPTIMIZE_ACTIVE_TRIMS_CRYPTO

#if AM_HAL_TEMPCO_LP
//*****************************************************************************
//
//! @name Saved handle and slot number for TempCo.
//! @{
//
// ****************************************************************************
static void    *g_TempcoADCHandle;
static uint32_t g_ui32TempcoADCslot;
static uint16_t g_ui16TempcoTEMP_code;
static bool     g_bTempcoValid           = false;
static float    g_pfTempMeasured;
//
//! @}
//
#endif // AM_HAL_TEMPCO_LP

//*****************************************************************************
//
//! @name DEVPWREN and DEVPWRSTATUS Mask Macros
//! @{
//! The below DEVPWREN and DEVPWRSTATUS masks are used to check if a peripheral
//!    has been disabled properly
//!
//! The original check of ((PWRCTRL->DEVPWRSTATUS & ui32PeriphStatus) == 0)
//!     will fail when more than one enable in the same domain is set and the
//!     user tries disable only one.
// ****************************************************************************

#define PWRCTRL_HCPB_DEVPWREN_MASK       ( \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM0, PWRCTRL_DEVPWREN_PWRENIOM0_EN) | \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM1, PWRCTRL_DEVPWREN_PWRENIOM1_EN) | \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM2, PWRCTRL_DEVPWREN_PWRENIOM2_EN) | \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM3, PWRCTRL_DEVPWREN_PWRENIOM3_EN))

#define PWRCTRL_HCPC_DEVPWREN_MASK       ( \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM4, PWRCTRL_DEVPWREN_PWRENIOM4_EN) | \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM5, PWRCTRL_DEVPWREN_PWRENIOM5_EN) | \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM6, PWRCTRL_DEVPWREN_PWRENIOM6_EN) | \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM7, PWRCTRL_DEVPWREN_PWRENIOM7_EN))

#define PWRCTRL_HCPA_DEVPWREN_MASK       ( \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART0, PWRCTRL_DEVPWREN_PWRENUART0_EN) | \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART1, PWRCTRL_DEVPWREN_PWRENUART1_EN) | \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART2, PWRCTRL_DEVPWREN_PWRENUART2_EN) | \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART3, PWRCTRL_DEVPWREN_PWRENUART3_EN))

#define PWRCTRL_MSPI_DEVPWREN_MASK       ( \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI0, PWRCTRL_DEVPWREN_PWRENMSPI0_EN) | \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI1, PWRCTRL_DEVPWREN_PWRENMSPI1_EN) | \
    _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI2, PWRCTRL_DEVPWREN_PWRENMSPI2_EN))

#define PWRCTRL_AUD_DEVPWREN_MASK       ( \
    _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENAUDREC, PWRCTRL_AUDSSPWREN_PWRENAUDREC_EN) | \
    _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENAUDPB, PWRCTRL_AUDSSPWREN_PWRENAUDPB_EN)   | \
    _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENPDM0, PWRCTRL_AUDSSPWREN_PWRENPDM0_EN)     | \
    _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENPDM1, PWRCTRL_AUDSSPWREN_PWRENPDM1_EN)     | \
    _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENPDM2, PWRCTRL_AUDSSPWREN_PWRENPDM2_EN)     | \
    _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENPDM3, PWRCTRL_AUDSSPWREN_PWRENPDM3_EN)     | \
    _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENI2S0, PWRCTRL_AUDSSPWREN_PWRENI2S0_EN)     | \
    _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENI2S1, PWRCTRL_AUDSSPWREN_PWRENI2S1_EN))

#define PWRCTRL_HCPB_DEVPWRSTATUS_MASK      ( \
    PWRCTRL_DEVPWRSTATUS_PWRSTIOM0_Msk | \
    PWRCTRL_DEVPWRSTATUS_PWRSTIOM1_Msk | \
    PWRCTRL_DEVPWRSTATUS_PWRSTIOM2_Msk | \
    PWRCTRL_DEVPWRSTATUS_PWRSTIOM3_Msk)

#define PWRCTRL_HCPC_DEVPWRSTATUS_MASK      ( \
    PWRCTRL_DEVPWRSTATUS_PWRSTIOM4_Msk | \
    PWRCTRL_DEVPWRSTATUS_PWRSTIOM5_Msk | \
    PWRCTRL_DEVPWRSTATUS_PWRSTIOM6_Msk | \
    PWRCTRL_DEVPWRSTATUS_PWRSTIOM7_Msk)

#define PWRCTRL_HCPA_DEVPWRSTATUS_MASK          ( \
    PWRCTRL_DEVPWRSTATUS_PWRSTUART0_Msk | \
    PWRCTRL_DEVPWRSTATUS_PWRSTUART1_Msk | \
    PWRCTRL_DEVPWRSTATUS_PWRSTUART2_Msk | \
    PWRCTRL_DEVPWRSTATUS_PWRSTUART3_Msk)

#define PWRCTRL_MSPI_DEVPWRSTATUS_MASK          ( \
    PWRCTRL_DEVPWRSTATUS_PWRSTMSPI0_Msk | \
    PWRCTRL_DEVPWRSTATUS_PWRSTMSPI1_Msk | \
    PWRCTRL_DEVPWRSTATUS_PWRSTMSPI2_Msk)

#define PWRCTRL_AUD_DEVPWRSTATUS_MASK      ( \
    PWRCTRL_AUDSSPWRSTATUS_PWRSTAUDREC_Msk | \
    PWRCTRL_AUDSSPWRSTATUS_PWRSTAUDPB_Msk  | \
    PWRCTRL_AUDSSPWRSTATUS_PWRSTPDM0_Msk   | \
    PWRCTRL_AUDSSPWRSTATUS_PWRSTPDM1_Msk   | \
    PWRCTRL_AUDSSPWRSTATUS_PWRSTPDM2_Msk   | \
    PWRCTRL_AUDSSPWRSTATUS_PWRSTPDM3_Msk   | \
    PWRCTRL_AUDSSPWRSTATUS_PWRSTI2S0_Msk   | \
    PWRCTRL_AUDSSPWRSTATUS_PWRSTI2S1_Msk)
//! @}

// **********************************************
//! Define the peripheral control structure.
// **********************************************
struct am_pwr_s
{
    uint32_t    ui32PwrEnRegAddr;
    uint32_t    ui32PeriphEnable;
    uint32_t    ui32PwrStatReqAddr;
    uint32_t    ui32PeriphStatus;
};

//
//! Peripheral control data structure
//
#ifndef AM_HAL_PWRCTRL_RAM_TABLE
const struct am_pwr_s am_hal_pwrctrl_peripheral_control[AM_HAL_PWRCTRL_PERIPH_MAX] =
{
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOS, PWRCTRL_DEVPWREN_PWRENIOS_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTIOS_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM0, PWRCTRL_DEVPWREN_PWRENIOM0_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPB_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM1, PWRCTRL_DEVPWREN_PWRENIOM1_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPB_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM2, PWRCTRL_DEVPWREN_PWRENIOM2_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPB_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM3, PWRCTRL_DEVPWREN_PWRENIOM3_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPB_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM4, PWRCTRL_DEVPWREN_PWRENIOM4_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPC_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM5, PWRCTRL_DEVPWREN_PWRENIOM5_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPC_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM6, PWRCTRL_DEVPWREN_PWRENIOM6_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPC_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM7, PWRCTRL_DEVPWREN_PWRENIOM7_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPC_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART0, PWRCTRL_DEVPWREN_PWRENUART0_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPA_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART1, PWRCTRL_DEVPWREN_PWRENUART1_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPA_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART2, PWRCTRL_DEVPWREN_PWRENUART2_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPA_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART3, PWRCTRL_DEVPWREN_PWRENUART3_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_HCPA_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENADC, PWRCTRL_DEVPWREN_PWRENADC_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTADC_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI0, PWRCTRL_DEVPWREN_PWRENMSPI0_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_MSPI_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI1, PWRCTRL_DEVPWREN_PWRENMSPI1_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_MSPI_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI2, PWRCTRL_DEVPWREN_PWRENMSPI2_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_MSPI_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENGFX, PWRCTRL_DEVPWREN_PWRENGFX_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTGFX_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENDISP, PWRCTRL_DEVPWREN_PWRENDISP_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTDISP_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENDISPPHY, PWRCTRL_DEVPWREN_PWRENDISPPHY_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTDISPPHY_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENCRYPTO, PWRCTRL_DEVPWREN_PWRENCRYPTO_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTCRYPTO_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENSDIO, PWRCTRL_DEVPWREN_PWRENSDIO_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTSDIO_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUSB, PWRCTRL_DEVPWREN_PWRENUSB_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTUSB_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUSBPHY, PWRCTRL_DEVPWREN_PWRENUSBPHY_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTUSBPHY_Msk
    },
    {
        AM_REGADDR(PWRCTRL, DEVPWREN),
        _VAL2FLD(PWRCTRL_DEVPWREN_PWRENDBG, PWRCTRL_DEVPWREN_PWRENDBG_EN),
        AM_REGADDR(PWRCTRL, DEVPWRSTATUS),
        PWRCTRL_DEVPWRSTATUS_PWRSTDBG_Msk
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENAUDREC, PWRCTRL_AUDSSPWREN_PWRENAUDREC_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUD_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENAUDPB, PWRCTRL_AUDSSPWREN_PWRENAUDPB_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUD_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENPDM0, PWRCTRL_AUDSSPWREN_PWRENPDM0_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUD_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENPDM1, PWRCTRL_AUDSSPWREN_PWRENPDM1_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUD_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENPDM2, PWRCTRL_AUDSSPWREN_PWRENPDM2_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUD_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENPDM3, PWRCTRL_AUDSSPWREN_PWRENPDM3_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUD_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENI2S0, PWRCTRL_AUDSSPWREN_PWRENI2S0_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUD_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENI2S1, PWRCTRL_AUDSSPWREN_PWRENI2S1_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUD_DEVPWRSTATUS_MASK
    },
    {
        AM_REGADDR(PWRCTRL, AUDSSPWREN),
        _VAL2FLD(PWRCTRL_AUDSSPWREN_PWRENAUDADC, PWRCTRL_AUDSSPWREN_PWRENAUDADC_EN),
        AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS),
        PWRCTRL_AUDSSPWRSTATUS_PWRSTAUDADC_Msk
    },
};

//*****************************************************************************
//
//! @brief  Return the pwr_ctrl entry for a given ePeripheral
//!
//! @param  pwr_ctrl address where the power entry is copied
//! @param  ePeripheral the peripheral to copy
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
static inline uint32_t
am_get_pwrctrl(struct am_pwr_s *pwr_ctrl, uint32_t ePeripheral)
{
    if ( pwr_ctrl == NULL || ePeripheral >= AM_HAL_PWRCTRL_PERIPH_MAX )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    *pwr_ctrl = am_hal_pwrctrl_peripheral_control[ePeripheral];

    return AM_HAL_STATUS_SUCCESS;
}
#else
//*****************************************************************************
//
//! @brief  Return the pwr_ctrl entry for a given ePeripheral
//!
//! @param  pwr_ctrl address where the power entry is generated
//! @param  ePeripheral the peripheral for which to generate:
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
static uint32_t
am_get_pwrctrl(struct am_pwr_s *pwr_ctrl, uint32_t ePeripheral)
{
    int shift_pos;

    if (pwr_ctrl == NULL || ePeripheral >= AM_HAL_PWRCTRL_PERIPH_MAX)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if (ePeripheral < AM_HAL_PWRCTRL_PERIPH_AUDREC)
    {
        pwr_ctrl->ui32PwrEnRegAddr = AM_REGADDR(PWRCTRL, DEVPWREN);
        pwr_ctrl->ui32PwrStatReqAddr = AM_REGADDR(PWRCTRL, DEVPWRSTATUS);
        pwr_ctrl->ui32PeriphEnable = 1 << ePeripheral;
        pwr_ctrl->ui32PeriphStatus = 1 << ePeripheral;
    }
    else
    {
        shift_pos = (ePeripheral - AM_HAL_PWRCTRL_PERIPH_AUDREC);
        if (ePeripheral > AM_HAL_PWRCTRL_PERIPH_I2S1)
        {
            shift_pos += 2;
        }

        pwr_ctrl->ui32PwrEnRegAddr =  AM_REGADDR(PWRCTRL, AUDSSPWREN);
        pwr_ctrl->ui32PwrStatReqAddr = AM_REGADDR(PWRCTRL, AUDSSPWRSTATUS);
        pwr_ctrl->ui32PeriphEnable = 1 << shift_pos;
        pwr_ctrl->ui32PeriphStatus = 1 << shift_pos;
    }

    return AM_HAL_STATUS_SUCCESS;
}
#endif // AM_HAL_PWRCTRL_RAM_TABLE

//*****************************************************************************
//
// Default configurations definitions
//
//*****************************************************************************
const am_hal_pwrctrl_mcu_memory_config_t    g_DefaultMcuMemCfg =
{
    .eCacheCfg          = AM_HAL_PWRCTRL_CACHE_ALL,
    .bRetainCache       = true,
    .eDTCMCfg           = AM_HAL_PWRCTRL_DTCM_384K,
    .eRetainDTCM        = AM_HAL_PWRCTRL_DTCM_384K,
    .bEnableNVM0        = true,
    .bRetainNVM0        = false
};

const am_hal_pwrctrl_sram_memcfg_t          g_DefaultSRAMCfg =
{
    //
    //! Default configuration for Shared SRAM:
    //! Enable all SSRAM
    //! All active bits = 0.
    //!   Active bits 0 allow memory to go to retention in deepsleep.
    //!   Active bits 1 force the memory to stay on, requiring more power.
    //! Retain all SSRAM in deepsleep.
    //
    .eSRAMCfg           = AM_HAL_PWRCTRL_SRAM_ALL,
    .eActiveWithMCU     = AM_HAL_PWRCTRL_SRAM_NONE,
    .eActiveWithGFX     = AM_HAL_PWRCTRL_SRAM_NONE,
    .eActiveWithDISP    = AM_HAL_PWRCTRL_SRAM_NONE,
    .eActiveWithDSP     = AM_HAL_PWRCTRL_SRAM_NONE,
    .eSRAMRetain        = AM_HAL_PWRCTRL_SRAM_ALL
};

const am_hal_pwrctrl_dsp_memory_config_t    g_DefaultDSPMemCfg =
{
    .bEnableICache      = false,
    .bRetainCache       = false,
    .bEnableRAM         = true,
    .bActiveRAM         = false,
    .bRetainRAM         = true
};

//*****************************************************************************
//
// Function to determine the chip's TRIM version.
//
// return Status code.
//
// pui32TrimVer: The uint32_t that will receive the trim version number.
//               If no valid trim version found, *pui32TrimVer returns as 0.
//
//
//*****************************************************************************
static uint32_t
TrimVersionGet(uint32_t *pui32TrimVer)
{
    uint32_t ui32Ret;

    //
    // Get the TRIM version and set the global variable.
    // This only needs to be done and verified once.
    //
    if ( g_ui32TrimVer == 0xFFFFFFFF )
    {
        ui32Ret = am_hal_mram_info_read(1, AM_REG_INFO1_TRIM_REV_O / 4, 1, &g_ui32TrimVer);

        if ( (ui32Ret != 0) || (g_ui32TrimVer == 0xFFFFFFFF) )
        {
            //
            // Invalid trim value. Set the global to indicate version 0.
            //
            g_ui32TrimVer = 0;
        }
    }

    if ( pui32TrimVer )
    {
        *pui32TrimVer = g_ui32TrimVer;
        return AM_HAL_STATUS_SUCCESS;
    }
    else
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

} // TrimVersionGet()

#if defined(AM_HAL_PWRCTL_HPLP_WA)
//*****************************************************************************
//
// Function to initialize Timer 13 to interrupt after ui32Delayus.
//
//*****************************************************************************
static uint32_t
am_hal_util_write_and_wait_timer_init(uint32_t ui32Delayus)
{
    am_hal_timer_config_t       TimerConfig;
    uint32_t ui32Status         = AM_HAL_STATUS_SUCCESS;

    //
    // Set the timer configuration
    //
    am_hal_timer_default_config_set(&TimerConfig);
    TimerConfig.eFunction = AM_HAL_TIMER_FN_EDGE;
    TimerConfig.ui32Compare0 = 0xFFFFFFFF;
    TimerConfig.ui32PatternLimit = 0;
    TimerConfig.ui32Compare1 = ui32Delayus * 6000000 / 1000000;
    ui32Status = internal_timer_config(AM_HAL_WRITE_WAIT_TIMER, &TimerConfig);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
       return ui32Status;
    }

    am_hal_timer_clear(AM_HAL_WRITE_WAIT_TIMER);
    am_hal_timer_stop(AM_HAL_WRITE_WAIT_TIMER);

    //
    // Clear the timer Interrupt
    //
    ui32Status = am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(AM_HAL_WRITE_WAIT_TIMER, AM_HAL_TIMER_COMPARE1));
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Status;
    }

    //
    // Enable the timer Interrupt.
    //
    ui32Status = am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(AM_HAL_WRITE_WAIT_TIMER, AM_HAL_TIMER_COMPARE1));
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
       return ui32Status;
    }

    //
    // Enable the timer interrupt in the NVIC.
    //
    // This interrupt needs to be set as the highest priority (0)
    //
    NVIC_SetPriority((IRQn_Type)((uint32_t)TIMER0_IRQn + AM_HAL_WRITE_WAIT_TIMER), 0);
    NVIC_EnableIRQ((IRQn_Type)((uint32_t)TIMER0_IRQn + AM_HAL_WRITE_WAIT_TIMER));

    //
    // No need to enable interrupt as we just need to get out of WFI
    // We just need to clear the NVIC pending
    // am_hal_interrupt_master_enable();
    //

    return ui32Status;

} // am_hal_util_write_and_wait_timer_init()

//*****************************************************************************
//
// Define a simple function that will write a value to register (or other
// memory location) and then go to sleep.
// The opcodes are aligned on a 16-byte boundary to guarantee that
// the write and the WFI are in the same M4 line buffer.
//
//*****************************************************************************
//
// Prototype the assembly function.
//
typedef void (*storeAndWFIfunc_t)(uint32_t ui32Val, uint32_t *pAddr);

#if (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION < 6000000)
__align(16)
#define WA_ATTRIB
#elif (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION >= 6000000)
#warning This attribute is not yet tested on ARM6.
#define WA_ATTRIB   __attribute__ ((aligned (16)))
#elif defined(__GNUC_STDC_INLINE__)
#define WA_ATTRIB   __attribute__ ((aligned (16)))
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = 16
#define WA_ATTRIB
#else
#error Unknown compiler.
#endif

static
uint16_t storeAndWFIRAM[16] WA_ATTRIB =
{
    //
    // r0: Value to be written to the location specified in the 2nd argument.
    // r1: Address of the location to be written.
    //
    // Begin 1st line buffer
    0x6008,             // str r0, [r1]
    0xF3BF, 0x8F4F,     // DSB
    0xBF30,             // WFI
    0xF3BF, 0x8F6F,     // ISB
    0x4770,             // bx lr
    0xBF00,             // nop
};

//
// Prototype the assembly function.
//
storeAndWFIfunc_t storeAndWFIfuncRAM = (storeAndWFIfunc_t)((uint8_t *)storeAndWFIRAM + 1);

//*****************************************************************************
//
// am_hal_util_write_and_wait()
// Function to perform the RevC HP/LP mode switch.
//
//*****************************************************************************
uint32_t
am_hal_util_write_and_wait(uint32_t *pAddr, uint32_t ui32Mask, uint32_t ui32Val, uint32_t ui32Delayus)

{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    uint32_t origBasePri;
    uint32_t basePrioGrouping;

    //
    // Begin critical section
    //
    AM_CRITICAL_BEGIN

    ui32Status = am_hal_util_write_and_wait_timer_init(ui32Delayus);
    if (ui32Status == AM_HAL_STATUS_SUCCESS)
    {

        basePrioGrouping = NVIC_GetPriorityGrouping();
        if (basePrioGrouping == 7)
        {
            //
            // We cannot implement this workaround
            //
            ui32Status = AM_HAL_STATUS_FAIL;
        }
        else
        {
            //
            // Before executing WFI as required later, flush any buffered core and peripheral writes.
            //
            am_hal_sysctrl_bus_write_flush();

            //
            // Mask off all other interrupts
            //
            origBasePri = __get_BASEPRI();
            if (basePrioGrouping >= (8 - __NVIC_PRIO_BITS))
            {
                __set_BASEPRI(1 << (basePrioGrouping + 1));
            }
            else
            {
                __set_BASEPRI(1 << (8 - __NVIC_PRIO_BITS));
            }

            //
            // Compute the value to write
            //
            if (ui32Mask != 0xFFFFFFFF)
            {
                ui32Val |= (AM_REGVAL((uint32_t)pAddr) & ~ui32Mask);
            }

            //
            // Clear the timer.
            //
            am_hal_timer_clear(AM_HAL_WRITE_WAIT_TIMER);

            //
            // Set for normal sleep before calling storeAndWFIfunc()
            //
            SCB->SCR &= ~_VAL2FLD(SCB_SCR_SLEEPDEEP, 1);

            //
            // Call the function to switch the performance mode and WFI.
            //
            storeAndWFIfuncRAM(ui32Val, pAddr);

            //
            // Stop/Disable the timer
            //
            am_hal_timer_stop(AM_HAL_WRITE_WAIT_TIMER);

            //
            // Clear the timer Interrupt
            //
            am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(AM_HAL_WRITE_WAIT_TIMER, AM_HAL_TIMER_COMPARE_BOTH));

            //
            // Before clearing the NVIC pending, avoid a race condition by
            // making sure the interrupt clear has propagated by reading
            // the INTSTAT register.
            //
            volatile uint32_t ui32IntStat;
            am_hal_timer_interrupt_status_get(true, (uint32_t*)&ui32IntStat);

            //
            // Clear pending NVIC interrupt for the timer-specific IRQ.
            //
            NVIC_ClearPendingIRQ((IRQn_Type)((uint32_t)TIMER0_IRQn + AM_HAL_WRITE_WAIT_TIMER));

            //
            // There is also a pending on the timer common IRQ. But it should
            // only be cleared if the workaround timer is the only interrupt.
            //
            if ( !(ui32IntStat &
                        ~AM_HAL_TIMER_MASK(AM_HAL_WRITE_WAIT_TIMER, AM_HAL_TIMER_COMPARE_BOTH)) )
            {
                NVIC_ClearPendingIRQ(TIMER_IRQn);

                //
                // One more race to consider.
                // If a different timer interrupt occurred while clearing the
                // common IRQ, set the timer common IRQ back to pending.
                //
                am_hal_timer_interrupt_status_get(true, (uint32_t*)&ui32IntStat);
                if ( ui32IntStat &
                        ~AM_HAL_TIMER_MASK(AM_HAL_WRITE_WAIT_TIMER, AM_HAL_TIMER_COMPARE_BOTH) )
                {
                    NVIC_SetPendingIRQ(TIMER_IRQn);
                }
            }

            //
            // Restore interrupts
            //
            __set_BASEPRI(origBasePri);
        }
    }

    //
    // End critical section
    //
    AM_CRITICAL_END

    return ui32Status;

} // am_hal_util_write_and_wait()
#endif // defined(AM_HAL_PWRCTL_HPLP_WA)

// ****************************************************************************
//
//  am_hal_pwrctrl_mcu_mode_status()
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_mcu_mode_status(am_hal_pwrctrl_mcu_mode_e *peCurrentPowerMode)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( peCurrentPowerMode == 0 )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    *peCurrentPowerMode = g_eCurrPwrMode;
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_pwrctrl_mcu_mode_status()

// ****************************************************************************
//
//  am_hal_pwrctrl_mcu_mode_select()
//  Select the MCU power mode.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_mcu_mode_select(am_hal_pwrctrl_mcu_mode_e ePowerMode)
{
    uint32_t ui32Status;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( (ePowerMode != AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER)      &&
         (ePowerMode != AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // We must be using SIMOBUCK in order to go to HP mode.
    //
    if ( (ePowerMode == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE)   &&
         (PWRCTRL->VRSTATUS_b.SIMOBUCKST != PWRCTRL_VRSTATUS_SIMOBUCKST_ACT) )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    if ( ePowerMode == g_eCurrPwrMode )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    g_eCurrPwrMode = ePowerMode;

    //
    // Set the MCU power mode.
    //
#ifdef AM_HAL_PWRCTL_HPLP_WA
    ui32Status = am_hal_util_write_and_wait((uint32_t*)&PWRCTRL->MCUPERFREQ,
                                            0xFFFFFFFF, (uint32_t)ePowerMode,
                                            AM_HAL_PWRCTL_HPLP_DELAY);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        //
        // This means that there is another interrupt with the highest
        // priority and the AM_HAL_PWRCTL_HPLP_WA will not work.
        //
        return ui32Status;
    }
#else
    PWRCTRL->MCUPERFREQ_b.MCUPERFREQ = ePowerMode;
#endif // AM_HAL_PWRCTL_HPLP_WA

    //
    // Wait for the ACK
    //
    ui32Status = AM_HAL_STATUS_TIMEOUT;
    for ( uint32_t i = 0; i < 5; i++ )
    {
        if ( PWRCTRL->MCUPERFREQ_b.MCUPERFACK > 0 )
        {
            ui32Status = AM_HAL_STATUS_SUCCESS;
            break;
        }
        am_hal_delay_us(1);
    }

    //
    // Check for timeout.
    //
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        //
        // Caution: Reaching this point means the device is in an unpredictable
        //          state and may not be able to recover.
        //
        return ui32Status;
    }

    //
    // Check the MCU power mode status and return SUCCESS/FAIL.
    //
    if ( PWRCTRL->MCUPERFREQ_b.MCUPERFSTATUS == ePowerMode )
    {
        return AM_HAL_STATUS_SUCCESS;
    }
    else
    {
        //
        // Caution: Reaching this point means the device is in an unpredictable
        //          state and may not be able to recover.
        //
        return AM_HAL_STATUS_FAIL;
    }

} // am_hal_pwrctrl_mcu_mode_select()

// ****************************************************************************
//
//  am_hal_pwrctrl_mcu_memory_config()
//  Configure the MCU memory.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_mcu_memory_config(am_hal_pwrctrl_mcu_memory_config_t *psConfig)
{
    uint32_t      ui32Status;

    //
    // Configure the MCU Cache.
    //
    switch ( psConfig->eCacheCfg )
    {
        case AM_HAL_PWRCTRL_CACHE_NONE:
            PWRCTRL->MEMPWREN_b.PWRENCACHEB0 = PWRCTRL_MEMPWREN_PWRENCACHEB0_DIS;
            PWRCTRL->MEMPWREN_b.PWRENCACHEB2 = PWRCTRL_MEMPWREN_PWRENCACHEB2_DIS;
            break;
        case AM_HAL_PWRCTRL_CACHEB0_ONLY:
            PWRCTRL->MEMPWREN_b.PWRENCACHEB0 = PWRCTRL_MEMPWREN_PWRENCACHEB0_EN;
            PWRCTRL->MEMPWREN_b.PWRENCACHEB2 = PWRCTRL_MEMPWREN_PWRENCACHEB2_DIS;
            break;
        case AM_HAL_PWRCTRL_CACHE_ALL:
            PWRCTRL->MEMPWREN_b.PWRENCACHEB0 = PWRCTRL_MEMPWREN_PWRENCACHEB0_EN;
            PWRCTRL->MEMPWREN_b.PWRENCACHEB2 = PWRCTRL_MEMPWREN_PWRENCACHEB2_EN;
            break;
    }

    //
    // Configure the MCU Tightly Coupled Memory.
    //
    PWRCTRL->MEMPWREN_b.PWRENDTCM = psConfig->eDTCMCfg;

    //
    // Configure the Non-Volatile Memory.
    //
    PWRCTRL->MEMPWREN_b.PWRENNVM0 = psConfig->bEnableNVM0;

    DIAG_SUPPRESS_VOLATILE_ORDER()
    //
    // Wait for Status
    //
    ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                              (uint32_t)&PWRCTRL->MEMPWRSTATUS,
                                              AM_HAL_PWRCTRL_MEMPWREN_MASK,
                                              PWRCTRL->MEMPWREN,
                                              true);

    //
    // Check for timeout.
    //
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Check the MCU power mode status and return SUCCESS/FAIL.
    //
    if ((PWRCTRL->MEMPWRSTATUS_b.PWRSTCACHEB0 != PWRCTRL->MEMPWREN_b.PWRENCACHEB0)  ||
        (PWRCTRL->MEMPWRSTATUS_b.PWRSTCACHEB2 != PWRCTRL->MEMPWREN_b.PWRENCACHEB2)  ||
        (PWRCTRL->MEMPWRSTATUS_b.PWRSTDTCM != PWRCTRL->MEMPWREN_b.PWRENDTCM)        ||
        (PWRCTRL->MEMPWRSTATUS_b.PWRSTNVM0 != PWRCTRL->MEMPWREN_b.PWRENNVM0))
    {
        return AM_HAL_STATUS_FAIL;
    }

    DIAG_DEFAULT_VOLATILE_ORDER()

    //
    // Configure Cache retention.
    //
    if (psConfig->bRetainCache)
    {
        PWRCTRL->MEMRETCFG_b.CACHEPWDSLP = PWRCTRL_MEMRETCFG_CACHEPWDSLP_DIS;
    }
    else
    {
        PWRCTRL->MEMRETCFG_b.CACHEPWDSLP = PWRCTRL_MEMRETCFG_CACHEPWDSLP_EN;
    }

    //
    // Configure the Non-Volatile Memory retention.
    //
    if (psConfig->bRetainNVM0)
    {
        PWRCTRL->MEMRETCFG_b.NVM0PWDSLP = PWRCTRL_MEMRETCFG_NVM0PWDSLP_DIS;
    }
    else
    {
        PWRCTRL->MEMRETCFG_b.NVM0PWDSLP = PWRCTRL_MEMRETCFG_NVM0PWDSLP_EN;
    }

    //
    // Configure the MCU Tightly Coupled Memory retention.
    //
    switch ( psConfig->eRetainDTCM )
    {
        case AM_HAL_PWRCTRL_DTCM_NONE:
            PWRCTRL->MEMRETCFG_b.DTCMPWDSLP = PWRCTRL_MEMRETCFG_DTCMPWDSLP_ALL;
            break;
        case AM_HAL_PWRCTRL_DTCM_8K:
            PWRCTRL->MEMRETCFG_b.DTCMPWDSLP = PWRCTRL_MEMRETCFG_DTCMPWDSLP_ALLBUTGROUP0DTCM0;
            break;
        case AM_HAL_PWRCTRL_DTCM_128K:
            PWRCTRL->MEMRETCFG_b.DTCMPWDSLP = PWRCTRL_MEMRETCFG_DTCMPWDSLP_GROUP1;
            break;
        case AM_HAL_PWRCTRL_DTCM_384K:
            PWRCTRL->MEMRETCFG_b.DTCMPWDSLP = PWRCTRL_MEMRETCFG_DTCMPWDSLP_NONE;
            break;
    }

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_pwrctrl_mcu_memory_config()

// ****************************************************************************
//
//  am_hal_pwrctrl_mcu_memory_config_get()
//  Get the MCU Memory configuration.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_mcu_memory_config_get(am_hal_pwrctrl_mcu_memory_config_t *psConfig)
{
    //
    // Get the MCU Cache configuration.
    //
    if (PWRCTRL->MEMPWREN_b.PWRENCACHEB0 == PWRCTRL_MEMPWREN_PWRENCACHEB0_EN)
    {
        if (PWRCTRL->MEMPWREN_b.PWRENCACHEB2 == PWRCTRL_MEMPWREN_PWRENCACHEB2_EN)
        {
            psConfig->eCacheCfg = AM_HAL_PWRCTRL_CACHE_ALL;
        }
        else
        {
            psConfig->eCacheCfg = AM_HAL_PWRCTRL_CACHEB0_ONLY;
        }
    }
    else
    {
        if (PWRCTRL->MEMPWREN_b.PWRENCACHEB2 == PWRCTRL_MEMPWREN_PWRENCACHEB2_EN)
        {
            return AM_HAL_STATUS_FAIL;  // Not allowed to select Cache B2 only.
            // This should never be possible.
        }
        else
        {
            psConfig->eCacheCfg = AM_HAL_PWRCTRL_CACHE_NONE;
        }
    }

    //
    // Get the MCU Tightly Coupled Memory configuration.
    //
    psConfig->eDTCMCfg =
        (am_hal_pwrctrl_dtcm_select_e)PWRCTRL->MEMPWREN_b.PWRENDTCM;

    //
    // Get the Non-Volatile Memory configuration.
    //
    psConfig->bEnableNVM0 = PWRCTRL->MEMPWREN_b.PWRENNVM0;

    //
    // Get the Cache retention configuration.
    //
    psConfig->bRetainCache =
        (PWRCTRL->MEMRETCFG_b.CACHEPWDSLP == PWRCTRL_MEMRETCFG_CACHEPWDSLP_DIS);

    //
    // Configure the Non-Volatile Memory retention.
    //
    psConfig->bRetainNVM0 =
        (PWRCTRL->MEMRETCFG_b.NVM0PWDSLP == PWRCTRL_MEMRETCFG_NVM0PWDSLP_DIS);

    //
    // Configure the MCU Tightly Coupled Memory retention.
    //
    if (PWRCTRL->MEMRETCFG_b.DTCMPWDSLP == PWRCTRL_MEMRETCFG_DTCMPWDSLP_ALL)
    {
        psConfig->eRetainDTCM = AM_HAL_PWRCTRL_DTCM_NONE;
    }
    else if (PWRCTRL->MEMRETCFG_b.DTCMPWDSLP == PWRCTRL_MEMRETCFG_DTCMPWDSLP_ALLBUTGROUP0DTCM0)
    {
        psConfig->eRetainDTCM = AM_HAL_PWRCTRL_DTCM_8K;
    }
    else if (PWRCTRL->MEMRETCFG_b.DTCMPWDSLP == PWRCTRL_MEMRETCFG_DTCMPWDSLP_GROUP1)
    {
        psConfig->eRetainDTCM = AM_HAL_PWRCTRL_DTCM_128K;
    }
    else if (PWRCTRL->MEMRETCFG_b.DTCMPWDSLP == PWRCTRL_MEMRETCFG_DTCMPWDSLP_NONE)
    {
        psConfig->eRetainDTCM = AM_HAL_PWRCTRL_DTCM_384K;
    }
    else
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_pwrctrl_mcu_memory_config_get()

// ****************************************************************************
//
//  am_hal_pwrctrl_sram_config()
//  Configure the Shared RAM.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_sram_config(am_hal_pwrctrl_sram_memcfg_t *psConfig)
{
    uint32_t      ui32Status;

    //
    // Configure the Shared RAM.
    //
    PWRCTRL->SSRAMPWREN_b.PWRENSSRAM = psConfig->eSRAMCfg;

    DIAG_SUPPRESS_VOLATILE_ORDER()

    //
    // Wait for Status
    //
    ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                              (uint32_t)&PWRCTRL->SSRAMPWRST,
                                              PWRCTRL_SSRAMPWRST_SSRAMPWRST_Msk,
                                              PWRCTRL->SSRAMPWREN,
                                              true);

    //
    // Check for error.
    //
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Check the Shared RAM power mode status.
    //
    if (PWRCTRL->SSRAMPWRST_b.SSRAMPWRST != PWRCTRL->SSRAMPWREN_b.PWRENSSRAM)
    {
        return AM_HAL_STATUS_FAIL;
    }
    DIAG_DEFAULT_VOLATILE_ORDER()

    //
    // Configure the Shared RAM domain active based on the states of the MCU,
    // graphics, and display.
    //
    PWRCTRL->SSRAMRETCFG_b.SSRAMACTMCU  = psConfig->eActiveWithMCU;
    PWRCTRL->SSRAMRETCFG_b.SSRAMACTGFX  = psConfig->eActiveWithGFX;
    PWRCTRL->SSRAMRETCFG_b.SSRAMACTDISP = psConfig->eActiveWithDISP;
    PWRCTRL->SSRAMRETCFG_b.SSRAMACTDSP  = psConfig->eActiveWithDSP;

    //
    // Configure the Shared RAM retention.
    //
    switch ( psConfig->eSRAMRetain )
    {
        case AM_HAL_PWRCTRL_SRAM_NONE:
            PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP = PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_ALL;
            break;
        case AM_HAL_PWRCTRL_SRAM_1M_GRP0:   // Retain lower 1M, pwr dwn upper 1M
            PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP = PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_GROUP1;
            break;
        case AM_HAL_PWRCTRL_SRAM_1M_GRP1:   // Retain upper 1M, pwr dwn lower 1M
            PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP = PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_GROUP0;
            break;
        case AM_HAL_PWRCTRL_SRAM_ALL:       // Retain all SSRAM, pwr dwn none
            PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP = PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_NONE;
            break;
    }

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_pwrctrl_sram_config()

// ****************************************************************************
//
//  am_hal_pwrctrl_sram_config_get()
//  Get the current Shared RAM configuration.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_sram_config_get(am_hal_pwrctrl_sram_memcfg_t *psConfig)
{
    //
    // Get the Shared RAM configuration.
    //
    psConfig->eSRAMCfg = (am_hal_pwrctrl_sram_select_e)PWRCTRL->SSRAMPWREN_b.PWRENSSRAM;

    //
    // Get the SRAM active configurations based for each of MCU, graphics, and display.
    //
    psConfig->eActiveWithMCU  = (am_hal_pwrctrl_sram_select_e)PWRCTRL->SSRAMRETCFG_b.SSRAMACTMCU;
    psConfig->eActiveWithGFX  = (am_hal_pwrctrl_sram_select_e)PWRCTRL->SSRAMRETCFG_b.SSRAMACTGFX;
    psConfig->eActiveWithDISP = (am_hal_pwrctrl_sram_select_e)PWRCTRL->SSRAMRETCFG_b.SSRAMACTDISP;
    psConfig->eActiveWithDSP  = (am_hal_pwrctrl_sram_select_e)PWRCTRL->SSRAMRETCFG_b.SSRAMACTDSP;

    //
    // Get the SRAM retention configuration.
    //
    if (PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP == PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_ALL)
    {
        psConfig->eSRAMRetain = AM_HAL_PWRCTRL_SRAM_NONE;
    }
    else if (PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP == PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_NONE)
    {
        psConfig->eSRAMRetain = AM_HAL_PWRCTRL_SRAM_ALL;
    }
    else if (PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP == PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_GROUP0)
    {
        psConfig->eSRAMRetain = AM_HAL_PWRCTRL_SRAM_1M_GRP1;
    }
    else if (PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP == PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_GROUP1)
    {
        psConfig->eSRAMRetain = AM_HAL_PWRCTRL_SRAM_1M_GRP0;
    }
    else
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_pwrctrl_sram_config_get()

// ****************************************************************************
//
//  am_hal_pwrctrl_dsp_mode_select()
//  Select the DSP power mode.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_dsp_mode_select(am_hal_dsp_select_e eDSP,
                               am_hal_pwrctrl_dsp_mode_e ePowerMode)
{
    uint32_t      ui32Status = AM_HAL_STATUS_SUCCESS;

    //
    // Set the DSP power mode.
    //
    switch ( eDSP )
    {
        case AM_HAL_DSP0:
            PWRCTRL->DSP0PERFREQ_b.DSP0PERFREQ = ePowerMode;
            break;
        case AM_HAL_DSP1:
            PWRCTRL->DSP1PERFREQ_b.DSP1PERFREQ = ePowerMode;
            break;
    }

    //
    // Wait for ACK
    //
    switch ( eDSP )
    {
        case AM_HAL_DSP0:
            ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                                      (uint32_t)&PWRCTRL->DSP0PERFREQ,
                                                      PWRCTRL_DSP0PERFREQ_DSP0PERFACK_Msk,
                                                      (1 << PWRCTRL_DSP0PERFREQ_DSP0PERFACK_Pos),
                                                      true);
        break;
    case AM_HAL_DSP1:
        ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                                  (uint32_t)&PWRCTRL->DSP1PERFREQ,
                                                  PWRCTRL_DSP1PERFREQ_DSP1PERFACK_Msk,
                                                  (1 << PWRCTRL_DSP1PERFREQ_DSP1PERFACK_Pos),
                                                  true);
        break;
    }

    //
    // Check for timeout.
    //
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Check the DSP power mode status and return SUCCESS/FAIL.
    //
    switch ( eDSP )
    {
        case AM_HAL_DSP0:
            if (ePowerMode != PWRCTRL->DSP0PERFREQ_b.DSP0PERFSTATUS)
            {
                return AM_HAL_STATUS_FAIL;
            }
            break;
        case AM_HAL_DSP1:
            if (ePowerMode != PWRCTRL->DSP1PERFREQ_b.DSP1PERFSTATUS)
            {
                return AM_HAL_STATUS_FAIL;
            }
            break;
    }

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_pwrctrl_dsp_mode_select()

// ****************************************************************************
//
//  dsp0_memory_config()
//
// ****************************************************************************
static uint32_t
dsp0_memory_config(am_hal_pwrctrl_dsp_memory_config_t *psConfig)
{
    uint32_t      ui32Status;

    // Configure ICache.
    if (psConfig->bEnableICache)
    {
        PWRCTRL->DSP0MEMPWREN_b.PWRENDSP0ICACHE = PWRCTRL_DSP0MEMPWREN_PWRENDSP0ICACHE_ON;
    }
    else
    {
        PWRCTRL->DSP0MEMPWREN_b.PWRENDSP0ICACHE = PWRCTRL_DSP0MEMPWREN_PWRENDSP0ICACHE_OFF;
    }

    // Configure RAM.
    if (psConfig->bEnableRAM)
    {
        PWRCTRL->DSP0MEMPWREN_b.PWRENDSP0RAM = PWRCTRL_DSP0MEMPWREN_PWRENDSP0RAM_ON;
    }
    else
    {
        PWRCTRL->DSP0MEMPWREN_b.PWRENDSP0RAM = PWRCTRL_DSP0MEMPWREN_PWRENDSP0RAM_OFF;
    }

    DIAG_SUPPRESS_VOLATILE_ORDER()

    //
    // Wait for Status
    //
    ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                              (uint32_t)&PWRCTRL->DSP0MEMPWRST,
                                              AM_HAL_PWRCTRL_DSPMEMPWRST_MASK,
                                              PWRCTRL->DSP0MEMPWREN,
                                              true);

    //
    // Check for error.
    //
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Check for timeout.
    //
    if ((PWRCTRL->DSP0MEMPWRST_b.PWRSTDSP0ICACHE != PWRCTRL->DSP0MEMPWREN_b.PWRENDSP0ICACHE) ||
        (PWRCTRL->DSP0MEMPWRST_b.PWRSTDSP0RAM != PWRCTRL->DSP0MEMPWREN_b.PWRENDSP0RAM) )
    {
        return AM_HAL_STATUS_FAIL;
    }
    DIAG_DEFAULT_VOLATILE_ORDER()

    // Configure ICache Retention.
    if (psConfig->bRetainCache)
    {
        PWRCTRL->DSP0MEMRETCFG_b.ICACHEPWDDSP0OFF = PWRCTRL_DSP0MEMRETCFG_ICACHEPWDDSP0OFF_RET;
    }
    else
    {
        PWRCTRL->DSP0MEMRETCFG_b.ICACHEPWDDSP0OFF = PWRCTRL_DSP0MEMRETCFG_ICACHEPWDDSP0OFF_PWD;
    }

    // Configure IRAM Retention.
    if (psConfig->bActiveRAM)
    {
        PWRCTRL->DSP0MEMRETCFG_b.DSP0RAMACTMCU = PWRCTRL_DSP0MEMRETCFG_DSP0RAMACTMCU_ACT;
    }
    else
    {
        PWRCTRL->DSP0MEMRETCFG_b.DSP0RAMACTMCU = PWRCTRL_DSP0MEMRETCFG_DSP0RAMACTMCU_WAKEONDEMAND;
    }
    if (psConfig->bRetainRAM)
    {
        PWRCTRL->DSP0MEMRETCFG_b.RAMPWDDSP0OFF = PWRCTRL_DSP0MEMRETCFG_RAMPWDDSP0OFF_RET;
    }
    else
    {
        PWRCTRL->DSP0MEMRETCFG_b.RAMPWDDSP0OFF = PWRCTRL_DSP0MEMRETCFG_RAMPWDDSP0OFF_PWD;
    }

    return AM_HAL_STATUS_SUCCESS;
} // dsp0_memory_config()

// ****************************************************************************
//
//  dsp1_memory_config()
//
// ****************************************************************************
static uint32_t
dsp1_memory_config(am_hal_pwrctrl_dsp_memory_config_t *psConfig)
{
    uint32_t      ui32Status;

    // Configure ICache.
    if (psConfig->bEnableICache)
    {
        PWRCTRL->DSP1MEMPWREN_b.PWRENDSP1ICACHE = PWRCTRL_DSP1MEMPWREN_PWRENDSP1ICACHE_ON;
    }
    else
    {
        PWRCTRL->DSP1MEMPWREN_b.PWRENDSP1ICACHE = PWRCTRL_DSP1MEMPWREN_PWRENDSP1ICACHE_OFF;
    }

    // Configure RAM.
    if (psConfig->bEnableRAM)
    {
        PWRCTRL->DSP1MEMPWREN_b.PWRENDSP1RAM = PWRCTRL_DSP1MEMPWREN_PWRENDSP1RAM_ON;
    }
    else
    {
        PWRCTRL->DSP1MEMPWREN_b.PWRENDSP1RAM = PWRCTRL_DSP1MEMPWREN_PWRENDSP1RAM_OFF;
    }

    DIAG_SUPPRESS_VOLATILE_ORDER()

    //
    // Wait for Status
    //
    ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                              (uint32_t)&PWRCTRL->DSP1MEMPWRST,
                                              AM_HAL_PWRCTRL_DSPMEMPWRST_MASK,
                                              PWRCTRL->DSP1MEMPWREN,
                                              true);

    //
    // Check for error.
    //
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Check for timeout.
    //
    if ((PWRCTRL->DSP1MEMPWRST_b.PWRSTDSP1ICACHE != PWRCTRL->DSP1MEMPWREN_b.PWRENDSP1ICACHE) ||
        (PWRCTRL->DSP1MEMPWRST_b.PWRSTDSP1RAM != PWRCTRL->DSP1MEMPWREN_b.PWRENDSP1RAM) )
    {
        return AM_HAL_STATUS_FAIL;
    }
    DIAG_DEFAULT_VOLATILE_ORDER()

    // Configure ICache Retention.
    if (psConfig->bRetainCache)
    {
        PWRCTRL->DSP1MEMRETCFG_b.ICACHEPWDDSP1OFF = PWRCTRL_DSP1MEMRETCFG_ICACHEPWDDSP1OFF_RET;
    }
    else
    {
        PWRCTRL->DSP1MEMRETCFG_b.ICACHEPWDDSP1OFF = PWRCTRL_DSP1MEMRETCFG_ICACHEPWDDSP1OFF_PWD;
    }

    // Configure IRAM Retention.
    if (psConfig->bActiveRAM)
    {
        PWRCTRL->DSP1MEMRETCFG_b.DSP1RAMACTMCU = PWRCTRL_DSP1MEMRETCFG_DSP1RAMACTMCU_ACT;
    }
    else
    {
        PWRCTRL->DSP1MEMRETCFG_b.DSP1RAMACTMCU = PWRCTRL_DSP1MEMRETCFG_DSP1RAMACTMCU_WAKEONDEMAND;
    }
    if (psConfig->bRetainRAM)
    {
        PWRCTRL->DSP1MEMRETCFG_b.RAMPWDDSP1OFF = PWRCTRL_DSP1MEMRETCFG_RAMPWDDSP1OFF_RET;
    }
    else
    {
        PWRCTRL->DSP1MEMRETCFG_b.RAMPWDDSP1OFF = PWRCTRL_DSP1MEMRETCFG_RAMPWDDSP1OFF_PWD;
    }

    return AM_HAL_STATUS_SUCCESS;
} // dsp1_memory_config()

// ****************************************************************************
//
//  am_hal_pwrctrl_dsp_memory_config()
//  Configure the DSP memory.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_dsp_memory_config(am_hal_dsp_select_e eDSP,
                                 am_hal_pwrctrl_dsp_memory_config_t *psConfig)
{
    uint32_t    retval = AM_HAL_STATUS_SUCCESS;

    switch ( eDSP )
    {
        case AM_HAL_DSP0:
            retval = dsp0_memory_config(psConfig);
            break;
        case AM_HAL_DSP1:
            retval = dsp1_memory_config(psConfig);
            break;
    }

    return retval;
} // am_hal_pwrctrl_dsp_memory_config()

// ****************************************************************************
//
//  dsp0_memory_get()
//
// ****************************************************************************
static uint32_t
dsp0_memory_get(am_hal_pwrctrl_dsp_memory_config_t *psConfig)
{

    // Read the ICache configuration.
    psConfig->bEnableICache = (PWRCTRL_DSP0MEMPWREN_PWRENDSP0ICACHE_ON == PWRCTRL->DSP0MEMPWREN_b.PWRENDSP0ICACHE );
    psConfig->bRetainCache  = (PWRCTRL_DSP0MEMRETCFG_ICACHEPWDDSP0OFF_RET == PWRCTRL->DSP0MEMRETCFG_b.ICACHEPWDDSP0OFF);

    // Read the RAM configuration.
    psConfig->bEnableRAM    = (PWRCTRL->DSP0MEMPWREN_b.PWRENDSP0RAM == PWRCTRL_DSP0MEMPWREN_PWRENDSP0RAM_ON);
    psConfig->bActiveRAM    = (PWRCTRL->DSP0MEMRETCFG_b.DSP0RAMACTMCU == PWRCTRL_DSP0MEMRETCFG_DSP0RAMACTMCU_ACT);
    psConfig->bRetainRAM    = (PWRCTRL->DSP0MEMRETCFG_b.RAMPWDDSP0OFF == PWRCTRL_DSP0MEMRETCFG_RAMPWDDSP0OFF_RET);

    return AM_HAL_STATUS_SUCCESS;
} // dsp0_memory_get()

// ****************************************************************************
//
//  dsp1_memory_get()
//
// ****************************************************************************
static uint32_t
dsp1_memory_get(am_hal_pwrctrl_dsp_memory_config_t *psConfig)
{
    // Read the ICache configuration.
    psConfig->bEnableICache = (PWRCTRL_DSP1MEMPWREN_PWRENDSP1ICACHE_ON == PWRCTRL->DSP1MEMPWREN_b.PWRENDSP1ICACHE );
    psConfig->bRetainCache  = (PWRCTRL_DSP1MEMRETCFG_ICACHEPWDDSP1OFF_RET == PWRCTRL->DSP1MEMRETCFG_b.ICACHEPWDDSP1OFF);

    // Read the RAM configuration.
    psConfig->bEnableRAM    = (PWRCTRL->DSP1MEMPWREN_b.PWRENDSP1RAM == PWRCTRL_DSP1MEMPWREN_PWRENDSP1RAM_ON);
    psConfig->bActiveRAM    = (PWRCTRL->DSP1MEMRETCFG_b.DSP1RAMACTMCU == PWRCTRL_DSP1MEMRETCFG_DSP1RAMACTMCU_ACT);
    psConfig->bRetainRAM    = (PWRCTRL->DSP1MEMRETCFG_b.RAMPWDDSP1OFF == PWRCTRL_DSP1MEMRETCFG_RAMPWDDSP1OFF_RET);

    return AM_HAL_STATUS_SUCCESS;
} // dsp1_memory_get()

// ****************************************************************************
//
//  am_hal_pwrctrl_dsp_memory_config_get()
//  Get the current the DSP memory configuration.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_dsp_memory_config_get(am_hal_dsp_select_e eDSP,
                                     am_hal_pwrctrl_dsp_memory_config_t *psConfig)
{
    uint32_t      retval = AM_HAL_STATUS_SUCCESS;

    switch ( eDSP )
    {
        case AM_HAL_DSP0:
            retval = dsp0_memory_get(psConfig);
            break;
        case AM_HAL_DSP1:
            retval = dsp1_memory_get(psConfig);
            break;
    }

    return retval;
} // am_hal_pwrctrl_dsp_memory_config_get()

#if AM_HAL_PWRCTL_OPTIMIZE_ACTIVE_TRIMS_CRYPTO
// ****************************************************************************
//
//  crypto_boost_trims()
//
// ****************************************************************************
void
crypto_boost_trims( bool bBoost, int32_t i32VddActAdj )
{
    int32_t i32VddfActTrim, i32LDOActTrim;

    //
    // Make sure we need to do something.
    //
    if ( (bBoost && g_bBoostForCryptoApplied) || (!bBoost && !g_bBoostForCryptoApplied) )
    {
        return;
    }

    AM_CRITICAL_BEGIN

    g_bBoostForCryptoApplied = bBoost;

    //
    // Handle the buck active boost.
    //
    g_i32LatestVddfActTrim = bBoost ? g_i32LatestVddfActTrim + i32VddActAdj
                                    : g_i32LatestVddfActTrim - i32VddActAdj;
    i32VddfActTrim = g_i32LatestVddfActTrim;

    if ( i32VddfActTrim < 0 )
    {
        i32VddfActTrim = 0;
    }
    else if ( i32VddfActTrim > MAX_ACTTRIMVDDF )
    {
        i32VddfActTrim = MAX_ACTTRIMVDDF;
    }

    //
    // Handle the mem LDO active boost.
    //
    g_i32LatestLDOActTrim = bBoost ? g_i32LatestLDOActTrim + i32VddActAdj
                                   : g_i32LatestLDOActTrim - i32VddActAdj;
    i32LDOActTrim = g_i32LatestLDOActTrim;

    if ( i32LDOActTrim < 0 )
    {
        i32LDOActTrim = 0;
    }
    else if ( i32LDOActTrim > MAX_MEMLDOACTIVETRIM )
    {
        i32LDOActTrim = MAX_MEMLDOACTIVETRIM;
    }

    if ( bBoost )
    {
        //
        // Boost Simobuck first
        //
        MCUCTRL->SIMOBUCK12_b.ACTTRIMVDDF = i32VddfActTrim;
        am_hal_delay_us(AM_HAL_PWRCTRL_VDDF_BOOST_DELAY);

        //
        // Boost VDDF LDO after simobuck
        //
        MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM = i32LDOActTrim;
    }
    else
    {
        //
        // Reduce VDDF LDO first
        //
        MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM = i32LDOActTrim;
        am_hal_delay_us(AM_HAL_PWRCTRL_MEMLDO_BOOST_DELAY);

        //
        // Reduce Simobuck
        //
        MCUCTRL->SIMOBUCK12_b.ACTTRIMVDDF = i32VddfActTrim;
    }

    AM_CRITICAL_END

#if ( AM_HAL_PWRCTRL_VDDF_BOOST_DELAY >= AM_HAL_PWRCTRL_MEMLDO_BOOST_DELAY )
    // Use the longer delay
    am_hal_delay_us(AM_HAL_PWRCTRL_VDDF_BOOST_DELAY);
#else
    am_hal_delay_us(AM_HAL_PWRCTRL_MEMLDO_BOOST_DELAY);
#endif

} // crypto_boost_trims()
#endif // AM_HAL_PWRCTL_OPTIMIZE_ACTIVE_TRIMS_CRYPTO

//******************************************************************************
//
// Function that waits for crypto peripheral to stabilize before or after
// powerup/powerdown.
//
//******************************************************************************
#define CRYPTO_WAIT_USEC        100
static uint32_t
crypto_quiesce(void)
{
    uint32_t ui32Status;

    //
    // Wait for crypto block idle.
    //
    ui32Status = am_hal_delay_us_status_change(CRYPTO_WAIT_USEC,
                                                (uint32_t)&CRYPTO->HOSTCCISIDLE,
                                                CRYPTO_HOSTCCISIDLE_HOSTCCISIDLE_Msk,
                                                CRYPTO_HOSTCCISIDLE_HOSTCCISIDLE_Msk);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Wait for OTP idle.
    //
    ui32Status = am_hal_delay_us_status_change(CRYPTO_WAIT_USEC,
                                               (uint32_t)&CRYPTO->NVMISIDLE,
                                               CRYPTO_NVMISIDLE_NVMISIDLEREG_Msk,
                                               CRYPTO_NVMISIDLE_NVMISIDLEREG_Msk);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Alert the CRYPTO block of imminent power down.
    //
    CRYPTO->HOSTPOWERDOWN_b.HOSTPOWERDOWN = 1;

    return AM_HAL_STATUS_SUCCESS;
} // crypto_quiesce()

// ****************************************************************************
//
//  am_hal_pwrctrl_periph_enable()
//  Enable power for a peripheral.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_periph_enable(am_hal_pwrctrl_periph_e ePeripheral)
{
    uint32_t      ui32Status;
    struct am_pwr_s pwr_ctrl;

    ui32Status = am_get_pwrctrl(&pwr_ctrl, ePeripheral);

    if ( AM_HAL_STATUS_SUCCESS != ui32Status )
    {
        return ui32Status;
    }

    if ( AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & pwr_ctrl.ui32PeriphEnable )
    {
        //
        // We're already enabled, nothing to do.
        //
        return AM_HAL_STATUS_SUCCESS;
    }

#if AM_HAL_PWRCTL_OPTIMIZE_ACTIVE_TRIMS_CRYPTO
    if ( ePeripheral == AM_HAL_PWRCTRL_PERIPH_CRYPTO )
    {
        //
        // Before enabling Crypto, make sure voltages have been boosted
        //
        if ( g_ui32VDDFAdjustCodes != 0 )
        {
            crypto_boost_trims( true, g_ui32VDDFAdjustCodes );
        }
    }
#endif // AM_HAL_PWRCTL_OPTIMIZE_ACTIVE_TRIMS_CRYPTO

    //
    // Enable power control for the given device.
    //
    AM_CRITICAL_BEGIN
    AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) |=
        pwr_ctrl.ui32PeriphEnable;
    AM_CRITICAL_END

    ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                              pwr_ctrl.ui32PwrStatReqAddr,
                                              pwr_ctrl.ui32PeriphStatus,
                                              pwr_ctrl.ui32PeriphStatus,
                                              true);

    //
    // Check for timeout.
    //
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
#if AM_HAL_PWRCTL_OPTIMIZE_ACTIVE_TRIMS_CRYPTO
        if ( ePeripheral == AM_HAL_PWRCTRL_PERIPH_CRYPTO )
        {
            //
            // Crypto enable was not successful after all, so revert the boost.
            //
            if ( g_ui32VDDFAdjustCodes != 0 )
            {
                crypto_boost_trims( false, g_ui32VDDFAdjustCodes );
            }
        }
#endif // AM_HAL_PWRCTL_OPTIMIZE_ACTIVE_TRIMS_CRYPTO

        return ui32Status;
    }

    //
    // Crypto peripheral needs more time to power up after the normal status bit
    // is set. We'll wait for the IDLE signal from the NVM as our signal that
    // crypto is ready.
    //
    if (ePeripheral == AM_HAL_PWRCTRL_PERIPH_CRYPTO)
    {
        ui32Status = am_hal_delay_us_status_change(CRYPTO_WAIT_USEC,
                                                   (uint32_t)&CRYPTO->NVMISIDLE,
                                                   CRYPTO_NVMISIDLE_NVMISIDLEREG_Msk,
                                                   CRYPTO_NVMISIDLE_NVMISIDLEREG_Msk);

        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return ui32Status;
        }
    }

    //
    // Check the device status.
    //
    if ( (AM_REGVAL(pwr_ctrl.ui32PwrStatReqAddr) &
        pwr_ctrl.ui32PeriphStatus) != 0)
    {
        return AM_HAL_STATUS_SUCCESS;
    }
    else
    {
        return AM_HAL_STATUS_FAIL;
    }
} // am_hal_pwrctrl_periph_enable()

// ****************************************************************************
//
//  am_hal_pwrctrl_periph_disable_msk_check()
//  Function checks the PWRCTRL->DEVPWREN
//
//  The original check of ((PWRCTRL->DEVPWRSTATUS & ui32PeriphStatus) == 0)
//      will fail when more than one enable in the same domain is set and the
//      user tries disable only one.
//
// ****************************************************************************
static uint32_t
pwrctrl_periph_disable_msk_check(am_hal_pwrctrl_periph_e ePeripheral)
{
    uint32_t      ui32Status;
    struct am_pwr_s pwr_ctrl;

    ui32Status = am_get_pwrctrl(&pwr_ctrl, ePeripheral);

    if ( AM_HAL_STATUS_SUCCESS != ui32Status )
    {
        return ui32Status;
    }

    switch (pwr_ctrl.ui32PeriphStatus)
    {
        case (PWRCTRL_HCPA_DEVPWRSTATUS_MASK):
            if (((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & PWRCTRL_HCPA_DEVPWREN_MASK) != 0) &&
                ((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & pwr_ctrl.ui32PeriphEnable) == 0))
            {
                ui32Status = AM_HAL_STATUS_SUCCESS;
            }
            break;

        case (PWRCTRL_HCPB_DEVPWRSTATUS_MASK):
            if (((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & PWRCTRL_HCPB_DEVPWREN_MASK) != 0) &&
                ((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & pwr_ctrl.ui32PeriphEnable) == 0))
            {
                ui32Status = AM_HAL_STATUS_SUCCESS;
            }
            break;

        case (PWRCTRL_HCPC_DEVPWRSTATUS_MASK):
            if (((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & PWRCTRL_HCPC_DEVPWREN_MASK) != 0) &&
                ((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & pwr_ctrl.ui32PeriphEnable) == 0))
            {
                ui32Status = AM_HAL_STATUS_SUCCESS;
            }
            break;

        case (PWRCTRL_MSPI_DEVPWRSTATUS_MASK):
            if (((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & PWRCTRL_MSPI_DEVPWREN_MASK) != 0) &&
                ((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & pwr_ctrl.ui32PeriphEnable) == 0))
            {
                ui32Status = AM_HAL_STATUS_SUCCESS;
            }
            break;

        case (PWRCTRL_AUD_DEVPWRSTATUS_MASK):
            if (((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & PWRCTRL_AUD_DEVPWREN_MASK) != 0) &&
                ((AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & pwr_ctrl.ui32PeriphEnable) == 0))
            {
                ui32Status = AM_HAL_STATUS_SUCCESS;
            }
            break;

        default:
            break;
    }

    return ui32Status;
}

// ****************************************************************************
//
//  am_hal_pwrctrl_periph_disable()
//  Disable power for a peripheral.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_periph_disable(am_hal_pwrctrl_periph_e ePeripheral)
{
    uint32_t      ui32Status;
    struct am_pwr_s pwr_ctrl;

    ui32Status = am_get_pwrctrl(&pwr_ctrl, ePeripheral);

    if ( AM_HAL_STATUS_SUCCESS != ui32Status )
    {
        return ui32Status;
    }

    if ( !(AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) & pwr_ctrl.ui32PeriphEnable) )
    {
        //
        // We're already disabled, nothing to do.
        //
        return AM_HAL_STATUS_SUCCESS;
    }

    //
    // The crypto block needs to be idle before it can be shut down. First,
    // we'll check to make sure crypto is actually on by checking the
    // peripheral ID bits (otherwise the next step would fault). If CRYPTO is
    // on, we poll to make sure NVM is inactive before setting the
    // HOSTPOWERDOWN bit to start the shutdown process.
    //
    if (ePeripheral == AM_HAL_PWRCTRL_PERIPH_CRYPTO)
    {
        if (CRYPTO->PERIPHERALID0 == 0xC0)
        {
            ui32Status = crypto_quiesce();

            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return ui32Status;
            }

#if AM_HAL_PWRCTL_OPTIMIZE_ACTIVE_TRIMS_CRYPTO
            //
            // After disabling Crypto, remove the voltage boost
            //
            if ( g_ui32VDDFAdjustCodes != 0 )
            {
                crypto_boost_trims( false, g_ui32VDDFAdjustCodes );
            }
#endif // AM_HAL_PWRCTL_OPTIMIZE_ACTIVE_TRIMS_CRYPTO
        }
    }

    //
    // The peripheral is not AM_HAL_PWRCTRL_PERIPH_CRYPTO.
    // Disable power domain for the given device.
    //
    AM_CRITICAL_BEGIN
    AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) &= ~pwr_ctrl.ui32PeriphEnable;
    AM_CRITICAL_END

    //
    //  This check will fail when more than one enable in the same domain
    //      is set and the user tries to disable only one.
    //
    ui32Status = am_hal_delay_us_status_check(AM_HAL_PWRCTRL_MAX_WAIT_US,
                                              pwr_ctrl.ui32PwrStatReqAddr,
                                              pwr_ctrl.ui32PeriphStatus,
                                              pwr_ctrl.ui32PeriphStatus,
                                              false);

    //
    // Check for success.
    //
    if (AM_HAL_STATUS_SUCCESS == ui32Status)
    {
        return ui32Status;
    }
    else
    {
        return pwrctrl_periph_disable_msk_check(ePeripheral);
    }

} // am_hal_pwrctrl_periph_disable()

// ****************************************************************************
//
//  am_hal_pwrctrl_periph_enabled()
//  Determine whether a peripheral is currently enabled.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_periph_enabled(am_hal_pwrctrl_periph_e ePeripheral,
                              bool *bEnabled)
{
    uint32_t      ui32Status;
    struct am_pwr_s pwr_ctrl;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( bEnabled == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Initialize bEnabled to false in case an error is encountered.
    //
    *bEnabled = false;

    ui32Status = am_get_pwrctrl(&pwr_ctrl, ePeripheral);

    if ( AM_HAL_STATUS_SUCCESS != ui32Status )
    {
        return ui32Status;
    }

    *bEnabled = ((AM_REGVAL(pwr_ctrl.ui32PwrStatReqAddr) &
                  pwr_ctrl.ui32PeriphStatus) != 0);

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_pwrctrl_periph_enabled()

// ****************************************************************************
//
//  am_hal_pwrctrl_status_get()
//  Get the current powercontrol status registers.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_status_get(am_hal_pwrctrl_status_t *psStatus)
{
    //
    // Device Power ON Status
    //
    psStatus->ui32Device = PWRCTRL->DEVPWRSTATUS;

    //
    // Audio Subsystem ON Status
    //
    psStatus->ui32AudioSS = PWRCTRL->AUDSSPWRSTATUS;

    //
    // MCU Memory Power ON Status
    //
    psStatus->ui32Memory = PWRCTRL->MEMPWRSTATUS;

    //
    // Power ON Status for MCU and DSP0/1 Cores
    //
    psStatus->ui32System = PWRCTRL->SYSPWRSTATUS;

    //
    // Shared SRAM Power ON Status
    //
    psStatus->ui32SSRAM = PWRCTRL->SSRAMPWRST;

    //
    // DSP0 Memories Power ON Status
    //
    psStatus->ui32DSP0MemStatus = PWRCTRL->DSP0MEMPWRST;

    //
    // DSP1 Memories Power ON Status
    //
    psStatus->ui32DSP1MemStatus = PWRCTRL->DSP1MEMPWRST;

    //
    // Voltage Regulators status
    //
    psStatus->ui32VRStatus = PWRCTRL->VRSTATUS;

    //
    // Power Status Register for ADC Block
    //
    psStatus->ui32ADC = PWRCTRL->ADCSTATUS;

    //
    // Power Status Register for audio ADC Block
    //
    psStatus->ui32AudioADC = PWRCTRL->AUDADCSTATUS;

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_pwrctrl_status_get()

// ****************************************************************************
//
//  am_hal_pwrctrl_low_power_init()
//  Initialize the device for low power operation.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_low_power_init(void)
{
#if AM_HAL_PWRCTL_OPTIMIZE_ACTIVE_TRIMS_CRYPTO
    uint32_t ui32Ret, ui32PatchTracker;

    //
    // Determine the initial state of Crypto.
    //
    g_bBoostForCryptoApplied = PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO ? true : false;
#endif // AM_HAL_PWRCTL_OPTIMIZE_ACTIVE_TRIMS_CRYPTO

    //
    // Set the default memory configuration.
    //
    am_hal_pwrctrl_mcu_memory_config((am_hal_pwrctrl_mcu_memory_config_t *)&g_DefaultMcuMemCfg);
    am_hal_pwrctrl_sram_config((am_hal_pwrctrl_sram_memcfg_t *)&g_DefaultSRAMCfg);

    //
    // Enable clock gate optimizations for Apollo4.
    //
    CLKGEN->MISC |=
        _VAL2FLD(CLKGEN_MISC_CM4DAXICLKGATEEN, 1)       |   // [18] CM4 DAXI CLK
        _VAL2FLD(CLKGEN_MISC_GFXCLKCLKGATEEN, 1)        |   // [19] GFX CLK
        _VAL2FLD(CLKGEN_MISC_GFXAXICLKCLKGATEEN, 1)     |   // [20] GFX AXI CLK
        _VAL2FLD(CLKGEN_MISC_APBDMACPUCLKCLKGATEEN, 1)  |   // [21] APB DMA CPU CLK
        _VAL2FLD(CLKGEN_MISC_ETMTRACECLKCLKGATEEN, 1)   |   // [22] ETM TRACE CLK
        _VAL2FLD(CLKGEN_MISC_HFRCFUNCCLKGATEEN, 1);         // [23] HFRC_FUNC_CLK

    //
    // Set the PWRCTRL PWRWEIGHTS all to 0's.
    //
    PWRCTRL->PWRWEIGHTULP0  = 0;
    PWRCTRL->PWRWEIGHTULP1  = 0;
    PWRCTRL->PWRWEIGHTULP2  = 0;
    PWRCTRL->PWRWEIGHTULP3  = 0;
    PWRCTRL->PWRWEIGHTULP4  = 0;
    PWRCTRL->PWRWEIGHTULP5  = 0;
    PWRCTRL->PWRWEIGHTLP0   = 0;
    PWRCTRL->PWRWEIGHTLP1   = 0;
    PWRCTRL->PWRWEIGHTLP2   = 0;
    PWRCTRL->PWRWEIGHTLP3   = 0;
    PWRCTRL->PWRWEIGHTLP4   = 0;
    PWRCTRL->PWRWEIGHTLP5   = 0;
    PWRCTRL->PWRWEIGHTHP0   = 0;
    PWRCTRL->PWRWEIGHTHP1   = 0;
    PWRCTRL->PWRWEIGHTHP2   = 0;
    PWRCTRL->PWRWEIGHTHP3   = 0;
    PWRCTRL->PWRWEIGHTHP4   = 0;
    PWRCTRL->PWRWEIGHTHP5   = 0;
    PWRCTRL->PWRWEIGHTSLP   = 0;

    //
    //  Set up the Default DAXICFG.
    //
    am_hal_daxi_config(&am_hal_daxi_defaults);
    am_hal_delay_us(100);

    //
    // Additional required settings
    //
    CLKGEN->MISC_b.PWRONCLKENDISP = 1;

    //
    // Initialize DSPRAM, SSRAM trims for proper retention operation.
    //
    MCUCTRL->PWRSW0 |=  _VAL2FLD(MCUCTRL_PWRSW0_PWRSWVDDMDSP0DYNSEL, 1)     |
                        _VAL2FLD(MCUCTRL_PWRSW0_PWRSWVDDMDSP0OVERRIDE, 1)   |
                        _VAL2FLD(MCUCTRL_PWRSW0_PWRSWVDDMDSP1DYNSEL, 1)     |
                        _VAL2FLD(MCUCTRL_PWRSW0_PWRSWVDDMDSP1OVERRIDE, 1)   |
                        _VAL2FLD(MCUCTRL_PWRSW0_PWRSWVDDMLDYNSEL, 1)        |
                        _VAL2FLD(MCUCTRL_PWRSW0_PWRSWVDDMLOVERRIDE, 1)      |
                        _VAL2FLD(MCUCTRL_PWRSW0_PWRSWVDDMCPUDYNSEL, 1)      |
                        _VAL2FLD(MCUCTRL_PWRSW0_PWRSWVDDMCPUOVERRIDE, 1)    |
                        _VAL2FLD(MCUCTRL_PWRSW0_PWRSWVDDMDSP0STATSEL, 1)    |
                        _VAL2FLD(MCUCTRL_PWRSW0_PWRSWVDDMDSP1STATSEL, 1);

    // Increases the reference recovery time between scans in LPMODE1 from 5us to 10us.
    MCUCTRL->AUDADCPWRDLY_b.AUDADCPWR1 = 4;

    //
    // Store the factory values for various trims.
    //
    if ( g_bOrigTrimsStored == false )
    {
        g_orig_ACTTRIMVDDF          = MCUCTRL->SIMOBUCK12_b.ACTTRIMVDDF;
        g_orig_MEMLDOACTIVETRIM     = MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM;
        g_orig_LPTRIMVDDF           = MCUCTRL->SIMOBUCK12_b.LPTRIMVDDF;
        g_orig_MEMLPLDOTRIM         = MCUCTRL->LDOREG2_b.MEMLPLDOTRIM;
        g_orig_TVRGVREFTRIM         = MCUCTRL->VREFGEN2_b.TVRGVREFTRIM;
        g_orig_CORELDOACTIVETRIM    = MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM;
        g_bOrigTrimsStored          = true;
    }

#if AM_HAL_PWRCTL_OPTIMIZE_ACTIVE_TRIMS_CRYPTO
    //
    // Initialize the global active trims
    //
    g_i32LatestVddfActTrim = g_orig_ACTTRIMVDDF;
    g_i32LatestLDOActTrim  = g_orig_MEMLDOACTIVETRIM;

    //
    // Get trimrev version
    //
    g_ui32VDDFAdjustCodes = 0;

    //
    // Determine the appropriate trimming adjustments by checking the
    // patch tracking level that has been applied to the device.
    //
    ui32Ret = am_hal_mram_info_read(1, AM_REG_INFO1_PATCH_TRACKER0_O / 4, 1, &ui32PatchTracker);
    if ( ui32Ret == 0 )
    {
        //
        // Determine the boost amount based on the patch level.
        //
        if ( (ui32PatchTracker & (0x3 << 1)) == 0 )
        {
            g_ui32VDDFAdjustCodes = 3;
        }
        else if ( (ui32PatchTracker & (0x1 << 1)) == 0 )
        {
            g_ui32VDDFAdjustCodes = 6;
        }
        else if ( (ui32PatchTracker & (0x1 << 2)) == 0 )
        {
            g_ui32VDDFAdjustCodes = 9;
        }
        else
        {
            g_ui32VDDFAdjustCodes = 0;
        }
    }
#endif // AM_HAL_PWRCTL_OPTIMIZE_ACTIVE_TRIMS_CRYPTO

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_pwrctrl_low_power_init()

void
buck_ldo_override_init(void)
{
    //
    // Force SIMOBUCK into active mode. SIMOBUCKOVER must be set last.
    // This override, even though enabled - is not effective till we go to DeepSleep
    //
    MCUCTRL->VRCTRL_b.SIMOBUCKPDNB   = 1;
    MCUCTRL->VRCTRL_b.SIMOBUCKRSTB   = 1;
    MCUCTRL->VRCTRL_b.SIMOBUCKACTIVE = 1;
    MCUCTRL->VRCTRL_b.SIMOBUCKOVER   = 1;

#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
    //
    // Force LDOs into active mode and to run in parallel with SIMO.
    //
    //
    // Core LDO. Set CORELDOOVER last
    //
    MCUCTRL->VRCTRL_b.CORELDOCOLDSTARTEN  = 0;
    MCUCTRL->VRCTRL |=
        MCUCTRL_VRCTRL_CORELDOACTIVE_Msk        |
        MCUCTRL_VRCTRL_CORELDOACTIVEEARLY_Msk   |
        MCUCTRL_VRCTRL_CORELDOPDNB_Msk;
    MCUCTRL->VRCTRL_b.CORELDOOVER         = 1;
    //
    // Mem LDO. Set MEMLDOOVER last
    //
    MCUCTRL->VRCTRL_b.MEMLDOCOLDSTARTEN   = 0;
    MCUCTRL->VRCTRL |=
        MCUCTRL_VRCTRL_MEMLDOACTIVE_Msk         |
        MCUCTRL_VRCTRL_MEMLDOACTIVEEARLY_Msk    |
        MCUCTRL_VRCTRL_MEMLDOPDNB_Msk;
    MCUCTRL->VRCTRL_b.MEMLDOOVER          = 1;
#endif // AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
} // buck_ldo_override_init()


// Dynamically turn on and off the overrides for buck and LDO
// Override configs are already set once in buck_ldo_override_init
void
buck_ldo_update_override(bool bEnable)
{
    MCUCTRL->VRCTRL_b.SIMOBUCKOVER   = bEnable;
#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
    MCUCTRL->VRCTRL_b.CORELDOOVER    = bEnable;
    MCUCTRL->VRCTRL_b.MEMLDOOVER     = bEnable;
#endif // AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
} // buck_ldo_update_override()

// ****************************************************************************
//
//  am_hal_pwrctrl_control()
//  Additional miscellaneous power controls.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_control(am_hal_pwrctrl_control_e eControl, void *pArgs)
{
    uint32_t ui32ReturnStatus = AM_HAL_STATUS_SUCCESS;
    uint32_t ui32TrimVer = 0;
    switch ( eControl )
    {
        case AM_HAL_PWRCTRL_CONTROL_SIMOBUCK_INIT:

            TrimVersionGet(&ui32TrimVer);

            //
            // Apply specific trim optimization for SIMOBUCK.
            //
            if ( ui32TrimVer < TRIMREV_PCM )
            {
                //
                // Set zero crossing for comparator to best value.
                //
                MCUCTRL->SIMOBUCK15_b.ZXCOMPOFFSETTRIM  = 0;
                MCUCTRL->SIMOBUCK7_b.ZXCOMPZXTRIM       = 0;

                //
                // Set Set VDDC active low and high TON trim.
                //
                MCUCTRL->SIMOBUCK2_b.VDDCACTLOWTONTRIM  = 0xA;
                MCUCTRL->SIMOBUCK2_b.VDDCACTHIGHTONTRIM = 0xA;

                //
                // Set VDDF active low and high TON trim.
                //
                MCUCTRL->SIMOBUCK7_b.VDDFACTLOWTONTRIM  = 0xF;
                MCUCTRL->SIMOBUCK6_b.VDDFACTHIGHTONTRIM = 0xF;

                //
                // Set VDDS active low and high TON trim.
                //
                MCUCTRL->SIMOBUCK9_b.VDDSACTLOWTONTRIM  = 0xF;
                MCUCTRL->SIMOBUCK9_b.VDDSACTHIGHTONTRIM = 0xF;
            }

            //
            // Update VDDF LP trims
            //
            MCUCTRL->SIMOBUCK3_b.VDDCLPLOWTONTRIM  = 0xA;
            MCUCTRL->SIMOBUCK3_b.VDDCLPHIGHTONTRIM = 0xA;
            MCUCTRL->SIMOBUCK8_b.VDDFLPLOWTONTRIM  = 0xF;
            MCUCTRL->SIMOBUCK8_b.VDDFLPHIGHTONTRIM = 0xF;

#if AM_HAL_PWRCTL_SHORT_VDDF_TO_VDDS
            //
            // Enable VDDF to VDDS short to increase load cap (2.2uF + 2.2uF).
            //
            MCUCTRL->PWRSW1_b.SHORTVDDFVDDSORVAL  = 1;
            MCUCTRL->PWRSW1_b.SHORTVDDFVDDSOREN   = 1;

            g_ui32origSimobuckVDDStrim = MCUCTRL->SIMOBUCK13_b.ACTTRIMVDDS;
            MCUCTRL->SIMOBUCK13_b.ACTTRIMVDDS = 0;    // VDDS trim level to 0
#endif // AM_HAL_PWRCTL_SHORT_VDDF_TO_VDDS

            //
            // Enable VDDC, VDDF, and VDDS.
            //
            MCUCTRL->SIMOBUCK0  =   _VAL2FLD(MCUCTRL_SIMOBUCK0_VDDCRXCOMPEN, 1) |
                                    _VAL2FLD(MCUCTRL_SIMOBUCK0_VDDSRXCOMPEN, 1) |
                                    _VAL2FLD(MCUCTRL_SIMOBUCK0_VDDFRXCOMPEN, 1);

            if ( ui32TrimVer < TRIMREV_PCM )
            {
                //
                // Set SIMOBUCK clock.
                //
                MCUCTRL->SIMOBUCK1_b.TONCLKTRIM       = 0;
                MCUCTRL->SIMOBUCK1_b.RXCLKACTTRIM     = 1;
            }
#if AM_HAL_PWRCTRL_CORE_PWR_OPTIMAL_EFFICIENCY
            if ( ui32TrimVer >= TRIMREV_PWRCTRL )
            {
                //
                // Connect MCU core to VDDC_LV for increased power efficiency.
                //
                // Enable the VDDC_LV rail
                //
                MCUCTRL->SIMOBUCK0 =
                    MCUCTRL_SIMOBUCK0_VDDCLVRXCOMPEN_Msk    |   // VDDC LV rail
                    MCUCTRL_SIMOBUCK0_VDDSRXCOMPEN_Msk      |   // VDDS rail
                    MCUCTRL_SIMOBUCK0_VDDFRXCOMPEN_Msk      |   // VDDF rail
                    MCUCTRL_SIMOBUCK0_VDDCRXCOMPEN_Msk;         // VDDC rail

                //
                // Connect the MCU core to run from the VDDC_LV rail in order
                // to reduce power consumption when active.
                //
                MCUCTRL->D2ASPARE &=
                    ~(MCUCTRL_D2ASPARE_VDDCPUOVERRIDE_Msk   |
                      MCUCTRL_D2ASPARE_VDDCAOROVERRIDE_Msk);
            }
#endif // AM_HAL_PWRCTRL_CORE_PWR_OPTIMAL_EFFICIENCY

#ifdef AM_HAL_PWRCTL_SHORT_VDDC_TO_VDDCLV
        //
        // This keeps the VDDC_LV cap from charging and discharging
        //
        MCUCTRL->PWRSW1_b.SHORTVDDCVDDCLVOREN  = 1; //<! bit 28
        MCUCTRL->PWRSW1_b.SHORTVDDCVDDCLVORVAL = 1; //<! bit 29
#endif

            //
            // Enable the SIMOBUCK
            //
            PWRCTRL->VRCTRL_b.SIMOBUCKEN = 1;

            //
            // Allow dynamic SIMOBUCK trim adjustments
            //
            MCUCTRL->SIMOBUCK15_b.TRIMLATCHOVER = 1;

            // Initialize the Buck and LDO override settings, and enable overrides
            buck_ldo_override_init();
            break;

        case AM_HAL_PWRCTRL_CONTROL_CRYPTO_POWERDOWN:
            {
                uint32_t    ui32Status;
                bool        bEnabled;

                //
                // Check if CRYPTO block is powered on.
                //
                bEnabled = false;
                am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_CRYPTO, &bEnabled);
                if ( bEnabled )
                {
                    //
                    // Power down the crypto block in case it was left on by SBR/SBL.
                    //
                    ui32Status = am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);
                    if (AM_HAL_STATUS_SUCCESS != ui32Status)
                    {
                        return ui32Status;
                    }
                }
            }
            break;

        case AM_HAL_PWRCTRL_CONTROL_XTAL_PWDN_DEEPSLEEP:
            //
            // This optimization is optional. Enable it IFF the 32KHz crystal is
            // not required during deep sleep. If enabled it will save ~0.8uA.
            //
            MCUCTRL->XTALGENCTRL_b.XTALBIASTRIM   = 0x20;

            MCUCTRL->XTALCTRL =
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALICOMPTRIM,  0 )                                       |
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALIBUFTRIM,   0 )                                       |
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOMPPDNB,   MCUCTRL_XTALCTRL_XTALCOMPPDNB_PWRDNCOMP ) |
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALPDNB,       MCUCTRL_XTALCTRL_XTALPDNB_PWRDNCORE )     |
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOMPBYPASS, MCUCTRL_XTALCTRL_XTALCOMPBYPASS_USECOMP ) |
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOREDISFB,  MCUCTRL_XTALCTRL_XTALCOREDISFB_EN )       |
                _VAL2FLD(MCUCTRL_XTALCTRL_XTALSWE,        MCUCTRL_XTALCTRL_XTALSWE_OVERRIDE_EN);
            break;

        case AM_HAL_PWRCTRL_CONTROL_DIS_PERIPHS_ALL:
            PWRCTRL->DEVPWREN =
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENDBG,     PWRCTRL_DEVPWREN_PWRENDBG_DIS)          |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUSBPHY,  PWRCTRL_DEVPWREN_PWRENUSBPHY_DIS)       |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUSB,     PWRCTRL_DEVPWREN_PWRENUSB_DIS)          |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENSDIO,    PWRCTRL_DEVPWREN_PWRENSDIO_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENCRYPTO,  PWRCTRL_DEVPWREN_PWRENCRYPTO_DIS)       |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENDISPPHY, PWRCTRL_DEVPWREN_PWRENDISPPHY_DIS)      |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENDISP,    PWRCTRL_DEVPWREN_PWRENDISP_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENGFX,     PWRCTRL_DEVPWREN_PWRENGFX_DIS)          |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI2,   PWRCTRL_DEVPWREN_PWRENMSPI2_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI1,   PWRCTRL_DEVPWREN_PWRENMSPI1_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENMSPI0,   PWRCTRL_DEVPWREN_PWRENMSPI0_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENADC,     PWRCTRL_DEVPWREN_PWRENADC_DIS)          |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART3,   PWRCTRL_DEVPWREN_PWRENUART3_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART2,   PWRCTRL_DEVPWREN_PWRENUART2_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART1,   PWRCTRL_DEVPWREN_PWRENUART1_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENUART0,   PWRCTRL_DEVPWREN_PWRENUART0_DIS)        |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM7,    PWRCTRL_DEVPWREN_PWRENIOM7_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM6,    PWRCTRL_DEVPWREN_PWRENIOM6_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM5,    PWRCTRL_DEVPWREN_PWRENIOM5_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM4,    PWRCTRL_DEVPWREN_PWRENIOM4_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM3,    PWRCTRL_DEVPWREN_PWRENIOM3_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM2,    PWRCTRL_DEVPWREN_PWRENIOM2_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM1,    PWRCTRL_DEVPWREN_PWRENIOM1_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOM0,    PWRCTRL_DEVPWREN_PWRENIOM0_DIS)         |
                _VAL2FLD(PWRCTRL_DEVPWREN_PWRENIOS,     PWRCTRL_DEVPWREN_PWRENIOS_DIS);
            break;

#if AM_HAL_TEMPCO_LP
        case AM_HAL_PWRCTRL_CONTROL_TEMPCO_GETMEASTEMP:
            if ( pArgs )
            {
                *((float*)pArgs) = g_pfTempMeasured;
            }
#endif // AM_HAL_TEMPCO_LP

        default:
            ui32ReturnStatus = AM_HAL_STATUS_INVALID_ARG;
            break;
    }

    //
    // Return success status.
    //
    return ui32ReturnStatus;

} // am_hal_pwrctrl_control()

// ****************************************************************************
// Function to restore factory trims.
// ****************************************************************************
static void
restore_factory_trims(void)
{
    if ( g_bOrigTrimsStored )
    {
        //
        // Restore the original factory trim values
        //
        MCUCTRL->SIMOBUCK12_b.ACTTRIMVDDF    = g_orig_ACTTRIMVDDF;
        MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM  = g_orig_MEMLDOACTIVETRIM;
        MCUCTRL->SIMOBUCK12_b.LPTRIMVDDF     = g_orig_LPTRIMVDDF;
        MCUCTRL->LDOREG2_b.MEMLPLDOTRIM      = g_orig_MEMLPLDOTRIM;
        MCUCTRL->VREFGEN2_b.TVRGVREFTRIM     = g_orig_TVRGVREFTRIM;
        MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM = g_orig_CORELDOACTIVETRIM;
    }
} // restore_factory_trims()

//*****************************************************************************
//
// Restore original power settings
//
// This function restores default power trims, reverting relative changes that
// were done as part of am_hal_pwrctrl_low_power_init, SIMOBUCK init, and
// dynamic updates such as are made with Temperature Compensation (TempCo)
// and/or by enabling Crypto.
//
// Important:
// - This function must be called before transition to a new application, such
//   as the case of a secondary bootloader transistioning to an application.
// - If previously enabled, TempCo must be disabled before this function is
//   called.
//
// - This function switches from SIMOBUCK to LDO which is known to affect
//   VDDC and VDDC_LV
//   Please see AM_HAL_PWRCTL_SHORT_VDDC_TO_VDDCLV in am_hal_pwrctrl.h.
//
//*****************************************************************************
uint32_t
am_hal_pwrctrl_settings_restore(void)
{
    uint32_t ui32Ret;

    //
    // Ensure - we're in LP mode
    //
    if (PWRCTRL->MCUPERFREQ_b.MCUPERFREQ != AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER)
    {
        // Device needs to be in LP mode before restore is called
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    //
    // Need to revert the trim changes, and turn to LDO mode
    //
    AM_CRITICAL_BEGIN

    //
    // Switch to LDO mode (if not already in LDO mode)
    // Delay 20us for rails to settle
    //
    if (PWRCTRL->VRSTATUS_b.SIMOBUCKST == PWRCTRL_VRSTATUS_SIMOBUCKST_ACT)
    {
#if AM_HAL_PWRCTL_SHORT_VDDF_TO_VDDS
        if (g_ui32origSimobuckVDDStrim != 0xFFFFFFFF)
        {
            MCUCTRL->SIMOBUCK13_b.ACTTRIMVDDS = g_ui32origSimobuckVDDStrim;
        }

        //
        // Remove VDDS/VDDF short
        //
        MCUCTRL->PWRSW1_b.SHORTVDDFVDDSORVAL  = 0;
        MCUCTRL->PWRSW1_b.SHORTVDDFVDDSOREN   = 0;
#endif

        PWRCTRL->VRCTRL_b.SIMOBUCKEN = 0;
        // Need to remove overrides
        buck_ldo_update_override(false);
    }

    //
    // Re-enable Crypto if not already on.
    // The enable function will check whether it's enabled or not.
    //
    ui32Ret = am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);

    //
    // Restore original factory trims.
    // This will apply whether or not AM_HAL_TEMPCO_LP is activated.
    //
    restore_factory_trims();

    AM_CRITICAL_END

    return ui32Ret;

} // am_hal_pwrctrl_settings_restore()

#if AM_HAL_TEMPCO_LP
// ****************************************************************************
//
//  am_hal_pwrctrl_tempco_init()
//  ui32UpdateInterval - Time in seconds. 10 is recommended.
//
//  User must also call am_hal_adc_initialize().
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_tempco_init(void *pADCHandle,
                           uint32_t ui32ADCslot)
{
    uint32_t                    ui32Retval;
    am_hal_adc_slot_config_t    sSlotCfg;
    uint32_t ui32TrimVer;

    uint32_t ui32Temp[3];

    //
    // First make sure the temperature calibration is valid.
    // am_hal_mram_info_read(INFOn, word_offset, num_wds, variable)
    //
    ui32Retval = am_hal_mram_info_read(1, AM_REG_INFO1_TEMP_CAL_ATE_O / 4, 3, &ui32Temp[0]);
    if ( (ui32Retval != 0)           || (ui32Temp[0] == 0xFFFFFFFF) ||
         (ui32Temp[1] == 0xFFFFFFFF) || (ui32Temp[2] == 0xFFFFFFFF) )
    {
        //
        // Invalidate the application of TempCo.
        //
        g_bTempcoValid = false;
        return AM_HAL_STATUS_HW_ERR;
    }

    //
    // Make sure this device can reliably support TempCo
    //
    TrimVersionGet(&ui32TrimVer);
    if ( ui32TrimVer < TRIMREV_PWRCTRL )
    {
        //
        // Invalidate that original trims have been saved, which
        // invalidates the application of TempCo.
        //
        //
        // Invalidate the application of TempCo.
        //
        g_bTempcoValid = false;
        return AM_HAL_STATUS_FAIL;
    }
    else
    {
        g_bTempcoValid = true;
    }

    //
    // Save the ADC handle and the slot number.
    //
    g_TempcoADCHandle   = pADCHandle;
    g_ui32TempcoADCslot = ui32ADCslot;

    //
    // At this point the ADC is expected to be initialized, powered, and configured.
    // Configure the temperature slot.
    //
    sSlotCfg.eMeasToAvg     = AM_HAL_ADC_SLOT_AVG_1;
    sSlotCfg.ui32TrkCyc     = 32;
    sSlotCfg.ePrecisionMode = AM_HAL_ADC_SLOT_12BIT;
    sSlotCfg.eChannel       = AM_HAL_ADC_SLOT_CHSEL_TEMP;
    sSlotCfg.bWindowCompare = false;
    sSlotCfg.bEnabled       = true;
    ui32Retval = am_hal_adc_configure_slot(g_TempcoADCHandle, g_ui32TempcoADCslot, &sSlotCfg);
    if ( ui32Retval != AM_HAL_STATUS_SUCCESS )
    {
       return ui32Retval;
    }

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_pwrctrl_tempco_init()

//
// TempCo trims lookup tables
// The VDD table is arranged row-by-row with 3 values as:
//  0: Min temperature
//  1: Max temperature
//  2: Trim code adjust
//

const static int8_t
g_VDDC_trimstbl[][3] =
{
    { -20, 60,    0},
    {  60, 90,  -16},   // Last actual table entry
    { 127, 127,   0}    // End of table: Probably bogus temp, do no adjustment
};

//
//! Trim adjust table for VDDF
//
const static int8_t
g_VDDF_trimstbl[][3] =
{
    { -20, -11,   0},
    { -11,  -2,  -1},
    {  -2,   8,  -2},
    {   8,  17,  -3},
    {  17,  26,  -4},
    {  26,  35,  -5},
    {  35,  44,  -6},
    {  44,  53,  -7},
    {  53,  60,  -8},
    {  60,  90,  -9},   // Last actual table entry
    { 127, 127,   0}    // End of table: Probably bogus temp, do no adjustment
};

//
//! Trim adjust table for VDDFLP
//
const static int8_t
g_VDDFLP_trimstbl[][3] =
{
    { -20, -11,   0},
    { -11,  -2,  -1},
    {  -2,   8,  -2},
    {   8,  17,  -3},
    {  17,  26,  -4},
    {  26,  35,  -5},
    {  35,  44,  -6},
    {  44,  53,  -7},
    {  53,  60,  -8},
    {  60,  90,  -9},   // Last actual table entry
    { 127, 127,   0}    // End of table: Probably bogus temp, do no adjustment
};

//
//! Trim adjust table for memlpldo
//
const static int8_t
g_memlpldo_trimstbl[][3] =
{
    { -18, -14,   8},
    { -14, -10,   7},
    { -10,  -6,   6},
    {  -6,  -2,   5},
    {  -2,   2,   4},
    {   2,   6,   3},
    {   6,  10,   2},
    {  10,  14,   1},
    {  14,  18,   0},
    {  18,  22,  -1},
    {  22,  26,  -2},
    {  26,  42,  -3},
    {  42,  60,  -4},
    {  60,  90,  -5},   // Last actual table entry
    { 127, 127,   0}    // End of table: Probably bogus temp, do no adjustment
};

//
//! Helper macro to round a float down.
//
#define FTOI_RNDDN(fval)        ( (fval < 0.00F) ? (int)fval - 1 : (int)fval )

// ****************************************************************************
// Function to lookup a trim given a temperature and a pointer
// to the appropriate lookup table.
// ****************************************************************************
static int8_t
lookup_trim(int8_t i8Temp, const int8_t pi8Tbl[][3])
{
    uint32_t ux;
    if ( i8Temp < pi8Tbl[0][0] )
    {
        //
        // Return the trim for the lowest temperature in the table
        //
        return pi8Tbl[0][2];
    }
    else
    {
        //
        // Lookup the trim
        //
        ux = 0;
        while ( pi8Tbl[ux][0] < 127 )
        {
            if ( i8Temp <= pi8Tbl[ux][1] )
            {
                return pi8Tbl[ux][2];
            }
            ux++;
        }

        //
        // The temperature is very high, so snap to the default trims
        ///
        return 0;
    }

} // lookup_trim()

// ****************************************************************************
// Function to validate and apply trim changes.
// ****************************************************************************
static void
tempco_set_trims(int32_t i32VDDFtrim,
                 int32_t i32VDDFLPtrim,
                 int32_t i32MemlpLDOtrim,
                 int32_t i32VDDCtrim)
{
    int32_t i32SimoVDDFact, i32Memldoact, i32SimoVDDFlp, i32Memlpldo, i32TvrgVref, i32CoreMemldoact;

    if ( !g_bOrigTrimsStored )
    {
        return;
    }

    i32SimoVDDFact   = g_orig_ACTTRIMVDDF       + i32VDDFtrim;
    i32Memldoact     = g_orig_MEMLDOACTIVETRIM  + i32VDDFtrim;
    i32SimoVDDFlp    = g_orig_LPTRIMVDDF        + i32VDDFLPtrim;
    i32Memlpldo      = g_orig_MEMLPLDOTRIM      + i32MemlpLDOtrim;
    i32TvrgVref      = g_orig_TVRGVREFTRIM      + i32VDDCtrim;
    i32CoreMemldoact = g_orig_CORELDOACTIVETRIM + i32VDDCtrim;

    //
    // If Crypto is currently off, adjust the VDDF active trim.
    //
    AM_CRITICAL_BEGIN

#if AM_HAL_PWRCTL_OPTIMIZE_ACTIVE_TRIMS_CRYPTO
    if ( !g_bBoostForCryptoApplied )
    {
        //
        // When Crypto is off, subtract from the factory codes for VDDF active
        // (both simobuck and memldo) to save power.
        //
        i32SimoVDDFact -= g_ui32VDDFAdjustCodes;
        i32Memldoact   -= g_ui32VDDFAdjustCodes;
    }

    //
    // Save the trim values
    //
    g_i32LatestVddfActTrim = i32SimoVDDFact;
    g_i32LatestLDOActTrim  = i32Memldoact;
#endif // AM_HAL_PWRCTL_OPTIMIZE_ACTIVE_TRIMS_CRYPTO

    if ( i32SimoVDDFact < 0 )
    {
        i32SimoVDDFact = 0;
    }
    else if ( i32SimoVDDFact > MAX_ACTTRIMVDDF )
    {
        i32SimoVDDFact = MAX_ACTTRIMVDDF;
    }

    if ( i32Memldoact < 0 )
    {
        i32Memldoact = 0;
    }
    else if ( i32Memldoact > MAX_MEMLDOACTIVETRIM )
    {
        i32Memldoact = MAX_MEMLDOACTIVETRIM;
    }

    if ( i32SimoVDDFlp < 0 )
    {
        i32SimoVDDFlp = 0;
    }
    else if ( i32SimoVDDFlp > MAX_LPTRIMVDDF )
    {
        i32SimoVDDFlp = MAX_LPTRIMVDDF;
    }

    if ( i32Memlpldo  < 0 )
    {
        i32Memlpldo  = 0;
    }
    else if ( i32Memlpldo > MAX_MEMLPLDOTRIM )
    {
        i32Memlpldo = MAX_MEMLPLDOTRIM;
    }

    if ( i32TvrgVref  < 0 )
    {
        i32TvrgVref  = 0;
    }
    else if ( i32TvrgVref > MAX_TVRGVREFTRIM )
    {
        i32TvrgVref = MAX_TVRGVREFTRIM;
    }

    if ( i32CoreMemldoact  < 0 )
    {
        i32CoreMemldoact  = 0;
    }
    else if ( i32CoreMemldoact > MAX_CORELDOACTIVETRIM )
    {
        i32CoreMemldoact = MAX_CORELDOACTIVETRIM;
    }

    //
    // Now set the new values
    //
    MCUCTRL->SIMOBUCK12_b.ACTTRIMVDDF    = i32SimoVDDFact;
    MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM  = i32Memldoact;
    MCUCTRL->SIMOBUCK12_b.LPTRIMVDDF     = i32SimoVDDFlp;
    MCUCTRL->LDOREG2_b.MEMLPLDOTRIM      = i32Memlpldo;
    MCUCTRL->VREFGEN2_b.TVRGVREFTRIM     = i32TvrgVref;
    MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM = i32CoreMemldoact;

    AM_CRITICAL_END
} // tempco_set_trims()

// ****************************************************************************
// Validate samples from temperature sensor, and average them.
// ****************************************************************************
static uint32_t
adc_get_temperature_volts(float *pfADCTempVolts,
                          uint32_t ui32NumSamples,
                          am_hal_adc_sample_t sSample[])
{
    uint32_t ux;
    float fSum;

    //
    // ui32NumSamples temperature samples have been obtained.
    // Make sure at least 2 of the samples are slightly different.
    //
    ux = 0;
    while ( ux < (ui32NumSamples - 1) )
    {
        if ( sSample[ux].ui32Sample != sSample[ux + 1].ui32Sample )
        {
            break;
        }
        ux++;
    }

    if ( ux == (ui32NumSamples - 1) )
    {
        //
        // This sample is not reliable, return with error.
        //
        return AM_HAL_STATUS_FAIL;
    }

    //
    // The measured temperature can be considered reliable.
    // Get an average of the temperature.
    //
    ux = 0;
    fSum = 0.0F;
    while ( ux < ui32NumSamples )
    {
        //
        // Convert and scale the temperature sample into its corresponding voltage.
        //
        g_ui16TempcoTEMP_code = AM_HAL_ADC_FIFO_SAMPLE(sSample[ux].ui32Sample);
        fSum += (float)g_ui16TempcoTEMP_code * AM_HAL_ADC_VREF / 4096.0f; // 12-bit sample
        ux++;
    }

    *pfADCTempVolts = fSum / ((float)ui32NumSamples);

    return AM_HAL_STATUS_SUCCESS;

} // adc_get_temperature_volts()

// ****************************************************************************
//
//  am_hal_pwrctrl_tempco_sample_handler()
//
//  This function to be called with a minimum periodicity as recommended
//  by Ambiq engineering.
//
// ****************************************************************************
uint32_t
am_hal_pwrctrl_tempco_sample_handler(uint32_t ui32NumSamples, am_hal_adc_sample_t sSamples[])
{
    uint32_t ui32Retval;
    float fVT[3];
    float fADCTempVolts, fADCTempDegreesC;
    int32_t i32VDDFtrim, i32VDDFLPtrim, i32MemlpLDOtrim, i32VDDCtrim;

    if ( (g_bTempcoValid == false) || (ui32NumSamples < AM_HAL_TEMPCO_NUMSAMPLES) )
    {
        return AM_HAL_STATUS_FAIL;
    }

    //
    // Query the ADC for the current temperature of the chip.
    //
    ui32Retval = adc_get_temperature_volts(&fADCTempVolts, ui32NumSamples, sSamples);
    if ( ui32Retval != AM_HAL_STATUS_SUCCESS )
    {
        //
        // This sample is not reliable.
        // Snap to the highest trim code settings from the tables and return.
        //
        tempco_set_trims(g_VDDF_trimstbl[0][2],
                         g_VDDFLP_trimstbl[0][2],
                         g_memlpldo_trimstbl[0][2],
                         g_VDDC_trimstbl[0][2]);
        return ui32Retval;
    }

    //
    // Now call the HAL routine to convert volts to degrees Celsius.
    //
    fVT[0] = fADCTempVolts;
    fVT[1] = 0.0f;
    fVT[2] = -123.456;
    ui32Retval = am_hal_adc_control(g_TempcoADCHandle, AM_HAL_ADC_REQ_TEMP_CELSIUS_GET, fVT);
    if ( ui32Retval == AM_HAL_STATUS_SUCCESS )
    {
        fADCTempDegreesC = fVT[1];  // Get the temperature
        g_pfTempMeasured = fADCTempDegreesC;
    }
    else
    {
        //
        // Error, restore default values.
        //
        tempco_set_trims(0, 0, 0, 0);
        g_pfTempMeasured = 0.0F;
        return ui32Retval;
    }

    //
    // The temperature in degC is stored in fADCTempDegreesC
    //
    int8_t  i8Temp;
    int32_t i32Temp;
    i32Temp = FTOI_RNDDN(fADCTempDegreesC);

    //
    // Get the integer value of the temperature.
    // Allow for the temperature sensor specified at +-3C.
    //
    i8Temp = (int8_t)(i32Temp - 3);

    //
    // Look up the 3 trim adjustments.
    //
    i32VDDFtrim      = (int32_t)lookup_trim(i8Temp, g_VDDF_trimstbl);
    i32VDDFLPtrim    = (int32_t)lookup_trim(i8Temp, g_VDDFLP_trimstbl);
    i32MemlpLDOtrim  = (int32_t)lookup_trim(i8Temp, g_memlpldo_trimstbl);
    i32VDDCtrim      = (int32_t)lookup_trim(i8Temp, g_VDDC_trimstbl);

    //
    // Now, set the trims appropriately.
    //
    tempco_set_trims(i32VDDFtrim, i32VDDFLPtrim, i32MemlpLDOtrim, i32VDDCtrim);

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_pwrctrl_tempco_sample_handler()

#endif // AM_HAL_TEMPCO_LP

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
