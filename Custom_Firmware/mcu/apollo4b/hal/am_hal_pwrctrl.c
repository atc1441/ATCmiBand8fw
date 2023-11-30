//*****************************************************************************
//
//! @file am_hal_pwrctrl.c
//!
//! @brief Functions for enabling and disabling power domains.
//!
//! @addtogroup pwrctrl4_4b PWRCTRL - Power Control
//! @ingroup apollo4b_hal
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
// Define max values of some particular fields
//
#define MAX_BUCKVDDFTRIM    _FLD2VAL(MCUCTRL_SIMOBUCK12_ACTTRIMVDDF, MCUCTRL_SIMOBUCK12_ACTTRIMVDDF_Msk)
#define MAX_LDOVDDFTRIM     _FLD2VAL(MCUCTRL_LDOREG2_MEMLDOACTIVETRIM, MCUCTRL_LDOREG2_MEMLDOACTIVETRIM_Msk)
#define MAX_BUCKVDDCTRIM    _FLD2VAL(MCUCTRL_VREFGEN2_TVRGVREFTRIM, MCUCTRL_VREFGEN2_TVRGVREFTRIM_Msk)
#define MAX_LDOVDDCTRIM     _FLD2VAL(MCUCTRL_LDOREG1_CORELDOACTIVETRIM, MCUCTRL_LDOREG1_CORELDOACTIVETRIM_Msk)

//
// Define the number of steps needed to boost the simobuck VDDF trims
// for PCM or pre-PCM parts.
// (Note that "PCM" is simply a trim level version of product test.)
//
#define TRIMREV_PCM                             6   // Trim revision number for PCM
#define BUCK_VDDF_BOOST_STEPS_PCM               7   // PCM parts
#define BUCK_VDDF_BOOST_STEPS_PREPCM            3   // Pre-PCM parts

#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
#define BUCK_VDDF_BOOST_STEPS_PARALLEL_PCM      3
#define BUCK_VDDF_BOOST_STEPS_PARALLEL_PREPCM   0
#endif // AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL

//
// Define min values of BUCK and LDO trim fields
//
#define MIN_LDOVDDFTRIM         1
#define MIN_LDOVDDCTRIM         1
#define MIN_BUCKVDDFTRIM        1
#define MIN_BUCKVDDCTRIM        1

#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
//
// Define parameters for LDO operation in parallel
//
#define TRIM_STEP                           1
#define BUCK_PULSE_COUNT_DELAY_US           50

#define CORELDO_STEPDOWN_TRIMCODE           12  // Core LDO to be kept 30mV below buck VDDC
#define MEMLDO_STEPDOWN_TRIMCODE            3   // Mem  LDO to be kept 35mV below buck VDDC
#endif // AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL

//
// VDDC boost to use, based on differing conditions.
//
#define SIMOBUCK_VDDC_BOOST_PCM_LT_240  43
#define SIMOBUCK_VDDC_BOOST_PCM_GE_240  23
#define SIMOBUCK_VDDC_BOOST_PARALLEL    22

#define AM_DONT_CARE        0                   // Define a "don't care" parameter

//
// Internal non-published function, since Timer13 is reserved for a workaround.
//
extern uint32_t internal_timer_config(uint32_t ui32TimerNumber,
                                      am_hal_timer_config_t *psTimerConfig);


//****************************************************************************
//
// Global variables.
//
//****************************************************************************
//
// Global State Variables for the VDDF and VDDC boosting
//
am_hal_pwrctrl_mcu_mode_e g_eCurrPwrMode = AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER;
uint32_t g_ui32TrimVer              = 0xFFFFFFFF;
uint32_t g_ui32TrimLVT              = 0xFFFFFFFF;

uint32_t g_ui32origSimobuckVDDFtrim = 0xFFFFFFFF;
uint32_t g_ui32origSimobuckVDDCtrim = 0xFFFFFFFF;
uint32_t g_ui32origLDOVDDCtrim      = 0xFFFFFFFF;
uint32_t g_ui32origMEMLDOActiveTrim = 0xFFFFFFFF;
uint32_t g_ui32origSimobuckVDDStrim = 0xFFFFFFFF;
bool     g_bVDDCbuckboosted         = false;
bool     g_bVDDCLDOboosted          = false;

#define ORIGTRIMREGSVALID   0xCD735A03      // Arbitrary random signature
struct trim_regs_s g_trim_reg_origvals = {0};

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
//
//****************************************************************************

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

//**********************************************
//
//! Define the peripheral control structure.
//
//**********************************************
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
//! @name Default configurations.
//! @{
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
//! @}

//****************************************************************************
//
//  preserve_orig_reg_value() - Saves the SIMOBUCK registers on the first call.
//
//****************************************************************************
static void
preserve_orig_reg_values(void)
{
#if 1
    //
    // STATIC_ASSERT will do a static (compile-time) check of a sizeof()
    // operation without creating any code. If the structure should change
    // from the current 21 members, a compile error will occur.
    //
    STATIC_ASSERT(sizeof(struct trim_regs_s) != (21 * 4));
#endif

    if ( g_trim_reg_origvals.ui32Valid == ORIGTRIMREGSVALID )
    {
        return;
    }

    g_trim_reg_origvals.ui32SB0             = MCUCTRL->SIMOBUCK0;
    g_trim_reg_origvals.ui32SB1             = MCUCTRL->SIMOBUCK1;
    g_trim_reg_origvals.ui32SB2             = MCUCTRL->SIMOBUCK2;
    g_trim_reg_origvals.ui32SB4             = MCUCTRL->SIMOBUCK4;
    g_trim_reg_origvals.ui32SB6             = MCUCTRL->SIMOBUCK6;
    g_trim_reg_origvals.ui32SB7             = MCUCTRL->SIMOBUCK7;
    g_trim_reg_origvals.ui32SB9             = MCUCTRL->SIMOBUCK9;
    g_trim_reg_origvals.ui32SB12            = MCUCTRL->SIMOBUCK12;
    g_trim_reg_origvals.ui32SB13            = MCUCTRL->SIMOBUCK13;
    g_trim_reg_origvals.ui32SB15            = MCUCTRL->SIMOBUCK15;

    g_trim_reg_origvals.ui32LDOREG1         = MCUCTRL->LDOREG1;
    g_trim_reg_origvals.ui32LDOREG2         = MCUCTRL->LDOREG2;
    g_trim_reg_origvals.ui32VREFGEN2        = MCUCTRL->VREFGEN2;

    g_trim_reg_origvals.ui32PWRSW0          = MCUCTRL->PWRSW0;
    g_trim_reg_origvals.ui32PWRSW1          = MCUCTRL->PWRSW1;
    g_trim_reg_origvals.ui32VRCTRL          = MCUCTRL->VRCTRL;
    g_trim_reg_origvals.ui32ACRG            = MCUCTRL->ACRG;
    g_trim_reg_origvals.ui32XTALGENCTRL     = MCUCTRL->XTALGENCTRL;
    g_trim_reg_origvals.ui32ADCPWRCTRL      = MCUCTRL->ADCPWRCTRL;
    g_trim_reg_origvals.ui32AUDADCPWRCTRL   = MCUCTRL->AUDADCPWRCTRL;

    g_trim_reg_origvals.ui32Valid = ORIGTRIMREGSVALID;

} // preserve_orig_reg_values()

//****************************************************************************
//
//  crypto_quiesce
//  Select the MCU power mode.
//
//****************************************************************************
#define CRYPTO_WAIT_USEC        100
static
uint32_t crypto_quiesce(void)

{
    uint32_t      ui32Status;

    //
    // Wait for crypto block to go idle.
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
    // Wait for OTP to go idle.
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
    // Power down the crypto block.
    //
    CRYPTO->HOSTPOWERDOWN_b.HOSTPOWERDOWN = 1;

    //
    // Wait (just in case).
    //
    am_hal_delay_us(15);

    return AM_HAL_STATUS_SUCCESS;
} // crypto_quiesce()

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

#if AM_HAL_PWRCTL_BOOST_VDDC
//*****************************************************************************
//
// Function to determine the chip's LVT TRIM value.
//
//*****************************************************************************
static uint32_t
TrimLVTGet(uint16_t *pui16LVTtrimHigh, uint16_t *pui16LVTtrimLow)
{
    uint32_t ui32Ret;
    uint16_t ui16LVTtrimHigh;
    uint16_t ui16LVTtrimLow;

    //
    // Get the LVT TRIM value and set the global variable.
    // This only needs to be done and verified once.
    //
    ui32Ret = am_hal_mram_info_read(1, AM_REG_INFO1_LVT_TRIMCODE_O / 4, 1, &g_ui32TrimLVT);
    if ( (ui32Ret != AM_HAL_STATUS_SUCCESS) || (g_ui32TrimLVT == 0xFFFFFFFF) )
    {
        ui16LVTtrimHigh = 0xFFFF;   // Set to max value
        ui16LVTtrimLow  = 0xFFFF;   //  ""
        ui32Ret = AM_HAL_STATUS_FAIL;
    }
    else
    {
        ui16LVTtrimHigh = _FLD2VAL(AM_REG_INFO1_LVT_TRIMCODE_HIGH, g_ui32TrimLVT);
        ui16LVTtrimLow  = _FLD2VAL(AM_REG_INFO1_LVT_TRIMCODE_LOW,  g_ui32TrimLVT);
        ui32Ret = AM_HAL_STATUS_SUCCESS;
    }

    if ( pui16LVTtrimHigh && pui16LVTtrimLow )
    {
        *pui16LVTtrimHigh = ui16LVTtrimHigh;
        *pui16LVTtrimLow  = ui16LVTtrimLow;
        return ui32Ret;
    }
    else
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

} // TrimLVTGet()
#endif // AM_HAL_PWRCTL_BOOST_VDDC

//*****************************************************************************
//
//  VDDF_simobuck_boost(bool bBoostOn)
//      bBoostOn        = true to do the boost,
//                        false to restore the boost to the original value.
//      ui32boostPCM    = Boost value required if post-PCM since different
//                        options require different values.
//      ui32boostPrePCM = Boost value if pre-PCM.
//
//*****************************************************************************
static uint32_t
VDDF_simobuck_boost(bool bBoostOn,
                    uint32_t ui32boostPCM,
                    uint32_t ui32boostPrePCM)
{
    uint32_t ui32booststeps, ui32TrimVer, ui32acttrimvddf;
    uint32_t ui32Trim;
    if ( g_ui32origSimobuckVDDFtrim == 0xFFFFFFFF )
    {
        //
        // Get and save the original value the very first time.
        //
        g_ui32origSimobuckVDDFtrim = MCUCTRL->SIMOBUCK12_b.ACTTRIMVDDF;
    }
    ui32Trim = g_ui32origSimobuckVDDFtrim;

    TrimVersionGet(&ui32TrimVer);

    if ( bBoostOn == false )
    {
#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
        ui32Trim += ( ui32TrimVer >= TRIMREV_PCM ) ?
                            BUCK_VDDF_BOOST_STEPS_PARALLEL_PCM :
                            BUCK_VDDF_BOOST_STEPS_PARALLEL_PREPCM;
#endif
        if ( MCUCTRL->SIMOBUCK12_b.ACTTRIMVDDF != ui32Trim )
        {
            AM_CRITICAL_BEGIN

            //
            // Restore the original SIMOBUCK trim and return
            //
            MCUCTRL->SIMOBUCK15_b.TRIMLATCHOVER = 1;

            MCUCTRL->SIMOBUCK12_b.ACTTRIMVDDF = ui32Trim;

            //
            // Delay to give voltage supply some time to transition to the new level
            //
            am_hal_delay_us(AM_HAL_PWRCTRL_VDDF_BOOST_DELAY);
            AM_CRITICAL_END
        }

        return AM_HAL_STATUS_SUCCESS;

    } //! bBoostOn

    if ( APOLLO4_GE_B1 )
    {
#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
        ui32boostPCM    += BUCK_VDDF_BOOST_STEPS_PARALLEL_PCM;
        ui32boostPrePCM += BUCK_VDDF_BOOST_STEPS_PARALLEL_PREPCM;
#endif
        ui32booststeps = ( ui32TrimVer >= TRIMREV_PCM ) ?
                            ui32boostPCM                :
                            ui32boostPrePCM;

        ui32acttrimvddf = ui32Trim <= (MAX_BUCKVDDFTRIM - ui32booststeps) ?
                            (ui32Trim + ui32booststeps) : MAX_BUCKVDDFTRIM;

        if ( MCUCTRL->SIMOBUCK12_b.ACTTRIMVDDF != ui32acttrimvddf )
        {
            AM_CRITICAL_BEGIN
            //
            // Increase VDDF the appropriate number of steps (or max it out).
            //
            MCUCTRL->SIMOBUCK15_b.TRIMLATCHOVER = 1;

            MCUCTRL->SIMOBUCK12_b.ACTTRIMVDDF = ui32acttrimvddf;

            //
            // Delay to give voltage supply some time to transition to the new level
            //
            am_hal_delay_us(AM_HAL_PWRCTRL_VDDF_BOOST_DELAY);
            AM_CRITICAL_END
        }
    }

    return AM_HAL_STATUS_SUCCESS;

} // VDDF_simobuck_boost()

#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
//*****************************************************************************
//
//  MemLDOActiveTrim_boost()
//  Boost LDO active trim to the same level as simobuck boost. The trim boost
//  level is based on the original factory trim version.
//
//  bBoost: false = Revert to the factory trim.
//          true  = Boost LDO active trim according to factory trim version.
//
//*****************************************************************************
static uint32_t
MemLDOActiveTrim_boost(bool bBoost)
{
    uint32_t ui32MemLdoTrim, ui32CurrTrim, ui32TrimVer, ui32TrimStep;

    //
    // Check if original has been saved.
    //
    ui32MemLdoTrim = MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM;
    ui32CurrTrim   = ui32MemLdoTrim;

    if ( g_ui32origMEMLDOActiveTrim == 0xFFFFFFFF )
    {
        g_ui32origMEMLDOActiveTrim = ui32CurrTrim;
    }

    if ( !bBoost )
    {
        if ( ui32CurrTrim != g_ui32origMEMLDOActiveTrim )
        {
            MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM = g_ui32origMEMLDOActiveTrim;
        }

        return AM_HAL_STATUS_SUCCESS;
    }

    //
    // Determine trim step
    //
    TrimVersionGet(&ui32TrimVer);

    //
    // Boost to same level as simobuck boost.
    //
    ui32TrimStep = ( ui32TrimVer >= TRIMREV_PCM ) ?
                        BUCK_VDDF_BOOST_STEPS_PCM :
                        BUCK_VDDF_BOOST_STEPS_PREPCM;

    //
    // Increase LDO
    //
    ui32MemLdoTrim = (ui32MemLdoTrim <= (MAX_LDOVDDFTRIM - ui32TrimStep)) ?
                            (ui32MemLdoTrim + ui32TrimStep) : MAX_LDOVDDFTRIM;

    if ( ui32CurrTrim != ui32MemLdoTrim )
    {
        AM_CRITICAL_BEGIN
        MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM = ui32MemLdoTrim;

        am_hal_delay_us(AM_HAL_PWRCTRL_VDDF_BOOST_DELAY);
        AM_CRITICAL_END
    }

    return AM_HAL_STATUS_SUCCESS;

} // MemLDOActiveTrim_boost()
#endif // AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL

#if defined(AM_HAL_PWRCTL_HPLP_WA) || defined(AM_HAL_PWRCTL_CRYPTO_WA)
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
// Function to perform the RevB HP/LP mode switch.
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
            // Clear the timer. This HAL function also enables the timer via AUXEN.
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
            // making sure the interrupt clear has propogated by reading
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
#endif // defined(AM_HAL_PWRCTL_HPLP_WA) || defined(AM_HAL_PWRCTL_CRYPTO_WA)

//****************************************************************************
//
//  VDDC_simobuck_boost()
//
//****************************************************************************
static uint32_t
VDDC_simobuck_boost(uint32_t ui32VDDCboostcode)
{

    if ( g_ui32origSimobuckVDDCtrim == 0xFFFFFFFF )
    {
        if ( !g_bVDDCbuckboosted )
        {
            //
            // Set g_bVDDCbuckboosted to be true for VDDC voltage reverting before entering deepsleep
            //
            g_bVDDCbuckboosted =  true;
        }

        //
        // Get and save the original value the very first time.
        // Apply voltage boost only for the very first time.
        //
        g_ui32origSimobuckVDDCtrim = MCUCTRL->VREFGEN2_b.TVRGVREFTRIM;

        //
        // SIMOBUCK trim adjustment
        //
        MCUCTRL->SIMOBUCK15_b.TRIMLATCHOVER = 1;
        uint32_t ui32newvalVDDC = g_ui32origSimobuckVDDCtrim <= (MAX_BUCKVDDCTRIM - ui32VDDCboostcode) ?
                        g_ui32origSimobuckVDDCtrim + ui32VDDCboostcode : MAX_BUCKVDDCTRIM;

        if ( MCUCTRL->VREFGEN2_b.TVRGVREFTRIM != ui32newvalVDDC )
        {
            AM_CRITICAL_BEGIN
            MCUCTRL->VREFGEN2_b.TVRGVREFTRIM = ui32newvalVDDC;

            //
            // Delay to give voltage supply some time to transition to the new level
            //
            am_hal_delay_us(AM_HAL_PWRCTRL_VDDC_BOOST_DELAY);
            AM_CRITICAL_END
        }
    }

    return AM_HAL_STATUS_SUCCESS;

} // VDDC_simobuck_boost()

//****************************************************************************
//
//  vddc_vddf_boost()
//
//****************************************************************************
static uint32_t
vddc_vddf_boost(void)
{
    uint32_t ui32Status;
    uint32_t ui32TrimVer = 0;
    uint32_t ui32VDDCboostCode = 0;
    bool bDoVDDCboost = false;

    //
    // Get the Apollo4B device trim version.
    //
    ui32Status = TrimVersionGet(&ui32TrimVer);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Status;
    }

#if AM_HAL_PWRCTL_BOOST_VDDC
    uint16_t ui16LVTtrimHigh = 0xFFFF;
    uint16_t ui16LVTtrimLow  = 0xFFFF;

    if ( APOLLO4_GE_B1 && (ui32TrimVer >= TRIMREV_PCM) )
    {
        //
        // Get the Apollo4B LVT trim value.
        //
        ui32Status = TrimLVTGet(&ui16LVTtrimHigh, &ui16LVTtrimLow);
        if ( ui32Status != AM_HAL_STATUS_SUCCESS )
        {
            return ui32Status;
        }

        if ( (ui16LVTtrimLow < 240) && (ui16LVTtrimHigh < 240) )
        {
            ui32VDDCboostCode = SIMOBUCK_VDDC_BOOST_PCM_LT_240;
        }
        else
        {
            ui32VDDCboostCode = SIMOBUCK_VDDC_BOOST_PCM_GE_240;
        }

        bDoVDDCboost = true;
    }
#elif AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
    if ( ui32TrimVer >= TRIMREV_PCM )
    {
        ui32VDDCboostCode = SIMOBUCK_VDDC_BOOST_PARALLEL;
        bDoVDDCboost = true;
    }
#endif // AM_HAL_PWRCTL_BOOST_VDDC || AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL

    if ( bDoVDDCboost )
    {
        VDDC_simobuck_boost(ui32VDDCboostCode);
    }

    uint32_t ui32boostCodePCM, ui32boostCodePrePCM;
    ui32boostCodePCM = ui32boostCodePrePCM = 0;
#if AM_HAL_PWRCTL_BOOST_VDDF_FOR_BOTH_LP_HP
    ui32boostCodePCM    += BUCK_VDDF_BOOST_STEPS_PCM;
    ui32boostCodePrePCM += BUCK_VDDF_BOOST_STEPS_PREPCM;
#endif
    VDDF_simobuck_boost(true, ui32boostCodePCM, ui32boostCodePrePCM);

    return AM_HAL_STATUS_SUCCESS;
} // vddc_vddf_boost()

//****************************************************************************
//
//  am_hal_pwrctrl_mcu_mode_status()
//
//****************************************************************************
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

//****************************************************************************
//
//  am_hal_pwrctrl_mcu_mode_select()
//  Select the MCU power mode.
//
//****************************************************************************
uint32_t
am_hal_pwrctrl_mcu_mode_select(am_hal_pwrctrl_mcu_mode_e ePowerMode)
{
    bool bApollo4B0;
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
    bApollo4B0   = APOLLO4_B0;

    if ( bApollo4B0 )
    {
        //
        // Check if we need to apply a workaround for 192MHz operation.
        //
        if ( ePowerMode == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE )
        {
            CLKGEN->MISC_b.USEHFRC2FQ192MHZ = 1;
            MCUCTRL->PWRSW1_b.USEVDDF4VDDRCPUINHP = 1;
        }
        else
        {
            CLKGEN->MISC_b.USEHFRC2FQ96MHZ = 1;
        }

        //
        // Delay after setting the HFRC2 mode.
        //
        am_hal_delay_us(10);
    }

    //
    // Set the MCU power mode.
    //
    if ( ePowerMode == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE )
    {
        //
        // Boost VDDF for High Performance mode
        //
#if !AM_HAL_PWRCTL_BOOST_VDDF_FOR_BOTH_LP_HP
        VDDF_simobuck_boost(true, BUCK_VDDF_BOOST_STEPS_PCM, BUCK_VDDF_BOOST_STEPS_PREPCM);
#endif // AM_HAL_PWRCTL_BOOST_VDDF_FOR_BOTH_LP_HP

#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
        MemLDOActiveTrim_boost(true);
#endif  // AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
    }

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

    if ( ePowerMode == AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER )
    {
        //
        // Revert the boost VDDF when transitioning to Low Power  mode
        //
#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
        MemLDOActiveTrim_boost(false);
#endif // AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL

#if !AM_HAL_PWRCTL_BOOST_VDDF_FOR_BOTH_LP_HP
        VDDF_simobuck_boost(false, AM_DONT_CARE, AM_DONT_CARE);
#endif  // AM_HAL_PWRCTL_BOOST_VDDF_FOR_BOTH_LP_HP || AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
    }

    if ( bApollo4B0 )
    {
        //
        // Read the current power mode.
        //
        //ui32PowerMode = PWRCTRL->MCUPERFREQ_b.MCUPERFSTATUS;

        //
        // Check if we need to apply a workaround for 192MHz operation.
        //
        if ( AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE == ePowerMode )
        {
            CLKGEN->MISC_b.USEHFRC2FQ192MHZ = 0;
        }
        else
        {
            CLKGEN->MISC_b.USEHFRC2FQ96MHZ = 0;
        }

        //
        // Delay after setting the HFRC2 mode.
        //
        am_hal_delay_us(10);
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

//****************************************************************************
//
//  am_hal_pwrctrl_mcu_memory_config()
//  Configure the MCU memory.
//
//****************************************************************************
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

//****************************************************************************
//
//  am_hal_pwrctrl_mcu_memory_config_get()
//  Get the MCU Memory configuration.
//
//****************************************************************************
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

//****************************************************************************
//
//  am_hal_pwrctrl_sram_config()
//  Configure the Shared RAM.
//
//****************************************************************************
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
        case AM_HAL_PWRCTRL_SRAM_512K_GRP0:
            PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP = PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_GROUP1;
            break;
        case AM_HAL_PWRCTRL_SRAM_512K_GRP1:
            PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP = PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_GROUP0;
            break;
        case AM_HAL_PWRCTRL_SRAM_ALL:
            PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP = PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_NONE;
            break;
    }

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_pwrctrl_sram_config()

//****************************************************************************
//
//  am_hal_pwrctrl_sram_config_get()
//  Get the current Shared RAM configuration.
//
//****************************************************************************
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
        psConfig->eSRAMRetain = AM_HAL_PWRCTRL_SRAM_1M;
    }
    else if (PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP == PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_GROUP0)
    {
        psConfig->eSRAMRetain = AM_HAL_PWRCTRL_SRAM_512K_GRP1;
    }
    else if (PWRCTRL->SSRAMRETCFG_b.SSRAMPWDSLP == PWRCTRL_SSRAMRETCFG_SSRAMPWDSLP_GROUP1)
    {
        psConfig->eSRAMRetain = AM_HAL_PWRCTRL_SRAM_512K_GRP0;
    }
    else
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_pwrctrl_sram_config_get()

//****************************************************************************
//
//  am_hal_pwrctrl_dsp_mode_select()
//  Select the DSP power mode.
//
//****************************************************************************
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

//****************************************************************************
//
//  dsp0_memory_config()
//
//****************************************************************************
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

//****************************************************************************
//
//  dsp1_memory_config()
//
//****************************************************************************
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

//****************************************************************************
//
//  am_hal_pwrctrl_dsp_memory_config()
//  Configure the DSP memory.
//
//****************************************************************************
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

//****************************************************************************
//
//  dsp0_memory_get()
//
//****************************************************************************
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

//****************************************************************************
//
//  dsp1_memory_get()
//
//****************************************************************************
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

//****************************************************************************
//
//  am_hal_pwrctrl_dsp_memory_config_get()
//  Get the current the DSP memory configuration.
//
//****************************************************************************
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

//****************************************************************************
//
//  am_hal_pwrctrl_periph_enable()
//  Enable power for a peripheral.
//
//****************************************************************************
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

#ifdef AM_HAL_PWRCTL_CRYPTO_WA
    //
    // Workaround for CRYPTO block power-up issue.
    //
    if (ePeripheral == AM_HAL_PWRCTRL_PERIPH_CRYPTO)
    {
        am_hal_util_write_and_wait((uint32_t*)&PWRCTRL->DEVPWREN,
                                    PWRCTRL_DEVPWREN_PWRENCRYPTO_Msk,
                                    PWRCTRL_DEVPWREN_PWRENCRYPTO_Msk,
                                    AM_HAL_PWRCTL_CRYPTO_DELAY);
    }
    else
#endif // AM_HAL_PWRCTL_CRYPTO_WA
    {
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
            return ui32Status;
        }
    }

    //
    // Check the device status.
    //
    if ( (AM_REGVAL(pwr_ctrl.ui32PwrStatReqAddr) &
        pwr_ctrl.ui32PeriphStatus) != 0)
    {
        //
        // Crypto peripheral needs more time to power up after the normal status
        // bit is set. We'll wait for the IDLE signal from the NVM as our signal
        // that the crypto module is ready.
        //
        if (ePeripheral == AM_HAL_PWRCTRL_PERIPH_CRYPTO)
        {
            ui32Status = am_hal_delay_us_status_change(CRYPTO_WAIT_USEC,
                                                        (uint32_t)&CRYPTO->NVMISIDLE,
                                                        CRYPTO_NVMISIDLE_NVMISIDLEREG_Msk,
                                                        CRYPTO_NVMISIDLE_NVMISIDLEREG_Msk);
            //
            // Done, return the final status (the delay status in this case).
            //
            return ui32Status;
        }
        else
        {
            return AM_HAL_STATUS_SUCCESS;
        }
    }
    else
    {
        return AM_HAL_STATUS_FAIL;
    }
} // am_hal_pwrctrl_periph_enable()

//****************************************************************************
//
//  am_hal_pwrctrl_periph_disable_msk_check()
//  Function checks the PWRCTRL->DEVPWREN
//
//  The original check of ((PWRCTRL->DEVPWRSTATUS & ui32PeriphStatus) == 0)
//      will fail when more than one enable in the same domain is set and the
//      user tries disable only one.
//
//****************************************************************************
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

//****************************************************************************
//
//  am_hal_pwrctrl_periph_disable()
//  Disable power for a peripheral.
//
//****************************************************************************
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
        }
    }

#ifdef AM_HAL_PWRCTL_CRYPTO_WA
    //
    // Workaround for CRYPTO block power-up issue.
    //
    if (ePeripheral == AM_HAL_PWRCTRL_PERIPH_CRYPTO)
    {
        am_hal_util_write_and_wait((uint32_t*)&PWRCTRL->DEVPWREN,
                                    PWRCTRL_DEVPWREN_PWRENCRYPTO_Msk,
                                    0,
                                    AM_HAL_PWRCTL_CRYPTO_DELAY);

        //
        // Check the device status.
        //
        if ( (AM_REGVAL(pwr_ctrl.ui32PwrStatReqAddr) &
              pwr_ctrl.ui32PeriphStatus) == 0 )
        {
            return AM_HAL_STATUS_SUCCESS;
        }
        else
        {
            return AM_HAL_STATUS_FAIL;
        }
    }
    else
#endif // AM_HAL_PWRCTL_CRYPTO_WA
    {
        //
        // NOT AM_HAL_PWRCTRL_PERIPH_CRYPTO
        //
        //
        // Disable power domain for the given device.
        //
        AM_CRITICAL_BEGIN
        AM_REGVAL(pwr_ctrl.ui32PwrEnRegAddr) &=
          ~pwr_ctrl.ui32PeriphEnable;
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
    }
} // am_hal_pwrctrl_periph_disable()

//****************************************************************************
//
//  am_hal_pwrctrl_periph_enabled()
//  Determine whether a peripheral is currently enabled.
//
//****************************************************************************
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

//****************************************************************************
//
//  am_hal_pwrctrl_status_get()
//  Get the current powercontrol status registers.
//
//****************************************************************************
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

//****************************************************************************
//
//  am_hal_pwrctrl_low_power_init()
//  Initialize the device for low power operation.
//
//****************************************************************************
uint32_t
am_hal_pwrctrl_low_power_init(void)
{
    //
    // Fix the MRAM DeepSleep params
    //
    am_hal_mram_ds_init();

    //
    // Set the default memory configuration.
    //
    am_hal_pwrctrl_mcu_memory_config((am_hal_pwrctrl_mcu_memory_config_t *)&g_DefaultMcuMemCfg);
    am_hal_pwrctrl_sram_config((am_hal_pwrctrl_sram_memcfg_t *)&g_DefaultSRAMCfg);

    //
    // Make sure the original values of all the power-related regs are saved.
    //
    preserve_orig_reg_values();

    //
    // Enable all the clock gating optimizations for Rev B silicon.
    //
    CLKGEN->MISC_b.CLKGENMISCSPARES = 0x3E;  // Do not enable DAXI Clock Gating for now.

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

#if defined(AM_HAL_PWRCTRL_LFRC_SIMOBUCK_TRIM_WA)
    if ( !APOLLO4_GE_B2 )
    {
      //
      // Set and enable LFRC trims
      //
      MCUCTRL->LFRC_b.TRIMTUNELFRC         = 31;
      MCUCTRL->LFRC_b.LFRCSWE              = MCUCTRL_LFRC_LFRCSWE_OVERRIDE_EN;
      MCUCTRL->LFRC_b.LFRCSIMOCLKDIV       = MCUCTRL_LFRC_LFRCSIMOCLKDIV_DIV2;
    }
#endif

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

#if AM_HAL_PWRCTL_BOOST_VDDC_LDO
    //
    // If LDO, do some trim updates.
    //
    if ( PWRCTRL->VRSTATUS_b.SIMOBUCKST != PWRCTRL_VRSTATUS_SIMOBUCKST_ACT )
    {
        uint32_t ui32TrimVer, ui32VDDCtrim;
        bool bDoVDDCLDOboost;

        TrimVersionGet(&ui32TrimVer);
        bDoVDDCLDOboost = ( APOLLO4_GE_B1  &&  (ui32TrimVer >= TRIMREV_PCM) ) ? true : false;

        if ( bDoVDDCLDOboost )
        {
            if ( !g_bVDDCLDOboosted )
            {
                //
                // Only 1 VDDC boost per purchase please.
                //
                g_bVDDCLDOboosted = true;

                //
                // Get current trim
                //
                ui32VDDCtrim = MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM;

                if ( g_ui32origLDOVDDCtrim == 0xFFFFFFFF )
                {
                    //
                    // Save the original value the very first time.
                    //
                    g_ui32origLDOVDDCtrim = ui32VDDCtrim;
                }

                //
                // Increase VDDC up by 10 steps (~40mv), or max it out.
                //
                ui32VDDCtrim = (ui32VDDCtrim <= (MAX_LDOVDDCTRIM - 10)) ?
                               (ui32VDDCtrim + 10) : MAX_LDOVDDCTRIM;

                AM_CRITICAL_BEGIN
                MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM = ui32VDDCtrim;

                //
                // Delay to give voltage supply some time to transition to the new level
                //
                am_hal_delay_us(AM_HAL_PWRCTRL_VDDC_BOOST_DELAY);
                AM_CRITICAL_END
            }
        }
    }
#endif // AM_HAL_PWRCTL_BOOST_VDDC_LDO

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_pwrctrl_low_power_init()

#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
//****************************************************************************
//
//  buck_interval_check()
//
//****************************************************************************
#define BUCK_PULSE_TIMER_INST               13
#define BUCK_INTERVAL_TIMER_INST            8
#define BUCK_CH_VDDC                        0
#define BUCK_CH_VDDF                        1
#define BUCK_VDDC_INTERVAL_US_MIN           50
#define BUCK_VDDF_INTERVAL_US_MIN           50
#define BUCK_VDDC_INTERVAL_US_MAX           150
#define BUCK_VDDF_INTERVAL_US_MAX           150
#define BUCK_INTERVAL_CHECK_TIMEOUT_MAX     10000   // each loop is ~1us, timeout in 10ms
#define BUCK_INTERVAL_CHECK_PULSE_COUNT     10      // check for this number of buck pulses

extern uint32_t timer_config_b1(uint32_t ui32TimerNumber,
                                am_hal_timer_config_t *psTimerConfig);

static uint32_t
buck_interval_check(uint32_t *pui32IntervalUs, uint32_t channel)
{
    am_hal_timer_clock_e eBuckInput;
    uint32_t ui32BuckIntervalUsMin  = 0;
    uint32_t ui32BuckIntervalUsMax  = 0;
    uint32_t ui32IntervalUs         = 0;
    uint32_t ui32BuckPulseRead      = 0;
    uint32_t ui32BuckIntervalRead   = 0;
    uint32_t ui32TimeoutLoopCount   = BUCK_INTERVAL_CHECK_TIMEOUT_MAX;

    if (PWRCTRL->MCUPERFREQ_b.MCUPERFSTATUS == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE)
    {
        ui32TimeoutLoopCount = ui32TimeoutLoopCount*2;  // Scale the loop timeout for 192MHz vs. 96MHz.
    }

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Validate the parameters to the function
    //
    if (pui32IntervalUs == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif //! AM_HAL_DISABLE_API_VALIDATION

    if (channel == BUCK_CH_VDDC)
    {
        eBuckInput = AM_HAL_TIMER_CLOCK_BUCKC;
        ui32BuckIntervalUsMin = BUCK_VDDC_INTERVAL_US_MIN;
        ui32BuckIntervalUsMax = BUCK_VDDC_INTERVAL_US_MAX;

    }
    else if (channel == BUCK_CH_VDDF)
    {
        eBuckInput = AM_HAL_TIMER_CLOCK_BUCKF;
        ui32BuckIntervalUsMin = BUCK_VDDF_INTERVAL_US_MIN;
        ui32BuckIntervalUsMax = BUCK_VDDF_INTERVAL_US_MAX;
    }
    else
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Check if SIMOBUCK is enabled
    //
    if (PWRCTRL->VRCTRL_b.SIMOBUCKEN == 0)
    {
        // SIMOBUCK is not turned on
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    //
    // Check if LDOs are off
    //
    uint32_t ui32LdoStatus = 0;
    ui32LdoStatus = _VAL2FLD(MCUCTRL_VRCTRL_CORELDOACTIVE, 1)       |
                    _VAL2FLD(MCUCTRL_VRCTRL_CORELDOACTIVEEARLY, 1)  |
                    _VAL2FLD(MCUCTRL_VRCTRL_CORELDOPDNB, 1)         |
                    _VAL2FLD(MCUCTRL_VRCTRL_CORELDOOVER, 1)         |
                    _VAL2FLD(MCUCTRL_VRCTRL_MEMLDOACTIVE, 1)        |
                    _VAL2FLD(MCUCTRL_VRCTRL_MEMLDOACTIVEEARLY, 1)   |
                    _VAL2FLD(MCUCTRL_VRCTRL_MEMLDOPDNB, 1)          |
                    _VAL2FLD(MCUCTRL_VRCTRL_MEMLDOOVER, 1);


    if (MCUCTRL->VRCTRL & ui32LdoStatus)
    {
        // LDOs are on
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    //
    // Initialize timers
    //
    am_hal_timer_clear_stop(BUCK_PULSE_TIMER_INST);
    am_hal_timer_stop(BUCK_PULSE_TIMER_INST);
    am_hal_timer_reset_config(BUCK_PULSE_TIMER_INST);

    am_hal_timer_config_t timerConfig_pulse;
    am_hal_timer_default_config_set(&timerConfig_pulse);
    timerConfig_pulse.eInputClock       = eBuckInput;
    timerConfig_pulse.eFunction         = AM_HAL_TIMER_FN_UPCOUNT;
    timer_config_b1(BUCK_PULSE_TIMER_INST, &timerConfig_pulse);

    am_hal_timer_clear_stop(BUCK_INTERVAL_TIMER_INST);
    am_hal_timer_stop(BUCK_INTERVAL_TIMER_INST);
    am_hal_timer_reset_config(BUCK_INTERVAL_TIMER_INST);

    am_hal_timer_config_t timerConfig_interval;
    am_hal_timer_default_config_set(&timerConfig_interval);
    timerConfig_interval.eInputClock    = AM_HAL_TIMER_CLOCK_HFRC_DIV16;
    timerConfig_interval.eFunction      = AM_HAL_TIMER_FN_UPCOUNT;
    timer_config_b1(BUCK_INTERVAL_TIMER_INST, &timerConfig_interval);

    //
    // Start timers and measure interval
    //
    AM_CRITICAL_BEGIN
    am_hal_timer_start(BUCK_INTERVAL_TIMER_INST);
    am_hal_timer_start(BUCK_PULSE_TIMER_INST);
    AM_CRITICAL_END

    volatile uint32_t ui32PrimaskSave;
    while (ui32TimeoutLoopCount--)
    {
        ui32PrimaskSave = am_hal_interrupt_master_disable();   // enter critical section
        ui32BuckPulseRead = am_hal_timer_read(BUCK_PULSE_TIMER_INST);
        if (ui32BuckPulseRead >= BUCK_INTERVAL_CHECK_PULSE_COUNT)
        {
            // loop finished
            ui32BuckIntervalRead = am_hal_timer_read(BUCK_INTERVAL_TIMER_INST);
            am_hal_interrupt_master_set(ui32PrimaskSave);      // exit critical section
            break;
        }
        am_hal_interrupt_master_set(ui32PrimaskSave);          // exit critical section
    }

    //
    // Reset timers
    //
    am_hal_timer_clear_stop(BUCK_PULSE_TIMER_INST);
    am_hal_timer_stop(BUCK_PULSE_TIMER_INST);
    am_hal_timer_reset_config(BUCK_PULSE_TIMER_INST);
    am_hal_timer_clear_stop(BUCK_INTERVAL_TIMER_INST);
    am_hal_timer_stop(BUCK_INTERVAL_TIMER_INST);
    am_hal_timer_reset_config(BUCK_INTERVAL_TIMER_INST);

    if (ui32BuckIntervalRead == 0)
    {
        // interval not detected correctly
        return AM_HAL_STATUS_FAIL;
    }
    else
    {
        // convert interval into us
        ui32IntervalUs = ui32BuckIntervalRead / 6;  // Counting clock is HFRC/16, 6MHz

        if (ui32IntervalUs < ui32BuckIntervalUsMin)
        {
            // guarantee a min interval
            ui32IntervalUs = ui32BuckIntervalUsMin;
        }

        if (ui32IntervalUs > ui32BuckIntervalUsMax)
        {
            // guarantee a min interval
            ui32IntervalUs = ui32BuckIntervalUsMax;
        }

        *pui32IntervalUs = ui32IntervalUs;
        return AM_HAL_STATUS_SUCCESS;
    }
} // buck_interval_check()

//****************************************************************************
//
// Helper functions for coreldo and memldo enable and parallel operation
//
//****************************************************************************
//****************************************************************************
//
//  memldo_vddf_parallel_set()
//  Turn on MEMLDO in parallel with VDDF SIMOBUCK.
//
//****************************************************************************
static void
memldo_vddf_parallel_set(uint32_t ui32BuckIntervalUs)
{
    uint32_t ui32BuckPulseRead0;
    uint32_t ui32BuckPulseRead1;                     // set initial value different
    uint32_t ui32MemLdoTrim     = 0;
    int32_t  i32TrimStep        = 0;

    //
    // use timer to capture buck pulse here
    //
    am_hal_timer_config_t timerConfig;
    am_hal_timer_default_config_set(&timerConfig);
    timerConfig.eInputClock = AM_HAL_TIMER_CLOCK_BUCKF;
    timerConfig.eFunction   = AM_HAL_TIMER_FN_UPCOUNT;
    timer_config_b1(BUCK_PULSE_TIMER_INST, &timerConfig);
    am_hal_timer_start(BUCK_PULSE_TIMER_INST);

    ui32BuckPulseRead0 = am_hal_timer_read(BUCK_PULSE_TIMER_INST);
    am_hal_delay_us(ui32BuckIntervalUs);
    ui32BuckPulseRead1 = am_hal_timer_read(BUCK_PULSE_TIMER_INST);

    if (ui32BuckPulseRead0 == ui32BuckPulseRead1)
    {
        //
        // LDO is higher than buck voltage
        //
        i32TrimStep -= TRIM_STEP;
    }
    else
    {
        //
        // LDO is lower than buck voltage
        //
        i32TrimStep += TRIM_STEP;
    }

    while (true)
    {
        ui32MemLdoTrim = MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM;

        if ( g_ui32origMEMLDOActiveTrim == 0xFFFFFFFF )
        {
            g_ui32origMEMLDOActiveTrim = ui32MemLdoTrim;
        }

        if ( i32TrimStep > 0 )
        {
            //
            // increasing LDO
            //
            if ( (ui32MemLdoTrim == MAX_LDOVDDFTRIM) || (ui32BuckPulseRead0 == ui32BuckPulseRead1) )
            {
                break;
            }

            ui32MemLdoTrim = (ui32MemLdoTrim <= (MAX_LDOVDDFTRIM - i32TrimStep)) ?
                                    (ui32MemLdoTrim + i32TrimStep) : MAX_LDOVDDFTRIM;
        }

        if ( i32TrimStep < 0 )
        {
            //
            // decreasing LDO
            //
            if ( (ui32MemLdoTrim <= MIN_LDOVDDFTRIM) || (ui32BuckPulseRead0 != ui32BuckPulseRead1) )
            {
                break;
            }

            ui32MemLdoTrim = (ui32MemLdoTrim >= (MIN_LDOVDDFTRIM - i32TrimStep)) ?
                                    (ui32MemLdoTrim + i32TrimStep) : MIN_LDOVDDFTRIM;
        }

        MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM = ui32MemLdoTrim;

        ui32BuckPulseRead0 = am_hal_timer_read(BUCK_PULSE_TIMER_INST);
        am_hal_delay_us(ui32BuckIntervalUs);
        ui32BuckPulseRead1 = am_hal_timer_read(BUCK_PULSE_TIMER_INST);
    }

    //
    // Out of the loop, make LDO voltage lower for parallel operation
    //
    if ( ui32MemLdoTrim > (MEMLDO_STEPDOWN_TRIMCODE + MIN_LDOVDDFTRIM) )
    {
        ui32MemLdoTrim -= MEMLDO_STEPDOWN_TRIMCODE;
    }
    else
    {
        ui32MemLdoTrim = MIN_LDOVDDFTRIM;
    }

    //
    // Apply setting and stop the timer
    //
    MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM = ui32MemLdoTrim;

    am_hal_timer_clear_stop(BUCK_PULSE_TIMER_INST);
    am_hal_timer_stop(BUCK_PULSE_TIMER_INST);
    am_hal_timer_reset_config(BUCK_PULSE_TIMER_INST);

} // memldo_vddf_parallel_set()

//****************************************************************************
//
//  coreldo_vddc_parallel_set()
//  Turn on CORELDO in parallel with VDDC SIMOBUCK.
//
//****************************************************************************
static void
coreldo_vddc_parallel_set(uint32_t ui32BuckIntervalUs)
{
    uint32_t ui32BuckPulseRead0;
    uint32_t ui32BuckPulseRead1;                     // set initial value different
    uint32_t ui32CoreLdoTrim    = 0;
    int32_t  i32TrimStep        = 0;

    //
    // use timer capture buck pulse here
    //
    am_hal_timer_config_t timerConfig;
    am_hal_timer_default_config_set(&timerConfig);
    timerConfig.eInputClock = AM_HAL_TIMER_CLOCK_BUCKC;
    timerConfig.eFunction   = AM_HAL_TIMER_FN_UPCOUNT;
    timer_config_b1(BUCK_PULSE_TIMER_INST, &timerConfig);
    am_hal_timer_start(BUCK_PULSE_TIMER_INST);

    ui32BuckPulseRead0 = am_hal_timer_read(BUCK_PULSE_TIMER_INST);
    am_hal_delay_us(ui32BuckIntervalUs);
    ui32BuckPulseRead1 = am_hal_timer_read(BUCK_PULSE_TIMER_INST);

    if ( ui32BuckPulseRead0 == ui32BuckPulseRead1 )
    {
        //
        // LDO is higher than buck voltage
        //
        i32TrimStep -= TRIM_STEP;
    }
    else
    {
        //
        // LDO is lower than buck voltage
        //
        i32TrimStep += TRIM_STEP;
    }

    while (true)
    {
        ui32CoreLdoTrim = MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM;

        if (i32TrimStep > 0)
        {
            //
            // increasing LDO
            //
            if ((ui32CoreLdoTrim == MAX_LDOVDDCTRIM) || (ui32BuckPulseRead0 == ui32BuckPulseRead1))
            {
                break;
            }

            ui32CoreLdoTrim = (ui32CoreLdoTrim <= (MAX_LDOVDDCTRIM - i32TrimStep)) ?
                                    (ui32CoreLdoTrim + i32TrimStep) : MAX_LDOVDDCTRIM;
        }

        if (i32TrimStep < 0)
        {
            //
            // decreasing LDO
            //
            if ( (ui32CoreLdoTrim <= MIN_LDOVDDCTRIM) || (ui32BuckPulseRead0 != ui32BuckPulseRead1) )
            {
                break;
            }

            ui32CoreLdoTrim = (ui32CoreLdoTrim >= (MIN_LDOVDDCTRIM - i32TrimStep)) ?
                                    (ui32CoreLdoTrim + i32TrimStep) : MIN_LDOVDDCTRIM;
        }

        if ( i32TrimStep == 0 )
        {
            // something wrong here.
            break;
        }

        MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM = ui32CoreLdoTrim;

        ui32BuckPulseRead0 = am_hal_timer_read(BUCK_PULSE_TIMER_INST);
        am_hal_delay_us(ui32BuckIntervalUs);
        ui32BuckPulseRead1 = am_hal_timer_read(BUCK_PULSE_TIMER_INST);
    }

    //
    // Out of the loop, make LDO voltage lower for parallel operation
    //
    if ( ui32CoreLdoTrim > (CORELDO_STEPDOWN_TRIMCODE + MIN_LDOVDDCTRIM) )
    {
        ui32CoreLdoTrim -= CORELDO_STEPDOWN_TRIMCODE;
    }
    else
    {
        ui32CoreLdoTrim = MIN_LDOVDDCTRIM;
    }

    //
    // Apply setting and stop the timer
    //
    MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM = ui32CoreLdoTrim;

    am_hal_timer_clear_stop(BUCK_PULSE_TIMER_INST);
    am_hal_timer_stop(BUCK_PULSE_TIMER_INST);
    am_hal_timer_reset_config(BUCK_PULSE_TIMER_INST);

} // coreldo_vddc_parallel_set()
#endif // AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL

//****************************************************************************
//
//  buck_ldo_override_init
//
//****************************************************************************
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
    //
    // Change CORELDOTEMPCOTRIM to 4
    // In order to give the CORELDO enough voltage range.
    // However, CORELDOOVER must be the last bit set.
    //
    MCUCTRL->LDOREG1_b.CORELDOTEMPCOTRIM  = 4;
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
}


//****************************************************************************
//
// Dynamically turn on and off the overrides for buck and LDO
// Override configs are already set once in buck_ldo_override_init
//
//****************************************************************************
void
buck_ldo_update_override(bool bEnable)
{
    MCUCTRL->VRCTRL_b.SIMOBUCKOVER   = bEnable;
#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
    MCUCTRL->VRCTRL_b.CORELDOOVER    = bEnable;
    MCUCTRL->VRCTRL_b.MEMLDOOVER     = bEnable;
#endif // AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
} // buck_ldo_update_override()

//****************************************************************************
//
//  am_hal_pwrctrl_control()
//  Additional miscellaneous power controls.
//
//****************************************************************************
uint32_t
am_hal_pwrctrl_control(am_hal_pwrctrl_control_e eControl, void *pArgs)
{
    uint32_t ui32ReturnStatus = AM_HAL_STATUS_SUCCESS;
#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
    uint32_t ui32BuckVddcIntervalUs = 0;
    uint32_t ui32BuckVddfIntervalUs = 0;
#endif // AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL

    switch ( eControl )
    {
        case AM_HAL_PWRCTRL_CONTROL_SIMOBUCK_INIT:
            //
            // Apply specific trim optimization for SIMOBUCK.
            //

            //
            // Set zero crossing for comparator to best value.
            //
            MCUCTRL->SIMOBUCK15_b.ZXCOMPOFFSETTRIM = 0;
            MCUCTRL->SIMOBUCK7_b.ZXCOMPZXTRIM      = 0;

            //
            // Set Set VDDC active low and high TON trim.
            //
            MCUCTRL->SIMOBUCK2_b.SIMOBUCKVDDCACTLOWTONTRIM     = 0xA;
            MCUCTRL->SIMOBUCK2_b.SIMOBUCKVDDCACTHIGHTONTRIM    = 0xA;

            //
            // Set VDDF active low and high TON trim.
            //
            MCUCTRL->SIMOBUCK7_b.VDDFACTLOWTONTRIM             = 0xF;
            MCUCTRL->SIMOBUCK6_b.SIMOBUCKVDDFACTHIGHTONTRIM    = 0xF;

            //
            // Set VDDS active low and high TON trim.
            //
            MCUCTRL->SIMOBUCK9_b.SIMOBUCKVDDSACTLOWTONTRIM     = 0xF;
            MCUCTRL->SIMOBUCK9_b.SIMOBUCKVDDSACTHIGHTONTRIM    = 0xF;

            //
            // Set the VDDS LP and VDDF LP trim values to same values.
            //
            MCUCTRL->SIMOBUCK13_b.SIMOBUCKLPTRIMVDDS = MCUCTRL->SIMOBUCK12_b.LPTRIMVDDF;

#if AM_HAL_PWRCTL_SHORT_VDDF_TO_VDDS
            //
            // Enable VDDF to VDDS short to increase load cap (2.2uF + 2.2uF).
            //
            MCUCTRL->PWRSW1_b.SHORTVDDFVDDSORVAL  = 1;
            MCUCTRL->PWRSW1_b.SHORTVDDFVDDSOREN   = 1;

            g_ui32origSimobuckVDDStrim = MCUCTRL->SIMOBUCK13_b.SIMOBUCKACTTRIMVDDS;
            MCUCTRL->SIMOBUCK13_b.SIMOBUCKACTTRIMVDDS = 0;    // VDDS trim level to 0
#endif // AM_HAL_PWRCTL_SHORT_VDDF_TO_VDDS

            //
            // Enable VDDC, VDDF, and VDDS.
            //
            MCUCTRL->SIMOBUCK0  =   _VAL2FLD(MCUCTRL_SIMOBUCK0_VDDCRXCOMPEN, 1) |
                                    _VAL2FLD(MCUCTRL_SIMOBUCK0_VDDSRXCOMPEN, 1) |
                                    _VAL2FLD(MCUCTRL_SIMOBUCK0_VDDFRXCOMPEN, 1);

            //
            // Set SIMOBUCK clock.
            //
            MCUCTRL->SIMOBUCK1_b.SIMOBUCKTONCLKTRIM       = 0;
            MCUCTRL->SIMOBUCK1_b.SIMOBUCKRXCLKACTTRIM     = 1;

            //
            // Perform adjustments to voltage trims to ensure proper functionality.
            //
            ui32ReturnStatus = vddc_vddf_boost();
            if (AM_HAL_STATUS_SUCCESS != ui32ReturnStatus)
            {
                return ui32ReturnStatus;
            }

#if AM_HAL_PWRCTL_KEEP_ANA_ACTIVE_IN_DS
            if ( APOLLO4_GE_B1 )
            {
                //
                // Keep the ANALDO and SIMOBUCK during DeepSleep.
                //
                //
                // Check if the patch has been applied to make the ACRG adjustments, otherwise return an error.
                uint32_t ui32PatchTracker;
                uint32_t ui32Ret = am_hal_mram_info_read(1, AM_REG_INFO1_PATCH_TRACKER0_O / 4, 1, &ui32PatchTracker);

                if ( (ui32Ret == 0) && (!(ui32PatchTracker & AM_REG_INFO1_PATCH_TRACKER0_ACRG_CV_M)) )
                {
                    //
                    // Keep ACRG always active.
                    // Note: this step requires INFO1 patch to open the ACRG register to customer modification.
                    //
                    MCUCTRL->ACRG_b.ACRGPWD       = 0;
                    MCUCTRL->ACRG_b.ACRGSWE       = 1;

                    //
                    // Keep ANALDO always active.
                    //
                    MCUCTRL->VRCTRL_b.ANALDOACTIVE      = 1;
                    MCUCTRL->VRCTRL_b.ANALDOPDNB        = 1;
                    MCUCTRL->VRCTRL_b.ANALDOOVER        = 1;
                }
                else
                {
                    ui32ReturnStatus = AM_HAL_STATUS_INVALID_OPERATION;
                }
            }
#endif // AM_HAL_PWRCTL_KEEP_ANA_ACTIVE_IN_DS

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


#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
            am_hal_delay_us(5000);
            if ( buck_interval_check(&ui32BuckVddcIntervalUs, BUCK_CH_VDDC) != AM_HAL_STATUS_SUCCESS)
            {
                // Simobuck VDDC interval check failed, skip this operation
                break;
            }

            if ( buck_interval_check(&ui32BuckVddfIntervalUs, BUCK_CH_VDDF) != AM_HAL_STATUS_SUCCESS )
            {
                // Simobuck VDDF interval check failed, skip this operation
                break;
            }
#endif // AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL

            // Initialize the Buck and LDO override settings, and enable overrides
            buck_ldo_override_init();

#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
            am_hal_delay_us(2500);

            coreldo_vddc_parallel_set(ui32BuckVddcIntervalUs);
            memldo_vddf_parallel_set(ui32BuckVddfIntervalUs);

#endif // AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
            break;

#ifdef AM_HAL_PWRCTL_CRYPTO_WA
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
#endif // AM_HAL_PWRCTL_CRYPTO_WA

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

        default:
            ui32ReturnStatus = AM_HAL_STATUS_INVALID_ARG;
            break;
    }

    //
    // Return success status.
    //
    return ui32ReturnStatus;

} // am_hal_pwrctrl_control()

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

    //
    // Ensure - we're in LP mode
    //
    if (PWRCTRL->MCUPERFREQ_b.MCUPERFREQ != AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER)
    {
        //
        // Device needs to be in LP mode before restore is called
        //
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    //
    // Need to revert the trim changes, and turn to LDO mode
    //
    AM_CRITICAL_BEGIN

    //
    // Revert LDO trims back to original info1 values
    //      VDDC and VDDF active trims
    //      VDDC and VDDF LP trims
    //
    if (g_ui32origLDOVDDCtrim != 0xFFFFFFFF)
    {
        MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM = g_ui32origLDOVDDCtrim;
    }

    if (g_ui32origMEMLDOActiveTrim != 0xFFFFFFFF)
    {
        MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM = g_ui32origMEMLDOActiveTrim;
    }

    //
    // Switch to LDO mode (if not already in LDO mode)
    // Delay 20us for rails to settle
    //
    if (PWRCTRL->VRSTATUS_b.SIMOBUCKST == PWRCTRL_VRSTATUS_SIMOBUCKST_ACT)
    {
#if AM_HAL_PWRCTL_SHORT_VDDF_TO_VDDS
        if (g_ui32origSimobuckVDDStrim != 0xFFFFFFFF)
        {
            MCUCTRL->SIMOBUCK13_b.SIMOBUCKACTTRIMVDDS = g_ui32origSimobuckVDDStrim;
        }

        //
        // Remove VDDS/VDDF short
        //
        MCUCTRL->PWRSW1_b.SHORTVDDFVDDSORVAL  = 0;
        MCUCTRL->PWRSW1_b.SHORTVDDFVDDSOREN   = 0;
#endif

        PWRCTRL->VRCTRL_b.SIMOBUCKEN = 0;

        //
        // Need to remove overrides
        //
        buck_ldo_update_override(false);

        //
        // Revert simobuck trims back to original info1 values
        //      VDDC active trim (LP and active use the same bitfield)
        //      VDDF LP and active trims
        //      VDDS LP and active trims
        //
        MCUCTRL->SIMOBUCK15_b.TRIMLATCHOVER = 0;
        if (g_ui32origSimobuckVDDFtrim != 0xFFFFFFFF)
        {
            MCUCTRL->SIMOBUCK12_b.ACTTRIMVDDF = g_ui32origSimobuckVDDFtrim;
        }

        if (g_ui32origSimobuckVDDCtrim != 0xFFFFFFFF)
        {
            MCUCTRL->VREFGEN2_b.TVRGVREFTRIM = g_ui32origSimobuckVDDCtrim;
        }
#if AM_HAL_PWRCTL_KEEP_ANA_ACTIVE_IN_DS
        MCUCTRL->ACRG_b.ACRGSWE       = 0;

        MCUCTRL->VRCTRL_b.ANALDOOVER        = 0;
        MCUCTRL->VRCTRL_b.ANALDOACTIVE      = 0;
        MCUCTRL->VRCTRL_b.ANALDOPDNB        = 0;
#endif
    }
    else
    {
        am_hal_delay_us(AM_HAL_PWRCTRL_GOTOLDO_DELAY);
    }

    AM_CRITICAL_END

    return AM_HAL_STATUS_SUCCESS;
}


//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
