//*****************************************************************************
//
//! @file am_hal_pwrctrl.h
//!
//! @brief Functions for enabling and disabling power domains.
//!
//! @addtogroup pwrctrl4_4b PWRCTRL - Power Control
//! @ingroup apollo4b_hal
//! @{

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

#ifndef AM_HAL_PWRCTRL_H
#define AM_HAL_PWRCTRL_H

#ifdef __cplusplus
extern "C"
{
#endif

//
// Delays (in uS) required for VDDF/VDDC trim enhancements.
//
#define AM_HAL_PWRCTRL_VDDF_BOOST_DELAY     20
#define AM_HAL_PWRCTRL_VDDC_BOOST_DELAY     20
#define AM_HAL_PWRCTRL_GOTOLDO_DELAY        20


//*****************************************************************************
//
//! Option to boost VDDF in Low Power as well as High Performance modes.
//!
//! AM_HAL_PWRCTL_BOOST_VDDF_FOR_BOTH_LP_HP:
//!      0 = Boost VDDF only when switching to HP mode.
//!      1 = Boost VDDF in both LP and HP modes.
//!  Default: 0
//
//*****************************************************************************
#define AM_HAL_PWRCTL_BOOST_VDDF_FOR_BOTH_LP_HP         0

//*****************************************************************************
//
//! Option for keeping portion of Analog in active mode during Deepsleep.
//!  AM_HAL_PWRCTL_KEEP_ANA_ACTIVE_IN_DS:
//!      0 = Leave inactive.
//!      1 = Set trims to keep active.  Uses extra power in Deepsleep.
//!  Default: 0
//! Setting this to 1 also mandates setting AM_HAL_PWRCTL_KEEP_SIMO_ACTIVE_IN_DS
//*****************************************************************************
#define AM_HAL_PWRCTL_KEEP_ANA_ACTIVE_IN_DS             0

//*****************************************************************************
//
//! Option for keeping Simobuck in active mode during Deepsleep.
//!  AM_HAL_PWRCTL_KEEP_SIMO_ACTIVE_IN_DS:
//!      0 = Let Simobuck go to LP mode when possible.
//!      1 = Simobuck is forced to stay active even in Deep Sleep.
//!          Uses extra power in Deepsleep.
//!  Default: 0
//
//*****************************************************************************
#define AM_HAL_PWRCTL_KEEP_SIMO_ACTIVE_IN_DS            0

#if AM_HAL_PWRCTL_KEEP_ANA_ACTIVE_IN_DS
#undef AM_HAL_PWRCTL_KEEP_SIMO_ACTIVE_IN_DS
#define AM_HAL_PWRCTL_KEEP_SIMO_ACTIVE_IN_DS            1
#endif

//*****************************************************************************
//
//! Option for Boosting VDDC, buck only.
//!  AM_HAL_PWRCTL_BOOST_VDDC
//!      0 = No Boost.
//!      1 = Boost VDDC for PCM trimmed parts
//!  Default: 0
//
//*****************************************************************************
#define AM_HAL_PWRCTL_BOOST_VDDC                        0

//*****************************************************************************
//
//! Option for Boosting VDDC, LDO only.
//!  AM_HAL_PWRCTL_BOOST_VDDC_LDO
//!      0 = No Boost.
//!      1 = Boost VDDC for PCM trimmed parts (this is rolled back in deepsleep)
//!  Default: 0
//
//! @note This option is no longer needed after new production test screen.
//
//*****************************************************************************
#define AM_HAL_PWRCTL_BOOST_VDDC_LDO                    0

//*****************************************************************************
//
//! Option for coreldo and memldo to operate in parallel with simobuck
//!  AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
//!      0 = Do not turn on LDOs in parallel with simobuck.
//!      1 = Turn on LDOs in parallel with simobuck and set their voltage levels
//!          ~35mV lower than minimum buck voltages.
//!  Default: 1
//
//*****************************************************************************
#define AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL    1

//*****************************************************************************
//
//! Option to assist VDDC by activating LDOs when disabling SIMOBUCK.
//!  AM_HAL_PWRCTRL_LDOS_FOR_VDDC
//!      0 = Do not assist VDDC.
//!      1 = Activate LDOs in parallel when disabling SIMOBUCK.
//!  Default: 1
//
//*****************************************************************************
#define AM_HAL_PWRCTRL_LDOS_FOR_VDDC                    1
#if ( (AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL == 0) && (AM_HAL_PWRCTRL_LDOS_FOR_VDDC != 0) )
#error AM_HAL_PWRCTRL_LDOS_FOR_VDDC requires AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL.
#endif
#if ( (AM_HAL_PWRCTL_KEEP_SIMO_ACTIVE_IN_DS == 0) && (AM_HAL_PWRCTRL_LDOS_FOR_VDDC == 0) )
#warning Both of the AM_HAL_PWRCTRL_LDOS_FOR_VDDC and AM_HAL_PWRCTL_KEEP_SIMO_ACTIVE_IN_DS options are disabled. This condition can cause operation problems.
#endif

//*****************************************************************************
//
//! Option for shorting VDDF to VDDS
//!  AM_HAL_PWRCTL_SHORT_VDDF_TO_VDDS
//!      0 = Leave inactive.
//!      1 = Set trims to short.
//!  Default: 1
//
//*****************************************************************************
#define AM_HAL_PWRCTL_SHORT_VDDF_TO_VDDS                1

//
// Check for invalid option combinations
//
#if ( (AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL != 0) && (AM_HAL_PWRCTL_SHORT_VDDF_TO_VDDS == 0) )
#error AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL must include AM_HAL_PWRCTL_SHORT_VDDF_TO_VDDS.
#endif

//*****************************************************************************
//
// Defines to enable Apollo4 revB workarounds.
//
//*****************************************************************************
//
//! AM_HAL_PWRCTL_HPLP_WA workaround implementation uses a TIMER interrupt
//! AM_HAL_WRITE_WAIT_TIMER (TIMER13 used for this in default SDK).
//! The interrupt is configured as the highest priority (0) interrupt to prevent
//! unintentional break out due to other interrupts. In order for this to work
//! reliably, it is required that all the other interrupts in the system are set
//! at a lower priority, reserving the highest priority interrupt exclusively
//! for AmbiqSuite workaround.
//
#define AM_HAL_PWRCTL_HPLP_WA

//
//! AM_HAL_PWRCTL_CRYPTO_WA workaround implementation uses a TIMER interrupt
//! AM_HAL_WRITE_WAIT_TIMER (TIMER13 used for this in default SDK).
//! The interrupt is configured as the highest priority (0) interrupt to prevent
//! unintentional break out due to other interrupts. In order for this to work
//! reliably, it is required that all the other interrupts in the system are set
//! at a lower priority, reserving the highest priority interrupt exclusively
//! for AmbiqSuite workaround.
//

#define AM_HAL_PWRCTL_CRYPTO_WA

#ifdef AM_HAL_PWRCTL_CRYPTO_WA
#define AM_HAL_PWRCTL_CRYPTO_DELAY 16
#endif

#ifdef AM_HAL_PWRCTL_HPLP_WA
#define AM_HAL_PWRCTL_HPLP_DELAY   10
#endif

//*****************************************************************************
//
//! Option for shorting VDDC to VDDC_LV During normal operation
//!
//! @note During normal operation in active and deep-sleep, VDDC_LV is
//!       disconnected from VDDC. While the VDDC_LV rail is disconnected from
//!       VDDC, the VDDC_LV capacitor slowly discharges due to leakage.
//!       A deeply discharged VDDC_LV capacitor can cause a large drop in VDDC
//!       voltage when VDDC_LV and VDDC are shorted together. When the device
//!       switches from SIMOBUCK to LDO mode, VDDC_LV and VDDC are shorted.
//!       this happens prior to deepsleep and in am_hal_settings_restore call.
//! 
//!  AM_HAL_PWRCTL_SHORT_VDDC_TO_VDDCLV
//!      Set trims to short. Uses extra power in Deepsleep.
//!  Default: undefined
//
//*****************************************************************************
//#define AM_HAL_PWRCTL_SHORT_VDDC_TO_VDDCLV

//*****************************************************************************
//
//! Define the structure for preserving various trim registers
//! including SIMOBUCK trim and other related registers.
//
//*****************************************************************************
struct trim_regs_s
{
    uint32_t    ui32Valid;

    // MCUCTRL SIMOBUCK regs
    uint32_t    ui32SB0;
    uint32_t    ui32SB1;
    uint32_t    ui32SB2;
    uint32_t    ui32SB4;
    uint32_t    ui32SB6;
    uint32_t    ui32SB7;
    uint32_t    ui32SB9;
    uint32_t    ui32SB12;
    uint32_t    ui32SB13;
    uint32_t    ui32SB15;

    // Other MCUCTRL regs
    uint32_t    ui32LDOREG1;
    uint32_t    ui32LDOREG2;
    uint32_t    ui32VREFGEN2;
    uint32_t    ui32PWRSW0;
    uint32_t    ui32PWRSW1;
    uint32_t    ui32VRCTRL;
    uint32_t    ui32ACRG;
    uint32_t    ui32XTALGENCTRL;
    uint32_t    ui32ADCPWRCTRL;
    uint32_t    ui32AUDADCPWRCTRL;
};

//
//! Expose the global structure for saving the registers.
//
extern struct trim_regs_s g_trim_reg_origvals;

//*****************************************************************************
//
//! @name Performace mode enums.
//! @{
//
//*****************************************************************************
//
//! Peripheral MCU Mode power enum.
//
typedef enum
{
    AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER        = PWRCTRL_MCUPERFREQ_MCUPERFSTATUS_LP, // 96 MHz
    AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE = PWRCTRL_MCUPERFREQ_MCUPERFSTATUS_HP, // 192 MHz
} am_hal_pwrctrl_mcu_mode_e;

//
//! Peripheral DSP Mode power enum.
//
typedef enum
{
    AM_HAL_PWRCTRL_DSP_MODE_ULTRA_LOW_POWER,  // 48 MHz
    AM_HAL_PWRCTRL_DSP_MODE_LOW_POWER,        // 192 MHz
    AM_HAL_PWRCTRL_DSP_MODE_HIGH_PERFORMANCE, // 384 MHz
} am_hal_pwrctrl_dsp_mode_e;
//
//! @}
//

//*****************************************************************************
//
//! Peripheral power enum.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_PWRCTRL_PERIPH_IOS,
    AM_HAL_PWRCTRL_PERIPH_IOM0,
    AM_HAL_PWRCTRL_PERIPH_IOM1,
    AM_HAL_PWRCTRL_PERIPH_IOM2,
    AM_HAL_PWRCTRL_PERIPH_IOM3,
    AM_HAL_PWRCTRL_PERIPH_IOM4,
    AM_HAL_PWRCTRL_PERIPH_IOM5,
    AM_HAL_PWRCTRL_PERIPH_IOM6,
    AM_HAL_PWRCTRL_PERIPH_IOM7,
    AM_HAL_PWRCTRL_PERIPH_UART0,
    AM_HAL_PWRCTRL_PERIPH_UART1,
    AM_HAL_PWRCTRL_PERIPH_UART2,
    AM_HAL_PWRCTRL_PERIPH_UART3,
    AM_HAL_PWRCTRL_PERIPH_ADC,
    AM_HAL_PWRCTRL_PERIPH_MSPI0,
    AM_HAL_PWRCTRL_PERIPH_MSPI1,
    AM_HAL_PWRCTRL_PERIPH_MSPI2,
    AM_HAL_PWRCTRL_PERIPH_GFX,
    AM_HAL_PWRCTRL_PERIPH_DISP,
    AM_HAL_PWRCTRL_PERIPH_DISPPHY,
    AM_HAL_PWRCTRL_PERIPH_CRYPTO,
    AM_HAL_PWRCTRL_PERIPH_SDIO,
    AM_HAL_PWRCTRL_PERIPH_USB,
    AM_HAL_PWRCTRL_PERIPH_USBPHY,
    AM_HAL_PWRCTRL_PERIPH_DEBUG,
    AM_HAL_PWRCTRL_PERIPH_AUDREC,
    AM_HAL_PWRCTRL_PERIPH_AUDPB,
    AM_HAL_PWRCTRL_PERIPH_PDM0,
    AM_HAL_PWRCTRL_PERIPH_PDM1,
    AM_HAL_PWRCTRL_PERIPH_PDM2,
    AM_HAL_PWRCTRL_PERIPH_PDM3,
    AM_HAL_PWRCTRL_PERIPH_I2S0,
    AM_HAL_PWRCTRL_PERIPH_I2S1,
    AM_HAL_PWRCTRL_PERIPH_AUDADC,
    AM_HAL_PWRCTRL_PERIPH_MAX
} am_hal_pwrctrl_periph_e;

//*****************************************************************************
//
// MCU memory control settings.
//
//*****************************************************************************

//
//! Cache enable settings.
//
typedef enum
{
    AM_HAL_PWRCTRL_CACHE_NONE,
    AM_HAL_PWRCTRL_CACHEB0_ONLY,
    AM_HAL_PWRCTRL_CACHE_ALL,
} am_hal_pwrctrl_cache_select_e;

//
//! DTCM enable settings.
//
typedef enum
{
    AM_HAL_PWRCTRL_DTCM_NONE    = PWRCTRL_MEMPWREN_PWRENDTCM_NONE,
    AM_HAL_PWRCTRL_DTCM_8K      = PWRCTRL_MEMPWREN_PWRENDTCM_TCM8K,
    AM_HAL_PWRCTRL_DTCM_128K    = PWRCTRL_MEMPWREN_PWRENDTCM_TCM128K,
    AM_HAL_PWRCTRL_DTCM_384K    = PWRCTRL_MEMPWREN_PWRENDTCM_TCM384K,
} am_hal_pwrctrl_dtcm_select_e;

//
//! MCU memory configuration structure.
//
typedef struct
{
    //
    //! Cache configuration.
    //
    am_hal_pwrctrl_cache_select_e       eCacheCfg;
    bool                                bRetainCache;

    //
    //! DTCM configuration
    //
    am_hal_pwrctrl_dtcm_select_e        eDTCMCfg;
    am_hal_pwrctrl_dtcm_select_e        eRetainDTCM;

    //
    //! NVM configuration.
    //
    bool                                bEnableNVM0;
    bool                                bRetainNVM0;
} am_hal_pwrctrl_mcu_memory_config_t;

//
//! Miscellaneous power controls.
//
typedef enum
{
    AM_HAL_PWRCTRL_CONTROL_SIMOBUCK_INIT,       // Enable the SIMOBUCK
    AM_HAL_PWRCTRL_CONTROL_CRYPTO_POWERDOWN,    // Power down Crypto
    AM_HAL_PWRCTRL_CONTROL_XTAL_PWDN_DEEPSLEEP, // Allow the crystal to power down during deepsleep
    AM_HAL_PWRCTRL_CONTROL_DIS_PERIPHS_ALL,     // Power down all peripherals
} am_hal_pwrctrl_control_e;



//*****************************************************************************
//
//! DSP memory control settings.
//
//*****************************************************************************
#if 0
  // Check that all the reg defs are consistent with enums below.
#if ((0 == PWRCTRL_DSP0MEMPWREN_PWRENDSP0IRAM_NONE)     &&  \
     (0 == PWRCTRL_DSP0MEMRETCFG_IRAMPWDDSP0OFF_NONE)   &&  \
     (1 == PWRCTRL_DSP0MEMPWREN_PWRENDSP0IRAM_GROUP0)   &&  \
     (1 == PWRCTRL_DSP0MEMRETCFG_IRAMPWDDSP0OFF_GROUP0) &&  \
     (3 == PWRCTRL_DSP0MEMPWREN_PWRENDSP0IRAM_ALL)      &&  \
     (3 == PWRCTRL_DSP0MEMRETCFG_IRAMPWDDSP0OFF_ALL)    &&  \
     (0 == PWRCTRL_DSP1MEMPWREN_PWRENDSP1IRAM_NONE)     &&  \
     (0 == PWRCTRL_DSP1MEMRETCFG_IRAMPWDDSP1OFF_NONE)   &&  \
     (1 == PWRCTRL_DSP1MEMPWREN_PWRENDSP1IRAM_GROUP0)   &&  \
     (1 == PWRCTRL_DSP1MEMRETCFG_IRAMPWDDSP1OFF_GROUP0))
#error DSP IRAM Register Definitions do not support enum defintion!
#endif

//
//! DSP IRAM Select
//
typedef enum
{
    AM_HAL_PWRCTRL_DSP_IRAM_NONE        = PWRCTRL_DSP0MEMPWREN_PWRENDSP0IRAM_NONE,
    AM_HAL_PWRCTRL_DSP_IRAM_BANK0_ONLY  = PWRCTRL_DSP0MEMPWREN_PWRENDSP0IRAM_GROUP0,
    AM_HAL_PWRCTRL_DSP_IRAM_ALL_BANKS   = PWRCTRL_DSP0MEMPWREN_PWRENDSP0IRAM_ALL
}
am_hal_pwrctrl_dsp_iram_select_e;

// Check that all the reg defs are consistent with enums below.
#if ((0 == PWRCTRL_DSP0MEMPWREN_PWRENDSP0DRAM_NONE)     &&  \
     (0 == PWRCTRL_DSP0MEMRETCFG_DRAMPWDDSP0OFF_NONE)   &&  \
     (1 == PWRCTRL_DSP0MEMPWREN_PWRENDSP0DRAM_GROUP0)   &&  \
     (1 == PWRCTRL_DSP0MEMRETCFG_DRAMPWDDSP0OFF_GROUP0) &&  \
     (3 == PWRCTRL_DSP0MEMPWREN_PWRENDSP0DRAM_ALL)      &&  \
     (3 == PWRCTRL_DSP0MEMRETCFG_DRAMPWDDSP0OFF_ALL)    &&  \
     (0 == PWRCTRL_DSP1MEMPWREN_PWRENDSP1DRAM_NONE)     &&  \
     (0 == PWRCTRL_DSP1MEMRETCFG_DRAMPWDDSP1OFF_NONE)   &&  \
     (1 == PWRCTRL_DSP1MEMPWREN_PWRENDSP1DRAM_GROUP0)   &&  \
     (1 == PWRCTRL_DSP1MEMRETCFG_DRAMPWDDSP1OFF_GROUP0))
)
#error DSP IRAM Register Definitions do not support enum defintion!
#endif

//
//! DPS DRAM Select
//
typedef enum
{
    AM_HAL_PWRCTRL_DSP_DRAM_NONE        = PWRCTRL_DSP0MEMPWREN_PWRENDSP0DRAM_NONE,
    AM_HAL_PWRCTRL_DSP_DRAM_BANK0_ONLY  = PWRCTRL_DSP0MEMPWREN_PWRENDSP0DRAM_GROUP0,
    AM_HAL_PWRCTRL_DSP_DRAM_ALL_BANKS   = PWRCTRL_DSP0MEMPWREN_PWRENDSP0DRAM_ALL
} am_hal_pwrctrl_dsp_dram_select_e;
#endif

//*****************************************************************************
//
//! DSP memory config settings.
//
//*****************************************************************************
typedef struct
{
    bool        bEnableICache;
    bool        bRetainCache;
    bool        bEnableRAM;
    bool        bActiveRAM;
    bool        bRetainRAM;
} am_hal_pwrctrl_dsp_memory_config_t;

//*****************************************************************************
//
//! Shared memory select settings.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_PWRCTRL_SRAM_NONE      = 0,  //!< No SSRAM
    AM_HAL_PWRCTRL_SRAM_512K_GRP0 = 1,  //!< Lower 512K
    AM_HAL_PWRCTRL_SRAM_512K_GRP1 = 2,  //!< Upper 512K
    AM_HAL_PWRCTRL_SRAM_ALL       = 3,  //!< All shared SRAM (1M)
    // Legacy naming
    AM_HAL_PWRCTRL_SRAM_512K      = AM_HAL_PWRCTRL_SRAM_512K_GRP0,
    AM_HAL_PWRCTRL_SRAM_1M        = AM_HAL_PWRCTRL_SRAM_ALL
} am_hal_pwrctrl_sram_select_e;

//*****************************************************************************
//
//! Shared memory config settings.
//
//*****************************************************************************
typedef struct
{
    //
    //! Shared SRAM (SSRAM) banks to enable.
    //!  AM_HAL_PWRCTRL_SRAM_NONE    = No SSRAM enabled.
    //!  AM_HAL_PWRCTRL_SRAM_1M_GRP0 = Lower 1M enabled, upper 1M disabled.
    //!  AM_HAL_PWRCTRL_SRAM_1M_GRP1 = Upper 1M enabled, lower 1M disabled.
    //!  AM_HAL_PWRCTRL_SRAM_ALL     = ALL SSRAM enabled.
    //! These bits are written to SSRAMPWREN.PWRENSSRAM.
    //
    am_hal_pwrctrl_sram_select_e        eSRAMCfg;

    //
    //! For each of the eActiveWithXxx settings (which apply to each master).
    //!  AM_HAL_PWRCTRL_SRAM_NONE    = Allow SSRAM to go to retention during deep sleep.
    //!  AM_HAL_PWRCTRL_SRAM_1M_GRP0 = Lower 1M forced on, upper 1M allowed to retention in deep sleep.
    //!  AM_HAL_PWRCTRL_SRAM_1M_GRP1 = Upper 1M forced on, lower 1M allowed to retention in deep sleep.
    //!  AM_HAL_PWRCTRL_SRAM_ALL     = SSRAM forced on even in deep sleep.
    //
    //! Activate SSRAM when the MCU is active.
    //! These bits are written to SSRAMRETCFG.SSRAMACTMCU.
    //
    am_hal_pwrctrl_sram_select_e        eActiveWithMCU;

    //
    //! Activate SSRAM when the Graphics is active.
    //! These bits are written to SSRAMRETCFG.SSRAMACTGFX.
    //
    am_hal_pwrctrl_sram_select_e        eActiveWithGFX;

    //
    //! Activate SSRAM when the DISPLAY is active.
    //! These bits are written to SSRAMRETCFG.SSRAMACTDISP.
    //
    am_hal_pwrctrl_sram_select_e        eActiveWithDISP;

    //
    //! Activate SSRAM when either of the DSPs are active.
    //! These bits are written to SSRAMRETCFG.SSRAMACTDSP.
    //
    am_hal_pwrctrl_sram_select_e        eActiveWithDSP;

    //
    //! Retain SSRAM in deep sleep.
    //! For SSRAM retention:
    //!  AM_HAL_PWRCTRL_SRAM_NONE    = Power down all SSRAM in deepsleep (no retention).
    //!  AM_HAL_PWRCTRL_SRAM_1M_GRP0 = Retain lower 1M, power down upper 1M in deepsleep.
    //!  AM_HAL_PWRCTRL_SRAM_1M_GRP1 = Retain upper 1M, power down lower 1M in deepsleep.
    //!  AM_HAL_PWRCTRL_SRAM_ALL     = Retain all SSRAM in deepsleep.
    //! The HAL writes the INVERSE of these bits to SSRAMRETCFG.SSRAMPWDSLP in order to
    //! provide the desired action.
    //
    am_hal_pwrctrl_sram_select_e        eSRAMRetain;

} am_hal_pwrctrl_sram_memcfg_t;

//*****************************************************************************
//
//! System power status structure
//
//*****************************************************************************
typedef struct
{
    //
    //! DEVPWRSTATUS - Device Power ON Status
    //
    uint32_t ui32Device;

    //
    //! AUDSSPWRSTATUS - Audio Subsystem ON Status
    //
    uint32_t ui32AudioSS;

    //
    //! MEMPWRSTATUS - MCU Memory Power ON Status
    //
    uint32_t ui32Memory;

    //
    //! SYSPWRSTATUS - Power ON Status for MCU and DSP0/1 Cores
    //
    uint32_t ui32System;

    //
    //! SSRAMPWRST - Shared SRAM Power ON Status
    //
    uint32_t ui32SSRAM;

    //
    //! AUDSSPWRSTATUS - Audio Subsystem Power ON Status
    //
    uint32_t ui32Audio;

    //
    //! DSP0MEMPWRST - DSP0 Memories Power ON Status
    //
    uint32_t ui32DSP0MemStatus;

    //
    //! DSP1MEMPWRST - DSP1 Memories Power ON Status
    //
    uint32_t ui32DSP1MemStatus;

    //
    //! VRSTATUS - Voltage Regulators status
    //
    uint32_t ui32VRStatus;

    //
    //! ADCSTATUS - Power Status Register for ADC Block
    //
    uint32_t ui32ADC;

    //
    //! AUDADCSTATUS - Power Status Register for audio ADC Block
    //
    uint32_t ui32AudioADC;
} am_hal_pwrctrl_status_t;

//*****************************************************************************
//
//! @name Default configurations
//! @{
//
//*****************************************************************************
extern const am_hal_pwrctrl_mcu_memory_config_t      g_DefaultMcuMemCfg;
extern const am_hal_pwrctrl_sram_memcfg_t            g_DefaultSRAMCfg;
extern const am_hal_pwrctrl_dsp_memory_config_t      g_DefaultDSPMemCfg;
//! @}

// ****************************************************************************
//
//! @brief Return the current MCU performance mode.
//!
//! @param peCurrentPowerMode is a ptr to a variable to save the current status.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors.
//
// ****************************************************************************
extern uint32_t am_hal_pwrctrl_mcu_mode_status(am_hal_pwrctrl_mcu_mode_e *peCurrentPowerMode);

//*****************************************************************************
//
//! @brief Change the MCU performance mode.
//!
//! @param ePowerMode is the performance mode for the MCU
//!
//! Select the power mode for the MCU.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors.
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_mcu_mode_select(am_hal_pwrctrl_mcu_mode_e ePowerMode);

//*****************************************************************************
//
//! @brief Configure the power settings for the MCU memory.
//!
//! @param psConfig is a structure describing the desired memory configuration.
//!
//! Use this function to enable, disable, or change the sleep configuration of
//! MCU memory.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_mcu_memory_config(am_hal_pwrctrl_mcu_memory_config_t *psConfig);

//*****************************************************************************
//
//! @brief Read the power settings for the MCU memory.
//!
//! @param psConfig is a structure describing the desired memory configuration.
//!
//! Use this function to check the current settings for the MCU memories.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_mcu_memory_config_get(am_hal_pwrctrl_mcu_memory_config_t *psConfig);

//*****************************************************************************
//
//! @brief Configure the power settings for the Shared RAM.
//!
//! @param psConfig is a structure describing the desired memory configuration.
//!
//! Use this function to enable, disable, or change the sleep configuration of
//! MCU memory.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_sram_config(am_hal_pwrctrl_sram_memcfg_t *psConfig);

//*****************************************************************************
//
//! @brief Get the power settings for the Shared RAM.
//!
//! @param psConfig is a structure describing the desired memory configuration.
//!
//! Use this function to check the current settings for the MCU memories.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_sram_config_get(am_hal_pwrctrl_sram_memcfg_t *psConfig);

//*****************************************************************************
//
//! @brief Change the DSP performance mode.
//!
//! @param eDSP is the instance of the DSP 0 or 1.
//! @param ePowerMode is the performance mode for the DSP
//!
//! Select the power mode for the DSP.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_dsp_mode_select(am_hal_dsp_select_e eDSP,
                                               am_hal_pwrctrl_dsp_mode_e ePowerMode);

//*****************************************************************************
//
//! @brief Configure the power and memory settings for the DSP.
//!
//! @param eDSP is the instance of the DSP 0 or 1.
//! @param psConfig is a structure containing settings for the DSP
//!
//! Use this function to configure the DSP. See the documentation for the
//! configuration structure for more information on the available settings.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_dsp_memory_config(am_hal_dsp_select_e eDSP,
                                                 am_hal_pwrctrl_dsp_memory_config_t *psConfig);

//*****************************************************************************
//
//! @brief Get the current power and memory settings for the DSP.
//!
//! @param eDSP is the instance of the DSP 0 or 1.
//! @param psConfig is a returns the current settings for the DSP
//!
//! Use this function to check the current configuration of the DSP. This will
//! populate a configuration structure with exactly the same format required by
//! the configuration function.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_dsp_memory_config_get(am_hal_dsp_select_e eDSP,
                                                     am_hal_pwrctrl_dsp_memory_config_t *psConfig);

//*****************************************************************************
//
//! @brief Enable power to a peripheral.
//!
//! @param ePeripheral - The peripheral to enable.
//!
//! This function enables power to the peripheral and waits for a
//! confirmation from the hardware.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_periph_enable(am_hal_pwrctrl_periph_e ePeripheral);

//*****************************************************************************
//
//! @brief Disable power to a peripheral.
//!
//! @param ePeripheral - The peripheral to disable.
//!
//! This function disables power to the peripheral and waits for a
//! confirmation from the hardware.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_periph_disable(am_hal_pwrctrl_periph_e ePeripheral);

//*****************************************************************************
//
//! @brief Determine whether a peripheral is currently enabled.
//!
//! @param ePeripheral - The peripheral to enable.
//! @param bEnabled - Pointer to a ui32 that will return as 1 or 0.
//!
//! This function determines to the caller whether a given peripheral is
//! currently enabled or disabled.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_periph_enabled(am_hal_pwrctrl_periph_e ePeripheral,
                                              bool *bEnabled);

//*****************************************************************************
//
//! @brief Get the current powercontrol status registers.
//!
//! @param psStatus returns a structure containing power status information.
//!
//! This function can be used to determine the current status of a wide variety
//! of system components.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_status_get(am_hal_pwrctrl_status_t *psStatus);

//*****************************************************************************
//
//! @brief Initialize system for low power configuration.
//!
//! This function implements various low power initialization and optimizations.
//! - See also am_hal_pwrctrl_control() for other power saving techniques.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_low_power_init(void);

//*****************************************************************************
//
//! @brief Miscellaneous power saving controls
//!
//! @param eControl - Power saving type, one of the following:
//!     AM_HAL_PWRCTRL_CONTROL_SIMOBUCK_INIT        - Enable the SIMOBUCK
//!     AM_HAL_PWRCTRL_CONTROL_XTAL_PWDN_DEEPSLEEP  - Allow the crystal to power
//!                                                   down during deepsleep
//!
//! @param pArgs - Pointer to arguments for Control Switch Case
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable PWRCTRL errors
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_control(am_hal_pwrctrl_control_e eControl, void *pArgs);

//*****************************************************************************
//
//! @brief Restore original Power settings
//!
//! This function restores default power trims reverting relative
//! changes done as part of low_power_init and SIMOBUCK init.
//! User needs to make sure device is running in Low Power mode before calling
//! this function.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_pwrctrl_settings_restore(void);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_PWRCTRL_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

