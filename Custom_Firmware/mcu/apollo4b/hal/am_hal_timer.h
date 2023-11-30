//*****************************************************************************
//
//! @file am_hal_timer.h
//!
//! @brief Functions for interfacing with the timer (TIMER).
//!
//! @addtogroup timer_4b Timer Functionality
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
#ifndef AM_HAL_TIMER_H
#define AM_HAL_TIMER_H

#ifdef __cplusplus
extern "C"
{
#endif

#undef AM_REG_NUM_TIMERS
#define AM_REG_NUM_TIMERS       16

//*****************************************************************************
//
// CMSIS-style macro for handling a variable TIMER module number.
//
//*****************************************************************************
#define AM_HAL_TIMER_OFFSET (&TIMER->CTRL1 - &TIMER->CTRL0)
#define TIMERn(n)   ((TIMER_Type*)(TIMER_BASE + (n * 4 * AM_HAL_TIMER_OFFSET)))
#define AM_HAL_TIMER_GLOBEN_DEFAULT     0x0000FFFF
#define AM_HAL_TIMER_MASK(timer, compare) (compare << (2 * timer))

//*****************************************************************************
//
//! Timer selection enum
//
//*****************************************************************************
typedef enum
{
    AM_HAL_TIMER_TMR0  = TIMER_GLOBEN_ENB0_Msk,
    AM_HAL_TIMER_TMR1  = TIMER_GLOBEN_ENB1_Msk,
    AM_HAL_TIMER_TMR2  = TIMER_GLOBEN_ENB2_Msk,
    AM_HAL_TIMER_TMR3  = TIMER_GLOBEN_ENB3_Msk,
    AM_HAL_TIMER_TMR4  = TIMER_GLOBEN_ENB4_Msk,
    AM_HAL_TIMER_TMR5  = TIMER_GLOBEN_ENB5_Msk,
    AM_HAL_TIMER_TMR6  = TIMER_GLOBEN_ENB6_Msk,
    AM_HAL_TIMER_TMR7  = TIMER_GLOBEN_ENB7_Msk,
    AM_HAL_TIMER_TMR8  = TIMER_GLOBEN_ENB8_Msk,
    AM_HAL_TIMER_TMR9  = TIMER_GLOBEN_ENB9_Msk,
    AM_HAL_TIMER_TMR10 = TIMER_GLOBEN_ENB10_Msk,
    AM_HAL_TIMER_TMR11 = TIMER_GLOBEN_ENB11_Msk,
    AM_HAL_TIMER_TMR12 = TIMER_GLOBEN_ENB12_Msk,
    AM_HAL_TIMER_TMR13 = TIMER_GLOBEN_ENB13_Msk,
    AM_HAL_TIMER_TMR14 = TIMER_GLOBEN_ENB14_Msk,
    AM_HAL_TIMER_TMR15 = TIMER_GLOBEN_ENB15_Msk,
}
am_hal_timer_select_e;

//*****************************************************************************
//
//! TIMER configuration enum
//
//*****************************************************************************
typedef enum
{
    AM_HAL_TIMER_CLOCK_HFRC_DIV16   = TIMER_CTRL0_TMR0CLK_HFRC_DIV16,
    AM_HAL_TIMER_CLOCK_HFRC_DIV64   = TIMER_CTRL0_TMR0CLK_HFRC_DIV64,
    AM_HAL_TIMER_CLOCK_HFRC_DIV256  = TIMER_CTRL0_TMR0CLK_HFRC_DIV256,
    AM_HAL_TIMER_CLOCK_HFRC_DIV1024 = TIMER_CTRL0_TMR0CLK_HFRC_DIV1024,
    AM_HAL_TIMER_CLOCK_HFRC_DIV4K   = TIMER_CTRL0_TMR0CLK_HFRC_DIV4K,
    AM_HAL_TIMER_CLOCK_LFRC         = TIMER_CTRL0_TMR0CLK_LFRC,
    AM_HAL_TIMER_CLOCK_LFRC_DIV2    = TIMER_CTRL0_TMR0CLK_LFRC_DIV2,
    AM_HAL_TIMER_CLOCK_LFRC_DIV32   = TIMER_CTRL0_TMR0CLK_LFRC_DIV32,
    AM_HAL_TIMER_CLOCK_LFRC_DIV1K   = TIMER_CTRL0_TMR0CLK_LFRC_DIV1K,
    AM_HAL_TIMER_CLOCK_XT           = TIMER_CTRL0_TMR0CLK_XT,
    AM_HAL_TIMER_CLOCK_XT_DIV2      = TIMER_CTRL0_TMR0CLK_XT_DIV2,
    AM_HAL_TIMER_CLOCK_XT_DIV4      = TIMER_CTRL0_TMR0CLK_XT_DIV4,
    AM_HAL_TIMER_CLOCK_XT_DIV8      = TIMER_CTRL0_TMR0CLK_XT_DIV8,
    AM_HAL_TIMER_CLOCK_XT_DIV16     = TIMER_CTRL0_TMR0CLK_XT_DIV16,
    AM_HAL_TIMER_CLOCK_XT_DIV32     = TIMER_CTRL0_TMR0CLK_XT_DIV32,
    AM_HAL_TIMER_CLOCK_XT_DIV128    = TIMER_CTRL0_TMR0CLK_XT_DIV128,
    AM_HAL_TIMER_CLOCK_RTC_100HZ    = TIMER_CTRL0_TMR0CLK_RTC_100HZ,
    AM_HAL_TIMER_CLOCK_BUCKC        = TIMER_CTRL0_TMR0CLK_BUCKC,
    AM_HAL_TIMER_CLOCK_BUCKF        = TIMER_CTRL0_TMR0CLK_BUCKF,
    AM_HAL_TIMER_CLOCK_BUCKS        = TIMER_CTRL0_TMR0CLK_BUCKS,
    AM_HAL_TIMER_CLOCK_BUCKC_LV     = TIMER_CTRL0_TMR0CLK_BUCKC_LV,
    AM_HAL_TIMER_CLOCK_TMR0_OUT0    = TIMER_CTRL0_TMR0CLK_TMR00,
    AM_HAL_TIMER_CLOCK_TMR0_OUT1    = TIMER_CTRL0_TMR0CLK_TMR01,
    AM_HAL_TIMER_CLOCK_TMR1_OUT0    = TIMER_CTRL0_TMR0CLK_TMR10,
    AM_HAL_TIMER_CLOCK_TMR1_OUT1    = TIMER_CTRL0_TMR0CLK_TMR11,
    AM_HAL_TIMER_CLOCK_TMR2_OUT0    = TIMER_CTRL0_TMR0CLK_TMR20,
    AM_HAL_TIMER_CLOCK_TMR2_OUT1    = TIMER_CTRL0_TMR0CLK_TMR21,
    AM_HAL_TIMER_CLOCK_TMR3_OUT0    = TIMER_CTRL0_TMR0CLK_TMR30,
    AM_HAL_TIMER_CLOCK_TMR3_OUT1    = TIMER_CTRL0_TMR0CLK_TMR31,
    AM_HAL_TIMER_CLOCK_TMR4_OUT0    = TIMER_CTRL0_TMR0CLK_TMR40,
    AM_HAL_TIMER_CLOCK_TMR4_OUT1    = TIMER_CTRL0_TMR0CLK_TMR41,
    AM_HAL_TIMER_CLOCK_TMR5_OUT0    = TIMER_CTRL0_TMR0CLK_TMR50,
    AM_HAL_TIMER_CLOCK_TMR5_OUT1    = TIMER_CTRL0_TMR0CLK_TMR51,
    AM_HAL_TIMER_CLOCK_TMR6_OUT0    = TIMER_CTRL0_TMR0CLK_TMR60,
    AM_HAL_TIMER_CLOCK_TMR6_OUT1    = TIMER_CTRL0_TMR0CLK_TMR61,
    AM_HAL_TIMER_CLOCK_TMR7_OUT0    = TIMER_CTRL0_TMR0CLK_TMR70,
    AM_HAL_TIMER_CLOCK_TMR7_OUT1    = TIMER_CTRL0_TMR0CLK_TMR71,
    AM_HAL_TIMER_CLOCK_TMR8_OUT0    = TIMER_CTRL0_TMR0CLK_TMR80,
    AM_HAL_TIMER_CLOCK_TMR8_OUT1    = TIMER_CTRL0_TMR0CLK_TMR81,
    AM_HAL_TIMER_CLOCK_TMR9_OUT0    = TIMER_CTRL0_TMR0CLK_TMR90,
    AM_HAL_TIMER_CLOCK_TMR9_OUT1    = TIMER_CTRL0_TMR0CLK_TMR91,
    AM_HAL_TIMER_CLOCK_TMR10_OUT0   = TIMER_CTRL0_TMR0CLK_TMR100,
    AM_HAL_TIMER_CLOCK_TMR10_OUT1   = TIMER_CTRL0_TMR0CLK_TMR101,
    AM_HAL_TIMER_CLOCK_TMR11_OUT0   = TIMER_CTRL0_TMR0CLK_TMR110,
    AM_HAL_TIMER_CLOCK_TMR11_OUT1   = TIMER_CTRL0_TMR0CLK_TMR111,
    AM_HAL_TIMER_CLOCK_TMR12_OUT0   = TIMER_CTRL0_TMR0CLK_TMR120,
    AM_HAL_TIMER_CLOCK_TMR12_OUT1   = TIMER_CTRL0_TMR0CLK_TMR121,
    AM_HAL_TIMER_CLOCK_TMR13_OUT0   = TIMER_CTRL0_TMR0CLK_TMR130,
    AM_HAL_TIMER_CLOCK_TMR13_OUT1   = TIMER_CTRL0_TMR0CLK_TMR131,
    AM_HAL_TIMER_CLOCK_TMR14_OUT0   = TIMER_CTRL0_TMR0CLK_TMR140,
    AM_HAL_TIMER_CLOCK_TMR14_OUT1   = TIMER_CTRL0_TMR0CLK_TMR140,
    AM_HAL_TIMER_CLOCK_TMR15_OUT0   = TIMER_CTRL0_TMR0CLK_TMR150,
    AM_HAL_TIMER_CLOCK_TMR15_OUT1   = TIMER_CTRL0_TMR0CLK_TMR151,
    AM_HAL_TIMER_CLOCK_GPIO0        = TIMER_CTRL0_TMR0CLK_GPIO0,
    AM_HAL_TIMER_CLOCK_GPIO1,
    AM_HAL_TIMER_CLOCK_GPIO2,
    AM_HAL_TIMER_CLOCK_GPIO3,
    AM_HAL_TIMER_CLOCK_GPIO4,
    AM_HAL_TIMER_CLOCK_GPIO5,
    AM_HAL_TIMER_CLOCK_GPIO6,
    AM_HAL_TIMER_CLOCK_GPIO7,
    AM_HAL_TIMER_CLOCK_GPIO8,
    AM_HAL_TIMER_CLOCK_GPIO9,
    AM_HAL_TIMER_CLOCK_GPIO10,
    AM_HAL_TIMER_CLOCK_GPIO11,
    AM_HAL_TIMER_CLOCK_GPIO12,
    AM_HAL_TIMER_CLOCK_GPIO13,
    AM_HAL_TIMER_CLOCK_GPIO14,
    AM_HAL_TIMER_CLOCK_GPIO15,
    AM_HAL_TIMER_CLOCK_GPIO16,
    AM_HAL_TIMER_CLOCK_GPIO17,
    AM_HAL_TIMER_CLOCK_GPIO18,
    AM_HAL_TIMER_CLOCK_GPIO19,
    AM_HAL_TIMER_CLOCK_GPIO20,
    AM_HAL_TIMER_CLOCK_GPIO21,
    AM_HAL_TIMER_CLOCK_GPIO22,
    AM_HAL_TIMER_CLOCK_GPIO23,
    AM_HAL_TIMER_CLOCK_GPIO24,
    AM_HAL_TIMER_CLOCK_GPIO25,
    AM_HAL_TIMER_CLOCK_GPIO26,
    AM_HAL_TIMER_CLOCK_GPIO27,
    AM_HAL_TIMER_CLOCK_GPIO28,
    AM_HAL_TIMER_CLOCK_GPIO29,
    AM_HAL_TIMER_CLOCK_GPIO30,
    AM_HAL_TIMER_CLOCK_GPIO31,
    AM_HAL_TIMER_CLOCK_GPIO32,
    AM_HAL_TIMER_CLOCK_GPIO33,
    AM_HAL_TIMER_CLOCK_GPIO34,
    AM_HAL_TIMER_CLOCK_GPIO35,
    AM_HAL_TIMER_CLOCK_GPIO36,
    AM_HAL_TIMER_CLOCK_GPIO37,
    AM_HAL_TIMER_CLOCK_GPIO38,
    AM_HAL_TIMER_CLOCK_GPIO39,
    AM_HAL_TIMER_CLOCK_GPIO40,
    AM_HAL_TIMER_CLOCK_GPIO41,
    AM_HAL_TIMER_CLOCK_GPIO42,
    AM_HAL_TIMER_CLOCK_GPIO43,
    AM_HAL_TIMER_CLOCK_GPIO44,
    AM_HAL_TIMER_CLOCK_GPIO45,
    AM_HAL_TIMER_CLOCK_GPIO46,
    AM_HAL_TIMER_CLOCK_GPIO47,
    AM_HAL_TIMER_CLOCK_GPIO48,
    AM_HAL_TIMER_CLOCK_GPIO49,
    AM_HAL_TIMER_CLOCK_GPIO50,
    AM_HAL_TIMER_CLOCK_GPIO51,
    AM_HAL_TIMER_CLOCK_GPIO52,
    AM_HAL_TIMER_CLOCK_GPIO53,
    AM_HAL_TIMER_CLOCK_GPIO54,
    AM_HAL_TIMER_CLOCK_GPIO55,
    AM_HAL_TIMER_CLOCK_GPIO56,
    AM_HAL_TIMER_CLOCK_GPIO57,
    AM_HAL_TIMER_CLOCK_GPIO58,
    AM_HAL_TIMER_CLOCK_GPIO59,
    AM_HAL_TIMER_CLOCK_GPIO60,
    AM_HAL_TIMER_CLOCK_GPIO61,
    AM_HAL_TIMER_CLOCK_GPIO62,
    AM_HAL_TIMER_CLOCK_GPIO63,
    AM_HAL_TIMER_CLOCK_GPIO64,
    AM_HAL_TIMER_CLOCK_GPIO65,
    AM_HAL_TIMER_CLOCK_GPIO66,
    AM_HAL_TIMER_CLOCK_GPIO67,
    AM_HAL_TIMER_CLOCK_GPIO68,
    AM_HAL_TIMER_CLOCK_GPIO69,
    AM_HAL_TIMER_CLOCK_GPIO70,
    AM_HAL_TIMER_CLOCK_GPIO71,
    AM_HAL_TIMER_CLOCK_GPIO72,
    AM_HAL_TIMER_CLOCK_GPIO73,
    AM_HAL_TIMER_CLOCK_GPIO74,
    AM_HAL_TIMER_CLOCK_GPIO75,
    AM_HAL_TIMER_CLOCK_GPIO76,
    AM_HAL_TIMER_CLOCK_GPIO77,
    AM_HAL_TIMER_CLOCK_GPIO78,
    AM_HAL_TIMER_CLOCK_GPIO79,
    AM_HAL_TIMER_CLOCK_GPIO80,
    AM_HAL_TIMER_CLOCK_GPIO81,
    AM_HAL_TIMER_CLOCK_GPIO82,
    AM_HAL_TIMER_CLOCK_GPIO83,
    AM_HAL_TIMER_CLOCK_GPIO84,
    AM_HAL_TIMER_CLOCK_GPIO85,
    AM_HAL_TIMER_CLOCK_GPIO86,
    AM_HAL_TIMER_CLOCK_GPIO87,
    AM_HAL_TIMER_CLOCK_GPIO88,
    AM_HAL_TIMER_CLOCK_GPIO89,
    AM_HAL_TIMER_CLOCK_GPIO90,
    AM_HAL_TIMER_CLOCK_GPIO91,
    AM_HAL_TIMER_CLOCK_GPIO92,
    AM_HAL_TIMER_CLOCK_GPIO93,
    AM_HAL_TIMER_CLOCK_GPIO94,
    AM_HAL_TIMER_CLOCK_GPIO95,
    AM_HAL_TIMER_CLOCK_GPIO96,
    AM_HAL_TIMER_CLOCK_GPIO97,
    AM_HAL_TIMER_CLOCK_GPIO98,
    AM_HAL_TIMER_CLOCK_GPIO99,
    AM_HAL_TIMER_CLOCK_GPIO100,
    AM_HAL_TIMER_CLOCK_GPIO101,
    AM_HAL_TIMER_CLOCK_GPIO102,
    AM_HAL_TIMER_CLOCK_GPIO103,
    AM_HAL_TIMER_CLOCK_GPIO104,
    AM_HAL_TIMER_CLOCK_GPIO105,
    AM_HAL_TIMER_CLOCK_GPIO106,
    AM_HAL_TIMER_CLOCK_GPIO107,
    AM_HAL_TIMER_CLOCK_GPIO108,
    AM_HAL_TIMER_CLOCK_GPIO109,
    AM_HAL_TIMER_CLOCK_GPIO110,
    AM_HAL_TIMER_CLOCK_GPIO111,
    AM_HAL_TIMER_CLOCK_GPIO112,
    AM_HAL_TIMER_CLOCK_GPIO113,
    AM_HAL_TIMER_CLOCK_GPIO114,
    AM_HAL_TIMER_CLOCK_GPIO115,
    AM_HAL_TIMER_CLOCK_GPIO116,
    AM_HAL_TIMER_CLOCK_GPIO117,
    AM_HAL_TIMER_CLOCK_GPIO118,
    AM_HAL_TIMER_CLOCK_GPIO119,
    AM_HAL_TIMER_CLOCK_GPIO120,
    AM_HAL_TIMER_CLOCK_GPIO121,
    AM_HAL_TIMER_CLOCK_GPIO122,
    AM_HAL_TIMER_CLOCK_GPIO123,
    AM_HAL_TIMER_CLOCK_GPIO124,
    AM_HAL_TIMER_CLOCK_GPIO125,
    AM_HAL_TIMER_CLOCK_GPIO126,
    AM_HAL_TIMER_CLOCK_GPIO127     = TIMER_CTRL0_TMR0CLK_GPIO127,

}
am_hal_timer_clock_e;

//*****************************************************************************
//
//! TIMER Function enum
//
//*****************************************************************************
typedef enum
{
    AM_HAL_TIMER_FN_CONTINUOUS      = TIMER_CTRL0_TMR0FN_CONTINUOUS,
    AM_HAL_TIMER_FN_EDGE            = TIMER_CTRL0_TMR0FN_EDGE,
    AM_HAL_TIMER_FN_UPCOUNT         = TIMER_CTRL0_TMR0FN_UPCOUNT,
    AM_HAL_TIMER_FN_PWM             = TIMER_CTRL0_TMR0FN_PWM,
    AM_HAL_TIMER_FN_DOWNCOUNT       = TIMER_CTRL0_TMR0FN_DOWNCOUNT,
    AM_HAL_TIMER_FN_SINGLEPATTERN   = TIMER_CTRL0_TMR0FN_SINGLEPATTERN,
    AM_HAL_TIMER_FN_REPEATPATTERN   = TIMER_CTRL0_TMR0FN_REPEATPATTERN,
    AM_HAL_TIMER_FN_EVENTTIMER      = TIMER_CTRL0_TMR0FN_EVENTTIMER,
}
am_hal_timer_function_e;

//*****************************************************************************
//
//! TIMER Compare Selection
//
//*****************************************************************************
typedef enum
{
    AM_HAL_TIMER_COMPARE0 = 1,
    AM_HAL_TIMER_COMPARE1 = 2,
    AM_HAL_TIMER_COMPARE_BOTH = 3
}
am_hal_timer_compare_e;

//*****************************************************************************
//
//! TIMER Trigger Type
//
//*****************************************************************************
typedef enum
{
    AM_HAL_TIMER_TRIGGER_DIS  = TIMER_CTRL0_TMR0TMODE_DIS,
    AM_HAL_TIMER_TRIGGER_RISE = TIMER_CTRL0_TMR0TMODE_RISE,
    AM_HAL_TIMER_TRIGGER_FALL = TIMER_CTRL0_TMR0TMODE_FALL,
    AM_HAL_TIMER_TRIGGER_BOTH = TIMER_CTRL0_TMR0TMODE_BOTH,
}
am_hal_timer_trigger_type_e;

//*****************************************************************************
//
//! TIMER Trigger pins
//
//*****************************************************************************
typedef enum
{
    AM_HAL_TIMER_TRIGGER_TMR0_OUT0  = TIMER_MODE0_TMR0TRIGSEL_TMR00,
    AM_HAL_TIMER_TRIGGER_TMR0_OUT1  = TIMER_MODE0_TMR0TRIGSEL_TMR01,
    AM_HAL_TIMER_TRIGGER_TMR1_OUT0  = TIMER_MODE0_TMR0TRIGSEL_TMR10,
    AM_HAL_TIMER_TRIGGER_TMR1_OUT1  = TIMER_MODE0_TMR0TRIGSEL_TMR11,
    AM_HAL_TIMER_TRIGGER_TMR2_OUT0  = TIMER_MODE0_TMR0TRIGSEL_TMR20,
    AM_HAL_TIMER_TRIGGER_TMR2_OUT1  = TIMER_MODE0_TMR0TRIGSEL_TMR21,
    AM_HAL_TIMER_TRIGGER_TMR3_OUT0  = TIMER_MODE0_TMR0TRIGSEL_TMR30,
    AM_HAL_TIMER_TRIGGER_TMR3_OUT1  = TIMER_MODE0_TMR0TRIGSEL_TMR31,
    AM_HAL_TIMER_TRIGGER_TMR4_OUT0  = TIMER_MODE0_TMR0TRIGSEL_TMR40,
    AM_HAL_TIMER_TRIGGER_TMR4_OUT1  = TIMER_MODE0_TMR0TRIGSEL_TMR41,
    AM_HAL_TIMER_TRIGGER_TMR5_OUT0  = TIMER_MODE0_TMR0TRIGSEL_TMR50,
    AM_HAL_TIMER_TRIGGER_TMR5_OUT1  = TIMER_MODE0_TMR0TRIGSEL_TMR51,
    AM_HAL_TIMER_TRIGGER_TMR6_OUT0  = TIMER_MODE0_TMR0TRIGSEL_TMR60,
    AM_HAL_TIMER_TRIGGER_TMR6_OUT1  = TIMER_MODE0_TMR0TRIGSEL_TMR61,
    AM_HAL_TIMER_TRIGGER_TMR7_OUT0  = TIMER_MODE0_TMR0TRIGSEL_TMR70,
    AM_HAL_TIMER_TRIGGER_TMR7_OUT1  = TIMER_MODE0_TMR0TRIGSEL_TMR71,
    AM_HAL_TIMER_TRIGGER_TMR8_OUT0  = TIMER_MODE0_TMR0TRIGSEL_TMR80,
    AM_HAL_TIMER_TRIGGER_TMR8_OUT1  = TIMER_MODE0_TMR0TRIGSEL_TMR81,
    AM_HAL_TIMER_TRIGGER_TMR9_OUT0  = TIMER_MODE0_TMR0TRIGSEL_TMR90,
    AM_HAL_TIMER_TRIGGER_TMR9_OUT1  = TIMER_MODE0_TMR0TRIGSEL_TMR91,
    AM_HAL_TIMER_TRIGGER_TMR10_OUT0 = TIMER_MODE0_TMR0TRIGSEL_TMR100,
    AM_HAL_TIMER_TRIGGER_TMR10_OUT1 = TIMER_MODE0_TMR0TRIGSEL_TMR101,
    AM_HAL_TIMER_TRIGGER_TMR11_OUT0 = TIMER_MODE0_TMR0TRIGSEL_TMR110,
    AM_HAL_TIMER_TRIGGER_TMR11_OUT1 = TIMER_MODE0_TMR0TRIGSEL_TMR111,
    AM_HAL_TIMER_TRIGGER_TMR12_OUT0 = TIMER_MODE0_TMR0TRIGSEL_TMR120,
    AM_HAL_TIMER_TRIGGER_TMR12_OUT1 = TIMER_MODE0_TMR0TRIGSEL_TMR121,
    AM_HAL_TIMER_TRIGGER_TMR13_OUT0 = TIMER_MODE0_TMR0TRIGSEL_TMR130,
    AM_HAL_TIMER_TRIGGER_TMR13_OUT1 = TIMER_MODE0_TMR0TRIGSEL_TMR131,
    AM_HAL_TIMER_TRIGGER_TMR14_OUT0 = TIMER_MODE0_TMR0TRIGSEL_TMR140,
    AM_HAL_TIMER_TRIGGER_TMR14_OUT1 = TIMER_MODE0_TMR0TRIGSEL_TMR141,
    AM_HAL_TIMER_TRIGGER_TMR15_OUT0 = TIMER_MODE0_TMR0TRIGSEL_TMR150,
    AM_HAL_TIMER_TRIGGER_TMR15_OUT1 = TIMER_MODE0_TMR0TRIGSEL_TMR151,

    AM_HAL_TIMER_TRIGGER_GPIO0      = TIMER_MODE0_TMR0TRIGSEL_GPIO0,
    AM_HAL_TIMER_TRIGGER_GPIO1,
    AM_HAL_TIMER_TRIGGER_GPIO2,
    AM_HAL_TIMER_TRIGGER_GPIO3,
    AM_HAL_TIMER_TRIGGER_GPIO4,
    AM_HAL_TIMER_TRIGGER_GPIO5,
    AM_HAL_TIMER_TRIGGER_GPIO6,
    AM_HAL_TIMER_TRIGGER_GPIO7,
    AM_HAL_TIMER_TRIGGER_GPIO8,
    AM_HAL_TIMER_TRIGGER_GPIO9,
    AM_HAL_TIMER_TRIGGER_GPIO10,
    AM_HAL_TIMER_TRIGGER_GPIO11,
    AM_HAL_TIMER_TRIGGER_GPIO12,
    AM_HAL_TIMER_TRIGGER_GPIO13,
    AM_HAL_TIMER_TRIGGER_GPIO14,
    AM_HAL_TIMER_TRIGGER_GPIO15,
    AM_HAL_TIMER_TRIGGER_GPIO16,
    AM_HAL_TIMER_TRIGGER_GPIO17,
    AM_HAL_TIMER_TRIGGER_GPIO18,
    AM_HAL_TIMER_TRIGGER_GPIO19,
    AM_HAL_TIMER_TRIGGER_GPIO20,
    AM_HAL_TIMER_TRIGGER_GPIO21,
    AM_HAL_TIMER_TRIGGER_GPIO22,
    AM_HAL_TIMER_TRIGGER_GPIO23,
    AM_HAL_TIMER_TRIGGER_GPIO24,
    AM_HAL_TIMER_TRIGGER_GPIO25,
    AM_HAL_TIMER_TRIGGER_GPIO26,
    AM_HAL_TIMER_TRIGGER_GPIO27,
    AM_HAL_TIMER_TRIGGER_GPIO28,
    AM_HAL_TIMER_TRIGGER_GPIO29,
    AM_HAL_TIMER_TRIGGER_GPIO30,
    AM_HAL_TIMER_TRIGGER_GPIO31,
    AM_HAL_TIMER_TRIGGER_GPIO32,
    AM_HAL_TIMER_TRIGGER_GPIO33,
    AM_HAL_TIMER_TRIGGER_GPIO34,
    AM_HAL_TIMER_TRIGGER_GPIO35,
    AM_HAL_TIMER_TRIGGER_GPIO36,
    AM_HAL_TIMER_TRIGGER_GPIO37,
    AM_HAL_TIMER_TRIGGER_GPIO38,
    AM_HAL_TIMER_TRIGGER_GPIO39,
    AM_HAL_TIMER_TRIGGER_GPIO40,
    AM_HAL_TIMER_TRIGGER_GPIO41,
    AM_HAL_TIMER_TRIGGER_GPIO42,
    AM_HAL_TIMER_TRIGGER_GPIO43,
    AM_HAL_TIMER_TRIGGER_GPIO44,
    AM_HAL_TIMER_TRIGGER_GPIO45,
    AM_HAL_TIMER_TRIGGER_GPIO46,
    AM_HAL_TIMER_TRIGGER_GPIO47,
    AM_HAL_TIMER_TRIGGER_GPIO48,
    AM_HAL_TIMER_TRIGGER_GPIO49,
    AM_HAL_TIMER_TRIGGER_GPIO50,
    AM_HAL_TIMER_TRIGGER_GPIO51,
    AM_HAL_TIMER_TRIGGER_GPIO52,
    AM_HAL_TIMER_TRIGGER_GPIO53,
    AM_HAL_TIMER_TRIGGER_GPIO54,
    AM_HAL_TIMER_TRIGGER_GPIO55,
    AM_HAL_TIMER_TRIGGER_GPIO56,
    AM_HAL_TIMER_TRIGGER_GPIO57,
    AM_HAL_TIMER_TRIGGER_GPIO58,
    AM_HAL_TIMER_TRIGGER_GPIO59,
    AM_HAL_TIMER_TRIGGER_GPIO60,
    AM_HAL_TIMER_TRIGGER_GPIO61,
    AM_HAL_TIMER_TRIGGER_GPIO62,
    AM_HAL_TIMER_TRIGGER_GPIO63,
    AM_HAL_TIMER_TRIGGER_GPIO64,
    AM_HAL_TIMER_TRIGGER_GPIO65,
    AM_HAL_TIMER_TRIGGER_GPIO66,
    AM_HAL_TIMER_TRIGGER_GPIO67,
    AM_HAL_TIMER_TRIGGER_GPIO68,
    AM_HAL_TIMER_TRIGGER_GPIO69,
    AM_HAL_TIMER_TRIGGER_GPIO70,
    AM_HAL_TIMER_TRIGGER_GPIO71,
    AM_HAL_TIMER_TRIGGER_GPIO72,
    AM_HAL_TIMER_TRIGGER_GPIO73,
    AM_HAL_TIMER_TRIGGER_GPIO74,
    AM_HAL_TIMER_TRIGGER_GPIO75,
    AM_HAL_TIMER_TRIGGER_GPIO76,
    AM_HAL_TIMER_TRIGGER_GPIO77,
    AM_HAL_TIMER_TRIGGER_GPIO78,
    AM_HAL_TIMER_TRIGGER_GPIO79,
    AM_HAL_TIMER_TRIGGER_GPIO80,
    AM_HAL_TIMER_TRIGGER_GPIO81,
    AM_HAL_TIMER_TRIGGER_GPIO82,
    AM_HAL_TIMER_TRIGGER_GPIO83,
    AM_HAL_TIMER_TRIGGER_GPIO84,
    AM_HAL_TIMER_TRIGGER_GPIO85,
    AM_HAL_TIMER_TRIGGER_GPIO86,
    AM_HAL_TIMER_TRIGGER_GPIO87,
    AM_HAL_TIMER_TRIGGER_GPIO88,
    AM_HAL_TIMER_TRIGGER_GPIO89,
    AM_HAL_TIMER_TRIGGER_GPIO90,
    AM_HAL_TIMER_TRIGGER_GPIO91,
    AM_HAL_TIMER_TRIGGER_GPIO92,
    AM_HAL_TIMER_TRIGGER_GPIO93,
    AM_HAL_TIMER_TRIGGER_GPIO94,
    AM_HAL_TIMER_TRIGGER_GPIO95,
    AM_HAL_TIMER_TRIGGER_GPIO96,
    AM_HAL_TIMER_TRIGGER_GPIO97,
    AM_HAL_TIMER_TRIGGER_GPIO98,
    AM_HAL_TIMER_TRIGGER_GPIO99,
    AM_HAL_TIMER_TRIGGER_GPIO100,
    AM_HAL_TIMER_TRIGGER_GPIO101,
    AM_HAL_TIMER_TRIGGER_GPIO102,
    AM_HAL_TIMER_TRIGGER_GPIO103,
    AM_HAL_TIMER_TRIGGER_GPIO104,
    AM_HAL_TIMER_TRIGGER_GPIO105,
    AM_HAL_TIMER_TRIGGER_GPIO106,
    AM_HAL_TIMER_TRIGGER_GPIO107,
    AM_HAL_TIMER_TRIGGER_GPIO108,
    AM_HAL_TIMER_TRIGGER_GPIO109,
    AM_HAL_TIMER_TRIGGER_GPIO110,
    AM_HAL_TIMER_TRIGGER_GPIO111,
    AM_HAL_TIMER_TRIGGER_GPIO112,
    AM_HAL_TIMER_TRIGGER_GPIO113,
    AM_HAL_TIMER_TRIGGER_GPIO114,
    AM_HAL_TIMER_TRIGGER_GPIO115,
    AM_HAL_TIMER_TRIGGER_GPIO116,
    AM_HAL_TIMER_TRIGGER_GPIO117,
    AM_HAL_TIMER_TRIGGER_GPIO118,
    AM_HAL_TIMER_TRIGGER_GPIO119,
    AM_HAL_TIMER_TRIGGER_GPIO120,
    AM_HAL_TIMER_TRIGGER_GPIO121,
    AM_HAL_TIMER_TRIGGER_GPIO122,
    AM_HAL_TIMER_TRIGGER_GPIO123,
    AM_HAL_TIMER_TRIGGER_GPIO124,
    AM_HAL_TIMER_TRIGGER_GPIO125,
    AM_HAL_TIMER_TRIGGER_GPIO126,
    AM_HAL_TIMER_TRIGGER_GPIO127     = TIMER_MODE0_TMR0TRIGSEL_GPIO127,
}
am_hal_timer_trigger_source_e;

//*****************************************************************************
//
//! TIMER Output
//
//*****************************************************************************
typedef enum
{
    AM_HAL_TIMER_OUTPUT_TMR0_OUT0   =   TIMER_OUTCFG0_OUTCFG0_TIMER00,
    AM_HAL_TIMER_OUTPUT_TMR0_OUT1   =   TIMER_OUTCFG0_OUTCFG0_TIMER01,
    AM_HAL_TIMER_OUTPUT_TMR1_OUT0   =   TIMER_OUTCFG0_OUTCFG0_TIMER10,
    AM_HAL_TIMER_OUTPUT_TMR1_OUT1   =   TIMER_OUTCFG0_OUTCFG0_TIMER11,
    AM_HAL_TIMER_OUTPUT_TMR2_OUT0   =   TIMER_OUTCFG0_OUTCFG0_TIMER20,
    AM_HAL_TIMER_OUTPUT_TMR2_OUT1   =   TIMER_OUTCFG0_OUTCFG0_TIMER21,
    AM_HAL_TIMER_OUTPUT_TMR3_OUT0   =   TIMER_OUTCFG0_OUTCFG0_TIMER30,
    AM_HAL_TIMER_OUTPUT_TMR3_OUT1   =   TIMER_OUTCFG0_OUTCFG0_TIMER31,
    AM_HAL_TIMER_OUTPUT_TMR4_OUT0   =   TIMER_OUTCFG0_OUTCFG0_TIMER40,
    AM_HAL_TIMER_OUTPUT_TMR4_OUT1   =   TIMER_OUTCFG0_OUTCFG0_TIMER41,
    AM_HAL_TIMER_OUTPUT_TMR5_OUT0   =   TIMER_OUTCFG0_OUTCFG0_TIMER50,
    AM_HAL_TIMER_OUTPUT_TMR5_OUT1   =   TIMER_OUTCFG0_OUTCFG0_TIMER51,
    AM_HAL_TIMER_OUTPUT_TMR6_OUT0   =   TIMER_OUTCFG0_OUTCFG0_TIMER60,
    AM_HAL_TIMER_OUTPUT_TMR6_OUT1   =   TIMER_OUTCFG0_OUTCFG0_TIMER61,
    AM_HAL_TIMER_OUTPUT_TMR7_OUT0   =   TIMER_OUTCFG0_OUTCFG0_TIMER70,
    AM_HAL_TIMER_OUTPUT_TMR7_OUT1   =   TIMER_OUTCFG0_OUTCFG0_TIMER71,
    AM_HAL_TIMER_OUTPUT_TMR8_OUT0   =   TIMER_OUTCFG0_OUTCFG0_TIMER80,
    AM_HAL_TIMER_OUTPUT_TMR8_OUT1   =   TIMER_OUTCFG0_OUTCFG0_TIMER81,
    AM_HAL_TIMER_OUTPUT_TMR9_OUT0   =   TIMER_OUTCFG0_OUTCFG0_TIMER90,
    AM_HAL_TIMER_OUTPUT_TMR9_OUT1   =   TIMER_OUTCFG0_OUTCFG0_TIMER91,
    AM_HAL_TIMER_OUTPUT_TMR10_OUT0  =   TIMER_OUTCFG0_OUTCFG0_TIMER100,
    AM_HAL_TIMER_OUTPUT_TMR10_OUT1  =   TIMER_OUTCFG0_OUTCFG0_TIMER101,
    AM_HAL_TIMER_OUTPUT_TMR11_OUT0  =   TIMER_OUTCFG0_OUTCFG0_TIMER110,
    AM_HAL_TIMER_OUTPUT_TMR11_OUT1  =   TIMER_OUTCFG0_OUTCFG0_TIMER111,
    AM_HAL_TIMER_OUTPUT_TMR12_OUT0  =   TIMER_OUTCFG0_OUTCFG0_TIMER120,
    AM_HAL_TIMER_OUTPUT_TMR12_OUT1  =   TIMER_OUTCFG0_OUTCFG0_TIMER121,
    AM_HAL_TIMER_OUTPUT_TMR13_OUT0  =   TIMER_OUTCFG0_OUTCFG0_TIMER130,
    AM_HAL_TIMER_OUTPUT_TMR13_OUT1  =   TIMER_OUTCFG0_OUTCFG0_TIMER131,
    AM_HAL_TIMER_OUTPUT_TMR14_OUT0  =   TIMER_OUTCFG0_OUTCFG0_TIMER140,
    AM_HAL_TIMER_OUTPUT_TMR14_OUT1  =   TIMER_OUTCFG0_OUTCFG0_TIMER141,
    AM_HAL_TIMER_OUTPUT_TMR15_OUT0  =   TIMER_OUTCFG0_OUTCFG0_TIMER150,
    AM_HAL_TIMER_OUTPUT_TMR15_OUT1  =   TIMER_OUTCFG0_OUTCFG0_TIMER151,
    AM_HAL_TIMER_OUTPUT_STIMER0     =   TIMER_OUTCFG0_OUTCFG0_STIMER0,
    AM_HAL_TIMER_OUTPUT_STIMER1     =   TIMER_OUTCFG0_OUTCFG0_STIMER1,
    AM_HAL_TIMER_OUTPUT_STIMER2     =   TIMER_OUTCFG0_OUTCFG0_STIMER2,
    AM_HAL_TIMER_OUTPUT_STIMER3     =   TIMER_OUTCFG0_OUTCFG0_STIMER3,
    AM_HAL_TIMER_OUTPUT_STIMER4     =   TIMER_OUTCFG0_OUTCFG0_STIMER4,
    AM_HAL_TIMER_OUTPUT_STIMER5     =   TIMER_OUTCFG0_OUTCFG0_STIMER5,
    AM_HAL_TIMER_OUTPUT_STIMER6     =   TIMER_OUTCFG0_OUTCFG0_STIMER6,
    AM_HAL_TIMER_OUTPUT_STIMER7     =   TIMER_OUTCFG0_OUTCFG0_STIMER7,
}
am_hal_timer_output_e;

//*****************************************************************************
//
//! TIMER configuration structure.
//
//*****************************************************************************
typedef struct
{
    am_hal_timer_clock_e                eInputClock;            // Input Clock
    am_hal_timer_function_e             eFunction;              // Function
    bool                                bInvertOutput0;         // Output0 polarity.
    bool                                bInvertOutput1;         // Output1 polarity.
    am_hal_timer_trigger_type_e         eTriggerType;           // Rising, Falling, Both, None.
    am_hal_timer_trigger_source_e       eTriggerSource;         // Input Trigger Source.
    bool                                bLowJitter;             // Asynch to bus to provide low-jitter clock.
    uint32_t                            ui32PatternLimit;       // End of pattern count.

    // Function dependent comparator values.  See register definitions and datasheet.
    uint32_t                            ui32Compare0;           // Primary comparator value.
    uint32_t                            ui32Compare1;           // Secondary comparator value.
}
am_hal_timer_config_t;

//*****************************************************************************
//
//! TIMER interrupt macros
//! @name timer_interrupt_macros
//! @{
//
//*****************************************************************************
#define AM_HAL_TIMER_INT_TMR0_CMP0    TIMER_INTSTAT_TMR00INT_Msk
#define AM_HAL_TIMER_INT_TMR0_CMP1    TIMER_INTSTAT_TMR01INT_Msk
#define AM_HAL_TIMER_INT_TMR1_CMP0    TIMER_INTSTAT_TMR10INT_Msk
#define AM_HAL_TIMER_INT_TMR1_CMP1    TIMER_INTSTAT_TMR11INT_Msk
#define AM_HAL_TIMER_INT_TMR2_CMP0    TIMER_INTSTAT_TMR20INT_Msk
#define AM_HAL_TIMER_INT_TMR2_CMP1    TIMER_INTSTAT_TMR21INT_Msk
#define AM_HAL_TIMER_INT_TMR3_CMP0    TIMER_INTSTAT_TMR30INT_Msk
#define AM_HAL_TIMER_INT_TMR3_CMP1    TIMER_INTSTAT_TMR31INT_Msk
#define AM_HAL_TIMER_INT_TMR4_CMP0    TIMER_INTSTAT_TMR40INT_Msk
#define AM_HAL_TIMER_INT_TMR4_CMP1    TIMER_INTSTAT_TMR41INT_Msk
#define AM_HAL_TIMER_INT_TMR5_CMP0    TIMER_INTSTAT_TMR50INT_Msk
#define AM_HAL_TIMER_INT_TMR5_CMP1    TIMER_INTSTAT_TMR51INT_Msk
#define AM_HAL_TIMER_INT_TMR6_CMP0    TIMER_INTSTAT_TMR60INT_Msk
#define AM_HAL_TIMER_INT_TMR6_CMP1    TIMER_INTSTAT_TMR61INT_Msk
#define AM_HAL_TIMER_INT_TMR7_CMP0    TIMER_INTSTAT_TMR70INT_Msk
#define AM_HAL_TIMER_INT_TMR7_CMP1    TIMER_INTSTAT_TMR71INT_Msk
#define AM_HAL_TIMER_INT_TMR8_CMP0    TIMER_INTSTAT_TMR80INT_Msk
#define AM_HAL_TIMER_INT_TMR8_CMP1    TIMER_INTSTAT_TMR81INT_Msk
#define AM_HAL_TIMER_INT_TMR9_CMP0    TIMER_INTSTAT_TMR90INT_Msk
#define AM_HAL_TIMER_INT_TMR9_CMP1    TIMER_INTSTAT_TMR91INT_Msk
#define AM_HAL_TIMER_INT_TMR10_CMP0   TIMER_INTSTAT_TMR100INT_Msk
#define AM_HAL_TIMER_INT_TMR10_CMP1   TIMER_INTSTAT_TMR101INT_Msk
#define AM_HAL_TIMER_INT_TMR11_CMP0   TIMER_INTSTAT_TMR110INT_Msk
#define AM_HAL_TIMER_INT_TMR11_CMP1   TIMER_INTSTAT_TMR111INT_Msk
#define AM_HAL_TIMER_INT_TMR12_CMP0   TIMER_INTSTAT_TMR120INT_Msk
#define AM_HAL_TIMER_INT_TMR12_CMP1   TIMER_INTSTAT_TMR121INT_Msk
#define AM_HAL_TIMER_INT_TMR13_CMP0   TIMER_INTSTAT_TMR130INT_Msk
#define AM_HAL_TIMER_INT_TMR13_CMP1   TIMER_INTSTAT_TMR131INT_Msk
#define AM_HAL_TIMER_INT_TMR14_CMP0   TIMER_INTSTAT_TMR140INT_Msk
#define AM_HAL_TIMER_INT_TMR14_CMP1   TIMER_INTSTAT_TMR141INT_Msk
#define AM_HAL_TIMER_INT_TMR15_CMP0   TIMER_INTSTAT_TMR150INT_Msk
#define AM_HAL_TIMER_INT_TMR15_CMP1   TIMER_INTSTAT_TMR151INT_Msk
//! @}

//*****************************************************************************
//
//! @brief Configure a TIMER
//!
//! @param ui32TimerNumber refers to one of the numbered TIMERs in the module.
//! @param psTimerConfig is a structure describing a timer configuration.
//!
//! Use this function to set important qualities about a TIMER, such as its
//! clock, compare values, and output pin configuration.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_timer_config(uint32_t ui32TimerNumber,
                                    am_hal_timer_config_t *psTimerConfig);

//*****************************************************************************
//
//! @brief Initialize a timer configuration structure with default values.
//!
//! @param psTimerConfig is a structure describing a timer configuration.
//!
//! This function will set the members of a timer config structure to default
//! values as follows:
//!    psTimerConfig->eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV4;
//!    psTimerConfig->eFunction = AM_HAL_TIMER_FN_EDGE;
//!    psTimerConfig->ui32Compare0 = 0xFFFFFFFF;
//!    psTimerConfig->ui32Compare1 = 0xFFFFFFFF;
//!    psTimerConfig->bInvertOutput0 = false;
//!    psTimerConfig->bInvertOutput1 = false;
//!    psTimerConfig->eTriggerType = AM_HAL_TIMER_TRIGGER_DIS;
//!    psTimerConfig->eTriggerSource = AM_HAL_TIMER_TRIGGER_TMR0_CMP0;
//!    psTimerConfig->ui32PatternLimit = 0;

//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_timer_default_config_set(am_hal_timer_config_t *psTimerConfig);

//*****************************************************************************
//
//! @brief Reset a timer configuration to the power up state.
//!
//! @param ui32TimerNumber is the number of the TIMER to enable.
//!
//! This function will reset the given timer to the power up configuration.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_timer_reset_config(uint32_t ui32TimerNumber);

//*****************************************************************************
//
//! @brief Enable a single TIMER
//!
//! @param ui32TimerNumber is the number of the TIMER to enable.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_timer_enable(uint32_t ui32TimerNumber);

//*****************************************************************************
//
//! @brief Disable a single TIMER
//!
//! @param ui32TimerNumber is the number of the TIMER to disable.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_timer_disable(uint32_t ui32TimerNumber);

//*****************************************************************************
//
//! @brief Enable a group of TIMERS all at once
//!
//! @param ui32TimerMask is a set of TIMERs to enable.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_timer_enable_sync(uint32_t ui32TimerMask);

//*****************************************************************************
//
//! @brief Enable a group of TIMERS all at once
//!
//! @param ui32TimerMask is a set of TIMERs to enable.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_timer_disable_sync(uint32_t ui32TimerMask);

//*****************************************************************************
//
//! @brief Start a single TIMER
//!
//! @param ui32TimerNumber is the number of the timer to use.
//!
//! Call this definition to start the timer.
//!
//
//*****************************************************************************
#define am_hal_timer_start(ui32TimerNumber)   am_hal_timer_enable(ui32TimerNumber)

//*****************************************************************************
//
//! @brief Disable a single TIMER
//!
//! @param ui32TimerNumber is the number of the timer to use.
//!
//! This definition will stop the timer.
//
//*****************************************************************************
#define am_hal_timer_stop(ui32TimerNumber)    am_hal_timer_disable(ui32TimerNumber)

//*****************************************************************************
//
//! @brief Clear a single TIMER and start the timer.
//!
//! @param ui32TimerNumber is the number of the timer to use.
//!
//! This function will reset a timer to its "start" value. For count-up timers
//! this will be zero, and for count-down timers this will be the value
//! COMPARE0.
//!
//! After clearing the timer, the timer is started in this function.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_timer_clear(uint32_t ui32TimerNumber);

//*****************************************************************************
//
//! @brief Clear a single TIMER, but don't start it.
//!
//! @param ui32TimerNumber is the number of the timer to use.
//!
//! This function will reset a timer to its "start" value. For count-up timers
//! this will be zero, and for count-down timers this will be the value
//! COMPARE0.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_timer_clear_stop(uint32_t ui32TimerNumber);

//*****************************************************************************
//
//! @brief Read the current value of a timer.
//!
//! @param ui32TimerNumber is the number of the timer to use.
//!
//! This function returns the 32-bit count value of a timer.
//!
//! @return Current value of the timer.
//
//*****************************************************************************
extern uint32_t am_hal_timer_read(uint32_t ui32TimerNumber);

//*****************************************************************************
//
//! @brief Set the COMPARE0 value for a single timer.
//!
//! @param ui32TimerNumber is the number of the timer to use.
//! @param ui32CompareValue is the value to use for COMPARE0
//!
//! This function will set COMPARE0 for the selected timer. COMPARE0 controls
//! the roll-over value for the selcted timer (or the stop value for
//! single-shot timers). This change is done "on the fly" without disabling
//! the timer for use with the DOWNCOUNT function.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_timer_compare0_set(uint32_t ui32TimerNumber,
                                          uint32_t ui32CompareValue);

//*****************************************************************************
//
//! @brief Set the COMPARE1 value for a single timer.
//!
//! @param ui32TimerNumber is the number of the timer to use.
//! @param ui32CompareValue is the value to use for COMPARE1
//!
//! This function will set COMPARE1 for the selected timer. COMPARE1 is used to
//! generate interrupts and output level shifts for a timer values between zero
//! and COMPARE0. Check the description of your selected TIMER mode for a
//! precise description of the function of COMPARE1.  This change is done
//! "on the fly" without disabling the timer for use with the DOWNCOUNT function.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_timer_compare1_set(uint32_t ui32TimerNumber,
                                          uint32_t ui32CompareValue);

//*****************************************************************************
//
//! @brief TIMER enable interrupts function
//!
//! @param ui32InterruptMask  - interface specific interrupt mask.
//!
//! This function enables the specific indicated interrupts (see above).
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_timer_interrupt_enable(uint32_t ui32InterruptMask);

//*****************************************************************************
//
//! @brief TIMER disable interrupts function
//!
//! @param ui32InterruptMask  - interface specific interrupt mask.
//!
//! This function disables the specified interrupts.
//!
//! @return status      - generic or interface specific status.
//!
//*****************************************************************************
extern uint32_t am_hal_timer_interrupt_disable(uint32_t ui32InterruptMask);

//*****************************************************************************
//
//! @brief TIMER get interrupt status
//!
//! @param bEnabledOnly   - If interrupt is enabled
//! @param pui32IntStatus - pointer to a uint32_t to return the interrupt status
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_timer_interrupt_status_get(bool bEnabledOnly, uint32_t *pui32IntStatus);

//*****************************************************************************
//
//! @brief TIMER interrupt clear
//!
//! @param ui32InterruptMask  - interface specific interrupt mask.
//!
//! This function clears the interrupts for the given peripheral.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_timer_interrupt_clear(uint32_t ui32InterruptMask);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_TIMER_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

