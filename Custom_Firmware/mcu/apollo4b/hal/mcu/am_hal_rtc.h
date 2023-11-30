//*****************************************************************************
//
//! @file am_hal_rtc.h
//!
//! @brief Functions for interfacing with the Real-Time Clock (RTC).
//!
//! @addtogroup rtc4_4b RTC - Real-Time Clock
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
#ifndef AM_HAL_RTC_H
#define AM_HAL_RTC_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name Oscillator Selector.
//! @{
//! Options for the RTC oscillator.
//!
//! Used with am_hal_rtc_config
//
//*****************************************************************************
typedef enum
{
    AM_HAL_RTC_OSC_XT   = 0x0,
}
am_hal_rtc_osc_select_e;
//! @}

//*****************************************************************************
//
//! @name RTC Interrupts
//! @{
//! Macro definitions for RTC interrupt status bits.
//!
//! These macros correspond to the bits in the RTC interrupt status register.
//! They may be used with any of the \e am_hal_rtc_int_x() functions.
//!
//
//*****************************************************************************
#define AM_HAL_RTC_INT_ALM                  RTC_INTEN_ALM_Msk
//! @}

//*****************************************************************************
//
//! @name RTC Alarm Repeat Interval.
//! @{
//! Macro definitions for the RTC alarm repeat interval.
//!
//! These macros correspond to the RPT bits in the RTCCTL register.
//! They may be used with the \e am_hal_rtc_alarm_interval_set() function.
//!
//! Note: AM_HAL_RTC_ALM_RPT_10TH and AM_HAL_RTC_ALM_RPT_100TH do not
//! correspond to the RPT bits but are used in conjunction with setting the
//! ALM100 bits in the ALMLOW register.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_RTC_ALM_RPT_DIS = RTC_RTCCTL_RPT_DIS,
    AM_HAL_RTC_ALM_RPT_YR  = RTC_RTCCTL_RPT_YEAR,
    AM_HAL_RTC_ALM_RPT_MTH = RTC_RTCCTL_RPT_MONTH,
    AM_HAL_RTC_ALM_RPT_WK  = RTC_RTCCTL_RPT_WEEK,
    AM_HAL_RTC_ALM_RPT_DAY = RTC_RTCCTL_RPT_DAY,
    AM_HAL_RTC_ALM_RPT_HR  = RTC_RTCCTL_RPT_HR,
    AM_HAL_RTC_ALM_RPT_MIN = RTC_RTCCTL_RPT_MIN,
    AM_HAL_RTC_ALM_RPT_SEC = RTC_RTCCTL_RPT_SEC,
    AM_HAL_RTC_ALM_RPT_10TH,
    AM_HAL_RTC_ALM_RPT_100TH,
}
am_hal_rtc_alarm_repeat_e;
//! @}

//*****************************************************************************
//
// RTC configuration structure.
//
//*****************************************************************************
typedef struct am_hal_rtc_config_struct
{
    am_hal_rtc_osc_select_e eOscillator;
    bool b12Hour;
}
am_hal_rtc_config_t;

//*****************************************************************************
//
//! @brief The basic time structure used by the HAL for RTC interaction.
//!
//! All values are positive whole numbers. The HAL routines convert back and
//! forth to BCD.
//
//*****************************************************************************
typedef struct am_hal_rtc_time_struct
{
    uint32_t ui32ReadError;
    uint32_t ui32CenturyEnable;
    uint32_t ui32Weekday;
    uint32_t ui32Century;
    uint32_t ui32Year;
    uint32_t ui32Month;
    uint32_t ui32DayOfMonth;
    uint32_t ui32Hour;
    uint32_t ui32Minute;
    uint32_t ui32Second;
    uint32_t ui32Hundredths;
}
am_hal_rtc_time_t;

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Configures the RTC
//!
//! @param psConfig is a pointer to a configuration structure for the RTC.
//!
//! Configures the oscillator and 12/24hr settings for the RTC. See the \e
//! am_hal_rtc_config_t structure for more information.
//!
//! @return AM_HAL_STATUS_SUCCESS or relevant HAL error code.
//
//*****************************************************************************
extern uint32_t am_hal_rtc_config(const am_hal_rtc_config_t *psConfig);

//*****************************************************************************
//
//! @brief Enable/Start the RTC oscillator.
//!
//! Starts the RTC oscillator.
//!
//! @return AM_HAL_STATUS_SUCCESS or relevant HAL error code.
//
//*****************************************************************************
extern uint32_t am_hal_rtc_osc_enable(void);

//*****************************************************************************
//
//! @brief Disable/Stop the RTC oscillator.
//!
//! Stops the RTC oscillator.
//!
//! @return AM_HAL_STATUS_SUCCESS or relevant HAL error code.
//
//*****************************************************************************
extern uint32_t am_hal_rtc_osc_disable(void);

//*****************************************************************************
//
//! @brief Enable/Start the RTC.
//!
//! Starts the RTC.
//!
//! @return AM_HAL_STATUS_SUCCESS or relevant HAL error code.
//
//*****************************************************************************
extern uint32_t am_hal_rtc_enable(void);

//*****************************************************************************
//
//! @brief Disable/Stop the RTC.
//!
//! Stops the RTC.
//!
//! @return AM_HAL_STATUS_SUCCESS or relevant HAL error code.
//
//*****************************************************************************
extern uint32_t am_hal_rtc_disable(void);

//*****************************************************************************
//
//! @brief Set the Real Time Clock counter registers.
//!
//! @param *pTime - A pointer to the time structure.
//!
//! Sets the RTC counter registers to the supplied values.
//!
//! @return AM_HAL_STATUS_SUCCESS or relevant HAL error code.
//
//*****************************************************************************
extern uint32_t am_hal_rtc_time_set(am_hal_rtc_time_t *pTime);

//*****************************************************************************
//
//! @brief Get the Real Time Clock current time.
//!
//! @param *pTime - A pointer to the time structure to store the current time.
//!
//! Gets the RTC's current time
//!
//! @return AM_HAL_STATUS_SUCCESS or relevant HAL error code.
//
//*****************************************************************************
extern uint32_t am_hal_rtc_time_get(am_hal_rtc_time_t *pTime);

//*****************************************************************************
//
//! @brief Sets the RTC's Alarm.
//!
//! @param *pTime - A pointer to the time structure.
//! @param eRepeatInterval - the desired alarm repeat interval.
//!
//! Set the Real Time Clock Alarm Parameters.
//!
//! @return AM_HAL_STATUS_SUCCESS or relevant HAL error code.
//
//*****************************************************************************
extern uint32_t am_hal_rtc_alarm_set(am_hal_rtc_time_t *pTime,
                                     am_hal_rtc_alarm_repeat_e eRepeatInterval);

//*****************************************************************************
//
//! @brief Get the Real Time Clock Alarm Parameters
//!
//! @param *pTime - A pointer to the time structure to store the current alarm.
//! @param peRepeatInterval - pointer to the desired alarm repeat interval.
//!
//! Retrieves the RTC's current alarm time and repeat interval settings.
//!
//! @return AM_HAL_STATUS_SUCCESS or relevant HAL error code.
//
//*****************************************************************************
extern uint32_t am_hal_rtc_alarm_get(am_hal_rtc_time_t *pTime,
                                     am_hal_rtc_alarm_repeat_e *peRepeatInterval);

//*****************************************************************************
//
//! @brief Enable selected RTC interrupts.
//!
//! @param ui32InterruptMask - desired interrupts
//!
//! Enables the RTC interrupts. \e ui32InterruptMask should be set to a
//! logical OR of one or more of the following:
//!
//!     AM_HAL_RTC_INT_ALM
//!
//! @return AM_HAL_STATUS_SUCCESS or relevant HAL error code.
//
//*****************************************************************************
extern uint32_t am_hal_rtc_interrupt_enable(uint32_t ui32InterruptMask);

//*****************************************************************************
//
//! @brief Return the enabled RTC interrupts.
//!
//! @param pui32InterruptMask is a pointer where the current RTC interrupt
//!        enable mask will be written.
//!
//! Returns the enabled RTC interrupts. \e pui32InterruptMask will be set to a
//! logical OR of one or more of the following:
//!
//!     AM_HAL_RTC_INT_ALM
//!
//! @return AM_HAL_STATUS_SUCCESS or relevant HAL error code.
//!
//
//*****************************************************************************
extern uint32_t am_hal_rtc_interrupt_enable_get(uint32_t *pui32InterruptMask);

//*****************************************************************************
//
//! @brief Disable selected RTC interrupts.
//!
//! @param ui32InterruptMask - desired interrupts
//!
//! Disables the RTC interrupts.
//!
//! ui32Interrupt should be an OR of the following:
//!
//!     AM_HAL_RTC_INT_ALM
//!
//! @return AM_HAL_STATUS_SUCCESS or relevant HAL error code.
//
//*****************************************************************************
extern uint32_t am_hal_rtc_interrupt_disable(uint32_t ui32InterruptMask);

//*****************************************************************************
//
//! @brief Clear selected RTC interrupts.
//!
//! @param ui32InterruptMask - desired interrupts
//!
//! Clears the RTC interrupts.
//!
//! ui32Interrupt should be an OR of the following:
//!
//!     AM_HAL_RTC_INT_ALM
//!
//! @return AM_HAL_STATUS_SUCCESS or relevant HAL error code.
//
//*****************************************************************************
extern uint32_t am_hal_rtc_interrupt_clear(uint32_t ui32InterruptMask);

//*****************************************************************************
//
//! @brief Sets the selected RTC interrupts.
//!
//! @param ui32InterruptMask - desired interrupts
//!
//! Sets the RTC interrupts causing them to immediately trigger.
//!
//! ui32Interrupt should be an OR of the following:
//!
//!     AM_HAL_RTC_INT_ALM
//!
//! @return AM_HAL_STATUS_SUCCESS or relevant HAL error code.
//
//*****************************************************************************
extern uint32_t am_hal_rtc_interrupt_set(uint32_t ui32InterruptMask);

//*****************************************************************************
//
//! @brief Returns the RTC interrupt status.
//!
//! @param bEnabledOnly if true, return the status of enabled interrupts only.
//! @param pui32InterruptMask pointer where the interrupt status will be
//!        written.
//!
//! This function will write the current RTC interrupt status to \e
//! pui32InterruptMask. The interrupt status value will be the logical OR of
//! one or more of the following:
//!
//!     AM_HAL_RTC_INT_ALM
//!
//! @return AM_HAL_STATUS_SUCCESS or relevant HAL error code.
//
//*****************************************************************************
extern uint32_t am_hal_rtc_interrupt_status_get(bool bEnabledOnly,
                                                uint32_t *pui32InterruptMask);

#if 1
//*****************************************************************************
//
//! @brief Selects the clock source for the RTC.
//!
//! @param ui32OSC the clock source for the RTC.
//!
//! This function selects the clock source for the RTC.
//!
//! Valid values for ui32OSC are:
//!
//!     AM_HAL_RTC_OSC_LFRC
//!     AM_HAL_RTC_OSC_XT
//!
//! @note After selection of the RTC oscillator, a 2 second delay occurs before
//! the new setting is reflected in status. Therefore the CLKGEN.STATUS.OMODE
//! bit will not reflect the new status until after the 2s wait period.
//!
//
//*****************************************************************************
extern void am_hal_rtc_osc_select(uint32_t ui32OSC);

//*****************************************************************************
//
//! @brief Sets the alarm repeat interval.
//!
//! @param ui32RepeatInterval the desired repeat interval.
//!
//! Sets the alarm repeat interval.
//!
//! Valid values for ui32RepeatInterval:
//!
//!     AM_HAL_RTC_ALM_RPT_DIS
//!     AM_HAL_RTC_ALM_RPT_YR
//!     AM_HAL_RTC_ALM_RPT_MTH
//!     AM_HAL_RTC_ALM_RPT_WK
//!     AM_HAL_RTC_ALM_RPT_DAY
//!     AM_HAL_RTC_ALM_RPT_HR
//!     AM_HAL_RTC_ALM_RPT_MIN
//!     AM_HAL_RTC_ALM_RPT_SEC
//!     AM_HAL_RTC_ALM_RPT_10TH
//!     AM_HAL_RTC_ALM_RPT_100TH
//!
//
//*****************************************************************************
extern void am_hal_rtc_alarm_interval_set(uint32_t ui32RepeatInterval);
#endif

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_RTC_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

