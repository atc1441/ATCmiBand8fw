//*****************************************************************************
//
//! @file am_hal_wdt.h
//!
//! @brief Watchdog Timer
//!
//! @addtogroup wdt_4b WDT - Watchdog Timer Functionality
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
#ifndef AM_HAL_WDT_H
#define AM_HAL_WDT_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @brief Convenience macros for watchdog locking/restarting.
//! @name watchdog_macros
//! @{
//
//*****************************************************************************
#define AM_HAL_WDT_MCU_LOCK_KEY      WDT_LOCK_LOCK_KEYVALUE
#define AM_HAL_WDT_DSP0_LOCK_KEY     WDT_DSP0TLOCK_DSP0LOCK_KEYVALUE
#define AM_HAL_WDT_DSP1_LOCK_KEY     WDT_DSP1TLOCK_DSP1LOCK_KEYVALUE

#define AM_HAL_WDT_MCU_RESTART_KEY   WDT_RSTRT_RSTRT_KEYVALUE
#define AM_HAL_WDT_DSP0_RESTART_KEY  WDT_DSP0RSTRT_DSP0RSTART_KEYVALUE
#define AM_HAL_WDT_DSP1_RESTART_KEY  WDT_DSP1RSTRT_DSP1RSTART_KEYVALUE
//! @}

//*****************************************************************************
//
//! Enumeration used to select a watchdog timer.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_WDT_MCU,
    AM_HAL_WDT_DSP0,
    AM_HAL_WDT_DSP1,
}
am_hal_wdt_select_e;

//*****************************************************************************
//
//! Enumeration used to select a watchdog clock source.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_WDT_OFF    = WDT_CFG_CLKSEL_OFF,
    AM_HAL_WDT_128HZ  = WDT_CFG_CLKSEL_128HZ,
    AM_HAL_WDT_16HZ   = WDT_CFG_CLKSEL_16HZ,
    AM_HAL_WDT_1HZ    = WDT_CFG_CLKSEL_1HZ,
    AM_HAL_WDT_1_16HZ = WDT_CFG_CLKSEL_1_16HZ,
}
am_hal_wdt_clock_select_e;

//*****************************************************************************
//
//! Watchdog timer configuration structure.
//
//*****************************************************************************
typedef struct
{
    //! Source clock for the watchdog timer.
    am_hal_wdt_clock_select_e eClockSource;

    //! Generate a watchdog interrupt.
    bool bInterruptEnable;

    //! Count value at which the watchdog timer will generate an interrupt.
    uint32_t ui32InterruptValue;

    //! Generate a watchdog interrupt.
    bool bResetEnable;

    //! Count value at which the watchdog timer with generate a reset.
    uint32_t ui32ResetValue;

    //! If true, the MCU watchdog will generate a separate interrupt for the MCU
    //! when a DSP watchdog issues a reset.
    bool bAlertOnDSPReset;
}
am_hal_wdt_config_t;

//*****************************************************************************
//
//! Watchdog timer dsp configuration structure.
//
//*****************************************************************************
typedef struct
{
    //! Generate a watchdog interrupt.
    bool bInterruptEnable;

    //! Count value at which the watchdog timer will generate an interrupt.
    uint32_t ui32InterruptValue;

    //! Generate a watchdog interrupt.
    bool bResetEnable;

    //! Count value at which the watchdog timer with generate a reset.
    uint32_t ui32ResetValue;

    //! Reset this DSP's power module when the watchdog expires.
    bool bPMResetEnable;

    //! Count value at which to generate a power manager reset.
    uint32_t ui32PMResetValue;
}
am_hal_wdt_config_dsp_t;

//*****************************************************************************
//
//!brief  WDT interrupts.
//! @name watchdog_interrupt_macros
//! @{
//
//*****************************************************************************
#define AM_HAL_WDT_INTERRUPT_DSP_RESET    WDT_WDTIEREN_DSPRESETINT_Msk
#define AM_HAL_WDT_INTERRUPT_MCU          WDT_WDTIEREN_WDTINT_Msk
#define AM_HAL_WDT_INTERRUPT_DSP0         WDT_DSP0IERSTAT_DSP0INT_Msk
#define AM_HAL_WDT_INTERRUPT_DSP1         WDT_DSP1IERSTAT_DSP1INT_Msk
//! @}

//*****************************************************************************
//
//! @brief Configure the watchdog timer.
//!
//! @param eTimer selects the desired watchdog timer.
//! @param pvConfig is a pointer to a configuration structure for the timer.
//!
//! This function applies a configuration to the selected watchdog timer.
//!
//! @return WDT status code.
//
//*****************************************************************************
extern uint32_t am_hal_wdt_config(am_hal_wdt_select_e eTimer, void *pvConfig);

//*****************************************************************************
//
//! @brief Enables the watchdog timer.
//!
//! @param eTimer selects the desired watchdog timer.
//! @param bLock can be used to lock the watchdog.
//!
//! This function will start the selected watchdog timer. If the \e bLock
//! parameter is set, for the correct watchdog instance, it will also activate
//! the LOCK register. Once a watchdog is enabled and locked, its configuration
//! cannot be changed until a device reset.
//!
//! @return WDT status code.
//
//*****************************************************************************
extern uint32_t am_hal_wdt_start(am_hal_wdt_select_e eTimer, bool bLock);

//*****************************************************************************
//
//! @brief Disables the watchdog timer.
//!
//! @param eTimer selects the desired watchdog timer.
//!
//! This function will stop the selected watchdog. It will not work if the WDT
//! is already locked.
//!
//! @return WDT status code.
//
//*****************************************************************************
extern uint32_t am_hal_wdt_stop(am_hal_wdt_select_e eTimer);

//*****************************************************************************
//
//! @brief Restart (pet/feed) the watchdgog
//!
//! @param eTimer selects the desired watchdog timer.
//!
//! This funtion forces the watchdog timer to start over from its beginning
//! value. The key value is supplied manually to help mitigate the risk of
//! accidental watchdog restarts due to runaway code.
//!
//! @return WDT status code.
//
//*****************************************************************************
extern uint32_t am_hal_wdt_restart(am_hal_wdt_select_e eTimer);

//*****************************************************************************
//
//! @brief Reads the watchdog timer's current value.
//!
//! @param eTimer selects the desired watchdog timer.
//! @param pui32Value is the destination for the watchdog timer value.
//!
//! This function reads the current counter value for the selected timer.
//!
//! @return WDT status code.
//
//*****************************************************************************
extern uint32_t am_hal_wdt_read(am_hal_wdt_select_e eTimer, uint32_t *pui32Value);

//*****************************************************************************
//
//! @brief Watchdog interrupt enable.
//!
//! @param eTimer selects the desired watchdog timer.
//! @param ui32InterruptMask is an OR'ed together set of interrupts.
//!
//! Enables a set of interrupts for a specific watchdog TIMER.
//!
//! @note Each WDT instance has its own interrupt register. This function
//! cannot access more than one register at once. If you want to interact with
//! more than one interrupt, you will need to call this function more than
//! once.
//!
//! For example:
//!
//! @code
//! // WRONG (The DSP0 interrupt doesn't belong to the MCU watchdog)
//! am_hal_wdt_interrupt_enable(AM_HAL_WDT_MCU,
//!                             (AM_HAL_WDT_INTERRUPT_MCU |
//!                              AM_HAL_WDT_INTERRUPT_DSP_RESET |
//!                              AM_HAL_WDT_INTERRUPT_DSP0));
//!
//! // RIGHT
//! am_hal_wdt_interrupt_enable(AM_HAL_WDT_MCU,
//!                             (AM_HAL_WDT_INTERRUPT_MCU |
//!                              AM_HAL_WDT_INTERRUPT_DSP_RESET));
//!
//! am_hal_wdt_interrupt_enable(AM_HAL_WDT_DSP0,
//!                             AM_HAL_WDT_INTERRUPT_DSP0);
//!
//! @endcode
//!
//!
//! @return WDT status code.
//
//*****************************************************************************
extern uint32_t am_hal_wdt_interrupt_enable(am_hal_wdt_select_e eTimer,
                                            uint32_t ui32InterruptMask);

//*****************************************************************************
//
//! @brief Check to see which WDT interrupts are enabled.
//!
//! @param eTimer selects the desired watchdog timer.
//! @param pui32InterruptMask returns the set of enabled interrupts for the
//! selected timer.
//!
//! This function returns the set of enabled interrupts for a specific watchdog
//! timer.
//!
//! @return WDT status code.
//
//*****************************************************************************
extern uint32_t am_hal_wdt_interrupt_enable_get(am_hal_wdt_select_e eTimer,
                                                uint32_t *pui32InterruptMask);

//*****************************************************************************
//
//! @brief Disable a WDT interrupt.
//!
//! @param eTimer selects the desired watchdog timer.
//! @param ui32InterruptMask is an OR'ed together set of interrupts to disable.
//!
//! This function disables WDT interrupts.
//!
//! @return WDT status code.
//
//*****************************************************************************
extern uint32_t am_hal_wdt_interrupt_disable(am_hal_wdt_select_e eTimer,
                                             uint32_t ui32InterruptMask);

//*****************************************************************************
//
//! @brief Read the WDT interrupt status.
//!
//! @param eTimer selects the desired watchdog timer.
//!
//! @param pui32InterruptMask returns the set of active interrupts
//!
//! @param bEnabledOnly will change ui32Interrupt mask to only show enabled
//!        interrupts
//!
//! This function reads the WDT interrupt status.
//!
//! @return WDT status code.
//
//*****************************************************************************
extern uint32_t am_hal_wdt_interrupt_status_get(am_hal_wdt_select_e eTimer,
                                                uint32_t *pui32InterruptMask,
                                                bool bEnabledOnly);

//*****************************************************************************
//
//! @brief Clears the WDT interrupt.
//!
//! @param eTimer selects the desired watchdog timer.
//! @param ui32InterruptMask is an OR'ed together set of interrupts.
//!
//! Clears the WDT interrupt status.
//!
//! @return WDT status code.
//
//*****************************************************************************
extern uint32_t am_hal_wdt_interrupt_clear(am_hal_wdt_select_e eTimer,
                                           uint32_t ui32InterruptMask);

//*****************************************************************************
//
//! @brief Sets a WDT interrupt.
//!
//! @param eTimer selects the desired watchdog timer.
//! @param ui32InterruptMask is an OR'ed together set of interrupts to activate.
//!
//! Sets a WDT interrupt directly through software.
//!
//! @return WDT status code.
//
//*****************************************************************************
extern uint32_t am_hal_wdt_interrupt_set(am_hal_wdt_select_e eTimer,
                                         uint32_t ui32InterruptMask);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_WDT_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

