//*****************************************************************************
//
//! @file am_hal_stimer.c
//!
//! @brief Functions for interfacing with the system timer (STIMER).
//!
//! @addtogroup stimer4_4p STIMER - System Timer
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

//
//! Timer Currently Configured
//
static bool bStimerConfigured = false;

//
//! Time at which last delta was set
//
static uint32_t g_lastStimer[8] =
{
    0xFFFFFFFE, 0xFFFFFFFE, 0xFFFFFFFE, 0xFFFFFFFE,
    0xFFFFFFFE, 0xFFFFFFFE, 0xFFFFFFFE, 0xFFFFFFFE
};

//*****************************************************************************
//
// Set up the stimer.
//
// This function should be used to perform the initial set-up of the
// stimer.
//
//*****************************************************************************
uint32_t
am_hal_stimer_config(uint32_t ui32STimerConfig)
{
    uint32_t ui32CurrVal;

    //
    // Read the current config
    //
    ui32CurrVal = STIMER->STCFG;

    //
    // Write our configuration value.
    //
    STIMER->STCFG = ui32STimerConfig;

    //
    // Indication that the STIMER has been configured.
    //
    bStimerConfigured = true;

    return ui32CurrVal;
}

//*****************************************************************************
//
// Check if the STIMER is running.
//
// This function should be used to perform the initial set-up of the
// stimer.
//
//*****************************************************************************
bool
am_hal_stimer_is_running(void)
{
  //
  // Check the STIMER has been configured and is currently counting
  //
  return (bStimerConfigured &&
      (STIMER_STCFG_CLKSEL_NOCLK != STIMER->STCFG_b.CLKSEL) &&
      (STIMER_STCFG_FREEZE_THAW == STIMER->STCFG_b.FREEZE) &&
      (STIMER_STCFG_CLEAR_RUN == STIMER->STCFG_b.CLEAR));
}
//*****************************************************************************
//
// Reset the current stimer block to power-up state.
//
//*****************************************************************************
void
am_hal_stimer_reset_config(void)
{
    STIMER->STCFG       = _VAL2FLD(STIMER_STCFG_FREEZE, 1);
    STIMER->SCAPCTRL0   = _VAL2FLD(STIMER_SCAPCTRL0_STSEL0, 0x7F);
    STIMER->SCAPCTRL1   = _VAL2FLD(STIMER_SCAPCTRL1_STSEL1, 0x7F);
    STIMER->SCAPCTRL2   = _VAL2FLD(STIMER_SCAPCTRL2_STSEL2, 0x7F);
    STIMER->SCAPCTRL3   = _VAL2FLD(STIMER_SCAPCTRL3_STSEL3, 0x7F);
    STIMER->SCMPR0      = 0;
    STIMER->SCMPR1      = 0;
    STIMER->SCMPR2      = 0;
    STIMER->SCMPR3      = 0;
    STIMER->SCMPR4      = 0;
    STIMER->SCMPR5      = 0;
    STIMER->SCMPR6      = 0;
    STIMER->SCMPR7      = 0;
    STIMER->SCAPT0      = 0;
    STIMER->SCAPT1      = 0;
    STIMER->SCAPT2      = 0;
    STIMER->SCAPT3      = 0;
    STIMER->SNVR0       = 0;
    STIMER->SNVR1       = 0;
    STIMER->SNVR2       = 0;
    STIMER->STMINTEN    = 0;
    STIMER->STMINTSTAT   = 0;
    STIMER->STMINTCLR    = 0xFFF;
}

//*****************************************************************************
//
// Get the current stimer value.
//
// This function can be used to read, uninvasively, the value in the stimer.
//
//*****************************************************************************
uint32_t
am_hal_stimer_counter_get(void)
{
    uint32_t      ui32TimerAddr = (uint32_t)&STIMER->STTMR;
    uint32_t      ui32TimerVals[3];

    //
    // Read the register into ui32TimerVals[].
    //
    am_hal_triple_read(ui32TimerAddr, ui32TimerVals);

    //
    // Now determine which of the three values is the correct value.
    // If the first 2 match, then the values are both correct and we're done.
    // Otherwise, the third value is taken to be the correct value.
    //
    if ( ui32TimerVals[0] == ui32TimerVals[1] )
    {
        //
        // If the first two values match, then neither one was a bad read.
        // We'll take this as the current time.
        //
        return ui32TimerVals[1];
    }
    else
    {
        return ui32TimerVals[2];
    }
}

//*****************************************************************************
//
// Clear the stimer counter.
//
// This function clears the STimer Counter and leaves the stimer running.
//
//*****************************************************************************
void
am_hal_stimer_counter_clear(void)
{
    //
    // Set the clear bit
    //
    STIMER->STCFG |= STIMER_STCFG_CLEAR_Msk;

    //
    // Reset the clear bit
    //
    STIMER->STCFG &= ~STIMER_STCFG_CLEAR_Msk;
}

//*****************************************************************************
//
// Check if the compare value can be set without blocking.
//
//*****************************************************************************
bool
am_hal_stimer_check_compare_delta_set(uint32_t ui32CmprInstance)
{
    uint32_t curTimer;
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( ui32CmprInstance > 7 )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }
#endif
    // Take a snapshot of STIMER
    curTimer = am_hal_stimer_counter_get();
    // We cannot set COMPARE back to back
    // Need to wait for previous write to complete, which takes 2 cycles
    if ((curTimer != g_lastStimer[ui32CmprInstance]) &&
        (curTimer != (g_lastStimer[ui32CmprInstance] + 1)))
    {
        return true;
    }
    else
    {
        return false;
    }
}

//*****************************************************************************
//
// Set the compare value.
//
//*****************************************************************************
uint32_t
am_hal_stimer_compare_delta_set(uint32_t ui32CmprInstance, uint32_t ui32Delta)
{
    uint32_t curTimer, curTimer0;
    uint32_t ui32Ret = AM_HAL_STATUS_SUCCESS;

    // Take a snapshot of STIMER at the beginning of the function
    curTimer = curTimer0 = am_hal_stimer_counter_get();

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( ui32CmprInstance > 7 )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }
#endif

    // We cannot set COMPARE back to back
    // Make sure we disable any previous COMPARE interrupts coming while we wait
    // NOTE: This does not address stale interrupts once we have written the COMPARE...
    // as it takes another 2 clocks for Writes to go through.
    // So - a stale interrupt could still come, that needs to be ignored by application
    uint32_t ui32CompareRestoreMsk = STIMER->STCFG & (STIMER_STCFG_COMPAREAEN_Msk << ui32CmprInstance);

    //
    // Disable the Compare
    //
    STIMER->STCFG &= ~(STIMER_STCFG_COMPAREAEN_Msk << ui32CmprInstance);

    do
    {
        // We cannot set COMPARE back to back
        // Need to wait for previous write to complete, which takes 2 cycles
        if ((curTimer != g_lastStimer[ui32CmprInstance]) &&
            (curTimer != (g_lastStimer[ui32CmprInstance] + 1)))
        {
            //
            // Start a critical section.
            //
            AM_CRITICAL_BEGIN
            curTimer = am_hal_stimer_counter_get();
            // Adjust Delta
            // It takes 2 STIMER clock cycles for writes to COMPARE to be effective
            // Also the interrupt itself is delayed by a cycle
            // This effectively means we need to adjust the delta by 3
            // Also adjust for the delay since we entered the function
            if (ui32Delta > (3 + (curTimer - curTimer0)))
            {
                ui32Delta -= 3 + (curTimer - curTimer0);
            }
            else
            {
                // Need to floor delta to 1
                ui32Delta = 1;
                ui32Ret = AM_HAL_STIMER_DELTA_TOO_SMALL;
            }
            //
            // Set the delta
            //
            AM_REGVAL(AM_REG_STIMER_COMPARE(0, ui32CmprInstance)) = ui32Delta;
            //
            // Restore the Compare Enable
            //
            STIMER->STCFG |= ui32CompareRestoreMsk;
            //
            // Get a snapshot when we set COMPARE
            //
            g_lastStimer[ui32CmprInstance] = am_hal_stimer_counter_get();
            //
            // End the critical section.
            //
            AM_CRITICAL_END
            break;
        }
        curTimer = am_hal_stimer_counter_get();
    } while (1);

    return ui32Ret;
}

//*****************************************************************************
//
// Get the current stimer compare register value.
//
// This function can be used to read the value in an stimer compare register.
//
//*****************************************************************************
uint32_t
am_hal_stimer_compare_get(uint32_t ui32CmprInstance)
{
    uint32_t curTimer;
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( ui32CmprInstance > 7 )
    {
        return 0;
    }
#endif
    //
    // Need to wait at least 3 cycle after setting the compare to read it
    // reliably
    //
    do
    {
        curTimer = am_hal_stimer_counter_get();
        if ((curTimer != g_lastStimer[ui32CmprInstance]) &&
            (curTimer != (g_lastStimer[ui32CmprInstance] + 1)) &&
            (curTimer != (g_lastStimer[ui32CmprInstance] + 2)))
        {
            return AM_REGVAL(AM_REG_STIMER_COMPARE(0, ui32CmprInstance));
        }
    } while (1);
}

//*****************************************************************************
//
// Start capturing data with the specified capture register.
//
// Use this function to start capturing.
//
//*****************************************************************************
void
am_hal_stimer_capture_start(uint32_t ui32CaptureNum,
                            uint32_t ui32GPIONumber,
                            bool bPolarity)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( ui32GPIONumber > (AM_HAL_GPIO_MAX_PADS-1) )
    {
        return;
    }
#endif

    //
    // Set the polarity and pin selection in the GPIO block.
    //
    switch (ui32CaptureNum)
    {
         case 0:
            STIMER->SCAPCTRL0_b.STPOL0 = bPolarity;
            STIMER->SCAPCTRL0_b.STSEL0 = ui32GPIONumber;
            STIMER->SCAPCTRL0_b.CAPTURE0 = STIMER_SCAPCTRL0_CAPTURE0_ENABLE;
            break;
         case 1:
            STIMER->SCAPCTRL1_b.STPOL1 = bPolarity;
            STIMER->SCAPCTRL1_b.STSEL1 = ui32GPIONumber;
            STIMER->SCAPCTRL1_b.CAPTURE1 = STIMER_SCAPCTRL1_CAPTURE1_ENABLE;
            break;
         case 2:
            STIMER->SCAPCTRL2_b.STPOL2 = bPolarity;
            STIMER->SCAPCTRL2_b.STSEL2 = ui32GPIONumber;
            STIMER->SCAPCTRL2_b.CAPTURE2 = STIMER_SCAPCTRL2_CAPTURE2_ENABLE;
            break;
         case 3:
            STIMER->SCAPCTRL3_b.STPOL3 = bPolarity;
            STIMER->SCAPCTRL3_b.STSEL3 = ui32GPIONumber;
            STIMER->SCAPCTRL3_b.CAPTURE3 = STIMER_SCAPCTRL3_CAPTURE3_ENABLE;
            break;
         default:
            return;     // error concealment.
    }

    //
    // Set TIMER Global Enable for GPIO inputs.
    //
    TIMER->GLOBEN_b.ENABLEALLINPUTS = 1;

}

//*****************************************************************************
//
// Stop capturing data with the specified capture register.
//
// Use this function to stop capturing.
//
//*****************************************************************************
void
am_hal_stimer_capture_stop(uint32_t ui32CaptureNum)
{
    //
    // Disable it in the STIMER block.
    //
    switch (ui32CaptureNum)
    {
         case 0:
             STIMER->SCAPCTRL0_b.CAPTURE0 = STIMER_SCAPCTRL0_CAPTURE0_DISABLE;
            break;
         case 1:
             STIMER->SCAPCTRL1_b.CAPTURE1 = STIMER_SCAPCTRL1_CAPTURE1_DISABLE;
            break;
         case 2:
             STIMER->SCAPCTRL2_b.CAPTURE2 = STIMER_SCAPCTRL2_CAPTURE2_DISABLE;
            break;
         case 3:
             STIMER->SCAPCTRL3_b.CAPTURE3 = STIMER_SCAPCTRL3_CAPTURE3_DISABLE;
            break;
         default:
            return;     // error concealment.
    }

    //
    // Start a critical section.
    //
    AM_CRITICAL_BEGIN

    //
    // Set TIMER Global Disable for GPIO inputs if all are DISABLED.
    //
    if ((STIMER->SCAPCTRL0_b.CAPTURE0 == STIMER_SCAPCTRL0_CAPTURE0_DISABLE) &&
            (STIMER->SCAPCTRL1_b.CAPTURE1 == STIMER_SCAPCTRL1_CAPTURE1_DISABLE) &&
            (STIMER->SCAPCTRL2_b.CAPTURE2 == STIMER_SCAPCTRL2_CAPTURE2_DISABLE) &&
            (STIMER->SCAPCTRL3_b.CAPTURE3 == STIMER_SCAPCTRL3_CAPTURE3_DISABLE))
    {
        TIMER->GLOBEN_b.ENABLEALLINPUTS = 0;
    }

    AM_CRITICAL_END
}

//*****************************************************************************
//
// Set the current stimer nvram register value.
//
// This function can be used to read the value in an stimer NVRAM register.
//
//*****************************************************************************
void
am_hal_stimer_nvram_set(uint32_t ui32NvramNum, uint32_t ui32NvramVal)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( ui32NvramNum > 3 )
    {
        return;
    }
#endif

    AM_REGVAL(AM_REG_STIMER_NVRAM(0, ui32NvramNum)) = ui32NvramVal;
}

//*****************************************************************************
//
// Get the current stimer nvram register value.
//
// This function can be used to read the value in an stimer NVRAM register.
//
//*****************************************************************************
uint32_t am_hal_stimer_nvram_get(uint32_t ui32NvramNum)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( ui32NvramNum > 3 )
    {
        return 0;
    }
#endif

    return AM_REGVAL(AM_REG_STIMER_NVRAM(0, ui32NvramNum));
}

//*****************************************************************************
//
// Get the current stimer capture register value.
//
// This function can be used to read the value in an stimer capture register.
//
//*****************************************************************************
uint32_t am_hal_stimer_capture_get(uint32_t ui32CaptureNum)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( ui32CaptureNum > 3 )
    {
        return 0;
    }
#endif

    return AM_REGVAL(AM_REG_STIMER_CAPTURE(0, ui32CaptureNum));
}

//*****************************************************************************
//
// Enables the selected system timer interrupt.
//
// This function will enable the selected interrupts in the STIMER interrupt
// enable register. In order to receive an interrupt from an stimer component,
// you will need to enable the interrupt for that component in this main
// register, as well as in the stimer configuration register (accessible though
// am_hal_stimer_config()), and in the NVIC.
//
//*****************************************************************************
void
am_hal_stimer_int_enable(uint32_t ui32Interrupt)
{
    //
    // Enable the interrupt at the module level.
    //
    STIMER->STMINTEN |= ui32Interrupt;
}

//*****************************************************************************
//
// Return the enabled stimer interrupts.
//
// This function will return all enabled interrupts in the STIMER
// interrupt enable register.
//
//*****************************************************************************
uint32_t
am_hal_stimer_int_enable_get(void)
{
    //
    // Return enabled interrupts.
    //
    return STIMER->STMINTEN;
}

//*****************************************************************************
//
// Disables the selected stimer interrupt.
//
// This function will disable the selected interrupts in the STIMER
// interrupt register.
//
//*****************************************************************************
void
am_hal_stimer_int_disable(uint32_t ui32Interrupt)
{
    //
    // Disable the interrupt at the module level.
    //
    STIMER->STMINTEN &= ~ui32Interrupt;
}

//*****************************************************************************
//
// Sets the selected stimer interrupt.
//
// This function will set the selected interrupts in the STIMER
// interrupt register.
//
//*****************************************************************************
void
am_hal_stimer_int_set(uint32_t ui32Interrupt)
{
    //
    // Set the interrupts.
    //
    STIMER->STMINTSET = ui32Interrupt;
}

//*****************************************************************************
//
// Clears the selected stimer interrupt.
//
// This function will clear the selected interrupts in the STIMER
// interrupt register.
//
//*****************************************************************************
void
am_hal_stimer_int_clear(uint32_t ui32Interrupt)
{
    //
    // Disable the interrupt at the module level.
    //
    STIMER->STMINTCLR = ui32Interrupt;
}

//*****************************************************************************
//
// Returns either the enabled or raw stimer interrupt status.
//
// This function will return the stimer interrupt status.
//
//*****************************************************************************
uint32_t
am_hal_stimer_int_status_get(bool bEnabledOnly)
{
    //
    // Return the desired status.
    //
    uint32_t ui32RetVal = STIMER->STMINTSTAT;

    if ( bEnabledOnly )
    {
        ui32RetVal &= STIMER->STMINTEN;
    }

    return ui32RetVal;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
