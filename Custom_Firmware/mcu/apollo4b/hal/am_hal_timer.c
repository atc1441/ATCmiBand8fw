//*****************************************************************************
//
//! @file am_hal_timer.c
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

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

static struct
{
    bool  bUseOut0;
    bool  bUseInt0;
    bool  bUseOut1;
    bool  bUseInt1;
} TimerFlags[AM_REG_NUM_TIMERS];

//*****************************************************************************
//
// Configure a TIMER
//
//*****************************************************************************
static uint32_t timer_config_b0(uint32_t ui32TimerNumber,
                                am_hal_timer_config_t *psTimerConfig)
{
    uint32_t ui32ConfigCtrl, ui32ConfigMode;

    //
    // Note: AUXEN is a new feature for Rev B1, so we don't touch it for B0.
    //

    //
    //
    // Build up a value in SRAM before we start writing to the timer control
    // registers.
    //
    ui32ConfigCtrl  = _VAL2FLD(TIMER_CTRL0_TMR0CLK,     psTimerConfig->eInputClock);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0FN,      psTimerConfig->eFunction);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0POL1,    psTimerConfig->bInvertOutput1);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0POL0,    psTimerConfig->bInvertOutput0);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0TMODE,   psTimerConfig->eTriggerType);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0LMT,     psTimerConfig->ui32PatternLimit);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0EN, 0);

    ui32ConfigMode = _VAL2FLD(TIMER_MODE0_TMR0TRIGSEL, psTimerConfig->eTriggerSource);

    //
    // Disable the timer.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0EN = 0;

    //
    // Apply the settings from the configuration structure.
    //
    TIMERn(ui32TimerNumber)->CTRL0    = ui32ConfigCtrl;
    TIMERn(ui32TimerNumber)->MODE0    = ui32ConfigMode;
    TIMERn(ui32TimerNumber)->TMR0CMP0 = psTimerConfig->ui32Compare0;
    TIMERn(ui32TimerNumber)->TMR0CMP1 = psTimerConfig->ui32Compare1;

    //
    // Clear the timer to make sure it has the appropriate starting value.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLR = 1;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Configure b1 TIMER
//
//*****************************************************************************
uint32_t timer_config_b1(uint32_t ui32TimerNumber,
                         am_hal_timer_config_t *psTimerConfig)
{
    uint32_t ui32ConfigCtrl, ui32ConfigMode;
    uint32_t ui32TimerLimit = psTimerConfig->ui32PatternLimit;
    uint32_t ui32Compare0 = psTimerConfig->ui32Compare0;
    uint32_t ui32Compare1 = psTimerConfig->ui32Compare1;

    TimerFlags[ui32TimerNumber].bUseOut0 = false;
    TimerFlags[ui32TimerNumber].bUseInt0 = false;
    TimerFlags[ui32TimerNumber].bUseOut1 = false;
    TimerFlags[ui32TimerNumber].bUseInt1 = false;

    //
    // Mode/function specific error checking.
    //
    switch ( psTimerConfig->eFunction )
    {
        case AM_HAL_TIMER_FN_EDGE:
            //
            // INT1 and OUT1 can be used.
            //
            TimerFlags[ui32TimerNumber].bUseInt1 = true;
            TimerFlags[ui32TimerNumber].bUseOut1 = true;

            //
            // User cannot use OUT0 or INT0.
            // Check that Compare0 is set to default.
            //
            if (ui32Compare0 != 0xFFFFFFFF)
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }

            //
            // Check is Compare1 is used.
            //
            if (ui32Compare1 != 0xFFFFFFFF)
            {
                //
                // CMP0 must be set to CMP1 + 1 to allow INT1/OUT1 to happen.
                //
                ui32Compare0 = ui32Compare1 + 1;
            }
            break;

        case AM_HAL_TIMER_FN_PWM:
            //
            // OUT0/1 and INT0/1 can be used;
            //
            TimerFlags[ui32TimerNumber].bUseInt0 = true;
            TimerFlags[ui32TimerNumber].bUseOut0 = true;
            TimerFlags[ui32TimerNumber].bUseInt1 = true;
            TimerFlags[ui32TimerNumber].bUseOut1 = true;

            //
            // Timer Limit is not allowed for PWM.
            //
            if (ui32TimerLimit > 0)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            break;

        case AM_HAL_TIMER_FN_UPCOUNT:
            //
            // OUT0/1 and INT0/1 can be used;
            //
            TimerFlags[ui32TimerNumber].bUseInt0 = true;
            TimerFlags[ui32TimerNumber].bUseOut0 = true;
            TimerFlags[ui32TimerNumber].bUseInt1 = true;
            TimerFlags[ui32TimerNumber].bUseOut1 = true;

            //
            // Timer Limit is not allowed for UPCOUNT.
            //
            if (ui32TimerLimit > 0)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            break;

        case AM_HAL_TIMER_FN_SINGLEPATTERN:
            //
            // Check that requested pattern is not too long.
            //
            if (psTimerConfig->ui32PatternLimit > 32)
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }

            //
            // Set the CMP1 to TMRLMT
            //
            ui32Compare1 = ui32TimerLimit;

            //
            // If TMRLMT > 0, then increment it by 1.
            //
            if (ui32TimerLimit > 0)
            {
                ui32TimerLimit++;
            }

            //
            // OUT0 and INT1 can be used;
            //
            TimerFlags[ui32TimerNumber].bUseOut0 = true;
            TimerFlags[ui32TimerNumber].bUseInt1 = true;
            TimerFlags[ui32TimerNumber].bUseOut1 = true;
            break;

        case AM_HAL_TIMER_FN_REPEATPATTERN:
            //
            // OUT0/1 and INT0/1 can be used;
            //
            TimerFlags[ui32TimerNumber].bUseOut0 = true;
            TimerFlags[ui32TimerNumber].bUseInt0 = true;
            TimerFlags[ui32TimerNumber].bUseOut1 = true;
            TimerFlags[ui32TimerNumber].bUseInt1 = true;
            break;

        case AM_HAL_TIMER_FN_CONTINUOUS:
        case AM_HAL_TIMER_FN_DOWNCOUNT:
        case AM_HAL_TIMER_FN_EVENTTIMER:
            return AM_HAL_STATUS_INVALID_OPERATION;
    }

    //
    // Check for valid timer output for clock source.
    //
    if ( (psTimerConfig->eInputClock >= AM_HAL_TIMER_CLOCK_TMR0_OUT0) &&
         (psTimerConfig->eInputClock <= AM_HAL_TIMER_CLOCK_TMR15_OUT1) )
    {
        uint32_t Timer = (psTimerConfig->eInputClock - AM_HAL_TIMER_CLOCK_TMR0_OUT0) >> 1;
        uint32_t Output = psTimerConfig->eInputClock & 1;
        if (((0 == Output) && (!TimerFlags[Timer].bUseOut0)) || ((1 == Output) && (!TimerFlags[Timer].bUseOut1)))
        {
            return AM_HAL_STATUS_INVALID_ARG;
        }
    }

    //
    // Check for valid timer output for trigger source.
    //
    if ((psTimerConfig->eTriggerType > AM_HAL_TIMER_TRIGGER_DIS) && (psTimerConfig->eTriggerSource <= AM_HAL_TIMER_TRIGGER_TMR15_OUT1))
    {
        uint32_t    Timer = (psTimerConfig->eTriggerSource - AM_HAL_TIMER_TRIGGER_TMR0_OUT0) >> 1;
        uint32_t    Output = psTimerConfig->eTriggerSource & 1;
        if (((0 == Output) && (!TimerFlags[Timer].bUseOut0)) || ((1 == Output) && (!TimerFlags[Timer].bUseOut1)))
        {
            return AM_HAL_STATUS_INVALID_ARG;
        }
    }

    //
    // Clear the AUXEN to gain safe access to CTRL register.
    //
    TIMER->AUXEN &= ~(1 << ui32TimerNumber);

    //
    //
    // Build up a value in SRAM before we start writing to the timer control
    // registers.
    //
    ui32ConfigCtrl  = _VAL2FLD(TIMER_CTRL0_TMR0CLK,     psTimerConfig->eInputClock);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0FN,      psTimerConfig->eFunction);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0POL1,    psTimerConfig->bInvertOutput1);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0POL0,    psTimerConfig->bInvertOutput0);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0TMODE,   psTimerConfig->eTriggerType);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0LMT,     ui32TimerLimit);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0EN, 0);

    ui32ConfigMode  = _VAL2FLD(TIMER_MODE0_TMR0TRIGSEL, psTimerConfig->eTriggerSource);

    //
    // Disable the timer.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0EN = 0;

    //
    // Apply the settings from the configuration structure.
    //
    TIMERn(ui32TimerNumber)->CTRL0 = ui32ConfigCtrl;
    TIMERn(ui32TimerNumber)->MODE0 = ui32ConfigMode;
    TIMERn(ui32TimerNumber)->TMR0CMP0 = ui32Compare0;
    TIMERn(ui32TimerNumber)->TMR0CMP1 = ui32Compare1;

    //
    // Clear the timer to make sure it has the appropriate starting value.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLR = 1;

    //
    // Set the AUXEN after changes to CTRL register.
    //
    TIMER->AUXEN |= (1 << ui32TimerNumber);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Not declared as static as this function can be used from within HAL
//
//*****************************************************************************
uint32_t
internal_timer_config(uint32_t ui32TimerNumber,
                      am_hal_timer_config_t *psTimerConfig)
{
    if (APOLLO4_B0)
    {
        return timer_config_b0(ui32TimerNumber, psTimerConfig);
    }
    else if (APOLLO4_GE_B1)
    {
        //
        // Check for broken timers which we don't want the customer to use.
        //
        if ( (1 == ui32TimerNumber)   ||
             (3 == ui32TimerNumber)   ||
             (10 == ui32TimerNumber)  ||
             (11 == ui32TimerNumber)  ||
             (12 == ui32TimerNumber)  ||
             (14 == ui32TimerNumber)  ||
             (15 == ui32TimerNumber) )
        {
            return AM_HAL_STATUS_INVALID_ARG;
        }

        //
        // Check for timers that don't support triggering.
        //
        if ( ((5 == ui32TimerNumber)  ||
              (7 == ui32TimerNumber)  ||
              (8 == ui32TimerNumber)  ||
              (9 == ui32TimerNumber)  ||
              (13 == ui32TimerNumber))  &&
              (psTimerConfig->eTriggerType > AM_HAL_TIMER_TRIGGER_DIS) )
        {
            return AM_HAL_STATUS_INVALID_OPERATION;
        }
        return timer_config_b1(ui32TimerNumber, psTimerConfig);  // Rev B1/B2 TIMER configuration should be the same.
    }
    else
    {
        return AM_HAL_STATUS_FAIL;
    }
} // internal_timer_config()

//*****************************************************************************
//
// Not declared as static as this function can be used from within HAL
//
//*****************************************************************************
uint32_t
am_hal_timer_config(uint32_t ui32TimerNumber,
                    am_hal_timer_config_t *psTimerConfig)
{
#if defined(AM_HAL_PWRCTL_HPLP_WA) || defined(AM_HAL_PWRCTL_CRYPTO_WA)
    //
    // Check for internal timers which we don't want the customer to use.
    //
    if ( AM_HAL_WRITE_WAIT_TIMER == ui32TimerNumber )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // defined(AM_HAL_PWRCTL_HPLP_WA) || defined(AM_HAL_PWRCTL_CRYPTO_WA)
    return internal_timer_config(ui32TimerNumber, psTimerConfig);
}

//*****************************************************************************
//
// Initialize a timer configuration structure with default values.
//
//*****************************************************************************
uint32_t
am_hal_timer_default_config_set(am_hal_timer_config_t *psTimerConfig)
{

    psTimerConfig->eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV16;
    psTimerConfig->eFunction = AM_HAL_TIMER_FN_EDGE;
    psTimerConfig->ui32Compare0 = 0xFFFFFFFF;
    psTimerConfig->ui32Compare1 = 0xFFFFFFFF;
    psTimerConfig->bInvertOutput0 = false;
    psTimerConfig->bInvertOutput1 = false;
    psTimerConfig->eTriggerType = AM_HAL_TIMER_TRIGGER_DIS;
    psTimerConfig->eTriggerSource = AM_HAL_TIMER_TRIGGER_TMR0_OUT1;
    psTimerConfig->ui32PatternLimit = 0;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Reset the timer to the default state.
//
//*****************************************************************************
uint32_t
am_hal_timer_reset_config(uint32_t ui32TimerNumber)
{
    //
    // Disable Interrupts.
    //
    am_hal_timer_interrupt_disable(3 << (ui32TimerNumber * 2));

    //
    // Enable the Global enable.
    //
    am_hal_timer_enable_sync(1 << ui32TimerNumber);

    //
    // Disable the Timer.
    //
    am_hal_timer_disable(ui32TimerNumber);

    //
    // Clear the AUXEN to gain safe access to CTRL register.
    //
    TIMER->AUXEN &= ~(1 << ui32TimerNumber);

    //
    // Reset the Timer specific registers.
    //
    TIMERn(ui32TimerNumber)->CTRL0 = 0;
    TIMERn(ui32TimerNumber)->TIMER0 = 0;
    TIMERn(ui32TimerNumber)->TMR0CMP0 = 0;
    TIMERn(ui32TimerNumber)->TMR0CMP1 = 0;
    TIMERn(ui32TimerNumber)->MODE0 = 0;

    //
    // Set the AUXEN after changes to CTRL register.
    //
    TIMER->AUXEN |= (1 << ui32TimerNumber);

    am_hal_timer_interrupt_clear(3 << (ui32TimerNumber *2));

    TimerFlags[ui32TimerNumber].bUseOut0 = false;
    TimerFlags[ui32TimerNumber].bUseInt0 = false;
    TimerFlags[ui32TimerNumber].bUseOut1 = false;
    TimerFlags[ui32TimerNumber].bUseInt1 = false;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Enable a single TIMER
//
//*****************************************************************************
uint32_t
am_hal_timer_enable(uint32_t ui32TimerNumber)
{
    AM_CRITICAL_BEGIN;
    //
    // Clear the AUXEN to gain safe access to CTRL register.
    //
    TIMER->AUXEN &= ~(1 << ui32TimerNumber);

    //
    // Enable the timer in both the individual enable register and the global
    // sync register and the AUXEN.
    //
    TIMER->GLOBEN |= 1 << ui32TimerNumber;
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0EN = 1;

    //
    // Set the AUXEN after changes to CTRL register.
    //
    TIMER->AUXEN |= (1 << ui32TimerNumber);

    AM_CRITICAL_END;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Disable a single TIMER
//
//*****************************************************************************
uint32_t
am_hal_timer_disable(uint32_t ui32TimerNumber)
{

    AM_CRITICAL_BEGIN;

    //
    // Clear the AUXEN to gain safe access to CTRL register.
    //
    TIMER->AUXEN &= ~(1 << ui32TimerNumber);

    //
    // Disable the timer.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0EN = 0;

    AM_CRITICAL_END;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Enable a group of TIMERS all at once
//
//*****************************************************************************
uint32_t
am_hal_timer_enable_sync(uint32_t ui32TimerMask)
{
    //
    // Disable the timers in the global sync register, make sure they are all
    // individually enabled, and then re-enable them in the global sync
    // register.
    //
    AM_CRITICAL_BEGIN;

    TIMER->GLOBEN &= ~(ui32TimerMask);

    for (uint32_t i = 0; i < AM_REG_NUM_TIMERS; i++)
    {
        if ((1 << i) & ui32TimerMask)
        {
          //
          // Clear the AUXEN to gain safe access to CTRL register.
          //
          TIMER->AUXEN &= ~(1 << i);

          //
          // Enable the timer.
          //
          TIMERn(i)->CTRL0_b.TMR0EN = 1;

          //
          // Set the AUXEN after changes to CTRL register.
          //
          TIMER->AUXEN |= (1 << i);
        }
    }

    TIMER->GLOBEN |= ui32TimerMask;

    AM_CRITICAL_END;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Disable a group of TIMERS all at once
//
//*****************************************************************************
uint32_t
am_hal_timer_disable_sync(uint32_t ui32TimerMask)
{
    //
    // Disable the timers in the global sync register, make sure they are all
    // individually disabled, and then re-enable them in the global sync
    // register.
    //
    AM_CRITICAL_BEGIN;

    TIMER->GLOBEN &= ~(ui32TimerMask);

    for (uint32_t i = 0; i < AM_REG_NUM_TIMERS; i++)
    {
        if ((1 << i) & ui32TimerMask)
        {
          //
          // Clear the AUXEN to gain safe access to CTRL register.
          //
          TIMER->AUXEN &= ~(1 << i);

          //
          // Disable the timer.
          //
          TIMERn(i)->CTRL0_b.TMR0EN = 0;
        }
    }

    AM_CRITICAL_END;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Clear a single TIMER and start the timer.
// For Apollo4b, TIMERs are started via the AUXEN.
//
//*****************************************************************************
uint32_t
am_hal_timer_clear(uint32_t ui32TimerNumber)
{
    AM_CRITICAL_BEGIN;

    //
    // Clear the AUXEN to gain safe access to CTRL register.
    //
    TIMER->AUXEN &= ~(1 << ui32TimerNumber);

    //
    // Disable the timer.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0EN = 0;

    //
    // Clear the timer.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLR = 1;

    //
    // Enable the timer.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0EN = 1;

    //
    // Set the AUXEN after changes to CTRL register.
    //
    TIMER->AUXEN |= (1 << ui32TimerNumber);

    AM_CRITICAL_END;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Clear a single TIMER but don't start it.
// For Apollo4b, TIMERs are started via the AUXEN.
//
//*****************************************************************************
uint32_t
am_hal_timer_clear_stop(uint32_t ui32TimerNumber)
{
    AM_CRITICAL_BEGIN;

    //
    // Clear the AUXEN to gain safe access to CTRL register.
    //
    TIMER->AUXEN &= ~(1 << ui32TimerNumber);

    //
    // Disable the timer.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0EN = 0;

    //
    // Clear the timer.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLR = 1;

    //
    // Enable the timer.
    //
    TIMERn(ui32TimerNumber)->CTRL0_b.TMR0EN = 1;

    AM_CRITICAL_END;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Read the current value of a timer.
//
//*****************************************************************************
uint32_t
am_hal_timer_read(uint32_t ui32TimerNumber)
{
    uint32_t      ui32TimerAddr = (uint32_t)&TIMERn(ui32TimerNumber)->TIMER0;
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
// Set the COMPARE0 value for a single timer.
//
//*****************************************************************************
uint32_t
am_hal_timer_compare0_set(uint32_t ui32TimerNumber,
                          uint32_t ui32CompareValue)
{
    //
    // Apply the Compare0 value without disabling the timer.
    //
    TIMERn(ui32TimerNumber)->TMR0CMP0 = ui32CompareValue;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Set the COMPARE1 value for a single timer.
//
//*****************************************************************************
uint32_t
am_hal_timer_compare1_set(uint32_t ui32TimerNumber,
                          uint32_t ui32CompareValue)
{
    //
    // Apply the Compare1 value without disabling the timer.
    //
    TIMERn(ui32TimerNumber)->TMR0CMP1 = ui32CompareValue;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Enable timer interrupts.
//
//*****************************************************************************
uint32_t
am_hal_timer_interrupt_enable(uint32_t ui32InterruptMask)
{
    uint32_t ui32Mask = ui32InterruptMask;

    if (APOLLO4_GE_B1)  // Rev B1/B2 should be the same.
    {
        for (uint32_t Timer = 0; Timer < AM_REG_NUM_TIMERS; Timer++)
        {
            if (((ui32Mask & 0x1) && (!TimerFlags[Timer].bUseInt0)) ||
                ((ui32Mask & 0x2) && (!TimerFlags[Timer].bUseInt1)))
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            ui32Mask >>= 2;
        }
    }
    TIMER->INTEN |= ui32InterruptMask;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Disable timer interrupts.
//
//*****************************************************************************
uint32_t
am_hal_timer_interrupt_disable(uint32_t ui32InterruptMask)
{
    TIMER->INTEN &= ~(ui32InterruptMask);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Get the timer interrupt status.
//
//*****************************************************************************
uint32_t
am_hal_timer_interrupt_status_get(bool bEnabledOnly, uint32_t *pui32IntStatus)
{
    DIAG_SUPPRESS_VOLATILE_ORDER()

    if (bEnabledOnly)
    {
        *pui32IntStatus = TIMER->INTSTAT & TIMER->INTEN;
    }
    else
    {
        *pui32IntStatus = TIMER->INTSTAT;
    }

    return AM_HAL_STATUS_SUCCESS;

    DIAG_DEFAULT_VOLATILE_ORDER()
}

//*****************************************************************************
//
// Clear timer interrupts.
//
//*****************************************************************************
uint32_t
am_hal_timer_interrupt_clear(uint32_t ui32InterruptMask)
{
    TIMER->INTCLR = ui32InterruptMask;
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
