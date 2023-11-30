//*****************************************************************************
//
//! @file am_hal_sysctrl.c
//!
//! @brief Functions for interfacing with the M4F system control registers
//!
//! @addtogroup sysctrl4_4p SYSCTRL - System Control
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
//  Globals
//
//*****************************************************************************
//
// Extern some variables needed in this module.
//
extern uint32_t g_ui32origSimobuckVDDStrim;


extern void buck_ldo_update_override(bool bEnable);

//*****************************************************************************
//
// Place the core into sleep or deepsleep.
//
// This function puts the MCU to sleep or deepsleep depending on bSleepDeep.
//
// Valid values for bSleepDeep are:
//     AM_HAL_SYSCTRL_SLEEP_NORMAL
//     AM_HAL_SYSCTRL_SLEEP_DEEP
//
//*****************************************************************************
void
am_hal_sysctrl_sleep(bool bSleepDeep)
{
    bool bBuckIntoLPinDS, bSimobuckAct;

    //
    // Ensure Stack is in TCM - Current implementation only works with TCM stack
    //
    void *pDummy;
    if (((uint32_t)&pDummy < SRAM_BASEADDR) || ((uint32_t)&pDummy >= (SRAM_BASEADDR + TCM_MAX_SIZE)))
    {
        while (1);
    }

    bBuckIntoLPinDS = bSimobuckAct = false;

    //
    // Disable interrupts and save the previous interrupt state.
    //
    AM_CRITICAL_BEGIN

    //
    // Get current mode.
    //
    bSimobuckAct = ( PWRCTRL->VRSTATUS_b.SIMOBUCKST == PWRCTRL_VRSTATUS_SIMOBUCKST_ACT );

    //
    // If the user selected DEEPSLEEP and the TPIU is off, attempt to enter
    // DEEP SLEEP.
    //
    if ( (bSleepDeep == AM_HAL_SYSCTRL_SLEEP_DEEP) &&
         (MCUCTRL->DBGCTRL_b.CM4TPIUENABLE == MCUCTRL_DBGCTRL_CM4TPIUENABLE_DIS) )
    {

        //
        // Check if SIMOBUCK needs to stay in Active mode in DeepSleep
        //
        if ( bSimobuckAct )
        {
            //
            // Check if SIMOBUCK would go into LP mode in DeepSleep
            //
            if ( !PWRCTRL->AUDSSPWRSTATUS_b.PWRSTAUDADC     &&
                 !(PWRCTRL->DEVPWRSTATUS &
                    (PWRCTRL_DEVPWRSTATUS_PWRSTDBG_Msk      |
                     PWRCTRL_DEVPWRSTATUS_PWRSTUSBPHY_Msk   |
                     PWRCTRL_DEVPWRSTATUS_PWRSTUSB_Msk      |
                     PWRCTRL_DEVPWRSTATUS_PWRSTSDIO_Msk     |
                     PWRCTRL_DEVPWRSTATUS_PWRSTCRYPTO_Msk   |
                     PWRCTRL_DEVPWRSTATUS_PWRSTDISPPHY_Msk  |
                     PWRCTRL_DEVPWRSTATUS_PWRSTDISP_Msk     |
                     PWRCTRL_DEVPWRSTATUS_PWRSTGFX_Msk      |
                     PWRCTRL_DEVPWRSTATUS_PWRSTMSPI2_Msk    |
                     PWRCTRL_DEVPWRSTATUS_PWRSTMSPI1_Msk    |
                     PWRCTRL_DEVPWRSTATUS_PWRSTMSPI0_Msk    |
                     PWRCTRL_DEVPWRSTATUS_PWRSTADC_Msk      |
                     PWRCTRL_DEVPWRSTATUS_PWRSTUART3_Msk    |
                     PWRCTRL_DEVPWRSTATUS_PWRSTUART2_Msk    |
                     PWRCTRL_DEVPWRSTATUS_PWRSTUART1_Msk    |
                     PWRCTRL_DEVPWRSTATUS_PWRSTUART0_Msk    |
                     PWRCTRL_DEVPWRSTATUS_PWRSTIOM7_Msk     |
                     PWRCTRL_DEVPWRSTATUS_PWRSTIOM6_Msk     |
                     PWRCTRL_DEVPWRSTATUS_PWRSTIOM5_Msk     |
                     PWRCTRL_DEVPWRSTATUS_PWRSTIOM4_Msk     |
                     PWRCTRL_DEVPWRSTATUS_PWRSTIOM3_Msk     |
                     PWRCTRL_DEVPWRSTATUS_PWRSTIOM2_Msk     |
                     PWRCTRL_DEVPWRSTATUS_PWRSTIOM1_Msk     |
                     PWRCTRL_DEVPWRSTATUS_PWRSTIOM0_Msk     |
                     PWRCTRL_DEVPWRSTATUS_PWRSTIOS_Msk)) )
            {
                bBuckIntoLPinDS = true;

#if AM_HAL_PWRCTL_SHORT_VDDF_TO_VDDS
                //
                // Restore VDDS trim and disable VDDF to VDDS short.
                //
                MCUCTRL->SIMOBUCK13_b.ACTTRIMVDDS = g_ui32origSimobuckVDDStrim;
                MCUCTRL->PWRSW1_b.SHORTVDDFVDDSORVAL  = 0;
                MCUCTRL->PWRSW1_b.SHORTVDDFVDDSOREN   = 0;
#endif // AM_HAL_PWRCTL_SHORT_VDDF_TO_VDDS
                // Remove overrides to allow buck to go in LP mode
                buck_ldo_update_override(false);
            }
        }
        //
        // Prepare the core for deepsleep (write 1 to the DEEPSLEEP bit).
        //
        SCB->SCR |= _VAL2FLD(SCB_SCR_SLEEPDEEP, 1);
    }
    else
    {
        //
        // Prepare the core for normal sleep (write 0 to the DEEPSLEEP bit).
        //
        SCB->SCR &= ~_VAL2FLD(SCB_SCR_SLEEPDEEP, 1);
    }

    //
    // Before executing WFI, flush any buffered core and peripheral writes.
    //
    am_hal_sysctrl_bus_write_flush();

    //
    // Execute the sleep instruction.
    //
    __WFI();

    //
    // Upon wake, execute the Instruction Sync Barrier instruction.
    //
    __ISB();

    if ( bBuckIntoLPinDS )
    {
        // Re-enable overrides
        buck_ldo_update_override(true);


#if AM_HAL_PWRCTL_SHORT_VDDF_TO_VDDS
        //
        // Enable VDDF to VDDS short to increase load cap (2.2uF + 2.2uF).
        //
        MCUCTRL->PWRSW1_b.SHORTVDDFVDDSORVAL  = 1;
        MCUCTRL->PWRSW1_b.SHORTVDDFVDDSOREN   = 1;

        MCUCTRL->SIMOBUCK13_b.ACTTRIMVDDS = 0;    // VDDS trim level to 0
#endif // AM_HAL_PWRCTL_SHORT_VDDF_TO_VDDS
    }

    //
    // Restore the interrupt state.
    //
    AM_CRITICAL_END
}

//*****************************************************************************
//
// Enable the floating point module.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_enable(void)
{
    //
    // Enable access to the FPU in both privileged and user modes.
    // NOTE: Write 0s to all reserved fields in this register.
    //
    SCB->CPACR = _VAL2FLD(SCB_CPACR_CP11, 0x3) |
                 _VAL2FLD(SCB_CPACR_CP10, 0x3);
}

//*****************************************************************************
//
// Disable the floating point module.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_disable(void)
{
    //
    // Disable access to the FPU in both privileged and user modes.
    // NOTE: Write 0s to all reserved fields in this register.
    //
    SCB->CPACR = 0x00000000                         &
                 ~(_VAL2FLD(SCB_CPACR_CP11, 0x3) |
                   _VAL2FLD(SCB_CPACR_CP10, 0x3));
}

//*****************************************************************************
//
// Enable stacking of FPU registers on exception entry.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_stacking_enable(bool bLazy)
{
    uint32_t ui32fpccr;

    //
    // Set the requested FPU stacking mode in ISRs.
    //
    AM_CRITICAL_BEGIN
#define SYSCTRL_FPCCR_LAZY  (FPU_FPCCR_ASPEN_Msk | FPU_FPCCR_LSPEN_Msk)
    ui32fpccr  = FPU->FPCCR;
    ui32fpccr &= ~SYSCTRL_FPCCR_LAZY;
    ui32fpccr |= (bLazy ? SYSCTRL_FPCCR_LAZY : FPU_FPCCR_ASPEN_Msk);
    FPU->FPCCR = ui32fpccr;
    AM_CRITICAL_END
}

//*****************************************************************************
//
// Disable FPU register stacking on exception entry.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_stacking_disable(void)
{
    //
    // Completely disable FPU context save on entry to ISRs.
    //
    AM_CRITICAL_BEGIN
    FPU->FPCCR &= ~SYSCTRL_FPCCR_LAZY;
    AM_CRITICAL_END
}

//*****************************************************************************
//
// Issue a system wide reset using the AIRCR bit in the M4 system ctrl.
//
//*****************************************************************************
void
am_hal_sysctrl_aircr_reset(void)
{
    //
    // Set the system reset bit in the AIRCR register
    //
    __NVIC_SystemReset();
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
