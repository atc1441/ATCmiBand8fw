//*****************************************************************************
//
//! @file am_hal_sysctrl.c
//!
//! @brief Functions for interfacing with the M4F system control registers
//!
//! @addtogroup sysctrl4_4b SYSCTRL - System Control
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
//  Globals
//
//*****************************************************************************
//
// Extern some variables needed in this module.
//
extern uint32_t g_ui32origSimobuckVDDCtrim;
extern uint32_t g_ui32origSimobuckVDDStrim;
extern bool     g_bVDDCbuckboosted;


extern void buck_ldo_update_override(bool bEnable);

//*****************************************************************************
//
//! @brief Place the core into sleep or deepsleep.
//!
//! @param bSleepDeep - False for Normal or True Deep sleep.
//!
//! This function puts the MCU to sleep or deepsleep depending on bSleepDeep.
//!
//! Valid values for bSleepDeep are:
//!
//!     AM_HAL_SYSCTRL_SLEEP_NORMAL
//!     AM_HAL_SYSCTRL_SLEEP_DEEP
//!
//
//*****************************************************************************
void
am_hal_sysctrl_sleep(bool bSleepDeep)
{
    am_hal_pwrctrl_mcu_mode_e ePowerMode;
    bool bRevertNvmDeepSleep = false;
#if !AM_HAL_PWRCTL_KEEP_SIMO_ACTIVE_IN_DS
    bool bSimobuckAct = false;
    uint32_t ui32VDDCsave = 0xFFFFFFFF;
    bool bBuckIntoLPinDS = false;
#endif //! AM_HAL_PWRCTL_KEEP_SIMO_ACTIVE_IN_DS

    // Ensure Stack is in TCM - Current implementation only works with TCM stack
    //
    void *pDummy;
    if (((uint32_t)&pDummy < SRAM_BASEADDR) || ((uint32_t)&pDummy >= (SRAM_BASEADDR + TCM_MAX_SIZE)))
    {
        while (1);
    }
    //
    // Disable interrupts and save the previous interrupt state.
    //
    AM_CRITICAL_BEGIN

    //
    // Get current mode.
    //
    ePowerMode = (am_hal_pwrctrl_mcu_mode_e)PWRCTRL->MCUPERFREQ_b.MCUPERFREQ;
#if !AM_HAL_PWRCTL_KEEP_SIMO_ACTIVE_IN_DS
    bSimobuckAct = ( PWRCTRL->VRSTATUS_b.SIMOBUCKST == PWRCTRL_VRSTATUS_SIMOBUCKST_ACT );
#endif //! AM_HAL_PWRCTL_KEEP_SIMO_ACTIVE_IN_DS

    //
    // If the user selected DEEPSLEEP and the TPIU is off, attempt to enter
    // DEEP SLEEP.
    //
    if ( (bSleepDeep == AM_HAL_SYSCTRL_SLEEP_DEEP) &&
         (MCUCTRL->DBGCTRL_b.DBGTPIUENABLE == MCUCTRL_DBGCTRL_DBGTPIUENABLE_DIS) )
    {

        //
        // If Crypto is On - we need to ensure that MRAM is kept ON during deepsleep
        //
        if ((PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO) && (PWRCTRL->MEMRETCFG_b.NVM0PWDSLP))
        {
            PWRCTRL->MEMRETCFG_b.NVM0PWDSLP = 0;
            bRevertNvmDeepSleep = true;
        }

        //
        // If HP mode, revert to LP.
        //
        if (  ePowerMode == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE )
        {
            am_hal_pwrctrl_mcu_mode_select(AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER);
        }

#if !AM_HAL_PWRCTL_KEEP_SIMO_ACTIVE_IN_DS
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
                if ( g_bVDDCbuckboosted )
                {
                    //
                    // Get and save the current trim value (this will obviously
                    //  be something other than 0xFFFFFFFF).
                    // Then revert the trim to the original value before deepsleep.
                    // Trimming down, so a long delay is not needed before deepsleep.
                    //
                    MCUCTRL->SIMOBUCK15_b.TRIMLATCHOVER = 1;
                    ui32VDDCsave = MCUCTRL->VREFGEN2_b.TVRGVREFTRIM;
                    MCUCTRL->VREFGEN2_b.TVRGVREFTRIM = g_ui32origSimobuckVDDCtrim;
                    am_hal_delay_us(1);
                }

#if AM_HAL_PWRCTL_SHORT_VDDF_TO_VDDS
                //
                // Restore VDDS trim and disable VDDF to VDDS short.
                //
                MCUCTRL->SIMOBUCK13_b.SIMOBUCKACTTRIMVDDS = g_ui32origSimobuckVDDStrim;
                MCUCTRL->PWRSW1_b.SHORTVDDFVDDSORVAL  = 0;
                MCUCTRL->PWRSW1_b.SHORTVDDFVDDSOREN   = 0;
#endif // AM_HAL_PWRCTL_SHORT_VDDF_TO_VDDS
                // Remove overrides to allow buck to go in LP mode
                buck_ldo_update_override(false);
            }
        }
#endif //! AM_HAL_PWRCTL_KEEP_SIMO_ACTIVE_IN_DS
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


#if !AM_HAL_PWRCTL_KEEP_SIMO_ACTIVE_IN_DS
    if ( bBuckIntoLPinDS )
    {
        // Re-enable overrides
        buck_ldo_update_override(true);

        //
        // The simobuck may not come out of LP mode if a wake from deepsleep interrupt
        // posts ~1.5us after entering deepsleep.  When this happens, the MCU exits
        // deepsleep, switches to the active state, and starts code execution.
        // However, the simobuck remains in LP mode and is unable to supply the
        // necessary amount of current to the MCU in active mode, causing the VDDC
        // and VDDF rails to dropout which could result in CPU hang.
        // The software fix is to disable and re-enable the simobuck (if the simobuck
        // has been enabled during deepsleep) immediately after deepsleep exit, which
        // manually switches the simobuck to active mode
        //
        //
        PWRCTRL->VRCTRL_b.SIMOBUCKEN = 0;
        buck_ldo_update_override(false);
        am_hal_delay_us(AM_HAL_PWRCTRL_GOTOLDO_DELAY);

        if ( ui32VDDCsave != 0xFFFFFFFF )
        {
            //
            // Restore the boosted trim.
            //
            MCUCTRL->VREFGEN2_b.TVRGVREFTRIM = ui32VDDCsave;
        }
        PWRCTRL->VRCTRL_b.SIMOBUCKEN = 1;
        buck_ldo_update_override(true);

#if AM_HAL_PWRCTL_SHORT_VDDF_TO_VDDS
        //
        // Enable VDDF to VDDS short to increase load cap (2.2uF + 2.2uF).
        //
        MCUCTRL->PWRSW1_b.SHORTVDDFVDDSORVAL  = 1;
        MCUCTRL->PWRSW1_b.SHORTVDDFVDDSOREN   = 1;

        g_ui32origSimobuckVDDStrim = MCUCTRL->SIMOBUCK13_b.SIMOBUCKACTTRIMVDDS;
        MCUCTRL->SIMOBUCK13_b.SIMOBUCKACTTRIMVDDS = 0;    // VDDS trim level to 0
#endif // AM_HAL_PWRCTL_SHORT_VDDF_TO_VDDS
    }
#endif //! AM_HAL_PWRCTL_KEEP_SIMO_ACTIVE_IN_DS

    if (bRevertNvmDeepSleep)
    {
        //
        // Revert original settings
        //
        PWRCTRL->MEMRETCFG_b.NVM0PWDSLP = 1;
    }

    //
    // Check if need to revert to HP.
    //
    if (  ePowerMode == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE )
    {
        am_hal_pwrctrl_mcu_mode_select(AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE);
    }

    //
    // Restore the interrupt state.
    //
    AM_CRITICAL_END
}

//*****************************************************************************
//
//! @brief Enable the floating point module.
//!
//! Call this function to enable the ARM hardware floating point module.
//!
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
//! @brief Disable the floating point module.
//!
//! Call this function to disable the ARM hardware floating point module.
//!
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
//! @brief Enable stacking of FPU registers on exception entry.
//!
//! @param bLazy - Set to "true" to enable "lazy stacking".
//!
//! This function allows the core to save floating-point information to the
//! stack on exception entry. Setting the bLazy option enables "lazy stacking"
//! for interrupt handlers.  Normally, mixing floating-point code and interrupt
//! driven routines causes increased interrupt latency, because the core must
//! save extra information to the stack upon exception entry. With the lazy
//! stacking option enabled, the core will skip the saving of floating-point
//! registers when possible, reducing average interrupt latency.
//!
//! @note At reset of the Cortex M4, the ASPEN and LSPEN bits are set to 1,
//! enabling Lazy mode by default. Therefore this function will generally
//! only have an affect when setting for full-context save (or when switching
//! from full-context to lazy mode).
//!
//! @note See also:
//! infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dai0298a/DAFGGBJD.html
//!
//! @note Three valid FPU context saving modes are possible.
//! 1. Lazy           ASPEN=1 LSPEN=1 am_hal_sysctrl_fpu_stacking_enable(true)
//!                                   and default.
//! 2. Full-context   ASPEN=1 LSPEN=0 am_hal_sysctrl_fpu_stacking_enable(false)
//! 3. No FPU state   ASPEN=0 LSPEN=0 am_hal_sysctrl_fpu_stacking_disable()
//! 4. Invalid        ASPEN=0 LSPEN=1
//!
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
//! @brief Disable FPU register stacking on exception entry.
//!
//! This function disables all stacking of floating point registers for
//! interrupt handlers.  This mode should only be used when it is absolutely
//! known that no FPU instructions will be executed in an ISR.
//!
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
//! @brief Issue a system wide reset using the AIRCR bit in the M4 system ctrl.
//!
//! This function issues a system wide reset (Apollo4B POR level reset).
//!
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
