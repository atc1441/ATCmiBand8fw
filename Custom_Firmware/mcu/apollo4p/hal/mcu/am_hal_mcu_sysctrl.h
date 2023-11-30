//*****************************************************************************
//
//! @file am_hal_mcu_sysctrl.h
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
#ifndef AM_HAL_MCU_SYSCTRL_H
#define AM_HAL_MCU_SYSCTRL_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! Write flush - This function will hold the bus until all queued write
//! operations have completed, thereby guaranteeing that all writes have
//! been flushed.
//
//*****************************************************************************
#define am_hal_sysctrl_membarrier()                                     \
    if (1)                                                              \
    {                                                                   \
        __DMB();                                                        \
        am_hal_sysctrl_bus_write_flush();                               \
    }

//
//! This Write will begin only after all previous load/store operations are done
//
#define am_hal_sysctrl_membarrier_write(addr, data)                     \
    if (1)                                                              \
    {                                                                   \
        am_hal_sysctrl_membarrier();                                    \
        *((uint32_t *)(addr)) = (data);                                 \
    }

//
//! This Read will be performed before any subsequent load/store operations are done
//
#define am_hal_sysctrl_membarrier_read(addr)                            \
    if (1)                                                              \
    {                                                                   \
        AM_REGVAL((addr));                                              \
        am_hal_sysctrl_membarrier();                                    \
    }

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Enable the floating point module.
//!
//! Call this function to enable the ARM hardware floating point module.
//
//*****************************************************************************
extern void am_hal_sysctrl_fpu_enable(void);

//*****************************************************************************
//
//! @brief Disable the floating point module.
//!
//! Call this function to disable the ARM hardware floating point module.
//
//*****************************************************************************
extern void am_hal_sysctrl_fpu_disable(void);

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
//
//*****************************************************************************
extern void am_hal_sysctrl_fpu_stacking_enable(bool bLazy);

//*****************************************************************************
//
//! @brief Disable FPU register stacking on exception entry.
//!
//! This function disables all stacking of floating point registers for
//! interrupt handlers.  This mode should only be used when it is absolutely
//! known that no FPU instructions will be executed in an ISR.
//
//*****************************************************************************
extern void am_hal_sysctrl_fpu_stacking_disable(void);

//*****************************************************************************
//
//! @brief Issue a system wide reset using the AIRCR bit in the M4 system ctrl.
//!
//! This function issues a system wide reset (Apollo4P POR level reset).
//
//*****************************************************************************
extern void am_hal_sysctrl_aircr_reset(void);
#ifdef __cplusplus
}
#endif

#endif // AM_HAL_MCU_SYSCTRL_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

