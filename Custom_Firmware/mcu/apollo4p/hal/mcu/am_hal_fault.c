//*****************************************************************************
//
//! @file am_hal_fault.c
//!
//! @brief Functions for interfacing with the fault control.
//!
//! @addtogroup fault_4p Fault - CPU Fault Control
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

// ****************************************************************************
//
//  am_hal_fault_capture_enable()
//  This function is used to enable fault capture on the CPU block.
//
// ****************************************************************************
uint32_t
am_hal_fault_capture_enable(void)
{
    //
    // Enable the Fault Capture registers.
    //
    CPU->FAULTCAPTUREEN_b.FAULTCAPTUREEN = CPU_FAULTCAPTUREEN_FAULTCAPTUREEN_EN;

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;
} // am_hal_fault_capture_enable()

// ****************************************************************************
//
//  am_hal_fault_capture_disable()
//  This function is used to disable fault capture on the CPU block.
//
// ****************************************************************************
uint32_t
am_hal_fault_capture_disable(void)
{
    //
    // Enable the Fault Capture registers.
    //
    CPU->FAULTCAPTUREEN_b.FAULTCAPTUREEN = CPU_FAULTCAPTUREEN_FAULTCAPTUREEN_DIS;

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;
} // am_hal_fault_capture_disable()

// ****************************************************************************
//
// am_hal_fault_status_get()
// This function returns  current fault status as obtained from the CPU block
// in Apollo4.
//
// ****************************************************************************
uint32_t
am_hal_fault_status_get(am_hal_fault_status_t *pFaultStatus)
{
    uint32_t  ui32FaultStat;

    if ( pFaultStatus == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Read the Fault Status Register.
    //
    ui32FaultStat = CPU->FAULTSTATUS;
    pFaultStatus->bICODE = (bool)(ui32FaultStat & CPU_FAULTSTATUS_ICODEFAULT_Msk);
    pFaultStatus->bDCODE = (bool)(ui32FaultStat & CPU_FAULTSTATUS_DCODEFAULT_Msk);
    pFaultStatus->bSYS   = (bool)(ui32FaultStat & CPU_FAULTSTATUS_SYSFAULT_Msk);

    //
    // Read the DCODE fault capture address register.
    //
    pFaultStatus->ui32DCODE = CPU->DCODEFAULTADDR;

    //
    // Read the ICODE fault capture address register.
    //
    pFaultStatus->ui32ICODE |= CPU->ICODEFAULTADDR;

    //
    // Read the ICODE fault capture address register.
    //
    pFaultStatus->ui32SYS |= CPU->SYSFAULTADDR;

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;
} // am_hal_fault_status_get()

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
