//*****************************************************************************
//
//! @file am_reg_macros.h
//!
//! @brief Helper macros for using hardware registers.
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

#ifndef AM_REG_MACROS_H
#define AM_REG_MACROS_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// For direct 32-bit access to a register or memory location, use AM_REGVAL:
//      AM_REGVAL(0x1234567) |= 0xDEADBEEF;
//
//*****************************************************************************
#define AM_REGVAL(x)               (*((volatile uint32_t *)(x)))
#define AM_REGVAL_FLOAT(x)         (*((volatile float *)(x)))

//*****************************************************************************
//
// AM_REGADDR()
// One thing CMSIS does not do well natively is to provide for static register
// address computation. The address-of operator (e.g. &periph->reg) is helpful,
// but does run into problems, such as when attempting to cast the resulting
// pointer to a uint32_t.  The standard C macro, offsetof() can help.
//
// Use AM_REGADDR() for single-module peripherals.
// Use AM_REGADDRn() for multi-module peripherals (e.g. IOM, UART).
//
//*****************************************************************************
#define AM_REGADDR(periph, reg) ( periph##_BASE + offsetof(periph##_Type, reg) )

#define AM_REGADDRn(periph, n, reg) ( periph##0_BASE                    +   \
                                      offsetof(periph##0_Type, reg)     +   \
                                      (n * (periph##1_BASE - periph##0_BASE)) )

//*****************************************************************************
//
// Bitfield macros.
//
//*****************************************************************************
#ifdef CMSIS_BFNOT
#define AM_BFW(module, reg, field, value)                                   \
    (module->reg = ((module->reg & ~(module##_##reg##_##field##_Msk)) |     \
    _VAL2FLD(module##_##reg##_##field, value)))

#define AM_BFR(module, reg, field)                                          \
    _FLD2VAL(module##_##reg##_##field, module->reg)

#define AM_BFRn(module, instance, reg, field)                               \
    (_FLD2VAL(module##0_##reg##_##field,                                    \
     *((volatile uint32_t *)(AM_REGADDRn(module, instance, reg)))) )

#define AM_BFWn(module, instance, reg, field, value)                        \
    (*((volatile uint32_t *)(AM_REGADDRn(module, instance, reg))) =         \
     (((*((volatile uint32_t *)(AM_REGADDRn(module, instance, reg)))) &     \
     ~(module##0_##reg##_##field##_Msk) ) |                                 \
     _VAL2FLD(module##0_##reg##_##field, value)) )

#endif

//*****************************************************************************
//
// Critical section assembly macros
//
// These macros implement critical section protection using inline assembly
// for various compilers.  They are intended to be used in other register
// macros or directly in sections of code.
//
// Important usage note: These macros create a local scope and therefore MUST
// be used in pairs.
//
//*****************************************************************************
#define AM_CRITICAL_BEGIN                                                   \
    if ( 1 )                                                                \
    {                                                                       \
        volatile uint32_t ui32Primask_04172010;                             \
        ui32Primask_04172010 = am_hal_interrupt_master_disable();

#define AM_CRITICAL_END                                                     \
        am_hal_interrupt_master_set(ui32Primask_04172010);                  \
    }

//*****************************************************************************
//
// Compiler-specific macros.
//
// Macros that accomplish compiler-specific tasks.
// For example, suppression of certain compiler warnings.
//
//*****************************************************************************
#if defined(__IAR_SYSTEMS_ICC__)
/* Suppress IAR compiler warning about volatile ordering  */
#define DIAG_SUPPRESS_VOLATILE_ORDER()  _Pragma("diag_suppress=Pa082")
/* Restore IAR compiler warning to default */
#define DIAG_DEFAULT_VOLATILE_ORDER()   _Pragma("diag_default=Pa082")
#else
#define DIAG_SUPPRESS_VOLATILE_ORDER()
#define DIAG_DEFAULT_VOLATILE_ORDER()
#endif

//
// The intrinsic for IAR's CLZ instruction is different than other compilers.
//
#ifdef __IAR_SYSTEMS_ICC__
#define AM_ASM_CLZ(ui32val)     __CLZ(ui32val)
#else
#define AM_ASM_CLZ(ui32val)     __builtin_clz(ui32val)
#endif

#ifdef __cplusplus
}
#endif

#endif // AM_REG_MACROS_H

