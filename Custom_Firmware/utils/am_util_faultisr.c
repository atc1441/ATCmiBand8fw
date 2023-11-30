//*****************************************************************************
//
//! @file am_util_faultisr.c
//!
//! @brief An extended hard-fault handler.
//!
//! This module is portable to all Ambiq Apollo products with minimal HAL or BSP
//! dependencies (SWO output).  It collects the fault information into the sHalFaultData
//! structure, which it then prints to stdout (typically SWO).
//!
//! By default this handler, when included in the build overrides the weak binding of
//! the default hardfault handler.  It allocates 512 bytes of global variable space for
//! a local stack which guarantees diagnostic output under all hardfault conditions. If
//! the local stack is not wanted/needed, remove the macro AM_LOCAL_STACK below. If
//! the local stack is disabled, and the SP was invalid at the time of the hardfault,
//! a second hardfault can occur before any diagnostic data is collected.
//!
//! This handler outputs information about the state of the processor at the time
//! the hardfault occurred to stdout (typically SWO).  If output is not desired remove
//! the macro AM_UTIL_FAULTISR_PRINT below. When prints are disabled, the fault
//! information is available in the local sHalFaultData structure.
//!
//! The handler does not return. After outputting the diagnostic information, it
//! spins forever, it does not recover or try and return to the program that caused
//! the hardfault.
//!
//! Upon entry (caused by a hardfault), it switches to a local stack in case
//! the cause of the hardfault was a stack related issue.  The stack is sized
//! large enough to provide for the local variables and the stack space needed
//! for the ouput functions calls being used.
//!
//! It is compiler/platform independent enabling it to be used with GCC, Keil,
//! IAR and easily ported to other tools chains
//!
//! @addtogroup faultisr FaultISR - Extended Hard Fault ISR
//! @ingroup utils
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
#include "am_mcu_apollo.h"

//*****************************************************************************
//
// Macros
//
//*****************************************************************************

#define AM_LOCAL_STACK              // when defined use local stack for HF Diagnostics
//#define AM_UTIL_FAULTISR_PRINT      // when defined print output to stdout (SWO)

//
// Macros used by am_util_faultisr_collect_data().
//
#define AM_REG_SYSCTRL_CFSR_O                        0xE000ED28
#define AM_REG_SYSCTRL_BFAR_O                        0xE000ED38
#define AM_REGVAL(x)               (*((volatile uint32_t *)(x)))

//
// Macros for valid stack ranges.
//
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
  #define AM_SP_LOW    SRAM_BASEADDR
  #define AM_SP_HIGH   (SRAM_BASEADDR + RAM_TOTAL_SIZE)
#elif defined(AM_PART_APOLLO3P)
  #define AM_SP_LOW    SRAM_BASEADDR
  #define AM_SP_HIGH   (SRAM_BASEADDR + ( 768 * 1024 ))
#elif defined(AM_PART_APOLLO3)
  #define AM_SP_LOW    SRAM_BASEADDR
  #define AM_SP_HIGH   (SRAM_BASEADDR + ( 384 * 1024 ))
#elif defined(AM_PART_APOLLO2)
  #define AM_SP_LOW     SRAM_BASEADDR
  #define AM_SP_HIGH   (SRAM_BASEADDR + ( 256 * 1024 ))
#elif defined(AM_PART_APOLLO)
  #define AM_SP_LOW     SRAM_BASEADDR
  #define AM_SP_HIGH   (SRAM_BASEADDR + ( 64 * 1024 ))
#endif

//*****************************************************************************
//
// Globals
//
//*****************************************************************************

// temporary stack (in case the HF was caused by invalid stack)
#if defined(AM_LOCAL_STACK)
uint8_t gFaultStack[512];    // needs ~320 bytes (+7 for 8-byte alignment)
#endif

//*****************************************************************************
//
// Data structures
//
//*****************************************************************************
//
// Define a structure for local storage in am_util_faultisr_collect_data().
// Set structure alignment to 1 byte to minimize storage requirements.
//
#pragma pack(1)
typedef struct
{
    //
    // Stacked registers
    //
    volatile uint32_t u32R0;
    volatile uint32_t u32R1;
    volatile uint32_t u32R2;
    volatile uint32_t u32R3;
    volatile uint32_t u32R12;
    volatile uint32_t u32LR;
    volatile uint32_t u32PC;
    volatile uint32_t u32PSR;

    //
    // Other data
    //
    volatile uint32_t u32FaultAddr;
    volatile uint32_t u32BFAR;
    volatile uint32_t u32CFSR;
    volatile uint8_t  u8MMSR;
    volatile uint8_t  u8BFSR;
    volatile uint16_t u16UFSR;

} am_fault_t;


//
// Restore the default structure alignment
//
#pragma pack()

//*****************************************************************************
//
// Prototypes
//
//*****************************************************************************
void am_util_faultisr_collect_data(uint32_t u32IsrSP);
bool am_valid_sp(uint32_t u32IsrSP);

//
// Prototype for printf, if used.
//
extern uint32_t am_util_stdio_printf(char *pui8Fmt, ...);


//*****************************************************************************
//
// getStackedReg() will retrieve a specified register value, as it was stacked
// by the processor after the fault, from the stack.
//
// The registers are stacked in the following order:
//  R0, R1, R2, R3, R12, LR, PC, PSR.
// To get R0 from the stack, call getStackedReg(0), r1 is getStackedReg(1)...
//
//*****************************************************************************
#if (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION < 6000000)
__asm uint32_t
#if AM_CMSIS_REGS
HardFault_Handler(void)
#else // AM_CMSIS_REGS
am_fault_isr(void)
#endif // AM_CMSIS_REGS

#if defined(AM_LOCAL_STACK)
{
    PRESERVE8
    import  am_util_faultisr_collect_data
    import  gFaultStack
    tst     lr, #4                        // Check if we should use MSP or PSP
    ite     eq                            // Instrs executed when: eq,ne
    mrseq   r0, msp                       // t: bit2=0 indicating MSP stack
    mrsne   r0, psp                       // e: bit2=1 indicating PSP stack
    ldr     r1, =gFaultStack              // get address of the base of the temp_stack
    add     r1, r1, #512                  // address of the top of the stack.
    bic     r1, #3                        // make sure the new stack is 8-byte aligned
    mov     sp, r1                        // move the new stack address to the SP
    b       am_util_faultisr_collect_data // no return - simple branch to get fault info
    nop                                   // Avoid compiler warning about padding 2 bytes
}
#else // no local stack
{
    PRESERVE8
    import  am_util_faultisr_collect_data
    tst     lr, #4                        // Check if we should use MSP or PSP
    ite     eq                            // Instrs executed when: eq,ne
    mrseq   r0, msp                       // t: bit2=0 indicating MSP stack
    mrsne   r0, psp                       // e: bit2=1 indicating PSP stack
    b       am_util_faultisr_collect_data // no return - simple branch to get fault info
}
#endif

__asm uint32_t
getStackedReg(uint32_t regnum, uint32_t u32SP)
{
    lsls    r0, r0, #2
    adds    r0, r0, r1
    ldr     r0, [r0]
    bx      lr
}

#elif (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION > 6000000)
uint32_t __attribute__((naked))
#if AM_CMSIS_REGS
HardFault_Handler(void)
#else // AM_CMSIS_REGS
am_fault_isr(void)
#endif // AM_CMSIS_REGS
{
    __asm("    tst    lr, #4\n"                          // Check if we should use MSP or PSP
          "    ite    eq\n"                              // Instrs executed when: eq,ne
          "    mrseq  r0, msp\n"                         // t: bit2=0 indicating MSP stack
          "    mrsne  r0, psp\n");                       // e: bit2=1 indicating PSP stack
#if defined(AM_LOCAL_STACK)
    __asm("    ldr    r1, =gFaultStack\n"                // get address of the base of the temp_stack
          "    add    r1, r1, #512\n"                    // address of the top of the stack.
          "    bic    r1, #3\n"                          // make sure the new stack is 8-byte aligned
          "    mov    sp,r1\n");                         // move the new stack address to the SP
#endif
    __asm("    b      am_util_faultisr_collect_data\n"); // no return - simple branch to get fault info
}

uint32_t __attribute__((naked))
getStackedReg(uint32_t regnum, uint32_t u32SP)
{
    __asm("    lsls    r0, r0, #2");
    __asm("    adds    r0, r1");
    __asm("    ldr     r0, [r0]");
    __asm("    bx      lr");
}
#elif defined(__GNUC_STDC_INLINE__)
uint32_t __attribute__((naked))
#if AM_CMSIS_REGS
HardFault_Handler(void)
#else // AM_CMSIS_REGS
am_fault_isr(void)
#endif // AM_CMSIS_REGS
{
     __asm("    tst    lr, #4\n"                          // Check if we should use MSP or PSP
          "    ite    eq\n"                              // Instrs executed when: eq,ne
          "    mrseq  r0, msp\n"                         // t: bit2=0 indicating MSP stack
          "    mrsne  r0, psp\n");                       // e: bit2=1 indicating PSP stack
#if defined(AM_LOCAL_STACK)
    __asm("    ldr    r1, =gFaultStack\n"                // get address of the base of the temp_stack
          "    add    r1, r1, #512\n"                    // address of the top of the stack.
          "    bic    r1, #3\n"                          // make sure the new stack is 8-byte aligned
          "    mov    sp,r1\n");                         // move the new stack address to the SP
#endif
    __asm("    b      am_util_faultisr_collect_data\n"); // no return - simple branch to get fault info
}

uint32_t __attribute__((naked))
getStackedReg(uint32_t regnum, uint32_t u32SP)
{
    __asm("    lsls    r0, r0, #2");
    __asm("    adds    r0, r1");
    __asm("    ldr     r0, [r0]");
    __asm("    bx      lr");
}
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma diag_suppress = Pe940   // Suppress IAR compiler warning about missing
                                // return statement on a non-void function
__stackless uint32_t
#if AM_CMSIS_REGS
HardFault_Handler(void)
#else // AM_CMSIS_REGS
am_fault_isr(void)
#endif // AM_CMSIS_REGS
{
    __asm("    tst    lr, #4\n"                          // Check if we should use MSP or PSP
          "    ite    eq\n"                              // Instrs executed when: eq,ne
          "    mrseq  r0, msp\n"                         // t: bit2=0 indicating MSP stack
          "    mrsne  r0, psp\n");                       // e: bit2=1 indicating PSP stack
#if defined(AM_LOCAL_STACK)
    __asm("    ldr    r1, =gFaultStack\n"                // get address of the base of the temp_stack
          "    add    r1, r1, #512\n"                    // address of the top of the stack.
          "    bic    r1, #3\n"                          // make sure the new stack is 8-byte aligned
          "    mov    sp,r1\n");                         // move the new stack address to the SP
#endif
    __asm("    b      am_util_faultisr_collect_data\n"); // no return - simple branch to get fault info
}

__stackless uint32_t
getStackedReg(uint32_t regnum, uint32_t u32SP)
{
    __asm("     lsls    r0, r0, #2");
    __asm("     adds    r0, r0, r1");
    __asm("     ldr     r0, [r0]");
    __asm("     bx      lr");
}
#pragma diag_default = Pe940    // Restore IAR compiler warning
#endif

//*****************************************************************************
//
// am_util_faultisr_collect_data(uint32_t u32IsrSP);
//
// This function is intended to be called by HardFault_Handler(), called
// when the processor receives a hard fault interrupt.  This part of the
// handler parses through the various fault codes and saves them into a data
// structure so they can be readily examined by the user in the debugger.
//
// The input u32IsrSP is expected to be the value of the stack pointer when
// HardFault_Handler() was called.
//
//*****************************************************************************
void
am_util_faultisr_collect_data(uint32_t u32IsrSP)
{
    volatile am_fault_t sFaultData;
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    am_hal_fault_status_t  sHalFaultData = {0};
#elif defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P) || defined(AM_PART_APOLLO2) || defined(AM_PART_APOLLO)
    am_hal_mcuctrl_fault_t sHalFaultData = {0};
#endif // if defined(AM_PART_APOLLO4X)

    uint32_t u32Mask = 0;

    //
    // Following is a brief overview of fault information provided by the M4.
    // More details can be found in the Cortex M4 User Guide.
    //
    // CFSR (Configurable Fault Status Reg) contains MMSR, BFSR, and UFSR:
    //   7:0    MMSR (MemManage)
    //          [0] IACCVIOL    Instr fetch from a location that does not
    //                          permit execution.
    //          [1] DACCVIOL    Data access violation flag. MMAR contains
    //                          address of the attempted access.
    //          [2] Reserved
    //          [3] MUNSTKERR   MemMange fault on unstacking for a return
    //                          from exception.
    //          [4] MSTKERR     MemMange fault on stacking for exception
    //                          entry.
    //          [5] MLSPERR     MemMange fault during FP lazy state
    //                          preservation.
    //          [6] Reserved
    //          [7] MMARVALID   MemManage Fault Addr Reg (MMFAR) valid flag.
    //  15:8    BusFault
    //          [0] IBUSERR     If set, instruction bus error.
    //          [1] PRECISERR   Data bus error. Stacked PC points to instr
    //                          that caused the fault.
    //          [2] IMPRECISERR Data bus error, but stacked return addr is not
    //                          related to the instr that caused the error and
    //                          BFAR is not valid.
    //          [3] UNSTKERR    Bus fault on unstacking for a return from
    //                          exception.
    //          [4] STKERR      Bus fault on stacking for exception entry.
    //          [5] LSPERR      Bus fault during FP lazy state preservation.
    //          [6] Reserved
    //          [7] BFARVALID   BFAR valid.
    //  31:16   UFSR (UsageFault)
    //          [0] UNDEFINSTR  Undefined instruction.
    //          [1] INVSTATE    Invalid state.
    //          [2] INVPC       Invalid PC load.
    //          [3] NOCP        No coprocessor.
    //        [7:4] Reserved
    //          [8] UNALIGNED   Unaligned access.
    //          [9] DIVBYZERO   Divide by zero.
    //      [15:10] Reserved
    //

    //
    // u32Mask is used for 2 things: 1) in the print loop, 2) as a spot to set
    // a breakpoint at the end of the routine.  If the printing is not used,
    // we'll get a compiler warning; so to avoid that warning, we'll use it
    // in a dummy assignment here.
    //
    sFaultData.u32CFSR = u32Mask;       // Avoid compiler warning
    sFaultData.u32CFSR = AM_REGVAL(AM_REG_SYSCTRL_CFSR_O);
    sFaultData.u8MMSR  = (sFaultData.u32CFSR >> 0)  & 0xff;
    sFaultData.u8BFSR  = (sFaultData.u32CFSR >> 8)  & 0xff;
    sFaultData.u16UFSR = (sFaultData.u32CFSR >> 16) & 0xffff;

    //
    // The address of the location that caused the fault.  e.g. if accessing an
    // invalid data location caused the fault, that address will appear here.
    //
    sFaultData.u32BFAR = AM_REGVAL(AM_REG_SYSCTRL_BFAR_O);

    // make sure that the SP points to a valid address (so that accessing the stack frame doesn't cause another fault).
    if (am_valid_sp(u32IsrSP))
    {
        //
        // The address of the instruction that caused the fault is the stacked PC
        // if BFSR bit1 is set.
        //
        sFaultData.u32FaultAddr = (sFaultData.u8BFSR & 0x02) ? getStackedReg(6, u32IsrSP) : 0xffffffff;

        //
        // Get the stacked registers.
        // Note - the address of the instruction that caused the fault is u32PC.
        //
        sFaultData.u32R0  = getStackedReg(0, u32IsrSP);
        sFaultData.u32R1  = getStackedReg(1, u32IsrSP);
        sFaultData.u32R2  = getStackedReg(2, u32IsrSP);
        sFaultData.u32R3  = getStackedReg(3, u32IsrSP);
        sFaultData.u32R12 = getStackedReg(4, u32IsrSP);
        sFaultData.u32LR  = getStackedReg(5, u32IsrSP);
        sFaultData.u32PC  = getStackedReg(6, u32IsrSP);
        sFaultData.u32PSR = getStackedReg(7, u32IsrSP);
    }
    //
    // Use the HAL MCUCTRL functions to read the fault data.
    //
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    am_hal_fault_status_get(&sHalFaultData);
#elif defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
    am_hal_mcuctrl_info_get(AM_HAL_MCUCTRL_INFO_FAULT_STATUS, &sHalFaultData);
#elif defined(AM_PART_APOLLO2) || defined(AM_PART_APOLLO)
    am_hal_mcuctrl_fault_status(&sHalFaultData);
#endif

#ifdef AM_UTIL_FAULTISR_PRINT
    //
    // If printf has previously been initialized in the application, we should
    // be able to print out the fault information.
    //
    am_util_stdio_printf("** Hard Fault Occurred:\n\n");
    if (!am_valid_sp(u32IsrSP))
    {
        am_util_stdio_printf("    Invalid SP when Hard Fault occured: 0x%08X (no Stacked data)\n\n");
    }
    else
    {
        am_util_stdio_printf("Hard Fault stacked data:\n");
        am_util_stdio_printf("    R0  = 0x%08X\n", sFaultData.u32R0);
        am_util_stdio_printf("    R1  = 0x%08X\n", sFaultData.u32R1);
        am_util_stdio_printf("    R2  = 0x%08X\n", sFaultData.u32R2);
        am_util_stdio_printf("    R3  = 0x%08X\n", sFaultData.u32R3);
        am_util_stdio_printf("    R12 = 0x%08X\n", sFaultData.u32R12);
        am_util_stdio_printf("    LR  = 0x%08X\n", sFaultData.u32LR);
        am_util_stdio_printf("    PC  = 0x%08X\n", sFaultData.u32PC);
        am_util_stdio_printf("    PSR = 0x%08X\n\n", sFaultData.u32PSR);
    }
    am_util_stdio_printf("Other Hard Fault data:\n");
    am_util_stdio_printf("    Fault address = 0x%08X\n", sFaultData.u32FaultAddr);
    am_util_stdio_printf("    BFAR (Bus Fault Addr Reg) = 0x%08X\n", sFaultData.u32BFAR);
    am_util_stdio_printf("    MMSR (Mem Mgmt Fault Status Reg) = 0x%02X\n", sFaultData.u8MMSR);
    am_util_stdio_printf("    UFSR (Usage Fault Status Reg) = 0x%04X\n", sFaultData.u16UFSR);
    am_util_stdio_printf("    BFSR (Bus Fault Status Reg) = 0x%02X\n", sFaultData.u8BFSR);
    //
    // Print out any bits set in the BFSR.
    //
    u32Mask = 0x80;
    while (u32Mask)
    {
        switch (sFaultData.u8BFSR & u32Mask)
        {
            case 0x80:
                am_util_stdio_printf("        BFSR bit7: BFARVALID\n");
                break;
            case 0x40:
                am_util_stdio_printf("        BFSR bit6: RESERVED\n");
                break;
            case 0x20:
                am_util_stdio_printf("        BFSR bit5: LSPERR\n");
                break;
            case 0x10:
                am_util_stdio_printf("        BFSR bit4: STKERR\n");
                break;
            case 0x08:
                am_util_stdio_printf("        BFSR bit3: UNSTKERR\n");
                break;
            case 0x04:
                am_util_stdio_printf("        BFSR bit2: IMPRECISERR\n");
                break;
            case 0x02:
                am_util_stdio_printf("        BFSR bit1: PRECISEERR\n");
                break;
            case 0x01:
                am_util_stdio_printf("        BFSR bit0: IBUSERR\n");
                break;
            default:
                break;
        }
        u32Mask >>= 1;
    }

    //
    // Print out any Apollo* Internal fault information - if any
    //
    if (sHalFaultData.bICODE || sHalFaultData.bDCODE || sHalFaultData.bSYS)
    {
        am_util_stdio_printf("\nMCU Fault data:\n");
    }
    if (sHalFaultData.bICODE)
    {
        am_util_stdio_printf("    ICODE Fault Address: 0x%08X\n", sHalFaultData.ui32ICODE);
    }
    if (sHalFaultData.bDCODE)
    {
        am_util_stdio_printf("    DCODE Fault Address: 0x%08X\n", sHalFaultData.ui32DCODE);
    }
    if (sHalFaultData.bSYS)
    {
        am_util_stdio_printf("    SYS Fault Address: 0x%08X\n", sHalFaultData.ui32SYS);
    }
    //
    // Spin in an infinite loop.
    // We need to spin here inside the function so that we have access to
    // local data, i.e. sFaultData.
    //
    am_util_stdio_printf("\n\nDone with output. Entering infinite loop.\n\n");

#endif  // AM_UTIL_FAULTISR_PRINT

    u32Mask = 0;

    while (1)  {  };   // spin forever
}

//*****************************************************************************
//
// am_valid_sp(uint32_t u32IsrSP);
//
// This function does a range on the SP to make sure it appears to be valid
//
// The input param u32IsrSP is expected to be the value of the stack pointer in
// use when the hardfault occured.
//
//*****************************************************************************
bool
am_valid_sp(uint32_t u32IsrSP)
{
    return ( (u32IsrSP >= AM_SP_LOW) && (u32IsrSP < AM_SP_HIGH) ) ? true : false;
}
//*****************************************************************************


//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
