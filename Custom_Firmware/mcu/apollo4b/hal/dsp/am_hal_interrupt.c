//*****************************************************************************
//
//! @file am_hal_interrupt.c
//!
//! @brief Helper functions supporting DSP interrupts.
//!
//! @addtogroup dsp_interrupt_4b Interrupt (DSP NVIC support functions)
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
#include <stddef.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_hal_internal_dsp.h"

//#define NULL 0
#define MAX_NO_OF_HANDLERS    32   //!< number of handlers supported per DSP core
//*****************************************************************************
//
//! @brief the linked list structure for the interrupt handlers defined by the user code
//!
//! This structure keeps the handler address with some information about the
//! status register bit associated with the interrupt.
//!
//
//*****************************************************************************
/*
   typedef struct
   {
        void     *pHandler;
        void     *pHandlerCtxt;
        void     *pIRQStatusRegAdd;
        uint32_t  IRQStatusRegVal;
        void     *dNextIntHandler;
        uint8_t   ui5IntterruptNo;// This is a 5bit value(3 to 25) for the current interrupt for this handler
        //  this is used by the am_hal_interrupt_register_handler to find all interrupts associated for the current
        // DSP interrupt.
        uint8_t   ui1FirstInterrupt; // when this flag is set. Then this Handler is the head of the linked list.
        uint8_t   ui8PeripheralIntNo; // The source interrupt number for this handler. This is useful for unregister function
        uint8_t   ui8PreviousElementIndex; // when 0xff then this Handler is the head of the linked list, else this is the link.
        uint32_t reserve[2]; // two of the members here can be used for one extra register access
   }IntHandlerStruct;
 */
void     am_hal_dsp_dispatcher(IntHandlerStruct * ihs); // this function is defined in assembly

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
//!
IntHandlerStruct sIntHandler[MAX_NO_OF_HANDLERS];

// The next lines should be removed
#define DSP_CORE0
#ifdef DSP_CORE0
#define DSPRAWIRQSTAT0    DSP->DSP0INTORMASK   // DSP0RAWIRQSTAT0
// replace the next line with actual register definition
int abc[10];
#define REG_DSP_DSP0OR1_0    abc
#define REG_DSP_DSP0OR2_0    abc
#elif DSP_CORE1
#define DSPRAWIRQSTAT0    DSP1RAWIRQSTAT0
#endif
// assumption:
// REG_DSP_DSP0OR1_0,...4
// REG_DSP_DSP0OR2_0,...4

//*****************************************************************************
//
//! @brief am_hal_interrupt_disable disables the raw interrupt( no more interrupt from the source)
//!
//! The function disable the raw interrupt by disconnecting the interrupt on the
//! source. The interrupt can be disable from the source or disable inside the DSP
//! by INTERRUPT register. This function masks the interrupt on the source and does
//! change the INTERRUPT register.
//!
//! @param ui8InterruptNo - The raw interrupt number (0 to 159)
//
//*****************************************************************************
void
am_hal_interrupt_disable(uint32_t ui8InterruptNo)
{
    uint8_t OrIndex;
    OrIndex = ui8InterruptNo>>5;
    if ( sHalSharedMem->sHalDSPInt[ui8InterruptNo].Or1 )
    {
        //clears OR1 bit
        REG_DSP_DSP0OR1_0[OrIndex] &= ~( 1 << (ui8InterruptNo & 0x1f) );
        // DOn't need to disable inside the DSP     _xtos_interrupt_disable();
    }
    if ( sHalSharedMem->sHalDSPInt[ui8InterruptNo].Or2 )
    {
        //clears OR1 bit
        REG_DSP_DSP0OR2_0[OrIndex] &= ~( 1 << (ui8InterruptNo & 0x1f) );
        // DOn't need to disable inside the DSP     _xtos_interrupt_disable();
    }
}

//*****************************************************************************
//
//! @brief am_hal_interrupt_enable enables the raw interrupt
//!
//! The function restore the raw interrupt connection between the source and DSP
//! pin. To enable an interrupt the raw source should be connected to DSP input
//! pins and then the corresponding INTERRUPT flag should be enabled.
//!
//! @param ui8InterruptNo  : The raw interrupt number (0 to 159)
//
//*****************************************************************************
void
am_hal_interrupt_enable(uint32_t ui8InterruptNo)
{
    uint8_t OrIndex;
    OrIndex = ui8InterruptNo>>5;
    if ( sHalSharedMem->sHalDSPInt[ui8InterruptNo].Or1 )
    {
        //clears OR1 bit
        REG_DSP_DSP0OR1_0[OrIndex] |= ( 1 << (ui8InterruptNo & 0x1f) );
        if ( sHalSharedMem->sHalDSPInt[ui8InterruptNo].intNo != INVALID_INTERRUPT)
        {
            _xtos_interrupt_enable(sHalSharedMem->sHalDSPInt[ui8InterruptNo].intNo);
        }
    }
    if ( sHalSharedMem->sHalDSPInt[ui8InterruptNo].Or2 )
    {
        //clears OR1 bit
        REG_DSP_DSP0OR2_0[OrIndex] |= ( 1 << (ui8InterruptNo & 0x1f) );
        if ( sHalSharedMem->sHalDSPInt[ui8InterruptNo].intNo != INVALID_INTERRUPT)
        {
            _xtos_interrupt_enable(sHalSharedMem->sHalDSPInt[ui8InterruptNo].intNo);
        }
    }

}

//*****************************************************************************
//
//! @brief am_hal_interrupt_master_disable disables all interrupt on DSP
//!
//! To disable all interrupts in DSP, the interrupt level should be raised to a
//! level higher the maximum level. Currently the maximum level is 6. This function
//! raises the interrupt level to 15 to disable all interrupt (the PS register have
//! 4 bits for interrupt level).
//! Note: disabling interrupt can not disable the exceptions.
//!
//! @return the contents of PS register.
//
//*****************************************************************************

uint32_t
am_hal_interrupt_master_disable(void)
{
    // return a value that represents the previous interrupt level mask. This
    // value can be passed to _xtos_restore_intlevel() to restore the interrupt
    // level mask to what it was before the interrupt level mask was set. The
    // format of the return value cannot be relied upon (In the current implementation,
    // this interrupt level mask value is the contents of the PS special register).
    return _xtos_set_intlevel(15);
}

//*****************************************************************************
//
//! @brief am_hal_interrupt_master_enable enables all interrupt on DSP
//!
//! To enable all interrupts in DSP, the interrupt level should be set to zero.
//! This function regardless of the current interrupt level, changes the interrupt
//! level to zero. The interrupt level is specified by 4 bits inside PS register.
//!
//! @return the contents of PS register.
//
//*****************************************************************************
uint32_t
am_hal_interrupt_master_enable(void)
{
    // return a value that represents the previous interrupt level mask. This
    // value can be passed to _xtos_restore_intlevel() to restore the interrupt
    // level mask to what it was before the interrupt level mask was set. The
    // format of the return value cannot be relied upon (In the current implementation,
    // this interrupt level mask value is the contents of the PS special register.
    return _xtos_set_intlevel(0);
}

//*****************************************************************************
//
//! @brief am_hal_interrupt_master_restore restores the interrupt level
//!
//! The function restore the interrupt level to its previous known state. DSP has
//! six level of interrupt. 18+7 of interrupts are level one and 5 more interrupts
//! are at level 2 to 6. changing the interrupt level can enable or disable all
//! interrupts with the same levels.
//!
//! @param ui32Restoreval
//! @return the contents of PS register.
//
//*****************************************************************************
uint32_t
am_hal_interrupt_master_restore(uint32_t ui32Restoreval)
{
    // This function restore the current interrupt level mask according to a value
    // returned by _xtos_set_intlevel().
    _xtos_set_intlevel(ui32Restoreval);
    return 1;
}

//*****************************************************************************
//
// am_hal_interrupt_master_set set the interrupt level
//
// The function can change the interrupt level to any level. By changing the
// interrupt level, all interrupts with the lower level are disabled.
//
//*****************************************************************************
uint32_t
am_hal_interrupt_master_set(uint32_t ui32Level)
{
    // return a value that represents the previous interrupt level mask. This value
    // can be passed to _xtos_restore_intlevel() to restore the interrupt level
    // mask to what it was before the interrupt level mask was set. The format of
    // the return value cannot be relied upon (In the current implementation,
    // this interrupt level mask value is the contents of the PS special register.
    return _xtos_set_intlevel(ui32Level);
}

//*****************************************************************************
//
//! @brief am_hal_interrupt_register_handler registers a handler for interrupt
//!
//! The function can change the interrupt level to any level. By changing the
//! interrupt level, all interrupts with the lower level are disabled.
//!
//! @param ui32Interrupt - The interrupt number (0 .. XCHAL_NUM_INTERRUPTS - 1)
//!
//! @param pHandler      - Address of interrupt handler to be registered.
//!                        Passing NULL will uninstall an existing handler.
//!
//! @param pHandlerCtxt  - Parameter to be passed to handler when invoked.
//!
//! @return             Zero on success, negative number on error.
//
//*****************************************************************************

uint32_t
am_hal_interrupt_register_handler(uint32_t ui32Interrupt, void *pHandler, void *pHandlerCtxt)
{
// This function registers the input function address(*pHandler) for the interrupt number ui32Interrupt.
// The external 160 interrupts are passing through ORed structured and then connected to 23 input interrupt
// of the DSP(interrupt No 3 to No 25). Each of these 23 interrupts has a unique handler inside the XTOS.
// If only one interrupt is connected to DSP interrupt, then the handler(*pHandler) is directly registered
// in XTOS. If more than one external interrupt is connected to a single DSP interrupt, then a linked list
// is created for all interrupts and am_hal_dsp_dispatcher() is registered as an handler inside  XTOS.

    // To keep track of all handlers, a handler pool is created. Handlers are stored inside the pool.
    // The maximum number of handler is limited by the handler pool capacity(MAX_NO_OF_HANDLERS).
    int16_t nextFreeHandlerIndex, lastHandlerIndex, firstHandlerIndex, handlerIndex;
    int statusIndex;
    nextFreeHandlerIndex = -1;
    lastHandlerIndex = -1;
    firstHandlerIndex = -1;

    if ( pHandler == NULL)
    {
        // unregister all the functions associated with ui32Interrupt
        // The for-loop is used because it is possible that an interrupt is registered for multiple interrupt level!!
        for ( handlerIndex = 0; handlerIndex < MAX_NO_OF_HANDLERS; handlerIndex++)
        {
            if ( sIntHandler[handlerIndex].pHandler != 0)
            {
                if (sIntHandler[handlerIndex].ui8PeripheralIntNo == ui32Interrupt)
                {
                    // The handler found
                    // Now removes this handler from the link-list. Removing the first element inside a linked-list
                    // is different than non-first elements.
// this should be done in critical section
                    sIntHandler[handlerIndex].pHandler = NULL;
                    //
                    if ( sIntHandler[handlerIndex].ui8PreviousElementIndex == 0xff)
                    {
                        // if this is the first(or the only) handler in the chain.
                        if ( sIntHandler[handlerIndex].dNextIntHandler == NULL)
                        {
                            // no more handler on the chain. Unregister the function.
#ifdef XTOS
                            xtos_set_interrupt_handler( sIntHandler[handlerIndex].ui5IntterruptNo, NULL, NULL, NULL);
#elif XOS
                            xos_register_interrupt_handler( sIntHandler[handlerIndex].ui5IntterruptNo, NULL, NULL );
#endif
                        }
                        else
                        {
                            // more handler on the chain.
                            // add assert here if the new handler is not valid.
                            ((IntHandlerStruct *)(sIntHandler[handlerIndex].dNextIntHandler))->ui8PreviousElementIndex = 0xff;
                            if ( ((IntHandlerStruct *)(sIntHandler[handlerIndex].dNextIntHandler))->dNextIntHandler == NULL  )
                            {
                                // If the new handler is the only handler, then register the handler directly.
#ifdef XTOS
                                xtos_set_interrupt_handler( sIntHandler[handlerIndex].ui5IntterruptNo, ((IntHandlerStruct *)(sIntHandler[handlerIndex].dNextIntHandler))->pHandler, ((IntHandlerStruct *)(sIntHandler[handlerIndex].dNextIntHandler)), NULL);
#elif XOS
                                xos_register_interrupt_handler(sIntHandler[handlerIndex].ui5IntterruptNo, ((IntHandlerStruct *)(sIntHandler[handlerIndex].dNextIntHandler))->pHandler, ((IntHandlerStruct *)(sIntHandler[handlerIndex].dNextIntHandler)) );
#endif
                            }
                            else
                            {
                                // if more than one handler is in the list, then update the chain with the new linked-list.
#ifdef XTOS
                                xtos_set_interrupt_handler( sIntHandler[handlerIndex].ui5IntterruptNo, (void *) am_hal_dsp_dispatcher, ((void *)(sIntHandler[handlerIndex].dNextIntHandler)), NULL);
#elif XOS
                                xos_register_interrupt_handler(sIntHandler[handlerIndex].ui5IntterruptNo, am_hal_dsp_dispatcher, sIntHandler[nextFreeHandlerIndex] );
#endif
                            }
                        }
                    }
                    else
                    {
                        // This is not the first element in the list.
                        // Find the previous element and then connect the previous element to the next element.
                        sIntHandler[ sIntHandler[handlerIndex].ui8PreviousElementIndex ].dNextIntHandler = sIntHandler[handlerIndex].dNextIntHandler;
                    }
// end of critical section
                }
            }
        }
        return 0;
    }

    // find the next free handler from the pool
    if ( sHalSharedMem->sHalDSPInt[ui32Interrupt].intNo == 31 )
    {
        //  no valid interrupt is assigned for this peripherals in shared memory
        return -2;
    }
    for ( handlerIndex = 0; handlerIndex < MAX_NO_OF_HANDLERS; handlerIndex++ )
    {
        if ( sIntHandler[handlerIndex].pHandler == NULL )
        {
            if ( nextFreeHandlerIndex == -1)
            {
                // a free Handler found in the pool
                // add the handler to the pool
                sIntHandler[handlerIndex].pHandler        = pHandler;
                sIntHandler[handlerIndex].pHandlerCtxt    = pHandlerCtxt;
                sIntHandler[handlerIndex].ui5IntterruptNo = sHalSharedMem->sHalDSPInt[ui32Interrupt].intNo;
                statusIndex = ui32Interrupt >> 5;         // find the index address for the status register
                //sIntHandler[handlerIndex].pIRQStatusRegAdd= &(( (uint32_t *)(&(DSPRAWIRQSTAT0)) )[statusIndex]);
                // the status register bit
                sIntHandler[handlerIndex].IRQStatusRegVal = 1 << (ui32Interrupt & 0x1f );
                sIntHandler[handlerIndex].dNextIntHandler = 0;
                sIntHandler[handlerIndex].ui8PreviousElementIndex = 0xff;         // for now assume this is not the first interrupt in the linked-list
                nextFreeHandlerIndex = handlerIndex;
            }
        }
        else
        {
            // Here  a brute force search is done to find the first and last element in the linked-list. This is just for more safety.
            // for all Handlers in the pool, look for those with the same DSP interrupt No.
            if (sIntHandler[handlerIndex].ui5IntterruptNo == sHalSharedMem->sHalDSPInt[ui32Interrupt].intNo )
            {
                // find the first handler in the linked-list.
                if ( sIntHandler[handlerIndex].ui8PreviousElementIndex == 0xff)
                {
                    // locate the first handler and use its address for dispatcher
                    firstHandlerIndex = handlerIndex;
                }
                // find the last handler in the linked-list.
                if ( sIntHandler[handlerIndex].dNextIntHandler == 0 )
                {
                    if ( lastHandlerIndex != -1)
                    {
                        //assert. The linked-list has multiple ends.
                    }
                    lastHandlerIndex = handlerIndex;
                }
            }
        }
    }
    if ( nextFreeHandlerIndex == -1 )
    {
        // No free space available in the pool. Solution:
        // 1- Make sure to use unregister function when any handler is not needed any more. The unregister function
        // remove the handler from the pool.
        // 2- If the number of handlers are more than MAX_NO_OF_HANDLERS, increase MAX_NO_OF_HANDLERS.
        return -1;
    }
// critical section starts here
    if ( lastHandlerIndex >= 0)
    {
        // add the handler to the end of the linked-list.
        sIntHandler[lastHandlerIndex].dNextIntHandler = (void*)&sIntHandler[nextFreeHandlerIndex];
        // Connect the previous handler in the linked-list to the current handler.
        // The previous handler is useful for unregister an interrupt(removing from the linked-list)
        sIntHandler[nextFreeHandlerIndex].ui8PreviousElementIndex = lastHandlerIndex;

        // Multiple handlers are registered for a single DSP interrupt.
        // So register the common interrupt handler (am_hal_dsp_dispatcher).
        if ( firstHandlerIndex == -1 )
        {
            //assert
        }
#ifdef XTOS
        xtos_set_interrupt_handler( sIntHandler[nextFreeHandlerIndex].ui5IntterruptNo, (xtos_handler)(void * )am_hal_dsp_dispatcher, (void *)&sIntHandler[firstHandlerIndex], NULL);
#elif XOS
        xos_register_interrupt_handler(sIntHandler[nextFreeHandlerIndex].ui5IntterruptNo, am_hal_dsp_dispatcher, sIntHandler[firstHandlerIndex] );
#else
        #error "XTOS or XOS must be defined."
#endif
    }
    else
    {
        // There is no other handlers registered for this interrupt
        // So register the pHandler directly inside the XTOS
        // sIntHandler[nextFreeHandlerIndex].ui8PreviousElementIndex = 0xff;// this is the first interrupt in the linked-list
#ifdef XTOS
        xtos_set_interrupt_handler(sIntHandler[nextFreeHandlerIndex].ui5IntterruptNo, pHandler, pHandlerCtxt, NULL);
#elif XOS
        xos_register_interrupt_handler(sIntHandler[nextFreeHandlerIndex].ui5IntterruptNo, pHandler, pHandlerCtxt );
#else
        #error "XTOS or XOS must be defined."
#endif
    }
// end of critical section
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
