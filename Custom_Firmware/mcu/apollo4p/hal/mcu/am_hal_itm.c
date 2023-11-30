//*****************************************************************************
//
//! @file am_hal_itm.c
//!
//! @brief Functions for operating the instrumentation trace macrocell
//!
//! @addtogroup itm4_4p ITM - Instrumentation Trace Macrocell
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
// Global Variables
//
//*****************************************************************************

//*****************************************************************************
//
// Enables the ITM
//
//*****************************************************************************
void
am_hal_itm_enable(void)
{
    //
    // To be able to access ITM registers, set the Trace Enable bit
    // in the Debug Exception and Monitor Control Register (DEMCR).
    //
    CoreDebug->DEMCR |= _VAL2FLD(CoreDebug_DEMCR_TRCENA, 1);
    while ( !(CoreDebug->DEMCR & _VAL2FLD(CoreDebug_DEMCR_TRCENA, 1)) );

    //
    // Write the key to the ITM Lock Access register to unlock the ITM_TCR.
    //
    ITM->LAR = ITM_LAR_KEYVAL;

    //
    // Set the enable bits in the ITM trace enable register, and the ITM
    // control registers to enable trace data output.
    //
    ITM->TPR = 0x0000000F;
    ITM->TER = 0xFFFFFFFF;

    //
    // Write to the ITM control and status register.
    //
    ITM->TCR =
        _VAL2FLD(ITM_TCR_TraceBusID, 0x15)      |
        _VAL2FLD(ITM_TCR_GTSFREQ, 1)            |
        _VAL2FLD(ITM_TCR_TSPrescale, 1)         |
        _VAL2FLD(ITM_TCR_SWOENA, 1)             |
        _VAL2FLD(ITM_TCR_DWTENA, 0)             |
        _VAL2FLD(ITM_TCR_SYNCENA, 0)            |
        _VAL2FLD(ITM_TCR_TSENA, 0)              |
        _VAL2FLD(ITM_TCR_ITMENA, 1);

} // am_hal_itm_enable()

//*****************************************************************************
//
// Disables the ITM
//
//*****************************************************************************
void
am_hal_itm_disable(void)
{

    if ( MCUCTRL->DBGCTRL == 0 )
    {
        //
        // This is a disable without enable, which could be the case with some
        // earlier versions of SBL. To avoid a hang, ITM (particularly TPIU
        // clock) must first be enabled.
        //
        am_hal_itm_enable();
    }

    //
    // Make sure the ITM/TPIU is not busy.
    //
    am_hal_itm_not_busy();

    //
    // Make sure the ITM_TCR is unlocked.
    //
    ITM->LAR = ITM_LAR_KEYVAL;

    //
    // Disable the ITM.
    //
    for (int ix = 0; ix < 100; ix++)
    {
        ITM->TCR &= ~_VAL2FLD(ITM_TCR_ITMENA, 1);
        while ( ITM->TCR  & (_VAL2FLD(ITM_TCR_ITMENA, 1)  |  _VAL2FLD(ITM_TCR_BUSY, 1)) );
    }

    //
    // Reset the TRCENA bit in the DEMCR register, which should disable the ITM
    // for operation.
    //
    CoreDebug->DEMCR &= ~_VAL2FLD(CoreDebug_DEMCR_TRCENA, 1);
    while ( CoreDebug->DEMCR & _VAL2FLD(CoreDebug_DEMCR_TRCENA, 1) );

    //
    // Disable the TPIU clock source in MCU control.
    //
    MCUCTRL->DBGCTRL_b.CM4CLKSEL = MCUCTRL_DBGCTRL_CM4CLKSEL_LOWPWR;
    MCUCTRL->DBGCTRL_b.CM4TPIUENABLE = MCUCTRL_DBGCTRL_CM4TPIUENABLE_DIS;
    MCUCTRL->DBGCTRL_b.DBGTSCLKSEL = MCUCTRL_DBGCTRL_DBGTSCLKSEL_LOWPWR;
    while (MCUCTRL->DBGCTRL);
} // am_hal_itm_disable()

//*****************************************************************************
//
// Checks if itm is busy and provides a delay to flush the fifo
//
//*****************************************************************************
void
am_hal_itm_not_busy(void)
{
    //
    // Make sure the ITM/TPIU is not busy.
    //
    while (ITM->TCR & _VAL2FLD(ITM_TCR_BUSY, 1));

    //
    // wait for 50us for the data to flush out
    //
    am_hal_delay_us(50);
} // am_hal_itm_not_busy()

//*****************************************************************************
//
// Enables tracing on a given set of ITM ports
//
//*****************************************************************************
void
am_hal_itm_trace_port_enable(uint8_t ui8portNum)
{
    ITM->TPR |= (0x00000001 << (ui8portNum>>3));
} // am_hal_itm_trace_port_enable()

//*****************************************************************************
//
// Disable tracing on the given ITM stimulus port.
//
//*****************************************************************************
void
am_hal_itm_trace_port_disable(uint8_t ui8portNum)
{
    ITM->TPR &= ~(0x00000001 << (ui8portNum >> 3));
} // am_hal_itm_trace_port_disable()

//*****************************************************************************
//
// Poll the given ITM stimulus register until not busy.
//
//*****************************************************************************
bool
am_hal_itm_stimulus_not_busy(uint32_t ui32StimReg)
{
    uint32_t ui32StimAddr = (uint32_t)&ITM->PORT[0] + (4 * ui32StimReg);

    //
    // Busy waiting until it is available, non-zero means ready.
    //
    while ( !AM_REGVAL(ui32StimAddr) );

    return true;
} // am_hal_itm_stimulus_not_busy()

//*****************************************************************************
//
// Writes a 32-bit value to the given ITM stimulus register.
//
//*****************************************************************************
void
am_hal_itm_stimulus_reg_word_write(uint32_t ui32StimReg, uint32_t ui32Value)
{
    uint32_t ui32StimAddr = (uint32_t)&ITM->PORT[0] + (4 * ui32StimReg);

    //
    // Busy waiting until it is available, non-zero means ready
    //
    while (!AM_REGVAL(ui32StimAddr));

    //
    // Write the register.
    //
    AM_REGVAL(ui32StimAddr) = ui32Value;
} // am_hal_itm_stimulus_reg_word_write()

//*****************************************************************************
//
// Writes a short to the given ITM stimulus register.
//
//*****************************************************************************
void
am_hal_itm_stimulus_reg_short_write(uint32_t ui32StimReg, uint16_t ui16Value)
{
    uint32_t ui32StimAddr = (uint32_t)&ITM->PORT[0] + (4 * ui32StimReg);

    //
    // Busy waiting until it is available non-zero means ready
    //
    while ( !AM_REGVAL(ui32StimAddr) );

    //
    // Write the register.
    //
    *((volatile uint16_t *) ui32StimAddr) = ui16Value;
} // am_hal_itm_stimulus_reg_short_write()

//*****************************************************************************
//
// Writes a byte to the given ITM stimulus register.
//
//*****************************************************************************
void
am_hal_itm_stimulus_reg_byte_write(uint32_t ui32StimReg, uint8_t ui8Value)
{
    uint32_t ui32StimAddr = (uint32_t)&ITM->PORT[0] + (4 * ui32StimReg);

    //
    // Busy waiting until it is available (non-zero means ready)
    //
    while (!AM_REGVAL(ui32StimAddr));

    //
    // Write the register.
    //
    *((volatile uint8_t *) ui32StimAddr) = ui8Value;
} // am_hal_itm_stimulus_reg_byte_write()

//*****************************************************************************
//
// Sends a Sync Packet.
//
//*****************************************************************************
void
am_hal_itm_sync_send(void)
{
    //
    // Write the register.
    //
    am_hal_itm_stimulus_reg_word_write(AM_HAL_ITM_SYNC_REG,
                                       AM_HAL_ITM_SYNC_VAL);
} // am_hal_itm_sync_send()

//*****************************************************************************
//
// Poll the print stimulus registers until not busy.
//
//*****************************************************************************
bool
am_hal_itm_print_not_busy(void)
{
    //
    // Poll stimulus register allocated for printing.
    //
    am_hal_itm_stimulus_not_busy(0);

    return true;
} // am_hal_itm_print_not_busy()

//*****************************************************************************
//
// Prints a char string out of the ITM.
//
//*****************************************************************************
void
am_hal_itm_print(char *pcString)
{
    uint32_t ui32Length = 0;

    //
    // Determine the length of the string.
    //
    while (*(pcString + ui32Length))
    {
        ui32Length++;
    }

    //
    // If there is no longer a word left, empty out the remaining characters.
    //
    while (ui32Length)
    {
        //
        // Print string out the ITM.
        //
        am_hal_itm_stimulus_reg_byte_write(0, (uint8_t)*pcString++);

        //
        // Subtract from length.
        //
        ui32Length--;
    }
} // am_hal_itm_print()

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
