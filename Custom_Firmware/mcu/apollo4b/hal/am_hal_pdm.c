//*****************************************************************************
//
//! @file am_hal_pdm.c
//!
//! @brief HAL implementation for the Pulse Density Modulation module.
//!
//! @addtogroup pdm_4b PDM - Pulse Density Modulation
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
//! @name PDM magic number for handle verification
//! @{
//
//*****************************************************************************
#define AM_HAL_MAGIC_PDM                0xF956E2

#define AM_HAL_PDM_HANDLE_VALID(h)                                            \
    ((h) &&                                                                   \
     ((am_hal_handle_prefix_t *)(h))->s.bInit &&                              \
     (((am_hal_handle_prefix_t *)(h))->s.magic == AM_HAL_MAGIC_PDM))
//
//! @}
//

//*****************************************************************************
//
//! Convenience macro for passing errors
//
//*****************************************************************************
#define RETURN_ON_ERROR(x)                                                    \
    if ((x) != AM_HAL_STATUS_SUCCESS)                                         \
    {                                                                         \
        return (x);                                                           \
    };

//*****************************************************************************
//
//! Abbreviation for validating handles and returning errors
//
//*****************************************************************************
#ifndef AM_HAL_DISABLE_API_VALIDATION

#define AM_HAL_PDM_HANDLE_CHECK(h)                                            \
    if (!AM_HAL_PDM_HANDLE_VALID(h))                                          \
    {                                                                         \
        return AM_HAL_STATUS_INVALID_HANDLE;                                  \
    }

#else

#define AM_HAL_PDM_HANDLE_CHECK(h)

#endif // AM_HAL_DISABLE_API_VALIDATION

//*****************************************************************************
//
//! @name Helper macros for delays.
//! @{
//
//*****************************************************************************
#define delay_ms(ms)                                                          \
    if (1)                                                                    \
    {                                                                         \
        am_hal_clkgen_status_t sClkGenStatus;                                 \
        am_hal_clkgen_status_get(&sClkGenStatus);                             \
        am_hal_flash_delay((ms) * (sClkGenStatus.ui32SysclkFreq / 3000));     \
    }

#define delay_us(us)                                                          \
    if (1)                                                                    \
    {                                                                         \
        am_hal_delay_us(us);                                                  \
    }
//
//! @}
//

//*****************************************************************************
//
//! Structure for handling PDM register state information for power up/down
//
//*****************************************************************************
am_hal_pdm_state_t g_PDMhandles[AM_REG_PDM_NUM_MODULES];

//*****************************************************************************
//
// am_hal_pdm_initialize
//
//*****************************************************************************
uint32_t
am_hal_pdm_initialize(uint32_t ui32Module, void **ppHandle)
{
    //
    // Check that the request module is in range.
    //
    if ( ui32Module >= AM_REG_PDM_NUM_MODULES )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    //
    // Check for valid arguements.
    //
    if (!ppHandle)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Check if the handle is unallocated.
    //
    if (g_PDMhandles[ui32Module].prefix.s.bInit)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    //
    // Initialize the handle.
    //
    g_PDMhandles[ui32Module].prefix.s.bInit = true;
    g_PDMhandles[ui32Module].prefix.s.magic = AM_HAL_MAGIC_PDM;
    g_PDMhandles[ui32Module].ui32Module = ui32Module;
    g_PDMhandles[ui32Module].sRegState.bValid = false;

    //
    // Return the handle.
    //
    *ppHandle = (void *)&g_PDMhandles[ui32Module];

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// De-Initialization function.
//
//*****************************************************************************
uint32_t
am_hal_pdm_deinitialize(void *pHandle)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *)pHandle;

    //
    // Reset the handle.
    //
    pState->prefix.s.bInit = false;
    pState->prefix.s.magic = 0;
    pState->ui32Module = 0;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Power control function.
//
//*****************************************************************************
uint32_t
am_hal_pdm_power_control(void *pHandle,
                         am_hal_sysctrl_power_state_e ePowerState,
                         bool bRetainState)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    am_hal_pwrctrl_periph_e ePDMPowerModule = ((am_hal_pwrctrl_periph_e)
                                               (AM_HAL_PWRCTRL_PERIPH_PDM0 +
                                                ui32Module));

    //
    // Decode the requested power state and update PDM operation accordingly.
    //
    switch (ePowerState)
    {
        //
        // Turn on the PDM.
        //
        case AM_HAL_SYSCTRL_WAKE:
            //
            // Make sure we don't try to restore an invalid state.
            //
            if (bRetainState && !pState->sRegState.bValid)
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }

            //
            // Enable power control.
            //
            am_hal_pwrctrl_periph_enable(ePDMPowerModule);

            if (bRetainState)
            {
                //
                // Restore PDM registers
                //
                AM_CRITICAL_BEGIN;

                pState->sRegState.bValid = false;

                AM_CRITICAL_END;
            }
            break;

        //
        // Turn off the PDM.
        //
        case AM_HAL_SYSCTRL_NORMALSLEEP:
        case AM_HAL_SYSCTRL_DEEPSLEEP:
            if (bRetainState)
            {
                AM_CRITICAL_BEGIN;

                pState->sRegState.bValid = true;

                AM_CRITICAL_END;
            }

            //
            // Disable power control.
            //
            am_hal_pwrctrl_periph_disable(ePDMPowerModule);
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Configure the PDM.
//
//*****************************************************************************
uint32_t
am_hal_pdm_configure(void *pHandle, am_hal_pdm_config_t *psConfig)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    //
    // Apply the config structure settings to the CORECFG0 register.
    //
    PDMn(ui32Module)->CORECFG0_b.LRSWAP = psConfig->bLRSwap;
    PDMn(ui32Module)->CORECFG0_b.SOFTMUTE = psConfig->bSoftMute;
    // Set number of PDMA_CKO cycles during gain setting changes or soft mute
    PDMn(ui32Module)->CORECFG0_b.SCYCLES = psConfig->ui32GainChangeDelay;

    PDMn(ui32Module)->CORECFG0_b.HPGAIN = psConfig->ui32HighPassCutoff;
    PDMn(ui32Module)->CORECFG0_b.ADCHPD = psConfig->bHighPassEnable;
    // PDMA_CKO frequency divisor. Fpdma_cko = Fmclk_l/(MCLKDIV+1)
    PDMn(ui32Module)->CORECFG0_b.MCLKDIV = psConfig->ePDMAClkOutDivder;
    PDMn(ui32Module)->CORECFG0_b.SINCRATE = psConfig->ui32DecimationRate;

    PDMn(ui32Module)->CORECFG0_b.PGAL = psConfig->eLeftGain;
    PDMn(ui32Module)->CORECFG0_b.PGAR = psConfig->eRightGain;

    //
    // Program the "CORECFG1_b" registers.
    //
    PDMn(ui32Module)->CORECFG1_b.PCMCHSET = psConfig->ePCMChannels;
    // Divide down ratio for generating internal master MCLKQ.
    PDMn(ui32Module)->CORECFG1_b.DIVMCLKQ = psConfig->eClkDivider;
    // PDMA_CKO clock phase delay in terms of PDMCLK period to internal sampler
    PDMn(ui32Module)->CORECFG1_b.CKODLY = psConfig->bPDMSampleDelay;
    // Fine grain step size for smooth PGA or Softmute attenuation transition
    PDMn(ui32Module)->CORECFG1_b.SELSTEP = psConfig->eStepSize;

    //
    // Set the PDM Control register.
    //
    PDMn(ui32Module)->CTRL_b.CLKEN = 0;
    PDMn(ui32Module)->CTRL_b.CLKSEL = psConfig->ePDMClkSpeed;

    PDMn(ui32Module)->CTRL_b.RSTB = 0;
    PDMn(ui32Module)->CTRL_b.PCMPACK = 0;    // not supported in rev B

    delay_us(100);

    PDMn(ui32Module)->CTRL_b.RSTB = 1;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Configure the PDM Ctrl Register
//
//*****************************************************************************
uint32_t
am_hal_pdm_config_ctrlReg(void *pHandle, am_hal_pdm_config_t *psConfig)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->CTRL_b.EN = 0;
    PDMn(ui32Module)->CTRL_b.PCMPACK = 0;   // not supported in rev B
    PDMn(ui32Module)->CTRL_b.RSTB = 0;
    PDMn(ui32Module)->CTRL_b.CLKSEL = psConfig->ePDMClkSpeed;
    PDMn(ui32Module)->CTRL_b.CLKEN = 0;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Enable the PDM.
//
//*****************************************************************************
uint32_t
am_hal_pdm_enable(void *pHandle)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->CTRL_b.RSTB = 0;
    delay_us(5000);
    PDMn(ui32Module)->CTRL_b.RSTB = 1;

    PDMn(ui32Module)->CTRL_b.CLKEN = 1;

    PDMn(ui32Module)->CTRL_b.EN    = 1;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Enable the PDM Control
//
//*****************************************************************************
uint32_t
am_hal_pdm_ctrl_enable(void *pHandle)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->CTRL_b.EN    = 1;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Enable the PDM CLK
//
//*****************************************************************************
uint32_t
am_hal_pdm_clk_enable(void *pHandle)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->CTRL_b.CLKEN = 1;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Reset the PDM.
//
//*****************************************************************************
uint32_t
am_hal_pdm_reset(void *pHandle)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->CTRL_b.RSTB = 1;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Disable the PDM.
//
//*****************************************************************************
uint32_t
am_hal_pdm_disable(void *pHandle)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    if (am_hal_pdm_dma_disable(pHandle) != AM_HAL_STATUS_SUCCESS)
    {
        PDMn(ui32Module)->DMACFG_b.DMAEN = 0;
    }

    PDMn(ui32Module)->CTRL_b.EN    = 0;
    PDMn(ui32Module)->CTRL_b.CLKEN = 0;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Given the total number of bytes in a DMA transaction, find a reasonable
// threshold setting.
//
//*****************************************************************************
static uint32_t
find_dma_threshold(uint32_t ui32TotalCount)
{
    //
    // Start with a threshold value of 24, and search downward for values that
    // fit our criteria.
    //
    uint32_t ui32Threshold;
    uint32_t ui32Minimum = AM_HAL_PDM_DMA_THRESHOLD_MIN;

    for ( ui32Threshold = 24; ui32Threshold >= ui32Minimum; ui32Threshold -= 4 )
    {
        //
        // With our loop parameters, we've already guaranteed that the
        // threshold will be no higher than 24, and that it will be divisible
        // by 4. The only remaining requirement is that ui32TotalCount must
        // also be divisible by the threshold.
        //
        if ((ui32TotalCount % ui32Threshold) == 0)
        {
            break;
        }
    }

    //
    // If we found an appropriate value, we'll return it here. Otherwise, we
    // will return zero.
    //
    if (ui32Threshold < ui32Minimum)
    {
        ui32Threshold = 0;
    }

    return ui32Threshold;
}

//*****************************************************************************
//
// Starts a DMA transaction from the PDM directly to SRAM
//
//*****************************************************************************
uint32_t
am_hal_pdm_dma_start(void *pHandle, am_hal_pdm_transfer_t *pTransferCfg)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

#if 0 //ndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check for DMA to/from DTCM.
    //
    if ( (pTransferCfg->ui32TargetAddr >= AM_HAL_SYSTEM_MCU_DTCM_ADDR) &&
      (pTransferCfg->ui32TargetAddr <= AM_HAL_SYSTEM_MCU_DTCM_ADDR + AM_HAL_SYSTEM_MCU_DTCM_SIZE) )
    {
      return AM_HAL_STATUS_OUT_OF_RANGE;
    }
#endif

    // Configure Pingpong Buffer
    pState->ui32BufferPtr  = \
    pState->ui32BufferPing = pTransferCfg->ui32TargetAddr;
    pState->ui32BufferPong = pTransferCfg->ui32TargetAddrReverse;

    //
    // Find an appropriate threshold size for this transfer.
    //
    uint32_t ui32Threshold = find_dma_threshold(pTransferCfg->ui32TotalCount);

    //
    // If we didn't find a threshold that will work, throw an error.
    //
    if (ui32Threshold == 0)
    {
        return AM_HAL_PDM_STATUS_BAD_TOTALCOUNT;
    }

    PDMn(ui32Module)->FIFOTHR = ui32Threshold;

    //
    // Configure DMA.
    //
    PDMn(ui32Module)->DMACFG = 0;
    //PDMn(ui32Module)->DMACFG_b.DMAPRI = PDM0_DMACFG_DMAPRI_LOW;
    //PDMn(ui32Module)->DMACFG_b.DMADIR = PDM0_DMACFG_DMADIR_P2M;
    PDMn(ui32Module)->DMATOTCOUNT = pTransferCfg->ui32TotalCount;
    PDMn(ui32Module)->DMATARGADDR = pTransferCfg->ui32TargetAddr;

    //
    // Make sure the trigger is set for threshold.
    //
    PDMn(ui32Module)->DMATRIGEN_b.DTHR = 1;

    //
    // Enable DMA
    //
    PDMn(ui32Module)->DMACFG_b.DMAEN = PDM0_DMACFG_DMAEN_EN;

    return AM_HAL_STATUS_SUCCESS;
}

void am_hal_pdm_dma_reset_count(uint32_t count)
{
  PDMn(0)->DMATOTCOUNT = count;
}

//*****************************************************************************
//
// Flush the PDM FIFO
//
//*****************************************************************************
uint32_t
am_hal_pdm_fifo_flush(void *pHandle)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->FIFOFLUSH = 1;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// set the PDM FIFO Threshold value
//
//*****************************************************************************
uint32_t
am_hal_pdm_fifo_threshold_setup(void *pHandle, uint32_t value)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->FIFOTHR = value;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// PDM interrupt service routine
//
//*****************************************************************************
uint32_t am_hal_pdm_interrupt_service(void *pHandle, uint32_t ui32IntMask, am_hal_pdm_transfer_t *pTransferCfg)
{
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    if (ui32IntMask & AM_HAL_PDM_INT_DCMP)
    {
        PDMn(ui32Module)->DMATARGADDR = pState->ui32BufferPtr = (pState->ui32BufferPtr == pState->ui32BufferPong)? pState->ui32BufferPing: pState->ui32BufferPong;
        PDMn(ui32Module)->DMATOTCOUNT = pTransferCfg->ui32TotalCount;
    }

    if (ui32IntMask & AM_HAL_PDM_INT_OVF)
    {

        am_hal_pdm_fifo_flush(pHandle);
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Interrupt enable.
//
//*****************************************************************************
uint32_t
am_hal_pdm_interrupt_enable(void *pHandle, uint32_t ui32IntMask)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->INTEN |= ui32IntMask;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Interrupt disable.
//
//*****************************************************************************
uint32_t
am_hal_pdm_interrupt_disable(void *pHandle, uint32_t ui32IntMask)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->INTEN &= ~ui32IntMask;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Interrupt clear.
//
//*****************************************************************************
uint32_t
am_hal_pdm_interrupt_clear(void *pHandle, uint32_t ui32IntMask)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->INTCLR = ui32IntMask;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Returns the interrupt status.
//
//*****************************************************************************
uint32_t
am_hal_pdm_interrupt_status_get(void *pHandle, uint32_t *pui32Status, bool bEnabledOnly)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    //
    // If requested, only return the interrupts that are enabled.
    //
    if ( bEnabledOnly )
    {
        *pui32Status = PDMn(ui32Module)->INTSTAT;
        *pui32Status &= PDMn(ui32Module)->INTEN;
    }
    else
    {
        *pui32Status = PDMn(ui32Module)->INTSTAT;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Get the DMA Bufffer.
//
//*****************************************************************************
uint32_t
am_hal_pdm_dma_get_buffer(void *pHandle)
{
    uint32_t ui32BufferPtr;
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;

    // Invalidate DAXI to make sure CPU sees the new data when loaded
    am_hal_daxi_control(AM_HAL_DAXI_CONTROL_INVALIDATE, 0);

    ui32BufferPtr = (pState->ui32BufferPtr == pState->ui32BufferPong)? pState->ui32BufferPing: pState->ui32BufferPong;

    return ui32BufferPtr;
}

//*****************************************************************************
//
// Read the FIFO.
//
//*****************************************************************************
uint32_t
am_hal_pdm_fifo_data_read(void *pHandle)
{
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    return PDMn(ui32Module)->FIFOREAD;
}

//*****************************************************************************
//
// Read the FIFOs.
//
//*****************************************************************************
uint32_t am_hal_pdm_fifo_data_reads(void *pHandle, uint8_t* buffer, uint32_t size)
{
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    uint32_t fiforead;

    uint8_t buf_size = 0;
    for ( uint32_t i = 0; i < size; i++ )
    {
        //if ((i%2) != 0)
        //{
        //    fiforead = PDMn(ui32Module)->FIFOREAD;
        //}
        //else
        {
            fiforead = PDMn(ui32Module)->FIFOREAD; //left/right channel
            buffer[buf_size++] = fiforead & 0xFF;
            buffer[buf_size++] = (fiforead & 0xFF00)>>8;
            buffer[buf_size++] = (fiforead & 0x00FF0000)>>16;
        }
    }

    return 0;
}

//*****************************************************************************
//
// am_hal_pdm_fifo_count_get
//
//*****************************************************************************
uint32_t am_hal_pdm_fifo_count_get(void *pHandle)
{
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    return PDMn(ui32Module)->FIFOCNT;
}

//*****************************************************************************
//
// am_hal_pdm_dma_state
//
//*****************************************************************************
uint32_t am_hal_pdm_dma_state(void *pHandle)
{
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    return PDMn(ui32Module)->DMASTAT;
}

//*****************************************************************************
//
// disable PDM DMA
//
//*****************************************************************************
uint32_t
am_hal_pdm_dma_disable(void *pHandle)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    //
    // disable this pdm module while disabling DMA
    //
    uint32_t ui32CtrlSaved = PDMn(ui32Module)->CTRL;

    PDMn(ui32Module)->CTRL = ui32CtrlSaved & ~PDM0_CTRL_CLKEN_Msk;
    PDMn(ui32Module)->DMACFG_b.DMAEN = 0;

    //
    // disable dma interrupts
    //
    PDMn(ui32Module)->INTEN &= ~(AM_HAL_PDM_INT_DERR | AM_HAL_PDM_INT_DCMP);

    //
    // clear interrupts
    //
    PDMn(ui32Module)->INTCLR = (AM_HAL_PDM_INT_DERR | AM_HAL_PDM_INT_DCMP);


    PDMn(ui32Module)->DMATOTCOUNT = 0;

    //
    // restore control register
    //
    PDMn(ui32Module)->CTRL = ui32CtrlSaved;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
