//*****************************************************************************
//
//! @file am_hal_i2s.c
//!
//! @brief HAL implementation for the Inter-IC Sound module.
//!
//! @addtogroup i2s4_4b I2S - Inter-IC Sound
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
//! @name I2S magic number macros for handle verification.
//! @{
//
//*****************************************************************************
#define AM_HAL_MAGIC_I2S            0x125125
#define AM_HAL_I2S_HANDLE_VALID(h)    ((h) && ((am_hal_handle_prefix_t *)(h))->s.bInit && (((am_hal_handle_prefix_t *)(h))->s.magic == AM_HAL_MAGIC_I2S))

#define AM_HAL_I2S_CHK_HANDLE(h)                                              \
    if (!AM_HAL_I2S_HANDLE_VALID(h))                                          \
    {                                                                         \
        return AM_HAL_STATUS_INVALID_HANDLE;                                  \
    }
//! @}

//*****************************************************************************
//
//! Structure for I2S registers.
//
//*****************************************************************************
typedef struct
{
    bool        bValid;

    uint32_t    regI2SCTL;
    uint32_t    regI2SDATACFG;
    uint32_t    regI2SIOCFG;
    uint32_t    regAMQCFG;
    uint32_t    regI2SCLKCFG;
    uint32_t    regIPBIRPT;
    uint32_t    regRXUPPERLIMIT;
    uint32_t    regTXLOWERLIMIT;
    uint32_t    regINTEN;
    uint32_t    regI2SDMACFG;
} am_hal_i2s_register_state_t;

//*****************************************************************************
//
//! Structure for handling I2S HAL state information.
//
//*****************************************************************************
typedef struct
{
    am_hal_handle_prefix_t           prefix;

    uint32_t                         ui32Module;

    am_hal_i2s_register_state_t      sRegState;
    //
    // DMA transaction Tranfer Control Buffer.
    //
    uint32_t                ui32RxBufferPing;
    uint32_t                ui32RxBufferPong;
    uint32_t                ui32TxBufferPing;
    uint32_t                ui32TxBufferPong;
    //
    // Current DMA buffer ptr.
    //
    uint32_t                ui32RxBufferPtr;
    uint32_t                ui32TxBufferPtr;
}am_hal_i2s_state_t;

//
//! Available frame size.
//
static const uint32_t ui32I2sWordLength[AM_HAL_I2S_FRAME_WDLEN_MAX] =
{
    AM_HAL_I2S_WORD_8BITS,
    AM_HAL_I2S_WORD_RESERVED,
    AM_HAL_I2S_WORD_16BITS,
    AM_HAL_I2S_WORD_RESERVED,
    AM_HAL_I2S_WORD_24BITS,
    AM_HAL_I2S_WORD_32BITS
};

//*****************************************************************************
//
// Globals
//
//*****************************************************************************
am_hal_i2s_state_t          g_I2Shandles[AM_REG_I2S_NUM_MODULES];

//*****************************************************************************
//
// I2S initialization function
//
//*****************************************************************************
uint32_t
am_hal_i2s_initialize(uint32_t ui32Module, void **ppHandle)
{
    //
    // Compile time check to ensure ENTRY_SIZE macros are defined correctly
    // incorrect definition will cause divide by 0 error at build time
    //
#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Validate the module number
    //
    if ( ui32Module >= AM_REG_I2S_NUM_MODULES )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    if (ppHandle == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if (g_I2Shandles[ui32Module].prefix.s.bInit)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    g_I2Shandles[ui32Module].prefix.s.bInit = true;
    g_I2Shandles[ui32Module].prefix.s.bEnable = false;
    g_I2Shandles[ui32Module].prefix.s.magic = AM_HAL_MAGIC_I2S;
    //
    // Initialize the handle.
    //
    g_I2Shandles[ui32Module].ui32Module = ui32Module;
    //
    // Return the handle.
    //
    *ppHandle = (void *)&g_I2Shandles[ui32Module];
    //
    // Return the status
    //
    return AM_HAL_STATUS_SUCCESS;
} // am_hal_i2s_initialize()

//*****************************************************************************
//
// am_hal_i2s_deinitialize
//
//*****************************************************************************
uint32_t
am_hal_i2s_deinitialize(void *pHandle)
{
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *)pHandle;
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);
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
//! @brief I2S control function
//!
//! @param pHandle       - handle for the I2S.
//! @param eReq         - device specific special request code.
//! @param pArgs        - pointer to the request specific arguments.
//!
//! This function allows advanced settings
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t am_hal_i2s_control(void *pHandle, am_hal_i2s_request_e eReq, void *pArgs)
{
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t*)pHandle;
    uint32_t ui32Module = pState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Validate the parameters
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);
#endif // AM_HAL_DISABLE_API_VALIDATION

    switch (eReq)
    {
        case AM_HAL_I2S_REQ_INTSET:
            I2Sn(ui32Module)->INTSET = *((uint32_t *)pArgs);
            break;
        case AM_HAL_I2S_REQ_INTCLR:
            I2Sn(ui32Module)->INTCLR = *((uint32_t *)pArgs);
            break;
        case AM_HAL_I2S_REQ_TXFIFOCNT:
            *((uint32_t*)pArgs) = I2Sn(ui32Module)->TXFIFOSTATUS_b.TXFIFOCNT;
            break;
        case AM_HAL_I2S_REQ_READ_RXUPPERLIMIT:
            *((uint32_t*)pArgs) = I2Sn(ui32Module)->RXUPPERLIMIT;
            break;
        case AM_HAL_I2S_REQ_READ_TXLOWERLIMIT:
            *((uint32_t*)pArgs) = I2Sn(ui32Module)->TXLOWERLIMIT;
            break;
        case AM_HAL_I2S_REQ_WRITE_RXUPPERLIMIT:
            I2Sn(ui32Module)->RXUPPERLIMIT = *((uint32_t*)pArgs);
            break;
        case AM_HAL_I2S_REQ_WRITE_TXLOWERLIMIT:
            I2Sn(ui32Module)->TXLOWERLIMIT = *((uint32_t*)pArgs);
            break;
        case AM_HAL_I2S_REQ_MAX:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I2S configuration function.
//
//*****************************************************************************
uint32_t
am_hal_i2s_configure(void *pHandle, am_hal_i2s_config_t *psConfig)
{
    uint32_t status = AM_HAL_STATUS_SUCCESS;

    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t*)pHandle;
    uint32_t ui32Module = pState->ui32Module;
    uint32_t ui32FramePeriod = 0;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Validate the parameters
    //
    if ((pHandle == NULL) || (psConfig == NULL) || (pState->ui32Module >= AM_REG_I2S_NUM_MODULES))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    AM_HAL_I2S_CHK_HANDLE(pHandle);
    //
    // Configure not allowed in Enabled state
    //
    if (pState->prefix.s.bEnable)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    am_hal_i2s_data_format_t* pI2SData = psConfig->eData;
    am_hal_i2s_io_signal_t* pI2SIOCfg  = psConfig->eIO;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ((pI2SData->ui32ChannelNumbersPhase1 > 8) || (pI2SData->ui32ChannelNumbersPhase2 > 8) || \
        (pI2SData->ui32ChannelNumbersPhase1 == 0) || (pI2SData->ui32ChannelNumbersPhase1 + pI2SData->ui32ChannelNumbersPhase2 > 8) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    if ((pI2SData->eSampleLenPhase1 != AM_HAL_I2S_SAMPLE_LENGTH_8BITS) && (pI2SData->eSampleLenPhase1 != AM_HAL_I2S_SAMPLE_LENGTH_16BITS) && \
        (pI2SData->eSampleLenPhase1 != AM_HAL_I2S_SAMPLE_LENGTH_24BITS) && (pI2SData->eSampleLenPhase1 != AM_HAL_I2S_SAMPLE_LENGTH_32BITS) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    if ((pI2SData->eSampleLenPhase2 != AM_HAL_I2S_SAMPLE_LENGTH_8BITS) && (pI2SData->eSampleLenPhase2 != AM_HAL_I2S_SAMPLE_LENGTH_16BITS) && \
        (pI2SData->eSampleLenPhase2 != AM_HAL_I2S_SAMPLE_LENGTH_24BITS) && (pI2SData->eSampleLenPhase2 != AM_HAL_I2S_SAMPLE_LENGTH_32BITS) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    if ((pI2SData->eChannelLenPhase1 != AM_HAL_I2S_FRAME_WDLEN_8BITS) && (pI2SData->eChannelLenPhase1 != AM_HAL_I2S_FRAME_WDLEN_16BITS) && \
        (pI2SData->eChannelLenPhase1 != AM_HAL_I2S_FRAME_WDLEN_24BITS) && (pI2SData->eChannelLenPhase1 != AM_HAL_I2S_FRAME_WDLEN_32BITS) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    if ((pI2SData->eChannelLenPhase2 != AM_HAL_I2S_FRAME_WDLEN_8BITS) && (pI2SData->eChannelLenPhase2 != AM_HAL_I2S_FRAME_WDLEN_16BITS) && \
        (pI2SData->eChannelLenPhase2 != AM_HAL_I2S_FRAME_WDLEN_24BITS) && (pI2SData->eChannelLenPhase2 != AM_HAL_I2S_FRAME_WDLEN_32BITS) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION
    //
    // 1.Reset the serial receiver or transmitter by asserting bits RXRST and/or TXRST in the I2SCTL register
    //
    I2Sn(ui32Module)->I2SCTL_b.RXRST = 1;
    I2Sn(ui32Module)->I2SCTL_b.TXRST = 1;
    am_hal_delay_us(200);
    //
    //2.I2S IPB clocks and IO signals
    //
    uint32_t ui32OEN = (psConfig->eXfer == AM_HAL_I2S_XFER_TX || psConfig->eXfer == AM_HAL_I2S_XFER_RXTX) ? 1 : 0;

    if (pI2SData->ePhase == AM_HAL_I2S_DATA_PHASE_SINGLE)
    {
        ui32FramePeriod = (pI2SData->ui32ChannelNumbersPhase1 * ui32I2sWordLength[pI2SData->eChannelLenPhase1]);
    }
    else
    {
        ui32FramePeriod = ((pI2SData->ui32ChannelNumbersPhase1 * ui32I2sWordLength[pI2SData->eChannelLenPhase1]) +  \
                           (pI2SData->ui32ChannelNumbersPhase2 * ui32I2sWordLength[pI2SData->eChannelLenPhase2]) );
    }

    I2Sn(ui32Module)->I2SIOCFG =
         _VAL2FLD(I2S0_I2SIOCFG_OEN, ui32OEN)                | /*Output enable for SDATA output */
         _VAL2FLD(I2S0_I2SIOCFG_FPER, ui32FramePeriod - 1)   | /*specifying a frame period of 2x WDLEN1 cycles*/
         _VAL2FLD(I2S0_I2SIOCFG_FSP, pI2SIOCfg->eFyncCpol)   |
         _VAL2FLD(I2S0_I2SIOCFG_PRTX, pI2SIOCfg->eTxCpol)    |
         _VAL2FLD(I2S0_I2SIOCFG_MSL, psConfig->eMode)        |
         _VAL2FLD(I2S0_I2SIOCFG_PRx, pI2SIOCfg->eRxCpol)     |
         _VAL2FLD(I2S0_I2SIOCFG_FWID, (ui32I2sWordLength[pI2SData->eChannelLenPhase1] - 1) );
    //
    // 3.Set the audio data format.
    //
    I2Sn(ui32Module)->I2SDATACFG =
         _VAL2FLD(I2S0_I2SDATACFG_SSZ1,   pI2SData->eSampleLenPhase1)     |
         _VAL2FLD(I2S0_I2SDATACFG_JUST,   pI2SData->eDataJust)            |
         _VAL2FLD(I2S0_I2SDATACFG_WDLEN1, pI2SData->eChannelLenPhase1)    |
         _VAL2FLD(I2S0_I2SDATACFG_FRLEN1, (pI2SData->ui32ChannelNumbersPhase1-1))    |
         _VAL2FLD(I2S0_I2SDATACFG_SSZ2,   pI2SData->eSampleLenPhase2)     |
         _VAL2FLD(I2S0_I2SDATACFG_DATADLY, pI2SData->eDataDelay)          |
         _VAL2FLD(I2S0_I2SDATACFG_WDLEN2, pI2SData->eChannelLenPhase2)    |
         _VAL2FLD(I2S0_I2SDATACFG_FRLEN2, (pI2SData->ui32ChannelNumbersPhase2-1))    |
         _VAL2FLD(I2S0_I2SDATACFG_PH, pI2SData->ePhase);
    //
    // 4.Control the enablement of the ASRC module and the source of the MCLK used in the IPB core.
    //  [1..1] ASRC sub module enable. 0: Enabled. 1: Disabled/Bypassed
    if ( psConfig->eMode == AM_HAL_I2S_IO_MODE_MASTER )
    {
        I2Sn(ui32Module)->AMQCFG = 0x2;
    }
    else
    {
        if ( psConfig->eASRC )
        {
            //
            //! !! RevB1 I2S Instance 0 DONOT support ASRC.
            //
            if ( (ui32Module == 0) && APOLLO4_GE_B1 )
            {
                status = AM_HAL_STATUS_INVALID_ARG;
            }
            else
            {
                I2Sn(ui32Module)->AMQCFG = 0x0;
            }
        }
        else
        {
          I2Sn(ui32Module)->AMQCFG = 0x2;
        }
    }
    //
    // 5.MCLK/sclk clock setting.
    //
    uint32_t clk_sel = psConfig->eClock;
    I2Sn(ui32Module)->CLKCFG_b.DIV3 = psConfig->eDiv3;
    I2Sn(ui32Module)->CLKCFG_b.FSEL = clk_sel;
    I2Sn(ui32Module)->CLKCFG_b.MCLKEN = 0x1;
    am_hal_delay_us(50);
    //
    // 6.RXTX DMA limit: FIFO 50 percent full
    //
    I2Sn(ui32Module)->RXUPPERLIMIT = 0x20;
    I2Sn(ui32Module)->TXLOWERLIMIT = 0x20;
    //
    // Return the status.
    //
    return status;
} // am_hal_i2s_configure()

//*****************************************************************************
//
// I2S DMA nonblocking transfer function
//
//*****************************************************************************
uint32_t
am_hal_i2s_dma_transfer_start(void *pHandle,  am_hal_i2s_config_t *pConfig)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);
    //
    // Poll Transmit FIFO Status register to
    // prevent the Transmit FIFO from becoming empty.
    //
    uint32_t ui32Fifocnt  = I2Sn(ui32Module)->TXFIFOSTATUS_b.TXFIFOCNT;
    uint32_t ui32Fifofull = I2Sn(ui32Module)->TXFIFOSTATUS_b.TXFIFOFULL;
    if ( ui32Fifocnt || ui32Fifofull )
    {
        return AM_HAL_STATUS_FAIL;
    }
    //
    // Enable the transmission of serial audio.
    // TXRST & RXRST must be cleared in advance!!!
    //
    if ( pConfig->eXfer == AM_HAL_I2S_XFER_RX )
    {
        I2Sn(ui32Module)->DMACFG_b.RXDMAEN = 0x1;
        I2Sn(ui32Module)->I2SCTL_b.RXRST   = 0x0;
        I2Sn(ui32Module)->I2SCTL_b.RXEN    = 0x1;
    }
    else if ( pConfig->eXfer == AM_HAL_I2S_XFER_TX )
    {
        //
        // Need to make sure all buffer writes are flushed
        //
        am_hal_sysctrl_bus_write_flush();

        I2Sn(ui32Module)->DMACFG_b.TXDMAEN = 0x1;
        I2Sn(ui32Module)->I2SCTL_b.TXRST   = 0x0;
        I2Sn(ui32Module)->I2SCTL_b.TXEN    = 0x1;
    }
    else if ( pConfig->eXfer == AM_HAL_I2S_XFER_RXTX )
    {
        //
        // Need to make sure all buffer writes are flushed
        //
        am_hal_sysctrl_bus_write_flush();

        I2Sn(ui32Module)->DMACFG_b.RXDMAEN = 0x1;
        I2Sn(ui32Module)->DMACFG_b.TXDMAEN = 0x1;
        I2Sn(ui32Module)->I2SCTL           = I2S0_I2SCTL_TXEN_Msk |
                                             I2S0_I2SCTL_RXEN_Msk;
    }

    return ui32Status;
}

//*****************************************************************************
//
// am_hal_i2s_dma_transfer_continue
//
//*****************************************************************************
uint32_t
am_hal_i2s_dma_transfer_continue(void *pHandle, am_hal_i2s_config_t* psConfig, am_hal_i2s_transfer_t *pTransferCfg)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;
    //
    // Once completed, software must first write the DMACFG register to 0.
    //
    I2Sn(ui32Module)->DMACFG = 0x0;
    //
    // Clear dma status.
    //
    I2Sn(ui32Module)->RXDMASTAT = 0x0;
    I2Sn(ui32Module)->TXDMASTAT = 0x0;
    //
    // High Priority (service immediately)
    //
    I2Sn(ui32Module)->DMACFG_b.RXDMAPRI = 0x1;
    I2Sn(ui32Module)->DMACFG_b.TXDMAPRI = 0x1;

    switch(psConfig->eXfer)
    {
        case AM_HAL_I2S_XFER_RX:
            I2Sn(ui32Module)->RXDMATOTCNT = pTransferCfg->ui32RxTotalCount;
            I2Sn(ui32Module)->RXDMAADDR   = pState->ui32RxBufferPtr
                                          = pTransferCfg->ui32RxTargetAddr;
            I2Sn(ui32Module)->DMACFG_b.RXDMAEN = 0x1;
            break;

        case AM_HAL_I2S_XFER_TX:
            //
            // Need to make sure all buffer writes are flushed
            //
            am_hal_sysctrl_bus_write_flush();

            I2Sn(ui32Module)->TXDMATOTCNT = pTransferCfg->ui32TxTotalCount;
            I2Sn(ui32Module)->TXDMAADDR   = pState->ui32TxBufferPtr
                                          = pTransferCfg->ui32TxTargetAddr;
            I2Sn(ui32Module)->DMACFG_b.TXDMAEN = 0x1;
            break;

        case AM_HAL_I2S_XFER_RXTX:
            //
            // Need to make sure all buffer writes are flushed
            //
            am_hal_sysctrl_bus_write_flush();

            I2Sn(ui32Module)->RXDMATOTCNT = pTransferCfg->ui32RxTotalCount;
            I2Sn(ui32Module)->RXDMAADDR   = pState->ui32RxBufferPtr
                                          = pTransferCfg->ui32RxTargetAddr;
            I2Sn(ui32Module)->TXDMATOTCNT = pTransferCfg->ui32TxTotalCount;
            I2Sn(ui32Module)->TXDMAADDR   = pState->ui32TxBufferPtr
                                          = pTransferCfg->ui32TxTargetAddr;
            I2Sn(ui32Module)->DMACFG_b.RXDMAEN = 0x1;
            I2Sn(ui32Module)->DMACFG_b.TXDMAEN = 0x1;
            break;
    }

    return ui32Status;
}

//*****************************************************************************
//
// Power control function.
//
//*****************************************************************************
uint32_t
am_hal_i2s_power_control(void *pHandle,
                         am_hal_sysctrl_power_state_e ePowerState,
                         bool bRetainState)
{
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    am_hal_pwrctrl_periph_e eI2SPowerModule = ((am_hal_pwrctrl_periph_e)(AM_HAL_PWRCTRL_PERIPH_I2S0 + ui32Module));
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);
    //
    // Decode the requested power state and update I2S operation accordingly.
    //
    switch (ePowerState)
    {
        //
        // Turn on the I2S.
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
            am_hal_pwrctrl_periph_enable(eI2SPowerModule);

            if (bRetainState)
            {
                //
                // Restore I2S registers
                //
                I2Sn(ui32Module)->I2SCTL       = pState->sRegState.regI2SCTL;
                I2Sn(ui32Module)->I2SDATACFG   = pState->sRegState.regI2SDATACFG;
                I2Sn(ui32Module)->I2SIOCFG     = pState->sRegState.regI2SIOCFG;
                I2Sn(ui32Module)->AMQCFG       = pState->sRegState.regAMQCFG;
                I2Sn(ui32Module)->CLKCFG       = pState->sRegState.regI2SCLKCFG;
                I2Sn(ui32Module)->IPBIRPT      = pState->sRegState.regIPBIRPT;
                I2Sn(ui32Module)->RXUPPERLIMIT = pState->sRegState.regRXUPPERLIMIT;
                I2Sn(ui32Module)->TXLOWERLIMIT = pState->sRegState.regTXLOWERLIMIT;
                I2Sn(ui32Module)->INTEN        = pState->sRegState.regINTEN;
                I2Sn(ui32Module)->DMACFG       = pState->sRegState.regI2SDMACFG;

                pState->sRegState.bValid = false;
            }
            break;
        //
        // Turn off the I2S.
        //
        case AM_HAL_SYSCTRL_NORMALSLEEP:
        case AM_HAL_SYSCTRL_DEEPSLEEP:
            if (bRetainState)
            {
                //
                // Save I2S Registers
                //
                pState->sRegState.regI2SCTL       = I2Sn(ui32Module)->I2SCTL;
                pState->sRegState.regI2SDATACFG   = I2Sn(ui32Module)->I2SDATACFG;
                pState->sRegState.regI2SIOCFG     = I2Sn(ui32Module)->I2SIOCFG;
                pState->sRegState.regAMQCFG       = I2Sn(ui32Module)->AMQCFG;
                pState->sRegState.regI2SCLKCFG    = I2Sn(ui32Module)->CLKCFG;
                pState->sRegState.regIPBIRPT      = I2Sn(ui32Module)->IPBIRPT;
                pState->sRegState.regRXUPPERLIMIT = I2Sn(ui32Module)->RXUPPERLIMIT;
                pState->sRegState.regTXLOWERLIMIT = I2Sn(ui32Module)->TXLOWERLIMIT;
                pState->sRegState.regINTEN        = I2Sn(ui32Module)->INTEN;
                pState->sRegState.regI2SDMACFG    = I2Sn(ui32Module)->DMACFG;

                pState->sRegState.bValid = true;
            }
            //
            // Disable power control.
            //
            am_hal_pwrctrl_periph_disable(eI2SPowerModule);
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
// Interrupt disable.
//
//*****************************************************************************
uint32_t
am_hal_i2s_interrupt_disable(void *pHandle, uint32_t ui32IntMask)
{
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);

    I2Sn(ui32Module)->INTEN &= ~ui32IntMask;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Interrupt clear.
//
//*****************************************************************************
uint32_t
am_hal_i2s_interrupt_clear(void *pHandle, uint32_t ui32IntMask)
{
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);

    I2Sn(ui32Module)->INTCLR = ui32IntMask;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Returns the interrupt status.
//
//*****************************************************************************
uint32_t
am_hal_i2s_interrupt_status_get(void *pHandle, uint32_t *pui32Status, bool bEnabledOnly)
{
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);
    //
    // If requested, only return the interrupts that are enabled.
    //
    if ( bEnabledOnly )
    {
        *pui32Status = I2Sn(ui32Module)->INTSTAT;
        *pui32Status &= I2Sn(ui32Module)->INTEN;
    }
    else
    {
        *pui32Status = I2Sn(ui32Module)->INTSTAT;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Returns the DMA status(TXMDASTAT/RXDMASTAT)
//
//*****************************************************************************
uint32_t
am_hal_i2s_dma_status_get(void *pHandle, uint32_t *pui32Status, am_hal_i2s_xfer_dir_e xfer)
{
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;
    //
    // If requested, only return the interrupts that are enabled.
    //
    if (xfer == AM_HAL_I2S_XFER_RX)
    {
        *pui32Status = I2Sn(ui32Module)->RXDMASTAT;
    }
    else if (xfer == AM_HAL_I2S_XFER_TX)
    {
      *pui32Status = I2Sn(ui32Module)->TXDMASTAT;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I2S interrupt service routine
//
//*****************************************************************************
uint32_t am_hal_i2s_interrupt_service(void *pHandle, uint32_t ui32IntMask, am_hal_i2s_config_t* psConfig)
{
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = ((am_hal_i2s_state_t*)pHandle)->ui32Module;
    //
    // Full duplex RX/TX mode
    //
    if ( psConfig->eXfer == AM_HAL_I2S_XFER_RXTX )
    {
        //
        // The RX DMACPL interrupt asserts when the programmed DMA completes,
        // or ends with an error condition.
        //
        if (ui32IntMask & AM_HAL_I2S_INT_RXDMACPL)
        {
            if ( I2Sn(ui32Module)->RXDMASTAT & AM_HAL_I2S_STAT_DMA_RX_ERR )
            {
                //
                // Clear DMAERR bit.
                //
                am_hal_i2s_dma_error(pHandle, AM_HAL_I2S_XFER_RX);
            }
            //
            // restart dma transcation.
            //
            I2Sn(ui32Module)->DMACFG_b.RXDMAEN = 0x0;
            I2Sn(ui32Module)->RXDMASTAT        = 0x0;
            I2Sn(ui32Module)->RXDMAADDR        = pState->ui32RxBufferPtr = (pState->ui32RxBufferPtr == pState->ui32RxBufferPong)? pState->ui32RxBufferPing: pState->ui32RxBufferPong;
            I2Sn(ui32Module)->RXDMATOTCNT      = psConfig->eTransfer->ui32RxTotalCount;
            I2Sn(ui32Module)->DMACFG_b.RXDMAEN = 0x1;
        }
        //
        // The TX DMACPL interrupt asserts when the programmed DMA completes,
        // or ends with an error condition.
        //
        if (ui32IntMask & AM_HAL_I2S_INT_TXDMACPL)
        {
            if ( I2Sn(ui32Module)->TXDMASTAT & AM_HAL_I2S_STAT_DMA_TX_ERR )
            {
                am_hal_i2s_dma_error(pHandle, AM_HAL_I2S_XFER_TX);
            }
            //
            // Need to make sure all buffer writes are flushed
            //
            am_hal_sysctrl_bus_write_flush();
            //
            //restart DMA transaction
            //
            I2Sn(ui32Module)->DMACFG_b.TXDMAEN = 0x0;
            //
            // Clear dma status.
            //
            I2Sn(ui32Module)->TXDMASTAT        = 0x0;
            I2Sn(ui32Module)->TXDMAADDR        = pState->ui32TxBufferPtr = (pState->ui32TxBufferPtr == pState->ui32TxBufferPong)? pState->ui32TxBufferPing: pState->ui32TxBufferPong;
            I2Sn(ui32Module)->TXDMATOTCNT      = psConfig->eTransfer->ui32TxTotalCount;
            I2Sn(ui32Module)->DMACFG_b.TXDMAEN = 0x1;
        }

        if (ui32IntMask & AM_HAL_I2S_INT_IPB)
        {
            am_hal_i2s_ipb_interrupt_service(pHandle);
        }
    }
    else
    {
        if (ui32IntMask & AM_HAL_I2S_INT_RXDMACPL)
        {
            //
            // Once completed, software must first write the DMACFG register to 0
            //
            I2Sn(ui32Module)->DMACFG = 0x0;

            if ( I2Sn(ui32Module)->RXDMASTAT & AM_HAL_I2S_STAT_DMA_RX_ERR )
            {
                //
                // Clear DMAERR bit.
                //
                am_hal_i2s_dma_error(pHandle, AM_HAL_I2S_XFER_RX);
            }
            //
            // Clear dma status.
            //
            I2Sn(ui32Module)->RXDMASTAT = 0x0;
            //
            // restart DMA transaction
            //
            I2Sn(ui32Module)->RXDMATOTCNT = psConfig->eTransfer->ui32RxTotalCount;
            I2Sn(ui32Module)->RXDMAADDR   = pState->ui32RxBufferPtr = (pState->ui32RxBufferPtr == pState->ui32RxBufferPong)? pState->ui32RxBufferPing: pState->ui32RxBufferPong;
            //
            // High Priority (service immediately)
            //
            I2Sn(ui32Module)->DMACFG = I2S0_DMACFG_RXDMAPRI_Msk |
                                       I2S0_DMACFG_RXDMAEN_Msk;
        }
        //
        // The TX DMACPL interrupt asserts when the programmed DMA completes,
        // or ends with an error condition.
        //
        if (ui32IntMask & AM_HAL_I2S_INT_TXDMACPL)
        {
            //
            // Once completed, software must first write the DMACFG register to 0
            //
            I2Sn(ui32Module)->DMACFG = 0x0;

            if ( I2Sn(ui32Module)->TXDMASTAT & AM_HAL_I2S_STAT_DMA_TX_ERR )
            {
              //
              // Clear DMAERR bit.
              //
              am_hal_i2s_dma_error(pHandle, AM_HAL_I2S_XFER_TX);
            }
            //
            // Need to make sure all buffer writes are flushed
            //
            am_hal_sysctrl_bus_write_flush();
            //
            // Clear dma status.
            //
            I2Sn(ui32Module)->TXDMASTAT = 0x0;
            //
            // restart DMA transaction
            //
            I2Sn(ui32Module)->TXDMATOTCNT = psConfig->eTransfer->ui32TxTotalCount;
            I2Sn(ui32Module)->TXDMAADDR   = pState->ui32TxBufferPtr = (pState->ui32TxBufferPtr == pState->ui32TxBufferPong) ? pState->ui32TxBufferPing : pState->ui32TxBufferPong;
            //
            // High Priority (service immediately)
            //
            I2Sn(ui32Module)->DMACFG = I2S0_DMACFG_TXDMAPRI_Msk |
                                       I2S0_DMACFG_TXDMAEN_Msk;
        }
        if (ui32IntMask & AM_HAL_I2S_INT_IPB)
        {
            am_hal_i2s_ipb_interrupt_service(pHandle);
        }
    }
    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I2S IPB interrupt service routine
//
//*****************************************************************************
uint32_t am_hal_i2s_ipb_interrupt_service(void *pHandle)
{
    uint32_t ui32Module;
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t*)pHandle;
    ui32Module = pState->ui32Module;

    uint32_t ui32IntMask = I2Sn(ui32Module)->IPBIRPT;
    //
    // When the number of samples in the Transmit FIFO drops below the value in the Transmit
    // FIFO Lower Limit register (TXLOWERLIMIT)
    //
    // Transmit FIFO become empty
    //
    if (ui32IntMask & I2S0_IPBIRPT_TXEI_Msk)
    {
        //
        // clear TX_Ei
        //
        I2Sn(ui32Module)->IPBIRPT_b.TXEI = 0x0;

        //I2Sn(ui32Module)->I2SCTL_b.TXEN  = 0x0;
        //I2Sn(ui32Module)->I2SCTL_b.TXRST = 0x0;
    }

    if (ui32IntMask & I2S0_IPBIRPT_TXFFI_Msk)
    {
        I2Sn(ui32Module)->IPBIRPT_b.TXFFI = 0x0;
    }
    //
    // Receive FIFO become full
    //
    if (ui32IntMask & I2S0_IPBIRPT_RXFI_Msk)
    {
        //
        // To clear TX_Ei and RX_Fi, '0' must be written in these fields.
        // Otherwise, the interrupt signal will remain active.
        //
        I2Sn(ui32Module)->IPBIRPT_b.RXFI = 0x0;

        //I2Sn(ui32Module)->I2SCTL_b.RXEN  = 0x0;
        //I2Sn(ui32Module)->I2SCTL_b.RXRST = 0x0;
    }
    //
    // Receive fifo high limit interrupt
    //
    if (ui32IntMask & I2S0_IPBIRPT_RXFFI_Msk)
    {
        I2Sn(ui32Module)->IPBIRPT_b.RXFFI = 0x0;
    }
    //
    // Return the status.
    //
    return ui32Status;
}

//*****************************************************************************
//
// DMA transaction configuration.
//
//*****************************************************************************
uint32_t
am_hal_i2s_dma_configure(void *pHandle, am_hal_i2s_config_t* psConfig, am_hal_i2s_transfer_t *pTransferCfg)
{
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    AM_HAL_I2S_CHK_HANDLE(pHandle);
#endif // AM_HAL_DISABLE_API_VALIDATION
    //
    // Save the buffers.
    //
    pState->ui32RxBufferPtr  = pState->ui32RxBufferPing = pTransferCfg->ui32RxTargetAddr;
    pState->ui32RxBufferPong = pTransferCfg->ui32RxTargetAddrReverse;
    pState->ui32TxBufferPtr  = pState->ui32TxBufferPing = pTransferCfg->ui32TxTargetAddr;
    pState->ui32TxBufferPong = pTransferCfg->ui32TxTargetAddrReverse;
    //
    // Save the handle.
    //
    psConfig->eTransfer      = pTransferCfg;
    //
    // Clear the interrupts
    //
    I2Sn(ui32Module)->INTCLR = AM_HAL_I2S_INT_RXDMACPL | AM_HAL_I2S_INT_TXDMACPL |
                               AM_HAL_I2S_INT_TXREQCNT | AM_HAL_I2S_INT_RXREQCNT |
                               AM_HAL_I2S_INT_IPB;

    I2Sn(ui32Module)->IPBIRPT = 0x0;

    switch ( psConfig->eXfer )
    {
        case AM_HAL_I2S_XFER_RX:
            //
            // Set RX/TX DMATOTCOUNT.
            //
            I2Sn(ui32Module)->RXDMATOTCNT = pTransferCfg->ui32RxTotalCount;
            I2Sn(ui32Module)->RXDMAADDR   = pTransferCfg->ui32RxTargetAddr;
            //
            // Enable IP interrupts
            //
            I2Sn(ui32Module)->IPBIRPT = AM_HAL_I2S_INT_IPBIRPT_RXFF | AM_HAL_I2S_INT_IPBIRPT_RXDMA;
            I2Sn(ui32Module)->INTEN   = AM_HAL_I2S_INT_RXDMACPL ; //| AM_HAL_I2S_INT_IPB;
            break;

        case AM_HAL_I2S_XFER_TX:
            I2Sn(ui32Module)->TXDMATOTCNT = pTransferCfg->ui32TxTotalCount;
            I2Sn(ui32Module)->TXDMAADDR   = pTransferCfg->ui32TxTargetAddr;
            //
            // Enable IP interrupts
            //
            I2Sn(ui32Module)->IPBIRPT = AM_HAL_I2S_INT_IPBIRPT_TXFF | AM_HAL_I2S_INT_IPBIRPT_TXDMA;
            I2Sn(ui32Module)->INTEN   = AM_HAL_I2S_INT_TXDMACPL; // | AM_HAL_I2S_INT_IPB;
            break;

        case AM_HAL_I2S_XFER_RXTX:
            I2Sn(ui32Module)->TXDMATOTCNT = pTransferCfg->ui32TxTotalCount;
            I2Sn(ui32Module)->TXDMAADDR   = pTransferCfg->ui32TxTargetAddr;
            I2Sn(ui32Module)->RXDMATOTCNT = pTransferCfg->ui32RxTotalCount;
            I2Sn(ui32Module)->RXDMAADDR   = pTransferCfg->ui32RxTargetAddr;
            //
            // Enable IP interrupts
            //
            I2Sn(ui32Module)->IPBIRPT = AM_HAL_I2S_INT_IPBIRPT_RXFF | AM_HAL_I2S_INT_IPBIRPT_RXDMA |
                                        AM_HAL_I2S_INT_IPBIRPT_TXFF | AM_HAL_I2S_INT_IPBIRPT_TXDMA;
            I2Sn(ui32Module)->INTEN   = AM_HAL_I2S_INT_TXDMACPL | AM_HAL_I2S_INT_RXDMACPL;
            break;
    }
    //
    // High Priority (service immediately)
    //
    I2Sn(ui32Module)->DMACFG_b.RXDMAPRI = 0x1;
    I2Sn(ui32Module)->DMACFG_b.TXDMAPRI = 0x1;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// am_hal_i2s_dma_get_buffer
//
//*****************************************************************************
uint32_t
am_hal_i2s_dma_get_buffer(void *pHandle, am_hal_i2s_xfer_dir_e xfer)
{
    uint32_t ui32BufferPtr;
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;

    if ( AM_HAL_I2S_XFER_RX == xfer )
    {
        //
        // Invalidate DAXI to make sure CPU sees the new data when loaded
        //
        am_hal_daxi_control(AM_HAL_DAXI_CONTROL_INVALIDATE, 0);

        ui32BufferPtr = (pState->ui32RxBufferPtr == pState->ui32RxBufferPong)? pState->ui32RxBufferPing: pState->ui32RxBufferPong;
    }
    else
    {
        ui32BufferPtr = (pState->ui32TxBufferPtr == pState->ui32TxBufferPong)? pState->ui32TxBufferPing: pState->ui32TxBufferPong;
    }

    return ui32BufferPtr;
}

//*****************************************************************************
//
// I2S enable function
//
//*****************************************************************************
uint32_t
am_hal_i2s_enable(void *pHandle)
{
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    AM_HAL_I2S_CHK_HANDLE(pHandle);
#endif // AM_HAL_DISABLE_API_VALIDATION

    if (pState->prefix.s.bEnable)
    {
        return AM_HAL_STATUS_SUCCESS;
    }
    //
    // Enable the audio clock.
    //
    //I2Sn(ui32Module)->CLKCFG_b.MCLKEN = 0x1;

    pState->prefix.s.bEnable = true;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I2S disable function
//
//*****************************************************************************
uint32_t
am_hal_i2s_disable(void *pHandle)
{
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t*)pHandle;
    uint32_t ui32Module = pState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    AM_HAL_I2S_CHK_HANDLE(pHandle);
#endif // AM_HAL_DISABLE_API_VALIDATION

    if (!pState->prefix.s.bEnable)
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    I2Sn(ui32Module)->CLKCFG = 0x0;

    pState->prefix.s.bEnable = false;

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_i2s_disable()

//*****************************************************************************
//
// am_hal_i2s_dma_transfer_complete
//
//*****************************************************************************
uint32_t
am_hal_i2s_dma_transfer_complete(void *pHandle)
{
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    //
    // Once completed, software must first write the DMACFG register to 0,
    // prior to making any update
    //
    I2Sn(ui32Module)->DMACFG = 0x0;

    //
    // Clear dma status.
    //
    I2Sn(ui32Module)->RXDMASTAT = 0x0;
    I2Sn(ui32Module)->TXDMASTAT = 0x0;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// am_hal_i2s_tx_fifo_empty
//
//*****************************************************************************
bool
am_hal_i2s_tx_fifo_empty(void *pHandle)
{
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    //
    // AM_HAL_I2S_INT_TXDMACPL is always triggered before the completion of FIFO TX,
    // So check the FIFOCNT to guarantee all datas transfered completely before next DMA transaction.
    //
    if ( I2Sn(ui32Module)->TXFIFOSTATUS_b.TXFIFOCNT == 0 )
    {
        return true;
    }
    else
    {
        return false;
    }
}

//*****************************************************************************
//
// am_hal_i2s_dma_error
//
//*****************************************************************************
uint32_t
am_hal_i2s_dma_error(void *pHandle, am_hal_i2s_xfer_dir_e xfer)
{
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    //
    // If an error condition did occur during a DMA operation, the DMA must first be disabled
    //
    I2Sn(ui32Module)->DMACFG = 0x0;

    //
    // DMA status bits cleared.
    //
    if (xfer == AM_HAL_I2S_XFER_RX)
    {
        I2Sn(ui32Module)->RXDMASTAT = 0x0;
    }
    else if (xfer == AM_HAL_I2S_XFER_TX)
    {
        I2Sn(ui32Module)->TXDMASTAT = 0x0;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
