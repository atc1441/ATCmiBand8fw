//*****************************************************************************
//
//! @file am_hal_i2s.h
//!
//! @brief HAL implementation for the Inter-IC Sound module.
//!
//! @addtogroup i2s4_4p I2S - Inter-IC Sound
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
#ifndef AM_HAL_I2S_H
#define AM_HAL_I2S_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! CMSIS-style macro for handling a variable I2S module number.
//
//*****************************************************************************
#define I2Sn(n) ((I2S0_Type*)(I2S0_BASE + (n * (I2S1_BASE - I2S0_BASE))))

//*****************************************************************************
//
//! @name I2S Specific status codes
//! @{
//
//*****************************************************************************
#define AM_HAL_I2S_POWER_ON           AM_HAL_SYSCTRL_WAKE
#define AM_HAL_I2S_POWER_OFF          AM_HAL_SYSCTRL_NORMALSLEEP
//! @}

//
//! The FIFO size for both the TX and RX FIFOs is 64 samples.
//
#define AM_HAL_I2S_FIFO_SIZE          64

//*****************************************************************************
//
//! I2S internal clock speed selection.
//
//*****************************************************************************
typedef enum
{
    eAM_HAL_I2S_CLKSEL_HFRC2_48MHz = 0,
    eAM_HAL_I2S_CLKSEL_HFRC2_24MHz,
    eAM_HAL_I2S_CLKSEL_HFRC2_12MHz,
    eAM_HAL_I2S_CLKSEL_HFRC2_6MHz,
    eAM_HAL_I2S_CLKSEL_HFRC2_3MHz,
    eAM_HAL_I2S_CLKSEL_HFRC2_1_5MHz,
    eAM_HAL_I2S_CLKSEL_HFRC2_750KHz,
    eAM_HAL_I2S_CLKSEL_HFRC2_375KHz,

    eAM_HAL_I2S_CLKSEL_HFRC_24MHz,
    eAM_HAL_I2S_CLKSEL_HFRC_12MHz,
    eAM_HAL_I2S_CLKSEL_HFRC_6MHz,
    eAM_HAL_I2S_CLKSEL_HFRC_3MHz,
    eAM_HAL_I2S_CLKSEL_HFRC_1_5MHz,
    eAM_HAL_I2S_CLKSEL_HFRC_750KHz,
    eAM_HAL_I2S_CLKSEL_HFRC_375KHz,

    eAM_HAL_I2S_CLKSEL_XTHS_EXTREF_CLK,
    eAM_HAL_I2S_CLKSEL_XTHS_16MHz,
    eAM_HAL_I2S_CLKSEL_XTHS_8MHz,
    eAM_HAL_I2S_CLKSEL_XTHS_4MHz,
    eAM_HAL_I2S_CLKSEL_XTHS_2MHz,
    eAM_HAL_I2S_CLKSEL_XTHS_1MHz,
    eAM_HAL_I2S_CLKSEL_XTHS_500KHz,
    eAM_HAL_I2S_CLKSEL_MAX
} am_hal_i2s_clksel_e;

//*****************************************************************************
//
//! @name I2S interrupts macros
//! @{
//
//*****************************************************************************
#define AM_HAL_I2S_INT_RXDMACPL         I2S0_INTSTAT_RXDMACPL_Msk
#define AM_HAL_I2S_INT_TXDMACPL         I2S0_INTSTAT_TXDMACPL_Msk
#define AM_HAL_I2S_INT_TXREQCNT         I2S0_INTSTAT_TXREQCNT_Msk
#define AM_HAL_I2S_INT_RXREQCNT         I2S0_INTSTAT_RXREQCNT_Msk
#define AM_HAL_I2S_INT_IPB              I2S0_INTSTAT_IPB_Msk

#define AM_HAL_I2S_STAT_DMA_TX_ERR      I2S0_TXDMASTAT_TXDMAERR_Msk
#define AM_HAL_I2S_STAT_DMA_RX_ERR      I2S0_TXDMASTAT_TXDMAERR_Msk

#define AM_HAL_I2S_INT_IPBIRPT_TXDMA    I2S0_IPBIRPT_TXDMAM_Msk
#define AM_HAL_I2S_INT_IPBIRPT_RXDMA    I2S0_IPBIRPT_RXDMAM_Msk
#define AM_HAL_I2S_INT_IPBIRPT_TXE      I2S0_IPBIRPT_TXEM_Msk
#define AM_HAL_I2S_INT_IPBIRPT_RXF      I2S0_IPBIRPT_RXFM_Msk
#define AM_HAL_I2S_INT_IPBIRPT_TXFF     I2S0_IPBIRPT_TXFFM_Msk
#define AM_HAL_I2S_INT_IPBIRPT_RXFF     I2S0_IPBIRPT_RXFFM_Msk

//
//! @}
//

//*****************************************************************************
//
// General defines
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief enumeration types for the I2S.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_I2S_REQ_INTSET = 0,
    AM_HAL_I2S_REQ_INTCLR,
    AM_HAL_I2S_REQ_TXFIFOCNT,
    AM_HAL_I2S_REQ_READ_RXUPPERLIMIT,
    AM_HAL_I2S_REQ_READ_TXLOWERLIMIT,
    AM_HAL_I2S_REQ_WRITE_RXUPPERLIMIT,
    AM_HAL_I2S_REQ_WRITE_TXLOWERLIMIT,
    AM_HAL_I2S_REQ_MAX
} am_hal_i2s_request_e;

//*****************************************************************************
//
//! @brief enumeration types for the I2S clock selection.
//
//*****************************************************************************
typedef enum
{
    //! no change to the clock selected by FSEL.
    AM_HAL_I2S_CLKDIV_1 = 0,

    //! frequency divide-by-3 of the clock selected by FSEL.
    AM_HAL_I2S_CLKDIV_3 = 1
} am_hal_i2s_clk_div_e;

//*****************************************************************************
//
//! @name I2S Available word length/fsync period
//! @{
//
//*****************************************************************************
#define AM_HAL_I2S_WORD_8BITS                  8
#define AM_HAL_I2S_WORD_16BITS                 16
#define AM_HAL_I2S_WORD_24BITS                 24
#define AM_HAL_I2S_WORD_32BITS                 32
#define AM_HAL_I2S_WORD_RESERVED               0
//
//! @}
//

//*****************************************************************************
//
//! I2S Channel Length
//
//*****************************************************************************
typedef enum
{
    AM_HAL_I2S_FRAME_WDLEN_8BITS  = I2S0_I2SDATACFG_WDLEN1_8b,
    AM_HAL_I2S_FRAME_WDLEN_16BITS = I2S0_I2SDATACFG_WDLEN1_16b,
    AM_HAL_I2S_FRAME_WDLEN_24BITS = I2S0_I2SDATACFG_WDLEN1_24b,
    AM_HAL_I2S_FRAME_WDLEN_32BITS = I2S0_I2SDATACFG_WDLEN1_32b,
    AM_HAL_I2S_FRAME_WDLEN_MAX,
} am_hal_i2s_channel_length_e;

//*****************************************************************************
//
//! I2S Sample Length
//
//*****************************************************************************
typedef enum
{
    AM_HAL_I2S_SAMPLE_LENGTH_8BITS  = I2S0_I2SDATACFG_SSZ1_8b,
    AM_HAL_I2S_SAMPLE_LENGTH_16BITS = I2S0_I2SDATACFG_SSZ1_16b,
    AM_HAL_I2S_SAMPLE_LENGTH_24BITS = I2S0_I2SDATACFG_SSZ1_24b,
    AM_HAL_I2S_SAMPLE_LENGTH_32BITS = I2S0_I2SDATACFG_SSZ1_32b
} am_hal_i2s_sample_length_e;

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
//! Specifies the data format of I2S sub frames
//! I2SDATACFG @0x00000040
//!
//! Read Phase Bit. 0: Single Phase frame; 1: Dual-Phase frame.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_I2S_DATA_PHASE_SINGLE,
    AM_HAL_I2S_DATA_PHASE_DUAL
} am_hal_i2s_data_phase_e;

//*****************************************************************************
//
//! Audio sample justification. 0: Left-justified, 1: Right-justified
//
//*****************************************************************************
typedef enum
{
    AM_HAL_I2S_DATA_JUSTIFIED_LEFT,
    AM_HAL_I2S_DATA_JUSTIFIED_RIGHT
} am_hal_i2s_data_justified_e;

//*****************************************************************************
//
//! @name Specified polarity and clock configuration of the I2S IPB clocks and
//!   IO signals
//!   REGISTER OFFSET: 0x44
//!
//! @{
//
typedef enum
{
    AM_HAL_I2S_IO_MODE_SLAVE,
    AM_HAL_I2S_IO_MODE_MASTER
} am_hal_i2s_io_mode_e;

typedef enum
{
    AM_HAL_I2S_IO_SDATA_OUTPUT_DISABLE,
    AM_HAL_I2S_IO_SDATA_OUTPUT_ENABLE
} am_hal_i2s_io_sdata_output_e;

typedef enum
{
    AM_HAL_I2S_IO_RX_CPOL_RISING,
    AM_HAL_I2S_IO_RX_CPOL_FALLING
} am_hal_i2s_io_rx_cpol_e;

typedef enum
{
    AM_HAL_I2S_IO_TX_CPOL_FALLING,
    AM_HAL_I2S_IO_TX_CPOL_RISING
} am_hal_i2s_io_tx_cpol_e;

typedef enum
{
    AM_HAL_I2S_IO_FSYNC_CPOL_HIGH,
    AM_HAL_I2S_IO_FSYNC_CPOL_LOW
} am_hal_i2s_io_fsync_cpol_e;

typedef enum
{
    AM_HAL_I2S_XFER_RX,
    AM_HAL_I2S_XFER_TX,
    AM_HAL_I2S_XFER_RXTX
} am_hal_i2s_xfer_dir_e;
//! @}

//*****************************************************************************
//
//! @brief Configuration structure for the I2S/TDM Data format.
//
//*****************************************************************************
typedef struct
{
    //
    //! PH: Read Phase Bit. 0: Single Phase frame; 1: Dual-Phase frame.
    //
    am_hal_i2s_data_phase_e ePhase;

    //
    //! FRLEN1: Number of channels in phase 1: N+1
    //
    uint32_t  ui32ChannelNumbersPhase1;

    //
    //! FRLEN2: Number of channels in phase 2: N+1
    //
    uint32_t  ui32ChannelNumbersPhase2;

    //
    //! WDLEN1: channel length in bits for phase 1
    //! 0: 8b,1: 12b, 2: 16b, 3: 20b, 4: 24b, 5: 32b.
    //
    am_hal_i2s_channel_length_e  eChannelLenPhase1;

    //
    //! WDLEN2: channel length in bits for phase 2
    //
    am_hal_i2s_channel_length_e  eChannelLenPhase2;

    //
    //! DATADLY: Receive data delay bit count. Valid values are 0-2
    //
    uint32_t  eDataDelay;

    //
    //! SSZ1: Receive audio sample length for phase 1. 0: 8b, 1: 12b, 2: 16b, 3: 20b, 4: 24b, 5: 32b.
    //
    am_hal_i2s_sample_length_e  eSampleLenPhase1;

    //
    //! SSZ2: Receive audio sample length for phase 2. 0: 8b, 1: 12b, 2: 16b, 3: 20b, 4: 24b, 5: 32b.
    //
    am_hal_i2s_sample_length_e  eSampleLenPhase2;

    //
    //! JUST: Audio sample justification. 0: Left-justified, 1: Right-justified
    //
    am_hal_i2s_data_justified_e eDataJust;
} am_hal_i2s_data_format_t;

//*****************************************************************************
//
//! @brief Configuration structure for the I2S IPB clocks and IO signals.
//
//*****************************************************************************
typedef struct
{
    //
    //! MSL: Master/Slave configuration. 0: External clock. 1: Internal clock.
    //
    //am_hal_i2s_io_mode_e eMode;

    //
    //! OEN: Output enable for SDATA output
    //
    am_hal_i2s_io_sdata_output_e eOutput;

    //
    //! FPER: Frame period in units of sclk. (N+1) in length
    //
    uint32_t eFramecPeriod;

    //
    //! FWID: period of fsync/lr_clk in units of sclks
    //
    uint32_t eFsyncPeriod;

    //
    //! FSP: Polarity of fsync/lr_clk signal. 0: Active high. 1: Active low
    //
    am_hal_i2s_io_fsync_cpol_e eFyncCpol;

    //
    //! PRx/CLKP: Receive clock edge polarity bit. 0: rising edge. 1: falling edge.
    //
    am_hal_i2s_io_rx_cpol_e eRxCpol;

    //
    //! PRTX: Transmit clock edge polarity bit. 0: sdata starting from the falling edge. 1: sdata starting from the rising edge.
    //
    am_hal_i2s_io_tx_cpol_e eTxCpol;
} am_hal_i2s_io_signal_t;

//*****************************************************************************
//
//! DMA transfer structure
//
//*****************************************************************************
typedef struct
{
    uint32_t ui32RxTargetAddr;
    uint32_t ui32RxTargetAddrReverse;
    uint32_t ui32RxTotalCount;

    uint32_t ui32TxTargetAddr;
    uint32_t ui32TxTargetAddrReverse;
    uint32_t ui32TxTotalCount;
} am_hal_i2s_transfer_t;

//*****************************************************************************
//
//! @brief Configuration structure for the I2S.
//
//*****************************************************************************
typedef struct
{
    //
    //! User setting.
    //
    am_hal_i2s_io_mode_e         eMode;
    am_hal_i2s_xfer_dir_e        eXfer;

    am_hal_i2s_clksel_e          eClock;
    uint32_t                     eDiv3;
    uint32_t                     eASRC;

    //
    //! I2S data format.
    //
    am_hal_i2s_data_format_t*    eData;

    //
    //! I2S signal io.
    //
    am_hal_i2s_io_signal_t*      eIO;

    //
    //! DMA transfer.
    //
    am_hal_i2s_transfer_t*      eTransfer;
} am_hal_i2s_config_t;

//*****************************************************************************
//
// Functions
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief initialize the I2S device controller
//!
//! @param ui32Module - the index to the I2S
//! @param ppHandle    - pointer the handle of initialized I2S instance
//!
//! This function should be called firstly before we use any other I2S HAL driver
//! functions.
//!
//! @return status    - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_i2s_initialize(uint32_t ui32Module, void **ppHandle);

//*****************************************************************************
//
//! @brief Uninitialize the I2S device controller
//!
//! @param pHandle - the handle of initialized I2S instance
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_i2s_deinitialize(void *pHandle);

//*****************************************************************************
//
//! @brief I2S Power control function. function
//!
//! @param pHandle      - handle for the I2S.
//! @param ePowerState  - power state requested
//! @param bRetainState - boolean on whether to retain state
//!
//! This function allows advanced settings
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_i2s_power_control(void *pHandle, am_hal_sysctrl_power_state_e ePowerState, bool bRetainState);

//*****************************************************************************
//
//! @brief I2S configuration function
//!
//! @param pHandle  - handle for the module instance.
//! @param psConfig - pointer to the configuration structure.
//!
//! This function configures the I2S for operation.
//!
//! @return status  - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_i2s_configure(void *pHandle, am_hal_i2s_config_t *psConfig);

//*****************************************************************************
//
//! @brief I2S enable function
//!
//! @param pHandle - handle for the module instance.
//!
//! This function enables the I2S operation.
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************

extern uint32_t am_hal_i2s_enable(void *pHandle);

//*****************************************************************************
//
//! @brief I2S disable function
//!
//! @param pHandle - handle for the module instance.
//!
//! This function disables the I2S operation.
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_i2s_disable(void *pHandle);

//*****************************************************************************
//
//! @brief I2S Interrupt Service Routine
//!
//! @param pHandle     - handle for the module instance.
//! @param ui32IntMask - uint32_t for interrupts to clear
//! @param psConfig    - Pointer to the I2S Config
//!
//! @return AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_i2s_interrupt_service(void *pHandle, uint32_t ui32IntMask, am_hal_i2s_config_t* psConfig);

//*****************************************************************************
//
//! @brief I2S IPB Interrupt Service Routine
//!
//! This function is currently called internal to am_hal_i2s_interrupt_service
//! IRB -> full-duplex I2S (stereo TX + stereo RX) using shared CLK & FS
//!
//! @param pHandle     - handle for the module instance.
//!
//! @return AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_i2s_ipb_interrupt_service(void *pHandle);

//*****************************************************************************
//
//! @brief I2S interrupt status function
//!
//! @param pHandle      - handle for the interface.
//! @param pui32Status  - pointer to status
//! @param bEnabledOnly - if I2S enabled
//!
//! This function returns the specific indicated interrupt status.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_i2s_interrupt_status_get(void *pHandle, uint32_t *pui32Status, bool bEnabledOnly);

//*****************************************************************************
//
//! @brief I2S interrupt clear
//!
//! @param pHandle     - handle for the interface.
//! @param ui32IntMask - uint32_t for interrupts to clear
//!
//! This function clears the interrupts for the given peripheral.
//!
//! @return status     - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_i2s_interrupt_clear(void *pHandle, uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief I2S disable interrupts function
//!
//! @param pHandle       - handle for the interface.
//! @param ui32IntMask   - I2S interrupt mask.
//!
//! This function disable the specific indicated interrupts.
//!
//! @return status       - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_i2s_interrupt_disable(void *pHandle, uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief I2S DMA Complete
//!
//! Gets the DMA status(TXMDASTAT/RXDMASTAT)
//!
//! @param pHandle     - handle for the interface.
//! @param pui32Status - Pointer to the DMA Status
//! @param xfer        - Type of DMA Transfer
//!
//! @return AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_i2s_dma_status_get(void *pHandle, uint32_t *pui32Status, am_hal_i2s_xfer_dir_e xfer);

//*****************************************************************************
//
//! @brief I2S DMA Transaction Configuration
//!
//! Gets the DMA status(TXMDASTAT/RXDMASTAT)
//!
//! @param pHandle      - handle for the interface.
//! @param psConfig     - Pointer to the I2S Config
//! @param pTransferCfg - Pointer to the I2S Transaction Config
//!
//! @return AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_i2s_dma_configure(void *pHandle, am_hal_i2s_config_t* psConfig, am_hal_i2s_transfer_t *pTransferCfg);

//*****************************************************************************
//
//! @brief I2S DMA NonBlocking Transfer Start
//!
//! Gets the DMA status(TXMDASTAT/RXDMASTAT)
//!
//! @param pHandle - handle for the interface.
//! @param pConfig - Pointer to the I2S Config
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_i2s_dma_transfer_start(void *pHandle, am_hal_i2s_config_t *pConfig);

//*****************************************************************************
//
//! @brief I2S DMA NonBlocking Transfer Continue
//!
//! Gets the DMA status(TXMDASTAT/RXDMASTAT)
//!
//! @param pHandle - handle for the interface.
//! @param psConfig - Pointer to the I2S Config
//! @param pTransferCfg - Pointer to the I2S Transaction Config
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_i2s_dma_transfer_continue(void *pHandle, am_hal_i2s_config_t* psConfig, am_hal_i2s_transfer_t *pTransferCfg);

//*****************************************************************************
//
//! @brief I2S DMA Complete
//!
//! Complete DMA Transfer by writing the DMACFG register to 0 and clearing the
//! DMA TX/RX status register.
//!
//! @param pHandle - handle for the interface.
//!
//! @return AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_i2s_dma_transfer_complete(void *pHandle);

//*****************************************************************************
//
//! @brief I2S DMA Get Buffer
//!
//! @param pHandle - handle for the interface.
//! @param xfer    - Type of DMA Transfer
//!
//! @return Pointer to the DMA Buffer
//
//*****************************************************************************
extern uint32_t am_hal_i2s_dma_get_buffer(void *pHandle, am_hal_i2s_xfer_dir_e xfer);

//*****************************************************************************
//
//! @brief I2S DMA Error Check
//!
//! @param pHandle - handle for the interface.
//! @param xfer    - Type of DMA Transfer
//!
//! @return AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_i2s_dma_error(void *pHandle, am_hal_i2s_xfer_dir_e xfer);

//*****************************************************************************
//
//! @brief I2S DMA TX Buffer Empty Check
//!
//! @param pHandle - handle for the interface.
//!
//! @return true if empty, false is not empty
//
//*****************************************************************************
extern bool am_hal_i2s_tx_fifo_empty(void *pHandle);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_I2S_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

