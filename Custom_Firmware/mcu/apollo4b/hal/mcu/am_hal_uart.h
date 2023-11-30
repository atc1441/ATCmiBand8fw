//*****************************************************************************
//
//! @file am_hal_uart.h
//!
//! @brief Hardware abstraction for the UART
//!
//! @addtogroup uart_4b UART Functionality
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

#ifndef AM_HAL_UART_H
#define AM_HAL_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Global definitions
//
//*****************************************************************************
#define UARTn(n)    ((UART0_Type*)(UART0_BASE + (n * (UART1_BASE - UART0_BASE))))

//*****************************************************************************
//
// UART error codes.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_UART_STATUS_BUS_ERROR = AM_HAL_STATUS_MODULE_SPECIFIC_START,
    AM_HAL_UART_STATUS_RX_QUEUE_FULL,
    AM_HAL_UART_STATUS_CLOCK_NOT_CONFIGURED,
    AM_HAL_UART_STATUS_BAUDRATE_NOT_POSSIBLE,
    AM_HAL_UART_STATUS_TX_CHANNEL_BUSY,
    AM_HAL_UART_STATUS_RX_CHANNEL_BUSY,
}
am_hal_uart_errors_t;

//*****************************************************************************
//
// UART interrupts.
//
//*****************************************************************************
#define AM_HAL_UART_INT_OVER_RUN            UART0_IER_OEIM_Msk
#define AM_HAL_UART_INT_BREAK_ERR           UART0_IER_BEIM_Msk
#define AM_HAL_UART_INT_PARITY_ERR          UART0_IER_PEIM_Msk
#define AM_HAL_UART_INT_FRAME_ERR           UART0_IER_FEIM_Msk
#define AM_HAL_UART_INT_RX_TMOUT            UART0_IER_RTIM_Msk
#define AM_HAL_UART_INT_TX                  UART0_IER_TXIM_Msk
#define AM_HAL_UART_INT_RX                  UART0_IER_RXIM_Msk
#define AM_HAL_UART_INT_DSRM                UART0_IER_DSRMIM_Msk
#define AM_HAL_UART_INT_DCDM                UART0_IER_DCDMIM_Msk
#define AM_HAL_UART_INT_CTSM                UART0_IER_CTSMIM_Msk
#define AM_HAL_UART_INT_TXCMP               UART0_IER_TXCMPMIM_Msk

//*****************************************************************************
//
//! @name UART Flag Register
//! @{
//! Macro definitions for UART Flag Register Bits.
//!
//! They may be used with the \e am_hal_uart_flags_get() function.
//!
//
//*****************************************************************************
#define AM_HAL_UART_FR_TX_EMPTY             UART0_FR_TXFE_Msk
#define AM_HAL_UART_FR_RX_FULL              UART0_FR_RXFF_Msk
#define AM_HAL_UART_FR_TX_FULL              UART0_FR_TXFF_Msk
#define AM_HAL_UART_FR_RX_EMPTY             UART0_FR_RXFE_Msk
#define AM_HAL_UART_FR_BUSY                 UART0_FR_BUSY_Msk
#define AM_HAL_UART_FR_DCD_DETECTED         UART0_FR_DCD_Msk
#define AM_HAL_UART_FR_DSR_READY            UART0_FR_DSR_Msk
#define AM_HAL_UART_FR_CTS                  UART0_FR_CTS_Msk
//! @}

//*****************************************************************************
//
// UART configuration options.
//
//*****************************************************************************

//
// Fifo interrupt level.
//
typedef enum
{
    AM_HAL_UART_FIFO_LEVEL_4  = 0,
    AM_HAL_UART_FIFO_LEVEL_8  = 1,
    AM_HAL_UART_FIFO_LEVEL_16 = 2,
    AM_HAL_UART_FIFO_LEVEL_24 = 3,
    AM_HAL_UART_FIFO_LEVEL_28 = 4,
}
am_hal_uart_fifo_level_e;

//
// Number of data bits per UART frame.
//
typedef enum
{
    AM_HAL_UART_DATA_BITS_5 = 0,
    AM_HAL_UART_DATA_BITS_6 = 1,
    AM_HAL_UART_DATA_BITS_7 = 2,
    AM_HAL_UART_DATA_BITS_8 = 3,
}
am_hal_uart_data_bits_e;

//
// Parity options.
//
typedef enum
{
    AM_HAL_UART_PARITY_ODD,
    AM_HAL_UART_PARITY_EVEN,
    AM_HAL_UART_PARITY_NONE,
}
am_hal_uart_parity_e;

//
// Stop bit options.
//
typedef enum
{
    AM_HAL_UART_ONE_STOP_BIT = 0,
    AM_HAL_UART_TWO_STOP_BITS = 1,
}
am_hal_uart_stop_bits_e;

//
// Flow control options.
//
typedef enum
{
    AM_HAL_UART_FLOW_CTRL_NONE     = 0,
    AM_HAL_UART_FLOW_CTRL_CTS_ONLY = UART0_CR_CTSEN_Msk,
    AM_HAL_UART_FLOW_CTRL_RTS_ONLY = UART0_CR_RTSEN_Msk,
    AM_HAL_UART_FLOW_CTRL_RTS_CTS  = (UART0_CR_CTSEN_Msk | UART0_CR_RTSEN_Msk),
}
am_hal_uart_flow_control_e;

typedef struct
{
    uint32_t                   ui32BaudRate; // Baud rate
    am_hal_uart_data_bits_e    eDataBits;    // Number of bits per frame
    am_hal_uart_parity_e       eParity;      // UART parity
    am_hal_uart_stop_bits_e    eStopBits;    // Number of stop bits
    am_hal_uart_flow_control_e eFlowControl; // Flow control option
    am_hal_uart_fifo_level_e   eTXFifoLevel; // TX fifo interrupt level
    am_hal_uart_fifo_level_e   eRXFifoLevel; // RX fifo interrupt level
}
am_hal_uart_config_t;

//*****************************************************************************
//
// UART transfer structure.
//
//*****************************************************************************

//
// The type of transfer to execute.
//
// For blocking transfers, the CPU will poll until the requested number of
// bytes have been transferred, or the timeout interval elapses, whichever
// happens first.
//
// For non-blocking transfers, the CPU will read or write as many bytes as
// possible immediately, and then the interrupt service routine will handle the
// rest.
//
typedef enum
{
    AM_HAL_UART_BLOCKING_WRITE,
    AM_HAL_UART_BLOCKING_READ,
    AM_HAL_UART_NONBLOCKING_WRITE,
    AM_HAL_UART_NONBLOCKING_READ,
}
am_hal_uart_transfer_type_e;

typedef struct
{
    //
    // Is this a write or a read?
    //
    am_hal_uart_transfer_type_e eType;

    //
    // Data location to use for this transaction.
    //
    uint8_t *pui8Data;

    //
    // How many bytes should we send?
    //
    uint32_t ui32NumBytes;

    //
    // When the transaction is complete, this will be set to the number of
    // bytes we read.
    //
    uint32_t *pui32BytesTransferred;

    //
    // For blocking transactions, this determines how long the UART HAL should
    // wait before aborting the transaction.
    //
    uint32_t ui32TimeoutMs;

    //
    // For non-blocking transfers, the UART HAL will call this callback
    // function as soon as the requested action is complete.
    //
    void (*pfnCallback)(uint32_t ui32ErrorStatus, void *pvContext);

    //
    // This context variable will be saved and provided to the callback
    // function when it is called.
    //
    void *pvContext;

    //
    // This context variable will be saved and provided to the callback
    // function when it is called.
    //
    uint32_t ui32ErrorStatus;
}
am_hal_uart_transfer_t;

//
// A few helpful UART transfer defaults.
//
#define AM_HAL_UART_BLOCKING_WRITE_DEFAULTS                                   \
{                                                                             \
    .eType = AM_HAL_UART_BLOCKING_WRITE,                                      \
    .pui8Data = 0,                                                            \
    .ui32NumBytes = 0,                                                        \
    .pui32BytesTransferred = 0,                                               \
    .ui32TimeoutMs = AM_HAL_UART_WAIT_FOREVER,                                \
    .pfnCallback = 0,                                                         \
    .pvContext = 0,                                                           \
    .ui32ErrorStatus = 0,                                                     \
};

#define AM_HAL_UART_BLOCKING_READ_DEFAULTS                                    \
{                                                                             \
    .eType = AM_HAL_UART_BLOCKING_READ,                                       \
    .pui8Data = 0,                                                            \
    .ui32NumBytes = 0,                                                        \
    .pui32BytesTransferred = 0,                                               \
    .ui32TimeoutMs = AM_HAL_UART_WAIT_FOREVER,                                \
    .pfnCallback = 0,                                                         \
    .pvContext = 0,                                                           \
    .ui32ErrorStatus = 0,                                                     \
}

#define AM_HAL_UART_NONBLOCKING_WRITE_DEFAULTS                                \
{                                                                             \
    .eType = AM_HAL_UART_NONBLOCKING_WRITE,                                   \
    .pui8Data = 0,                                                            \
    .ui32NumBytes = 0,                                                        \
    .pui32BytesTransferred = 0,                                               \
    .ui32TimeoutMs = AM_HAL_UART_WAIT_FOREVER,                                \
    .pfnCallback = 0,                                                         \
    .pvContext = 0,                                                           \
    .ui32ErrorStatus = 0,                                                     \
}

#define AM_HAL_UART_NONBLOCKING_READ_DEFAULTS                                 \
{                                                                             \
    .eType = AM_HAL_UART_NONBLOCKING_READ,                                    \
    .pui8Data = 0,                                                            \
    .ui32NumBytes = 0,                                                        \
    .pui32BytesTransferred = 0,                                               \
    .ui32TimeoutMs = AM_HAL_UART_WAIT_FOREVER,                                \
    .pfnCallback = 0,                                                         \
    .pvContext = 0,                                                           \
    .ui32ErrorStatus = 0,                                                     \
}

//
// Use this value if you want to keep a UART transaction blocking forever.
//
#define AM_HAL_UART_WAIT_FOREVER            0xFFFFFFFF

//*****************************************************************************
//
//! @brief UART enable macro
//!
//! This will allow the user a more standard function call for enabling the UART
//! If the user wishes to retain the state, use am_hal_uart_power_control(..., true)
//!
//! @return HAL status code.
//
//*****************************************************************************
#define am_hal_uart_power_enable(phandle) am_hal_uart_power_control(phandle, AM_HAL_SYSCTRL_WAKE, false);

//*****************************************************************************
//
//! @brief UART disable macro
//!
//! This will allow the user a more standard function call for disabling the UART
//! If the user wishes to retain the state, use am_hal_uart_power_control(..., true)
//!
//! @return HAL status code.
//
//*****************************************************************************
#define am_hal_uart_power_disable(phandle) am_hal_uart_power_control(phandle, AM_HAL_SYSCTRL_DEEPSLEEP, false);

//*****************************************************************************
//
// External functions.
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Initialize the UART internal RX/TX queue
//!
//! @param pHandle is a pointer to a UART handle to initialize.
//! @param pui8TxBuffer is the pointer to buffer with length of ui32TxBufferSize.
//! @param ui32TxBufferSize is the length of Tx buffer.
//! @param pui8RxBuffer is the pointer to buffer with length of ui32RxBufferSize.
//! @param ui32RxBufferSize is the length of Rx buffer.
//!
//! This function enables the internal UART queue (FIFO) for UART Tx/Rx. When
//! Tx or Rx Queue are enabled, the UART data is stored internally when UART
//! hardware FIFO is full.
//!
//! @return HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_uart_buffer_configure(void *pHandle,
                                             uint8_t *pui8TxBuffer,
                                             uint32_t ui32TxBufferSize,
                                             uint8_t *pui8RxBuffer,
                                             uint32_t ui32RxBufferSize);

//*****************************************************************************
//
//! @brief Initialize the UART
//!
//! @param ui32Module is the module number of the UART to use.
//! @param ppHandle is a pointer to a UART handle to initialize.
//!
//! This function prepares a UART module for use, and returns an initialized
//! handle for use by the application.
//!
//! @return HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_uart_initialize(uint32_t ui32Module, void **ppHandle);

//*****************************************************************************
//
//! @brief Deinitialize the UART interface.
//!
//! @param pHandle is a previously initialized UART handle.
//!
//! This function effectively disables future calls to interact with the UART
//! refered to by \e Handle. The user may call this function if UART operation
//! is no longer desired.
//!
//! @return HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_uart_deinitialize(void *pHandle);

//*****************************************************************************
//
//! @brief Change the power state of the UART module.
//!
//! @param pHandle is the handle for the UART to operate on.
//! @param ePowerState is the desired power state of the UART.
//! @param bRetainState is a flag to ask the HAL to save UART registers.
//!
//! This function can be used to switch the power to the UART on or off. If \e
//! bRetainState is true during a powerdown operation, it will store the UART
//! configuration registers to SRAM, so it can restore them on power-up.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable UART errors.
//
//*****************************************************************************
extern uint32_t am_hal_uart_power_control(void *pHandle,
                                          uint32_t ePowerState,
                                          bool bRetainState);

//*****************************************************************************
//
//! @brief Used to configure basic UART settings.
//!
//! @param pHandle is the handle for the UART to operate on.
//! @param psConfig is a structure of UART configuration options.
//!
//! This function takes the options from an \e am_hal_uart_config_t structure,
//! and applies them to the UART referred to by \e pHandle.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable UART errors.
//
//*****************************************************************************
extern uint32_t am_hal_uart_configure(void *pHandle,
                                      const am_hal_uart_config_t *psConfig);

//*****************************************************************************
//
//! @brief Read from the UART RX FIFO.
//!
//! @param pHandle is the UART handle to use.
//! @param pui8Data is the buffer to read bytes into.
//! @param ui32NumBytes is the number of bytes to try to read.
//! @param pui32NumBytesRead will be set to the number of bytes actually read.
//!
//! This function reads bytes directly from the UART FIFO into the specified
//! data location.
//!
//! @return HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_uart_fifo_read(void *pHandle,
                                      uint8_t *pui8Data,
                                      uint32_t ui32NumBytes,
                                      uint32_t *pui32NumBytesRead);

//*****************************************************************************
//
//! @brief Write to the UART TX FIFO
//!
//! @param pHandle is the UART handle to use.
//! @param pui8Data is the buffer to write bytes from.
//! @param ui32NumBytes is the number of bytes to attempt to write.
//! @param pui32NumBytesWritten will be set to the number of bytes written.
//!
//! This function writes bytes from memory into the UART FIFO.
//!
//! @return HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_uart_fifo_write(void *pHandle,
                                       uint8_t *pui8Data,
                                       uint32_t ui32NumBytes,
                                       uint32_t *pui32NumBytesWritten);

//*****************************************************************************
//
//! @brief Wait for the UART TX to become idle
//!
//! @param pHandle is the handle for the UART to operate on.
//!
//! This function waits (polling) for all data in the UART TX FIFO and UART TX
//! buffer (if configured) to be fully sent on the physical UART interface.
//! This is not the most power-efficient way to wait for UART idle, but it can be
//! useful in simpler applications, or where power-efficiency is less important.
//!
//! Once this function returns, the UART can be safely disabled without
//! interfering with any previous transmissions.
//!
//! For a more power-efficient way to shut down the UART, check the
//! \e am_hal_uart_interrupt_service() function.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable UART errors.
//
//*****************************************************************************
extern uint32_t am_hal_uart_tx_flush(void *pHandle);

//*****************************************************************************
//
//! @brief Run a UART transaction.
//!
//! @param pHandle is the UART handle to use.
//! @param psTransfer is a structure defining the desired transfer.
//!
//! This function is used for all UART transactions. See the documentation for
//! \e am_hal_uart_transfer_t for more information about the options.
//!
//! @return HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_uart_transfer(void *pHandle,
                                     const am_hal_uart_transfer_t *psTransfer);

//*****************************************************************************
//
//! @brief Read the UART flags.
//!
//! @param pHandle is the UART handle to use.
//! @param pui32Flags is the destination pointer for the UART flags.
//!
//! The UART hardware provides some information about the state of the physical
//! interface at all times. This function provides a way to read that data
//! directly. Below is a list of all possible UART flags.
//!
//! These correspond directly to the bits in the UART_FR register.
//!
//! @code
//!
//! AM_HAL_UART_FR_TX_EMPTY
//! AM_HAL_UART_FR_RX_FULL
//! AM_HAL_UART_FR_TX_FULL
//! AM_HAL_UART_FR_RX_EMPTY
//! AM_HAL_UART_FR_BUSY
//! AM_HAL_UART_FR_DCD_DETECTED
//! AM_HAL_UART_FR_DSR_READY
//! AM_HAL_UART_FR_CTS
//!
//! @endcode
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable UART errors.
//
//*****************************************************************************
extern uint32_t am_hal_uart_flags_get(void *pHandle, uint32_t *pui32Flags);

//*****************************************************************************
//
//! @brief Enable interrupts.
//!
//! @param pHandle is the handle for the UART to operate on.
//! @param ui32IntMask is the bitmask of interrupts to enable.
//!
//! This function enables the UART interrupt(s) given by ui32IntMask. If
//! multiple interrupts are desired, they can be OR'ed together.
//!
//! @note This function need not be called for UART FIFO interrupts if the UART
//! buffer service provided by \e am_hal_uart_buffer_configure() and \e
//! am_hal_uart_interrupt_service() is already in use. Non-FIFO-related
//! interrupts do require the use of this function.
//!
//! The full list of interrupts is given by the following:
//!
//! @code
//!
//! AM_HAL_UART_INT_OVER_RUN
//! AM_HAL_UART_INT_BREAK_ERR
//! AM_HAL_UART_INT_PARITY_ERR
//! AM_HAL_UART_INT_FRAME_ERR
//! AM_HAL_UART_INT_RX_TMOUT
//! AM_HAL_UART_INT_TX
//! AM_HAL_UART_INT_RX
//! AM_HAL_UART_INT_DSRM
//! AM_HAL_UART_INT_DCDM
//! AM_HAL_UART_INT_CTSM
//! AM_HAL_UART_INT_TXCMP
//!
//! @endcode
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable UART errors.
//
//*****************************************************************************
extern uint32_t am_hal_uart_interrupt_enable(void *pHandle,
                                             uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief Disable interrupts.
//!
//! @param pHandle is the handle for the UART to operate on.
//! @param ui32IntMask is the bitmask of interrupts to disable.
//!
//! This function disables the UART interrupt(s) given by ui32IntMask. If
//! multiple interrupts need to be disabled, they can be OR'ed together.
//!
//! @note This function need not be called for UART FIFO interrupts if the UART
//! buffer service provided by \e am_hal_uart_buffer_configure() and \e
//! am_hal_uart_interrupt_service() is already in use. Non-FIFO-related
//! interrupts do require the use of this function.
//!
//! The full list of interrupts is given by the following:
//!
//! @code
//!
//! AM_HAL_UART_INT_OVER_RUN
//! AM_HAL_UART_INT_BREAK_ERR
//! AM_HAL_UART_INT_PARITY_ERR
//! AM_HAL_UART_INT_FRAME_ERR
//! AM_HAL_UART_INT_RX_TMOUT
//! AM_HAL_UART_INT_TX
//! AM_HAL_UART_INT_RX
//! AM_HAL_UART_INT_DSRM
//! AM_HAL_UART_INT_DCDM
//! AM_HAL_UART_INT_CTSM
//! AM_HAL_UART_INT_TXCMP
//!
//! @endcode
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable UART errors.
//
//*****************************************************************************
extern uint32_t am_hal_uart_interrupt_disable(void *pHandle,
                                              uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief Clear interrupt status.
//!
//! @param pHandle is the handle for the UART to operate on.
//! @param ui32IntMask is the bitmask of interrupts to clear.
//!
//! This function clears the UART interrupt(s) given by ui32IntMask. If
//! multiple interrupts need to be cleared, they can be OR'ed together.
//!
//! The full list of interrupts is given by the following:
//!
//! @code
//!
//! AM_HAL_UART_INT_OVER_RUN
//! AM_HAL_UART_INT_BREAK_ERR
//! AM_HAL_UART_INT_PARITY_ERR
//! AM_HAL_UART_INT_FRAME_ERR
//! AM_HAL_UART_INT_RX_TMOUT
//! AM_HAL_UART_INT_TX
//! AM_HAL_UART_INT_RX
//! AM_HAL_UART_INT_DSRM
//! AM_HAL_UART_INT_DCDM
//! AM_HAL_UART_INT_CTSM
//! AM_HAL_UART_INT_TXCMP
//!
//! @endcode
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable UART errors.
//
//*****************************************************************************
extern uint32_t am_hal_uart_interrupt_clear(void *pHandle,
                                            uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief Read interrupt status.
//!
//! @param pHandle is the handle for the UART to operate on.
//!
//! @param pui32Status is the returned interrupt status (all bits OR'ed
//! together)
//!
//! @param bEnabledOnly determines whether to read interrupts that were not
//! enabled.
//!
//! This function reads the status the UART interrupt(s) if \e bEnabled is
//! true, it will only return the status of the enabled interrupts. Otherwise,
//! it will return the status of all interrupts, enabled or disabled.
//!
//! The full list of interrupts is given by the following:
//!
//! @code
//!
//! AM_HAL_UART_INT_OVER_RUN
//! AM_HAL_UART_INT_BREAK_ERR
//! AM_HAL_UART_INT_PARITY_ERR
//! AM_HAL_UART_INT_FRAME_ERR
//! AM_HAL_UART_INT_RX_TMOUT
//! AM_HAL_UART_INT_TX
//! AM_HAL_UART_INT_RX
//! AM_HAL_UART_INT_DSRM
//! AM_HAL_UART_INT_DCDM
//! AM_HAL_UART_INT_CTSM
//! AM_HAL_UART_INT_TXCMP
//!
//! @endcode
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable UART errors.
//
//*****************************************************************************
extern uint32_t am_hal_uart_interrupt_status_get(void *pHandle,
                                                 uint32_t *pui32Status,
                                                 bool bEnabledOnly);

//*****************************************************************************
//
//! @brief Check to see which interrupts are enabled.
//!
//! @param pHandle is the handle for the UART to operate on.
//!
//! @param pui32IntMask is the current set of interrupt enable bits (all bits
//!                     OR'ed together)
//!
//! This function checks the UART Interrupt Enable Register to see which UART
//! interrupts are currently enabled. The result will be an interrupt mask with
//! one bit set for each of the currently enabled UART interrupts.
//!
//! The full set of UART interrupt bits is given by the list below:
//!
//! @code
//!
//! AM_HAL_UART_INT_OVER_RUN
//! AM_HAL_UART_INT_BREAK_ERR
//! AM_HAL_UART_INT_PARITY_ERR
//! AM_HAL_UART_INT_FRAME_ERR
//! AM_HAL_UART_INT_RX_TMOUT
//! AM_HAL_UART_INT_TX
//! AM_HAL_UART_INT_RX
//! AM_HAL_UART_INT_DSRM
//! AM_HAL_UART_INT_DCDM
//! AM_HAL_UART_INT_CTSM
//! AM_HAL_UART_INT_TXCMP
//!
//! @endcode
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable UART errors.
//
//*****************************************************************************
extern uint32_t am_hal_uart_interrupt_enable_get(void *pHandle, uint32_t *pui32IntMask);
//*****************************************************************************
//
//! @brief transfer data between hardware and software queue in the background.
//!
//! @param pHandle is the handle for the UART to operate on.
//!
//! @param ui32Status is the interrupt flag and can be any combination of
//!  AM_HAL_UART_INT_RX, AM_HAL_UART_INT_RX_TMOUT, or AM_HAL_UART_INT_TX.
//!
//! This function should be called from the ISR and then recieve/transmit data
//! from/to hardware FIFO.
//!
//! @return AM_HAL_STATUS_SUCCESS or applicable UART errors.
//
//*****************************************************************************
extern uint32_t am_hal_uart_interrupt_service(void *pHandle,
                                              uint32_t ui32Status);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_UART_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

