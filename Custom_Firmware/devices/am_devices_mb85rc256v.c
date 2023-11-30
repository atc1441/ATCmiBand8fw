//*****************************************************************************
//
//! @file am_devices_mb85rc256v.c
//!
//! @brief Fujitsu 256K I2C FRAM driver.
//!
//! @addtogroup mb85rc256v MB85RC256V I2C FRAM driver
//! @ingroup devices
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

#include "am_mcu_apollo.h"
#include "am_devices_mb85rc256v.h"
#include "am_bsp.h"

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
typedef struct
{
    uint32_t                    ui32Module;
    void                        *pIomHandle;
    bool                        bOccupied;
} am_devices_iom_mb85rc256v_t;

am_devices_iom_mb85rc256v_t gAmMb85rc256v[AM_DEVICES_MB85RC256V_MAX_DEVICE_NUM];

am_hal_iom_config_t     g_sIomMb85rc256vCfg =
{
    .eInterfaceMode       = AM_HAL_IOM_I2C_MODE,
    .ui32ClockFreq        = AM_HAL_IOM_1MHZ,
    .eSpiMode             = AM_HAL_IOM_SPI_MODE_0,
    .ui32NBTxnBufLength   = 0,
    .pNBTxnBuf = NULL,
};

//*****************************************************************************
//
// Generic Command Write function.
//
//*****************************************************************************
static uint32_t
am_device_command_write(void *pHandle, uint8_t ui8DevAddr, uint32_t ui32InstrLen,
                        uint64_t ui64Instr, bool bCont,
                        uint32_t *pData, uint32_t ui32NumBytes)
{
    am_hal_iom_transfer_t Transaction;

    //
    // Create the transaction.
    //
    Transaction.ui32InstrLen    = ui32InstrLen;
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    Transaction.ui64Instr       = ui64Instr;
#else
    Transaction.ui32Instr       = (uint32_t)ui64Instr;
#endif
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes    = ui32NumBytes;
    Transaction.pui32TxBuffer   = pData;
    Transaction.uPeerInfo.ui32I2CDevAddr = ui8DevAddr;
    Transaction.bContinue       = bCont;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    //
    // Execute the transction over IOM.
    //
    if (am_hal_iom_blocking_transfer(pHandle, &Transaction))
    {
        return AM_DEVICES_MB85RC256V_STATUS_ERROR;
    }
    return AM_DEVICES_MB85RC256V_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Generic Command Read function.
//
//*****************************************************************************
static uint32_t
am_device_command_read(void *pHandle, uint8_t ui8DevAddr, uint32_t ui32InstrLen, uint64_t ui64Instr,
                       bool bCont, uint32_t *pData, uint32_t ui32NumBytes)
{
    am_hal_iom_transfer_t  Transaction;

    //
    // Create the transaction.
    //
    Transaction.ui32InstrLen    = ui32InstrLen;
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    Transaction.ui64Instr       = ui64Instr;
#else
    Transaction.ui32Instr       = (uint32_t)ui64Instr;
#endif
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes    = ui32NumBytes;
    Transaction.pui32RxBuffer   = pData;
    Transaction.uPeerInfo.ui32I2CDevAddr = ui8DevAddr;
    Transaction.bContinue       = bCont;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    //
    // Execute the transction over IOM.
    //
    if (am_hal_iom_blocking_transfer(pHandle, &Transaction))
    {
        return AM_DEVICES_MB85RC256V_STATUS_ERROR;
    }
    return AM_DEVICES_MB85RC256V_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Initialize the mb85rc256v driver.
//
//*****************************************************************************
uint32_t
am_devices_mb85rc256v_init(uint32_t ui32Module, am_devices_mb85rc256v_config_t *pDevConfig, void **ppHandle, void **ppIomHandle)
{
    void *pIomHandle;
    am_hal_iom_config_t     stIOMMB85RC256VSettings;
    uint32_t      ui32Index = 0;

    // Allocate a vacant device handle
    for ( ui32Index = 0; ui32Index < AM_DEVICES_MB85RC256V_MAX_DEVICE_NUM; ui32Index++ )
    {
        if ( gAmMb85rc256v[ui32Index].bOccupied == false )
        {
            break;
        }
    }
    if ( ui32Index == AM_DEVICES_MB85RC256V_MAX_DEVICE_NUM )
    {
        return AM_DEVICES_MB85RC256V_STATUS_ERROR;
    }

    if ( (ui32Module > AM_REG_IOM_NUM_MODULES) || (pDevConfig == NULL) )
    {
        return AM_DEVICES_MB85RC256V_STATUS_ERROR;
    }

    //
    // Enable fault detection.
    //
#if defined(AM_PART_APOLLO4_API)
    am_hal_fault_capture_enable();
#elif AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_FAULT_CAPTURE_ENABLE, 0);
#else
    am_hal_mcuctrl_fault_capture_enable();
#endif

    stIOMMB85RC256VSettings = g_sIomMb85rc256vCfg;
    stIOMMB85RC256VSettings.ui32NBTxnBufLength = pDevConfig->ui32NBTxnBufLength;
    stIOMMB85RC256VSettings.pNBTxnBuf = pDevConfig->pNBTxnBuf;
    stIOMMB85RC256VSettings.ui32ClockFreq = pDevConfig->ui32ClockFreq;

    //
    // Initialize the IOM instance.
    // Enable power to the IOM instance.
    // Configure the IOM for Serial operation during initialization.
    // Enable the IOM.
    //
    if (am_hal_iom_initialize(ui32Module, &pIomHandle) ||
        am_hal_iom_power_ctrl(pIomHandle, AM_HAL_SYSCTRL_WAKE, false) ||
        am_hal_iom_configure(pIomHandle, &stIOMMB85RC256VSettings) ||
        am_hal_iom_enable(pIomHandle))
    {
        return AM_DEVICES_MB85RC256V_STATUS_ERROR;
    }
    else
    {
        //
        // Configure the IOM pins.
        //
        am_bsp_iom_pins_enable(ui32Module, AM_HAL_IOM_I2C_MODE);

        gAmMb85rc256v[ui32Index].bOccupied = true;
        gAmMb85rc256v[ui32Index].ui32Module = ui32Module;
        *ppIomHandle = gAmMb85rc256v[ui32Index].pIomHandle = pIomHandle;
        *ppHandle = (void *)&gAmMb85rc256v[ui32Index];
        //
        // Return the status.
        //
        return AM_DEVICES_MB85RC256V_STATUS_SUCCESS;
    }
}

//*****************************************************************************
//
// De-Initialize the mb85rc256v driver.
//
//*****************************************************************************
uint32_t
am_devices_mb85rc256v_term(void *pHandle)
{
    am_devices_iom_mb85rc256v_t *pIom = (am_devices_iom_mb85rc256v_t *)pHandle;

    if ( pIom->ui32Module > AM_REG_IOM_NUM_MODULES )
    {
        return AM_DEVICES_MB85RC256V_STATUS_ERROR;
    }

    //
    // Disable the IOM.
    //
    am_hal_iom_disable(pIom->pIomHandle);

    //
    // Disable power to and uninitialize the IOM instance.
    //
    am_hal_iom_power_ctrl(pIom->pIomHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false);

    am_hal_iom_uninitialize(pIom->pIomHandle);

    // Free this device handle
    pIom->bOccupied = false;

    //
    // Configure the IOM pins.
    //
    am_bsp_iom_pins_disable(pIom->ui32Module, AM_HAL_IOM_I2C_MODE);

    //
    // Return the status.
    //
    return AM_DEVICES_MB85RC256V_STATUS_SUCCESS;
}


//*****************************************************************************
//
//  Reads the ID of the external flash and returns the value.
//
//*****************************************************************************
uint32_t
am_devices_mb85rc256v_read_id(void *pHandle, uint32_t *pDeviceID)
{
    am_devices_iom_mb85rc256v_t *pIom = (am_devices_iom_mb85rc256v_t *)pHandle;

    //
    // Send the command sequence to read the Device ID.
    //
    if (am_device_command_read(pIom->pIomHandle, AM_DEVICES_MB85RC256V_RSVD_SLAVE_ID, 1,
                           (AM_DEVICES_MB85RC256V_SLAVE_ID << 1),
                           false, pDeviceID, 3))
    {
        return AM_DEVICES_MB85RC256V_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MB85RC256V_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Programs the given range of flash addresses.
//
//*****************************************************************************
uint32_t
am_devices_mb85rc256v_blocking_write(void *pHandle, uint8_t *pui8TxBuffer,
                                     uint32_t ui32WriteAddress,
                                     uint32_t ui32NumBytes)
{
    am_devices_iom_mb85rc256v_t *pIom = (am_devices_iom_mb85rc256v_t *)pHandle;

    //
    // Write the data to the device.
    //
    if (am_device_command_write(pIom->pIomHandle, AM_DEVICES_MB85RC256V_SLAVE_ID, 2,
                            (ui32WriteAddress & 0x0000FFFF),
                            false, (uint32_t *)pui8TxBuffer, ui32NumBytes))
    {
        return AM_DEVICES_MB85RC256V_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MB85RC256V_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Programs the given range of flash addresses.
//
//*****************************************************************************
uint32_t
am_devices_mb85rc256v_nonblocking_write(void *pHandle, uint8_t *pui8TxBuffer,
                                        uint32_t ui32WriteAddress,
                                        uint32_t ui32NumBytes,
                                        am_hal_iom_callback_t pfnCallback,
                                        void *pCallbackCtxt)
{
    am_hal_iom_transfer_t         Transaction;
    am_devices_iom_mb85rc256v_t   *pIom = (am_devices_iom_mb85rc256v_t *)pHandle;

    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.ui8Priority     = 1;        // High priority for now.
    Transaction.uPeerInfo.ui32I2CDevAddr = AM_DEVICES_MB85RC256V_SLAVE_ID;
    Transaction.bContinue       = false;

    //
    // Set up the IOM transaction.
    //
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.ui32InstrLen    = 2;
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    Transaction.ui64Instr       = ui32WriteAddress & 0x0000FFFF;
#else
    Transaction.ui32Instr       = ui32WriteAddress & 0x0000FFFF;
#endif
    Transaction.ui32NumBytes    = ui32NumBytes;
    Transaction.pui32TxBuffer   = (uint32_t *)pui8TxBuffer;

    //
    // Add this transaction to the command queue (no callback).
    //
    if (am_hal_iom_nonblocking_transfer(pIom->pIomHandle, &Transaction, pfnCallback, pCallbackCtxt))
    {
        return AM_DEVICES_MB85RC256V_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MB85RC256V_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Reads the contents of the fram into a buffer.
//
//*****************************************************************************
uint32_t
am_devices_mb85rc256v_blocking_read(void *pHandle, uint8_t *pui8RxBuffer,
                                    uint32_t ui32ReadAddress,
                                    uint32_t ui32NumBytes)
{
    am_devices_iom_mb85rc256v_t   *pIom = (am_devices_iom_mb85rc256v_t *)pHandle;

    if (am_device_command_read(pIom->pIomHandle, AM_DEVICES_MB85RC256V_SLAVE_ID, 2,
                           (ui32ReadAddress & 0x0000FFFF),
                           false, (uint32_t *)pui8RxBuffer, ui32NumBytes))
    {
        return AM_DEVICES_MB85RC256V_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MB85RC256V_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Reads the contents of the fram into a buffer.
//
//*****************************************************************************
uint32_t
am_devices_mb85rc256v_nonblocking_read(void *pHandle, uint8_t *pui8RxBuffer,
                                       uint32_t ui32ReadAddress,
                                       uint32_t ui32NumBytes,
                                       am_hal_iom_callback_t pfnCallback,
                                       void * pCallbackCtxt)
{
    am_hal_iom_transfer_t Transaction;
    am_devices_iom_mb85rc256v_t   *pIom = (am_devices_iom_mb85rc256v_t *)pHandle;

    //
    // Set up the IOM transaction.
    //
    Transaction.ui8Priority     = 1;        // High priority for now.
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32InstrLen    = 2;
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    Transaction.ui64Instr       = (ui32ReadAddress & 0x0000FFFF);
#else
    Transaction.ui32Instr       = (ui32ReadAddress & 0x0000FFFF);
#endif
    Transaction.ui32NumBytes    = ui32NumBytes;
    Transaction.pui32RxBuffer   = (uint32_t *)pui8RxBuffer;
    Transaction.uPeerInfo.ui32I2CDevAddr = AM_DEVICES_MB85RC256V_SLAVE_ID;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.bContinue       = false;

    //
    // Start the transaction.
    //
    if (am_hal_iom_nonblocking_transfer(pIom->pIomHandle, &Transaction, pfnCallback, pCallbackCtxt))
    {
        return AM_DEVICES_MB85RC256V_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MB85RC256V_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

