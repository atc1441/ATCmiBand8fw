//*****************************************************************************
//
//! @file am_devices_spipsram.c
//!
//! @brief General SPI PSRAM driver.
//!
//! @addtogroup spipsram SPI PSRAM Driver
//! @ingroup devices
//! @{
//
//**************************************************************************

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

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "am_mcu_apollo.h"
#include "am_devices_spipsram.h"
#include "am_util_stdio.h"
#include "am_bsp.h"
#include "am_util_delay.h"

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
typedef struct
{
    uint32_t                    ui32Module;
    uint32_t                    ui32CS;
    uint32_t                    ui32MaxTransSize;
    void                        *pIomHandle;
    am_hal_iom_config_t         sSpiPsramCfg ;
    bool                        bOccupied;
} am_devices_iom_aps6404l_t;

am_devices_iom_aps6404l_t gAmAps6404l[AM_DEVICES_APS6404L_MAX_DEVICE_NUM] = {0};

bool g_IOMSave[AM_REG_IOM_NUM_MODULES] = {0};

am_hal_iom_config_t     g_sSpiPsramCfg =
{
    .eInterfaceMode       = AM_HAL_IOM_SPI_MODE,
    .ui32ClockFreq        = AM_HAL_IOM_1MHZ,
    .eSpiMode             = AM_HAL_IOM_SPI_MODE_0,
    .ui32NBTxnBufLength   = 0,
    .pNBTxnBuf = NULL,
};

const struct
{
    const uint32_t MHz;
    const uint32_t MaxSize;
} g_SPI_SpeedMax[] =
{
    {AM_HAL_IOM_48MHZ, AM_DEVICES_SPIPSRAM_48MHZ_MAX_BYTES},
    {AM_HAL_IOM_24MHZ, AM_DEVICES_SPIPSRAM_24MHZ_MAX_BYTES},
    {AM_HAL_IOM_16MHZ, AM_DEVICES_SPIPSRAM_16MHZ_MAX_BYTES},
    {AM_HAL_IOM_12MHZ, AM_DEVICES_SPIPSRAM_12MHZ_MAX_BYTES},
    {AM_HAL_IOM_8MHZ,  AM_DEVICES_SPIPSRAM_8MHZ_MAX_BYTES}  //!< Leave this in for PSRAM initialization at 8MHz.
};


#define AM_DEVICES_SPIPSRAM_TIMEOUT             1000000


//*****************************************************************************
//
//! @brief
//! @param pCallbackCtxt
//! @param status
//
//*****************************************************************************
static void pfnSPI_PSRAM_Callback(void *pCallbackCtxt, uint32_t status)
{
    //! Set the DMA complete flag.
    *(volatile bool *)pCallbackCtxt = true;
}

typedef struct
{
   uint32_t    ui32OFFSETHIAddr;
   uint32_t    ui32OFFSETHIVal; //!< 0
   uint32_t    ui32DEVCFGAddr;
   uint32_t    ui32DEVCFGVal;
   uint32_t    ui32DMACFGdis1Addr;
   uint32_t    ui32DMACFGdis1Val;
   uint32_t    ui32DMATOTCOUNTAddr;
   uint32_t    ui32DMATOTCOUNTVal; //!< Corresponding to the Read size
   uint32_t    ui32DMATARGADDRAddr;
   uint32_t    ui32DMATARGADDRVal;
   uint32_t    ui32DMACFGAddr; //!< Configure for the Read (second transaction)
   uint32_t    ui32DMACFGVal;
#if !defined(AM_PART_APOLLO4B) && !defined(AM_PART_APOLLO4P) && !defined(AM_PART_APOLLO4L)
   uint32_t    ui32FIFOPushAddr1;
   uint32_t    ui32FIFOPushVal1; // Pointer to a variable initialized with 4 bytes value as : 1 Byte Command + 3 byte address
   uint32_t    ui32FIFOPushAddr2;
   uint32_t    ui32FIFOPushVal2; // 1 dummy byte to take care of 8 bit wait time for fast read
   uint32_t    ui32CMD1Addr;
   uint32_t    ui32CMD1Val; // First command which is a Write transaction with size 4 bytes, offset size 0, Continue ON
#endif
   uint32_t    ui32CMD2Addr;
   uint32_t    ui32CMD2Val; //!< Second command which is Read transaction with given size, offset size 0, Continue OFF
} spi_psram_trans_read_txn_t;

spi_psram_trans_read_txn_t  gIomTransReadTxn;

typedef struct
{
    uint32_t    ui32OFFSETHIAddr;
    uint32_t    ui32OFFSETHIVal; //!< 0
    uint32_t    ui32DEVCFGAddr;
    uint32_t    ui32DEVCFGVal;
    uint32_t    ui32DMACFGdis1Addr;
    uint32_t    ui32DMACFGdis1Val;
    uint32_t    ui32DMATOTCOUNTAddr;
    uint32_t    ui32DMATOTCOUNTVal; //!< Corresponding to the Write size
    uint32_t    ui32DMATARGADDRAddr;
    uint32_t    ui32DMATARGADDRVal;
#if !defined(AM_PART_APOLLO4B) && !defined(AM_PART_APOLLO4P) && !defined(AM_PART_APOLLO4L)
    uint32_t    ui32FIFOPushAddr1;
    uint32_t    ui32FIFOPushVal1; //!< Pointer to a variable initialized with 4 bytes value as : 1 Byte Command + 3 byte address
#endif
    uint32_t    ui32DMACFGAddr; //!< Configure for the Write
    uint32_t    ui32DMACFGVal;
    uint32_t    ui32CMD1Addr;
    uint32_t    ui32CMD1Val; //!< command which is a Write transaction with size (Length + 4) bytes, offset size 0, Continue OFF
} spi_psram_trans_write_txn_t;

spi_psram_trans_write_txn_t  gIomTransWriteTxn;


//*****************************************************************************
//
//! @brief Function to build the CMD value.
//!
//! @param ui32CS
//! @param ui32Dir
//! @param ui32Cont
//! @param ui32Offset
//! @param ui32OffsetCnt
//! @param ui32nBytes
//!
//! @note The OFFSETHI register must still be handled by the caller, e.g.
//!      AM_REGn(IOM, ui32Module, OFFSETHI) = (uint16_t)(ui32Offset >> 8);
//!
//! @return Returns the CMD value, but does not set the CMD register.
//
//*****************************************************************************
static uint32_t
build_iom_cmd(uint32_t ui32CS,     uint32_t ui32Dir, uint32_t ui32Cont,
              uint32_t ui32Offset, uint32_t ui32OffsetCnt,
              uint32_t ui32nBytes)
{
    //
    // Initialize the CMD variable
    //
    uint32_t ui32Cmd = 0;

    //
    // If SPI, we'll need the chip select
    //
    ui32Cmd |= _VAL2FLD(IOM0_CMD_CMDSEL, ui32CS);

    //
    // Build the CMD with number of bytes and direction.
    //
    ui32Cmd |= _VAL2FLD(IOM0_CMD_TSIZE, ui32nBytes);

    if (ui32Dir == AM_HAL_IOM_RX)
    {
       ui32Cmd |= _VAL2FLD(IOM0_CMD_CMD, IOM0_CMD_CMD_READ);
    }
    else
    {
       ui32Cmd |= _VAL2FLD(IOM0_CMD_CMD, IOM0_CMD_CMD_WRITE);
    }

    ui32Cmd |= _VAL2FLD(IOM0_CMD_CONT, ui32Cont);

    //
    // Now add the OFFSETLO and OFFSETCNT information.
    //
    ui32Cmd |= _VAL2FLD(IOM0_CMD_OFFSETLO, (uint8_t)ui32Offset);
    ui32Cmd |= _VAL2FLD(IOM0_CMD_OFFSETCNT, ui32OffsetCnt);

    return ui32Cmd;
} // build_iom_cmd()

//*****************************************************************************
//
//! @brief
//! @param ui32Module
//
//*****************************************************************************
static void
iom_init_cq_element(uint32_t ui32Module)
{
    spi_psram_trans_read_txn_t *pIomCqRd = &gIomTransReadTxn;
    spi_psram_trans_write_txn_t *pIomCqWr = &gIomTransWriteTxn;
    uint32_t ui32DMACFGValRd     =
       _VAL2FLD(IOM0_DMACFG_DMAPRI, 1) |
       _VAL2FLD(IOM0_DMACFG_DMADIR, 0) | IOM0_DMACFG_DMAEN_Msk;
    uint32_t ui32DMACFGValWr     =
       _VAL2FLD(IOM0_DMACFG_DMAPRI, 1) |
       _VAL2FLD(IOM0_DMACFG_DMADIR, 1) | IOM0_DMACFG_DMAEN_Msk;

    // Initialize the cq element
    //
    // Command for OFFSETHI
    //
    pIomCqWr->ui32OFFSETHIAddr = pIomCqRd->ui32OFFSETHIAddr  = (uint32_t)&IOMn(ui32Module)->OFFSETHI;
    pIomCqWr->ui32OFFSETHIVal   = pIomCqRd->ui32OFFSETHIVal   = 0;

    //
    // Command for I2C DEVADDR field in DEVCFG
    //
    pIomCqWr->ui32DEVCFGAddr    = pIomCqRd->ui32DEVCFGAddr    = (uint32_t)&IOMn(ui32Module)->DEVCFG;
    pIomCqWr->ui32DEVCFGVal     = pIomCqRd->ui32DEVCFGVal     = 0;

    //
    // Command to disable DMA before writing TOTCOUNT.
    //
    pIomCqWr->ui32DMACFGdis1Addr =  pIomCqRd->ui32DMACFGdis1Addr = (uint32_t)&IOMn(ui32Module)->DMACFG;
    pIomCqWr->ui32DMACFGdis1Val = pIomCqRd->ui32DMACFGdis1Val = 0x0;

    //
    // Command to set DMATOTALCOUNT
    //
    pIomCqWr->ui32DMATOTCOUNTAddr = pIomCqRd->ui32DMATOTCOUNTAddr = (uint32_t)&IOMn(ui32Module)->DMATOTCOUNT;

    //
    // Command to set DMATARGADDR
    //
    pIomCqWr->ui32DMATARGADDRAddr = pIomCqRd->ui32DMATARGADDRAddr = (uint32_t)&IOMn(ui32Module)->DMATARGADDR;

#if !defined(AM_PART_APOLLO4B) && !defined(AM_PART_APOLLO4P) && !defined(AM_PART_APOLLO4L)
    //
    // Command to set FIFO
    //
    pIomCqWr->ui32FIFOPushAddr1 = pIomCqRd->ui32FIFOPushAddr1 = (uint32_t)&IOMn(ui32Module)->FIFOPUSH;
    pIomCqRd->ui32FIFOPushAddr2 = (uint32_t)&IOMn(ui32Module)->FIFOPUSH;
#endif

    //
    // Command to set DMACFG to start the DMA operation
    //
    pIomCqWr->ui32DMACFGAddr =  pIomCqRd->ui32DMACFGAddr =  (uint32_t)&IOMn(ui32Module)->DMACFG;
    pIomCqWr->ui32DMACFGVal = ui32DMACFGValWr;
    pIomCqRd->ui32DMACFGVal = ui32DMACFGValRd;

#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    pIomCqWr->ui32CMD1Addr = (uint32_t)&IOMn(ui32Module)->CMD;
    pIomCqRd->ui32CMD2Addr = (uint32_t)&IOMn(ui32Module)->CMD;
#else
    pIomCqWr->ui32CMD1Addr = pIomCqRd->ui32CMD1Addr = (uint32_t)&IOMn(ui32Module)->CMD;
    pIomCqRd->ui32CMD2Addr = (uint32_t)&IOMn(ui32Module)->CMD;
#endif
}

#if !defined(AM_PART_APOLLO4B) && !defined(AM_PART_APOLLO4P) && !defined(AM_PART_APOLLO4L)
//*****************************************************************************
//
//! @brief creates an iom read transaction
//!
//! @param pHandle      - pointer to iom driver struct
//! @param pui8Buffer   - iom will save data to this buffer
//! @param ui32Address  - the iom will read from this address
//! @param ui32NumBytes - the iom will read this many bytes
//
//*****************************************************************************
static void
create_iom_read_transaction(void *pHandle, uint8_t *pui8Buffer,
                            uint32_t ui32Address,
                            uint32_t ui32NumBytes)
{
    am_devices_iom_aps6404l_t *pIom = (am_devices_iom_aps6404l_t *)pHandle;

    uint32_t ui32Cmd;
    uint32_t ui32Inst = 0;
    uint32_t ui32Dir = 0;
    uint32_t ui32Bytes = 0;

    uint32_t Addr1 = (ui32Address & 0xFF) << 24;
    uint32_t Addr2 = (ui32Address & 0xFF00) << 8;
    uint32_t Addr3 = (ui32Address & 0xFF0000) >> 8;

    ui32Dir = AM_HAL_IOM_RX;
    ui32Inst = AM_DEVICES_SPIPSRAM_FAST_READ;
    ui32Bytes = 5;
    gIomTransReadTxn.ui32FIFOPushVal2 = 0; // dummy

    gIomTransReadTxn.ui32FIFOPushVal1 = ui32Inst | Addr1 | Addr2 | Addr3;
    gIomTransReadTxn.ui32DMATARGADDRVal = (uint32_t)pui8Buffer;

    //
    // Command to start the transfer.
    //
    ui32Cmd = build_iom_cmd(pIom->ui32CS, // ChipSelect
                       AM_HAL_IOM_TX,          // ui32Dir
                       true,           // ui32Cont
                       0,           // ui32Offset
                       0,        // ui32OffsetCnt
                       ui32Bytes);  // ui32Bytes
    gIomTransReadTxn.ui32CMD1Val = ui32Cmd;

    ui32Cmd = build_iom_cmd(pIom->ui32CS, // ChipSelect
                       ui32Dir,          // ui32Dir
                       false,           // ui32Cont
                       0,           // ui32Offset
                       0,        // ui32OffsetCnt
                       ui32NumBytes);  // ui32Bytes
    gIomTransReadTxn.ui32DMATOTCOUNTVal = ui32NumBytes;
    gIomTransReadTxn.ui32CMD2Val   = ui32Cmd;
}
//*****************************************************************************
//
//! @brief
//!
//! @param pHandle      - pointer to iom driver struct
//! @param pui8Buffer   - iom will save data to this buffer
//! @param ui32Address  - the iom will read from this address
//! @param ui32NumBytes - the iom will read this many bytes
//
//*****************************************************************************
static void
create_iom_write_transaction(void *pHandle, uint8_t *pui8Buffer,
                             uint32_t ui32Address,
                             uint32_t ui32NumBytes)
{
    am_devices_iom_aps6404l_t *pIom = (am_devices_iom_aps6404l_t *)pHandle;

    uint32_t ui32Cmd;
    uint32_t ui32Inst = 0;
    uint32_t ui32Dir = 0;
    uint32_t ui32Bytes = 0;

    uint32_t Addr1 = (ui32Address & 0xFF) << 24;
    uint32_t Addr2 = (ui32Address & 0xFF00) << 8;
    uint32_t Addr3 = (ui32Address & 0xFF0000) >> 8;

    ui32Dir = AM_HAL_IOM_TX;
    ui32Inst = AM_DEVICES_SPIPSRAM_WRITE;
    ui32Bytes = 4;

    gIomTransWriteTxn.ui32FIFOPushVal1 = ui32Inst | Addr1 | Addr2 | Addr3;
    gIomTransWriteTxn.ui32DMATARGADDRVal = (uint32_t)pui8Buffer;

    //
    // Command to start the transfer.
    //
    ui32Cmd = build_iom_cmd(pIom->ui32CS, // ChipSelect
                       ui32Dir,          // ui32Dir
                       false,           // ui32Cont
                       0,           // ui32Offset
                       0,        // ui32OffsetCnt
                       ui32NumBytes + ui32Bytes);  // ui32Bytes
    gIomTransWriteTxn.ui32DMATOTCOUNTVal = ui32NumBytes;
    gIomTransWriteTxn.ui32CMD1Val   = ui32Cmd;
}

#else
//*****************************************************************************
//
//! @brief creates an iom read transaction
//!
//! @param pHandle      - pointer to iom driver struct
//! @param pui8Buffer   - iom will save data to this buffer
//! @param ui32Address  - the iom will read from this address
//! @param ui32NumBytes - the iom will read this many bytes
//
//*****************************************************************************
static void
create_iom_read_transaction(void *pHandle,
                            uint8_t *pui8Buffer,
                            uint32_t ui32Address,
                            uint32_t ui32NumBytes)
{
    am_devices_iom_aps6404l_t *pIom = (am_devices_iom_aps6404l_t *)pHandle;

    uint32_t ui32Cmd;

    gIomTransReadTxn.ui32OFFSETHIVal = (AM_DEVICES_SPIPSRAM_FAST_READ << 24) | (ui32Address & 0x00FFFFFF);

    gIomTransReadTxn.ui32DMATARGADDRVal = (uint32_t)pui8Buffer;

    ui32Cmd = build_iom_cmd(pIom->ui32CS, // ChipSelect
                       AM_HAL_IOM_RX,          // ui32Dir
                       false,           // ui32Cont
                       0,           // ui32Offset
                       5,        // ui32OffsetCnt - dummy byte is placed in OFFSETLO to handle
                                 //                 turnaround of 8 clocks.  Value does not matter.
                       ui32NumBytes);  // ui32Bytes
    gIomTransReadTxn.ui32DMATOTCOUNTVal = ui32NumBytes;
    gIomTransReadTxn.ui32CMD2Val   = ui32Cmd;
}
//*****************************************************************************
//
//! @brief
//! @param pHandle
//! @param pui8Buffer
//! @param ui32Address
//! @param ui32NumBytes
//
//*****************************************************************************
static void
create_iom_write_transaction(void *pHandle, uint8_t *pui8Buffer,
                             uint32_t ui32Address,
                             uint32_t ui32NumBytes)
{
    am_devices_iom_aps6404l_t *pIom = (am_devices_iom_aps6404l_t *)pHandle;

    uint32_t ui32Cmd;

    gIomTransWriteTxn.ui32OFFSETHIVal = (AM_DEVICES_SPIPSRAM_WRITE << 16) | ((ui32Address & 0x00FFFF00) >> 8);
    gIomTransWriteTxn.ui32DMATARGADDRVal = (uint32_t)pui8Buffer;

    //
    // Command to start the transfer.
    //
    ui32Cmd = build_iom_cmd(pIom->ui32CS, // ChipSelect
                       AM_HAL_IOM_TX,          // ui32Dir
                       false,           // ui32Cont
                       (ui32Address & 0xFF), // ui32Offset
                       4,        // ui32OffsetCnt
                       ui32NumBytes);  // ui32Bytes
    gIomTransWriteTxn.ui32DMATOTCOUNTVal = ui32NumBytes;
    gIomTransWriteTxn.ui32CMD1Val   = ui32Cmd;
}

#endif

//*****************************************************************************
//
//Generic Command Write function.
//
//*****************************************************************************
uint32_t
am_devices_spipsram_command_write(void *pHandle,
                                  bool bHiPrio,
                                  uint32_t ui32InstrLen,
                                  uint64_t ui64Instr,
                                  uint32_t *pData,
                                  uint32_t ui32NumBytes,
                                  bool bContinue)
{
    am_hal_iom_transfer_t     Transaction;
    am_devices_iom_aps6404l_t *pIom = (am_devices_iom_aps6404l_t *)pHandle;

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
    Transaction.uPeerInfo.ui32SpiChipSelect = pIom->ui32CS;
    Transaction.bContinue       = bContinue;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    if (bHiPrio)
    {
        volatile bool bDMAComplete = false;
        //
        // Execute the transction over IOM.
        //
        if (am_hal_iom_highprio_transfer(pIom->pIomHandle, &Transaction, pfnSPI_PSRAM_Callback, (void*)&bDMAComplete))
        {
            return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
        }
        while (!bDMAComplete);
    }
    else
    {
        //
        // Execute the transction over IOM.
        //
        if (am_hal_iom_blocking_transfer(pIom->pIomHandle, &Transaction))
        {
            return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
        }
    }
    return AM_DEVICES_SPIPSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//
//*****************************************************************************
uint32_t
am_devices_spipsram_command_read(void *pHandle,
                                 bool bHiPrio,
                                 uint32_t ui32InstrLen,
                                 uint64_t ui64Instr,
                                 uint32_t *pData,
                                 uint32_t ui32NumBytes,
                                 bool bContinue)
{
    am_hal_iom_transfer_t Transaction;
    am_devices_iom_aps6404l_t *pIom = (am_devices_iom_aps6404l_t *)pHandle;

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
    Transaction.uPeerInfo.ui32SpiChipSelect = pIom->ui32CS;
    Transaction.bContinue       = bContinue;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    if ( 0 == Transaction.ui32NumBytes )
    {
        Transaction.eDirection      = AM_HAL_IOM_TX;
    }

    if (bHiPrio)
    {
        volatile bool bDMAComplete = false;
        //
        // Execute the transction over IOM.
        //
        if (am_hal_iom_highprio_transfer(pIom->pIomHandle, &Transaction, pfnSPI_PSRAM_Callback, (void*)&bDMAComplete))
        {
            return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
        }
        while (!bDMAComplete);
    }
    else
    {
        //
        // Execute the transction over IOM.
        //
        if (am_hal_iom_blocking_transfer(pIom->pIomHandle, &Transaction))
        {
            return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
        }
    }
    return AM_DEVICES_SPIPSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Reset the external psram.
//
//*****************************************************************************
uint32_t
am_devices_spipsram_reset(void *pHandle)
{
    uint32_t Dummy;
    //
    // Send the command sequence to reset the device and return status.
    //
    if (AM_HAL_STATUS_SUCCESS != am_devices_spipsram_command_write(pHandle, false, 1, AM_DEVICES_SPIPSRAM_RESET_ENABLE, &Dummy, 0, false))
    {
        return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
    }
    if (AM_HAL_STATUS_SUCCESS != am_devices_spipsram_command_write(pHandle, false, 1, AM_DEVICES_SPIPSRAM_RESET_MEMORY, &Dummy, 0, false))
    {
        return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
    }
    return AM_DEVICES_SPIPSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Reads the ID of the external psram and returns the value.
//
//*****************************************************************************
uint32_t
am_devices_spipsram_read_id(void *pHandle, uint32_t *pDeviceID)
{
    //
    // Send the command sequence to read the Device ID.
    //
    if (am_devices_spipsram_command_read(pHandle, false, 1, AM_DEVICES_SPIPSRAM_READ_ID, pDeviceID, 5, false))
    {
        return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
    }
    return AM_DEVICES_SPIPSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Initialize the spipsram driver.
//
//*****************************************************************************
uint32_t
am_devices_spipsram_init(uint32_t ui32Module,
                         am_devices_spipsram_config_t *pDevConfig,
                         void **ppHandle,
                         void **ppIomHandle)
{
    void *pIomHandle;
    uint32_t aui32Rawdata[2] = {0};
    uint32_t ui32DeviceID = 0;
    uint32_t ui32Status = AM_DEVICES_SPIPSRAM_STATUS_SUCCESS;
#ifdef APOLLO3P_EVB_CYGNUS
    uint32_t g_CS[AM_REG_IOM_NUM_MODULES] =
    {
        0,
        AM_BSP_IOM1_CS_CHNL,
        0,
        AM_BSP_IOM3_CS_CHNL,
        AM_BSP_IOM4_CS_CHNL,
        0
    };
#endif

    uint32_t      ui32Index = 0;

    // Allocate a vacant device handle
    for ( ui32Index = 0; ui32Index < AM_DEVICES_APS6404L_MAX_DEVICE_NUM; ui32Index++ )
    {
        if ( gAmAps6404l[ui32Index].bOccupied == false )
        {
            break;
        }
    }
    if (ui32Index >= AM_DEVICES_APS6404L_MAX_DEVICE_NUM)
    {
        return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
    }

    if ( (ui32Module > AM_REG_IOM_NUM_MODULES)  || (pDevConfig == NULL) )
    {
        return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
    }

    //
    // Look up the Max Transaction size to fit into 8usec for CE asserted
    //
    gAmAps6404l[ui32Index].ui32MaxTransSize = 0;
    for (uint32_t i = 0; i < (sizeof(g_SPI_SpeedMax) / sizeof(g_SPI_SpeedMax[0])); i++)
    {
        if (g_SPI_SpeedMax[i].MHz == pDevConfig->ui32ClockFreq)
        {
            gAmAps6404l[ui32Index].ui32MaxTransSize = g_SPI_SpeedMax[i].MaxSize;
            break;
        }
    }
    if ( 0 == gAmAps6404l[ui32Index].ui32MaxTransSize )  // Return an error if Max Transaction size not found.
    {
        return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
    }

    //
    // Configure the IOM pins.
    //
    am_bsp_iom_pins_enable(ui32Module, AM_HAL_IOM_SPI_MODE);

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

    //
    // Initialize the IOM instance.
    // if the iom is already inited, skip these steps
    //
    void *extIomHandle = *ppIomHandle;
    bool already_inited = false;

    ui32Status = am_hal_iom_initialize(ui32Module, &pIomHandle);
    if (ui32Status == AM_HAL_STATUS_INVALID_OPERATION)
    {
        //
        // already inited
        //
        already_inited = true;
        pIomHandle = extIomHandle;
    }
    else if (ui32Status)
    {
        //
        // an error occurred
        //
        return ui32Status;
    }

    // populate and assign local struct (handle)
    gAmAps6404l[ui32Index].ui32Module    = ui32Module;
#ifdef APOLLO3P_EVB_CYGNUS
    gAmAps6404l[ui32Index].ui32CS        = g_CS[ui32Module];
#else
    gAmAps6404l[ui32Index].ui32CS        = pDevConfig->ui32ChipSelectNum;
#endif
    gAmAps6404l[ui32Index].pIomHandle    = pIomHandle;

    *ppHandle    = (void *) &gAmAps6404l[ui32Index];

    gAmAps6404l[ui32Index].sSpiPsramCfg.ui32NBTxnBufLength   = pDevConfig->ui32NBTxnBufLength;
    gAmAps6404l[ui32Index].sSpiPsramCfg.pNBTxnBuf            = pDevConfig->pNBTxnBuf;
    gAmAps6404l[ui32Index].sSpiPsramCfg.ui32ClockFreq        = pDevConfig->ui32ClockFreq;
    gAmAps6404l[ui32Index].sSpiPsramCfg.eInterfaceMode       = AM_HAL_IOM_SPI_MODE;
    gAmAps6404l[ui32Index].sSpiPsramCfg.eSpiMode             = AM_HAL_IOM_SPI_MODE_0;


    if ( !already_inited )
    {
        //
        // Enable power to the IOM instance.
        //
        *ppIomHandle = pIomHandle;

        ui32Status = am_hal_iom_power_ctrl(pIomHandle, AM_HAL_SYSCTRL_WAKE, g_IOMSave[ui32Module]);
        if (ui32Status)
        {
            return ui32Status;
        }

        //
        // Configure the IOM for Serial operation during initialization.
        //
        ui32Status = am_hal_iom_configure(pIomHandle, &gAmAps6404l[ui32Index].sSpiPsramCfg);
        if (ui32Status)
        {
            return ui32Status;
        }

        //
        // Enable the IOM.
        //
        ui32Status = am_hal_iom_enable(pIomHandle);
        if (ui32Status)
        {
            return ui32Status;
        }
    }
    am_util_delay_us(150);

    if (AM_DEVICES_SPIPSRAM_STATUS_SUCCESS != am_devices_spipsram_reset((void *) &gAmAps6404l[ui32Index]))
    {
        return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
    }

    //
    // test the setup and the psram by reading device id
    //
    ui32Status   = am_devices_spipsram_read_id((void *) &gAmAps6404l[ui32Index], aui32Rawdata);
    ui32DeviceID = ((aui32Rawdata[0] & 0xFF000000) >> 24) | ((aui32Rawdata[1] & 0xFF) << 8);
    //am_util_stdio_printf("PSRAM ID is 0x%x\n", ui32DeviceID);
    if ((AM_DEVICES_SPIPSRAM_KGD_PASS == ui32DeviceID) &&
        (AM_HAL_STATUS_SUCCESS == ui32Status))
    {
        gAmAps6404l[ui32Index].bOccupied = true;
        iom_init_cq_element(ui32Module);
        return AM_DEVICES_SPIPSRAM_STATUS_SUCCESS;
    }

    return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
}


//*****************************************************************************
//
//
//
//*****************************************************************************
uint32_t
am_devices_spipsram_init_no_check(uint32_t ui32Module,
                                  am_devices_spipsram_config_t *pDevConfig,
                                  void **ppHandle,
                                  void **ppIomHandle)
{
    void *pIomHandle;
    am_hal_iom_config_t     stIOMPSRAMSettings;
    uint32_t g_CS[AM_REG_IOM_NUM_MODULES] =
#ifdef APOLLO3P_EVB_CYGNUS
    {
        0,
        AM_BSP_IOM1_CS_CHNL,
        0,
        AM_BSP_IOM3_CS_CHNL,
        AM_BSP_IOM4_CS_CHNL,
        0
    };
#elif defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    { 0, 0, 0, 0, 0, 0, 0, 0 };
#endif


    uint32_t      ui32Index = 0;

    // Allocate a vacant device handle
    for ( ui32Index = 0; ui32Index < AM_DEVICES_APS6404L_MAX_DEVICE_NUM; ui32Index++ )
    {
        if ( gAmAps6404l[ui32Index].bOccupied == false )
        {
            break;
        }
    }
    if ( ui32Index == AM_DEVICES_APS6404L_MAX_DEVICE_NUM )
    {
        return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
    }

    if ( (ui32Module > AM_REG_IOM_NUM_MODULES)  || (pDevConfig == NULL) )
    {
        return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
    }

    stIOMPSRAMSettings = g_sSpiPsramCfg;
    stIOMPSRAMSettings.ui32NBTxnBufLength = pDevConfig->ui32NBTxnBufLength;
    stIOMPSRAMSettings.pNBTxnBuf = pDevConfig->pNBTxnBuf;
    stIOMPSRAMSettings.ui32ClockFreq = pDevConfig->ui32ClockFreq;

    //
    // Look up the Max Transaction size to fit into 8usec for CE asserted
    //
    gAmAps6404l[ui32Index].ui32MaxTransSize = 0;
    for (uint32_t i = 0; i < (sizeof(g_SPI_SpeedMax) / sizeof(g_SPI_SpeedMax[0])); i++)
    {
        if (g_SPI_SpeedMax[i].MHz == stIOMPSRAMSettings.ui32ClockFreq)
        {
            gAmAps6404l[ui32Index].ui32MaxTransSize = g_SPI_SpeedMax[i].MaxSize;
            break;
        }
    }
    if ( 0 == gAmAps6404l[ui32Index].ui32MaxTransSize )  // Return an error if Max Transaction size not found.
    {
        return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
    }

    //
    // Configure the IOM pins.
    //
    am_bsp_iom_pins_enable(ui32Module, AM_HAL_IOM_SPI_MODE);

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

    //
    // Initialize the IOM instance.
    // Enable power to the IOM instance.
    // Configure the IOM for Serial operation during initialization.
    // Enable the IOM.
    //
    if (am_hal_iom_initialize(ui32Module, &pIomHandle) ||
        am_hal_iom_power_ctrl(pIomHandle, AM_HAL_SYSCTRL_WAKE, g_IOMSave[ui32Module] == true ? true : false) ||
        am_hal_iom_configure(pIomHandle, &stIOMPSRAMSettings) ||
        am_hal_iom_enable(pIomHandle))
    {
        return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
    }
    else
    {
        am_util_delay_us(150);
        gAmAps6404l[ui32Index].ui32Module = ui32Module;
        gAmAps6404l[ui32Index].ui32CS = g_CS[ui32Module];
        *ppIomHandle = gAmAps6404l[ui32Index].pIomHandle = pIomHandle;
        *ppHandle = (void *)&gAmAps6404l[ui32Index];

        if (AM_DEVICES_SPIPSRAM_STATUS_SUCCESS != am_devices_spipsram_reset((void*)&gAmAps6404l[ui32Index]))
        {
            return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
        }

        gAmAps6404l[ui32Index].bOccupied = true;
        iom_init_cq_element(ui32Module);
        return AM_DEVICES_SPIPSRAM_STATUS_SUCCESS;
    }
}

//*****************************************************************************
//
// DeInitialize the spipsram driver.
//
//*****************************************************************************
uint32_t
am_devices_spipsram_term(void *pHandle)
{
    am_devices_iom_aps6404l_t *pIom = (am_devices_iom_aps6404l_t *)pHandle;

    if ( pIom->ui32Module > AM_REG_IOM_NUM_MODULES )
    {
        return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
    }

    // Disable the pins
    am_bsp_iom_pins_disable(pIom->ui32Module, AM_HAL_IOM_SPI_MODE);

    //
    // Disable the IOM.
    //
    am_hal_iom_disable(pIom->pIomHandle);

    //
    // Disable power to and uninitialize the IOM instance.
    //
    am_hal_iom_power_ctrl(pIom->pIomHandle, AM_HAL_SYSCTRL_DEEPSLEEP, true);
    g_IOMSave[pIom->ui32Module] = true;

    am_hal_iom_uninitialize(pIom->pIomHandle);

    // Free this device handle
    pIom->bOccupied = false;

    //
    // Return the status.
    //
    return AM_DEVICES_SPIPSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Programs the given range of psram addresses.
//
//*****************************************************************************
uint32_t
am_devices_spipsram_blocking_write(void *pHandle,
                                   uint8_t *pui8TxBuffer,
                                   uint32_t ui32WriteAddress,
                                   uint32_t ui32NumBytes)
{
    am_hal_iom_transfer_t Transaction;
    am_devices_iom_aps6404l_t *pIom = (am_devices_iom_aps6404l_t *)pHandle;

    while (ui32NumBytes)
    {
        //
        // Create the transaction.
        //
        Transaction.ui32InstrLen    = 1;
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
        Transaction.ui64Instr       = AM_DEVICES_SPIPSRAM_WRITE;
#else
        Transaction.ui32Instr       = AM_DEVICES_SPIPSRAM_WRITE;
#endif
        Transaction.eDirection      = AM_HAL_IOM_TX;
        Transaction.ui32NumBytes    = 0;
        Transaction.pui32TxBuffer   = (uint32_t *)pui8TxBuffer;
        Transaction.uPeerInfo.ui32SpiChipSelect = pIom->ui32CS;
        Transaction.bContinue       = true;
        Transaction.ui8RepeatCount  = 0;
        Transaction.ui32PauseCondition = 0;
        Transaction.ui32StatusSetClr = 0;

        //
        // Execute the transction over IOM.
        //
        if (am_hal_iom_blocking_transfer(pIom->pIomHandle, &Transaction))
        {
            return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
        }

        uint32_t maxSize = AM_DEVICES_SPIPSRAM_PAGE_SIZE - (ui32WriteAddress & (AM_DEVICES_SPIPSRAM_PAGE_SIZE - 1));
        uint32_t limit = (maxSize > pIom->ui32MaxTransSize) ? pIom->ui32MaxTransSize : maxSize;
        uint32_t size = (ui32NumBytes > limit) ? limit : ui32NumBytes;

        //
        // Create the transaction.
        //
        Transaction.ui32InstrLen    = 3;
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
        Transaction.ui64Instr       = ui32WriteAddress & 0x00FFFFFF;
#else
        Transaction.ui32Instr       = ui32WriteAddress & 0x00FFFFFF;
#endif
        Transaction.eDirection      = AM_HAL_IOM_TX;
        Transaction.ui32NumBytes    = size;
        Transaction.bContinue       = false;
        //
        // Execute the transction over IOM.
        //
        if (am_hal_iom_blocking_transfer(pIom->pIomHandle, &Transaction))
        {
            return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
        }
        ui32NumBytes -= Transaction.ui32NumBytes;
        pui8TxBuffer += Transaction.ui32NumBytes;
        ui32WriteAddress += Transaction.ui32NumBytes;
    }
    return AM_DEVICES_SPIPSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Reads the contents of the fram into a buffer.
//
//*****************************************************************************
uint32_t
am_devices_spipsram_blocking_read(void *pHandle,
                                  uint8_t *pui8RxBuffer,
                                  uint32_t ui32ReadAddress,
                                  uint32_t ui32NumBytes)
{
    am_hal_iom_transfer_t Transaction;
    am_devices_iom_aps6404l_t *pIom = (am_devices_iom_aps6404l_t *)pHandle;

    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition       = 0;
    Transaction.ui32StatusSetClr = 0;

    while (ui32NumBytes)
    {
        // Corvette RX HW bug, CORVETTE-874. 0-byte xfer must be TX.
        // This HW bug should be covered in the HAL, so test the HAL fix here by
        // attempting to do a 0-byte RX transfer.
        Transaction.eDirection      = AM_HAL_IOM_TX;
        Transaction.ui32InstrLen    = 1;
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
        Transaction.ui64Instr       = AM_DEVICES_SPIPSRAM_READ;
#else
        Transaction.ui32Instr       = AM_DEVICES_SPIPSRAM_READ;
#endif
        Transaction.ui32NumBytes    = 0;
        Transaction.pui32TxBuffer   = (uint32_t *)pui8RxBuffer;
        Transaction.uPeerInfo.ui32SpiChipSelect = pIom->ui32CS;
        Transaction.bContinue       = true;

        //
        // Start the transaction.
        //
        if (am_hal_iom_blocking_transfer(pIom->pIomHandle, &Transaction))
        {
            return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
        }

        uint32_t maxSize = AM_DEVICES_SPIPSRAM_PAGE_SIZE - (ui32ReadAddress & (AM_DEVICES_SPIPSRAM_PAGE_SIZE - 1));
        uint32_t limit = (maxSize > pIom->ui32MaxTransSize) ? pIom->ui32MaxTransSize : maxSize;
        uint32_t size = (ui32NumBytes > limit) ? limit : ui32NumBytes;

        //
        // Set up the IOM transaction to write to the device with offset address.
        //
        Transaction.eDirection      = AM_HAL_IOM_RX;
        Transaction.ui32InstrLen    = 3;
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
        Transaction.ui64Instr       = ui32ReadAddress & 0x00FFFFFF;
#else
        Transaction.ui32Instr       = ui32ReadAddress & 0x00FFFFFF;
#endif
        Transaction.ui32NumBytes    = size;
        Transaction.pui32RxBuffer   = (uint32_t *)pui8RxBuffer;
        Transaction.bContinue       = false;

        //
        // Start the transaction.
        //
        if (am_hal_iom_blocking_transfer(pIom->pIomHandle, &Transaction))
        {
            return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
        }
        ui32NumBytes -= Transaction.ui32NumBytes;
        pui8RxBuffer += Transaction.ui32NumBytes;
        ui32ReadAddress += Transaction.ui32NumBytes;
    }
    return AM_DEVICES_SPIPSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Starts a non blocking read or write using PSRAM
//!
//! This function reads the external flash at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//!
//! @param pHandle
//! @param bWrite
//! @param pui8Buffer
//! @param ui32Address
//! @param ui32NumBytes
//! @param ui32PauseCondition
//! @param ui32StatusSetClr
//! @param pfnCallback
//! @param pCallbackCtxt
//! @return
//
//*****************************************************************************
static uint32_t
spi_psram_nonblocking_transfer(void *pHandle,
                               bool bWrite,
                               uint8_t *pui8Buffer,
                               uint32_t ui32Address,
                               uint32_t ui32NumBytes,
                               uint32_t ui32PauseCondition,
                               uint32_t ui32StatusSetClr,
                               am_hal_iom_callback_t pfnCallback,
                               void *pCallbackCtxt)
{
    bool bLast = false;
    am_hal_iom_cq_raw_t rawIomCfg;
    am_devices_iom_aps6404l_t *pIom = (am_devices_iom_aps6404l_t *)pHandle;
    uint32_t size;


    rawIomCfg.ui32PauseCondition = ui32PauseCondition;
    rawIomCfg.ui32StatusSetClr = 0;

    while (ui32NumBytes)
    {
        uint32_t maxSize = AM_DEVICES_SPIPSRAM_PAGE_SIZE - (ui32Address & (AM_DEVICES_SPIPSRAM_PAGE_SIZE - 1));
        if (bWrite)
        {
            uint32_t limit = (maxSize > pIom->ui32MaxTransSize) ? pIom->ui32MaxTransSize : maxSize;
            size = (ui32NumBytes > limit) ? limit : ui32NumBytes;
            create_iom_write_transaction(pHandle, pui8Buffer, ui32Address, size);
            rawIomCfg.pCQEntry = (am_hal_cmdq_entry_t*)&gIomTransWriteTxn;
            rawIomCfg.numEntries = sizeof(gIomTransWriteTxn) / 8;
        }
        else
        {
            uint32_t limit = (maxSize > pIom->ui32MaxTransSize - 3) ? (pIom->ui32MaxTransSize - 3) : maxSize; // calibrate for the gap bewteen two transactions
            size = (ui32NumBytes > limit) ? limit : ui32NumBytes;
            create_iom_read_transaction(pHandle, pui8Buffer, ui32Address, size);
            rawIomCfg.pCQEntry = (am_hal_cmdq_entry_t*)&gIomTransReadTxn;
            rawIomCfg.numEntries = sizeof(gIomTransReadTxn) / 8;
        }

        bLast = (size == ui32NumBytes);
        if (bLast)
        {
            rawIomCfg.ui32StatusSetClr = ui32StatusSetClr;
        }
        rawIomCfg.pfnCallback = bLast ? pfnCallback : NULL;
        rawIomCfg.pCallbackCtxt = bLast ? pCallbackCtxt : NULL;
        rawIomCfg.pJmpAddr = 0;
        if ( am_hal_iom_control(pIom->pIomHandle, AM_HAL_IOM_REQ_CQ_RAW, &rawIomCfg) )
        {
            return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
        }
        ui32NumBytes -= size;
        pui8Buffer += size;
        ui32Address += size;
        rawIomCfg.ui32PauseCondition = 0;
    }
    return AM_DEVICES_SPIPSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Reads the contents of the external psram into a buffer.
//
//*****************************************************************************
uint32_t
am_devices_spipsram_read_adv(void *pHandle,
                             uint8_t *pui8RxBuffer,
                             uint32_t ui32ReadAddress,
                             uint32_t ui32NumBytes,
                             uint32_t ui32PauseCondition,
                             uint32_t ui32StatusSetClr,
                             am_hal_iom_callback_t pfnCallback,
                             void *pCallbackCtxt)
{
    uint32_t                      ui32Status;

    ui32Status = spi_psram_nonblocking_transfer(pHandle, false,
                          pui8RxBuffer,
                          ui32ReadAddress,
                          ui32NumBytes,
                          ui32PauseCondition,
                          ui32StatusSetClr,
                          pfnCallback,
                          pCallbackCtxt);

    // Check the transaction status.
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_SPIPSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//
//*****************************************************************************
uint32_t
am_devices_spipsram_write_adv(void *pHandle, uint8_t *pui8TxBuffer,
                              uint32_t ui32WriteAddress,
                              uint32_t ui32NumBytes,
                              uint32_t ui32PauseCondition,
                              uint32_t ui32StatusSetClr,
                              am_hal_iom_callback_t pfnCallback,
                              void *pCallbackCtxt)
{
    uint32_t                      ui32Status;

    ui32Status = spi_psram_nonblocking_transfer(pHandle, true,
                        pui8TxBuffer,
                        ui32WriteAddress,
                        ui32NumBytes,
                        ui32PauseCondition,
                        ui32StatusSetClr,
                        pfnCallback,
                        pCallbackCtxt);

    // Check the transaction status.
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_SPIPSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Reads the contents of the external psram into a buffer.
//
//*****************************************************************************
uint32_t
am_devices_spipsram_read(void *pHandle, uint8_t *pui8RxBuffer,
                         uint32_t ui32ReadAddress,
                         uint32_t ui32NumBytes,
                         bool bWaitForCompletion)
{
    uint32_t                      ui32Status;

    if (bWaitForCompletion)
    {
        // Start the transaction.
        volatile bool bDMAComplete = false;
        ui32Status = spi_psram_nonblocking_transfer(pHandle, false,
                            pui8RxBuffer,
                            ui32ReadAddress,
                            ui32NumBytes,
                            0,
                            0,
                            pfnSPI_PSRAM_Callback,
                            (void*)&bDMAComplete);

        // Check the transaction status.
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
        }

        // Wait for DMA Complete or Timeout
        for (uint32_t i = 0; i < AM_DEVICES_SPIPSRAM_TIMEOUT; i++)
        {
            if (bDMAComplete)
            {
                break;
            }
            //
            // Call the BOOTROM cycle function to delay for about 1 microsecond.
            //
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
            am_hal_flash_delay( FLASH_CYCLES_US(1) );
#elif defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
            am_hal_delay_us(1);
#else
#warning "Driver is only defined for APOLLO3, APOLLO3P, and APOLLO4!!!"
#endif
        }

        // Check the status.
        if (!bDMAComplete)
        {
            return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
        }
    }
    else
    {
        // Check the transaction status.
        ui32Status = spi_psram_nonblocking_transfer(pHandle, false,
                            pui8RxBuffer,
                            ui32ReadAddress,
                            ui32NumBytes,
                            0,
                            0,
                            NULL,
                            NULL);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
        }
    }

    //
    // Return the status.
    //
    return AM_DEVICES_SPIPSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Programs the given range of psram addresses.
//
//*****************************************************************************
uint32_t
am_devices_spipsram_write(void *pHandle,
                          uint8_t *pui8TxBuffer,
                          uint32_t ui32WriteAddress,
                          uint32_t ui32NumBytes,
                          bool bWaitForCompletion)
{
    uint32_t              ui32Status;

    if (bWaitForCompletion)
    {
        // Start the transaction.
        volatile bool bDMAComplete = false;
        ui32Status = spi_psram_nonblocking_transfer(pHandle,
                            true,
                            pui8TxBuffer,
                            ui32WriteAddress,
                            ui32NumBytes,
                            0,
                            0,
                            pfnSPI_PSRAM_Callback,
                            (void*)&bDMAComplete);

        // Check the transaction status.
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
        }

        // Wait for DMA Complete or Timeout
        for (uint32_t i = 0; i < AM_DEVICES_SPIPSRAM_TIMEOUT; i++)
        {
            if (bDMAComplete)
            {
                break;
            }
            //
            // Call the BOOTROM cycle function to delay for about 1 microsecond.
            //
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
            am_hal_flash_delay( FLASH_CYCLES_US(1) );
#elif defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
            am_hal_delay_us(1);
#else
#warning "Driver is only defined for APOLLO3, APOLLO3P, and APOLLO4!!!"
#endif
        }

        //
        // Check the status.
        //
        if (!bDMAComplete)
        {
            return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
        }
    }
    else // bWaitForCompletion
    {
        //
        // do not wait for completion
        // Check the transaction status.
        //
        ui32Status = spi_psram_nonblocking_transfer(pHandle,
                            true,
                            pui8TxBuffer,
                            ui32WriteAddress,
                            ui32NumBytes,
                            0,
                            0,
                            NULL,
                            NULL);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
        }
    } // bWaitForCompletion

    //
    // Return the status.
    //
    return AM_DEVICES_SPIPSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Reads the contents of the external psram into a buffer
//
//*****************************************************************************
uint32_t
am_devices_spipsram_nonblocking_read(void *pHandle,
                                     uint8_t *pui8RxBuffer,
                                     uint32_t ui32ReadAddress,
                                     uint32_t ui32NumBytes,
                                     am_hal_iom_callback_t pfnCallback,
                                     void *pCallbackCtxt)
{
    uint32_t                      ui32Status;

    //
    // Check the transaction status.
    //
    ui32Status = spi_psram_nonblocking_transfer(pHandle, false,
                        pui8RxBuffer,
                        ui32ReadAddress,
                        ui32NumBytes,
                        0,
                        0,
                        pfnCallback,
                        pCallbackCtxt);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_SPIPSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//
//
//*****************************************************************************
uint32_t
am_devices_spipsram_nonblocking_write(void *pHandle,
                                      uint8_t *pui8TxBuffer,
                                      uint32_t ui32WriteAddress,
                                      uint32_t ui32NumBytes,
                                      am_hal_iom_callback_t pfnCallback,
                                      void *pCallbackCtxt)
{
    uint32_t                      ui32Status;

    //
    // Check the transaction status.
    //
    ui32Status = spi_psram_nonblocking_transfer(pHandle, true,
                        pui8TxBuffer,
                        ui32WriteAddress,
                        ui32NumBytes,
                        0,
                        0,
                        pfnCallback,
                        pCallbackCtxt);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_SPIPSRAM_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_SPIPSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

