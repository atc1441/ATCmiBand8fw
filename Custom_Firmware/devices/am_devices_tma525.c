//*****************************************************************************
//
//! @file am_devices_tma525.c
//!
//! @brief General I2C touch driver.
//!
//! @addtogroup tma525 TMA525 - I2C Touch Driver
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

#include "am_mcu_apollo.h"
#include "am_devices_tma525.h"
#include "am_bsp.h"
#include "string.h"
#include "am_util_delay.h"

//*****************************************************************************
//
// Local definitions.
//
//*****************************************************************************
am_hal_iom_config_t     g_sI2cTms525Cfg =
{
    .eInterfaceMode       = AM_HAL_IOM_I2C_MODE,
    .ui32ClockFreq        = AM_HAL_IOM_400KHZ,
    .ui32NBTxnBufLength   = 0,
    .pNBTxnBuf = NULL,
};

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
static am_devices_iom_tma525_t gAmTma525[1] = {0};
static void *g_tma525_IOMHandle;

//*****************************************************************************
//
// Local functions public.
//
//*****************************************************************************
static uint32_t tma525_i2c_read(uint32_t ui32BusAddress, uint32_t *pBuf, uint32_t size);
static uint32_t tma525_i2c_write(uint32_t ui32BusAddress, uint32_t *pBuf, uint32_t size);
static uint32_t tma525_master_xfer(struct rt_i2c_msg          *msgs, unsigned int num);

//*****************************************************************************
//
//! @brief
//! @param addr
//! @param buffer
//! @param length
//! @return
//
//*****************************************************************************
static uint32_t
tma525_read(uint16_t addr, uint8_t *buffer, unsigned long length)
{
    int ret;

    if (buffer == NULL)
    {
        return -1;
    }

    struct rt_i2c_msg msgs[] =
    {
        {
            .addr   = TMA525_SLAVE_ADDR,
            .flags  = AM_DEVICES_TMA525_I2C_WR,
            .len    = sizeof(addr),
            .buf    = (uint8_t *)&addr,
        },
        {
            .addr   = TMA525_SLAVE_ADDR,
            .flags  = AM_DEVICES_TMA525_I2C_RD,
            .len    = length,
            .buf    = buffer,
        },
    };

    ret = tma525_master_xfer(msgs, 2);

    if ( ret == (msgs[0].len - 1 + msgs[1].len) )
    {
        return AM_DEVICES_TMA525_STATUS_SUCCESS;
    }

    return AM_DEVICES_TMA525_STATUS_ERROR;
}

//*****************************************************************************
//
//! @brief
//! @param addr
//! @param buffer
//! @param length
//! @return
//
//*****************************************************************************
static uint32_t
tma525_write(rt_uint16_t addr, rt_uint8_t *buffer, rt_size_t length)
{
    if (buffer == NULL)
    {
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }

    uint8_t send_buffer[sizeof(rt_uint16_t) + 32 + 1];
    send_buffer[0] = (addr >> 0) & 0xff;  // reg addr
    send_buffer[1] = (addr >> 8) & 0xff;
    memcpy(send_buffer + sizeof(rt_uint16_t), buffer, length);

    struct rt_i2c_msg msgs[] =
    {
        {
            .addr   = TMA525_SLAVE_ADDR,
            .flags  = AM_DEVICES_TMA525_I2C_WR,
            .len    = length + sizeof(addr),
            .buf    = send_buffer,
        }
    };

    if (tma525_master_xfer(msgs, 1) == msgs->len - 1)
    {
        return AM_DEVICES_TMA525_STATUS_SUCCESS;
    }
    else
    {
        // TOUCH_LOG("tma525_write rt_i2c_transfer error!\r\n");
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }
}

//*****************************************************************************
//
//! @brief
//! @param addr
//! @param data
//! @return
//
//*****************************************************************************
static uint32_t
tma525_write_reg(rt_uint16_t addr, rt_uint16_t data)
{
    if (tma525_write(addr, (uint8_t *)&data, sizeof(addr)) == AM_DEVICES_TMA525_STATUS_SUCCESS)
    {
        return AM_DEVICES_TMA525_STATUS_SUCCESS;
    }
    else
    {
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }
}

//*****************************************************************************
//
//! @brief
//! @param addr
//! @param data
//! @return
//
//*****************************************************************************
static uint32_t
tma525_read_reg(rt_uint16_t addr, rt_uint16_t *data)
{
    if (tma525_read(addr, (uint8_t *)data, 2) == AM_DEVICES_TMA525_STATUS_SUCCESS)
    {
        return AM_DEVICES_TMA525_STATUS_SUCCESS;
    }
    else
    {
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }
}

//*****************************************************************************
//
//! @brief
//! @param cmd
//! @return
//
//*****************************************************************************
static uint32_t
tma525_write_cmd(rt_uint16_t cmd)
{
    uint16_t data = 1;
    return tma525_write_reg(cmd, data);
}

//*****************************************************************************
//
//! @brief
//! @return
//
//*****************************************************************************
static uint32_t
tma525_power_sequence(void)
{
    uint16_t chip_code = 0;

    if (tma525_write_reg( ZINITIX_VENDOR_REG, 0x0001) != AM_DEVICES_TMA525_STATUS_SUCCESS)
    {
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }

    am_util_delay_us(10);

    if (tma525_read(CHIP_ID_REG, (uint8_t *)&chip_code, 2) != AM_DEVICES_TMA525_STATUS_SUCCESS)
    {
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }

    am_util_delay_us(10);

    if (tma525_write_cmd(ZINITIX_INTN_CLEAR_CMD) != AM_DEVICES_TMA525_STATUS_SUCCESS)
    {
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }

    am_util_delay_us(10);

    if (tma525_write_reg(ZINITIX_NVM_REG, 0x0001) != AM_DEVICES_TMA525_STATUS_SUCCESS)
    {
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }

    am_util_delay_us(50);

    if (tma525_write_reg(ZINITIX_PROGRAM_START_CMD, 0x0001) != AM_DEVICES_TMA525_STATUS_SUCCESS)
    {
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }

    am_util_delay_us(500);
    return AM_DEVICES_TMA525_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief
//! @return
//
//*****************************************************************************
static uint32_t
tma525_reg_init(void)
{
    uint8_t i;
    uint16_t eeprom_info;

    for (i = 0; i < AM_DEVICES_TMA525_SWRESET_COUNT; i++)
    {
        if ( tma525_write_cmd( ZINITIX_SWRESET_CMD) == 0 )
        {
            // return 0 mean write success then break
            break;
        }

        am_util_delay_us(10);
    }

    uint16_t reg_val = 0;
    zinitix_bit_set(reg_val, BIT_PT_CNT_CHANGE);
    zinitix_bit_set(reg_val, BIT_DOWN);
    zinitix_bit_set(reg_val, BIT_MOVE);
    zinitix_bit_set(reg_val, BIT_UP);
    zinitix_bit_set(reg_val, BIT_PALM);
    zinitix_bit_set(reg_val, BIT_PT_EXIST);

    if (tma525_read_reg(ZINITIX_EEPROM_INFO, &eeprom_info) != AM_DEVICES_TMA525_STATUS_SUCCESS)
    {
        // TOUCH_LOG("fail to read eeprom info");
    }

    tma525_write_reg(ZINITIX_SUPPORTED_FINGER_NUM, MAX_SUPPORTED_FINGER_NUM);
    tma525_write_reg(ZINITIX_X_RESOLUTION, AM_DEVICES_TMA525_DISP_RESX);
    tma525_write_reg(ZINITIX_Y_RESOLUTION, AM_DEVICES_TMA525_DISP_RESY);

    tma525_write_reg(ZINITIX_INITIAL_TOUCH_MODE, TOUCH_POINT_MODE);
    tma525_write_reg(ZINITIX_TOUCH_MODE, TOUCH_POINT_MODE);

    tma525_write_reg(ZINITIX_INT_ENABLE_FLAG, reg_val);

    for (i = 0; i < AM_DEVICES_TMA525_SWRESET_COUNT; i++)
    {
        tma525_write_cmd( ZINITIX_CLEAR_INT_STATUS_CMD);
        am_util_delay_us(10);
    }

    return AM_DEVICES_TMA525_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief
//! @param ui32BusAddress
//! @param pBuf
//! @param size
//! @return
//
//*****************************************************************************
static uint32_t
tma525_i2c_read(uint32_t ui32BusAddress, uint32_t *pBuf, uint32_t size)
{
    am_hal_iom_transfer_t       Transaction;

    Transaction.ui8Priority     = 1;
    Transaction.ui32InstrLen    = 0;
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    Transaction.ui64Instr       = 0;
#else
    Transaction.ui32Instr       = 0;
#endif
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes    = size;
    Transaction.pui32RxBuffer   = pBuf;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    Transaction.uPeerInfo.ui32I2CDevAddr = ui32BusAddress;
    if (am_hal_iom_blocking_transfer(g_tma525_IOMHandle, &Transaction) != AM_HAL_STATUS_SUCCESS)
    {
         return AM_DEVICES_TMA525_STATUS_ERROR;
    }

    return AM_DEVICES_TMA525_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief
//! @param ui32BusAddress
//! @param pBuf
//! @param size
//! @return
//
//*****************************************************************************
static uint32_t
tma525_i2c_write(uint32_t ui32BusAddress, uint32_t *pBuf, uint32_t size)
{
    am_hal_iom_transfer_t       Transaction;

    Transaction.ui8Priority     = 1;
    Transaction.ui32InstrLen    = 0;
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    Transaction.ui64Instr       = 0;
#else
    Transaction.ui32Instr       = 0;    //IOSOFFSET_WRITE_CMD;
#endif
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes    = size;
    Transaction.pui32TxBuffer   = pBuf;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    Transaction.uPeerInfo.ui32I2CDevAddr = ui32BusAddress;
    if (am_hal_iom_blocking_transfer(g_tma525_IOMHandle, &Transaction) != AM_HAL_STATUS_SUCCESS)
    {
         return AM_DEVICES_TMA525_STATUS_ERROR;
    }

    return AM_DEVICES_TMA525_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief
//! @param msgs
//! @param num
//! @return
//
//*****************************************************************************
static uint32_t
tma525_master_xfer(struct rt_i2c_msg          *msgs, rt_uint32_t num)
{
    struct rt_i2c_msg *msg;
    int i;
    rt_uint32_t msg_len = 0;

    for (i = 0; i < num; i++)
    {
        msg = &msgs[i];

        if (msg->flags == AM_DEVICES_TMA525_I2C_RD)
        {
            tma525_i2c_read(msg->addr, (uint32_t *)msg->buf, msg->len);
            msg_len += msg->len;
        }
        else if ( msg->flags == AM_DEVICES_TMA525_I2C_WR )
        {
            tma525_i2c_write(msg->addr, (uint32_t *)msg->buf, msg->len);
            msg_len += (msg->len - 1);
        }
    }

    return msg_len;
}

// ****************************************************************************
//
//! @brief  init for the tma525
//
// ****************************************************************************
static void
tma525_finish_init(void)
{
    tma525_power_sequence();

    uint16_t eeprom_info;

    if (tma525_read_reg(ZINITIX_EEPROM_INFO, &eeprom_info) != AM_DEVICES_TMA525_STATUS_SUCCESS)
    {
        // TOUCH_LOG("fail to read eeprom info");
    }
    else
    {
        // TOUCH_LOG("eeprom_info = 0x%08x\n", eeprom_info);
    }

    for ( int i = 0; i < AM_DEVICES_TMA525_SWRESET_COUNT; i++ )
    {
        if ( tma525_write_cmd(ZINITIX_SWRESET_CMD) == 0 )
        {
            // return 0 mean write success then break
            break;
        }

        am_util_delay_us(AM_DEVICES_TMA525_SWRESET_DELAY_US);
    }

    tma525_write_reg(ZINITIX_INT_ENABLE_FLAG, 0);

    tma525_reg_init();
}

// ****************************************************************************
//
//! @brief
//! @param p
//! @return
//
// ****************************************************************************
static uint16_t
get_unaligned_le16(const void *p)
{
    const uint8_t *_p = p;
    return _p[0] | _p[1] << 8;
}

// ****************************************************************************
//
//  request data from tma525
//
// ****************************************************************************
uint32_t
am_devices_tma525_data_read(uint8_t *pui8RxBuffer, uint32_t RxNumBytes)
{
    am_hal_iom_transfer_t Transaction;

    Transaction.ui8Priority     = 1;        // High priority for now.
    Transaction.uPeerInfo.ui32I2CDevAddr = 0x38;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32InstrLen    = 1;
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    Transaction.ui64Instr       = 0;
#else
    Transaction.ui32Instr       = 0;
#endif
    Transaction.ui32NumBytes    = RxNumBytes;   //8;
    // Transaction.pui32TxBuffer   = (uint32_t *)pui8TxBuffer;
    Transaction.pui32RxBuffer    = (uint32_t *)pui8RxBuffer;
    // Transaction.pui32RxBuffer    = (uint32_t *)RxBuffer;
    //Transaction.bContinue       = true;
    Transaction.ui32PauseCondition = 0;
    // Now set the post processing condition
    Transaction.ui32StatusSetClr = 0;

    if (am_hal_iom_blocking_transfer(g_tma525_IOMHandle, &Transaction) != AM_HAL_STATUS_SUCCESS)
    {
         return AM_DEVICES_TMA525_STATUS_ERROR;
    }

    return AM_DEVICES_TMA525_STATUS_SUCCESS;
}

// ****************************************************************************
//
//  Get the actual touch x coordinate and y coordinate
//
// ****************************************************************************
uint32_t
am_devices_tma525_get_point(uint16_t *x, uint16_t *y, bool *touch_released)
{
    uint32_t size;
    uint8_t buf[2];
    uint8_t buf_data[256];
    uint16_t touch_x, touch_y;
    uint8_t touch_event;

    tma525_read(AM_DEVICES_TMA525_READ_DATA_BLOCK, buf, 2);

    size = get_unaligned_le16(&buf[0]);

    if (size > 2)
    {
        tma525_read(AM_DEVICES_TMA525_READ_DATA_BLOCK, buf_data, size);

        touch_x = buf_data[10];
        touch_x <<= 8;
        touch_x += buf_data[9];
        *x = AM_DEVICES_TMA525_DISP_RESX - touch_x;

        touch_y = buf_data[12];
        touch_y <<= 8;
        touch_y += buf_data[11];
        *y = AM_DEVICES_TMA525_DISP_RESY - touch_y;

        touch_event = buf_data[8];
        touch_event >>= 5;
        touch_event &= 0x03;
        switch (touch_event)
        {
            case AM_DEVICES_TMA525_STATUS_NO_EVENT:
            break;

            case AM_DEVICES_TMA525_STATUS_TOUCH_DOWN:
                *touch_released = false;
            break;

            case AM_DEVICES_TMA525_STATUS_SIGNIFICANT_DISPLACEMENT:
            break;

            case AM_DEVICES_TMA525_STATUS_LIFT_OFF:
                *touch_released = true;
            break;

            default:break;
        }
    }
    return AM_DEVICES_TMA525_STATUS_SUCCESS;
}

// ****************************************************************************
//
//  Initialize the TMA525 driver for IOM I2C and DMA
//
// ****************************************************************************
uint32_t
am_devices_tma525_init(uint32_t ui32Module, am_hal_iom_config_t *psIOMSettings, void **ppIomHandle)
{
    if ( ui32Module > AM_REG_IOM_NUM_MODULES )
    {
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }

    //
    // Configure the IOM pins.
    //
    am_bsp_iom_pins_enable(ui32Module, AM_HAL_IOM_I2C_MODE);

    //
    // Initialize the IOM instance.
    // Enable power to the IOM instance.
    // Configure the IOM for Serial operation during initialization.
    // Enable the IOM.
    // HAL Success return is 0
    //
    if (am_hal_iom_initialize(ui32Module, &g_tma525_IOMHandle) ||
        am_hal_iom_power_ctrl(g_tma525_IOMHandle, AM_HAL_SYSCTRL_WAKE, false) ||
        am_hal_iom_configure(g_tma525_IOMHandle, psIOMSettings) ||
        am_hal_iom_enable(g_tma525_IOMHandle))
    {
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }
    else
    {
        *ppIomHandle = g_tma525_IOMHandle;
        //
        // Return the status.
        //
    }

    tma525_finish_init();

    return AM_DEVICES_TMA525_STATUS_SUCCESS;
}

// ****************************************************************************
//
//  Deinitialize the TMA525 driver for IOM I2C and DMA
//
// ****************************************************************************
uint32_t
am_devices_tma525_deinit(uint32_t ui32Module)
{
    if ( ui32Module > AM_REG_IOM_NUM_MODULES )
    {
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }

    am_bsp_iom_pins_disable(ui32Module, AM_HAL_IOM_I2C_MODE);

    if (am_hal_iom_uninitialize(g_tma525_IOMHandle) ||
        am_hal_iom_power_ctrl(g_tma525_IOMHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false) ||
        am_hal_iom_disable(g_tma525_IOMHandle))
    {
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }

    return AM_DEVICES_TMA525_STATUS_SUCCESS;
}

// ****************************************************************************
//
//  Initialize the TMA525 driver for IOM I2C multidrop and DMA
//
// ****************************************************************************
uint32_t
am_devices_tma525_multidrop_iom_init(uint32_t ui32Module, am_hal_iom_config_t *pDevConfig, void **ppHandle, void **ppIomHandle)
{
    void *pIomHandle;
    am_hal_iom_config_t     stIOMALSSettings;
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
    for ( ui32Index = 0; ui32Index < 1; ui32Index++ )
    {
        if ( gAmTma525[ui32Index].bOccupied == false )
        {
            break;
        }
    }

    if ( ui32Index == 1 )
    {
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }

    if ( (ui32Module > AM_REG_IOM_NUM_MODULES)  || (pDevConfig == NULL) )
    {
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }

    stIOMALSSettings = g_sI2cTms525Cfg;
    stIOMALSSettings.ui32NBTxnBufLength = pDevConfig->ui32NBTxnBufLength;
    stIOMALSSettings.pNBTxnBuf = pDevConfig->pNBTxnBuf;
    stIOMALSSettings.ui32ClockFreq = pDevConfig->ui32ClockFreq;

    //
    // Look up the Max Transaction size to fit into 8usec for CE asserted
    //
    gAmTma525[ui32Index].ui32MaxTransSize = 0;

    //
    // Configure the IOM pins.
    //
    am_bsp_iom_pins_enable(ui32Module, AM_HAL_IOM_I2C_MODE);

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
        am_hal_iom_power_ctrl(pIomHandle, AM_HAL_SYSCTRL_WAKE, true) ||
        am_hal_iom_configure(pIomHandle, &stIOMALSSettings) ||
        am_hal_iom_enable(pIomHandle))
    {
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }
    else
    {
        am_util_delay_us(150);
        gAmTma525[ui32Index].ui32Module = ui32Module;
        gAmTma525[ui32Index].ui32CS = g_CS[ui32Module];
        *ppIomHandle = gAmTma525[ui32Index].pIomHandle = g_tma525_IOMHandle = pIomHandle;
        *ppHandle = (void *)&gAmTma525[ui32Index];
    }

    tma525_finish_init();

    return AM_DEVICES_TMA525_STATUS_SUCCESS;
}

// ****************************************************************************
//
//  Initialize the TMA525 driver for Multidrop if another device has already
//  initialized the IOM for DMA and multidrop
//
// ****************************************************************************
uint32_t
am_devices_tma525_multidrop_no_iom_init(uint32_t ui32Module, am_hal_iom_config_t *pDevConfig, void **ppHandle, void **ppIomHandle)
{
    am_devices_iom_tma525_t *pIom = (am_devices_iom_tma525_t *)ppHandle;
    gAmTma525[0] = *pIom;
    gAmTma525[0].pIomHandle = *ppIomHandle;

    tma525_finish_init();

    return AM_DEVICES_TMA525_STATUS_SUCCESS;
}

// ****************************************************************************
//
//  Deinitialize & shutdown the IOM and multidrop TMA525 driver
//
// ****************************************************************************
uint32_t
am_devices_tma525_multidrop_term(void *pHandle)
{
    am_devices_iom_tma525_t *pIom = (am_devices_iom_tma525_t *)pHandle;

    if (pIom->ui32Module > AM_REG_IOM_NUM_MODULES)
    {
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }

    // Disable the pins
    am_bsp_iom_pins_disable(pIom->ui32Module, g_sI2cTms525Cfg.eInterfaceMode);

    //
    // Disable the IOM.
    //
    if ( AM_DEVICES_TMA525_STATUS_SUCCESS != am_hal_iom_disable(pIom->pIomHandle) )
    {
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }

    //
    // Disable power to and uninitialize the IOM instance.
    //
    if ( AM_DEVICES_TMA525_STATUS_SUCCESS != am_hal_iom_power_ctrl(pIom->pIomHandle, AM_HAL_SYSCTRL_DEEPSLEEP, true) )
    {
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }

    if ( AM_DEVICES_TMA525_STATUS_SUCCESS != am_hal_iom_uninitialize(pIom->pIomHandle) )
    {
        return AM_DEVICES_TMA525_STATUS_ERROR;
    }

    // Free this device handle
    pIom->bOccupied = false;

    //
    // Return the status.
    //
    return AM_DEVICES_TMA525_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

