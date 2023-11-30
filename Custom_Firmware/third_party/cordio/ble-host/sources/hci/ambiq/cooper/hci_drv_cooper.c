//*****************************************************************************
//
//! @file hci_drv_cooper.c
//!
//! @brief HCI driver interface.
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
#include "am_util.h"
#include "am_bsp.h"

#include "wsf_types.h"
#include "wsf_timer.h"
#include "bstream.h"
#include "wsf_msg.h"
#include "wsf_cs.h"
#include "hci_drv.h"
#include "hci_cmd.h"
#include "hci_drv_apollo.h"
#include "hci_tr_apollo.h"
#include "hci_core.h"
#include "dm_api.h"

#include "hci_drv_cooper.h"
#include "hci_dbg_trc.h"

#include <string.h>

//*****************************************************************************
//
//*****************************************************************************
//
// Enable the heartbeat command
//
// Setting this to 1 will cause the MCU to send occasional HCI packets to the
// BLE core if there hasn't been any activity for a while. This can help catch
// communication issues that might otherwise go unnoticed.
//
//*****************************************************************************
#define ENABLE_BLE_HEARTBEAT            1

// Configurable error-detection thresholds.
//
//*****************************************************************************


//*****************************************************************************
//
// Heartbeat implementation functions.
//
//*****************************************************************************
#if ENABLE_BLE_HEARTBEAT
#define HEARTBEAT_TIMEOUT_MS            (10000)   //milli-seconds
#define BLE_HEARTBEAT_EVENT             (0x12)

#define BLE_HEARTBEAT_START()                                                 \
    do { WsfTimerStartMs(&g_HeartBeatTimer, HEARTBEAT_TIMEOUT_MS); } while (0)

#define BLE_HEARTBEAT_STOP()                                                  \
    do { WsfTimerStop(&g_HeartBeatTimer); } while (0)

#define BLE_HEARTBEAT_RESTART()                                               \
    do                                                                        \
    {                                                                         \
            WsfTimerStop(&g_HeartBeatTimer);                                  \
            WsfTimerStartMs(&g_HeartBeatTimer, HEARTBEAT_TIMEOUT_MS);         \
    } while (0)

wsfTimer_t g_HeartBeatTimer;
#else

#define BLE_HEARTBEAT_START()
#define BLE_HEARTBEAT_STOP()
#define BLE_HEARTBEAT_RESTART()

#endif

#if DISABLE_WAKEUP_NOP_EVENT
#define WAKEUP_TIMEOUT_MS            (100)   //milli-seconds

#define BLE_WAKEUP_TIMER_START()                                                 \
    do { WsfTimerStartMs(&g_WakeUpTimer, WAKEUP_TIMEOUT_MS); } while (0)

#define BLE_WAKEUP_TIMER_STOP()                                                  \
    do { WsfTimerStop(&g_WakeUpTimer); } while (0)

wsfTimer_t g_WakeUpTimer;
#else
#define BLE_WAKEUP_TIMER_START()
#define BLE_WAKEUP_TIMER_STOP()
#endif

uint32_t DMATCBBuffer[AM_DEVICES_COOPER_MAX_TX_PACKET / 4];

// set the BLE MAC address to a special value
uint8_t g_BLEMacAddress[6] = {0};

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
// BLE module handle
void *g_IomDevHdl = NULL;
void *pvHciSpiHandle = NULL;

// Global handle used to send BLE events about the Hci driver layer.
wsfHandlerId_t g_HciDrvHandleID = 0;
//*****************************************************************************
//
// Structure for holding outgoing HCI packets.
//
//*****************************************************************************

// Buffers for HCI read data.
uint32_t g_pui32ReadBuffer[AM_DEVICES_COOPER_MAX_RX_PACKET / 4];
uint8_t *g_pui8ReadBuffer = (uint8_t *) g_pui32ReadBuffer;

uint32_t g_ui32NumBytes = 0;
uint32_t g_consumed_bytes = 0;

//*****************************************************************************
//
// Events for the HCI driver interface.
//
//*****************************************************************************
#define BLE_TRANSFER_NEEDED_EVENT                   0x01

#if ENABLE_BLE_HEARTBEAT
#define BLE_HEARTBEAT_EVENT                         (0x12)
#endif

#if DISABLE_WAKEUP_NOP_EVENT
#define BLE_WAKEUP_EVENT                            (0x13)
#endif


//*****************************************************************************
//
// Error-handling wrapper macro.
//
//*****************************************************************************
#define ERROR_CHECK_VOID(status)                                              \
    {                                                                         \
        uint32_t ui32ErrChkStatus;                                            \
        if (0 != (ui32ErrChkStatus = (status)))                               \
        {                                                                     \
            am_util_debug_printf("ERROR_CHECK_VOID "#status "\n");            \
            error_check(ui32ErrChkStatus);                                    \
            return;                                                           \
        }                                                                     \
    }

#define ERROR_RETURN(status, retval)                                          \
    if ((status))                                                             \
    {                                                                         \
        error_check(status);                                                  \
        return (retval);                                                      \
    }

#define ERROR_RECOVER(status)                                                 \
    if ((status))                                                             \
    {                                                                         \
        error_check(status);                                                  \
        HciDrvRadioShutdown();                                                \
        HciDrvRadioBoot(0);                                                   \
        DmDevReset();                                                         \
        return;                                                               \
    }

//*****************************************************************************
//
// Debug section.
//
//*****************************************************************************
#if 1
#define CRITICAL_PRINT(...)                                                   \
    do                                                                        \
    {                                                                         \
        AM_CRITICAL_BEGIN;                                                    \
        am_util_debug_printf(__VA_ARGS__);                                    \
        AM_CRITICAL_END;                                                      \
    } while (0)
#else
#define CRITICAL_PRINT(...)
#endif

//*****************************************************************************
//
// Function pointer for redirecting errors
//
//*****************************************************************************
hci_drv_error_handler_t g_hciDrvErrorHandler = 0;
static uint32_t g_ui32FailingStatus = 0;

//*****************************************************************************
//
// By default, errors will be printed. If there is an error handler defined,
// they will be sent there intead.
//
//*****************************************************************************
static void
error_check(uint32_t ui32Status)
{
    //
    // Don't do anything unless there's an error.
    //
    if (ui32Status)
    {
        //
        // Set the global error status. If there's an error handler function,
        // call it. Otherwise, just print the error status and wait.
        //
        g_ui32FailingStatus = ui32Status;

        if (g_hciDrvErrorHandler)
        {
            g_hciDrvErrorHandler(g_ui32FailingStatus);
        }
        else
        {
            CRITICAL_PRINT("\nError detected: 0x%08x\n", g_ui32FailingStatus);
        }
    }
}


//*****************************************************************************
//
// Simple interrupt handler to call
//
// Note: These two lines need to be added to the exactle initialization
// function at the beginning of all Cordio applications:
//
//     handlerId = WsfOsSetNextHandler(HciDrvHandler);
//     HciDrvHandler(handlerId);
//
//*****************************************************************************
static void HciDrvIntService(void *pArg)
{
    //
    // Send an event to get processed in the HCI handler.
    //
    WsfSetEvent(g_HciDrvHandleID, BLE_TRANSFER_NEEDED_EVENT);
}

static void ClkReqIntService(void *pArg)
{
    if (am_devices_cooper_clkreq_read(g_IomDevHdl))
    {
        // Power up the 32MHz Crystal
        am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_KICK_START, (void *)&g_amHalMcuctrlArgBLEDefault);
    }
    else
    {
        am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_DISABLE,(void *) &g_amHalMcuctrlArgBLEDefault);
    }
    am_hal_gpio_intdir_toggle(AM_DEVICES_COOPER_CLKREQ_PIN);
}



//*****************************************************************************
//
// Boot the radio.
//
//*****************************************************************************
uint32_t
HciDrvRadioBoot(bool bColdBoot)
{
    uint32_t ui32Status = AM_DEVICES_COOPER_STATUS_SUCCESS;

    am_devices_cooper_config_t stCooperConfig;
    stCooperConfig.pNBTxnBuf = DMATCBBuffer;
    stCooperConfig.ui32NBTxnBufLength = sizeof(DMATCBBuffer) / 4;

    //
    // Initialize the SPI module.
    //
    ui32Status = am_devices_cooper_init(SPI_MODULE, &stCooperConfig, &g_IomDevHdl, &pvHciSpiHandle);
    ERROR_RETURN(ui32Status, ui32Status);

    //
    // Set the default BLE TX Output power.
    //
    am_util_ble_tx_power_set(g_IomDevHdl, TX_POWER_LEVEL_DEFAULT);

    uint32_t IntNum = AM_DEVICES_COOPER_IRQ_PIN;
    am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, IntNum, HciDrvIntService, NULL);
    am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0,
                                  AM_HAL_GPIO_INT_CTRL_INDV_ENABLE,
                                  (void *)&IntNum);
    IntNum = AM_DEVICES_COOPER_CLKREQ_PIN;
    am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, IntNum, ClkReqIntService, NULL);
    am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0,
                                  AM_HAL_GPIO_INT_CTRL_INDV_ENABLE,
                                  (void *)&IntNum);
#ifdef AM_IRQ_PRIORITY_DEFAULT
    NVIC_SetPriority(AM_COOPER_IRQn, AM_IRQ_PRIORITY_DEFAULT);
#endif // AM_IRQ_PRIORITY_DEFAULT
    NVIC_EnableIRQ(AM_COOPER_IRQn);

    // When it's bColdBoot, it will use Apollo's Device ID to form Bluetooth address.
    if (bColdBoot)
    {
        am_hal_mcuctrl_device_t sDevice;
        am_hal_mcuctrl_info_get(AM_HAL_MCUCTRL_INFO_DEVICEID, &sDevice);

        // Bluetooth address formed by ChipID1 (32 bits) and ChipID0 (8-23 bits).
        memcpy(g_BLEMacAddress, &sDevice.ui32ChipID1, sizeof(sDevice.ui32ChipID1));
        // ui32ChipID0 bit 8-31 is test time during chip manufacturing
        g_BLEMacAddress[4] = (sDevice.ui32ChipID0 >> 8) & 0xFF;
        g_BLEMacAddress[5] = (sDevice.ui32ChipID0 >> 16) & 0xFF;
    }

    return AM_DEVICES_COOPER_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Shut down the BLE core.
//
//*****************************************************************************
void
HciDrvRadioShutdown(void)
{
    BLE_HEARTBEAT_STOP();

    uint32_t IntNum = AM_DEVICES_COOPER_IRQ_PIN;
    am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0,
                                  AM_HAL_GPIO_INT_CTRL_INDV_DISABLE,
                                  (void *)&IntNum);
    IntNum = AM_DEVICES_COOPER_CLKREQ_PIN;
    am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0,
                                  AM_HAL_GPIO_INT_CTRL_INDV_DISABLE,
                                  (void *)&IntNum);

    am_devices_cooper_term(g_IomDevHdl);
}

//*****************************************************************************
//
// Function used by the BLE stack to send HCI messages to the BLE controller.
//
// Internally, the Cordio BLE stack will allocate memory for an HCI message,
//
//*****************************************************************************

uint16_t
hciDrvWrite(uint8_t type, uint16_t len, uint8_t *pData)
{
    uint32_t ui32ErrorStatus = 0;

    if (len > AM_DEVICES_COOPER_MAX_TX_PACKET)
    {
        CRITICAL_PRINT("\nERROR: Trying to send an HCI packet larger than the hci driver buffer size (needs %d bytes of space).\n",
                       len);
        ERROR_RETURN(HCI_DRV_TX_PACKET_TOO_LARGE, 0);
    }

    ui32ErrorStatus = am_devices_cooper_blocking_write(g_IomDevHdl, type, (uint32_t*)pData, (uint16_t)len, false);

    if (ui32ErrorStatus == AM_DEVICES_COOPER_STATUS_SUCCESS)
    {
        //
        // Restart the heartbeat timer.
        //
        BLE_HEARTBEAT_RESTART();
        return len;
    }
    else if (ui32ErrorStatus == AM_DEVICES_COOPER_STATUS_CONTROLLER_NOT_READY)
    {
        // Cooper is not ready for current writing request,
        // HOST will start the timmer to check if there is any pending HCI data.
        BLE_WAKEUP_TIMER_START();
        return 0;
    }
    else if (ui32ErrorStatus == AM_DEVICES_COOPER_STATUS_TIMEOUT)
    {
        // When using polling mechanism to send HCI packets to Cooper,
        // consider Cooper needs to reboot if no response within setting timeout
        HciDrvRadioShutdown();
        HciDrvRadioBoot(0);
        DmDevReset();
        return 0;
    }
    else
    {
        return 0;
    }
}

//*****************************************************************************
//
// Save the handler ID of the HciDrvHandler so we can send it events through
// the WSF task system.
//
// Note: These two lines need to be added to the exactle initialization
// function at the beginning of all Cordio applications:
//
//     handlerId = WsfOsSetNextHandler(HciDrvHandler);
//     HciDrvHandler(handlerId);
//
//*****************************************************************************
void
HciDrvHandlerInit(wsfHandlerId_t handlerId)
{
    g_HciDrvHandleID = handlerId;

#if ENABLE_BLE_HEARTBEAT
    g_HeartBeatTimer.handlerId = handlerId;
    g_HeartBeatTimer.msg.event = BLE_HEARTBEAT_EVENT;
#endif

#if DISABLE_WAKEUP_NOP_EVENT
    g_WakeUpTimer.handlerId = handlerId;
    g_WakeUpTimer.msg.event = BLE_WAKEUP_EVENT;
#endif

}


//*****************************************************************************
//
// Event handler for HCI-related events.
//
// This handler can perform HCI reads or writes, and keeps the actions in the
// correct order.
//
//*****************************************************************************
void
HciDrvHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    uint32_t ui32ErrorStatus = 0;

#if ENABLE_BLE_HEARTBEAT
    //
    // If this handler was called in response to a heartbeat event, then it's
    // time to run a benign HCI command. Normally, the BLE controller should
    // handle this command without issue. If it doesn't acknowledge the
    // command, we will eventually get an HCI command timeout error, which will
    // alert us to the fact that the BLE core has become unresponsive in
    // general.
    //
    if (pMsg != NULL)
    {
        if (pMsg->event == BLE_HEARTBEAT_EVENT)
        {
            HciReadLocalVerInfoCmd();
            BLE_HEARTBEAT_START();

            return;
        }
    }
#endif

#if DISABLE_WAKEUP_NOP_EVENT
    //
    // If this handler was called in response to a wakeup event, then it
    // means the last HCI packet wakeup interrupt signal is not coming up
    // within 100ms, will trigger checking pending HCI command/ACL data.
    //
    if (pMsg != NULL)
    {
        if (pMsg->event == BLE_WAKEUP_EVENT)
        {
            BLE_WAKEUP_TIMER_STOP();

            WsfSetEvent(g_HciDrvHandleID, BLE_TRANSFER_NEEDED_EVENT);
        }
    }
#endif

    //
    //need to check the each bit of event mask
    //
    if ( event & BLE_TRANSFER_NEEDED_EVENT )
    {
        am_devices_cooper_t* pBle = (am_devices_cooper_t*)g_IomDevHdl;

        if ( pBle->bWakingUp )
        {
            BLE_WAKEUP_TIMER_STOP();
            pBle->bWakingUp = false;
            /* Send any pending HCI command */
            hciCmdSend(NULL);

            /* Send any pending ACL data */
            hciCoreTxReady(0);
        }
        else // read
        {
            //
            // Check to see if we read any bytes over the HCI interface that we haven't
            // already sent to the BLE stack.
            //
            if (g_ui32NumBytes > g_consumed_bytes)
            {
                //
                // If we have any bytes saved, we should send them to the BLE stack
                // now.
                //
                g_consumed_bytes += hciTrSerialRxIncoming(g_pui8ReadBuffer + g_consumed_bytes,
                                                        g_ui32NumBytes - g_consumed_bytes);

                //
                // If the stack doesn't accept all of the bytes we had, we will need to
                // keep the event set and come back later. Otherwise, we can just reset
                // our variables and exit the loop.
                //
                // Make sure g_ui32NumBytes is not zero.
                if (g_ui32NumBytes && (g_consumed_bytes != g_ui32NumBytes))
                {
                    WsfSetEvent(g_HciDrvHandleID, BLE_TRANSFER_NEEDED_EVENT);
                    return;
                }
                else
                {
                    g_ui32NumBytes   = 0;
                    g_consumed_bytes = 0;
                }
            }

            // Reset
            g_ui32NumBytes = 0;
            ui32ErrorStatus = am_devices_cooper_blocking_read(g_IomDevHdl, g_pui32ReadBuffer, &g_ui32NumBytes);
            BLE_HEARTBEAT_RESTART();

            if (g_ui32NumBytes > AM_DEVICES_COOPER_MAX_RX_PACKET)
            {
                CRITICAL_PRINT("\nERROR: Trying to receive an HCI packet larger than the hci driver buffer size (needs %d bytes of space).\n",
                            g_ui32NumBytes);
                ERROR_CHECK_VOID(HCI_DRV_RX_PACKET_TOO_LARGE);
            }
            else if ( ui32ErrorStatus != AM_DEVICES_COOPER_STATUS_SUCCESS)
            {
                //
                // If the read didn't succeed for some physical reason, we need
                // to know. We shouldn't get failures here. We checked the IRQ
                // signal before calling the read function, and this driver
                // only contains a single call to the blocking read function,
                // so there shouldn't be any physical reason for the read to
                // fail.
                //
                CRITICAL_PRINT("\nHCI READ failed with status %d. Try recording with a logic analyzer to catch the error.\n",
                            ui32ErrorStatus);
                ERROR_RECOVER(ui32ErrorStatus);
            }
            else
            {
        #ifdef ENABLE_BLE_CTRL_TRACE
                if(*g_pui8ReadBuffer == HCI_TRACE_TYPE)
                {
                #if 0
                    {
                    char tmp[400] = {0};
                    int i = 0;
                    int tmp_len = g_ui32NumBytes;
                    if(tmp_len > 100) tmp_len = 100;

                    for(i=0; i<tmp_len; i++)
                        sprintf(tmp+(i*3), "%02x ", g_pui8ReadBuffer[i]);

                    am_util_debug_printf("Rx dbg trace: %s, len :%d\n", tmp, g_ui32NumBytes);
                    }
                #endif

                    hci_process_trace_data(g_pui8ReadBuffer+1, g_ui32NumBytes-1);
                    g_consumed_bytes = g_ui32NumBytes;
                }
                else
        #endif
                {
                    //
                    // Pass the data along to the stack. The stack should be able
                    // to read as much data as we send it.  If it can't, we need to
                    // know that.
                    //
                    g_consumed_bytes = hciTrSerialRxIncoming(g_pui8ReadBuffer, g_ui32NumBytes);

                    // Make sure g_ui32NumBytes is not zero.
                    if (g_ui32NumBytes && (g_consumed_bytes != g_ui32NumBytes))
                    {

                        // need to come back again
                        WsfSetEvent(g_HciDrvHandleID, BLE_TRANSFER_NEEDED_EVENT);
                    }
                }
            }
        }
    }
}

//*****************************************************************************
//
// Register an error handler for the HCI driver.
//
//*****************************************************************************
void
HciDrvErrorHandlerSet(hci_drv_error_handler_t pfnErrorHandler)
{
    g_hciDrvErrorHandler = pfnErrorHandler;
}

/*************************************************************************************************/
/*!
 *  \fn     HciVscSetRfPowerLevelEx
 *
 *  \brief  Vendor specific command for settting Radio transmit power level
 *          for Ambiq.
 *
 *  \param  txPowerlevel    valid range from 0 to 15 in decimal.
 *
 *  \return true when success, otherwise false
 */
/*************************************************************************************************/
bool HciVscSetRfPowerLevelEx(txPowerLevel_t txPowerlevel)
{
    // make sure it's 8 bit
    uint8_t tx_power_level = (uint8_t)txPowerlevel;

    if(tx_power_level < TX_POWER_LEVEL_INVALID) {
      HciVendorSpecificCmd(HCI_VSC_SET_TX_POWER_LEVEL_CFG_CMD_OPCODE, HCI_VSC_SET_TX_POWER_LEVEL_CFG_CMD_LENGTH, &tx_power_level);
      return true;
    }
    else {
      return false;
    }
}

/*************************************************************************************************/
/*!
 *  \brief  read memory variable
 *
 *  \param  start_addr   Start address to read
 *  \param  access_type  Access type
 *  \param  length       Length to read
 *
 *  \return true when success, otherwise false
 */
/*************************************************************************************************/
bool HciVscReadMem(uint32_t start_addr, eMemAccess_type access_type, uint8_t length)
{
    if((length > MAX_MEM_ACCESS_SIZE)
        || ((access_type != RD_8_Bit) && (access_type != RD_16_Bit) && (access_type != RD_32_Bit)))
    {
        return false;
    }

    hciRdMemCmd_t rdMemCmd = {0};
    uint8_t *p = (uint8_t *)&rdMemCmd;
    UINT32_TO_BSTREAM(p, start_addr);
    UINT8_TO_BSTREAM(p, access_type);
    UINT8_TO_BSTREAM(p, length);

    HciVendorSpecificCmd(HCI_VSC_RD_MEM_CMD_OPCODE, HCI_VSC_RD_MEM_CMD_LENGTH, (uint8_t *)&rdMemCmd);

    return true;
}

/*************************************************************************************************/
/*!
 *  \brief  write memory variable
 *
 *  \param  start_addr   Start address to write
 *  \param  access_type  Access type
 *  \param  length       Length to write
 *  \param  data         Data to write
 *
 *  \return true when success, otherwise false
 */
/*************************************************************************************************/
bool HciVscWriteMem(uint32_t start_addr, eMemAccess_type access_type, uint8_t length, uint8_t *data)
{
    if((length > MAX_MEM_ACCESS_SIZE)
        || ((access_type != RD_8_Bit) && (access_type != RD_16_Bit) && (access_type != RD_32_Bit)))
    {
        return false;
    }

    hciWrMemCmd_t wrMemCmd = {0};
    uint8_t *p = (uint8_t *)&wrMemCmd;
    UINT32_TO_BSTREAM(p, start_addr);
    UINT8_TO_BSTREAM(p, access_type);
    UINT8_TO_BSTREAM(p, length);
    
    memset(wrMemCmd.data, 0x0, MAX_MEM_ACCESS_SIZE);
    memcpy(wrMemCmd.data, data, length);

    HciVendorSpecificCmd(HCI_VSC_WR_MEM_CMD_OPCODE, HCI_VSC_WR_MEM_CMD_LENGTH, (uint8_t *)&wrMemCmd);

    return true;
}


/*************************************************************************************************/
/*!
 *  \brief  Get flash ID
 *
 *  \param  NULL
 *
 *  \return None
 */
/*************************************************************************************************/
void HciVscGetFlashId(void)
{
    HciVendorSpecificCmd(HCI_VSC_ID_FLASH_CMD_OPCODE, HCI_VSC_ID_FLASH_CMD_LENGTH, NULL);
}


/*************************************************************************************************/
/*!
 *  \brief  Erase specifide flash space
 *
 *  \param  flash_type  Flash type
 *  \param  offset      Start offset address
 *  \param  length      Length to erase
 *
 *  \return None
 */
/*************************************************************************************************/
void HciVscEraseFlash(uint8_t flash_type, uint32_t offset, uint32_t length)
{
    hciErFlashCmd_t erFlashCmd = {0};
    uint8_t *p = (uint8_t *)&erFlashCmd;
    UINT8_TO_BSTREAM(p, flash_type);
    UINT32_TO_BSTREAM(p, offset);
    UINT32_TO_BSTREAM(p, length);

    HciVendorSpecificCmd(HCI_VSC_ER_FLASH_CMD_OPCODE, HCI_VSC_ER_FLASH_CMD_LENGTH, (uint8_t *)&erFlashCmd);
}


/*************************************************************************************************/
/*!
 *  \brief  write flash
 *
 *  \param  flash_type  Flash type
 *  \param  offset      Start offset address
 *  \param  length      Length to write
 *  \param  data        Data to write
 *
 *  \return true when success, otherwise false
 */
/*************************************************************************************************/
bool HciVscWriteFlash(uint8_t flash_type, uint32_t offset, uint8_t length, uint8_t *data)
{
    if(data == NULL)
    {
        return false;
    }

    hciWrFlashCmd_t wrFlashCmd = {0};
    uint8_t *p = (uint8_t *)&wrFlashCmd;
    UINT8_TO_BSTREAM(p, flash_type);
    UINT32_TO_BSTREAM(p, offset);
    UINT8_TO_BSTREAM(p, length);

    memset(wrFlashCmd.data, 0x0, MAX_FLASH_ACCESS_SIZE);
    memcpy(wrFlashCmd.data, data, length);

    HciVendorSpecificCmd(HCI_VSC_WR_FLASH_CMD_OPCODE, HCI_VSC_WR_FLASH_CMD_LENGTH, (uint8_t *)&wrFlashCmd);

    return true;
}



/*************************************************************************************************/
/*!
 *  \brief  Read flash
 *
 *  \param  flash_type   Flash type
 *  \param  offset       Start offset address
 *  \param  length       Length to read
 *
 *  \return true when success, otherwise false
 */
/*************************************************************************************************/
bool HciVscReadFlash(uint8_t flash_type, uint32_t offset, uint8_t length)
{
    hciRdFlashCmd_t rdFlashCmd = {0};
    uint8_t *p = (uint8_t *)&rdFlashCmd;
    UINT8_TO_BSTREAM(p, flash_type);
    UINT32_TO_BSTREAM(p, offset);
    UINT8_TO_BSTREAM(p, length);

    HciVendorSpecificCmd(HCI_VSC_RD_FLASH_CMD_OPCODE, HCI_VSC_RD_FLASH_CMD_LENGTH, (uint8_t *)&rdFlashCmd);

    return true;
}

/*************************************************************************************************/
/*!
 *  \brief  Read Register value
 *
 *  \param  reg_addr  Register address to read
 *
 *  \return None
 */
/*************************************************************************************************/
void HciVscReadReg(uint32_t reg_addr)
{
    hciRegRdCmd_t rdRegCmd =
    {
        .addr = reg_addr
    };

    HciVendorSpecificCmd(HCI_VSC_REG_RD_CMD_OPCODE, HCI_VSC_REG_RD_CMD_LENGTH, (uint8_t *)&rdRegCmd);
}

/*************************************************************************************************/
/*!
 *  \brief  Write Register value
 *
 *  \param  reg_addr   Register address to read
 *  \param  value      Value to write
 *
 *  \return None
 */
/*************************************************************************************************/
void HciVscWriteReg(uint32_t reg_addr, uint32_t value)
{
    hciRegWrCmd_t wrRegCmd =
    {
        .addr = reg_addr,
        .value = value
    };

    HciVendorSpecificCmd(HCI_VSC_REG_WR_CMD_OPCODE, HCI_VSC_REG_WR_CMD_LENGTH, (uint8_t *)&wrRegCmd);
}

/*************************************************************************************************/
/*!
 *
 *  \brief  Vendor specific command for updating firmware
 *
 *  \param  update_sign    firmware type to update
 *
 *  \return None
 */
/*************************************************************************************************/
void HciVscUpdateFw(uint32_t update_sign)
{
    HciVendorSpecificCmd(HCI_VSC_UPDATE_FW_CFG_CMD_OPCODE, HCI_VSC_UPDATE_FW_CFG_CMD_LENGTH, (uint8_t *)&update_sign);
}

/*************************************************************************************************/
/*!
 *  \brief  Get device ID
 *
 *  \param  NULL
 *
 *  \return None
 */
/*************************************************************************************************/
void HciVscGetDeviceId(void)
{
    HciVendorSpecificCmd(HCI_VSC_GET_DEVICE_ID_CFG_CMD_OPCODE, HCI_VSC_GET_DEVICE_ID_CFG_CMD_LENGTH, NULL);
}

/*************************************************************************************************/
/*!
 *  \brief  platform reset
 *
 *  \param  reson   Reson to reset platform
 *
 *  \return None
 */
/*************************************************************************************************/
void HciVscPlfReset(ePlfResetReason_type reason)
{
    hciPlfResetCmd_t resetPlfCmd =
    {
        .reason = reason
    };

    HciVendorSpecificCmd(HCI_VSC_PLF_RESET_CMD_OPCODE, HCI_VSC_PLF_RESET_CMD_LENGTH, (uint8_t *)&resetPlfCmd);
}

#ifdef ENABLE_BLE_CTRL_TRACE
/*************************************************************************************************/
/*!
 *  \brief  Set trace bitmap to enable which traces to output to host.
 *
 *  \param  bit_map    bit map for trace module
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciVscSetTraceBitMap(ble_trace_cfg bit_map)
{
    HciVendorSpecificCmd(HCI_VSC_SET_LOG_BITMAP_CFG_CMD_OPCODE, HCI_VSC_SET_LOG_BITMAP_CFG_CMD_LENGTH, (uint8_t *)&bit_map);
}
#endif

/*************************************************************************************************/
/*!
 *  \brief  Configure the specified event mask
 *  \param evt_mask, the event mask to configure, see #enum eCfgEvtMsk_type
 */
/*************************************************************************************************/
void HciVscConfigEvtMask(uint32_t evt_mask)
{
    HciVendorSpecificCmd(HCI_VSC_CFG_EVT_MASK_CMD_OPCODE, HCI_VSC_CFG_EVT_MASK_CMD_LENGTH, (uint8_t *)&evt_mask);
}


/*************************************************************************************************/
/*!
 *  \fn     HciVscSetCustom_BDAddr
 *
 *  \brief  This procedure is to set customer-provided Bluetooth address if needed.
 *
 *  \param  bd_addr  pointer to a bluetooth address customer allocates or NULL to use Apollo Device ID.
 *
 *  \return true when success
 */
/*************************************************************************************************/
bool_t HciVscSetCustom_BDAddr(uint8_t *bd_addr)
{
    uint8_t invalid_bd_addr[6] = {0};

    // When bd_addr is null, it will use Apollo's Device ID to form Bluetooth address.
    if ((bd_addr == NULL) || (memcmp(invalid_bd_addr, bd_addr, 6) == 0))
        return false;
    else {
        memcpy(g_BLEMacAddress, bd_addr, 6);
        return true;
    }
}

void HciVscUpdateBDAddress(void)
{
    HciVendorSpecificCmd(HCI_VSC_SET_BD_ADDR_CFG_CMD_OPCODE, HCI_VSC_SET_BD_ADDR_CFG_CMD_LENGTH, g_BLEMacAddress);
}


uint8_t nvds_data[HCI_VSC_UPDATE_NVDS_CFG_CMD_LENGTH]=
{
    NVDS_PARAMETER_MAGIC_NUMBER,
    NVDS_PARAMETER_SLEEP_ALGO_DUR,
    NVDS_PARAMETER_LPCLK_DRIFT,
    NVDS_PARAMETER_EXT_WAKEUP_TIME,
    NVDS_PARAMETER_OSC_WAKEUP_TIME
};

void HciVscUpdateNvdsParam(void)
{
    HciVendorSpecificCmd(HCI_VSC_UPDATE_NVDS_CFG_CMD_OPCODE, HCI_VSC_UPDATE_NVDS_CFG_CMD_LENGTH, nvds_data);
}

uint8_t ll_local_feats[LE_FEATS_LEN] = {0};

void HciVscUpdateLinklayerFeature(void)
{
    ll_local_feats[0] = (uint8_t)LL_FEATURES_BYTE0;
    ll_local_feats[1] = (uint8_t)(LL_FEATURES_BYTE1>>8);
    ll_local_feats[2] = (uint8_t)(LL_FEATURES_BYTE2>>16);
    ll_local_feats[3] = (uint8_t)(LL_FEATURES_BYTE3>>24);

    HciVendorSpecificCmd(HCI_VSC_UPDATE_LL_FEATURE_CFG_CMD_OPCODE, LE_FEATS_LEN, ll_local_feats);
}

/*************************************************************************************************/
/*!
 *  \brief  Get RSSI in DTM mode. The command complete event of this command inlcudes one byte BLE
 *          standard error status and one byte signed RSSI value. When the status is 0x0 (success),
 *          the following RSSI value is valid. Before using this command to read the RSSI in DTM mode,
 *          the application needs to send HCI receiver start command to put the controller to RX mode.
 *          Only controller firmware v1.21.1.0 and higher version supports this feature.
 *
 *  \param  NULL
 *
 *  \return None
 */
/*************************************************************************************************/
void HciVscGetDtmRssi(void)
{
    HciVendorSpecificCmd(HCI_VSC_GET_DTM_RSSI_CMD_OPCODE, HCI_VSC_GET_DTM_RSSI_CMD_LENGTH, NULL);
}

