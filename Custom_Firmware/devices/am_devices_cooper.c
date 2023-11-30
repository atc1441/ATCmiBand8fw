//*****************************************************************************
//
//! @file am_devices_cooper.c
//!
//! @brief An implementation of the Apollo inteface to Cooper using the IOM.
//!
//! @addtogroup cooper Cooper BLE Device Driver
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_devices_cooper.h"

//*****************************************************************************
//
// Timing and configuration.
//
//*****************************************************************************
#define AM_DEVICES_COOPER_TIMEOUT                 10
#define AM_DEVICES_COOPER_RETRY_TIMES             500

#define OPCODE_H2WRITE_HANDSHAKE   0x80
#define OPCODE_H2READ_HANDSHAKE    0x04
#define OPCODE_READ_STATUS         0x08

// Set to 1 to turn on logging for SBL diagnosis
#define SBL_DEBUG_LOG_ON           0

//*****************************************************************************
//
// Global state variables.
//
//*****************************************************************************
am_devices_cooper_t gAmCooper[AM_DEVICES_COOPER_MAX_DEVICE_NUM];

//*****************************************************************************
//
// Global SBL update data.
//
//*****************************************************************************
am_devices_cooper_buffer(2) sLengthBytes;

//
//!
//
static am_devices_cooper_sbl_update_state_t gsSblUpdateState;

//
//!
//
static am_devices_cooper_sbl_update_data_t     g_sFwImage =
{
    0,
    0,
    AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_FW,
    0

};

//
//!
//
static am_devices_cooper_sbl_update_data_t     g_sInfo0PatchImage =
{
    0,
    sizeof(am_sbl_info0_patch_blob_t),
    AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_0,
    0
};

//
//!
//
static am_devices_cooper_sbl_update_data_t     g_sInfo1PatchImage =
{
    0,
    0,
    AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_1,
    0
};




//*****************************************************************************
//
// COOPER_CS (54) - COOPER Master chip select.
//
//*****************************************************************************
am_hal_gpio_pincfg_t g_AM_DEVICES_COOPER_SPI_CS =
{
#if defined(AM_PART_APOLLO4L) || defined(APOLLO4P_BLUE_KXR)
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_54_NCE54,
#else
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_43_NCE43,
#endif
    .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
    .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
    .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P5X,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_NONE,
    .GP.cfg_b.uNCE                 = AM_HAL_GPIO_NCE_IOM4CE0,
    .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
    .GP.cfg_b.uRsvd_0              = 0,
    .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
    .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.uRsvd_1              = 0,
};

//*****************************************************************************
//
//  BLE_32M_CLK (46) - BLE 32M CLK OUT.
//
//*****************************************************************************
am_hal_gpio_pincfg_t g_AM_DEVICES_COOPER_32M_CLK =
{
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_46_CLKOUT_32M,
    .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
    .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
    .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_NONE,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_NONE,
    .GP.cfg_b.uNCE                 = 0,
    .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
    .GP.cfg_b.uRsvd_0              = 0,
    .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
    .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.uRsvd_1              = 0,
};

//*****************************************************************************
//
//  BLE_32K_CLK (45) - BLE 32K CLK OUT.
//
//*****************************************************************************
am_hal_gpio_pincfg_t g_AM_DEVICES_COOPER_32K_CLK =
{
#if defined(AM_PART_APOLLO4L) || defined(APOLLO4P_BLUE_KXR)
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_4_32KHzXT,
#else
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_45_32KHzXT,
#endif
    .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
    .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
    .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_NONE,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_NONE,
    .GP.cfg_b.uNCE                 = 0,
    .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
    .GP.cfg_b.uRsvd_0              = 0,
    .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
    .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.uRsvd_1              = 0,
};

//*****************************************************************************
//
// BLE_CLKREQ (40) - BLE CLK request pin.
//
//*****************************************************************************
am_hal_gpio_pincfg_t g_AM_DEVICES_COOPER_CLKREQ =
{
#if defined(AM_PART_APOLLO4L) || defined(APOLLO4P_BLUE_KXR)
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_52_GPIO,
#else
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_40_GPIO,
#endif
    .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_ENABLE,
    .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
    .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_NONE,
    .GP.cfg_b.uNCE                 = 0,
    .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
    .GP.cfg_b.uRsvd_0              = 0,
    .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
    .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.uRsvd_1              = 0,
};

static uint32_t sbl_status = 0;

//*****************************************************************************
//
//  Set up pins for Cooper.
//
//*****************************************************************************
void
am_devices_cooper_pins_enable(void)
{
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_RESET_PIN, am_hal_gpio_pincfg_output);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_IRQ_PIN, am_hal_gpio_pincfg_input);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_CLKREQ_PIN, am_hal_gpio_pincfg_input);
#if (!AM_DEVICES_COOPER_QFN_PART)
#if !defined(AM_PART_APOLLO4L)
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_SWDIO, am_hal_gpio_pincfg_output);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_SWCLK, am_hal_gpio_pincfg_output);
    am_hal_gpio_output_clear(AM_DEVICES_COOPER_SWDIO);
    am_hal_gpio_output_clear(AM_DEVICES_COOPER_SWCLK);
#endif
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_32M_CLK, g_AM_DEVICES_COOPER_32M_CLK);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_32K_CLK, g_AM_DEVICES_COOPER_32K_CLK);
#else
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_32M_OSCEN_PIN, am_hal_gpio_pincfg_output);
    am_hal_gpio_output_set(AM_DEVICES_COOPER_32M_OSCEN_PIN);
    am_util_stdio_printf("\nThe Cooper QFN board is attached for debug\n");
#endif
}

//*****************************************************************************
//
//  Disable pins for Cooper.
//
//*****************************************************************************
void
am_devices_cooper_pins_disable(void)
{
    am_hal_gpio_output_clear(AM_DEVICES_COOPER_RESET_PIN);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_IRQ_PIN, am_hal_gpio_pincfg_disabled);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_CLKREQ_PIN, am_hal_gpio_pincfg_disabled);
#if (!AM_DEVICES_COOPER_QFN_PART)
#if !defined(AM_PART_APOLLO4L)
    am_hal_gpio_output_clear(AM_DEVICES_COOPER_SWDIO);
    am_hal_gpio_output_clear(AM_DEVICES_COOPER_SWCLK);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_SWDIO, am_hal_gpio_pincfg_disabled);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_SWCLK, am_hal_gpio_pincfg_disabled);
#endif
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_32M_CLK, am_hal_gpio_pincfg_disabled);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_32K_CLK, am_hal_gpio_pincfg_disabled);
#else
    am_hal_gpio_output_clear(AM_DEVICES_COOPER_32M_OSCEN_PIN);
    am_hal_gpio_pinconfig(AM_DEVICES_COOPER_32M_OSCEN_PIN, am_hal_gpio_pincfg_disabled);
#endif
}

//*****************************************************************************
//
//  Initialize the BLE controller driver.
//
//*****************************************************************************
uint32_t
am_devices_cooper_init(uint32_t ui32Module, am_devices_cooper_config_t* pDevConfig, void** ppHandle, void** ppBleHandle)
{
    void* pBleHandle;
    am_hal_iom_config_t     stIOMCOOPERSettings;
    uint32_t      ui32Index = 0;
    uint32_t g_CS[AM_REG_IOM_NUM_MODULES] =
    {
        0, 0, 0, 0, 0, 0, 0, 0
    };
    // Allocate a vacant device handle
    for ( ui32Index = 0; ui32Index < AM_DEVICES_COOPER_MAX_DEVICE_NUM; ui32Index++ )
    {
        if ( gAmCooper[ui32Index].bOccupied == false )
        {
            break;
        }
    }
    if ( ui32Index == AM_DEVICES_COOPER_MAX_DEVICE_NUM )
    {
        return AM_DEVICES_COOPER_STATUS_ERROR;
    }
    if ( (ui32Module > AM_REG_IOM_NUM_MODULES)  || (pDevConfig == NULL) )
    {
        return AM_DEVICES_COOPER_STATUS_ERROR;
    }
    //
    // Enable fault detection.
    //
    am_hal_fault_capture_enable();
    stIOMCOOPERSettings.ui32ClockFreq        = COOPER_IOM_FREQ;
    stIOMCOOPERSettings.eInterfaceMode       = AM_HAL_IOM_SPI_MODE,
    stIOMCOOPERSettings.eSpiMode             = AM_HAL_IOM_SPI_MODE_3,
    stIOMCOOPERSettings.ui32NBTxnBufLength   = pDevConfig->ui32NBTxnBufLength;
    stIOMCOOPERSettings.pNBTxnBuf            = pDevConfig->pNBTxnBuf;
    //
    // Initialize the IOM instance.
    // Enable power to the IOM instance.
    // Configure the IOM for Serial operation during initialization.
    // Enable the IOM.
    // HAL Success return is 0
    //
    if (am_hal_iom_initialize(ui32Module, &pBleHandle) ||
            am_hal_iom_power_ctrl(pBleHandle, AM_HAL_SYSCTRL_WAKE, false) ||
            am_hal_iom_configure(pBleHandle, &stIOMCOOPERSettings) ||
            am_hal_iom_enable(pBleHandle))
    {
        return AM_DEVICES_COOPER_STATUS_ERROR;
    }
    else
    {
        //
        // Configure the IOM pins.
        //

#if (SPI_MODULE == 2) && defined(AM_BSP_GPIO_IOM2_CS)
#undef AM_BSP_GPIO_IOM2_CS
#define AM_BSP_GPIO_IOM2_CS  AM_DEVICES_COOPER_SPI_CS // BGA&SIP share the same CS pin(NCE72) on the QFN shiled board
#elif (SPI_MODULE == 4) && defined(AM_BSP_GPIO_IOM4_CS)
#undef AM_BSP_GPIO_IOM4_CS
#define AM_BSP_GPIO_IOM4_CS  AM_DEVICES_COOPER_SPI_CS
#endif
        am_bsp_iom_pins_enable(ui32Module, AM_HAL_IOM_SPI_MODE);
        am_devices_cooper_pins_enable();
#if (!AM_DEVICES_COOPER_QFN_PART)
        //
        // Enable crystals for Cooper
        //
        am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32K_ENABLE, 0);

        //
        // Enable the 32Mnz HF XTAL clock
        //


        am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_KICK_START, (void *)&g_amHalMcuctrlArgBLEDefault );


#endif
        am_devices_cooper_reset();

        gAmCooper[ui32Index].pfnCallback = NULL;
        gAmCooper[ui32Index].pCallbackCtxt = NULL;
        gAmCooper[ui32Index].bBusy = false;
        gAmCooper[ui32Index].bOccupied = true;
        gAmCooper[ui32Index].bNeedCallback = true;
        gAmCooper[ui32Index].bDMAComplete = false;
        gAmCooper[ui32Index].bWakingUp = false;
        gAmCooper[ui32Index].ui32CS = g_CS[ui32Module];
        gAmCooper[ui32Index].ui32Module = ui32Module;
        gAmCooper[ui32Index].ui32CSDuration = 100;
        *ppBleHandle = gAmCooper[ui32Index].pBleHandle = pBleHandle;
        *ppHandle = (void*)&gAmCooper[ui32Index];
        // SBL checking
        am_devices_cooper_image_update_init(*ppHandle, pDevConfig->pNBTxnBuf);
        sbl_status = AM_DEVICES_COOPER_SBL_STATUS_INIT;
        sbl_status = am_devices_cooper_update_image();

        while ( (sbl_status != AM_DEVICES_COOPER_SBL_STATUS_OK) &&
                ( sbl_status != AM_DEVICES_COOPER_SBL_STATUS_FAIL) )
        {
            while (am_devices_cooper_irq_read() == 0)
            {
                am_hal_delay_us(50);
            }
            sbl_status = am_devices_cooper_update_image();
        }
        //
        // Return the status.
        //
        if (sbl_status == AM_DEVICES_COOPER_SBL_STATUS_OK)
        {
            // The CS assertation duration optimization only takes effect after V1.14
            if ( gsSblUpdateState.ui32CooperFWImageVersion < 0x10E )
            {
                gAmCooper[ui32Index].ui32CSDuration = 300;
            }
            gAmCooper[ui32Index].ui32Firmver = gsSblUpdateState.ui32CooperFWImageVersion;
            // need to wait a bit to jump from SBL to Cooper application firmware
            am_util_delay_ms(10);
            am_util_stdio_printf("BLE Controller Init Done\r\n");
            return AM_DEVICES_COOPER_STATUS_SUCCESS;
        }
        else
        {
            // free up resource that won't be used.
            am_devices_cooper_term(*ppHandle);
            *ppHandle = NULL;
            am_util_stdio_printf("BLE Controller SBL Error 0x%x\r\n", sbl_status);
            return gsSblUpdateState.ui32CooperSblStatus;
        }
    }
}

//*****************************************************************************
//
//  De-Initialize the BLE controller driver.
//
//*****************************************************************************
uint32_t
am_devices_cooper_term(void* pHandle)
{
    am_devices_cooper_t* pBle = (am_devices_cooper_t*)pHandle;
    if ( pBle->ui32Module > AM_REG_IOM_NUM_MODULES )
    {
        return AM_DEVICES_COOPER_STATUS_ERROR;
    }
#if (!AM_DEVICES_COOPER_QFN_PART)
    // Disable crystals
    am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32K_DISABLE, 0);

    am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_DISABLE, (void *)&g_amHalMcuctrlArgBLEDefault);
#endif
    // Disable the pins
    am_bsp_iom_pins_disable(pBle->ui32Module, AM_HAL_IOM_SPI_MODE);
    am_devices_cooper_pins_disable();
    //
    // Disable the IOM.
    //
    am_hal_iom_disable(pBle->pBleHandle);
    //
    // Disable power to and uninitialize the IOM instance.
    //
    // Clear local register values first
    am_hal_iom_power_ctrl(pBle->pBleHandle, AM_HAL_SYSCTRL_WAKE, true);
    am_hal_iom_power_ctrl(pBle->pBleHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false);
    am_hal_iom_uninitialize(pBle->pBleHandle);
    // Free this device handle
    pBle->bOccupied = false;
    // Not in waking up state
    pBle->bWakingUp = false;
    //
    // Return the status.
    //
    return AM_DEVICES_COOPER_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Reset BLE Controller.
//
//*****************************************************************************
void
am_devices_cooper_reset(void)
{
    am_hal_gpio_output_set(AM_DEVICES_COOPER_RESET_PIN);
    am_util_delay_ms(20);
    am_hal_gpio_output_clear(AM_DEVICES_COOPER_RESET_PIN);
    am_util_delay_ms(20);
    am_hal_gpio_output_set(AM_DEVICES_COOPER_RESET_PIN);
    // Give some delay for the 32K clock stablization
    am_util_delay_ms(700);
}

//*****************************************************************************
//
//  Enable the IOM bus
//
//*****************************************************************************
uint32_t
am_devices_cooper_bus_enable(void* pHandle)
{
    am_devices_cooper_t* pBle = (am_devices_cooper_t*)pHandle;
    if ( pBle->bOccupied != true )
    {
        return AM_DEVICES_COOPER_STATUS_INVALID_OPERATION;
    }
    // Mark the BLE interface busy so it doesn't get used by more than one
    // interface.
    if (pBle->bBusy == false)
    {
        pBle->bBusy = true;
    }
    else
    {
        return AM_DEVICES_COOPER_STATUS_BUS_BUSY;
    }
    am_hal_iom_power_ctrl(pBle->pBleHandle, AM_HAL_SYSCTRL_WAKE, true);
    am_hal_iom_enable(pBle->pBleHandle);
    return AM_DEVICES_COOPER_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Disable the IOM bus
//
//*****************************************************************************
uint32_t
am_devices_cooper_bus_disable(void* pHandle)
{
    am_devices_cooper_t* pBle = (am_devices_cooper_t*)pHandle;
    if ( pBle->bOccupied != true )
    {
        return AM_DEVICES_COOPER_STATUS_INVALID_OPERATION;
    }
    //
    // Disable the IOM.
    //
    am_hal_iom_disable(pBle->pBleHandle);
    //
    // Disable power.
    //
    am_hal_iom_power_ctrl(pBle->pBleHandle, AM_HAL_SYSCTRL_DEEPSLEEP, true);
    // Release the bus so someone else can use it.
    pBle->bBusy = false;
    return AM_DEVICES_COOPER_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Execute HCI blocking write to the BLE controller
//
//*****************************************************************************
uint32_t
am_devices_cooper_blocking_write(void* pHandle, uint8_t ui8Type, uint32_t* pui32Data,
                                 uint32_t ui32NumBytes, bool bWaitReady)
{
    uint32_t ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_SUCCESS;
    uint32_t ui32WaitReadyCount = 0;
    memset(&sLengthBytes.bytes, 0, 2);
    //
    // Make a structure for the IOM transfer.
    //
    am_hal_iom_transfer_t sIOMTransfer;
    am_devices_cooper_t *pBle = (am_devices_cooper_t *)pHandle;

    if ( pBle->bWakingUp )
    {
        ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_CONTROLLER_NOT_READY;
        return ui32ErrorStatus;
    }

    // Awake IOM and lock the bus
    ui32ErrorStatus = am_devices_cooper_bus_enable(pHandle);
    if (ui32ErrorStatus != AM_DEVICES_COOPER_STATUS_SUCCESS)
    {
        return ui32ErrorStatus;
    }

#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
    sIOMTransfer.ui64Instr = OPCODE_H2WRITE_HANDSHAKE;
#else
    sIOMTransfer.ui32Instr = OPCODE_H2WRITE_HANDSHAKE;
#endif
    sIOMTransfer.ui32InstrLen = 1;
    sIOMTransfer.eDirection = AM_HAL_IOM_RX;
    sIOMTransfer.ui32NumBytes = 2;
    sIOMTransfer.bContinue = true;
    sIOMTransfer.uPeerInfo.ui32SpiChipSelect = pBle->ui32CS;
    sIOMTransfer.pui32RxBuffer = sLengthBytes.words;
    sIOMTransfer.ui8RepeatCount = 0;
    sIOMTransfer.ui32PauseCondition = 0;
    sIOMTransfer.ui32StatusSetClr = 0;
    do
    {
        if (am_hal_iom_blocking_transfer(pBle->pBleHandle, &sIOMTransfer))
        {
            ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_PACKET_INCOMPLETE;
            break;
        }
        // Cooper is not ready now
        if ((sLengthBytes.bytes[0] != 0x68) || (sLengthBytes.bytes[1] != 0xA8))
        {
            ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_CONTROLLER_NOT_READY;
            //
            // Cooper needs CS to low/asserted for 100us to detect wakeup request,
            // and it takes about 2ms for Cooper to be ready to accept packet by asserting
            // IRQ pin.
            //
            am_util_delay_us(pBle->ui32CSDuration);
            // For the applications which do not enable IRQ interrupt, we need to do continuous try here
            if ( bWaitReady )
            {
                //
                // We need to set CS pin (first configured as GPIO) high to trigger Cooper to start wakeup process,
                // after that we reconfigure CS pin back to IOM mode for next transfer.
                //
                am_hal_gpio_pinconfig(AM_DEVICES_COOPER_SPI_CS, am_hal_gpio_pincfg_output);
                am_hal_gpio_output_set(AM_DEVICES_COOPER_SPI_CS);

#if (!AM_DEVICES_COOPER_QFN_PART)
                // If the CS PIN is pulled high and then pulled low too quickly, Cooper may not be able to detect it.
                am_util_delay_us(pBle->ui32CSDuration);
#endif

                am_hal_gpio_pinconfig(AM_DEVICES_COOPER_SPI_CS, g_AM_DEVICES_COOPER_SPI_CS);
                am_util_delay_us(1700);
                //
                // One count is about 2ms, if Cooper is not available to receive HCI packets
                // in configured timeout, need to return the error status and jump out
                // from the infinite trying.
                //
                if (ui32WaitReadyCount == AM_DEVICES_COOPER_RETRY_TIMES)
                {
                    ui32WaitReadyCount = 0;
                    ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_TIMEOUT;
                    break;
                }
                ui32WaitReadyCount ++;
                continue;
            }
            else
            {
                pBle->bWakingUp = true;
                break;
            }
        }
        ui32WaitReadyCount = 0;
        //
        // If this isn't a "raw" transaction, we need to make sure the "type" byte
        // gets through to the interface.
        //
        if (ui8Type != AM_DEVICES_COOPER_RAW)
        {
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
            sIOMTransfer.ui64Instr = ui8Type;
#else
            sIOMTransfer.ui32Instr = ui8Type;
#endif
            sIOMTransfer.ui32InstrLen = 1;
        }
        else
        {
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
            sIOMTransfer.ui64Instr = 0;
#else
            sIOMTransfer.ui32Instr = 0;
#endif
            sIOMTransfer.ui32InstrLen = 0;
        }
        sIOMTransfer.eDirection = AM_HAL_IOM_TX;
        sIOMTransfer.ui32NumBytes = ui32NumBytes;
        sIOMTransfer.pui32TxBuffer = pui32Data;
        sIOMTransfer.bContinue = false;
        //
        // If the previous step succeeded, we can go ahead and send the data.
        //
        ui32ErrorStatus = am_hal_iom_blocking_transfer(pBle->pBleHandle, &sIOMTransfer);
        if (ui32ErrorStatus != AM_DEVICES_COOPER_STATUS_SUCCESS)
        {
            //
            // The layer above this one doesn't understand IOM errors, so we
            // will intercept and rename it here.
            //
            ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_PACKET_INCOMPLETE;
            break;
        }
        break;
    }
    while (1);

    // Disable IOM to save power
    am_devices_cooper_bus_disable(pHandle);

    return ui32ErrorStatus;
}


//*****************************************************************************
//
//  Execute HCI blocking read from the BLE controller
//
//*****************************************************************************
uint32_t
am_devices_cooper_blocking_read(void* pHandle, uint32_t* pui32Data,
                                uint32_t* pui32BytesReceived)
{
    uint32_t ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_SUCCESS;
    memset(&sLengthBytes.bytes, 0, 2);
    am_devices_cooper_t* pBle = (am_devices_cooper_t*)pHandle;

    // Skip if IRQ pin is low -- no pending incoimng packet from Cooper.
    if (!am_devices_cooper_irq_read())
    {
        *pui32BytesReceived = 0;
        return AM_DEVICES_COOPER_STATUS_SUCCESS;
    }
    //
    // Make a structure for the IOM transfer.
    //
    am_hal_iom_transfer_t sIOMTransfer;
    // Awake IOM and lock the bus
    ui32ErrorStatus = am_devices_cooper_bus_enable(pHandle);
    if (ui32ErrorStatus != AM_DEVICES_COOPER_STATUS_SUCCESS)
    {
        return ui32ErrorStatus;
    }
    do
    {
        sIOMTransfer.uPeerInfo.ui32SpiChipSelect = pBle->ui32CS;
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
        sIOMTransfer.ui64Instr = OPCODE_H2READ_HANDSHAKE;
#else
        sIOMTransfer.ui32Instr = OPCODE_H2READ_HANDSHAKE;
#endif
        sIOMTransfer.ui32InstrLen = 1;
        sIOMTransfer.eDirection = AM_HAL_IOM_RX;
        sIOMTransfer.ui32NumBytes = 2;
        sIOMTransfer.pui32RxBuffer = sLengthBytes.words;
        sIOMTransfer.bContinue = true;
        sIOMTransfer.ui8RepeatCount = 0;
        sIOMTransfer.ui32PauseCondition = 0;
        sIOMTransfer.ui32StatusSetClr = 0;
        //
        // First we should get the byte available back
        //
        if (am_hal_iom_blocking_transfer(pBle->pBleHandle, &sIOMTransfer))
        {
            ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_PACKET_INCOMPLETE;
            break;
        }
        if ((sLengthBytes.bytes[0] == 0) && (sLengthBytes.bytes[1] == 0))
        {
            *pui32BytesReceived = 0;
            ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_SUCCESS;
            break;
        }
        //
        // This is the second frame of the read, which contains the actual HCI
        // data.
        //
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
        sIOMTransfer.ui64Instr = 0;
#else
        sIOMTransfer.ui32Instr = 0;
#endif
        sIOMTransfer.ui32InstrLen = 0;
        sIOMTransfer.pui32RxBuffer = pui32Data;
        sIOMTransfer.ui32NumBytes = (sLengthBytes.bytes[0] +
                                     (sLengthBytes.bytes[1] << 8));
        sIOMTransfer.bContinue = false;
        // check ui32NumBytes
        if (sIOMTransfer.ui32NumBytes > AM_DEVICES_COOPER_MAX_RX_PACKET)
        {
            ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_WRONG_DATA_LENGTH;
            *pui32BytesReceived = 0;
            break;
        }
        //
        // Make sure the caller knows how many bytes we got.
        //
        *pui32BytesReceived = sIOMTransfer.ui32NumBytes;
        //
        // Execute the second part of the transfer.
        //
        ui32ErrorStatus = am_hal_iom_blocking_transfer(pBle->pBleHandle, &sIOMTransfer);
        //
        // A failure here indicates that the second part of the read was bad.
        //
        if (ui32ErrorStatus)
        {
            ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_PACKET_INCOMPLETE;
            *pui32BytesReceived = 0;
            break;
        }
    }
    while (0);

    // Disable IOM to save power
    am_devices_cooper_bus_disable(pHandle);

    return ui32ErrorStatus;
}


//*****************************************************************************
//
//  Send HCI raw command to the BLE controller
//
//*****************************************************************************
uint32_t
am_devices_cooper_command_write(void* pHandle, uint32_t* pui32Cmd, uint32_t ui32Length, uint32_t* pui32Response, uint32_t* pui32BytesReceived)
{
    uint32_t ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_SUCCESS;
    if ( !pui32Cmd || !pui32Response || !pui32BytesReceived )
    {
        return AM_DEVICES_COOPER_STATUS_INVALID_OPERATION;
    }
    do
    {
        ui32ErrorStatus = am_devices_cooper_blocking_write(pHandle,
                          AM_DEVICES_COOPER_RAW,
                          pui32Cmd,
                          ui32Length, true);
        if (ui32ErrorStatus)
        {
            break;
        }
        //
        // Wait for the response, and return it to the caller via our variable.
        //
        WHILE_TIMEOUT_MS ( am_devices_cooper_irq_read() == 0, 5000, ui32ErrorStatus );
        if (ui32ErrorStatus)
        {
            ui32ErrorStatus = AM_DEVICES_COOPER_STATUS_NO_RESPONSE;
            break;
        }
        while(1)
        {
            ui32ErrorStatus = am_devices_cooper_blocking_read(pHandle, pui32Response, pui32BytesReceived);
            // Keep reading until we get the corresponding response
            if ((ui32ErrorStatus) || ((UINT32_TO_BYTE0(pui32Response[1]) == UINT32_TO_BYTE1(pui32Cmd[0])) && (UINT32_TO_BYTE1(pui32Response[1]) == UINT32_TO_BYTE2(pui32Cmd[0]))))
            {
                break;
            }
        }
    } while (0);
    //
    // Return the status.
    //
    return ui32ErrorStatus;
}

//*****************************************************************************
//
// Check the state of the IRQ pin.
//
//*****************************************************************************
uint32_t
am_devices_cooper_irq_read(void)
{
    return am_hal_gpio_input_read(AM_DEVICES_COOPER_IRQ_PIN);
}

//*****************************************************************************
//
// Check the state of the CLKREQ pin.
//
//*****************************************************************************
uint32_t
am_devices_cooper_clkreq_read(void* pHandle)
{
    return am_hal_gpio_input_read(AM_DEVICES_COOPER_CLKREQ_PIN);
}

//*****************************************************************************
//
//  Set the 32M crystal frequency based on the tested values at customer side.
//
//*****************************************************************************
uint32_t
am_devices_cooper_crystal_trim_set(void *pHandle, uint32_t ui32TrimValue)
{
    if ( ui32TrimValue > 0x3FF )
    {
        return AM_DEVICES_COOPER_STATUS_ERROR;
    }

    //
    // XTALHSCAP2TRIM : 6;  [5..0] xtalhs_cap2_trim
    // XTALHSCAPTRIM : 4;   [9..6] xtalhs_cap_trim
    //

    g_ui32xtalhscap2trim = (ui32TrimValue & MCUCTRL_XTALHSTRIMS_XTALHSCAP2TRIM_Msk);
    g_ui32xtalhscaptrim  = (ui32TrimValue & MCUCTRL_XTALHSTRIMS_XTALHSCAPTRIM_Msk) >> MCUCTRL_XTALHSTRIMS_XTALHSCAPTRIM_Pos;

    return AM_DEVICES_COOPER_STATUS_SUCCESS;
}



/////////////////////////////////////////////////////////////////////////////////
//
// SBL Driver
//
/////////////////////////////////////////////////////////////////////////////////

//*****************************************************************************
//
// Read a packet from the SBL IOS.
//
//*****************************************************************************
bool iom_slave_read(void* pHandle, uint32_t* pBuf, uint32_t* psize)
{
    am_secboot_wired_msghdr_t* msg;
    uint32_t crc32;
    am_devices_cooper_blocking_read(pHandle, pBuf, psize);
    // Verify the received data CRC
    msg = (am_secboot_wired_msghdr_t*)pBuf;
    am_hal_crc32((uint32_t)&msg->msgType, msg->length - sizeof(uint32_t), &crc32);


    return (crc32 == msg->crc32);
}

//*****************************************************************************
//
// Send a "HELLO" packet.
//
//*****************************************************************************
void send_hello(void* pHandle)
{
    am_secboot_wired_msghdr_t msg;
    msg.msgType = AM_SBL_HOST_MSG_HELLO;
    msg.length = sizeof(am_secboot_wired_msghdr_t);
    //
    // Compute CRC
    //
    //PRT_INFO("send_hello: sending bytes: %d.\n", msg.length );
    am_hal_crc32((uint32_t)&msg.msgType, msg.length - sizeof(uint32_t), &msg.crc32);
    am_devices_cooper_blocking_write(pHandle, AM_DEVICES_COOPER_RAW, (uint32_t*)&msg, sizeof(msg), true);
}

//*****************************************************************************
//
// Send a "UPDATE" packet.
//
//*****************************************************************************
void send_update(void* pHandle, uint32_t imgBlobSize)
{
    am_sbl_host_msg_update_t msg;
    msg.msgHdr.msgType = AM_SBL_HOST_MSG_UPDATE;
    msg.msgHdr.msgLength = sizeof(am_sbl_host_msg_update_t);
    msg.imageSize = imgBlobSize;
    // Check if we are downloading a newer FW versiion
    if ((gsSblUpdateState.ui32CooperFWImageVersion < g_sFwImage.version)
         || (gsSblUpdateState.ui32CooperVerRollBackConfig & 0x00000001))
    {
        msg.versionNumber = g_sFwImage.version;
    }
    else
    {
        msg.versionNumber = gsSblUpdateState.ui32CooperFWImageVersion;
    }
    msg.NumPackets = gsSblUpdateState.ui32TotalPackets + 1; // One addition packet as header will be a seperate packet

    // imageSize will be zero if Apollo4 has no available image/patch for Cooper to load
    // set maxPacketSize to invalid parameter to let Cooper to reply NACK and clear signature
    if ( msg.imageSize == 0 )
    {
        msg.maxPacketSize = AM_DEVICES_COOPER_SBL_UPADTE_INVALID_PSI_PKT_SIZE;
    }
    else
    {
        msg.maxPacketSize = AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
    }
    //
    // Compute CRC
    //
    am_hal_crc32((uint32_t)&msg.msgHdr.msgType, msg.msgHdr.msgLength - sizeof(uint32_t), &msg.msgHdr.msgCrc);
    am_devices_cooper_blocking_write(pHandle, AM_DEVICES_COOPER_RAW, (uint32_t*)&msg, sizeof(msg), true);
}

//*****************************************************************************
//
// Send a "Data" packet.
//
//*****************************************************************************
void send_data(void* pHandle, uint32_t address, uint32_t size, uint32_t pktNumber)
{
    // reuse same buffer for receiving
    am_sbl_host_msg_data_t* msg = (am_sbl_host_msg_data_t*)gsSblUpdateState.pWorkBuf;
    msg->msgHdr.msgType = AM_SBL_HOST_MSG_DATA;
    msg->msgHdr.msgLength = sizeof(am_sbl_host_msg_data_t) + size;
    msg->packetNumber = pktNumber;
    memcpy((uint8_t*)msg->data, (uint8_t*)address, size);
    //
    // Compute CRC
    //
    am_hal_crc32((uint32_t) & (msg->msgHdr.msgType), msg->msgHdr.msgLength - sizeof(uint32_t), &msg->msgHdr.msgCrc);
    am_devices_cooper_blocking_write(pHandle, AM_DEVICES_COOPER_RAW, (uint32_t*)msg, (sizeof(am_sbl_host_msg_data_t) + size), true);
}

//*****************************************************************************
//
// Send a "Reset" packet.
//
//*****************************************************************************
void send_reset(void* pHandle)
{
    am_sbl_host_msg_reset_t msg;
    msg.msgHdr.msgType = AM_SBL_HOST_MSG_RESET;
    msg.msgHdr.msgLength = sizeof(am_sbl_host_msg_reset_t);
    //
    // Compute CRC
    //
    am_hal_crc32((uint32_t)&msg.msgHdr.msgType, msg.msgHdr.msgLength - sizeof(uint32_t), &msg.msgHdr.msgCrc);
    am_devices_cooper_blocking_write(pHandle, AM_DEVICES_COOPER_RAW, (uint32_t*)&msg, sizeof(msg), true);
}

//*****************************************************************************
//
// Send a "FW Continue  packet.
//
//*****************************************************************************
void send_fwContinue(void* pHandle)
{
    am_sbl_host_msg_fw_continue_t msg;
    msg.msgHdr.msgType = AM_SBL_HOST_MSG_FW_CONTINUE;
    msg.msgHdr.msgLength = sizeof(am_sbl_host_msg_fw_continue_t);
    //
    // Compute CRC
    //
    am_hal_crc32((uint32_t)&msg.msgHdr.msgType, msg.msgHdr.msgLength - sizeof(uint32_t), &msg.msgHdr.msgCrc);
    am_devices_cooper_blocking_write(pHandle, AM_DEVICES_COOPER_RAW, (uint32_t*)&msg, sizeof(msg), true);
}

//*****************************************************************************
//
// Update the state machine based on the image to download
//
//*****************************************************************************
static bool am_devices_cooper_sbl_update_state_data(uint32_t ui32updateType)
{
    // Pointer to the data to be updated
    am_devices_cooper_sbl_update_data_t* p_sUpdateImageData = NULL;
    if ( ui32updateType == AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_FW )
    {
        p_sUpdateImageData = &g_sFwImage;
    }
    else if ( ui32updateType == AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_0 )
    {
        p_sUpdateImageData = &g_sInfo0PatchImage;
    }
    else if ( ui32updateType == AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_1 )
    {
        p_sUpdateImageData = &g_sInfo1PatchImage;
    }
    else
    {
        return false;
    }
    // Check if the data is valid
    if (    (p_sUpdateImageData != NULL)                &&
            (p_sUpdateImageData->pImageAddress != 0 )    &&
            (p_sUpdateImageData->imageSize != 0 )       &&
            (p_sUpdateImageData->imageType == ui32updateType) )
    {
        // Load the INFO 0 Patch address
        gsSblUpdateState.pImageBuf          = p_sUpdateImageData->pImageAddress;
        // Image size
        gsSblUpdateState.ui32ImageSize      = p_sUpdateImageData->imageSize;
        // image type
        gsSblUpdateState.ui32ImageType      = p_sUpdateImageData->imageType;
        // Get the size of the data without headers
        gsSblUpdateState.ui32DataSize       = gsSblUpdateState.ui32ImageSize - AM_DEVICES_COOPER_SBL_UPADTE_IMAGE_HDR_SIZE;
        // Get the start address of the data without headers
        gsSblUpdateState.pDataBuf           = gsSblUpdateState.pImageBuf + AM_DEVICES_COOPER_SBL_UPADTE_IMAGE_HDR_SIZE;
        // Calculate number of packets
        gsSblUpdateState.ui32TotalPackets   = gsSblUpdateState.ui32DataSize / AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
        if (  (gsSblUpdateState.ui32DataSize % AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE) != 0 )
        {
            gsSblUpdateState.ui32TotalPackets++;
        }
        gsSblUpdateState.ui32PacketNumber = 0;
        return true;
    }

    return false;
}
//*****************************************************************************
//
//  Initialize the Image Update state machine
//
//*****************************************************************************
uint32_t am_devices_cooper_image_update_init(void* pHandle, uint32_t* pWorkBuf)
{
    // Check for the input data validity
    if (pHandle != NULL)
    {
        // Initialize state machine
        gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_INIT;
        // Load the image address
        gsSblUpdateState.pImageBuf          = NULL;
        // Image size
        gsSblUpdateState.ui32ImageSize      = 0;
        // image type
        gsSblUpdateState.ui32ImageType      = AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_NONE;
        // Get the size of the data without headers
        gsSblUpdateState.ui32DataSize       = 0;
        // Get the start address of the data without headers
        gsSblUpdateState.pDataBuf           = NULL;
        // Calculate number of packets
        gsSblUpdateState.ui32TotalPackets   = 0;
        // Initialize Packet number in progress
        gsSblUpdateState.ui32PacketNumber = 0;
        //
        // Save cooper device handle
        //
        gsSblUpdateState.pHandle = pHandle;
        //
        // Work buffer reuse non-blocking work buffer.
        //
        gsSblUpdateState.pWorkBuf = pWorkBuf;
        // State is ready to go. Reset the cooper device
        return 0;
    }
    else
    {
        // Return with error
        return 1;
    }
}

//*****************************************************************************
//
// @breif Update Image
// @return uint32_t
//
//*****************************************************************************
uint32_t am_devices_cooper_update_image(void)
{
    uint32_t     ui32dataPktSize = 0;
    uint32_t     ui32Size        = 0;
    uint32_t     ui32Ret         = AM_DEVICES_COOPER_SBL_STATUS_INIT;
    am_sbl_host_msg_status_t*    psStatusMsg;
    am_sbl_host_msg_ack_nack_t*  psAckMsg;
    switch (gsSblUpdateState.ui32SblUpdateState)
    {
        case AM_DEVICES_COOPER_SBL_UPDATE_STATE_INIT:
            //
            // Send the "HELLO" message to connect to the interface.
            //
            send_hello(gsSblUpdateState.pHandle);
            gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_HELLO;
            // Tell application that we are not done with SBL
            ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
            break;
        case AM_DEVICES_COOPER_SBL_UPDATE_STATE_HELLO:
            // Read the "STATUS" response from the IOS and check for CRC Error
            if ( iom_slave_read(gsSblUpdateState.pHandle, (uint32_t*)gsSblUpdateState.pWorkBuf, &ui32Size) == false )
            {
                // Increment the Error Counter
                gsSblUpdateState.ui32ErrorCounter++;
                // Check if the Error has happened more than the limit
                if ( gsSblUpdateState.ui32ErrorCounter > AM_DEVICES_COOPER_SBL_MAX_COMM_ERR_COUNT )
                {
                    // Return fail
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
                else
                {
                    // Resend the previous message
                    send_hello(gsSblUpdateState.pHandle);
                    // Tell application that we are not done with SBL
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                }
            }
            else
            {
                // No CRC error and if there was one, then reset the error counter
                if ( gsSblUpdateState.ui32ErrorCounter )
                {
                    gsSblUpdateState.ui32ErrorCounter = 0;
                }
                // Check the status
                psStatusMsg = (am_sbl_host_msg_status_t*) (gsSblUpdateState.pWorkBuf);
                gsSblUpdateState.ui32CooperSblStatus = psStatusMsg->bootStatus;
                // Get the Cooper FW version
                if ( psStatusMsg->versionNumber == AM_DEVICES_COOPER_SBL_DEFAULT_FW_VERSION )
                {
                    gsSblUpdateState.ui32CooperFWImageVersion = 0;
                }
                else
                {
                    gsSblUpdateState.ui32CooperFWImageVersion = psStatusMsg->versionNumber;
                }
                am_util_stdio_printf("BLE Controller Info:\n");
                /*
                 * Before Cooper firmware version 1.19 (0x00000113), only the lower 16-bit of 32-bit Cooper firmware version
                 * word was used to identify Cooper firmware. It was limited to distinguish the difference of testing binaries.
                 * To restructure the Cooper firmware version to a.b.c.d from a.b may solve this problem.
                 * The higher 16-bit is used to identify the major and minor version of based release firmware.
                 * The lower 16-bit is used to identify the version for testing before next release.
                 * Originally the code only prints the lower 16-bit of FW version, need to print all the bytes
                 * based on new structure of firmware version now.
                 */
                if ((psStatusMsg->versionNumber & 0xFFFF0000) == 0)
                {
                    am_util_stdio_printf("\tFW Ver:      %d.%d\n", (psStatusMsg->versionNumber & 0xF00) >> 8, psStatusMsg->versionNumber & 0xFF);
                }
                else
                {
                    am_util_stdio_printf("\tFW Ver:      %d.%d.%d.%d\n", (psStatusMsg->versionNumber & 0xFF000000) >> 24, (psStatusMsg->versionNumber & 0xFF0000) >> 16,
                                                                        (psStatusMsg->versionNumber & 0xFF00) >> 8, psStatusMsg->versionNumber & 0xFF);
                }
                if (ui32Size == sizeof(am_sbl_host_msg_status_t))
                {
                    // Get the version rollback configuration
                    gsSblUpdateState.ui32CooperVerRollBackConfig = psStatusMsg->verRollBackStatus;
#if (SBL_DEBUG_LOG_ON == 1)
                    if ( psStatusMsg->verRollBackStatus == AM_DEVICES_COOPER_SBL_STAT_VER_ROLL_BACK_EN )
                    {
                        am_util_stdio_printf("Version RollBack Enabled \n");
                    }
                    else if ( psStatusMsg->verRollBackStatus == AM_DEVICES_COOPER_SBL_STAT_VER_ROLL_BACK_DBL )
                    {
                        am_util_stdio_printf("Version RollBack Disabled \n");
                    }
                    else
                    {
                        am_util_stdio_printf("Version RollBack Config invalid !!! \n");
                    }
#endif
                    am_util_stdio_printf("\tChip ID0:    0x%x\n", psStatusMsg->copperChipIdWord0);
                    am_util_stdio_printf("\tChip ID1:    0x%x\n\n", psStatusMsg->copperChipIdWord1);

                    gsSblUpdateState.ui32copperChipIdWord0 = psStatusMsg->copperChipIdWord0;
                    gsSblUpdateState.ui32copperChipIdWord1 = psStatusMsg->copperChipIdWord1;

                }
                else
                {
                    gsSblUpdateState.ui32CooperVerRollBackConfig = 0x0;
                }
                #if (SBL_DEBUG_LOG_ON == 1)
                am_util_stdio_printf("BLE Controller SBL Status:  0x%x\n", sbl_status);
                am_util_stdio_printf("bootStatus 0x%x\n", psStatusMsg->bootStatus);
                #endif
                // check if the Boot Status is success
                if ( psStatusMsg->bootStatus == AM_DEVICES_COOPER_SBL_STAT_RESP_SUCCESS )
                {
                    // Check if we have some FW available
                    if (  am_devices_cooper_sbl_update_state_data(AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_FW) == true )
                    {
                        // Check if we have a newer FW version
                        if ( psStatusMsg->versionNumber < g_sFwImage.version )
                        {
                            // We have newer FW available, Letus upgrade
                            ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_FW;
                            if ((g_sFwImage.version & 0xFFFF0000) == 0)
                            {
                                am_util_stdio_printf("Received new BLE Controller FW version = %d.%d Going for upgrade\n", (g_sFwImage.version & 0xF00) >> 8, g_sFwImage.version & 0xFF);
                            }
                            else
                            {
                                am_util_stdio_printf("Received new BLE Controller FW version = %d.%d.%d.%d Going for upgrade\n", (g_sFwImage.version & 0xFF000000) >> 24, (g_sFwImage.version & 0xFF0000) >> 16,
                                                                                                                            (g_sFwImage.version & 0xFF00) >> 8, g_sFwImage.version & 0xFF);
                            }
                        }
                    }
                    // If we don't have any FW or any newer FW then continue with the current FW in Cooper
                    if ( ui32Ret != AM_DEVICES_COOPER_SBL_STATUS_UPDATE_FW )
                    {
                        // We don't have any other FW, so continue with one already there is Cooper device
                        gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_IMAGE_OK;
                        // Not done yet
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                        am_util_stdio_printf("No new image to upgrade\n");
                        // Send the command to continue to FW
                        send_fwContinue(gsSblUpdateState.pHandle);
                        am_util_stdio_printf("BLE Controller FW Auth Passed, Continue with FW\n");
                    }
                }
                else if ( psStatusMsg->bootStatus == AM_DEVICES_COOPER_SBL_STAT_RESP_FW_UPDATE_REQ )
                {
                    am_util_stdio_printf("BLE Controller Requires FW\n");
                    if (  am_devices_cooper_sbl_update_state_data(AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_FW) == true )
                    {
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_FW;
                    }
                    else
                    {
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_IMAGE_FAIL;
                    }
                }
                else if ( psStatusMsg->bootStatus == AM_DEVICES_COOPER_SBL_STAT_RESP_INFO0_UPDATE_REQ )
                {
                    am_util_stdio_printf("BLE Controller Requires INFO 0\n");
                    if ( am_devices_cooper_sbl_update_state_data(AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_0) == true )
                    {
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_INFO_0;
                    }
                    else
                    {
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_IMAGE_FAIL;
                    }
                }
                else if ( psStatusMsg->bootStatus == AM_DEVICES_COOPER_SBL_STAT_RESP_INFO1_UPDATE_REQ )
                {
                    am_util_stdio_printf("BLE Controller Requires INFO 1\n");
                    if ( am_devices_cooper_sbl_update_state_data(AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_1) == true )
                    {
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_INFO_1;
                    }
                    else
                    {
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_IMAGE_FAIL;
                    }
                }
                else
                {
                    am_util_stdio_printf("BLE Controller Wrong Response\n");
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
            }
            if (  (ui32Ret == AM_DEVICES_COOPER_SBL_STATUS_OK) || (ui32Ret == AM_DEVICES_COOPER_SBL_STATUS_FAIL) ||
                  (gsSblUpdateState.ui32SblUpdateState == AM_DEVICES_COOPER_SBL_UPDATE_STATE_IMAGE_OK) )
            {
                // Do nothing
            }
            else
            {
                // for the case ui32Ret == AM_DEVICES_COOPER_SBL_STATUS_UPDATE_IMAGE_FAIL,
                // it indicates Cooper has available FW/Info0/Info1 signature and requests update,
                // but Apollo4 does not have such image/patch at this moment, gsSblUpdateState.ui32ImageSize should be zero.
                // Need to send_update with invalid parameter to let Cooper reply NACK and clear signature

                // Update the state machine
                gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_UPDATE;
                // Send the update message
                send_update(gsSblUpdateState.pHandle, gsSblUpdateState.ui32ImageSize);
                ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
            }
            break;
        case AM_DEVICES_COOPER_SBL_UPDATE_STATE_UPDATE:
            // Read the "ACK/NACK" response from the IOS and check for CRC Error
            if ( iom_slave_read(gsSblUpdateState.pHandle, (uint32_t*)gsSblUpdateState.pWorkBuf, &ui32Size) == false )
            {
                // Increment the Error Counter
                gsSblUpdateState.ui32ErrorCounter++;
                // Check if the Error has happened more than the limit
                if ( gsSblUpdateState.ui32ErrorCounter > AM_DEVICES_COOPER_SBL_MAX_COMM_ERR_COUNT )
                {
                    // Return fail
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
                else
                {
                    // Resend the previous message
                    send_update(gsSblUpdateState.pHandle, gsSblUpdateState.ui32ImageSize);
                    // Tell application that we are not done with SBL
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                }
            }
            else
            {
                // No CRC error and if there was one, then reset the error counter
                if ( gsSblUpdateState.ui32ErrorCounter )
                {
                    gsSblUpdateState.ui32ErrorCounter = 0;
                }
                // Get the response status
                psAckMsg = (am_sbl_host_msg_ack_nack_t*)(gsSblUpdateState.pWorkBuf);
                // Process the response
                if ( (psAckMsg->msgHdr.msgType == AM_SBL_HOST_MSG_ACK) && (NULL != gsSblUpdateState.pImageBuf))
                {
                    // Save the status
                    gsSblUpdateState.ui32CooperSblStatus = psAckMsg->status;
                    // Change the state
                    gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_DATA;
                    // Send the Encrypted image header - first 64 bytes
                    send_data(gsSblUpdateState.pHandle, (uint32_t)gsSblUpdateState.pImageBuf,
                            AM_DEVICES_COOPER_SBL_UPADTE_IMAGE_HDR_SIZE, gsSblUpdateState.ui32PacketNumber);
                    am_util_stdio_printf("BLE controller upgrade in progress, wait...\n");
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                }
                else if ( (psAckMsg->msgHdr.msgType == AM_SBL_HOST_MSG_NACK) && (psAckMsg->status == AM_DEVICES_COOPER_SBL_ACK_RESP_INVALID_PARAM) )
                {
                    am_util_stdio_printf("Clear Cooper Signature, reset Cooper and talk with SBL again\n");
                    // Add some delay for Cooper SBL to clear signature
                    am_util_delay_ms(1200);
                    am_devices_cooper_reset();
                    gsSblUpdateState.pImageBuf        = NULL;
                    gsSblUpdateState.ui32ImageSize    = 0;
                    gsSblUpdateState.ui32ImageType    = AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_NONE;
                    gsSblUpdateState.ui32DataSize     = 0;
                    gsSblUpdateState.pDataBuf         = NULL;
                    gsSblUpdateState.ui32TotalPackets = 0;
                    gsSblUpdateState.ui32PacketNumber = 0;

                    // Send the "HELLO" message to connect to the interface.
                    send_hello(gsSblUpdateState.pHandle);
                    gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_HELLO;
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                }
                else
                {
                    am_util_stdio_printf("Update Failed !!!\n");
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
            }
            break;
        case AM_DEVICES_COOPER_SBL_UPDATE_STATE_DATA:
            // Read the "ACK/NACK" response from the IOS.
            if ( iom_slave_read(gsSblUpdateState.pHandle, (uint32_t*)gsSblUpdateState.pWorkBuf, &ui32Size) == false )
            {
                // Increment the Error Counter
                gsSblUpdateState.ui32ErrorCounter++;
                // Check if the Error has happened more than the limit
                if ( gsSblUpdateState.ui32ErrorCounter > AM_DEVICES_COOPER_SBL_MAX_COMM_ERR_COUNT )
                {
                    // Return fail
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
                else
                {
                    // Resend the previous message
                    if ( gsSblUpdateState.ui32PacketNumber == 0 )
                    {
                        // Send the Encrypted image header - first 64 bytes
                        send_data(gsSblUpdateState.pHandle, (uint32_t)gsSblUpdateState.pImageBuf,
                                  AM_DEVICES_COOPER_SBL_UPADTE_IMAGE_HDR_SIZE, gsSblUpdateState.ui32PacketNumber);
                    }
                    else
                    {
                        // Reset the packet counters to the previous ones, to resend the packet
                        //gsSblUpdateState.ui32TotalPackets++;
                        // increment the packet number as we have already sent the header
                        //gsSblUpdateState.ui32PacketNumber--;
                        //Check if this is the last packet - Increase by one as we have already decremented after TX
                        if (  (gsSblUpdateState.ui32TotalPackets + 1) == 1 )
                        {
                            // Get the size of the leftover data
                            ui32dataPktSize = gsSblUpdateState.ui32DataSize % AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                            if (ui32dataPktSize == 0)
                            {
                                ui32dataPktSize = AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                            }
                        }
                        else
                        {
                            ui32dataPktSize = AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                        }
                        // Resend the same packet - Need to decrement the packet numbers as those are already incremented
                        send_data(gsSblUpdateState.pHandle, (uint32_t) gsSblUpdateState.pDataBuf + ( (gsSblUpdateState.ui32PacketNumber - 1) * AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE),
                                  ui32dataPktSize, gsSblUpdateState.ui32PacketNumber);
                    }
                    // Tell application that we are not done with SBL
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                }
            }
            else
            {
                // No CRC error and if there was one, then reset the error counter
                if ( gsSblUpdateState.ui32ErrorCounter )
                {
                    gsSblUpdateState.ui32ErrorCounter = 0;
                }
                // Get the response status
                psAckMsg = (am_sbl_host_msg_ack_nack_t*)(gsSblUpdateState.pWorkBuf);
                // Save the status
                gsSblUpdateState.ui32CooperSblStatus = psAckMsg->status;
                if (  (psAckMsg->srcMsgType == AM_SBL_HOST_MSG_DATA ) || (psAckMsg->srcMsgType == AM_SBL_HOST_MSG_UPDATE_STATUS) )
                {
                    if (  (psAckMsg->status == AM_DEVICES_COOPER_SBL_ACK_RESP_SUCCESS) || (psAckMsg->status == AM_DEVICES_COOPER_SBL_ACK_RESP_SEQ) )
                    {
                        if ( gsSblUpdateState.ui32TotalPackets > 0 )
                        {
                            //Check if this is the last packet
                            if ( gsSblUpdateState.ui32TotalPackets == 1 )
                            {
                                // Get the size of the left over data
                                ui32dataPktSize = gsSblUpdateState.ui32DataSize % AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                                if (ui32dataPktSize == 0)
                                {
                                    ui32dataPktSize = AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                                }
                            }
                            else
                            {
                                ui32dataPktSize = AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                            }
                            send_data(gsSblUpdateState.pHandle, (uint32_t) gsSblUpdateState.pDataBuf + (gsSblUpdateState.ui32PacketNumber * AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE),
                                      ui32dataPktSize, gsSblUpdateState.ui32PacketNumber + 1);
                            gsSblUpdateState.ui32TotalPackets--;
                            // increment the packet number as we have already sent the header
                            gsSblUpdateState.ui32PacketNumber++;
                            ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                        }
                        else
                        {
                            if ( psAckMsg->status == AM_DEVICES_COOPER_SBL_ACK_RESP_SUCCESS )
                            {
                                // If FW is updated successfuly, then jump to BLE image
                                if ( gsSblUpdateState.ui32ImageType == AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_FW )
                                {
                                    gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_IMAGE_OK;
                                    gsSblUpdateState.ui32CooperFWImageVersion = g_sFwImage.version;
                                    // Not done yet
                                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                                    // Send the command to continue to FW
                                    send_fwContinue(gsSblUpdateState.pHandle);
                                    // If INFO 0 or INFO 1 is updated successfully, the apply send reset
                                }
                                else
                                {
                                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_OK;
                                }
                            }
                            else
                            {
                                am_util_stdio_printf("Update fails status = 0x%x\n", psAckMsg->status);
                                ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                            }
                        }
                    }
                    else
                    {
                        am_util_stdio_printf("Update fails status = 0x%x\n", psAckMsg->status);
                        // We have received NACK
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                    }
                }
                else
                {
                    // Wrong Response type
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
            }
            break;
        case AM_DEVICES_COOPER_SBL_UPDATE_STATE_IMAGE_OK:
        {
            // Read the "ACK/NACK" response from the IOS and check for CRC Error
            if ( iom_slave_read(gsSblUpdateState.pHandle, (uint32_t*)gsSblUpdateState.pWorkBuf, &ui32Size) == false )
            {
                // Increment the Error Counter
                gsSblUpdateState.ui32ErrorCounter++;
                // Check if the Error has happened more than the limit
                if ( gsSblUpdateState.ui32ErrorCounter > AM_DEVICES_COOPER_SBL_MAX_COMM_ERR_COUNT )
                {
                    // Return fail
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
                else
                {
                    // Resend the previous message
                    send_fwContinue(gsSblUpdateState.pHandle);
                    // Tell application that we are not done with SBL
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                }
            }
            else
            {
                // No CRC error and if there was one, then reset the error counter
                if ( gsSblUpdateState.ui32ErrorCounter )
                {
                    gsSblUpdateState.ui32ErrorCounter = 0;
                }
            }
            // Get the response status
            psAckMsg = (am_sbl_host_msg_ack_nack_t*)(gsSblUpdateState.pWorkBuf);
            // Save the status
            gsSblUpdateState.ui32CooperSblStatus = psAckMsg->status;
            if ( psAckMsg->status == AM_DEVICES_COOPER_SBL_ACK_RESP_SUCCESS )
            {
                // FW has gone to BLE, end the SBL driver state machine
                ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_OK;
            }
            else
            {
                ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
            }
        }
        break;
        default:
            // Bad state, update the state machine
            break;
    }
    return ui32Ret;
}

//*****************************************************************************
//
//  Get cooper firmware image from local binary
//
//*****************************************************************************
bool am_devices_cooper_get_FwImage(am_devices_cooper_sbl_update_data_t *pFwImage )
{
    if (pFwImage != NULL)
    {
        memcpy(&g_sFwImage, pFwImage, sizeof(am_devices_cooper_sbl_update_data_t));
        // Get version from the firmware image
        g_sFwImage.version = (pFwImage->pImageAddress[27] << 24) | (pFwImage->pImageAddress[26] << 16) | (pFwImage->pImageAddress[25] << 8) | (pFwImage->pImageAddress[24]);
    }

    return (pFwImage != NULL);
}

//*****************************************************************************
//
//  Get cooper info1 image from local binary
//
//*****************************************************************************
bool am_devices_cooper_get_info1_patch(am_devices_cooper_sbl_update_data_t *pInfo1Image)
{
    if (pInfo1Image != NULL)
    {
        memcpy(&g_sInfo1PatchImage, pInfo1Image, sizeof(am_devices_cooper_sbl_update_data_t));
    }

    return (pInfo1Image != NULL);
}

//*****************************************************************************
//
//  Get cooper info0 image from local binary
//
//*****************************************************************************
bool am_devices_cooper_get_info0_patch(am_devices_cooper_sbl_update_data_t *pInfo0Image)
{
    if (pInfo0Image != NULL)
    {
        memcpy(&g_sInfo0PatchImage, pInfo0Image, sizeof(am_devices_cooper_sbl_update_data_t));
    }

    return (pInfo0Image != NULL);
}

//*****************************************************************************
//
//  Reset the BLE controller and check if there's request to update
//
//*****************************************************************************
uint32_t am_devices_cooper_reset_with_sbl_check(void* pHandle, am_devices_cooper_config_t* pDevConfig)
{
    uint32_t u32SblStatus = 0;
    am_devices_cooper_t *pBle = (am_devices_cooper_t *)pHandle;
    am_devices_cooper_reset();
    am_devices_cooper_image_update_init(pHandle, pDevConfig->pNBTxnBuf);
    u32SblStatus = AM_DEVICES_COOPER_SBL_STATUS_INIT;
    u32SblStatus = am_devices_cooper_update_image();
    while ( (u32SblStatus != AM_DEVICES_COOPER_SBL_STATUS_OK) && ( u32SblStatus != AM_DEVICES_COOPER_SBL_STATUS_FAIL) )
    {
        while (am_devices_cooper_irq_read() == 0)
        {
            am_hal_delay_us(50);
        }
        u32SblStatus = am_devices_cooper_update_image();
    }
    //
    // Return the status.
    //
    if (u32SblStatus == AM_DEVICES_COOPER_SBL_STATUS_OK)
    {
        // The CS assertation duration optimization only takes effect after V1.14
        if ( gsSblUpdateState.ui32CooperFWImageVersion < 0x10E )
        {
            pBle->ui32CSDuration = 300;
        }
        pBle->ui32Firmver = gsSblUpdateState.ui32CooperFWImageVersion;
        // need to wait a bit to jump from SBL to Cooper application firmware
        am_util_delay_ms(10);
        am_util_stdio_printf("Update Done\r\n");
        return AM_DEVICES_COOPER_STATUS_SUCCESS;
    }
    else
    {
        // free up resource that won't be used.
        am_devices_cooper_term(pHandle);
        am_util_stdio_printf("BLE Controller SBL Error 0x%x\r\n", u32SblStatus);
        return u32SblStatus;
    }
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

