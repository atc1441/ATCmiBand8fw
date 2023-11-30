//*****************************************************************************
//
//! @file am_hal_card_host.h
//!
//! @brief Functions for interfacing with the SDHC or SPI SD/MMC/SDIO card host.
//!
//! @addtogroup card_host_4p Card Host for SD/MMC/eMMC/SDIO
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
#ifndef AM_HAL_CARD_HOST_H
#define AM_HAL_CARD_HOST_H

#ifdef __cplusplus
extern "C"
{
#endif

//
//! SD/MMC/SDIO host instance index
//
typedef enum
{
    AM_HAL_SDHC_CARD_HOST,
    AM_HAL_CARD_HOST_NUM ,
} am_hal_host_inst_index_e;

//
//! Command Errors Types
//
typedef enum
{
    AM_HAL_CMD_ERR_NONE,
    AM_HAL_CMD_ERR_INHIBIT,
    AM_HAL_CMD_ERR_TIMEOUT,
    AM_HAL_CMD_ERR_INDEX,
    AM_HAL_CMD_ERR_CRC,
    AM_HAL_CMD_ERR_ENDBIT,
    AM_HAL_CMD_ERR_NO_RESPONSE,
} am_hal_card_cmd_err_e;

//
//! Data Errors Types
//
typedef enum
{
    AM_HAL_DATA_ERR_NONE,
    AM_HAL_DATA_ERR_ADMAERROR,
    AM_HAL_DATA_ERR_DATACRCERROR,
    AM_HAL_DATA_ERR_DATATIMEOUTERROR,
    AM_HAL_DATA_ERR_DATAENDBITERROR,
    AM_HAL_DATA_ERR_TIMEOUT,
} am_hal_card_data_err_e;

//
//! Data Direction
//
typedef enum
{
    AM_HAL_DATA_DIR_READ,
    AM_HAL_DATA_DIR_WRITE,
} am_hal_data_dir_e;

//
//! SDHC bus Voltage
//
typedef enum
{
    AM_HAL_HOST_BUS_VOLTAGE_1_8,
    AM_HAL_HOST_BUS_VOLTAGE_3_0,
    AM_HAL_HOST_BUS_VOLTAGE_3_3,
} am_hal_host_bus_voltage_e;

//
//! SDHC bus width
//
typedef enum
{
    AM_HAL_HOST_BUS_WIDTH_1 = 1,
    AM_HAL_HOST_BUS_WIDTH_4 = 4,
    AM_HAL_HOST_BUS_WIDTH_8 = 8,
} am_hal_host_bus_width_e;

//
//! SDHC bus transfer mode
//
typedef enum
{
    AM_HAL_HOST_XFER_DEFAULT,
    AM_HAL_HOST_XFER_PIO,
    AM_HAL_HOST_XFER_SDMA,
    AM_HAL_HOST_XFER_ADMA,
} am_hal_host_xfer_mode_e;

//
//! SDHC bus UHS mode
//
typedef enum
{
    AM_HAL_HOST_UHS_NONE = 0,
    AM_HAL_HOST_UHS_SDR12 = 0,
    AM_HAL_HOST_UHS_SDR25 = 1,
    AM_HAL_HOST_UHS_SDR50 = 2,
    AM_HAL_HOST_UHS_SDR104 = 3,
    AM_HAL_HOST_UHS_DDR50 = 4,
} am_hal_host_uhs_mode_e;

//
//! Command Data Struct
//
typedef struct
{
    am_hal_data_dir_e dir;
    am_hal_host_xfer_mode_e eXferMode;
    bool bNotUseDataLine;
    uint8_t *pui8Buf;
    uint32_t ui32BlkSize;
    uint32_t ui32BlkCnt;
    am_hal_card_data_err_e eDataError;
} am_hal_card_cmd_data_t;

//
//! Command Struct
//
typedef struct
{
    uint8_t  ui8Idx;
    uint32_t ui32Arg;
    uint32_t ui32RespType;
    bool bCheckBusyCmd;
    bool bASync;
    bool bAutoCMD12;
    bool bAutoCMD23;
    uint32_t ui32Resp[4];
    am_hal_card_cmd_err_e eError;
} am_hal_card_cmd_t;

typedef struct am_hal_card_host_ops am_hal_card_host_ops_t;

//
//! SDHC Event Type
//
typedef enum
{
    AM_HAL_EVT_CARD_NOT_PRESENT,
    AM_HAL_EVT_CARD_PRESENT,
    AM_HAL_EVT_PIO_DONE,
    AM_HAL_EVT_SDMA_DONE,
    AM_HAL_EVT_XFER_COMPLETE,
    AM_HAL_EVT_CMD_ERR,
    AM_HAL_EVT_DAT_ERR,
} am_hal_host_evt_type_e;

//
//! SDHC Host Event Struct
//
typedef struct
{
    am_hal_host_evt_type_e eType;
    uint32_t ui32BlkCnt;
    void *pCtx;
} am_hal_host_evt_t;

typedef void (*am_hal_host_event_cb_t)(am_hal_host_evt_t *pEvt);

//
//! SDHC Host Struct
//
typedef struct am_hal_card_host
{
    void *pHandle;
    uint32_t ui32Module;
    bool bCardInSlot;
    bool bInited;
    am_hal_host_xfer_mode_e eXferMode;
    am_hal_host_bus_width_e eBusWidth;
    am_hal_host_bus_voltage_e eBusVoltage;
    am_hal_host_uhs_mode_e eUHSMode;
    uint32_t ui32MaxADMA2BlkNums;
    uint32_t ui32Clock;
    uint8_t  ui8Version;
    uint32_t ui32MaxClock;
    uint32_t ui32MinClock;
    uint32_t ui32OCRAvail;
    am_hal_card_cmd_t AsyncCmd;
    am_hal_card_cmd_data_t AsyncCmdData;
    am_hal_card_host_ops_t *ops;
    am_hal_host_event_cb_t pfunEvtCallback;
} am_hal_card_host_t;

//
//! SDHC Config
//
typedef struct am_hal_card_cfg_t
{
    uint32_t ui32Clock;
    am_hal_host_bus_width_e eBusWidth;
    am_hal_host_bus_voltage_e eIoVoltage;
    am_hal_host_uhs_mode_e eUHSMode;
} am_hal_card_cfg_t;

//
//! SDHC Host Operations
//
struct am_hal_card_host_ops
{
    uint32_t (*init)(am_hal_card_host_t *pHost);
    uint32_t (*deinit)(void *pHandle);
    uint32_t (*pwr_ctrl)(void *pHandle, bool bOnOff);
    uint32_t (*execute_cmd)(void *pHandle, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pData);
    uint32_t (*set_bus_voltage)(void *pHandle, am_hal_host_bus_voltage_e eBusVoltage);
    uint32_t (*set_bus_width)(void *pHandle, am_hal_host_bus_width_e eBusWidth);
    uint32_t (*set_bus_clock)(void *pHandle, uint32_t ui32Clock);
    uint32_t (*set_uhs_mode)(void *pHandle, am_hal_host_uhs_mode_e eUHSMode);
    void (*set_txrx_delay)(void *pHandle, uint8_t ui8TxRxDelays[2]);
    bool (*get_cd)(void *pHandle);
    uint32_t (*card_busy)(void *pHandle, uint32_t ui32TimeoutMS);
};

//*****************************************************************************
//
//! @brief Get the card host instance function
//!
//! @param eIndex       - index to the underlying card host instance.
//! @param bReInit      - flag that controling the reinitialization of the card host.
//!
//! This function will find a card host instance and trying to initialize it. if
//! card host's initialization succeeds, return a pointer to the instance. otherwise
//! a NULL pointer is returned.
//!
//! @return status      - NULL or a pointer to the card host instance.
//
//*****************************************************************************
extern am_hal_card_host_t *am_hal_get_card_host(am_hal_host_inst_index_e eIndex, bool bReInit);

//*****************************************************************************
//
//! @brief set the card host the transfer mode
//!
//! @param pHost       - a pointer to the card host instance
//! @param eXferMode   - the default transfer mode, like PIO, SDMA or ADMA.
//!
//! This function sets the default transfer mode like PIO, SDMA or ADMA.
//
//*****************************************************************************
extern void am_hal_card_host_set_xfer_mode(am_hal_card_host_t *pHost, am_hal_host_xfer_mode_e eXferMode);

//*****************************************************************************
//
//! @brief set the card host the TX/RX delay if card host has this feature
//!
//! @param pHost         - a pointer to the card host instance
//! @param ui8TxRxDelays - the TX/RX setting from the 'am_hal_card_emmc_calibrate'
//!
//! This function sets the TX/RX delay setting if card host card host has this feature
//
//*****************************************************************************
extern void am_hal_card_host_set_txrx_delay(am_hal_card_host_t *pHost, uint8_t ui8TxRxDelays[2]);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CARD_HOST_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

