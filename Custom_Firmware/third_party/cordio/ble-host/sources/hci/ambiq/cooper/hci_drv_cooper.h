//*****************************************************************************
//
//! @file hci_drv_cooper.h
//!
//! @brief Support functions for the AMBIQ BTLE radio.
//
//*****************************************************************************
#include <stdbool.h>
#include "wsf_os_int.h"
#include "wsf_os.h"
#include "am_devices_cooper.h"

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
#ifndef HCI_DRV_COOPER_H
#define HCI_DRV_COOPER_H

//*****************************************************************************
//
//*****************************************************************************
//
// Disable the NOP(None Opcode) HCI event mechanism,
// it needs to enable macro ENABLE_SPECIFIED_EVENT_MASK at the same time
//
// Setting this macro to 1 will disable NOP HCI event sent from controller
// when it' awakened up. And HOST will check and send out any pending HCI
// packets if BLE core wakeup interrupt is not coming up within
// WAKEUP_TIMEOUT_MS milliseconds.

// This feature is dsiabled by default, which means there will be NOP event coming up
// when controller wakes up.
//*****************************************************************************
#define DISABLE_WAKEUP_NOP_EVENT          0

// This is to enable specified event from controller, default disabled
#define ENABLE_SPECIFIED_EVENT_MASK       0

//*****************************************************************************
//
// AMBIQ vendor specific events
//
//*****************************************************************************
// Tx power level in dBm.
typedef enum
{
    TX_POWER_LEVEL_MINUS_20P0_dBm,
    TX_POWER_LEVEL_MINUS_15P0_dBm,
    TX_POWER_LEVEL_MINUS_10P0_dBm,
    TX_POWER_LEVEL_MINUS_5P0_dBm,
    TX_POWER_LEVEL_0P0_dBm,
    TX_POWER_LEVEL_PLUS_3P0_dBm,
    TX_POWER_LEVEL_PLUS_4P0_dBm,
    TX_POWER_LEVEL_PLUS_6P0_dBm,
    TX_POWER_LEVEL_INVALID,
}txPowerLevel_t;

// Set the default BLE TX Output power to +0dBm.
#define TX_POWER_LEVEL_DEFAULT TX_POWER_LEVEL_0P0_dBm

// For FCC continous wave testing
#define PAYL_CONTINUOUS_WAVE        0x10
// For FCC continuous modulation testing
#define PAYL_CONTINUOUS_MODULATE    0x11

#define CHL_2402_INDEX    0        // low frequency
#define CHL_2440_INDEX    19       // medium frequency
#define CHL_2480_INDEX    39       // high frequency

#define MAX_MEM_ACCESS_SIZE   128
#define MAX_FLASH_ACCESS_SIZE   128

typedef enum
{
    /// PLATFORM RESET REASON: Reset and load FW from flash
    PLATFORM_RESET_TO_FW        = 0,
    /// PLATFORM RESET REASON: Reset and stay in ROM code
    PLATFORM_RESET_TO_ROM       = 1,
}ePlfResetReason_type;

typedef enum
{
    EVT_MASK_NULL              = 0x00000000,
    /// configure to disable no opreation command code event
    DISABLE_WAKEUP_NOP_EVT_MASK  = 0x00000001,
}eCfgEvtMsk_type;

// configuration for specified event mask
#define CFG_EVENT_MASK  (DISABLE_WAKEUP_NOP_EVT_MASK)

/*! read memory variable command */
typedef struct
{
    ///Start address to read
    uint32_t start_addr;
    ///Access type
    uint8_t type;
    ///Length to read
    uint8_t length;
} __attribute__ ((__packed__))hciRdMemCmd_t;


/*! write memory variable command */
typedef struct
{
    ///Start address to read
    uint32_t start_addr;
    ///Access type
    uint8_t type;
    ///Length to write
    uint8_t length;
    uint8_t data[MAX_MEM_ACCESS_SIZE];
} __attribute__ ((__packed__))hciWrMemCmd_t;

typedef struct
{
    ///Flash type
    uint8_t flashtype;
    ///Start offset address
    uint32_t startoffset;
    ///Length to erase
    uint32_t length;
} __attribute__ ((__packed__))hciErFlashCmd_t;

typedef struct
{
    ///Flash type
    uint8_t flashtype;
    ///Start offset address
    uint32_t startoffset;
    ///Length to write
    uint8_t length;
    uint8_t data[MAX_FLASH_ACCESS_SIZE];
} __attribute__ ((__packed__))hciWrFlashCmd_t;

typedef struct
{
    ///Flash type
    uint8_t flashtype;
    ///Start offset address
    uint32_t startoffset;
    ///Length to read
    uint8_t length;
} __attribute__ ((__packed__))hciRdFlashCmd_t;

typedef struct
{
    /// register address
    uint32_t addr;
} hciRegRdCmd_t;

typedef struct
{
    /// register address
    uint32_t addr;
    /// register value
    uint32_t value;
} hciRegWrCmd_t;

typedef struct
{
    /// reason
    uint8_t reason;
} hciPlfResetCmd_t;

#define LL_FEATURES_BYTE0  ( HCI_LE_SUP_FEAT_ENCRYPTION  \
                                 | HCI_LE_SUP_FEAT_CONN_PARAM_REQ_PROC \
                                 | HCI_LE_SUP_FEAT_EXT_REJECT_IND \
                                 | HCI_LE_SUP_FEAT_SLV_INIT_FEAT_EXCH \
                                 | HCI_LE_SUP_FEAT_LE_PING \
                                 | HCI_LE_SUP_FEAT_DATA_LEN_EXT \
                                 | HCI_LE_SUP_FEAT_PRIVACY \
                                 | HCI_LE_SUP_FEAT_EXT_SCAN_FILT_POLICY )

#define LL_FEATURES_BYTE1  ( HCI_LE_SUP_FEAT_LE_2M_PHY \
                             | HCI_LE_SUP_FEAT_LE_EXT_ADV \
                             | HCI_LE_SUP_FEAT_LE_PER_ADV \
                             | HCI_LE_SUP_FEAT_CH_SEL_2 )

#define LL_FEATURES_BYTE2  ( HCI_LE_SUP_FEAT_MIN_NUN_USED_CHAN \
                             | HCI_LE_SUP_FEAT_CONN_CTE_REQ \
                             | HCI_LE_SUP_FEAT_CONN_CTE_RSP \
                             | HCI_LE_SUP_FEAT_CONNLESS_CTE_TRANS \
                             | HCI_LE_SUP_FEAT_CONNLESS_CTE_RECV \
                             | HCI_LE_SUP_FEAT_ANTENNA_SWITCH_AOD \
                             | HCI_LE_SUP_FEAT_ANTENNA_SWITCH_AOA \
                             | HCI_LE_SUP_FEAT_RECV_CTE )

#define LL_FEATURES_BYTE3  (HCI_LE_SUP_FEAT_PAST_SENDER \
                             | HCI_LE_SUP_FEAT_PAST_RECIPIENT \
                             | HCI_LE_SUP_FEAT_SCA_UPDATE \
                             | HCI_LE_SUP_FEAT_REMOTE_PUB_KEY_VALIDATION  )

#define MIN_SWITCHING_PATTERN_LEN  (0x02)
#define TEST_LEN_DEFAULT        (0x25)

//*****************************************************************************
//
// Hci driver functions unique to Cooper
//
//*****************************************************************************
extern void HciDrvHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);
extern void HciDrvHandlerInit(wsfHandlerId_t handlerId);
extern bool HciVscSetRfPowerLevelEx(txPowerLevel_t txPowerlevel);
extern void HciVscUpdateFw(uint32_t update_sign);


extern bool HciVscReadMem(uint32_t start_addr, eMemAccess_type access_type, uint8_t length);
extern bool HciVscWriteMem(uint32_t start_addr, eMemAccess_type access_type, uint8_t length, uint8_t *data);
extern void HciVscGetFlashId(void);
extern void HciVscEraseFlash(uint8_t flash_type, uint32_t offset,uint32_t length);
extern bool HciVscWriteFlash(uint8_t flash_type, uint32_t offset, uint8_t length, uint8_t *data);
extern bool HciVscReadFlash(uint8_t flash_type, uint32_t offset, uint8_t length);
extern void HciVscReadReg(uint32_t reg_addr);
extern void HciVscWriteReg(uint32_t reg_addr, uint32_t value);
extern void HciVscPlfReset(ePlfResetReason_type reason);
extern void HciVscUpdateBDAddress(void);
extern bool_t HciVscSetCustom_BDAddr(uint8_t *bd_addr);
void HciVscUpdateNvdsParam(void);
void HciVscUpdateLinklayerFeature(void);
void HciVscGetDtmRssi(void);
void HciVscConfigEvtMask(uint32_t evt_mask);

#endif // HCI_DRV_COOPER_H
