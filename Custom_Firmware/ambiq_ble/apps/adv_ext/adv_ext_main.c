//*****************************************************************************
//
//! @file adv_ext_main.c
//!
//! @brief Ambiq Micro's demonstration of Extended Advertising.
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

#include <string.h>
#include "wsf_types.h"
#include "util/bstream.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "hci_api.h"
#include "dm_api.h"
#include "att_api.h"
#include "smp_api.h"
#include "app_api.h"
#include "app_db.h"
#include "app_ui.h"
#include "app_hw.h"
#include "app_main.h"
#include "svc_ch.h"
#include "svc_core.h"
#include "svc_hrs.h"
#include "svc_dis.h"
#include "svc_batt.h"
#include "svc_rscs.h"
#include "gatt/gatt_api.h"
#include "bas/bas_api.h"
#include "hrps/hrps_api.h"
#include "rscp/rscp_api.h"
#include "adv_ext_api.h"
#include "atts_main.h"
/**************************************************************************************************
  Macros
**************************************************************************************************/
/* used to enable periodic advertising testing */
#define PERIOD_ADV_TEST_EN    0

/* used to enable connectionless AoA/AoD testing
*  NEED to enable PERIOD_ADV_TEST_EN macro
*/
#define CONLESS_AOA_AOD_TEST_EN  0

/*! WSF message event starting value */
#define ADV_EXT_MSG_START               0xA0
#define ADV_BUF_LEN_MAX                 (252*5)
#define CONLESS_CTE_CNT                  1
#define CONLESS_CTE_LEN                  (0x2)
/* Default Running Speed and Cadence Measurement period (seconds) */
#define ADV_EXT_DEFAULT_RSCM_PERIOD        1

/*! WSF message event enumeration */
enum
{
  ADV_EXT_HR_TIMER_IND = ADV_EXT_MSG_START,       /*! Heart rate measurement timer expired */
  ADV_EXT_BATT_TIMER_IND,                     /*! Battery measurement timer expired */
  ADV_EXT_RUNNING_TIMER_IND                   /*! Running speed and cadence measurement timer expired */
};

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! Application message type */
typedef union
{
  wsfMsgHdr_t     hdr;
  dmEvt_t         dm;
  attsCccEvt_t    ccc;
  attEvt_t        att;
} advExtMsg_t;

/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/

appExtAdvCfg_t advExtAdvCfg =
{
#if (DM_NUM_ADV_SETS == 1)
    {0},
    {800},
    {0},
    {FALSE},
    {0},
#elif (DM_NUM_ADV_SETS > 1)
    {0, 0},
    {800, 800},
    {0, 0},
    {FALSE, TRUE},
#if (PERIOD_ADV_TEST_EN == 1)
    {800, 800},
#else
    {0, 0}
#endif
#endif

};

uint8_t adv_handle[DM_NUM_ADV_SETS] = {0};
extern appExtConnCb_t appExtConnCb[DM_CONN_MAX];

#if (DM_NUM_ADV_SETS > 1)
/*! advertising data, discoverable mode */
static const uint8_t advExtAdvDataDisc_2[HCI_EXT_ADV_CONN_DATA_LEN] =
{
  /*! flags */
  2,                                      /*! length */
  DM_ADV_TYPE_FLAGS,                      /*! AD type */
  DM_FLAG_LE_GENERAL_DISC |               /*! flags */
  DM_FLAG_LE_BREDR_NOT_SUP,

  /*! tx power */
  2,                                      /*! length */
  DM_ADV_TYPE_TX_POWER,                   /*! AD type */
  0,                                      /*! tx power */

  /*! service UUID list */
  9,                                      /*! length */
  DM_ADV_TYPE_16_UUID,                    /*! AD type */
  UINT16_TO_BYTES(ATT_UUID_HEART_RATE_SERVICE),
  UINT16_TO_BYTES(ATT_UUID_RUNNING_SPEED_SERVICE),
  UINT16_TO_BYTES(ATT_UUID_DEVICE_INFO_SERVICE),
  UINT16_TO_BYTES(ATT_UUID_BATTERY_SERVICE),

    /*! device name */
  9,                                      /*! length */
  DM_ADV_TYPE_LOCAL_NAME,                 /*! AD type */
  'A',
  'm',
  'b',
  'q',
  '_',
  'A',
  'E',
  '2'
};

/*! scan data, discoverable mode */
static const uint8_t advExtScanDataDisc_2[] =
{
  /*! device name */
  13,                                      /*! length */
  DM_ADV_TYPE_LOCAL_NAME,                 /*! AD type */
  'A',
  'm',
  'b',
  'i',
  'q',
  '_',
  'A',
  'd',
  'E',
  'x',
  't',
  '2'
};
#endif

#if (PERIOD_ADV_TEST_EN == 1)

/*! advertising data, discoverable mode */
static const uint8_t advExtAdvDataDisc_per[] =
{
  /*! flags */
  2,                                      /*! length */
  DM_ADV_TYPE_FLAGS,                      /*! AD type */
  DM_FLAG_LE_GENERAL_DISC |               /*! flags */
  DM_FLAG_LE_BREDR_NOT_SUP,

  /*! device name */
  9,                                      /*! length */
  DM_ADV_TYPE_LOCAL_NAME,                 /*! AD type */
  'A',
  'm',
  'b',
  'q',
  '_',
  'P',
  'e',
  'r'
};
#endif

/*! configurable parameters for slave */
static const appSlaveCfg_t advExtSlaveCfg =
{
  ADV_EXT_CONN_MAX,                           /*! Maximum connections */
};

/*! configurable parameters for security */
static const appSecCfg_t advExtSecCfg =
{
  DM_AUTH_SC_FLAG,    /*! Authentication and bonding flags */
  0,                                      /*! Initiator key distribution flags */
  DM_KEY_DIST_LTK,                        /*! Responder key distribution flags */
  FALSE,                                  /*! TRUE if Out-of-band pairing data is present */
  FALSE                                    /*! TRUE to initiate security upon connection */
};

/*! configurable parameters for connection parameter update */
static const appUpdateCfg_t advExtUpdateCfg =
{
  3000,                                      /*! Connection idle period in ms before attempting
                                              connection parameter update; set to zero to disable */
  48,                                    /*! Minimum connection interval in 1.25ms units */
  60,                                    /*! Maximum connection interval in 1.25ms units */
  4,                                      /*! Connection latency */
  600,                                    /*! Supervision timeout in 10ms units */
  5                                       /*! Number of update attempts before giving up */
};

/*! heart rate measurement configuration */
static const hrpsCfg_t advExtHrpsCfg =
{
  100      /*! Measurement timer expiration period in ms */
};

/*! battery measurement configuration */
static const basCfg_t advExtBasCfg =
{
  30,       /*! Battery measurement timer expiration period in seconds */
  1,        /*! Perform battery measurement after this many timer periods */
  100       /*! Send battery level notification to peer when below this level. */
};

/*! SMP security parameter configuration */
static const smpCfg_t advExtSmpCfg =
{
  3000,                                   /*! 'Repeated attempts' timeout in msec */
  SMP_IO_NO_IN_NO_OUT,                    /*! I/O Capability */
  7,                                      /*! Minimum encryption key length */
  16,                                     /*! Maximum encryption key length */
  3,                                      /*! Attempts to trigger 'repeated attempts' timeout */
  0,                                      /*! Device authentication requirements */
  64000,                                  /*! Maximum repeated attempts timeout in msec */
  64000,                                  /*! Time msec before attemptExp decreases */
  2                                       /*! Repeated attempts multiplier exponent */
};

/**************************************************************************************************
  Advertising Data
**************************************************************************************************/

// AUX_SYNC_IND packet header len is 13, so maximum adv data len(254-13)=241
#define PER_DATA_CONTENT_LEN     (241)
// AUX_CHAIN_IND packet header len is 4, so maximum adv data len(254-4) = 250
#define PER_CHAIN_DATA_LEN       (250)
// last AUX_CHAIN_IND packet header len is 0, so maximum adv data len 254
#define PER_LAST_CHAIN_DATA_LEN  (254)

char manuInfo[PER_DATA_CONTENT_LEN-2] = "Ambiq comany";
char perAdvName[PER_CHAIN_DATA_LEN-2] = "BLE_5.1_device";
uint8_t uuid_128[PER_CHAIN_DATA_LEN-2] = {0x2E, 0xC7, 0x8A, 0x0E, 0x73, 0x90, \
                                            0xE1, 0x11, 0xC2, 0x08, 0x60, 0x27, 0x00, 0x00};
uint8_t advExtDataFlags[PER_CHAIN_DATA_LEN-2] =
{
  DM_FLAG_LE_LIMITED_DISC | DM_FLAG_LE_BREDR_NOT_SUP
};
uint8_t bd_addr[PER_LAST_CHAIN_DATA_LEN-2] = {0x12, 0x34, 0xFF, 0x87, 0x65, 0x99};

#if (CONLESS_AOA_AOD_TEST_EN == 1)
char aoaAodTest[] = "AoA/AoD test";
#endif

// connectable extended advertising header 9
#define EXT_DATA_MAX_LEN  (HCI_EXT_ADV_CONN_DATA_LEN-2)
char extAdvManuInfo[EXT_DATA_MAX_LEN] = "1234";
char extAdvName[EXT_DATA_MAX_LEN] = "Ambiq Ext Adv Test";

/*! advertising data, discoverable mode */
static const uint8_t advExtAdvDataDisc[HCI_EXT_ADV_CONN_DATA_LEN] =
{
  /*! flags */
  2,                                      /*! length */
  DM_ADV_TYPE_FLAGS,                      /*! AD type */
  DM_FLAG_LE_GENERAL_DISC |               /*! flags */
  DM_FLAG_LE_BREDR_NOT_SUP,

  /*! tx power */
  2,                                      /*! length */
  DM_ADV_TYPE_TX_POWER,                   /*! AD type */
  0,                                      /*! tx power */

  /*! service UUID list */
  9,                                      /*! length */
  DM_ADV_TYPE_16_UUID,                    /*! AD type */
  UINT16_TO_BYTES(ATT_UUID_HEART_RATE_SERVICE),
  UINT16_TO_BYTES(ATT_UUID_RUNNING_SPEED_SERVICE),
  UINT16_TO_BYTES(ATT_UUID_DEVICE_INFO_SERVICE),
  UINT16_TO_BYTES(ATT_UUID_BATTERY_SERVICE),

    /*! device name */
  8,                                      /*! length */
  DM_ADV_TYPE_LOCAL_NAME,                 /*! AD type */
  'A',
  'm',
  'b',
  'q',
  '_',
  'A',
  'E'
};

/*! scan data, discoverable mode */
static const uint8_t advExtScanDataDisc[] =
{
  /*! device name */
  12,                                      /*! length */
  DM_ADV_TYPE_LOCAL_NAME,                 /*! AD type */
  'A',
  'm',
  'b',
  'i',
  'q',
  '_',
  'A',
  'd',
  'E',
  'x',
  't'
};

/**************************************************************************************************
  Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! enumeration of client characteristic configuration descriptors */
enum
{
  ADV_EXT_GATT_SC_CCC_IDX,                    /*! GATT service, service changed characteristic */
  ADV_EXT_HRS_HRM_CCC_IDX,                    /*! Heart rate service, heart rate monitor characteristic */
  ADV_EXT_BATT_LVL_CCC_IDX,                   /*! Battery service, battery level characteristic */
  ADV_EXT_RSCS_SM_CCC_IDX,                   /*! Runninc speed and cadence measurement characteristic */
  ADV_EXT_NUM_CCC_IDX
};

/*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t advExtCccSet[ADV_EXT_NUM_CCC_IDX] =
{
  /* cccd handle          value range               security level */
  {GATT_SC_CH_CCC_HDL,    ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_NONE},   /* ADV_EXT_GATT_SC_CCC_IDX */
  {HRS_HRM_CH_CCC_HDL,    ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},   /* ADV_EXT_HRS_HRM_CCC_IDX */
  {BATT_LVL_CH_CCC_HDL,   ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},   /* ADV_EXT_BATT_LVL_CCC_IDX */
  {RSCS_RSM_CH_CCC_HDL,   ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE}    /* ADV_EXT_RSCS_SM_CCC_IDX */
};

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! WSF handler ID */
wsfHandlerId_t advExtHandlerId;

/* WSF Timer to send running speed and cadence measurement data */
wsfTimer_t     advExtRscmTimer;

/* Running Speed and Cadence Measurement period - Can be changed at runtime to vary period */
static uint16_t advExtRscmPeriod = ADV_EXT_DEFAULT_RSCM_PERIOD;

/* Heart Rate Monitor feature flags */
static uint8_t advExtHrmFlags = CH_HRM_FLAGS_VALUE_8BIT | CH_HRM_FLAGS_ENERGY_EXP;

typedef enum
{
    MODE_TYPE_INVALID,
    MODE_TYPE_PRIM_PHY,
    MODE_TYPE_SEC_PHY,
    MODE_TYPE_ADV_TYPE,
    MODE_TYPE_MAX
}advModeSel_t;

uint8_t advPrimPhyChanl  = HCI_ADV_PHY_LE_1M;
uint8_t advSecdAdvChanl  = HCI_ADV_PHY_LE_1M;
advModeSel_t modeSet     = MODE_TYPE_INVALID;
uint8_t advEvtType       = DM_ADV_NONE;

char *modeStr[] =
{
    NULL,
    "Set primary PHY",
    "Set Secondary PHY",
    "Set Adv Event Type"
};

char *advChannelStr[] =
{
    "LE 1M",
    "LE 2M",
    "LE Coded"
};

char *advTypeStr[] =
{
    "ADV_CONN_UNDIRECT",
    "ADV_CONN_DIRECT",
    "ADV_SCAN_UNDIRECT",
    "ADV_NONCONN_UNDIRECT",

    "ADV_CONN_DIRECT_LO_DUTY",
    "EXT_ADV_CONN_UNDIRECT",
    "EXT_ADV_NONCONN_DIRECT",
    "EXT_ADV_SCAN_DIRECT",
};

typedef struct
{
    hciExtAdvParam_t advParam;
    uint8_t advType;
    bool_t useLegAdvPdu;
}extAdvParam_t;

uint8_t peerAddr[BDA_ADDR_LEN] = {0x12, 0x34, 0x56, 0x78, 0xAB, 0xFF};
uint8_t perDataBuf[ADV_BUF_LEN_MAX] = {0};

extAdvParam_t  extAdvParam[DM_NUM_ADV_SETS] =
{
  {
    {
      .priAdvInterMin  = 1000 / 0.625,      // 1s minimum interval
      .priAdvInterMax  = 1000 / 0.625,
      .priAdvChanMap   = DM_ADV_CHAN_ALL,
      .ownAddrType     = DM_ADDR_PUBLIC,
      .peerAddrType    = DM_ADDR_PUBLIC,
      .pPeerAddr       = peerAddr,
      .advFiltPolicy   = HCI_ADV_FILT_NONE,
      .advTxPwr        = HCI_TX_PWR_NO_PREFERENCE,
      .priAdvPhy       = HCI_ADV_PHY_LE_1M,
      .secAdvMaxSkip   = 0,
      .secAdvPhy       = HCI_ADV_PHY_LE_1M,
      .advSID          = 0,
      .scanReqNotifEna = FALSE,
    },
    .advType           = DM_ADV_CONN_UNDIRECT,
#if (PERIOD_ADV_TEST_EN == 1)
    .useLegAdvPdu      = FALSE,
#else
    .useLegAdvPdu      = TRUE,
#endif
  },

  {
      {
        .priAdvInterMin  = 1000 / 0.625,    // 1s minimum interval
        .priAdvInterMax  = 1000 / 0.625,
        .priAdvChanMap   = DM_ADV_CHAN_ALL,
        .ownAddrType     = DM_ADDR_PUBLIC,
        .peerAddrType    = DM_ADDR_PUBLIC,
        .pPeerAddr       = peerAddr,
        .advFiltPolicy   = HCI_ADV_FILT_NONE,
        .advTxPwr        = HCI_TX_PWR_NO_PREFERENCE,
        .priAdvPhy       = HCI_ADV_PHY_LE_1M,
        .secAdvMaxSkip   = 0,
        .secAdvPhy       = HCI_ADV_PHY_LE_2M,
        .advSID          = 1,
        .scanReqNotifEna = FALSE,
      },
      .advType           = DM_EXT_ADV_CONN_UNDIRECT,    //DM_ADV_CONN_UNDIRECT,
      .useLegAdvPdu      = FALSE,
  }
};


/*************************************************************************************************/
/*!
 *  \brief  Application DM callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void advExtDmCback(dmEvt_t *pDmEvt)
{
  dmEvt_t *pMsg;
  uint16_t len;

  len = DmSizeOfEvt(pDmEvt);

  if ((pMsg = WsfMsgAlloc(len)) != NULL)
  {
    memcpy(pMsg, pDmEvt, len);
    WsfMsgSend(advExtHandlerId, pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Application ATT callback.
 *
 *  \param  pEvt    ATT callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void advExtAttCback(attEvt_t *pEvt)
{
  attEvt_t *pMsg;

  if ((pMsg = WsfMsgAlloc(sizeof(attEvt_t) + pEvt->valueLen)) != NULL)
  {
    memcpy(pMsg, pEvt, sizeof(attEvt_t));
    pMsg->pValue = (uint8_t *) (pMsg + 1);
    memcpy(pMsg->pValue, pEvt->pValue, pEvt->valueLen);
    WsfMsgSend(advExtHandlerId, pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Application ATTS client characteristic configuration callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void advExtCccCback(attsCccEvt_t *pEvt)
{
  attsCccEvt_t  *pMsg;
  appDbHdl_t    dbHdl;

  /* If CCC not set from initialization and there's a device record and currently bonded */
  if ((pEvt->handle != ATT_HANDLE_NONE) &&
      ((dbHdl = AppDbGetHdl((dmConnId_t) pEvt->hdr.param)) != APP_DB_HDL_NONE) &&
      AppCheckBonded((dmConnId_t)pEvt->hdr.param))
  {
    /* Store value in device database. */
    AppDbSetCccTblValue(dbHdl, pEvt->idx, pEvt->value);
  }

  if ((pMsg = WsfMsgAlloc(sizeof(attsCccEvt_t))) != NULL)
  {
    memcpy(pMsg, pEvt, sizeof(attsCccEvt_t));
    WsfMsgSend(advExtHandlerId, pMsg);
  }
}


/*************************************************************************************************/
/*!
*  \brief  Send a Running Speed and Cadence Measurement Notification.
*
*  \param  connId    connection ID
*
*  \return None.
*/
/*************************************************************************************************/
static void advExtSendRunningSpeedMeasurement(dmConnId_t connId)
{
  if (AttsCccEnabled(connId, ADV_EXT_RSCS_SM_CCC_IDX))
  {
    static uint8_t walk_run = 1;

    RscpsSetParameter(RSCP_SM_PARAM_SPEED, 1);
    RscpsSetParameter(RSCP_SM_PARAM_CADENCE, 2);
    RscpsSetParameter(RSCP_SM_PARAM_STRIDE_LENGTH, 3);
    RscpsSetParameter(RSCP_SM_PARAM_TOTAL_DISTANCE, 4);

    /* Toggle running/walking */
    walk_run = walk_run? 0 : 1;
    RscpsSetParameter(RSCP_SM_PARAM_STATUS, walk_run);

    RscpsSendSpeedMeasurement(connId);
  }

  /* Configure and start timer to send the next measurement */
  advExtRscmTimer.msg.event = ADV_EXT_RUNNING_TIMER_IND;
  advExtRscmTimer.msg.status = ADV_EXT_RSCS_SM_CCC_IDX;
  advExtRscmTimer.handlerId = advExtHandlerId;
  advExtRscmTimer.msg.param = connId;

  WsfTimerStartSec(&advExtRscmTimer, advExtRscmPeriod);
}

/*************************************************************************************************/
/*!
 *  \brief  Process CCC state change.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void advExtProcCccState(advExtMsg_t *pMsg)
{
  APP_TRACE_INFO3("ccc state ind value:%d handle:%d idx:%d", pMsg->ccc.value, pMsg->ccc.handle, pMsg->ccc.idx);

  /* handle heart rate measurement CCC */
  if (pMsg->ccc.idx == ADV_EXT_HRS_HRM_CCC_IDX)
  {
    if (pMsg->ccc.value == ATT_CLIENT_CFG_NOTIFY)
    {
      HrpsMeasStart((dmConnId_t) pMsg->ccc.hdr.param, ADV_EXT_HR_TIMER_IND, ADV_EXT_HRS_HRM_CCC_IDX);
    }
    else
    {
      HrpsMeasStop((dmConnId_t) pMsg->ccc.hdr.param);
    }
    return;
  }

  /* handle running speed and cadence measurement CCC */
  if (pMsg->ccc.idx == ADV_EXT_RSCS_SM_CCC_IDX)
  {
    if (pMsg->ccc.value == ATT_CLIENT_CFG_NOTIFY)
    {
      advExtSendRunningSpeedMeasurement((dmConnId_t)pMsg->ccc.hdr.param);
    }
    else
    {
      WsfTimerStop(&advExtRscmTimer);
    }
    return;
  }

  /* handle battery level CCC */
  if (pMsg->ccc.idx == ADV_EXT_BATT_LVL_CCC_IDX)
  {
    if (pMsg->ccc.value == ATT_CLIENT_CFG_NOTIFY)
    {
      BasMeasBattStart((dmConnId_t) pMsg->ccc.hdr.param, ADV_EXT_BATT_TIMER_IND, ADV_EXT_BATT_LVL_CCC_IDX);
    }
    else
    {
      BasMeasBattStop((dmConnId_t) pMsg->ccc.hdr.param);
    }
    return;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Perform UI actions on connection close.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void advExtClose(advExtMsg_t *pMsg)
{
  /* stop heart rate measurement */
  HrpsMeasStop((dmConnId_t) pMsg->hdr.param);

  /* stop battery measurement */
  BasMeasBattStop((dmConnId_t) pMsg->hdr.param);

  /* Stop running speed and cadence timer */
  WsfTimerStop(&advExtRscmTimer);
}


/*************************************************************************************************/
/*!
 *  \brief  Set up advertising parameters used in Extended Advertising
 *
 *  \param  NULL.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppExtSetParam(void)
{
    uint8_t advHandle = 0;

    for ( advHandle = 0; advHandle < DM_NUM_ADV_SETS; advHandle++ )
    {
        // set extended advertising interval
        advExtAdvCfg.advInterval[advHandle] = (uint16_t)(extAdvParam[advHandle].advParam.priAdvInterMin);

        DmAdvSetChannelMap(advHandle, extAdvParam[advHandle].advParam.priAdvChanMap);

        DmAdvSetAddrType(extAdvParam[advHandle].advParam.ownAddrType);

        AppExtSetAdvPeerAddr(advHandle, extAdvParam[advHandle].advParam.peerAddrType,
                            extAdvParam[advHandle].advParam.pPeerAddr);

        DmDevSetExtFilterPolicy(advHandle, DM_FILT_POLICY_MODE_ADV, HCI_ADV_FILT_NONE);

        DmAdvIncTxPwr(advHandle, FALSE, extAdvParam[advHandle].advParam.advTxPwr);

        DmAdvSetPhyParam(advHandle, extAdvParam[advHandle].advParam.priAdvPhy,
                            extAdvParam[advHandle].advParam.secAdvMaxSkip, extAdvParam[advHandle].advParam.secAdvPhy);

        DmAdvScanReqNotifEnable(advHandle, extAdvParam[advHandle].advParam.scanReqNotifEna);

        AppExtSetAdvType(advHandle, extAdvParam[advHandle].advType);

        DmAdvUseLegacyPdu(advHandle, extAdvParam[advHandle].useLegAdvPdu);
        advExtAdvCfg.useLegacyPdu[advHandle] = extAdvParam[advHandle].useLegAdvPdu;
    }
}

uint8_t antena_id[HCI_MIN_NUM_ANTENNA_IDS] = {0, 1};

/*************************************************************************************************/
/*!
 *  \brief  Set up advertising and other procedures that need to be performed after
 *          device reset.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void advExtSetup(advExtMsg_t *pMsg)
{
#if (PERIOD_ADV_TEST_EN == 1)

    APP_TRACE_INFO0("going to send period adv packet");

    AppExtAdvSetData(adv_handle[0], APP_ADV_DATA_DISCOVERABLE, sizeof(advExtAdvDataDisc_per), (uint8_t *) advExtAdvDataDisc_per, ADV_BUF_LEN_MAX);
    AppExtSetParam();
    AppExtSetAdvType(adv_handle[0], DM_ADV_NONCONN_UNDIRECT);
    AppExtAdvStart(1, &adv_handle[0], APP_MODE_AUTO_INIT);

    memset(perDataBuf, sizeof(perDataBuf), 0x0);
    AppPerAdvSetData(adv_handle[0], 0, (uint8_t *) perDataBuf, ADV_BUF_LEN_MAX);
#if (CONLESS_AOA_AOD_TEST_EN == 1)
    AppPerAdvSetAdValue(adv_handle[0], DM_ADV_TYPE_LOCAL_NAME, sizeof(aoaAodTest), (uint8_t *) aoaAodTest);
#else
    AppPerAdvSetAdValue(adv_handle[0], DM_ADV_TYPE_MANUFACTURER, sizeof(manuInfo), (uint8_t *) manuInfo);
    AppPerAdvSetAdValue(adv_handle[0], DM_ADV_TYPE_LOCAL_NAME, sizeof(perAdvName), (uint8_t *) perAdvName);
    AppPerAdvSetAdValue(adv_handle[0], DM_ADV_TYPE_FLAGS, sizeof(advExtDataFlags), (uint8_t *) advExtDataFlags);
    AppPerAdvSetAdValue(adv_handle[0], DM_ADV_TYPE_128_UUID, sizeof(uuid_128), (uint8_t *) uuid_128);
    AppPerAdvSetAdValue(adv_handle[0], DM_ADV_TYPE_BD_ADDR, sizeof(bd_addr), (uint8_t *) bd_addr);
#endif
    AppPerAdvStart(adv_handle[0]);
#else
    APP_TRACE_INFO0("going to send Extended adv");
    AppExtSetParam();

    AppExtAdvSetData(adv_handle[0], APP_ADV_DATA_DISCOVERABLE, sizeof(advExtAdvDataDisc), (uint8_t *) advExtAdvDataDisc, ADV_BUF_LEN_MAX);
    AppExtAdvSetData(adv_handle[0], APP_SCAN_DATA_DISCOVERABLE, sizeof(advExtScanDataDisc), (uint8_t *) advExtScanDataDisc, ADV_BUF_LEN_MAX);

#if DM_NUM_ADV_SETS > 1
    adv_handle[1] = 1;
    AppExtAdvSetData(adv_handle[1], APP_ADV_DATA_DISCOVERABLE, sizeof(advExtAdvDataDisc_2), (uint8_t *) advExtAdvDataDisc_2, ADV_BUF_LEN_MAX);
    AppExtAdvSetData(adv_handle[1], APP_SCAN_DATA_DISCOVERABLE, sizeof(advExtScanDataDisc_2), (uint8_t *) advExtScanDataDisc_2, ADV_BUF_LEN_MAX);
#endif
    AppExtAdvStart(DM_NUM_ADV_SETS, adv_handle, APP_MODE_AUTO_INIT);

#endif
}

/*************************************************************************************************/
/*!
 *  \brief  Button press callback.
 *
 *  \param  btn    Button press.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void advExtBtnCback(uint8_t btn)
{
    if (AppConnIsOpen() == DM_CONN_ID_NONE)
    {
        if ( AppSlaveIsAdvertising() )
        {
            APP_TRACE_INFO0("Stop advertising");
            AppExtAdvStop(DM_NUM_ADV_SETS, adv_handle);
            return;
        }

        switch (btn)
        {
            /*
                mode selection:
                primary advertising phy
                secondary advertising phy
                advertising event type
            */
            case APP_UI_BTN_1_SHORT:
                if ( ++modeSet >= MODE_TYPE_MAX )
                {
                    modeSet = MODE_TYPE_PRIM_PHY;
                }

                APP_TRACE_INFO1("Mode --> %s", modeStr[modeSet]);

            break;

            // change corresponding value of mode selection
            case APP_UI_BTN_1_MED:
                switch(modeSet)
                {
                    case MODE_TYPE_PRIM_PHY:
                        advPrimPhyChanl = (advPrimPhyChanl == HCI_ADV_PHY_LE_1M) ? (HCI_ADV_PHY_LE_CODED) : (HCI_ADV_PHY_LE_1M);
                        APP_TRACE_INFO1("Set Primary PHY to %s", (advPrimPhyChanl == HCI_ADV_PHY_LE_1M) ? "1M PHY" : "LE Coded PHY");

                    break;

                    case MODE_TYPE_SEC_PHY:
                        APP_TRACE_INFO1("Set secondary PHY to: %s", advChannelStr[advSecdAdvChanl-1]);

                        if ( ++advSecdAdvChanl > HCI_ADV_PHY_LE_CODED )
                        {
                            advSecdAdvChanl = HCI_ADV_PHY_LE_1M;
                        }

                    break;

                    case MODE_TYPE_ADV_TYPE:
                        if ( ++advEvtType > DM_EXT_ADV_SCAN_DIRECT )
                        {
                            advEvtType = DM_ADV_CONN_UNDIRECT;
                        }

                        APP_TRACE_INFO1("Set adv evt type to:%s", advTypeStr[advEvtType]);
                    break;
                    default:
                        APP_TRACE_INFO1("Wrong Modset:%d", modeSet);

                }

            break;

            // start advertising
            case APP_UI_BTN_1_LONG:
            {
                bool_t use_leg_adv = TRUE;
                uint8_t advType = (advEvtType == DM_ADV_NONE) ? 0 : (advEvtType);

                AppExtAdvSetData(adv_handle[0], APP_ADV_DATA_DISCOVERABLE, sizeof(advExtAdvDataDisc), (uint8_t *) advExtAdvDataDisc, ADV_BUF_LEN_MAX);
                AppExtAdvSetData(adv_handle[0], APP_SCAN_DATA_DISCOVERABLE, sizeof(advExtScanDataDisc), (uint8_t *) advExtScanDataDisc, ADV_BUF_LEN_MAX);
                DmAdvSetPhyParam(adv_handle[0], advPrimPhyChanl, 0, advSecdAdvChanl);
                AppExtSetAdvType(adv_handle[0], advType);

                //if ( DM_ADV_DIRECTED(advEvtType) )
                {
                    uint8_t bda_addr[BDA_ADDR_LEN] = {0x94, 0x87, 0xE0, 0xA5, 0x18, 0xEB};

                    AppExtSetAdvPeerAddr(adv_handle[0], HCI_ADDR_TYPE_PUBLIC, bda_addr);
                }

                use_leg_adv = (advEvtType>DM_ADV_CONN_DIRECT_LO_DUTY)?FALSE:TRUE;
                APP_TRACE_INFO1("use legacy adv=%d", use_leg_adv);
                DmAdvUseLegacyPdu(adv_handle[0], use_leg_adv);

                //APP_TRACE_INFO3("start adv, prim PHY=%s, sec PHY=%s, adv event Type=%s",
                 // advChannelStr[advPrimPhyChanl], advChannelStr[advSecdAdvChanl], advTypeStr[advEvtType]);

                AppExtAdvStart(1, &adv_handle[0], APP_MODE_AUTO_INIT);
            }
            break;
            default:
            break;
        }
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Process messages from the event handler.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void advExtProcMsg(advExtMsg_t *pMsg)
{
  uint8_t uiEvent = APP_UI_NONE;
  uint8_t i = 0;

  switch(pMsg->hdr.event)
  {
    case ADV_EXT_RUNNING_TIMER_IND:
      advExtSendRunningSpeedMeasurement((dmConnId_t)pMsg->ccc.hdr.param);
      break;

    case ADV_EXT_HR_TIMER_IND:
      HrpsProcMsg(&pMsg->hdr);
      break;

    case ADV_EXT_BATT_TIMER_IND:
      BasProcMsg(&pMsg->hdr);
      break;

    case ATTS_HANDLE_VALUE_CNF:
      HrpsProcMsg(&pMsg->hdr);
      BasProcMsg(&pMsg->hdr);
      break;

    case ATTS_CCC_STATE_IND:
      advExtProcCccState(pMsg);
      break;

    case ATT_MTU_UPDATE_IND:
      APP_TRACE_INFO1("Negotiated MTU %d", ((attEvt_t *)pMsg)->mtu);
      break;

    case DM_RESET_CMPL_IND:
      // set database hash calculating status to true until a new hash is generated after reset
      attsCsfSetHashUpdateStatus(TRUE);

      // Generate ECC key if configured support secure connection,
      // else will calcualte ATT database hash
      if( advExtSecCfg.auth & DM_AUTH_SC_FLAG )
      {
          DmSecGenerateEccKeyReq();
      }
      else
      {
          AttsCalculateDbHash();
      }

      uiEvent = APP_UI_RESET_CMPL;
      break;

    case ATTS_DB_HASH_CALC_CMPL_IND:
      advExtSetup(pMsg);
      break;

    case DM_ADV_SET_START_IND:
      uiEvent = APP_UI_ADV_SET_START_IND;
#if (CONLESS_AOA_AOD_TEST_EN == 1)
      APP_TRACE_INFO0("going to enable connectionless CTE");
      HciLeSetConnectionlessCteTxParamsCmd(adv_handle[0], CONLESS_CTE_LEN, CONLESS_CTE_CNT, HCI_CTE_TYPE_REQ_AOA, HCI_MIN_NUM_ANTENNA_IDS, antena_id);
      HciLeConnectionlessCteTxEnableCmd(adv_handle[0], TRUE);
#endif
      break;

    case DM_ADV_SET_STOP_IND:
    {
        for ( i = 0; i< DM_CONN_MAX; i++ )
        {
          if ( !appExtConnCb[i].used )
          {
              appExtConnCb[i].used = TRUE;
              appExtConnCb[i].advHandle = pMsg->dm.advSetStop.advHandle;
              appExtConnCb[i].connHandle = pMsg->dm.advSetStop.handle;

              break;
          }
        }

        uiEvent = APP_UI_ADV_SET_STOP_IND;
    }
    break;

    case DM_ADV_START_IND:
        uiEvent = APP_UI_ADV_START;
    break;

    case DM_ADV_STOP_IND:
      uiEvent = APP_UI_ADV_STOP;
      break;

    case DM_CONN_OPEN_IND:
      HrpsProcMsg(&pMsg->hdr);
      BasProcMsg(&pMsg->hdr);
      // AppSlaveSecurityReq(1);
      uiEvent = APP_UI_CONN_OPEN;
      break;

    case DM_CONN_CLOSE_IND:
    {
//      APP_TRACE_INFO1("conn close reason = 0x%x\n", pMsg->connClose.reason);
        uint8_t connHdl = pMsg->dm.connClose.handle;

        for ( i = 0; i< DM_CONN_MAX; i++ )
        {
            if ( appExtConnCb[i].used && (connHdl == appExtConnCb[i].connHandle) )
            {
                appExtConnCb[i].used = FALSE;
                AppExtAdvStart(1, &appExtConnCb[i].advHandle, APP_MODE_AUTO_INIT);

                break;
            }
        }

        advExtClose(pMsg);
        uiEvent = APP_UI_CONN_CLOSE;
    }
    break;

    case DM_PHY_UPDATE_IND:
      APP_TRACE_INFO3("DM_PHY_UPDATE_IND status: %d, RX: %d, TX: %d", pMsg->dm.phyUpdate.status, pMsg->dm.phyUpdate.rxPhy, pMsg->dm.phyUpdate.txPhy);
      break;

    case DM_SEC_PAIR_CMPL_IND:
      DmSecGenerateEccKeyReq();
      uiEvent = APP_UI_SEC_PAIR_CMPL;
      break;

    case DM_SEC_PAIR_FAIL_IND:
      DmSecGenerateEccKeyReq();
      uiEvent = APP_UI_SEC_PAIR_FAIL;
      break;

    case DM_SEC_ENCRYPT_IND:
      uiEvent = APP_UI_SEC_ENCRYPT;
      break;

    case DM_SEC_ENCRYPT_FAIL_IND:
      uiEvent = APP_UI_SEC_ENCRYPT_FAIL;
      break;

    case DM_SEC_AUTH_REQ_IND:
      AppHandlePasskey(&pMsg->dm.authReq);
      break;

    case DM_SEC_ECC_KEY_IND:
      DmSecSetEccKey(&pMsg->dm.eccMsg.data.key);
      // Only calculate database hash if the calculating status is in progress
      if( attsCsfGetHashUpdateStatus() )
      {
        AttsCalculateDbHash();
      }
      break;

    case DM_SEC_COMPARE_IND:
      AppHandleNumericComparison(&pMsg->dm.cnfInd);
      break;

    case DM_PRIV_CLEAR_RES_LIST_IND:
      APP_TRACE_INFO1("Clear resolving list status 0x%02x", pMsg->hdr.status);
      break;

    case DM_HW_ERROR_IND:
      uiEvent = APP_UI_HW_ERROR;
      break;

    case DM_VENDOR_SPEC_CMD_CMPL_IND:
      break;

    default:
      break;
  }

  if (uiEvent != APP_UI_NONE)
  {
    AppUiAction(uiEvent);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AdvExtHandlerInit(wsfHandlerId_t handlerId)
{
  APP_TRACE_INFO0("advExtHandlerInit");

  /* store handler ID */
  advExtHandlerId = handlerId;

  /* Set configuration pointers */
  pAppExtAdvCfg = (appExtAdvCfg_t*)&advExtAdvCfg;
  pAppSlaveCfg = (appSlaveCfg_t *) &advExtSlaveCfg;
  pAppSecCfg = (appSecCfg_t *) &advExtSecCfg;
  pAppUpdateCfg = (appUpdateCfg_t *) &advExtUpdateCfg;

  /* Initialize application framework */
  AppSlaveInit();
  AppServerInit();

  /* Set stack configuration pointers */
  pSmpCfg = (smpCfg_t *) &advExtSmpCfg;

  /* initialize heart rate profile sensor */
  HrpsInit(handlerId, (hrpsCfg_t *) &advExtHrpsCfg);
  HrpsSetFlags(advExtHrmFlags);

  /* initialize battery service server */
  BasInit(handlerId, (basCfg_t *) &advExtBasCfg);
}

/*************************************************************************************************/
/*!
 *  \brief  WSF event handler for application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AdvExtHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
  if (pMsg != NULL)
  {
    APP_TRACE_INFO1("advExt got evt %d", pMsg->event);

    /* process ATT messages */
    if (pMsg->event >= ATT_CBACK_START && pMsg->event <= ATT_CBACK_END)
    {
      /* process server-related ATT messages */
      AppServerProcAttMsg(pMsg);
    }
    /* process DM messages */
    else if (pMsg->event >= DM_CBACK_START && pMsg->event <= DM_CBACK_END)
    {
      /* process advertising and connection-related messages */
      AppSlaveProcDmMsg((dmEvt_t *) pMsg);

      /* process security-related messages */
      AppSlaveSecProcDmMsg((dmEvt_t *) pMsg);
    }

    /* perform profile and user interface-related operations */
    advExtProcMsg((advExtMsg_t *) pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AdvExtStart(void)
{
  /* Register for stack callbacks */
  DmRegister(advExtDmCback);
  DmConnRegister(DM_CLIENT_ID_APP, advExtDmCback);
  AttRegister(advExtAttCback);
  AttConnRegister(AppServerConnCback);
  AttsCccRegister(ADV_EXT_NUM_CCC_IDX, (attsCccSet_t *) advExtCccSet, advExtCccCback);

  /* Register for app framework callbacks */
  AppUiBtnRegister(advExtBtnCback);

  /* Initialize attribute server database */
  SvcCoreGattCbackRegister(GattReadCback, GattWriteCback);
  SvcCoreAddGroup();
  SvcHrsCbackRegister(NULL, HrpsWriteCback);
  SvcHrsAddGroup();
  SvcDisAddGroup();
  SvcBattCbackRegister(BasReadCback, NULL);
  SvcBattAddGroup();
  SvcRscsAddGroup();

  /* Set Service Changed CCCD index. */
  GattSetSvcChangedIdx(ADV_EXT_GATT_SC_CCC_IDX);

  /* Set running speed and cadence features */
  RscpsSetFeatures(RSCS_ALL_FEATURES);

  /* Reset the device */
  DmDevReset();
}
