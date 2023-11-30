/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Cycling server sample application.
 *
 *  Copyright (c) 2016-2019 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2019 Packetcraft, Inc.
 *  
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *  
 *      http://www.apache.org/licenses/LICENSE-2.0
 *  
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
/*************************************************************************************************/

#include <string.h>
#include "wsf_types.h"
#include "util/bstream.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "wsf_buf.h"
#include "hci_api.h"
#include "dm_api.h"
#include "att_api.h"
#include "app_api.h"
#include "app_main.h"
#include "app_db.h"
#include "app_ui.h"
#include "svc_ch.h"
#include "svc_core.h"
#include "svc_cps.h"
#include "svc_cscs.h"
#include "svc_batt.h"
#include "bas/bas_api.h"
#include "cpp/cpp_api.h"
#include "cscp/cscp_api.h"
#include "gatt/gatt_api.h"
#include "util/calc128.h"


/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Enumeration of client characteristic configuration descriptors */
enum
{
  CYCLING_GATT_SC_CCC_IDX,           /*! GATT service, service changed characteristic */
  CYCLING_CPS_CPM_CCC_IDX,           /*! Cycling Power Measurement, service changed characteristic */
  CYCLING_CSCS_CSM_CCC_IDX,          /*! Cycling Speed Measurement, service changed characteristic */
  CYCLING_BATT_LVL_CCC_IDX,          /*! Battery service, battery level characteristic */
  CYCLING_NUM_CCC_IDX
};

/*! WSF message event starting value */
#define CYCLING_MSG_START               0xA0

/*! WSF message event enumeration */
enum
{
  CYCLING_CPP_PM_TIMER_IND = CYCLING_MSG_START,   /*! Cycling power measurement timer expired */
  CYCLING_CSCP_SM_TIMER_IND,                      /*! Cycling speed measurement timer expired */
  CYCLING_BATT_TIMER_IND                          /*! Battery measurement timer expired */
};

/* Default Cycling Power Measurement period (seconds) */
#define CYCLING_DEFAULT_CPM_PERIOD        1

/* Default Cycling Speed Measurement period (seconds) */
#define CYCLING_DEFAULT_CSM_PERIOD        1

/* Default MTU */
#define CYCLING_DEFAULT_MTU               50

/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/

/*! configurable parameters for advertising */
static const appAdvCfg_t cyclingAdvCfg =
{
  {30000,     0,     0},                  /*! Advertising durations in ms */
  {   96,  1600,     0}                   /*! Advertising intervals in 0.625 ms units */
};

/*! configurable parameters for slave */
static const appSlaveCfg_t cyclingSlaveCfg =
{
  1,                                      /*! Maximum connections */
};

/*! configurable parameters for security */
static const appSecCfg_t cyclingSecCfg =
{
  DM_AUTH_BOND_FLAG | SMP_AUTH_SC_FLAG,   /*! Authentication and bonding flags */
  DM_KEY_DIST_IRK,                        /*! Initiator key distribution flags */
  DM_KEY_DIST_LTK | DM_KEY_DIST_IRK,      /*! Responder key distribution flags */
  FALSE,                                  /*! TRUE if Out-of-band pairing data is present */
  TRUE                                    /*! TRUE to initiate security upon connection */
};

/*! configurable parameters for connection parameter update */
static const appUpdateCfg_t cyclingUpdateCfg =
{
  0,                                      /*! Connection idle period in ms before attempting
                                              connection parameter update; set to zero to disable */
  640,                                    /*! Minimum connection interval in 1.25ms units */
  800,                                    /*! Maximum connection interval in 1.25ms units */
  3,                                      /*! Connection latency */
  600,                                    /*! Supervision timeout in 10ms units */
  5                                       /*! Number of update attempts before giving up */
};

/*! ATT configurable parameters (increase MTU) */
static const attCfg_t cyclingAttCfg =
{
  15,                               /* ATT server service discovery connection idle timeout in seconds */
  CYCLING_DEFAULT_MTU,              /* desired ATT MTU */
  ATT_MAX_TRANS_TIMEOUT,            /* transcation timeout in seconds */
  4                                 /* number of queued prepare writes supported by server */
};

/*! local IRK */
static uint8_t localIrk[] =
{
  0x95, 0xC8, 0xEE, 0x6F, 0xC5, 0x0D, 0xEF, 0x93, 0x35, 0x4E, 0x7C, 0x57, 0x08, 0xE2, 0xA3, 0x85
};

/**************************************************************************************************
  Advertising Data
**************************************************************************************************/

/*! advertising data, discoverable mode */
static const uint8_t cyclingAdvDataDisc[] =
{
  /*! flags */
  2,                                      /*! length */
  DM_ADV_TYPE_FLAGS,                      /*! AD type */
  DM_FLAG_LE_GENERAL_DISC |               /*! flags */
  DM_FLAG_LE_BREDR_NOT_SUP,

  /*! manufacturer specific data */
  3,                                      /*! length */
  DM_ADV_TYPE_MANUFACTURER,               /*! AD type */
  UINT16_TO_BYTES(HCI_ID_PACKETCRAFT),    /*! company ID */

  /*! service UUID list */
  9,                                      /*! length */
  DM_ADV_TYPE_16_UUID,                    /*! AD type */
  UINT16_TO_BYTES(ATT_UUID_CYCLING_POWER_SERVICE),
  UINT16_TO_BYTES(ATT_UUID_CYCLING_SPEED_SERVICE),
  UINT16_TO_BYTES(ATT_UUID_DEVICE_INFO_SERVICE),
  UINT16_TO_BYTES(ATT_UUID_BATTERY_SERVICE)
};

/*! scan data, discoverable mode */
static const uint8_t cyclingScanDataDisc[] =
{
  /*! device name */
  8,                                      /*! length */
  DM_ADV_TYPE_LOCAL_NAME,                 /*! AD type */
  'C',
  'y',
  'c',
  'l',
  'i',
  'n',
  'g'
};

/**************************************************************************************************
  Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t cyclingCccSet[CYCLING_NUM_CCC_IDX] =
{
  /* cccd handle          value range               security level */
  { GATT_SC_CH_CCC_HDL,    ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_NONE },   /* CYCLING_GATT_SC_CCC_IDX */
  { CPS_CPM_CH_CCC_HDL,    ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE },   /* CYCLING_CPS_CPM_CCC_IDX */
  { CSCS_CSM_CH_CCC_HDL,   ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE },   /* CYCLING_CSCS_CSM_CCC_IDX */
  { BATT_LVL_CH_CCC_HDL,   ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE }    /* CYCLING_BATT_LVL_CCC_IDX */
};

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! application control block */
static struct
{
  wsfHandlerId_t    handlerId;        /* WSF handler ID */
  wsfTimer_t        cpmTimer;         /* WSF Timer to send cycling power measurement data */
  wsfTimer_t        csmTimer;         /* WSF Timer to send cycling speed measurement data */
} cyclingCb;

/* Cycling Power Measurement period - Can be changed at runtime to vary period */
static uint16_t cyclingCpmPeriod = CYCLING_DEFAULT_CPM_PERIOD;

/* Cycling Speed Measurement period - Can be changed at runtime to vary period */
static uint16_t cyclingCsmPeriod = CYCLING_DEFAULT_CSM_PERIOD;

/*************************************************************************************************/
/*!
 *  \brief  Application DM callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void cyclingDmCback(dmEvt_t *pDmEvt)
{
  dmEvt_t *pMsg;

  if (pDmEvt->hdr.event == DM_SEC_ECC_KEY_IND)
  {
    DmSecSetEccKey(&pDmEvt->eccMsg.data.key);
  }
  else
  {
    uint16_t len = DmSizeOfEvt(pDmEvt);

    if ((pMsg = WsfMsgAlloc(len)) != NULL)
    {
      memcpy(pMsg, pDmEvt, len);
      WsfMsgSend(cyclingCb.handlerId, pMsg);
    }
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
static void cyclingAttCback(attEvt_t *pEvt)
{
  attEvt_t  *pMsg;

  if ((pMsg = WsfMsgAlloc(sizeof(attEvt_t) + pEvt->valueLen)) != NULL)
  {
    memcpy(pMsg, pEvt, sizeof(attEvt_t));
    pMsg->pValue = (uint8_t *)(pMsg + 1);
    memcpy(pMsg->pValue, pEvt->pValue, pEvt->valueLen);
    WsfMsgSend(cyclingCb.handlerId, pMsg);
  }
}

/*************************************************************************************************/
/*!
*  \brief  Send a Cycling Speed Measurement Notification.
*
*  \param  connId    connection ID
*
*  \return None.
*/
/*************************************************************************************************/
static void cyclingSendCyclingSpeedMeasurement(dmConnId_t connId)
{
  static uint32_t revolutions = 1;
  static uint32_t lastTime = 1;

  if (AttsCccEnabled(connId, CYCLING_CSCS_CSM_CCC_IDX))
  {

    revolutions++;
    lastTime += 100;

    CscpsSetParameter(CSCP_SM_PARAM_WHEEL_REVOLUTIONS, revolutions);
    CscpsSetParameter(CSCP_SM_PARAM_LAST_WHEEL_EVT_TIME, lastTime);
    CscpsSetParameter(CSCP_SM_PARAM_CRANK_REVOLUTIONS, revolutions);
    CscpsSetParameter(CSCP_SM_PARAM_LAST_CRANK_TIME, lastTime);

    CscpsSendSpeedMeasurement(connId);
  }

  /* Configure and start timer to send the next measurement */
  cyclingCb.csmTimer.msg.event = CYCLING_CSCP_SM_TIMER_IND;
  cyclingCb.csmTimer.msg.status = CYCLING_CSCS_CSM_CCC_IDX;
  cyclingCb.csmTimer.handlerId = cyclingCb.handlerId;
  cyclingCb.csmTimer.msg.param = connId;

  WsfTimerStartSec(&cyclingCb.csmTimer, cyclingCsmPeriod);
}

/*************************************************************************************************/
/*!
*  \brief  Send a Cycling Power Measurement Notification.
*
*  \param  connId    connection ID
*
*  \return None.
*/
/*************************************************************************************************/
static void cyclingSendCyclingPowerMeasurement(dmConnId_t connId)
{
  static uint32_t revolutions = 1;
  static uint32_t lastTime = 1;

  if (AttsCccEnabled(connId, CYCLING_CPS_CPM_CCC_IDX))
  {

    revolutions++;
    lastTime += 100;

    CppsSetParameter(CPP_PM_PARAM_INSTANTANEOUS_POWER, 1);
    CppsSetParameter(CPP_PM_PARAM_PEDAL_POWER, 2);
    CppsSetParameter(CPP_PM_PARAM_ACCUMULATED_TORQUE, 4);
    CppsSetParameter(CPP_PM_PARAM_WHEEL_REVOLUTIONS, revolutions);
    CppsSetParameter(CPP_PM_PARAM_LAST_WHEEL_REV_TIME, lastTime);
    CppsSetParameter(CPP_PM_PARAM_CRANK_REVOLUTIONS, revolutions);
    CppsSetParameter(CPP_PM_PARAM_LAST_CRANK_TIME, lastTime);
    CppsSetParameter(CPP_PM_PARAM_MAX_FORCE_MAGNITUDE, 9);
    CppsSetParameter(CPP_PM_PARAM_MIN_FORCE_MAGNITUDE, 10);
    CppsSetParameter(CPP_PM_PARAM_MAX_TORQUE_MAGNITUDE, 11);
    CppsSetParameter(CPP_PM_PARAM_MIN_TORQUE_MAGNITUDE, 12);
    CppsSetParameter(CPP_PM_PARAM_MAX_EXTREME_ANGLE, 13);
    CppsSetParameter(CPP_PM_PARAM_MIN_EXTREME_ANGLE, 14);
    CppsSetParameter(CPP_PM_PARAM_TOP_DEAD_SPOT, 15);
    CppsSetParameter(CPP_PM_PARAM_BOTTOM_DEAD_SPOT, 16);
    CppsSetParameter(CPP_PM_PARAM_ACCUMULATED_ENERGY, 17);

    CppsSendPowerMeasurement(connId);
  }

  /* Configure and start timer to send the next measurement */
  cyclingCb.cpmTimer.msg.event = CYCLING_CPP_PM_TIMER_IND;
  cyclingCb.cpmTimer.msg.status = CYCLING_CPS_CPM_CCC_IDX;
  cyclingCb.cpmTimer.handlerId = cyclingCb.handlerId;
  cyclingCb.cpmTimer.msg.param = connId;

  WsfTimerStartSec(&cyclingCb.cpmTimer, cyclingCpmPeriod);
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
static void cyclingProcCccState(attsCccEvt_t *pMsg)
{
  APP_TRACE_INFO3("ccc state ind value:%d handle:%d idx:%d", pMsg->value, pMsg->handle, pMsg->idx);

  /* handle cycling power measurement CCC */
  if (pMsg->idx == CYCLING_CPS_CPM_CCC_IDX)
  {
    if (pMsg->value == ATT_CLIENT_CFG_NOTIFY)
    {
      cyclingSendCyclingPowerMeasurement((dmConnId_t)pMsg->hdr.param);
    }
    else
    {
      WsfTimerStop(&cyclingCb.cpmTimer);
    }
    return;
  }

  /* handle cycling speed measurement CCC */
  if (pMsg->idx == CYCLING_CSCS_CSM_CCC_IDX)
  {
    if (pMsg->value == ATT_CLIENT_CFG_NOTIFY)
    {
      cyclingSendCyclingSpeedMeasurement((dmConnId_t)pMsg->hdr.param);
    }
    else
    {
      WsfTimerStop(&cyclingCb.csmTimer);
    }
    return;
  }

  /* handle battery level CCC */
  if (pMsg->idx == CYCLING_BATT_LVL_CCC_IDX)
  {
    if (pMsg->value == ATT_CLIENT_CFG_NOTIFY)
    {
      BasMeasBattStart((dmConnId_t)pMsg->hdr.param, CYCLING_BATT_TIMER_IND, CYCLING_BATT_LVL_CCC_IDX);
    }
    else
    {
      BasMeasBattStop((dmConnId_t)pMsg->hdr.param);
    }
    return;
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
static void cyclingCccCback(attsCccEvt_t *pEvt)
{
  attsCccEvt_t  *pMsg;
  appDbHdl_t    dbHdl;

  /* If CCC not set from initialization and there's a bond record and currently bonded */
  if ((pEvt->handle != ATT_HANDLE_NONE) &&
      ((dbHdl = AppDbGetHdl((dmConnId_t)pEvt->hdr.param)) != APP_DB_HDL_NONE) &&
      AppCheckBonded((dmConnId_t)pEvt->hdr.param))
  {
    /* Store value in device database. */
    AppDbSetCccTblValue(dbHdl, pEvt->idx, pEvt->value);
  }

  if ((pMsg = WsfMsgAlloc(sizeof(attsCccEvt_t))) != NULL)
  {
    memcpy(pMsg, pEvt, sizeof(attsCccEvt_t));
    WsfMsgSend(cyclingCb.handlerId, pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     cyclingGetPeerKey
 *
 *  \brief  Get peer key from a device database record.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return Pointer to peer key if key is valid or NULL if not valid.
 */
/*************************************************************************************************/
static dmSecKey_t *cyclingGetPeerKey(dmEvt_t *pMsg)
{
  appDbHdl_t dbHdl;

  /* get device database record handle */
  dbHdl = AppDbGetHdl((dmConnId_t) pMsg->hdr.param);

  /* if database record handle valid */
  if (dbHdl != APP_DB_HDL_NONE)
  {
    return AppDbGetKey(dbHdl, DM_KEY_IRK, NULL);
  }

  return NULL;
}

/*************************************************************************************************/
/*!
*  \fn     datsPrivAddDevToResListInd
*
*  \brief  Handle add device to resolving list indication.
*
*  \param  pMsg    Pointer to DM callback event message.
*
*  \return None.
*/
/*************************************************************************************************/
static void cyclingPrivAddDevToResListInd(dmEvt_t *pMsg)
{
  dmSecKey_t *pPeerKey;

  /* if peer IRK present */
  if ((pPeerKey = cyclingGetPeerKey(pMsg)) != NULL)
  {
    /* set advertising peer address */
    AppSetAdvPeerAddr(pPeerKey->irk.addrType, pPeerKey->irk.bdAddr);
  }
}

/*************************************************************************************************/
/*!
*  \fn     cyclingPrivRemDevFromResListInd
*
*  \brief  Handle remove device from resolving list indication.
*
*  \param  pMsg    Pointer to DM callback event message.
*
*  \return None.
*/
/*************************************************************************************************/
static void cyclingPrivRemDevFromResListInd(dmEvt_t *pMsg)
{
  if (pMsg->hdr.status == HCI_SUCCESS)
  {
    if (AppDbGetHdl((dmConnId_t) pMsg->hdr.param) != APP_DB_HDL_NONE)
    {
      uint8_t addrZeros[BDA_ADDR_LEN] = { 0 };

      /* clear advertising peer address and its type */
      AppSetAdvPeerAddr(HCI_ADDR_TYPE_PUBLIC, addrZeros);
    }
  }
}

/*************************************************************************************************/
/*!
 *  \fn     cyclingSetup
 *
 *  \brief  Set up advertising and other procedures that need to be performed after
 *          device reset.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void cyclingSetup(dmEvt_t *pMsg)
{
  /* set advertising and scan response data for discoverable mode */
  AppAdvSetData(APP_ADV_DATA_DISCOVERABLE, sizeof(cyclingAdvDataDisc), (uint8_t *) cyclingAdvDataDisc);
  AppAdvSetData(APP_SCAN_DATA_DISCOVERABLE, sizeof(cyclingScanDataDisc), (uint8_t *) cyclingScanDataDisc);

  /* set advertising and scan response data for connectable mode */
  AppAdvSetData(APP_ADV_DATA_CONNECTABLE, sizeof(cyclingAdvDataDisc), (uint8_t *) cyclingAdvDataDisc);
  AppAdvSetData(APP_SCAN_DATA_CONNECTABLE, sizeof(cyclingScanDataDisc), (uint8_t *) cyclingScanDataDisc);

  /* start advertising; automatically set connectable/discoverable mode and bondable mode */
  AppAdvStart(APP_MODE_AUTO_INIT);
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
static void cyclingProcMsg(dmEvt_t *pMsg)
{
  uint8_t uiEvent = APP_UI_NONE;

  switch(pMsg->hdr.event)
  {
    case CYCLING_CPP_PM_TIMER_IND:
      cyclingSendCyclingPowerMeasurement((dmConnId_t) pMsg->hdr.param);
      break;

    case CYCLING_CSCP_SM_TIMER_IND:
      cyclingSendCyclingSpeedMeasurement((dmConnId_t) pMsg->hdr.param);
      break;

    case ATTS_CCC_STATE_IND:
      cyclingProcCccState((attsCccEvt_t*) pMsg);
      break;

    case ATT_MTU_UPDATE_IND:
      APP_TRACE_INFO1("Negotiated MTU %d", ((attEvt_t *)pMsg)->mtu);
      break;

    case DM_RESET_CMPL_IND:
      AttsCalculateDbHash();
      DmSecGenerateEccKeyReq();
      cyclingSetup(pMsg);
      uiEvent = APP_UI_RESET_CMPL;
      break;

    case DM_ADV_START_IND:
      uiEvent = APP_UI_ADV_START;
      break;

    case DM_ADV_STOP_IND:
      uiEvent = APP_UI_ADV_STOP;
      break;

    case DM_CONN_OPEN_IND:
      CppsConnOpen((dmConnId_t)pMsg->hdr.param);
      uiEvent = APP_UI_CONN_OPEN;
      break;

    case DM_CONN_CLOSE_IND:
      WsfTimerStop(&cyclingCb.cpmTimer);
      WsfTimerStop(&cyclingCb.csmTimer);
      uiEvent = APP_UI_CONN_CLOSE;
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
      AppHandlePasskey(&pMsg->authReq);
      break;

    case DM_SEC_COMPARE_IND:
      AppHandleNumericComparison(&pMsg->cnfInd);
      break;

    case DM_PRIV_ADD_DEV_TO_RES_LIST_IND:
      cyclingPrivAddDevToResListInd(pMsg);
      break;

    case DM_PRIV_REM_DEV_FROM_RES_LIST_IND:
      cyclingPrivRemDevFromResListInd(pMsg);
      break;

    case DM_ADV_NEW_ADDR_IND:
      break;

    case DM_VENDOR_SPEC_CMD_CMPL_IND:
      {
        #if defined(AM_PART_APOLLO) || defined(AM_PART_APOLLO2)

          uint8_t *param_ptr = &pMsg->vendorSpecCmdCmpl.param[0];

          switch (pMsg->vendorSpecCmdCmpl.opcode)
          {
            case 0xFC20: //read at address
            {
              uint32_t read_value;

              BSTREAM_TO_UINT32(read_value, param_ptr);

              APP_TRACE_INFO3("VSC 0x%0x complete status %x param %x",
                pMsg->vendorSpecCmdCmpl.opcode,
                pMsg->hdr.status,
                read_value);
            }

            break;
            default:
                APP_TRACE_INFO2("VSC 0x%0x complete status %x",
                    pMsg->vendorSpecCmdCmpl.opcode,
                    pMsg->hdr.status);
            break;
          }

        #endif
      }
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
void CyclingHandlerInit(wsfHandlerId_t handlerId)
{
  APP_TRACE_INFO0("CyclingHandlerInit");

  /* store handler ID */
  cyclingCb.handlerId = handlerId;

  /* Set configuration pointers */
  pAppSlaveCfg = (appSlaveCfg_t *) &cyclingSlaveCfg;
  pAppAdvCfg = (appAdvCfg_t *) &cyclingAdvCfg;
  pAppSecCfg = (appSecCfg_t *) &cyclingSecCfg;
  pAppUpdateCfg = (appUpdateCfg_t *) &cyclingUpdateCfg;
  pAttCfg = (attCfg_t *) &cyclingAttCfg;

  /* Initialize application framework */
  AppSlaveInit();
  AppServerInit();

  /* Set IRK for the local device */
  DmSecSetLocalIrk(localIrk);
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
void CyclingHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
  if (pMsg != NULL)
  {
    APP_TRACE_INFO1("Cycling got evt %d", pMsg->event);

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
    cyclingProcMsg((dmEvt_t *) pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void CyclingStart(void)
{
  /* Register for stack callbacks */
  DmRegister(cyclingDmCback);
  DmConnRegister(DM_CLIENT_ID_APP, cyclingDmCback);
  AttRegister(cyclingAttCback);
  AttConnRegister(AppServerConnCback);
  AttsCccRegister(CYCLING_NUM_CCC_IDX, (attsCccSet_t *) cyclingCccSet, cyclingCccCback);

  /* Initialize attribute server database */
  SvcCoreGattCbackRegister(GattReadCback, GattWriteCback);
  SvcCoreAddGroup();
  SvcCpsAddGroup();
  SvcCscsAddGroup();
  SvcBattCbackRegister(BasReadCback, NULL);
  SvcBattAddGroup();

  /* Set Service Changed CCCD index. */
  GattSetSvcChangedIdx(CYCLING_GATT_SC_CCC_IDX);

  /* Set the cycling power features */
  CppsSetFeatures(CPP_ALL_FEATURES);

  /* Set the cycling speed and cadence features */
  CscpsSetFeatures(CSCS_ALL_FEATURES);

  /* Reset the device */
  DmDevReset();
}
