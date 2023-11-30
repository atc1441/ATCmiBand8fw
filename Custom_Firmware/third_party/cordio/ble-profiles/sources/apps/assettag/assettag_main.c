/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Asset Tracking Tag sample application.
 *
 *  Copyright (c) 2018 - 2019 Arm Ltd. All Rights Reserved.
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
#include "svc_cte.h"
#include "svc_batt.h"
#include "bas/bas_api.h"
#include "atps/atps_api.h"
#include "gatt/gatt_api.h"
#include "util/calc128.h"
#include "assettag_api.h"
#include "atts_main.h"
/**************************************************************************************************
  Macros
**************************************************************************************************/

uint8_t advHandle[DM_NUM_ADV_SETS]  = {0, 1};

static const appExtAdvCfg_t assetTagExtAdvCfg =
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
#endif
};

uint8_t assetTagExtAdvDataDisc[] =
{
    /*! flags */
    2,                                      /*! length */
    DM_ADV_TYPE_FLAGS,                      /*! AD type */
    DM_FLAG_LE_GENERAL_DISC |               /*! flags */
    DM_FLAG_LE_BREDR_NOT_SUP,


    /*! device name */
    /*! device name */
    10,                                      /*! length */
    DM_ADV_TYPE_LOCAL_NAME,                  /*! AD type */
    'A',
    's',
    's',
    'e',
    't',
    ' ',
    'T',
    'a',
    'g',
};


/*! Enumeration of client characteristic configuration descriptors */
enum
{
  ASSETTAG_GATT_SC_CCC_IDX,               /*! GATT service, service changed characteristic */
  ASSETTAG_BATT_LVL_CCC_IDX,              /*! Battery service, battery level characteristic */
  ASSETTAG_NUM_CCC_IDX
};

/*! WSF message event starting value */
#define ASSETTAG_MSG_START                0xA0

/*! WSF message event enumeration */
enum
{
  ASSETTAG_BATT_TIMER_IND = ASSETTAG_MSG_START,     /*! Battery measurement timer expired */
};

/*! Default MTU */
#define ASSETTAG_DEFAULT_MTU              50

/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/
/*! configurable parameters for slave */
static const appSlaveCfg_t assetTagSlaveCfg =
{
  DM_NUM_ADV_SETS,                                      /*! Maximum connections */
};

/*! configurable parameters for security */
static const appSecCfg_t assetTagSecCfg =
{
  DM_AUTH_BOND_FLAG | SMP_AUTH_SC_FLAG,   /*! Authentication and bonding flags */
  DM_KEY_DIST_IRK,                        /*! Initiator key distribution flags */
  DM_KEY_DIST_LTK | DM_KEY_DIST_IRK,      /*! Responder key distribution flags */
  FALSE,                                  /*! TRUE if Out-of-band pairing data is present */
  TRUE                                    /*! TRUE to initiate security upon connection */
};

/*! configurable parameters for connection parameter update */
static const appUpdateCfg_t assetTagUpdateCfg =
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
static const attCfg_t assetTagAttCfg =
{
  15,                                     /* ATT server service discovery connection idle timeout in seconds */
  ASSETTAG_DEFAULT_MTU,                   /* desired ATT MTU */
  ATT_MAX_TRANS_TIMEOUT,                  /* transcation timeout in seconds */
  4                                       /* number of queued prepare writes supported by server */
};

/*! local IRK */
static uint8_t localIrk[] =
{
  0x95, 0xC8, 0xEE, 0x6F, 0xC5, 0x0D, 0xEF, 0x93, 0x35, 0x4E, 0x7C, 0x57, 0x08, 0xE2, 0xA3, 0x85
};

/**************************************************************************************************
  Advertising Data
**************************************************************************************************/

/*! scan data, discoverable mode */
static const uint8_t assetTagScanDataDisc[] =
{
  /*! device name */
  10,                                      /*! length */
  DM_ADV_TYPE_LOCAL_NAME,                  /*! AD type */
  'A',
  's',
  's',
  'e',
  't',
  ' ',
  'T',
  'a',
  'g',
};
extern appExtConnCb_t appExtConnCb[DM_CONN_MAX];

/**************************************************************************************************
  Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t assetTagCccSet[ASSETTAG_NUM_CCC_IDX] =
{
  /* cccd handle          value range               security level */
  { GATT_SC_CH_CCC_HDL,    ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_NONE },   /* ASSETTAG_GATT_SC_CCC_IDX */
  { BATT_LVL_CH_CCC_HDL,   ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE }    /* ASSETTAG_BATT_LVL_CCC_IDX */
};

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! application control block */
static struct
{
  wsfHandlerId_t    handlerId;        /* WSF handler ID */
} assetTagCb;

/* Identifiers for antenna */
static uint8_t assetTagAntennaIds[] = {0, 1};

/*************************************************************************************************/
/*!
 *  \brief  Application DM callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void assetTagDmCback(dmEvt_t *pDmEvt)
{
  dmEvt_t *pMsg;

  if (pDmEvt->hdr.event == DM_SEC_ECC_KEY_IND)
  {
    DmSecSetEccKey(&pDmEvt->eccMsg.data.key);
    // Only calculate database hash if the calculating status is in progress
    if( attsCsfGetHashUpdateStatus() )
    {
      AttsCalculateDbHash();
    }
  }
  else
  {
    uint16_t len = DmSizeOfEvt(pDmEvt);

    if ((pMsg = WsfMsgAlloc(len)) != NULL)
    {
      memcpy(pMsg, pDmEvt, len);
      WsfMsgSend(assetTagCb.handlerId, pMsg);
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
static void assetTagAttCback(attEvt_t *pEvt)
{
  attEvt_t  *pMsg;

  if ((pMsg = WsfMsgAlloc(sizeof(attEvt_t) + pEvt->valueLen)) != NULL)
  {
    memcpy(pMsg, pEvt, sizeof(attEvt_t));
    pMsg->pValue = (uint8_t *)(pMsg + 1);
    memcpy(pMsg->pValue, pEvt->pValue, pEvt->valueLen);
    WsfMsgSend(assetTagCb.handlerId, pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Perform actions on connection open.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void assetTagOpen(dmEvt_t *pMsg)
{
  /* Set the antenna identifiers for the connection */
  AtpsSetAntennaIds((dmConnId_t) pMsg->hdr.param, sizeof(assetTagAntennaIds), assetTagAntennaIds);
}

/*************************************************************************************************/
/*!
 *  \brief  Application ATTS client characteristic configuration callback.
 *
 *  \param  pMsg    DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void assetTagProcCccState(attsCccEvt_t *pMsg)
{
  APP_TRACE_INFO3("ccc state ind value:%d handle:%d idx:%d", pMsg->value, pMsg->handle, pMsg->idx);

  /* handle battery level CCC */
  if (pMsg->idx == ASSETTAG_BATT_LVL_CCC_IDX)
  {
    if (pMsg->value == ATT_CLIENT_CFG_NOTIFY)
    {
      BasMeasBattStart((dmConnId_t)pMsg->hdr.param, ASSETTAG_BATT_TIMER_IND, ASSETTAG_BATT_LVL_CCC_IDX);
    }
    else
    {
      BasMeasBattStop((dmConnId_t)pMsg->hdr.param);
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Application ATTS client characteristic configuration callback.
 *
 *  \param  pEvt    DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void assetTagCccCback(attsCccEvt_t *pEvt)
{
  attsCccEvt_t  *pMsg;
  appDbHdl_t    dbHdl;

  /* if CCC not set from initialization and there's a device record and currently bonded */
  if ((pEvt->handle != ATT_HANDLE_NONE) &&
      ((dbHdl = AppDbGetHdl((dmConnId_t)pEvt->hdr.param)) != APP_DB_HDL_NONE) &&
      AppCheckBonded((dmConnId_t)pEvt->hdr.param))
  {
    /* store value in device database */
    AppDbSetCccTblValue(dbHdl, pEvt->idx, pEvt->value);
  }

  if ((pMsg = WsfMsgAlloc(sizeof(attsCccEvt_t))) != NULL)
  {
    memcpy(pMsg, pEvt, sizeof(attsCccEvt_t));
    WsfMsgSend(assetTagCb.handlerId, pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Get peer key from a device database record.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return Pointer to peer key if key is valid or NULL if not valid.
 */
/*************************************************************************************************/
static dmSecKey_t *assetTagGetPeerKey(dmEvt_t *pMsg)
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
 *  \brief  Handle add device to resolving list indication.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void assetTagPrivAddDevToResListInd(dmEvt_t *pMsg)
{
  dmSecKey_t *pPeerKey;

  /* if peer IRK present */
  if ((pPeerKey = assetTagGetPeerKey(pMsg)) != NULL)
  {
    /* set advertising peer address */
    AppSetAdvPeerAddr(pPeerKey->irk.addrType, pPeerKey->irk.bdAddr);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Handle remove device from resolving list indication.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void assetTagPrivRemDevFromResListInd(dmEvt_t *pMsg)
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
 *  \brief  Set up advertising and other procedures that need to be performed after
 *          device reset.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void assetTagSetup(dmEvt_t *pMsg)
{
    uint8_t adv_set = 0;

    for(adv_set=0; adv_set<DM_NUM_ADV_SETS; adv_set++)
    {
        AppExtAdvSetData(advHandle[adv_set], APP_ADV_DATA_DISCOVERABLE, sizeof(assetTagExtAdvDataDisc), (uint8_t *) assetTagExtAdvDataDisc, HCI_EXT_ADV_DATA_LEN);
        AppExtAdvSetData(advHandle[adv_set], APP_SCAN_DATA_DISCOVERABLE, sizeof(assetTagScanDataDisc), (uint8_t *) assetTagScanDataDisc, HCI_EXT_ADV_DATA_LEN);
    }

    DmAdvSetPhyParam(advHandle[0], HCI_ADV_PHY_LE_1M, 0, HCI_ADV_PHY_LE_2M);
    DmAdvSetPhyParam(advHandle[1], HCI_ADV_PHY_LE_1M, 0, HCI_ADV_PHY_LE_1M);

    AppExtAdvStart(DM_NUM_ADV_SETS, advHandle, APP_MODE_AUTO_INIT);
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
static void assetTagProcMsg(dmEvt_t *pMsg)
{
  uint8_t uiEvent = APP_UI_NONE;
  uint8_t i = 0;

  switch(pMsg->hdr.event)
  {
    case ATTS_CCC_STATE_IND:
      assetTagProcCccState((attsCccEvt_t*) pMsg);
      break;

    case DM_RESET_CMPL_IND:
      // set database hash calculating status to true until a new hash is generated after reset
      attsCsfSetHashUpdateStatus(TRUE);

      // Generate ECC key if configured support secure connection,
      // else will calcualte ATT database hash
      if( assetTagSecCfg.auth & DM_AUTH_SC_FLAG )
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
      assetTagSetup(pMsg);
      break;

    case DM_CONN_OPEN_IND:
      assetTagOpen(pMsg);
      uiEvent = APP_UI_CONN_OPEN;
      break;

    case DM_CONN_CLOSE_IND:
    {
        APP_TRACE_INFO1("conn close reason = 0x%x\n", pMsg->connClose.reason);
        uint8_t connHdl = pMsg->connClose.handle;

        for(i = 0; i< DM_CONN_MAX; i++)
        {
            if(appExtConnCb[i].used && (connHdl==appExtConnCb[i].connHandle))
            {
                appExtConnCb[i].used = FALSE;
                AppExtAdvStart(1, &appExtConnCb[i].advHandle, APP_MODE_AUTO_INIT);

                break;
            }
        }
        uiEvent = APP_UI_CONN_CLOSE;
    }
    break;

    case DM_PHY_UPDATE_IND:
      APP_TRACE_INFO3("DM_PHY_UPDATE_IND status: %d, RX: %d, TX: %d", pMsg->phyUpdate.status,pMsg->phyUpdate.rxPhy, pMsg->phyUpdate.txPhy);
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
      assetTagPrivAddDevToResListInd(pMsg);
      break;

    case DM_PRIV_REM_DEV_FROM_RES_LIST_IND:
      assetTagPrivRemDevFromResListInd(pMsg);
      break;

    case DM_ADV_NEW_ADDR_IND:
      break;

    case DM_ADV_SET_START_IND:
      uiEvent = APP_UI_ADV_SET_START_IND;
      break;

    case DM_ADV_SET_STOP_IND:
    {
        for(i = 0; i< DM_CONN_MAX; i++)
        {
          if(!appExtConnCb[i].used)
          {
              appExtConnCb[i].used = TRUE;
              appExtConnCb[i].advHandle = pMsg->advSetStop.advHandle;
              appExtConnCb[i].connHandle = pMsg->advSetStop.handle;

              break;
          }
        }

        uiEvent = APP_UI_ADV_SET_STOP_IND;
    }
    break;

    case DM_SCAN_REQ_RCVD_IND:
      uiEvent = APP_UI_SCAN_REQ_RCVD_IND;
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
void AssetTagHandlerInit(wsfHandlerId_t handlerId)
{
  APP_TRACE_INFO0("AssetTagHandlerInit");

  /* store handler ID */
  assetTagCb.handlerId = handlerId;

  /* Set configuration pointers */
  pAppSlaveCfg = (appSlaveCfg_t *) &assetTagSlaveCfg;
  pAppExtAdvCfg = (appExtAdvCfg_t *) &assetTagExtAdvCfg;
  pAppSecCfg = (appSecCfg_t *) &assetTagSecCfg;
  pAppUpdateCfg = (appUpdateCfg_t *) &assetTagUpdateCfg;
  pAttCfg = (attCfg_t *) &assetTagAttCfg;

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
void AssetTagHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
  if (pMsg != NULL)
  {
    APP_TRACE_INFO1("AssetTag got evt %d", pMsg->event);

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

      /* process asset tracking profile-related messages */
      AtpsProcDmMsg((dmEvt_t *) pMsg);
    }

    /* perform profile and user interface-related operations */
    assetTagProcMsg((dmEvt_t *) pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AssetTagStart(void)
{
  /* Register for stack callbacks */
  DmRegister(assetTagDmCback);
  DmConnRegister(DM_CLIENT_ID_APP, assetTagDmCback);
  AttRegister(assetTagAttCback);
  AttConnRegister(AppServerConnCback);
  AttsCccRegister(ASSETTAG_NUM_CCC_IDX, (attsCccSet_t *) assetTagCccSet, assetTagCccCback);

  /* Initialize attribute server database */
  SvcCoreGattCbackRegister(GattReadCback, GattWriteCback);
  SvcCoreAddGroup();
  SvcBattCbackRegister(BasReadCback, NULL);
  SvcBattAddGroup();

  /* Set Service Changed CCCD index. */
  GattSetSvcChangedIdx(ASSETTAG_GATT_SC_CCC_IDX);

  /* Initialize Asset Tracking Profile */
  AtpsInit();

  /* Reset the device */
  DmDevReset();
}
