/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Application framework main module.
 *
 *  Copyright (c) 2011-2019 Arm Ltd. All Rights Reserved.
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
#include "wsf_msg.h"
#include "sec_api.h"
#include "wsf_trace.h"
#include "wsf_timer.h"
#include "wsf_assert.h"
#include "util/bstream.h"
#include "dm_api.h"
#include "app_api.h"
#include "app_main.h"
#include "app_ui.h"

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! Configuration pointer for advertising */
appAdvCfg_t *pAppAdvCfg;

/*! Configuration pointer for extended and periodic advertising */
appExtAdvCfg_t *pAppExtAdvCfg;

/*! Configuration pointer for slave */
appSlaveCfg_t *pAppSlaveCfg;

/*! Configuration pointer for master */
appMasterCfg_t *pAppMasterCfg;

/*! Configuration pointer for extended master */
appExtMasterCfg_t *pAppExtMasterCfg;

/*! Configuration pointer for security */
appSecCfg_t *pAppSecCfg;

/*! Configuration pointer for connection parameter update */
appUpdateCfg_t *pAppUpdateCfg;

/*! Configuration pointer for discovery */
appDiscCfg_t *pAppDiscCfg;

/*! Configuration pointer for application */
appCfg_t *pAppCfg;

/*! Connection control block array */
appConnCb_t appConnCb[DM_CONN_MAX];

appExtConnCb_t appExtConnCb[DM_CONN_MAX];

/*! WSF handler ID */
wsfHandlerId_t appHandlerId;

/*! Main control block */
appCb_t appCb;

/*! Configuration structure for incoming request actions */
const appReqActCfg_t appReqActCfg =
{
  APP_ACT_ACCEPT        /*! Action for the remote connection parameter request */
};

/*! Configuration pointer for incoming request actions on master */
appReqActCfg_t *pAppMasterReqActCfg = (appReqActCfg_t *) &appReqActCfg;

/*! Configurable pointer for incoming request actions on slave */
appReqActCfg_t *pAppSlaveReqActCfg = (appReqActCfg_t *) &appReqActCfg;

/*************************************************************************************************/
/*!
 *  \brief  Process messages from the event handler.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appProcMsg(wsfMsgHdr_t *pMsg)
{
  switch(pMsg->event)
  {
    case APP_BTN_POLL_IND:
      appUiBtnPoll();
      break;

    case APP_UI_TIMER_IND:
      appUiTimerExpired(pMsg);
      break;

    case APP_HCI_READ_REMOTE_FEAT:
      DmReadRemoteFeatures((dmConnId_t) pMsg->param);
      break;

    default:
      break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Check the bond-by-LTK state of a connection.
 *
 *  \param  connId      DM connection ID.
 *
 *  \return Bond-by-LTK state.
 */
/*************************************************************************************************/
bool_t appCheckBondByLtk(dmConnId_t connId)
{
  WSF_ASSERT((connId > 0) && (connId <= DM_CONN_MAX));

  return appConnCb[connId - 1].bondByLtk;
}

/*************************************************************************************************/
/*!
 *  \brief  Return the number of existing connections of the given role.
 *
 *  \param  role      Connection role
 *
 *  \return Number of connections.
 */
/*************************************************************************************************/
uint8_t appNumConns(uint8_t role)
{
  appConnCb_t   *pCcb = appConnCb;
  uint8_t       i, j;

  for (i = DM_CONN_MAX, j = 0; i > 0; i--, pCcb++)
  {
    if ((pCcb->connId != DM_CONN_ID_NONE) && (DmConnRole(pCcb->connId) == role))
    {
      j++;
    }
  }

  return j;
}

/*************************************************************************************************/
/*!
 *  \brief  Check the bonded state of a connection.
 *
 *  \param  connId      DM connection ID.
 *
 *  \return Bonded state.
 */
/*************************************************************************************************/
bool_t AppCheckBonded(dmConnId_t connId)
{
  WSF_ASSERT((connId > 0) && (connId <= DM_CONN_MAX));

  return appConnCb[connId - 1].bonded;
}

/*************************************************************************************************/
/*!
 *  \brief  App framework handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID for App.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppHandlerInit(wsfHandlerId_t handlerId)
{
  appHandlerId = handlerId;

  AppDbInit();
}

/*************************************************************************************************/
/*!
 *  \brief  WSF event handler for app framework.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
  if (pMsg != NULL)
  {
    APP_TRACE_INFO1("App got evt %d", pMsg->event);

    if (pMsg->event >= APP_MASTER_MSG_START)
    {
      /* pass event to master handler */
      (*appCb.masterCback)(pMsg);
    }
    else if (pMsg->event >= APP_SLAVE_MSG_START)
    {
      /* pass event to slave handler */
      (*appCb.slaveCback)(pMsg);
    }
    else
    {
      appProcMsg(pMsg);
    }
  }
  else
  {
    if (event & APP_BTN_DOWN_EVT)
    {
      AppUiBtnPressed();
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Handle a passkey request during pairing.  If the passkey is to displayed, a
 *          random passkey is generated and displayed.  If the passkey is to be entered
 *          the user is prompted to enter the passkey.
 *
 *  \param  pAuthReq  DM authentication requested event structure.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppHandlePasskey(dmSecAuthReqIndEvt_t *pAuthReq)
{
  uint32_t passkey;
  uint8_t  buf[SMP_PIN_LEN];

  if (pAuthReq->display)
  {
    /* generate random passkey, limit to 6 digit max */
    SecRand((uint8_t *) &passkey, sizeof(uint32_t));
    passkey %= 1000000;

    /* convert to byte buffer */
    buf[0] = UINT32_TO_BYTE0(passkey);
    buf[1] = UINT32_TO_BYTE1(passkey);
    buf[2] = UINT32_TO_BYTE2(passkey);

    /* send authentication response to DM */
    DmSecAuthRsp((dmConnId_t) pAuthReq->hdr.param, SMP_PIN_LEN, buf);

    /* display passkey */
    AppUiDisplayPasskey(passkey);
  }
  else
  {
    /* prompt user to enter passkey */
    AppUiAction(APP_UI_PASSKEY_PROMPT);
  }
}

/*************************************************************************************************/
/*!
*  \brief  Handle a numeric comparison indication during pairing.  The confirmation value is
*          displayed and the user is prompted to verify that the local and peer confirmation
*          values match.
*
*  \param  pCnfInd  DM confirmation indication event structure.
*
*  \return None.
*/
/*************************************************************************************************/
void AppHandleNumericComparison(dmSecCnfIndEvt_t *pCnfInd)
{
  uint32_t confirm = DmSecGetCompareValue(pCnfInd->confirm);

  /* display confirmation value */
  AppUiDisplayConfirmValue(confirm);

  DmSecCompareRsp((dmConnId_t)pCnfInd->hdr.param, TRUE);
}

/*************************************************************************************************/
/*!
 *  \brief  Close the connection with the give connection identifier.
 *
 *  \param  connId    Connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppConnClose(dmConnId_t connId)
{
  DmConnClose(DM_CLIENT_ID_APP, connId, HCI_ERR_REMOTE_TERMINATED);
}

/*************************************************************************************************/
/*!
 *  \brief  Get a list of connection identifiers of open connections.
 *
 *  \param  pConnIdList    Buffer to hold connection IDs (must be DM_CONN_MAX bytes).
 *
 *  \return Number of open connections.
 *
 */
/*************************************************************************************************/
uint8_t AppConnOpenList(dmConnId_t *pConnIdList)
{
  appConnCb_t   *pCcb = appConnCb;
  uint8_t       i;
  uint8_t       pos = 0;

  memset(pConnIdList, DM_CONN_ID_NONE, DM_CONN_MAX);

  for (i = DM_CONN_MAX; i > 0; i--, pCcb++)
  {
    if (pCcb->connId != DM_CONN_ID_NONE)
    {
      pConnIdList[pos++] = pCcb->connId;
    }
  }

  return pos;
}

/*************************************************************************************************/
/*!
 *  \brief  Check if a connection is open.
 *
 *  \return Connection ID of open connection or DM_CONN_ID_NONE if no open connections.
 */
/*************************************************************************************************/
dmConnId_t AppConnIsOpen(void)
{
  appConnCb_t   *pCcb = appConnCb;
  uint8_t       i;

  for (i = DM_CONN_MAX; i > 0; i--, pCcb++)
  {
    if (pCcb->connId != DM_CONN_ID_NONE)
    {
      return pCcb->connId;
    }
  }

  return DM_CONN_ID_NONE;
}

/*************************************************************************************************/
/*!
 *  \brief  Get the device database record handle associated with an open connection.
 *
 *  \param  connId    Connection identifier.
 *
 *  \return Database record handle or APP_DB_HDL_NONE.
 */
/*************************************************************************************************/
appDbHdl_t AppDbGetHdl(dmConnId_t connId)
{
  return appConnCb[connId-1].dbHdl;
}

/*************************************************************************************************/
/*!
 *  \brief  Add device to resolving list.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *  \param  connId  Connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppAddDevToResList(dmEvt_t *pMsg, dmConnId_t connId)
{
  dmSecKey_t *pPeerKey;
  appDbHdl_t hdl = appConnCb[connId - 1].dbHdl;

  /* if LL Privacy is supported and the peer device has distributed its IRK */
  if (HciLlPrivacySupported() && ((pPeerKey = AppDbGetKey(hdl, DM_KEY_IRK, NULL))!= NULL))
  {
    /* add peer device to resolving list. If all-zero local or peer IRK is used then
       LL will only use or accept local or peer identity address respectively. */
    DmPrivAddDevToResList(pPeerKey->irk.addrType, pPeerKey->irk.bdAddr, pPeerKey->irk.key,
                          DmSecGetLocalIrk(), TRUE, pMsg->hdr.param);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Add next device to resolving list. For the first device, the function should be
 *          called with 'hdl' set to 'APP_DB_HDL_NONE'.
 *
 *  \param  hdl     The last handle returned by AppAddNextDevToResList or APP_DB_HDL_NONE to begin.
 *
 *  \return The handle being restored or APP_DB_HDL_NONE when the operation is complete.
 *
 *  \note   Applications supporting address resolution should call this functions after DmDevReset.
 *          This function will restore the resolving list in the Controller using information in the
 *          app device database.
 *
 *  \note   After each device is added to resolving list, the DM will send
 *          DM_PRIV_ADD_DEV_TO_RES_LIST_IND to the application.  The application must call
 *          AppAddNextDevToResList again to continue the restore process until
 *          AppAddNextDevToResList returns APP_DB_HDL_NONE.
 */
/*************************************************************************************************/
appDbHdl_t AppAddNextDevToResList(appDbHdl_t hdl)
{
  appDbHdl_t nextHdl;

  /* Complete restoration of the previous record. */
  if (hdl != APP_DB_HDL_NONE)
  {
    if (!AppDbGetPeerRpao(hdl))
    {
      dmSecKey_t *pPeerKey = AppDbGetKey(hdl, DM_KEY_IRK, NULL);

      WSF_ASSERT(pPeerKey);

      /* Update device privacy mode. */
      DmPrivSetPrivacyMode(pPeerKey->irk.addrType, pPeerKey->irk.bdAddr, DM_PRIV_MODE_DEVICE);
    }
  }

  /* Look for the next record with an IRK. */
  nextHdl = AppDbGetNextRecord(hdl);

  while (nextHdl != APP_DB_HDL_NONE)
  {
    dmSecKey_t *pPeerKey = AppDbGetKey(nextHdl, DM_KEY_IRK, NULL);

    if (pPeerKey)
    {
      /* Add the device to the resolving list. */
      DmPrivAddDevToResList(pPeerKey->irk.addrType, pPeerKey->irk.bdAddr, pPeerKey->irk.key,
                            DmSecGetLocalIrk(), FALSE, DM_CONN_ID_NONE);

      return nextHdl;
    }

    nextHdl = AppDbGetNextRecord(nextHdl);
  }

  if (hdl != APP_DB_HDL_NONE)
  {
    /* If any device was added to the resolving list. Enable address resolution in LL. */
    DmPrivSetAddrResEnable(TRUE);
  }

  return APP_DB_HDL_NONE;
}

/*************************************************************************************************/
/*!
 *  \brief  Clear all bonding information.
 *
 *  \return None.
 *
 *  \Note   This API should not be used when:
 *          - Advertising (other than periodic advertising) is enabled,
 *          - Scanning is enabled, or
 *          - (Extended) Create connection or Create Sync command is outstanding.
 *
 *          Otherwise, clearing the resolving list in the Controller may fail.
 */
/*************************************************************************************************/
void AppClearAllBondingInfo(void)
{
  APP_TRACE_INFO0("Clear bonding info");

  /* clear bonded device info */
  AppDbDeleteAllRecords();

  /* if LL Privacy is supported */
  if (HciLlPrivacySupported())
  {
    /* if LL Privacy has been enabled */
    if (DmLlPrivEnabled())
    {
      /* make sure LL Privacy is disabled before clearing resolving list */
      DmPrivSetAddrResEnable(FALSE);
    }

    /* clear resolving list */
    DmPrivClearResList();
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Update privacy mode for a given peer device.
 *
 *  \param  hdl     Database record handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppUpdatePrivacyMode(appDbHdl_t hdl)
{
  /* if peer device's been added to resolving list but RPA Only attribute not found on peer device */
  if ((hdl != APP_DB_HDL_NONE) && AppDbGetPeerAddedToRl(hdl) && !AppDbGetPeerRpao(hdl))
  {
    dmSecKey_t *pPeerKey = AppDbGetKey(hdl, DM_KEY_IRK, NULL);
    if (pPeerKey != NULL)
    {
      /* set device privacy mode for this peer device */
      DmPrivSetPrivacyMode(pPeerKey->irk.addrType, pPeerKey->irk.bdAddr, DM_PRIV_MODE_DEVICE);

      /* make sure resolving list flag cleared */
      AppDbSetPeerAddedToRl(hdl, FALSE);
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Start the reading remote feature timer after connection establish.
 *
 *  \param  connId    DM connection ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void appConnReadRemoteFeatTimerStart(dmConnId_t connId)
{
  /* look up app connection control block from DM connection ID */
  appConnCb_t *pCb = &appConnCb[connId - 1];

  pCb->readRemoteFeatTimer.handlerId = appHandlerId;
  pCb->readRemoteFeatTimer.msg.event = APP_HCI_READ_REMOTE_FEAT;
  pCb->readRemoteFeatTimer.msg.param = connId;
  WsfTimerStartMs(&pCb->readRemoteFeatTimer, APP_TIME_READ_REMOTE_FEAT_MS);
}

