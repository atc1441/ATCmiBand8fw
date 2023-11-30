/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Application framework module for master.
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

#include "wsf_types.h"
#include "wsf_msg.h"
#include "wsf_timer.h"
#include "wsf_trace.h"
#include "wsf_assert.h"
#include "dm_api.h"
#include "att_api.h"
#include "svc_core.h"
#include "app_api.h"
#include "app_main.h"
#include "app_cfg.h"
#include <stdbool.h>
#include "hci_drv_apollo.h"
#include "dm_api.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/* Constant used in the address type indicating value not present */
#define APP_ADDR_NONE             0xFF

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/* Master control block */
appMasterCb_t appMasterCb;

/*************************************************************************************************/
/*!
 *  \brief  Initiate security
 *
 *  \param  connId           Connection ID.
 *  \param  initiatePairing  TRUE to initiate pairing.
 *  \param  pCb              Connection control block.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appMasterInitiateSec(dmConnId_t connId, bool_t initiatePairing, appConnCb_t *pCb)
{
  uint8_t     rKeyDist;
  uint8_t     secLevel;
  dmSecKey_t  *pKey;

  /* if we have an LTK for peer device */
  if ((pCb->dbHdl != APP_DB_HDL_NONE) &&
      ((pKey = AppDbGetKey(pCb->dbHdl, DM_KEY_PEER_LTK, &secLevel)) != NULL))
  {
    pCb->bondByLtk = TRUE;
    pCb->initiatingSec = TRUE;

    /* encrypt with LTK */
    DmSecEncryptReq(connId, secLevel, &pKey->ltk);
  }
  /* no key; initiate pairing only if requested */
  else if (initiatePairing)
  {
    /* store bonding state */
    pCb->bondByPairing = (pAppSecCfg->auth & DM_AUTH_BOND_FLAG) == DM_AUTH_BOND_FLAG;

    /* if bonding and no device record */
    if (pCb->bondByPairing && pCb->dbHdl == APP_DB_HDL_NONE)
    {
      /* create a device record if none exists */      
      pCb->dbHdl = AppDbNewRecord(DmConnPeerAddrType(connId), DmConnPeerAddr(connId), TRUE);
    }

    /* initialize stored keys */
    pCb->rcvdKeys = 0;

    /* if peer is using random address request IRK */
    rKeyDist = pAppSecCfg->rKeyDist;
    if (DmConnPeerAddrType(connId) == DM_ADDR_RANDOM)
    {
      rKeyDist |= DM_KEY_DIST_IRK;
    }

    pCb->initiatingSec = TRUE;

    /* initiate pairing */
    DmSecPairReq(connId, pAppSecCfg->oob, pAppSecCfg->auth, pAppSecCfg->iKeyDist, rKeyDist);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Clear all scan results.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appScanResultsClear(void)
{
  uint8_t       i;
  appDevInfo_t  *pDev = appMasterCb.scanResults;

  appMasterCb.numScanResults = 0;
  for (i = APP_SCAN_RESULT_MAX; i > 0; i--, pDev++)
  {
    pDev->addrType = APP_ADDR_NONE;
  }

  /* end address resolution */
  appMasterCb.inProgress = FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Add a scan report to the scan result list.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appScanResultAdd(dmEvt_t *pMsg)
{
  uint8_t       i;
  appDevInfo_t  *pDev = appMasterCb.scanResults;

  /* see if device is in list already */
  for (i = 0; i < APP_SCAN_RESULT_MAX; i++, pDev++)
  {
    if(DmScanModeLeg())
    {
        /* if address matches list entry */
        if ((pDev->addrType == pMsg->scanReport.addrType) &&
            BdaCmp(pDev->addr, pMsg->scanReport.addr))
        {
          /* device already exists in list; we are done */
          break;
        }
        /* if entry is free end then of list has been reached */
        else if (pDev->addrType == APP_ADDR_NONE)
        {
          /* add device to list */
          pDev->addrType = pMsg->scanReport.addrType;
          BdaCpy(pDev->addr, pMsg->scanReport.addr);
          pDev->directAddrType = pMsg->scanReport.directAddrType;
          BdaCpy(pDev->directAddr, pMsg->scanReport.directAddr);
          appMasterCb.numScanResults++;
          break;
        }
    }
    else
    {
        /* if address matches list entry */
        if ((pDev->addrType == pMsg->extScanReport.addrType) &&
            BdaCmp(pDev->addr, pMsg->extScanReport.addr))
        {
          /* device already exists in list; we are done */
          break;
        }
        /* if entry is free end then of list has been reached */
        else if (pDev->addrType == APP_ADDR_NONE)
        {
          /* add device to list */
          pDev->addrType = pMsg->extScanReport.addrType;
          pDev->secPhy = pMsg->extScanReport.secPhy;
          BdaCpy(pDev->addr, pMsg->extScanReport.addr);
          pDev->directAddrType = pMsg->extScanReport.directAddrType;
          BdaCpy(pDev->directAddr, pMsg->extScanReport.directAddr);
          appMasterCb.numScanResults++;
          break;
        }
    }
  }
}

/*************************************************************************************************/
/*!
*  \brief  Find a scan report in the scan result list.
*
*  \param  pMsg    Pointer to DM callback event message.
*
*  \return Index of result in scan result list. APP_SCAN_RESULT_MAX, otherwise.
*/
/*************************************************************************************************/
static uint8_t appScanResultFind(dmEvt_t *pMsg)
{
  uint8_t       i;
  appDevInfo_t  *pDev = appMasterCb.scanResults;

  /* see if device is in list already */
  for (i = 0; i < APP_SCAN_RESULT_MAX; i++, pDev++)
  {
    if(DmScanModeLeg())
    {
        /* if address matches list entry */
        if ((pDev->addrType == pMsg->scanReport.addrType) &&
            BdaCmp(pDev->addr, pMsg->scanReport.addr))
        {
          /* device already exists in list; we are done */
          break;
        }
    }
    else
    {
        /* if address matches list entry */
        if ((pDev->addrType == pMsg->extScanReport.addrType) &&
            BdaCmp(pDev->addr, pMsg->extScanReport.addr))
        {
          /* device already exists in list; we are done */
          break;
        }
    }
  }

  return i;
}

/*************************************************************************************************/
/*!
 *  \brief  Handle a DM_SCAN_START_IND event.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appMasterScanStart(dmEvt_t *pMsg)
{
  if (pMsg->hdr.status == HCI_SUCCESS)
  {
    /* clear current scan results */
    appScanResultsClear();
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Handle a DM_SCAN_STOP_IND event.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appMasterScanStop(dmEvt_t *pMsg)
{
  if (pMsg->hdr.status == HCI_SUCCESS)
  {
    APP_TRACE_INFO1("Scan results: %d", AppScanGetNumResults());
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Handle a DM_SCAN_REPORT_IND event.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appMasterScanReport(dmEvt_t *pMsg)
{
  /* add to scan result list */
  appScanResultAdd(pMsg);
}

/*************************************************************************************************/
/*!
 *  \brief  Handle a DM_CONN_OPEN_IND event.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *  \param  pCb     Connection control block.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appMasterConnOpen(dmEvt_t *pMsg, appConnCb_t *pCb)
{
  appConnReadRemoteFeatTimerStart((dmConnId_t) pMsg->hdr.param);
}

/*************************************************************************************************/
/*!
 *  \brief  Handle a DM_CONN_CLOSE_IND event.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *  \param  pCb     Connection control block.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appMasterConnClose(dmEvt_t *pMsg, appConnCb_t *pCb)
{
  /* update privacy mode for peer device */
  AppUpdatePrivacyMode(pCb->dbHdl);

  /* clear connection ID */
  pCb->connId = DM_CONN_ID_NONE;

  /* cancel any address resolution in progress */
  appMasterCb.inProgress = FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Perform master security procedures on connection open.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *  \param  pCb     Connection control block.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appMasterSecConnOpen(dmEvt_t *pMsg, appConnCb_t *pCb)
{
  /* initialize state variables */
  pCb->bonded = FALSE;
  pCb->bondByLtk = FALSE;
  pCb->bondByPairing = FALSE;
  pCb->initiatingSec = FALSE;

  /* if master initiates security on connection open */
  appMasterInitiateSec((dmConnId_t) pMsg->hdr.param, pAppSecCfg->initiateSec, pCb);
}

/*************************************************************************************************/
/*!
 *  \brief  Perform security procedures on connection close.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *  \param  pCb     Connection control block.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appMasterSecConnClose(dmEvt_t *pMsg, appConnCb_t *pCb)
{
  /* if a device record was created check if it is valid */
  if (pCb->dbHdl != APP_DB_HDL_NONE)
  {
    AppDbCheckValidRecord(pCb->dbHdl);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Handle a slave security request.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *  \param  pCb     Connection control block.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appMasterSecSlaveReq(dmEvt_t *pMsg, appConnCb_t *pCb)
{
  if (DmConnSecLevel((dmConnId_t) pMsg->hdr.param) == DM_SEC_LEVEL_NONE)
  {
    /* if master is not initiating security */
    if (!pAppSecCfg->initiateSec && !pCb->initiatingSec)
    {
      appMasterInitiateSec((dmConnId_t) pMsg->hdr.param, TRUE, pCb);
    }
  }
  else
  {
    /* attempt to refresh keys */
    appMasterInitiateSec((dmConnId_t) pMsg->hdr.param, FALSE, pCb);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Handle set address resolution enable indication.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appPrivSetAddrResEnableInd(dmEvt_t *pMsg)
{
  if (pMsg->hdr.status == HCI_SUCCESS)
  {
    SvcCoreGapCentAddrResUpdate(DmLlPrivEnabled());
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Handle add device to resolving list indication.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *  \param  pCb     Connection control block.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appPrivAddDevToResListInd(dmEvt_t *pMsg, appConnCb_t *pCb)
{
  if ((pMsg->hdr.status == HCI_SUCCESS) && pCb && (pCb->dbHdl != APP_DB_HDL_NONE))
  {
    /* peer device's been added to resolving list */
    AppDbSetPeerAddedToRl(pCb->dbHdl, TRUE);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Handle remove device from resolving list indication.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *  \param  pCb     Connection control block.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appPrivRemDevFromResListInd(dmEvt_t *pMsg, appConnCb_t *pCb)
{
  if ((pMsg->hdr.status == HCI_SUCCESS) && (pCb->dbHdl != APP_DB_HDL_NONE))
  {
    /* peer device's been removed from resolving list */
    AppDbSetPeerAddedToRl(pCb->dbHdl, FALSE);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Store security key.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *  \param  pCb     Connection control block.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appMasterSecStoreKey(dmEvt_t *pMsg, appConnCb_t *pCb)
{
  if (pCb->bondByPairing && pCb->dbHdl != APP_DB_HDL_NONE)
  {
    /* key was received */
    pCb->rcvdKeys |= pMsg->keyInd.type;

    /* store key in record */
    AppDbSetKey(pCb->dbHdl, &pMsg->keyInd);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Handle pairing complete.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *  \param  pCb     Connection control block.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appMasterSecPairCmpl(dmEvt_t *pMsg, appConnCb_t *pCb)
{
  /* if bonding */
  if (pMsg->pairCmpl.auth & DM_AUTH_BOND_FLAG)
  {
    /* set bonded state */
    pCb->bonded = TRUE;

    /* validate record and received keys */
    if (pCb->dbHdl != APP_DB_HDL_NONE)
    {
      AppDbValidateRecord(pCb->dbHdl, pCb->rcvdKeys);
    }

    /* if bonded, add device to resolving list */
    if (pCb->dbHdl != APP_DB_HDL_NONE)
    {
      AppAddDevToResList(pMsg, pCb->connId);
    }
  }

  pCb->initiatingSec = FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Handle pairing failed
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *  \param  pCb     Connection control block.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appMasterSecPairFailed(dmEvt_t *pMsg, appConnCb_t *pCb)
{
  pCb->initiatingSec = FALSE;
  // disconnect the connection when paired failed to avoid spoofing reported by https://nvd.nist.gov/vuln/detail/CVE-2020-9770
  AppConnClose(pMsg->hdr.param);
  return;
}

/*************************************************************************************************/
/*!
 *  \brief  Handle encryption indication
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *  \param  pCb     Connection control block.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appMasterSecEncryptInd(dmEvt_t *pMsg, appConnCb_t *pCb)
{
  /* check if bonding state should be set */
  if (pCb->bondByLtk && pMsg->encryptInd.usingLtk)
  {
    pCb->bonded = TRUE;
    pCb->bondByLtk = FALSE;
    pCb->initiatingSec = FALSE;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Process app framework messages for a master.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void appMasterProcMsg(wsfMsgHdr_t *pMsg)
{
  switch(pMsg->event)
  {
    case APP_CONN_UPDATE_TIMEOUT_IND:
      break;

    default:
      break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Process a received privacy resolved address indication.
 *
 *  \param  pMsg    Pointer to DM message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appMasterResolvedAddrInd(dmEvt_t *pMsg)
{
  appDevInfo_t *pDev;
  dmSecKey_t *pPeerKey;

  /* if address resolution is not in progress */
  if (!appMasterCb.inProgress)
  {
    return;
  }

  /* get device record */
  pDev = &appMasterCb.scanResults[appMasterCb.idx];

  /* if RPA resolved */
  if (pMsg->hdr.status == HCI_SUCCESS)
  {
    /* if resolved advertising was directed with an RPA initiator address */
    if ((pMsg->hdr.param == APP_RESOLVE_ADV_RPA) && DM_RAND_ADDR_RPA(pDev->directAddr, pDev->directAddrType))
    {
      /* resolve initiator's RPA to see if directed advertisement was addressed to us */
      DmPrivResolveAddr(pDev->directAddr, DmSecGetLocalIrk(), APP_RESOLVE_DIRECT_RPA);

      /* not done yet */
      return;
    }

    if(DmScanModeLeg())
    {
        /* stop scanning */
        AppScanStop();

        /* connect to peer device using its advertising address */
        AppConnOpen(pDev->addrType, pDev->addr, appMasterCb.dbHdl);
    }
    else
    {
        AppExtScanStop();
        AppExtConnOpen(pDev->secPhy, pDev->addrType, pDev->addr, appMasterCb.dbHdl);
    }
  }
  /* if RPA did not resolve and there're more bonded records to go through */
  else if ((pMsg->hdr.status == HCI_ERR_AUTH_FAILURE) && (appMasterCb.dbHdl != APP_DB_HDL_NONE))
  {
    /* get the next database record */
    appMasterCb.dbHdl = AppDbGetNextRecord(appMasterCb.dbHdl);

    /* if there's another bond record */
    if ((appMasterCb.dbHdl != APP_DB_HDL_NONE) &&
        ((pPeerKey = AppDbGetKey(appMasterCb.dbHdl, DM_KEY_IRK, NULL)) != NULL))
    {
      /* resolve RPA using the next stored IRK */
      DmPrivResolveAddr(pDev->addr, pPeerKey->irk.key, APP_RESOLVE_ADV_RPA);

      /* not done yet */
      return;
    }
  }

  /* done with this address resolution */
  appMasterCb.inProgress = FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Handle a DM_REM_CONN_PARAM_REQ_IND event.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *  \param  pCb     Connection control block.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appMasterRemoteConnParamReq(dmEvt_t *pMsg, appConnCb_t *pCb)
{
  /* if configured to accept the remote connection parameter request */
  if (pAppMasterReqActCfg->remConnParamReqAct == APP_ACT_ACCEPT)
  {
    hciConnSpec_t connSpec;

    connSpec.connIntervalMin = pMsg->remConnParamReq.intervalMin;
    connSpec.connIntervalMax = pMsg->remConnParamReq.intervalMax;
    connSpec.connLatency = pMsg->remConnParamReq.latency;
    connSpec.supTimeout = pMsg->remConnParamReq.timeout;
    connSpec.minCeLen = connSpec.maxCeLen = 0;

    /* accept the remote device's request to change connection parameters */
    DmRemoteConnParamReqReply(pCb->connId, &connSpec);
  }
  /* if configured to reject the remote connection parameter request */
  else if (pAppMasterReqActCfg->remConnParamReqAct == APP_ACT_REJECT)
  {
    /* reject the remote device's request to change connection parameters */
    DmRemoteConnParamReqNegReply(pCb->connId, HCI_ERR_UNSUP_FEAT);
  }
  /* else - app will handle the remote connection parameter request */
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize app framework master.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppMasterInit(void)
{
  appMasterCb.inProgress = FALSE;

  /* initialize scan mode */
  appMasterCb.scanMode = APP_SCAN_MODE_NONE;

  /* set up callback from main */
  appCb.masterCback = appMasterProcMsg;
}

/*************************************************************************************************/
/*!
 *  \brief  Process connection-related DM messages for a master.  This function should be called
 *          from the application's event handler.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppMasterProcDmMsg(dmEvt_t *pMsg)
{
  appConnCb_t *pCb = NULL;

  /* look up app connection control block from DM connection ID */
  if (pMsg->hdr.event == DM_CONN_OPEN_IND  ||
      pMsg->hdr.event == DM_CONN_CLOSE_IND ||
      pMsg->hdr.event == DM_REM_CONN_PARAM_REQ_IND)
  {
    pCb = &appConnCb[pMsg->hdr.param - 1];
  }

  switch(pMsg->hdr.event)
  {
    case DM_RESET_CMPL_IND:
      /* reset scan mode */
      appMasterCb.scanMode = APP_SCAN_MODE_NONE;
      break;

    case DM_EXT_SCAN_START_IND:
    case DM_SCAN_START_IND:
      appMasterScanStart(pMsg);
      break;

    case DM_EXT_SCAN_STOP_IND:
    case DM_SCAN_STOP_IND:
      appMasterScanStop(pMsg);
      break;

    case DM_EXT_SCAN_REPORT_IND:
    case DM_SCAN_REPORT_IND:
      appMasterScanReport(pMsg);
      break;

    case DM_CONN_OPEN_IND:
      appMasterConnOpen(pMsg, pCb);
      break;

    case DM_CONN_CLOSE_IND:
      appMasterConnClose(pMsg, pCb);
      break;

    case DM_PRIV_RESOLVED_ADDR_IND:
      appMasterResolvedAddrInd(pMsg);
      break;

    case DM_REM_CONN_PARAM_REQ_IND:
      appMasterRemoteConnParamReq(pMsg, pCb);
      break;

    default:
      break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Process security-related DM messages for a master.  This function should be called
 *          from the application's event handler.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppMasterSecProcDmMsg(dmEvt_t *pMsg)
{
  appConnCb_t *pCb;

  if (pMsg->hdr.param != DM_CONN_ID_NONE)
  {
    /* look up app connection control block from DM connection ID */
    pCb = &appConnCb[pMsg->hdr.param - 1];
  }
  else
  {
    pCb = NULL;
  }

  switch(pMsg->hdr.event)
  {
    case DM_CONN_OPEN_IND:
      appMasterSecConnOpen(pMsg, pCb);
      break;

    case DM_CONN_CLOSE_IND:
      appMasterSecConnClose(pMsg, pCb);
      break;

    case DM_SEC_PAIR_CMPL_IND:
      appMasterSecPairCmpl(pMsg, pCb);
      break;

    case DM_SEC_PAIR_FAIL_IND:
      appMasterSecPairFailed(pMsg, pCb);
      break;

    case DM_SEC_ENCRYPT_IND:
      appMasterSecEncryptInd(pMsg, pCb);
      break;

    case DM_SEC_ENCRYPT_FAIL_IND:
      // disconnect the connection when paired failed to avoid spoofing reported by https://nvd.nist.gov/vuln/detail/CVE-2020-9770
      AppConnClose(pMsg->hdr.param);
      break;

    case DM_SEC_KEY_IND:
      appMasterSecStoreKey(pMsg, pCb);
      break;

    case DM_SEC_SLAVE_REQ_IND:
      appMasterSecSlaveReq(pMsg, pCb);
      break;

    case DM_PRIV_SET_ADDR_RES_ENABLE_IND:
      appPrivSetAddrResEnableInd(pMsg);
      break;

    case DM_PRIV_ADD_DEV_TO_RES_LIST_IND:
      appPrivAddDevToResListInd(pMsg, pCb);
      break;

    case DM_PRIV_REM_DEV_FROM_RES_LIST_IND:
      appPrivRemDevFromResListInd(pMsg, pCb);
      break;

    case DM_HW_ERROR_IND:
      HciDrvRadioBoot(0);
      DmDevReset();
      break;

    default:
      break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Get a stored scan result from the scan result list.
 *
 *  \param  idx     Index of result in scan result list.
 *
 *  \return Pointer to scan result device info or NULL if index contains no result.
 */
/*************************************************************************************************/
appDevInfo_t *AppScanGetResult(uint8_t idx)
{
  if (idx < APP_SCAN_RESULT_MAX && appMasterCb.scanResults[idx].addrType != APP_ADDR_NONE)
  {
    return &appMasterCb.scanResults[idx];
  }
  else
  {
    return NULL;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Get the number of stored scan results.
 *
 *  \return Number of stored scan results.
 */
/*************************************************************************************************/
uint8_t AppScanGetNumResults(void)
{
  return appMasterCb.numScanResults;
}

/*************************************************************************************************/
/*!
 *  \brief  Open a connection to a peer device with the given initiator PHYs, and address.
 *
 *  \param  initPhys  Initiator PHYs.
 *  \param  addrType  Address type.
 *  \param  pAddr     Peer device address.
 *  \param  dbHdl     Device database handle.
 *
 *  \return Connection identifier.
 */
/*************************************************************************************************/
dmConnId_t appConnOpen(uint8_t initPhys, uint8_t addrType, uint8_t *pAddr, appDbHdl_t dbHdl)
{
  dmConnId_t  connId;
  appConnCb_t *pCb;

  /* open connection */
  connId = DmConnOpen(DM_CLIENT_ID_APP, initPhys, addrType, pAddr);

  if (connId != DM_CONN_ID_NONE)
  {
    /* set up conn. control block */
    pCb = &appConnCb[connId - 1];

    pCb->connId = connId;

    /* if database record handle is in use */
    if ((dbHdl != APP_DB_HDL_NONE) && AppDbRecordInUse(dbHdl))
    {
      pCb->dbHdl = dbHdl;
    }
    else
    {
      pCb->dbHdl = AppDbFindByAddr(addrType, pAddr);
    }
  }

  return connId;
}

/*************************************************************************************************/
/*!
 *  \brief  Initiate security as a master device.  If there is a stored encryption key
 *          for the peer device this function will initiate encryption, otherwise it will
 *          initiate pairing.
 *
 *  \param  connId    Connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppMasterSecurityReq(dmConnId_t connId)
{
  appConnCb_t *pCb;

  WSF_ASSERT((connId > 0) && (connId <= DM_CONN_MAX));

  /* look up app connection control block from DM connection ID */
  pCb = &appConnCb[connId - 1];

  /* if master is not initiating security and not already secure */
  if (!pAppSecCfg->initiateSec && !pCb->initiatingSec &&
      (DmConnSecLevel(connId) == DM_SEC_LEVEL_NONE))
  {
    appMasterInitiateSec(connId, TRUE, pCb);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Resolve the advertiser's RPA (AdvA) or the initiator's RPA (InitA) of a directed
 *          advertisement report. If the addresses resolved then a connection will be initiated.
 *
 *  \param  pMsg         Pointer to DM callback event message.
 *  \param  dbHdl        Database record handle.
 *  \param  resolveType  Which address in advertising report to be resolved.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppMasterResolveAddr(dmEvt_t *pMsg, appDbHdl_t dbHdl, uint8_t resolveType)
{
  uint8_t    idx;

  /* if address resolution's in progress or scan record is not found */
  if ((appMasterCb.inProgress) || ((idx = appScanResultFind(pMsg)) >= APP_SCAN_RESULT_MAX))
  {
    return;
  }

  /* if asked to resolve direct address */
  if (resolveType == APP_RESOLVE_DIRECT_RPA)
  {
    if(DmScanModeLeg())
    {
        /* resolve initiator's RPA to see if the directed advertisement is addressed to us */
        DmPrivResolveAddr(pMsg->scanReport.directAddr, DmSecGetLocalIrk(), APP_RESOLVE_DIRECT_RPA);
    }
    else
    {
        /* resolve initiator's RPA to see if the directed advertisement is addressed to us */
        DmPrivResolveAddr(pMsg->extScanReport.directAddr, DmSecGetLocalIrk(), APP_RESOLVE_DIRECT_RPA);
    }

    /* store scan record index and database record handle for later */
    appMasterCb.idx = idx;
    appMasterCb.dbHdl = dbHdl;
    appMasterCb.inProgress = TRUE;
  }
  /* if asked to resolve advertiser's address */
  else if (resolveType == APP_RESOLVE_ADV_RPA)
  {
    dmSecKey_t *pPeerKey;
    appDbHdl_t hdl = AppDbGetNextRecord(APP_DB_HDL_NONE);

    /* if we have any bond records */
    if ((hdl != APP_DB_HDL_NONE) && ((pPeerKey = AppDbGetKey(hdl, DM_KEY_IRK, NULL)) != NULL))
    {
        if(DmScanModeLeg())
        {
          /* resolve advertiser's RPA to see if we already have a bond with this device */
          DmPrivResolveAddr(pMsg->scanReport.addr, pPeerKey->irk.key, APP_RESOLVE_ADV_RPA);
        }
        else
        {
          /* resolve advertiser's RPA to see if we already have a bond with this device */
          DmPrivResolveAddr(pMsg->extScanReport.addr, pPeerKey->irk.key, APP_RESOLVE_ADV_RPA);
        }
        

      /* store scan record index and database record handle for later */
      appMasterCb.idx = idx;
      appMasterCb.dbHdl = hdl;
      appMasterCb.inProgress = TRUE;
    }
  }
}
