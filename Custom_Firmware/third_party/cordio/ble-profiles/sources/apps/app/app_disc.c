/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Application framework service discovery and configuration.
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
#include "wsf_buf.h"
#include "wsf_trace.h"
#include "util/bstream.h"
#include "dm_api.h"
#include "att_api.h"
#include "svc_ch.h"
#include "app_api.h"
#include "app_main.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! "In progress" values */
#define APP_DISC_IDLE                 0
#define APP_DISC_SVC_DISC_IN_PROGRESS 1
#define APP_DISC_CFG_IN_PROGRESS      2
#define APP_DISC_READ_DBH_IN_PROGRESS 3

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! \brief Application Discovery controler block */
typedef struct
{
  attcDiscCb_t    *pDiscCb;                 /*! ATT discovery control block */
  uint16_t        *pHdlList;                /*! Handle list */
  uint8_t         connCfgStatus;            /*! Connection setup configuration status */
  uint8_t         cmplStatus;               /*! Discovery or configuration complete status */
  uint8_t         hdlListLen;               /*! Handle list length */
  uint8_t         inProgress;               /*! Discovery or configuration in progress */
  bool_t          alreadySecure;            /*! TRUE if connection was already secure */
  bool_t          secRequired;              /*! TRUE if security is required for configuration */
  bool_t          scPending;                /*! TRUE if service changed from peer is pending */
} appDiscCb_t;

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! Discovery connection control blocks */
static appDiscCb_t appDiscCb[DM_CONN_MAX];

/*! Discovery callback */
static appDiscCback_t appDiscCback;

/*************************************************************************************************/
/*!
 *  \brief  Start discovery or configuration
 *
 *  \param  connId  Connection ID.
 *  \param  status  Status of discovery process.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appDiscCfgStart(dmConnId_t connId, uint8_t status)
{
  appDiscCb_t *pAppDiscCb = &appDiscCb[connId - 1];

  /* if configuration not complete */
  if (status < APP_DISC_CFG_CMPL)
  {
    /* notify application to start configuration */
    (*appDiscCback)(connId, APP_DISC_CFG_START);
  }
  /* else if configuration complete start connection setup configuration */
  else if (status == APP_DISC_CFG_CMPL && pAppDiscCb->connCfgStatus == APP_DISC_INIT)
  {
    (*appDiscCback)(connId, APP_DISC_CFG_CONN_START);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Start discovery or configuration
 *
 *  \param  dmConnId_t  Connection ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void appDiscStart(dmConnId_t connId)
{
  appDbHdl_t  hdl;
  uint8_t     status;
  appDiscCb_t *pAppDiscCb = &appDiscCb[connId - 1];

  if (pAppDiscCb->inProgress == APP_DISC_IDLE)
  {
    /* get discovery status */
    if ((hdl = AppDbGetHdl(connId)) != APP_DB_HDL_NONE)
    {
      status = AppDbGetDiscStatus(hdl);
    }
    else
    {
      status = pAppDiscCb->cmplStatus;
    }

    /* if discovery not complete */
    if (status < APP_DISC_CMPL)
    {
      /* Read database hash first if not bonded or if secure but without bond. */
      if ((!pAppDiscCb->alreadySecure) || (pAppDiscCb->alreadySecure && !AppCheckBonded(connId)))
      {
        /* notify application to start discovery */
        (*appDiscCback)(connId, APP_DISC_READ_DATABASE_HASH);
      }
      else
      {
        /* notify application to start discovery */
        (*appDiscCback)(connId, APP_DISC_START);
      }
    }
    /* else if discovery was completed successfully */
    else if (status != APP_DISC_FAILED)
    {
      /* get stored handle list if present */
      if (hdl != APP_DB_HDL_NONE && pAppDiscCb->pHdlList != NULL)
      {
        /* Read hash before using handles */
        if (AppDbIsCacheCheckedByHash(hdl))
        {
          /* Read the database hash. */
          AppDiscReadDatabaseHash(connId);
          return;
        }
        else
        {
          memcpy(pAppDiscCb->pHdlList, AppDbGetHdlList(hdl), (pAppDiscCb->hdlListLen * sizeof(uint16_t)));
        }
      }

      appDiscCfgStart(connId, status);
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Reset service discovery.
 *
 *  \param  connId    DM Connection ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void appDiscRestartDiscovery(dmConnId_t connId)
{
  appDiscCb_t *pAppDiscCb = &appDiscCb[connId - 1];
  appDbHdl_t  dbHdl;

  /* otherwise initialize discovery and configuration status */
  pAppDiscCb->connCfgStatus = APP_DISC_INIT;
  pAppDiscCb->cmplStatus = APP_DISC_INIT;
  pAppDiscCb->secRequired = FALSE;
  pAppDiscCb->scPending = FALSE;

  /* initialize handle list */
  if (pAppDiscCb->pHdlList != NULL)
  {
    memset(pAppDiscCb->pHdlList, 0, (pAppDiscCb->hdlListLen * sizeof(uint16_t)));

    /* clear stored discovery status and handle list */
    if ((dbHdl = AppDbGetHdl(connId)) != APP_DB_HDL_NONE)
    {
      AppDbSetDiscStatus(dbHdl, APP_DISC_INIT);
      AppDbSetHdlList(dbHdl, pAppDiscCb->pHdlList);
    }
  }

  /* if configuration in progress */
  if (pAppDiscCb->inProgress == APP_DISC_CFG_IN_PROGRESS)
  {
    /* set pending status to set up abort of configuration */
    pAppDiscCb->scPending = TRUE;
  }
  /* else no procedure in progress */
  else
  {
    /* if not waiting for security or connection is already secure, then
    * initiate discovery now; otherwise discovery will be initiated after
    * security is done
    */
    if (!pAppDiscCfg->waitForSec || pAppDiscCb->alreadySecure)
    {
      appDiscStart(connId);
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Handle a DM_CONN_OPEN_IND event.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appDiscConnOpen(dmEvt_t *pMsg)
{
  appDiscCb_t *pAppDiscCb = &appDiscCb[(dmConnId_t) pMsg->hdr.param - 1];

  pAppDiscCb->alreadySecure = FALSE;
  pAppDiscCb->connCfgStatus = APP_DISC_INIT;
  pAppDiscCb->cmplStatus = APP_DISC_INIT;
  pAppDiscCb->secRequired = FALSE;
  pAppDiscCb->scPending = FALSE;

  /* tell app to set up handle list */
  (*appDiscCback)((dmConnId_t) pMsg->hdr.param, APP_DISC_INIT);

  /* initialize handle list */
  if (pAppDiscCb->pHdlList != NULL)
  {
    memset(pAppDiscCb->pHdlList, 0, (pAppDiscCb->hdlListLen * sizeof(uint16_t)));
  }

  /* if not waiting for security start discovery/configuration */
  if (!pAppDiscCfg->waitForSec)
  {
    appDiscStart((dmConnId_t) pMsg->hdr.param);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Handle a DM_CONN_CLOSE_IND event.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appDiscConnClose(dmEvt_t *pMsg)
{
  appDiscCb_t *pAppDiscCb = &appDiscCb[(dmConnId_t) pMsg->hdr.param - 1];

  pAppDiscCb->inProgress = APP_DISC_IDLE;

  appDbHdl_t  hdl;
  
  if ((hdl = AppDbGetHdl((dmConnId_t) pMsg->hdr.param)) != APP_DB_HDL_NONE)
  {
    // reset discovery status
    AppDbSetDiscStatus(hdl, APP_DISC_INIT);
  }


  if (pAppDiscCb->pDiscCb != NULL)
  {
    WsfBufFree(pAppDiscCb->pDiscCb);
    pAppDiscCb->pDiscCb = NULL;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Handle pairing complete.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appDiscPairCmpl(dmEvt_t *pMsg)
{
  appDiscCb_t *pAppDiscCb = &appDiscCb[(dmConnId_t) pMsg->hdr.param - 1];
  appDbHdl_t hdl;

  /* procedures triggered by security are only executed once */
  if (pAppDiscCb->alreadySecure)
  {
    return;
  }

  /* if bonded, disable hash check on cache if not already disabled */
  if (((hdl = AppDbGetHdl((dmConnId_t)pMsg->hdr.param)) != APP_DB_HDL_NONE) &&
      AppCheckBonded((dmConnId_t) pMsg->hdr.param) &&
      AppDbIsCacheCheckedByHash(hdl))
  {
    AppDbSetCacheByHash(appConnCb[pMsg->hdr.param - 1].dbHdl, FALSE);
  }


  /* if we are now bonded and discovery/configuration was performed before bonding */
  if (AppCheckBonded((dmConnId_t) pMsg->hdr.param) && (pAppDiscCb->cmplStatus != APP_DISC_INIT))
  {
    if (hdl != APP_DB_HDL_NONE)
    {
      /* store discovery status */
      AppDbSetDiscStatus(hdl, pAppDiscCb->cmplStatus);

      /* store handle list */
      if (pAppDiscCb->cmplStatus == APP_DISC_CMPL || pAppDiscCb->cmplStatus == APP_DISC_CFG_CMPL)
      {
        if (pAppDiscCb->pHdlList != NULL)
        {
          AppDbSetHdlList(hdl, pAppDiscCb->pHdlList);
        }
      }
    }

    /* if configuration was waiting for security */
    if (pAppDiscCb->secRequired)
    {
      pAppDiscCb->secRequired = FALSE;

      /* resume configuration */
      if (pAppDiscCb->pDiscCb != NULL)
      {
        AttcDiscConfigResume((dmConnId_t) pMsg->hdr.param, pAppDiscCb->pDiscCb);
      }
    }
  }
  else {

  /* if waiting for security start discovery now that connection is secure */
  if (pAppDiscCfg->waitForSec)
  {
    appDiscStart((dmConnId_t) pMsg->hdr.param);
  }
  }  


  pAppDiscCb->alreadySecure = TRUE;
}

/*************************************************************************************************/
/*!
 *  \brief  Handle encryption indication
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appDiscEncryptInd(dmEvt_t *pMsg)
{
  appDiscCb_t *pAppDiscCb = &appDiscCb[(dmConnId_t) pMsg->hdr.param - 1];

  /* if encrypted with ltk */
  if (pMsg->encryptInd.usingLtk)
  {
    /* procedures triggered by security are only executed once */
    if (pAppDiscCb->alreadySecure)
    {
      return;
    }

    /* if we waiting for security start discovery now that connection is secure */
    if (pAppDiscCfg->waitForSec)
    {
      appDiscStart((dmConnId_t) pMsg->hdr.param);
    }
    /* else if configuration was waiting for security */
    else if (pAppDiscCb->secRequired)
    {
      pAppDiscCb->secRequired = FALSE;

      /* resume configuration */
      if (pAppDiscCb->pDiscCb != NULL)
      {
        AttcDiscConfigResume((dmConnId_t) pMsg->hdr.param, pAppDiscCb->pDiscCb);
      }
    }

    pAppDiscCb->alreadySecure = TRUE;
  }
}

/*************************************************************************************************/
/*!
*  \brief  Handle pairing failure
*
*  \param  pMsg    Pointer to DM callback event message.
*
*  \return None.
*/
/*************************************************************************************************/
static void appDiscPairFail(dmEvt_t *pMsg)
{
  appDiscCb_t *pAppDiscCb = &appDiscCb[(dmConnId_t)pMsg->hdr.param - 1];

  /* Procedures triggered by security are only executed once. */
  if (pAppDiscCb->alreadySecure)
  {
    return;
  }

  /* Fall back to relying on database hash to verify handles if configured to do so. */
  if (pAppDiscCfg->readDbHash)
  {
    /* Read the database hash instead of re-performing service discovery. */
    AppDiscReadDatabaseHash((dmConnId_t) pMsg->hdr.param);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Process discovery-related DM messages.  This function should be called
 *          from the application's event handler.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDiscProcDmMsg(dmEvt_t *pMsg)
{
  switch(pMsg->hdr.event)
  {
    case DM_CONN_OPEN_IND:
      appDiscConnOpen(pMsg);
      break;

    case DM_CONN_CLOSE_IND:
      appDiscConnClose(pMsg);
      break;

    case DM_SEC_PAIR_CMPL_IND:
      appDiscPairCmpl(pMsg);
      break;

    case DM_SEC_PAIR_FAIL_IND:
      appDiscPairFail(pMsg);
      break;

    case DM_SEC_ENCRYPT_IND:
      appDiscEncryptInd(pMsg);
      break;

    default:
      break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Process discovery-related ATT messages.  This function should be called
 *          from the application's event handler.
 *
 *  \param  pMsg    Pointer to ATT callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDiscProcAttMsg(attEvt_t *pMsg)
{
  appDiscCb_t *pAppDiscCb = &appDiscCb[(dmConnId_t)pMsg->hdr.param - 1];
  uint8_t status;

  /* Check status */
  if (pMsg->hdr.status == ATT_ERR_DATABASE_OUT_OF_SYNC)
  {
    /* Restart discovery as cached handle list is out of sync with server's database. */
    appDiscRestartDiscovery((dmConnId_t)pMsg->hdr.param);
  }

  if (pAppDiscCb->inProgress == APP_DISC_READ_DBH_IN_PROGRESS)
  {
    if (pMsg->hdr.event == ATTC_READ_BY_TYPE_RSP)
    {
      dmConnId_t connId = (dmConnId_t)pMsg->hdr.param;

      if (pMsg->hdr.status != ATT_SUCCESS)
      {
        /* No Database hash found on peer, notify application to start discovery */
        (*appDiscCback)(connId, APP_DISC_START);
      }
      else
      {
        appDbHdl_t hdl;

        pAppDiscCb->inProgress = APP_DISC_IDLE;

        /* If there is no existing record, create one now.  The cached handles will be
         * validated across connections by the peer's database hash.
         */
        if ((hdl = AppDbGetHdl(connId)) == APP_DB_HDL_NONE)
        {
          hdl = appConnCb[connId - 1].dbHdl = AppDbNewRecord(DmConnPeerAddrType(connId),
                                                             DmConnPeerAddr(connId), 
                                                             (DmConnRole(connId)==DM_ROLE_MASTER)?TRUE:FALSE);
        }

        /* Compare with existing database hash for this server.
         * If they do not match, perform service discovery.
         */
        if (memcmp(AppDbGetPeerDbHash(hdl), pMsg->pValue + 3, ATT_DATABASE_HASH_LEN))
        {
          /* The new hash is different.  Store it. */
          AppDbSetPeerDbHash(hdl, pMsg->pValue + 3);

          /* Note: it is possible this record was created without a pairing or after
           * a pairing failed, validate record now so that it can be stored persistently.
           */
          /* AppDbValidateRecord(hdl, 0); */

          /* The validity of the cached handles is checked by the hash and not necessarily by
           * service changed indications.
           */
          AppDbSetCacheByHash(hdl, TRUE);

          /* notify application to start discovery */
          (*appDiscCback)(connId, APP_DISC_START);
        }
        else
        {
          /* Hash is the same, cached handles may be reused. */
          memcpy(pAppDiscCb->pHdlList, AppDbGetHdlList(hdl),
                 (pAppDiscCb->hdlListLen * sizeof(uint16_t)));

          /* get discovery status */
          status = AppDbGetDiscStatus(hdl);

          appDiscCfgStart(connId, status);
        }
      }
    }
  }
  else if (pAppDiscCb->inProgress == APP_DISC_SVC_DISC_IN_PROGRESS)
  {
    /* service discovery */
    if (pMsg->hdr.event == ATTC_FIND_BY_TYPE_VALUE_RSP)
    {
      /* continue with service discovery */
      status = AttcDiscServiceCmpl(pAppDiscCb->pDiscCb, pMsg);

      APP_TRACE_INFO1("AttcDiscServiceCmpl status 0x%02x", status);

      /* if discovery complete  and successful */
      if (status == ATT_SUCCESS)
      {
        /* proceed with characteristic discovery */
        AttcDiscCharStart((dmConnId_t) pMsg->hdr.param, pAppDiscCb->pDiscCb);
      }
      /* else if failed */
      else if (status != ATT_CONTINUING)
      {
        /* notify application of discovery failure */
        (*appDiscCback)((dmConnId_t) pMsg->hdr.param, APP_DISC_FAILED);
      }
    }
    /* characteristic discovery */
    else if (pMsg->hdr.event == ATTC_READ_BY_TYPE_RSP ||
             pMsg->hdr.event == ATTC_FIND_INFO_RSP)
    {
      /* continue with characteristic discovery */
      status = AttcDiscCharCmpl(pAppDiscCb->pDiscCb, pMsg);

      APP_TRACE_INFO1("AttcDiscCharCmpl status 0x%02x", status);

      /* if discovery complete and successful */
      if (status == ATT_SUCCESS)
      {
        /* notify application of discovery success */
        (*appDiscCback)((dmConnId_t) pMsg->hdr.param, APP_DISC_CMPL);
      }
      /* else if failed */
      else if (status != ATT_CONTINUING)
      {
        /* notify application of discovery failure */
        (*appDiscCback)((dmConnId_t) pMsg->hdr.param, APP_DISC_FAILED);
      }
    }
  }
  /* characteristic configuration */
  else if ((pAppDiscCb->inProgress == APP_DISC_CFG_IN_PROGRESS) &&
           (pMsg->hdr.event == ATTC_READ_RSP || pMsg->hdr.event == ATTC_WRITE_RSP))
  {
    /* if service changed is pending */
    if (pAppDiscCb->scPending)
    {
      /* clear pending flag */
      pAppDiscCb->scPending = FALSE;

      /* start discovery */
      pAppDiscCb->inProgress = APP_DISC_IDLE;
      appDiscStart((dmConnId_t) pMsg->hdr.param);
    }
    /* else if security failure */
    else if ((pMsg->hdr.status == ATT_ERR_AUTH || pMsg->hdr.status == ATT_ERR_ENC) &&
             (DmConnSecLevel((dmConnId_t) pMsg->hdr.param) == DM_SEC_LEVEL_NONE))
    {
      /* tell application to request security */
      pAppDiscCb->secRequired = TRUE;
      (*appDiscCback)((dmConnId_t) pMsg->hdr.param, APP_DISC_SEC_REQUIRED);
    }
    else
    {
      status = AttcDiscConfigCmpl((dmConnId_t) pMsg->hdr.param, pAppDiscCb->pDiscCb);

      APP_TRACE_INFO1("AttcDiscConfigCmpl status 0x%02x", status);

      /* if configuration complete */
      if (status != ATT_CONTINUING)
      {
        /* notify application of config success */
        (*appDiscCback)((dmConnId_t) pMsg->hdr.param, APP_DISC_CFG_CMPL);
      }
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize app framework discovery.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDiscInit(void)
{
  uint8_t i;

  for (i = 0; i < DM_CONN_MAX; i++)
  {
    appDiscCb[i].inProgress = APP_DISC_IDLE;
    appDiscCb[i].pDiscCb = NULL;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Register a callback function to service discovery status.
 *
 *  \param  cback   Application service discovery callback function.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDiscRegister(appDiscCback_t cback)
{
  appDiscCback = cback;
}

/*************************************************************************************************/
/*!
 *  \brief  Set the discovery cached handle list for a given connection.
 *
 *  \param  connId    Connection identifier.
 *  \param  listLen   Length of characteristic and handle lists.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDiscSetHdlList(dmConnId_t connId, uint8_t hdlListLen, uint16_t *pHdlList)
{
  appDiscCb_t *pAppDiscCb = &appDiscCb[connId - 1];

  pAppDiscCb->hdlListLen = hdlListLen;
  pAppDiscCb->pHdlList = pHdlList;
}

/*************************************************************************************************/
/*!
 *  \brief  Service discovery or configuration procedure complete.
 *
 *  \param  connId    Connection identifier.
 *  \param  status    Service or configuration status.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDiscComplete(dmConnId_t connId, uint8_t status)
{
  appDiscCb_t *pAppDiscCb = &appDiscCb[connId - 1];
  appDbHdl_t hdl;

  /* set connection as idle */
  DmConnSetIdle(connId, DM_IDLE_APP_DISC, DM_CONN_IDLE);

  /* store status if not doing connection setup configuration */
  if (!(status == APP_DISC_CFG_CMPL && pAppDiscCb->connCfgStatus == APP_DISC_CFG_CONN_START))
  {
    pAppDiscCb->cmplStatus = status;
  }

  /* initialize control block */
  pAppDiscCb->inProgress = APP_DISC_IDLE;
  if (pAppDiscCb->pDiscCb != NULL)
  {
    WsfBufFree(pAppDiscCb->pDiscCb);
    pAppDiscCb->pDiscCb = NULL;
  }

  if ((hdl = AppDbGetHdl(connId)) != APP_DB_HDL_NONE)
  {
    /* Don't store configuration complete if not bonded - it must be re-done on reconnection. */
    uint8_t discComplete = AppCheckBonded(connId) ? APP_DISC_CFG_CMPL : APP_DISC_CMPL;

    /* store discovery status if not doing connection setup configuration */
    if (!(status == APP_DISC_CFG_CMPL && pAppDiscCb->connCfgStatus == APP_DISC_CFG_CONN_START) && (status <= discComplete))
    {
      AppDbSetDiscStatus(hdl, status);
    }

    if (pAppDiscCb->pHdlList != NULL)
    {
      /* if discovery complete store handles */
      if (status == APP_DISC_CMPL)
      {
        AppDbSetHdlList(hdl, pAppDiscCb->pHdlList);
      }
    }
  }

  /* set connection setup configuration status as complete if either discovery-initiated
   * configuration is complete or connection setup configuration is complete
   */
  if (status == APP_DISC_CFG_CMPL)
  {
    pAppDiscCb->connCfgStatus = APP_DISC_CFG_CMPL;
  }

  APP_TRACE_INFO2("AppDiscComplete connId:%d status:0x%02x", connId, status);
}

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for a given service.
 *
 *  \param  connId    Connection identifier.
 *  \param  uuidLen   Length of UUID (2 or 16).
 *  \param  pUuid     Pointer to UUID data.
 *  \param  listLen   Length of characteristic and handle lists.
 *  \param  pCharList Characterisic list for discovery.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDiscFindService(dmConnId_t connId, uint8_t uuidLen, uint8_t *pUuid, uint8_t listLen,
                        attcDiscChar_t **pCharList, uint16_t *pHdlList)
{
  appDiscCb_t *pAppDiscCb = &appDiscCb[connId - 1];

  if (pAppDiscCb->pDiscCb == NULL)
  {
    pAppDiscCb->pDiscCb = WsfBufAlloc(sizeof(attcDiscCb_t));
  }

  if (pAppDiscCb->pDiscCb != NULL)
  {
    /* set connection as busy */
    DmConnSetIdle(connId, DM_IDLE_APP_DISC, DM_CONN_BUSY);

    pAppDiscCb->inProgress = APP_DISC_SVC_DISC_IN_PROGRESS;

    pAppDiscCb->pDiscCb->pCharList = pCharList;
    pAppDiscCb->pDiscCb->pHdlList = pHdlList;
    pAppDiscCb->pDiscCb->charListLen = listLen;
    AttcDiscService(connId, pAppDiscCb->pDiscCb, uuidLen, pUuid);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Configure characteristics for discovered services.
 *
 *  \param  connId      Connection identifier.
 *  \param  status      APP_DISC_CFG_START or APP_DISC_CFG_CONN_START.
 *  \param  cfgListLen  Length of characteristic configuration list.
 *  \param  pCfgList    Characteristic configuration list.
 *  \param  hdlListLen  Length of characteristic handle list.
 *  \param  pHdlList    Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDiscConfigure(dmConnId_t connId, uint8_t status, uint8_t cfgListLen,
                      attcDiscCfg_t *pCfgList, uint8_t hdlListLen, uint16_t *pHdlList)
{
  appDiscCb_t *pAppDiscCb = &appDiscCb[connId - 1];
  uint8_t ret;

  if (pAppDiscCb->pDiscCb == NULL)
  {
    pAppDiscCb->pDiscCb = WsfBufAlloc(sizeof(attcDiscCb_t));
  }

  if (pAppDiscCb->pDiscCb != NULL)
  {
    /* set connection as busy */
    DmConnSetIdle(connId, DM_IDLE_APP_DISC, DM_CONN_BUSY);

    pAppDiscCb->inProgress = APP_DISC_CFG_IN_PROGRESS;

    if (status == APP_DISC_CFG_CONN_START)
    {
      pAppDiscCb->connCfgStatus = APP_DISC_CFG_CONN_START;
    }

    /* start configuration */
    pAppDiscCb->pDiscCb->pCfgList = pCfgList;
    pAppDiscCb->pDiscCb->cfgListLen = cfgListLen;
    pAppDiscCb->pDiscCb->pHdlList = pHdlList;
    pAppDiscCb->pDiscCb->charListLen = hdlListLen;
    ret = AttcDiscConfigStart(connId, pAppDiscCb->pDiscCb);

    /* nothing to configure; configuration complete */
    if (ret == ATT_SUCCESS)
    {
      (*appDiscCback)(connId, APP_DISC_CFG_CMPL);
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Perform the GATT service changed procedure.  This function is called when an
 *          indication is received containing the GATT service changed characteristic.  This
 *          function may initialize the discovery state and initiate service discovery
 *          and configuration.
 *
 *  \param  pMsg    Pointer to ATT callback event message containing received indication.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDiscServiceChanged(attEvt_t *pMsg)
{
  appDiscCb_t *pAppDiscCb = &appDiscCb[pMsg->hdr.param - 1];
  uint16_t    startHdl;
  uint16_t    endHdl;
  uint8_t     *p;
  uint16_t    *pHdl;
  uint8_t     i;
  bool_t      foundHdl;

  /* verify characteristic length */
  if (pMsg->valueLen != CH_SC_LEN)
  {
    return;
  }

  /* parse and verify handles */
  p = pMsg->pValue;
  BSTREAM_TO_UINT16(startHdl, p);
  BSTREAM_TO_UINT16(endHdl, p);
  if (startHdl == 0 || endHdl < startHdl)
  {
    return;
  }

  /* if we don't have any stored handles within service changed handle range, ignore */
  foundHdl = FALSE;
  if (pAppDiscCb->pHdlList != NULL)
  {
    pHdl = pAppDiscCb->pHdlList;
    for (i = pAppDiscCb->hdlListLen; i > 0; i--, pHdl++)
    {
      if (*pHdl >= startHdl && *pHdl <= endHdl)
      {
        foundHdl = TRUE;
        break;
      }
    }
  }
  if (foundHdl == FALSE)
  {
    return;
  }

  /* if discovery procedure already in progress */
  if (pAppDiscCb->inProgress == APP_DISC_SVC_DISC_IN_PROGRESS)
  {
    /* ignore service changed */
    return;
  }

  /* Prepare to restart service discovery*/
  appDiscRestartDiscovery((dmConnId_t) pMsg->hdr.param);
}

/*************************************************************************************************/
/*!
 *  \brief  Read peer's database hash
 *
 *  \param  dmConnId_t  Connection ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDiscReadDatabaseHash(dmConnId_t connId)
{
  appDiscCb_t *pAppDiscCb = &appDiscCb[connId - 1];

  /* if discovery not in progress */
  if (pAppDiscCb->inProgress == APP_DISC_IDLE)
  {
    pAppDiscCb->inProgress = APP_DISC_READ_DBH_IN_PROGRESS;

    /* Read the database hash. */
    AttcReadByTypeReq(connId, ATT_HANDLE_START, ATT_HANDLE_MAX, ATT_16_UUID_LEN,
                      (uint8_t *) attGattDbhChUuid, FALSE);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Get the handle range of the latest service discovery operation.
 *
 *  May be called after receiving a \ref APP_DISC_CMPL event, but before calling AppDiscComplete().
 *
 *  \param  connId       connection identifier.
 *  \param  pStartHdl    output parameter for start handle.
 *  \param  pEndHdl      output parameter for end handle.
 *
 *  \return \ref TRUE if handles were set, \ref FALSE otherwise.
 */
/*************************************************************************************************/
bool_t AppDiscGetHandleRange(dmConnId_t connId, uint16_t *pStartHdl, uint16_t *pEndHdl)
{
  appDiscCb_t *pAppDiscCb = &appDiscCb[connId - 1];

  if (pAppDiscCb->pDiscCb != NULL)
  {
    *pStartHdl = pAppDiscCb->pDiscCb->svcStartHdl;
    *pEndHdl = pAppDiscCb->pDiscCb->svcEndHdl;

    return TRUE;
  }

  return FALSE;
}
