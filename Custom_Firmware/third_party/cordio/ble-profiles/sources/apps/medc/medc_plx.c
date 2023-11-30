/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Health/medical collector, Pulse Oximeter profile
 *
 *  Copyright (c) 2016-2018 Arm Ltd. All Rights Reserved.
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
#include "wsf_trace.h"
#include "wsf_assert.h"
#include "dm_api.h"
#include "att_api.h"
#include "app_cfg.h"
#include "app_api.h"
#include "app_db.h"
#include "app_ui.h"
#include "app_main.h"
#include "svc_ch.h"
#include "plxpc/plxpc_api.h"
#include "medc/medc_main.h"

/**************************************************************************************************
  ATT Client Discovery Data
**************************************************************************************************/

/* Start of cached health pulse oximeter service handles; begins after DIS */
#define MEDC_DISC_PLXS_START         (MEDC_DISC_DIS_START + DIS_HDL_LIST_LEN)

/* Total cached handle list length */
#define MEDC_DISC_HDL_LIST_LEN      (MEDC_DISC_PLXS_START + PLXPC_PLXS_HDL_LIST_LEN)

/*! Pointers into handle list for health pulse oximeter service handles */
static uint16_t *pMedcPlxsHdlList = &medcCb.hdlList[MEDC_DISC_PLXS_START];

/* sanity check:  make sure handle list length is <= app db handle list length */
WSF_CT_ASSERT(MEDC_DISC_HDL_LIST_LEN <= APP_DB_HDL_LIST_LEN);

/* Default MTU */
#define MEDC_PLX_DEFAULT_MTU        50

/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/

/*! ATT configurable parameters (increase MTU) */
static const attCfg_t medcPlxpAttCfg =
{
  15,                               /* ATT server service discovery connection idle timeout in seconds */
  MEDC_PLX_DEFAULT_MTU,             /* desired ATT MTU */
  ATT_MAX_TRANS_TIMEOUT,            /* transcation timeout in seconds */
  4                                 /* number of queued prepare writes supported by server */
};

/**************************************************************************************************
  ATT Client Configuration Data
**************************************************************************************************/

/* List of characteristics to configure after service discovery */
static const attcDiscCfg_t medcCfgPlxsList[] =
{
  /* Read:  Features */
  {NULL, 0, PLXPC_PLXS_PLXF_HDL_IDX},

  /* Write:  Spot Check CCC descriptor  */
  {medcCccIndVal, sizeof(medcCccIndVal), PLXPC_PLXS_PLXSC_CCC_HDL_IDX},

  /* Write:  Continuous Measurement CCC descriptor  */
  {medcCccNtfVal, sizeof(medcCccNtfVal), PLXPC_PLXS_PLXC_CCC_HDL_IDX},

  /* Write:  Record Access Control Point CCC descriptor  */
  {medcCccIndVal, sizeof(medcCccIndVal), PLXPC_PLXS_RACP_CCC_HDL_IDX},
};

/* Characteristic configuration list length */
#define MEDC_CFG_PLXS_LIST_LEN   (sizeof(medcCfgPlxsList) / sizeof(attcDiscCfg_t))

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/* Control block */
static struct
{
  wsfTimer_t  racpTimer;          /*! RACP procedure timer */
  bool_t      inProgress;         /*! RACP procedure in progress */
  bool_t      restoreConnection;  /*! Indicates if a dropped connection should be restored */
} medcPlxCb;

/**************************************************************************************************
  Local Functions
**************************************************************************************************/

static void medcPlxpInit(void);
static bool_t medcPlxpDiscover(dmConnId_t connId);
static void medcPlxpConfigure(dmConnId_t connId, uint8_t status);
static void medcPlxpProcMsg(wsfMsgHdr_t *pMsg);
static void medcPlxpBtn(dmConnId_t connId, uint8_t btn);

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! profile interface pointer */
medcIf_t medcPlxpIf =
{
  medcPlxpInit,
  medcPlxpDiscover,
  medcPlxpConfigure,
  medcPlxpProcMsg,
  medcPlxpBtn
};

/**************************************************************************************************
  External Variables
**************************************************************************************************/

/*! MEDC application master configuration */
extern const appMasterCfg_t medcMasterCfg;

/*! App connection control block */
extern appConnCb_t appConnCb[DM_CONN_MAX];

/*************************************************************************************************/
/*!
 *  \brief  Process a received ATT read response, notification, or indication.
 *
 *  \param  pMsg    Pointer to ATT callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void medcPlxsValueUpdate(attEvt_t *pMsg)
{
  if (pMsg->hdr.status == ATT_SUCCESS)
  {
    /* determine which profile the handle belongs to; start with most likely */

    /* pulse oximeter */
    if (PlxpcPlxsValueUpdate(pMedcPlxsHdlList, pMsg) == ATT_SUCCESS)
    {
      return;
    }

    /* device information */
    if (DisValueUpdate(pMedcDisHdlList, pMsg) == ATT_SUCCESS)
    {
      return;
    }

    /* GATT */
    if (GattValueUpdate(pMedcGattHdlList, pMsg) == ATT_SUCCESS)
    {
      return;
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
static void medcPlxpProcMsg(wsfMsgHdr_t *pMsg)
{
  appConnCb_t *pCb;

  switch(pMsg->event)
  {
    case DM_CONN_CLOSE_IND:
      /* if procedure in progress stop procedure timer */
      if (medcPlxCb.inProgress)
      {
        medcPlxCb.inProgress = FALSE;
        WsfTimerStop(&medcPlxCb.racpTimer);
      }

      if (medcPlxCb.restoreConnection == TRUE)
      {
        medcCb.autoConnect = TRUE;
        AppScanStart(medcMasterCfg.discMode, medcMasterCfg.scanType,
                       medcMasterCfg.scanDuration);
      }
      break;

    case DM_SEC_ENCRYPT_IND:
      if (medcPlxCb.restoreConnection == TRUE)
      {
        AppDiscConfigure((dmConnId_t) pMsg->param, HCI_SUCCESS, MEDC_CFG_PLXS_LIST_LEN,
                         (attcDiscCfg_t *) medcCfgPlxsList,
                         PLXPC_PLXS_HDL_LIST_LEN, pMedcPlxsHdlList);

        medcPlxCb.restoreConnection = FALSE;
      }
      break;

    case DM_SEC_ENCRYPT_FAIL_IND:
      if (medcPlxCb.restoreConnection == TRUE)
      {
        /* look up app connection control block from DM connection ID */
        pCb = &appConnCb[pMsg->param - 1];

        pCb->initiatingSec = FALSE;

        /* if database record handle valid */
        if (pCb->dbHdl != APP_DB_HDL_NONE)
        {
          AppDbDeleteRecord(pCb->dbHdl);
          pCb->dbHdl = APP_DB_HDL_NONE;
        }

        AppMasterSecurityReq((dmConnId_t) pMsg->param);
      }
      break;

    case ATTC_READ_RSP:
    case ATTC_HANDLE_VALUE_NTF:
      medcPlxsValueUpdate((attEvt_t *) pMsg);
      break;

    case ATTC_WRITE_RSP:
      /* if write to RACP was successful, start procedure timer */
      if ((((attEvt_t *) pMsg)->hdr.status == ATT_SUCCESS) &&
          (((attEvt_t *) pMsg)->handle == pMedcPlxsHdlList[PLXPC_PLXS_RACP_HDL_IDX]))
      {
        medcPlxCb.inProgress = TRUE;
        medcPlxCb.racpTimer.msg.param = pMsg->param;    /* conn ID */
        WsfTimerStartSec(&medcPlxCb.racpTimer, ATT_MAX_TRANS_TIMEOUT);
      }
      break;

    case ATTC_HANDLE_VALUE_IND:
      /* if procedure in progress stop procedure timer */
      if (medcPlxCb.inProgress)
      {
        medcPlxCb.inProgress = FALSE;
        WsfTimerStop(&medcPlxCb.racpTimer);
      }

      medcPlxsValueUpdate((attEvt_t *) pMsg);
      break;

    case MEDC_TIMER_IND:
      /* if procedure in progress then close connection */
      if (medcPlxCb.inProgress && pMsg->param != DM_CONN_ID_NONE)
      {
        medcPlxCb.inProgress = FALSE;

        /* if configured to disconnect upon ATT transaction timeout */
        if (pAppCfg->disconnect)
        {
          AppConnClose((dmConnId_t)pMsg->param);
        }
      }
      break;

    default:
      break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Profile initialization function.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void medcPlxpInit(void)
{
  /* Set configuration pointers */
  pAttCfg = (attCfg_t *) &medcPlxpAttCfg;

  /* set handle list length */
  medcCb.hdlListLen = MEDC_DISC_HDL_LIST_LEN;

  /* set autoconnect UUID */
  medcCb.autoUuid[0] = ATT_UUID_PULSE_OXIMITER_SERVICE;

  /* initialize timer */
  medcPlxCb.racpTimer.handlerId = medcCb.handlerId;
  medcPlxCb.racpTimer.msg.event = MEDC_TIMER_IND;
}

/*************************************************************************************************/
/*!
 *  \brief  Discover service for profile.
 *
 *  \param  connId    Connection identifier.
 *
 *  \return TRUE - finished discovering services. FALSE - more services are to be discovered.
 */
/*************************************************************************************************/
static bool_t medcPlxpDiscover(dmConnId_t connId)
{
  /* discover health pulse oximeter service */
  PlxpcPlxsDiscover(connId, pMedcPlxsHdlList);

  return TRUE;
}

/*************************************************************************************************/
/*!
 *  \brief  Configure service for profile.
 *
 *  \param  connId    Connection identifier.
 *  \param  status    APP_DISC_CFG_START or APP_DISC_CFG_CONN_START.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void medcPlxpConfigure(dmConnId_t connId, uint8_t status)
{
  /* configure health pulse oximeter service */
  AppDiscConfigure(connId, status, MEDC_CFG_PLXS_LIST_LEN,
                   (attcDiscCfg_t *) medcCfgPlxsList,
                   PLXPC_PLXS_HDL_LIST_LEN, pMedcPlxsHdlList);
}

/*************************************************************************************************/
/*!
 *  \brief  Handle a button press.
 *
 *  \param  connId    Connection identifier.
 *  \param  btn       Button press.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void medcPlxpBtn(dmConnId_t connId, uint8_t btn)
{
  /* button actions when connected */
  if (connId != DM_CONN_ID_NONE)
  {
    switch (btn)
    {
      case APP_UI_BTN_1_MED:
        medcPlxCb.restoreConnection = TRUE;
        AppConnClose(connId);
        break;

      case APP_UI_BTN_2_SHORT:
        PlxpcPlxsRacpSend(connId, pMedcPlxsHdlList[PLXPC_PLXS_RACP_HDL_IDX], CH_RACP_OPCODE_DELETE, CH_RACP_OPERATOR_ALL);
        break;

      case APP_UI_BTN_2_MED:
        PlxpcPlxsRacpSend(connId, pMedcPlxsHdlList[PLXPC_PLXS_RACP_HDL_IDX], CH_RACP_OPCODE_REPORT, CH_RACP_OPERATOR_ALL);
        break;

      case APP_UI_BTN_2_LONG:
        PlxpcPlxsRacpSend(connId, pMedcPlxsHdlList[PLXPC_PLXS_RACP_HDL_IDX], CH_RACP_OPCODE_REPORT_NUM, CH_RACP_OPERATOR_ALL);
        break;

      case APP_UI_BTN_2_EX_LONG:
        PlxpcPlxsRacpSend(connId, pMedcPlxsHdlList[PLXPC_PLXS_RACP_HDL_IDX], CH_RACP_OPCODE_ABORT, CH_RACP_OPERATOR_ALL);
        break;
    }
  }
}
