/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Health/medical collector, Weight Scale profile
 *
 *  Copyright (c) 2012-2019 Arm Ltd. All Rights Reserved.
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
#include "svc_ch.h"
#include "wspc/wspc_api.h"
#include "udsc/udsc_api.h"
#include "medc/medc_main.h"


/**************************************************************************************************
  ATT Client Discovery Data
**************************************************************************************************/

/*! Discovery states:  enumeration of services to be discovered */
enum
{
  MEDC_DISC_WSP_UDS_SVC,       /*! User data service */
  MEDC_DISC_WSP_WSS_SVC,       /*! Weight scale service */
};

/* Start of cached weight scale service handles; begins after DIS */
#define MEDC_DISC_WSS_START         (MEDC_DISC_DIS_START + DIS_HDL_LIST_LEN)

/* Start of User Data Service handles; begins after weight scale serivce */
#define MEDC_DISC_UDS_START         (MEDC_DISC_WSS_START + WSPC_WSS_HDL_LIST_LEN)

/* Total cached handle list length */
#define MEDC_DISC_HDL_LIST_LEN      (MEDC_DISC_UDS_START + UDSC_HDL_LIST_LEN)

/*! Pointers into handle list for weight scale service handles */
static uint16_t *pMedcWssHdlList = &medcCb.hdlList[MEDC_DISC_WSS_START];

/*! Pointers into handle list for weight scale service handles */
static uint16_t *pMedcUdsHdlList = &medcCb.hdlList[MEDC_DISC_UDS_START];

/*! Weight Scale Profile discovery state */
static uint8_t pMedcWspDiscState;

/* sanity check:  make sure handle list length is <= app db handle list length */
WSF_CT_ASSERT(MEDC_DISC_HDL_LIST_LEN <= APP_DB_HDL_LIST_LEN);

/**************************************************************************************************
  ATT Client Configuration Data
**************************************************************************************************/

/* List of characteristics to configure after service discovery */
static const attcDiscCfg_t medcCfgWssList[] =
{
  /* Read:  Weight scale feature */
  {NULL, 0, MEDC_DISC_WSS_START + WSPC_WSS_WSF_HDL_IDX},

  /* Write:  Weight scale measurement CCC descriptor  */
  {medcCccIndVal, sizeof(medcCccIndVal), MEDC_DISC_WSS_START + WSPC_WSS_WSM_CCC_HDL_IDX},

  /* Write:  Database Change Interval CCC descriptor  */
  {medcCccNtfVal, sizeof(medcCccNtfVal), MEDC_DISC_UDS_START + UDSC_DCBI_CCC_HDL_IDX},

  /* Write:  User Data Service Control Point CCC descriptor  */
  {medcCccIndVal, sizeof(medcCccIndVal), MEDC_DISC_UDS_START + UDSC_UCP_CCC_HDL_IDX}
};

/* Characteristic configuration list length */
#define MEDC_CFG_WSS_LIST_LEN   (sizeof(medcCfgWssList) / sizeof(attcDiscCfg_t))

/**************************************************************************************************
  Local Functions
**************************************************************************************************/

static void medcWspInit(void);
static bool_t medcWspDiscover(dmConnId_t connId);
static void medcWspConfigure(dmConnId_t connId, uint8_t status);
static void medcWspProcMsg(wsfMsgHdr_t *pMsg);
static void medcWspBtn(dmConnId_t connId, uint8_t btn);

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! profile interface pointer */
medcIf_t medcWspIf =
{
  medcWspInit,
  medcWspDiscover,
  medcWspConfigure,
  medcWspProcMsg,
  medcWspBtn
};

/*************************************************************************************************/
/*!
 *  \brief  Process a received ATT read response, notification, or indication.
 *
 *  \param  pMsg    Pointer to ATT callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void medcWssValueUpdate(attEvt_t *pMsg)
{
  if (pMsg->hdr.status == ATT_SUCCESS)
  {
    /* determine which profile the handle belongs to; start with most likely */

    /* weight scale */
    if (WspcWssValueUpdate(pMedcWssHdlList, pMsg) == ATT_SUCCESS)
    {
      return;
    }
    /* user data service */
    if (UdscValueUpdate(pMedcUdsHdlList, pMsg) == ATT_SUCCESS)
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
static void medcWspProcMsg(wsfMsgHdr_t *pMsg)
{
  switch(pMsg->event)
  {
    case DM_CONN_CLOSE_IND:
      /* initialize discovery state */
      pMedcWspDiscState = MEDC_DISC_WSP_UDS_SVC;

      /* Notify UDSC the connection closed */
      UdscClose();
      break;

    case ATTC_HANDLE_VALUE_IND:
      medcWssValueUpdate((attEvt_t *) pMsg);
      break;

    case MEDC_TIMER_IND:
      APP_TRACE_INFO0("medcWspProcMsg - Timeout waiting for UDS control point response.");
      break;

    default:
      break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  UDS Control Point Response Callback.
 *
 *  \param  connId      Connection ID.
 *  \param  opcode      Cmd opcode being responded to.
 *  \param  response    Response code.
 *  \param  index       USer index (only set when opcode is UDSC_UCP_OPCODE_RNU)
 *
 *  \return None
 */
/*************************************************************************************************/
static void medcWspUdsRspCallback(dmConnId_t connId, uint8_t opcode, uint8_t response, uint8_t index)
{
  APP_TRACE_INFO3("medcWspUdsRspCallback - op: %d  rsp: %d  index: %d", opcode, response, index);
}

/*************************************************************************************************/
/*!
 *  \brief  Profile initialization function.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void medcWspInit(void)
{
  /* set handle list length */
  medcCb.hdlListLen = MEDC_DISC_HDL_LIST_LEN;

  /* set autoconnect UUID */
  medcCb.autoUuid[0] = ATT_UUID_WEIGHT_SCALE_SERVICE;
  medcCb.autoUuid[1] = ATT_UUID_USER_DATA_SERVICE;

  /* Register the UDS Control Point Callback */
  UdscInit(medcCb.handlerId, MEDC_TIMER_IND, medcWspUdsRspCallback);
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
static bool_t medcWspDiscover(dmConnId_t connId)
{
  if (pMedcWspDiscState == MEDC_DISC_WSP_UDS_SVC)
  {
    /* user data service */
    UdscDiscover(connId, pMedcUdsHdlList);
    pMedcWspDiscState++;
    return FALSE;
  }

  if (pMedcWspDiscState == MEDC_DISC_WSP_WSS_SVC)
  {
    /* discover weight scale service */
    WspcWssDiscover(connId, pMedcWssHdlList);
    pMedcWspDiscState++;
  }

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
static void medcWspConfigure(dmConnId_t connId, uint8_t status)
{
  /* configure weight scale service */
  AppDiscConfigure(connId, status, MEDC_CFG_WSS_LIST_LEN,
                   (attcDiscCfg_t *) medcCfgWssList,
                   WSPC_WSS_HDL_LIST_LEN, medcCb.hdlList);
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
static void medcWspBtn(dmConnId_t connId, uint8_t btn)
{
  switch (btn)
  {
    case APP_UI_BTN_2_SHORT:
      UdscRegisterNewUser(connId, pMedcUdsHdlList[UDSC_UCP_IDX], 1234);
      break;
    case APP_UI_BTN_2_MED:
      UdscConsent(connId, pMedcUdsHdlList[UDSC_UCP_IDX], 2, 1234);
      break;
    case APP_UI_BTN_2_LONG:
      UdscDeleteUserData(connId, pMedcUdsHdlList[UDSC_UCP_IDX]);
      break;
    case APP_UI_BTN_2_EX_LONG:
      UdscDeleteUserData(connId, pMedcUdsHdlList[UDSC_UCP_IDX]);
      break;
    case APP_UI_BTN_1_SHORT:
      UdscReadDatabaseChangeIncrement(connId, pMedcUdsHdlList[UDSC_DBCI_HDL_IDX]);
      break;
    case APP_UI_BTN_1_MED:
      UdscReadUserIndex(connId, pMedcUdsHdlList[UDSC_UI_HDL_IDX]);
      break;
    case APP_UI_BTN_1_EX_LONG:
      UdscWriteDatabaseChangeIncrement(connId, pMedcUdsHdlList[UDSC_DBCI_HDL_IDX], 1);
      break;
    default:
      break;
  }
}
