/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Health/medical collector, Heart Rate profile
 *
 *  Copyright (c) 2012-2018 Arm Ltd. All Rights Reserved.
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
#include "svc_ch.h"
#include "hrpc/hrpc_api.h"
#include "medc/medc_main.h"

/**************************************************************************************************
  ATT Client Discovery Data
**************************************************************************************************/

/* Start of cached heart rate service handles; begins after DIS */
#define MEDC_DISC_HRS_START         (MEDC_DISC_DIS_START + DIS_HDL_LIST_LEN)

/* Total cached handle list length */
#define MEDC_DISC_HDL_LIST_LEN      (MEDC_DISC_HRS_START + HRPC_HRS_HDL_LIST_LEN)

/*! Pointers into handle list heart rate service handles */
static uint16_t *pMedcHrsHdlList = &medcCb.hdlList[MEDC_DISC_HRS_START];

/* sanity check:  make sure handle list length is <= app db handle list length */
WSF_CT_ASSERT(MEDC_DISC_HDL_LIST_LEN <= APP_DB_HDL_LIST_LEN);

/**************************************************************************************************
  ATT Client Configuration Data
**************************************************************************************************/

/* HRS Control point "Reset Energy Expended" */
static const uint8_t medcHrsRstEnExp[] = {CH_HRCP_RESET_ENERGY_EXP};

/* List of characteristics to configure after service discovery */
static const attcDiscCfg_t medcCfgHrsList[] =
{
  /* Read:  HRS Body sensor location */
  {NULL, 0, HRPC_HRS_BSL_HDL_IDX},

  /* Write:  HRS Control point "Reset Energy Expended"  */
  {medcHrsRstEnExp, sizeof(medcHrsRstEnExp), HRPC_HRS_HRCP_HDL_IDX},

  /* Write:  HRS Heart rate measurement CCC descriptor  */
  {medcCccNtfVal, sizeof(medcCccNtfVal), HRPC_HRS_HRM_CCC_HDL_IDX},
};

/* Characteristic configuration list length */
#define MEDC_CFG_HRS_LIST_LEN   (sizeof(medcCfgHrsList) / sizeof(attcDiscCfg_t))

/**************************************************************************************************
  Local Functions
**************************************************************************************************/

static void medcHrpInit(void);
static bool_t medcHrpDiscover(dmConnId_t connId);
static void medcHrpConfigure(dmConnId_t connId, uint8_t status);
static void medcHrpProcMsg(wsfMsgHdr_t *pMsg);
static void medcHrpBtn(dmConnId_t connId, uint8_t btn);

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! profile interface pointer */
medcIf_t medcHrpIf =
{
  medcHrpInit,
  medcHrpDiscover,
  medcHrpConfigure,
  medcHrpProcMsg,
  medcHrpBtn
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
static void medcHrsValueUpdate(attEvt_t *pMsg)
{
  if (pMsg->hdr.status == ATT_SUCCESS)
  {
    /* determine which profile the handle belongs to; start with most likely */

    /* heart rate */
    if (HrpcHrsValueUpdate(pMedcHrsHdlList, pMsg) == ATT_SUCCESS)
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
static void medcHrpProcMsg(wsfMsgHdr_t *pMsg)
{
  switch(pMsg->event)
  {
    case ATTC_READ_RSP:
    case ATTC_HANDLE_VALUE_NTF:
    case ATTC_HANDLE_VALUE_IND:
      medcHrsValueUpdate((attEvt_t *) pMsg);
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
static void medcHrpInit(void)
{
  /* set handle list length */
  medcCb.hdlListLen = MEDC_DISC_HDL_LIST_LEN;

  /* set autoconnect UUID */
  medcCb.autoUuid[0] = ATT_UUID_HEART_RATE_SERVICE;
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
static bool_t medcHrpDiscover(dmConnId_t connId)
{
  /* discover heart rate service */
  HrpcHrsDiscover(connId, pMedcHrsHdlList);

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
static void medcHrpConfigure(dmConnId_t connId, uint8_t status)
{
  /* configure heart rate service */
  AppDiscConfigure(connId, status, MEDC_CFG_HRS_LIST_LEN,
                   (attcDiscCfg_t *) medcCfgHrsList,
                   HRPC_HRS_HDL_LIST_LEN, pMedcHrsHdlList);
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
static void medcHrpBtn(dmConnId_t connId, uint8_t btn)
{
  return;
}
