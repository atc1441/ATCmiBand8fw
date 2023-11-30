/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Health/medical collector, Health Thermometer profile
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
#include "htpc/htpc_api.h"
#include "medc/medc_main.h"

/**************************************************************************************************
  ATT Client Discovery Data
**************************************************************************************************/

/* Start of cached health thermometer service handles; begins after DIS */
#define MEDC_DISC_HTS_START         (MEDC_DISC_DIS_START + DIS_HDL_LIST_LEN)

/* Total cached handle list length */
#define MEDC_DISC_HDL_LIST_LEN      (MEDC_DISC_HTS_START + HTPC_HTS_HDL_LIST_LEN)

/*! Pointers into handle list for health thermometer service handles */
static uint16_t *pMedcHtsHdlList = &medcCb.hdlList[MEDC_DISC_HTS_START];

/* sanity check:  make sure handle list length is <= app db handle list length */
WSF_CT_ASSERT(MEDC_DISC_HDL_LIST_LEN <= APP_DB_HDL_LIST_LEN);

/**************************************************************************************************
  ATT Client Configuration Data
**************************************************************************************************/

/* List of characteristics to configure after service discovery */
static const attcDiscCfg_t medcCfgHtsList[] =
{
  /* Read:  Temperature type */
  {NULL, 0, HTPC_HTS_TT_HDL_IDX},

  /* Write:  Temperature measurement CCC descriptor  */
  {medcCccIndVal, sizeof(medcCccIndVal), HTPC_HTS_TM_CCC_HDL_IDX},

  /* Write:  Intermediate temperature CCC descriptor  */
  {medcCccNtfVal, sizeof(medcCccNtfVal), HTPC_HTS_IT_CCC_HDL_IDX},
};

/* Characteristic configuration list length */
#define MEDC_CFG_HTS_LIST_LEN   (sizeof(medcCfgHtsList) / sizeof(attcDiscCfg_t))

/**************************************************************************************************
  Local Functions
**************************************************************************************************/

static void medcHtpInit(void);
static bool_t medcHtpDiscover(dmConnId_t connId);
static void medcHtpConfigure(dmConnId_t connId, uint8_t status);
static void medcHtpProcMsg(wsfMsgHdr_t *pMsg);
static void medcHtpBtn(dmConnId_t connId, uint8_t btn);

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! profile interface pointer */
medcIf_t medcHtpIf =
{
  medcHtpInit,
  medcHtpDiscover,
  medcHtpConfigure,
  medcHtpProcMsg,
  medcHtpBtn
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
static void medcHtsValueUpdate(attEvt_t *pMsg)
{
  if (pMsg->hdr.status == ATT_SUCCESS)
  {
    /* determine which profile the handle belongs to; start with most likely */

    /* health thermometer */
    if (HtpcHtsValueUpdate(pMedcHtsHdlList, pMsg) == ATT_SUCCESS)
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
static void medcHtpProcMsg(wsfMsgHdr_t *pMsg)
{
  switch(pMsg->event)
  {
    case ATTC_READ_RSP:
    case ATTC_HANDLE_VALUE_NTF:
    case ATTC_HANDLE_VALUE_IND:
      medcHtsValueUpdate((attEvt_t *) pMsg);
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
static void medcHtpInit(void)
{
  /* set handle list length */
  medcCb.hdlListLen = MEDC_DISC_HDL_LIST_LEN;

  /* set autoconnect UUID */
  medcCb.autoUuid[0] = ATT_UUID_HEALTH_THERM_SERVICE;
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
static bool_t medcHtpDiscover(dmConnId_t connId)
{
  /* discover health thermometer service */
  HtpcHtsDiscover(connId, pMedcHtsHdlList);

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
static void medcHtpConfigure(dmConnId_t connId, uint8_t status)
{
  /* configure health thermometer service */
  AppDiscConfigure(connId, status, MEDC_CFG_HTS_LIST_LEN,
                   (attcDiscCfg_t *) medcCfgHtsList,
                   HTPC_HTS_HDL_LIST_LEN, pMedcHtsHdlList);
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
static void medcHtpBtn(dmConnId_t connId, uint8_t btn)
{
  return;
}
