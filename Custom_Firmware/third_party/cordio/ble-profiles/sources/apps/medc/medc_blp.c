/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Health/medical collector, Blood Pressure profile
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
#include "blpc/blpc_api.h"
#include "medc/medc_main.h"

/**************************************************************************************************
  ATT Client Discovery Data
**************************************************************************************************/

/* Start of cached blood pressure service handles; begins after DIS */
#define MEDC_DISC_BPS_START         (MEDC_DISC_DIS_START + DIS_HDL_LIST_LEN)

/* Total cached handle list length */
#define MEDC_DISC_HDL_LIST_LEN      (MEDC_DISC_BPS_START + BLPC_BPS_HDL_LIST_LEN)

/*! Pointers into handle list for blood pressure service handles */
static uint16_t *pMedcBpsHdlList = &medcCb.hdlList[MEDC_DISC_BPS_START];

/* sanity check:  make sure handle list length is <= app db handle list length */
WSF_CT_ASSERT(MEDC_DISC_HDL_LIST_LEN <= APP_DB_HDL_LIST_LEN);

/**************************************************************************************************
  ATT Client Configuration Data
**************************************************************************************************/

/* List of characteristics to configure after service discovery */
static const attcDiscCfg_t medcCfgBpsList[] =
{
  /* Read:  Blood pressure feature */
  {NULL, 0, BLPC_BPS_BPF_HDL_IDX},

  /* Write:  Blood pressure measurement CCC descriptor  */
  {medcCccIndVal, sizeof(medcCccIndVal), BLPC_BPS_BPM_CCC_HDL_IDX},

  /* Write:  Intermediate cuff pressure CCC descriptor  */
  {medcCccNtfVal, sizeof(medcCccNtfVal), BLPC_BPS_ICP_CCC_HDL_IDX},
};

/* Characteristic configuration list length */
#define MEDC_CFG_BPS_LIST_LEN   (sizeof(medcCfgBpsList) / sizeof(attcDiscCfg_t))

/**************************************************************************************************
  Local Functions
**************************************************************************************************/

static void medcBlpInit(void);
static bool_t medcBlpDiscover(dmConnId_t connId);
static void medcBlpConfigure(dmConnId_t connId, uint8_t status);
static void medcBlpProcMsg(wsfMsgHdr_t *pMsg);
static void medcBlpBtn(dmConnId_t connId, uint8_t btn);

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! profile interface pointer */
medcIf_t medcBlpIf =
{
  medcBlpInit,
  medcBlpDiscover,
  medcBlpConfigure,
  medcBlpProcMsg,
  medcBlpBtn
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
static void medcBpsValueUpdate(attEvt_t *pMsg)
{
  if (pMsg->hdr.status == ATT_SUCCESS)
  {
    /* determine which profile the handle belongs to; start with most likely */

    /* blood pressure */
    if (BlpcBpsValueUpdate(pMedcBpsHdlList, pMsg) == ATT_SUCCESS)
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
static void medcBlpProcMsg(wsfMsgHdr_t *pMsg)
{
  switch(pMsg->event)
  {
    case ATTC_READ_RSP:
    case ATTC_HANDLE_VALUE_NTF:
    case ATTC_HANDLE_VALUE_IND:
      medcBpsValueUpdate((attEvt_t *) pMsg);
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
static void medcBlpInit(void)
{
  /* set handle list length */
  medcCb.hdlListLen = MEDC_DISC_HDL_LIST_LEN;

  /* set autoconnect UUID */
  medcCb.autoUuid[0] = ATT_UUID_BLOOD_PRESSURE_SERVICE;
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
static bool_t medcBlpDiscover(dmConnId_t connId)
{
  /* discover blood pressure service */
  BlpcBpsDiscover(connId, pMedcBpsHdlList);

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
static void medcBlpConfigure(dmConnId_t connId, uint8_t status)
{
  /* configure blood pressure service */
  AppDiscConfigure(connId, status, MEDC_CFG_BPS_LIST_LEN,
                   (attcDiscCfg_t *) medcCfgBpsList,
                   BLPC_BPS_HDL_LIST_LEN, pMedcBpsHdlList);
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
static void medcBlpBtn(dmConnId_t connId, uint8_t btn)
{
  return;
}
