/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Medical sensor sample, weight scale profile
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
#include "util/bstream.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "hci_api.h"
#include "dm_api.h"
#include "att_api.h"
#include "app_api.h"
#include "app_ui.h"
#include "svc_ch.h"
#include "svc_wss.h"
#include "svc_dis.h"
#include "svc_core.h"
#include "gatt/gatt_api.h"
#include "wsps/wsps_api.h"
#include "meds/meds_main.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! enumeration of client characteristic configuration descriptors */
enum
{
  MEDS_WSP_GATT_SC_CCC_IDX,                /*! GATT service, service changed characteristic */
  MEDS_WSP_WSS_WSM_CCC_IDX,                /*! Weight scale service, weight scale measurement characteristic */
  MEDS_WSP_NUM_CCC_IDX
};

/**************************************************************************************************
  Advertising Data
**************************************************************************************************/

/*! Service UUID list */
static const uint8_t medsSvcUuidList[] =
{
  UINT16_TO_BYTES(ATT_UUID_WEIGHT_SCALE_SERVICE),
  UINT16_TO_BYTES(ATT_UUID_DEVICE_INFO_SERVICE)
};

/**************************************************************************************************
  Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t medsWspCccSet[MEDS_WSP_NUM_CCC_IDX] =
{
  /* cccd handle          value range               security level */
  {GATT_SC_CH_CCC_HDL,    ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_ENC},    /* MEDS_WSP_GATT_SC_CCC_IDX */
  {WSS_WM_CH_CCC_HDL,     ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_ENC}     /* MEDS_WSP_WSS_WSM_CCC_IDX */
};

/**************************************************************************************************
  Local Functions
**************************************************************************************************/

static void medsWspStart(void);
static void medsWspProcMsg(wsfMsgHdr_t *pMsg);
static void medsWspBtn(dmConnId_t connId, uint8_t btn);

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! profile interface pointer */
medsIf_t medsWspIf =
{
  NULL,
  medsWspStart,
  medsWspProcMsg,
  medsWspBtn
};

/*************************************************************************************************/
/*!
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void medsWspStart(void)
{
  /* set up CCCD table and callback */
  AttsCccRegister(MEDS_WSP_NUM_CCC_IDX, (attsCccSet_t *) medsWspCccSet, medsCccCback);

  /* add weight scale service */
  SvcWssAddGroup();

  /* Set Service Changed CCCD index. */
  GattSetSvcChangedIdx(MEDS_WSP_GATT_SC_CCC_IDX);

  /* initialize weight scale profile sensor */
  WspsSetWsmFlags(CH_WSM_FLAG_UNITS_LBS | CH_WSM_FLAG_TIMESTAMP);

  /* set advertising data */
  AppAdvSetAdValue(APP_ADV_DATA_DISCOVERABLE, DM_ADV_TYPE_16_UUID, sizeof(medsSvcUuidList),
                   (uint8_t *) medsSvcUuidList);
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
static void medsWspProcMsg(wsfMsgHdr_t *pMsg)
{
  return;
}

/*************************************************************************************************/
/*!
 *  \brief  Button press callback.
 *
 *  \param  connId  Connection identifier.
 *  \param  btn     Button press.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void medsWspBtn(dmConnId_t connId, uint8_t btn)
{
  /* button actions when connected */
  if (connId != DM_CONN_ID_NONE)
  {
    switch (btn)
    {
      case APP_UI_BTN_1_SHORT:
        /* send measurement */
        WspsMeasComplete(connId, MEDS_WSP_WSS_WSM_CCC_IDX);
        break;

      default:
        break;
    }
  }
}
