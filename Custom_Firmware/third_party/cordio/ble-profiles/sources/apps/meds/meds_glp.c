/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Medical sensor sample, glucose profile
 *
 *  Copyright (c) 2016-2019 Arm Ltd. All Rights Reserved.
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
#include "app_hw.h"
#include "svc_ch.h"
#include "svc_gls.h"
#include "svc_dis.h"
#include "svc_core.h"
#include "gatt/gatt_api.h"
#include "glps/glps_api.h"
#include "meds/meds_main.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! enumeration of client characteristic configuration descriptors */
enum
{
  MEDS_GLS_GATT_SC_CCC_IDX,               /*! GATT service, service changed characteristic */
  MEDS_GLS_GLS_GLM_CCC_IDX,               /*! Glucose measurement characteristic */
  MEDS_GLS_GLS_GLMC_CCC_IDX,              /*! Glucose measurement context characteristic */
  MEDS_GLS_GLS_RACP_CCC_IDX,              /*! Glucose record access control point characteristic */
  MEDS_GLS_NUM_CCC_IDX
};

/**************************************************************************************************
  Advertising Data
**************************************************************************************************/

/*! Service UUID list */
static const uint8_t medsSvcUuidList[] =
{
  UINT16_TO_BYTES(ATT_UUID_GLUCOSE_SERVICE),
  UINT16_TO_BYTES(ATT_UUID_DEVICE_INFO_SERVICE)
};

/**************************************************************************************************
  Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t medsGlpCccSet[MEDS_GLS_NUM_CCC_IDX] =
{
  /* cccd handle          value range               security level */
  {GATT_SC_CH_CCC_HDL,    ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_NONE},    /* MEDS_GLS_GATT_SC_CCC_IDX */
  {GLS_GLM_CH_CCC_HDL,    ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},    /* MEDS_GLS_GLS_GLM_CCC_IDX */
  {GLS_GLMC_CH_CCC_HDL,   ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},    /* MEDS_GLS_GLS_GLMC_CCC_IDX */
  {GLS_RACP_CH_CCC_HDL,   ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_NONE}     /* MEDS_GLS_GLS_RACP_CCC_IDX */
};

/**************************************************************************************************
  Local Functions
**************************************************************************************************/

static void medsGlpStart(void);
static void medsGlpProcMsg(wsfMsgHdr_t *pMsg);
static void medsGlpBtn(dmConnId_t connId, uint8_t btn);

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! profile interface pointer */
medsIf_t medsGlpIf =
{
  NULL,
  medsGlpStart,
  medsGlpProcMsg,
  medsGlpBtn
};

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void medsGlpStart(void)
{
  /* set up CCCD table and callback */
  AttsCccRegister(MEDS_GLS_NUM_CCC_IDX, (attsCccSet_t *) medsGlpCccSet, medsCccCback);

  /* add glucose service */
  SvcGlsAddGroup();
  SvcGlsCbackRegister(NULL, GlpsRacpWriteCback);

  /* Set Service Changed CCCD index. */
  GattSetSvcChangedIdx(MEDS_GLS_GATT_SC_CCC_IDX);

  /* initialize glucose profile sensor */
  GlpsInit();
  GlpsSetCccIdx(MEDS_GLS_GLS_GLM_CCC_IDX, MEDS_GLS_GLS_GLMC_CCC_IDX, MEDS_GLS_GLS_RACP_CCC_IDX);

  GlpsSetFeature(GLP_ALL_SUPPORTED_FEATURES);

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
static void medsGlpProcMsg(wsfMsgHdr_t *pMsg)
{
  GlpsProcMsg(pMsg);
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
static void medsGlpBtn(dmConnId_t connId, uint8_t btn)
{
  GlpsBtn(connId, btn);
}
