/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Medical sensor sample, pulse oximeter profile
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
#include "svc_plxs.h"
#include "svc_dis.h"
#include "svc_core.h"
#include "gatt/gatt_api.h"
#include "plxps/plxps_api.h"
#include "plxps/plxps_main.h"
#include "meds/meds_main.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! enumeration of client characteristic configuration descriptors */
enum
{
  MEDS_PLX_GATT_SC_CCC_IDX,               /*! GATT service, service changed characteristic */
  MEDS_PLX_PLX_SCM_CCC_IDX,               /*! Pulse Oximeter Spot Check measurement characteristic */
  MEDS_PLX_PLX_CM_CCC_IDX,                /*! Pulse Oximeter Continuous characteristic */
  MEDS_PLX_PLX_RACP_CCC_IDX,              /*! Pulse Oximeter record access control point characteristic */
  MEDS_PLX_NUM_CCC_IDX
};

/* Default MTU */
#define MEDS_PLX_DEFAULT_MTU              50

/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/

/*! Pulse Oximeter Continuous measurement configuration */
static const plxpsCfg_t medsPlxpsCfg =
{
  2000      /*! Measurement timer expiration period in ms */
};

/*! ATT configurable parameters (increase MTU) */
static const attCfg_t medsPlxAttCfg =
{
  15,                               /* ATT server service discovery connection idle timeout in seconds */
  MEDS_PLX_DEFAULT_MTU,             /* desired ATT MTU */
  ATT_MAX_TRANS_TIMEOUT,            /* transcation timeout in seconds */
  4                                 /* number of queued prepare writes supported by server */
};

/**************************************************************************************************
  Advertising Data
**************************************************************************************************/

/*! Service UUID list */
static const uint8_t medsSvcUuidList[] =
{
  UINT16_TO_BYTES(ATT_UUID_PULSE_OXIMITER_SERVICE),
  UINT16_TO_BYTES(ATT_UUID_DEVICE_INFO_SERVICE)
};

/**************************************************************************************************
  Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t medsPlxCccSet[MEDS_PLX_NUM_CCC_IDX] =
{
  /* cccd handle                  value range               security level */
  {GATT_SC_CH_CCC_HDL,            ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_NONE},    /* MEDS_PLX_GATT_SC_CCC_IDX */
  {PLXS_SPOT_CHECK_CH_CCC_HDL,    ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_NONE},    /* MEDS_PLX_PLX_SCM_CCC_IDX */
  {PLXS_CONTINUOUS_CH_CCC_HDL,    ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},    /* MEDS_PLX_PLX_CM_CCC_IDX */
  {PLXS_RECORD_ACCESS_CH_CCC_HDL, ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_NONE}     /* MEDS_PLX_PLX_RACP_CCC_IDX */
};

/**************************************************************************************************
  Local Functions
**************************************************************************************************/

static void medsPlxInit(void);
static void medsPlxStart(void);
static void medsPlxProcMsg(wsfMsgHdr_t *pMsg);
static void medsPlxBtn(dmConnId_t connId, uint8_t btn);

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! profile interface pointer */
medsIf_t medsPlxIf =
{
  medsPlxInit,
  medsPlxStart,
  medsPlxProcMsg,
  medsPlxBtn
};

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! application control block */
static struct
{
  bool_t            measuring;
} medsPlxCb;

/*************************************************************************************************/
/*!
 *  \brief  Perform actions on connection close.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void medsPlxClose(wsfMsgHdr_t *pMsg)
{
  /* Reset control information */
  medsPlxCb.measuring = FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void medsPlxStart(void)
{
  /* set up CCCD table and callback */
  AttsCccRegister(MEDS_PLX_NUM_CCC_IDX, (attsCccSet_t *) medsPlxCccSet, medsCccCback);

  /* add pulse oximeter service */
  SvcPlxsAddGroup();
  SvcPlxsCbackRegister(NULL, PlxpsWriteCback);

  /* Set Service Changed CCCD index. */
  GattSetSvcChangedIdx(MEDS_PLX_GATT_SC_CCC_IDX);

  /* initialize pulse oximeter profile sensor */
  PlxpsInit(medsCb.handlerId, (plxpsCfg_t *) &medsPlxpsCfg);
  PlxpsSetCccIdx(MEDS_PLX_PLX_SCM_CCC_IDX, MEDS_PLX_PLX_CM_CCC_IDX, MEDS_PLX_PLX_RACP_CCC_IDX);

  PlxpsSetFeature(CH_PLF_FLAG_SENSOR_STATUS_SUP, 0, 0);

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
static void medsPlxProcMsg(wsfMsgHdr_t *pMsg)
{
  if (pMsg->event == DM_CONN_CLOSE_IND)
  {
    medsPlxClose(pMsg);
  }

  PlxpsProcMsg(pMsg);
}

/*************************************************************************************************/
/*!
 *  \brief  Profile initialization function.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void medsPlxInit(void)
{
  /* Set configuration pointers */
  pAttCfg = (attCfg_t *) &medsPlxAttCfg;
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
static void medsPlxBtn(dmConnId_t connId, uint8_t btn)
{
  appPlxCm_t record;

  /* button actions when connected */
  if (connId != DM_CONN_ID_NONE)
  {
    switch (btn)
    {
      case APP_UI_BTN_2_SHORT:
        AppHwPlxcmRead(&record);
        record.flags = CH_PLXC_FLAG_PULSE_AMP_INDX;
        plxpsSendContinuousMeas(connId, &record);
        break;

      case APP_UI_BTN_2_MED:
        plxpsDbDeleteRecords(CH_RACP_OPERATOR_ALL);
        break;

      case APP_UI_BTN_1_SHORT:
        /* start or complete measurement */
        if (!medsPlxCb.measuring)
        {
          PlxpsMeasStart(connId, MEDS_TIMER_IND, MEDS_PLX_PLX_CM_CCC_IDX);
          medsPlxCb.measuring = TRUE;
        }
        else
        {
          PlxpsMeasStop();
          medsPlxCb.measuring = FALSE;
        }
        break;

      default:
        break;
    }
  }
}
