/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Medical sensor sample, health thermometer profile
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
#include "app_hw.h"
#include "app_ui.h"
#include "svc_ch.h"
#include "svc_hts.h"
#include "svc_dis.h"
#include "svc_core.h"
#include "gatt/gatt_api.h"
#include "htps/htps_api.h"
#include "meds/meds_main.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! enumeration of client characteristic configuration descriptors */
enum
{
  MEDS_HTP_GATT_SC_CCC_IDX,                /*! GATT service, service changed characteristic */
  MEDS_HTP_HTS_TM_CCC_IDX,                 /*! Health thermometer service, temperature measurement characteristic */
  MEDS_HTP_HTS_IT_CCC_IDX,                 /*! Health thermometer service, intermediate temperature characteristic */
  MEDS_HTP_NUM_CCC_IDX
};

/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/

/*! health thermometer measurement configuration */
static const htpsCfg_t medsHtpsCfg =
{
  2000      /*! Measurement timer expiration period in ms */
};

/**************************************************************************************************
  Advertising Data
**************************************************************************************************/

/*! Service UUID list */
static const uint8_t medsSvcUuidList[] =
{
  UINT16_TO_BYTES(ATT_UUID_HEALTH_THERM_SERVICE),
  UINT16_TO_BYTES(ATT_UUID_DEVICE_INFO_SERVICE)
};

/**************************************************************************************************
  Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t medsHtpCccSet[MEDS_HTP_NUM_CCC_IDX] =
{
  /* cccd handle          value range               security level */
  {GATT_SC_CH_CCC_HDL,    ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_ENC},    /* MEDS_HTP_GATT_SC_CCC_IDX */
  {HTS_TM_CH_CCC_HDL,     ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_NONE},    /* MEDS_HTP_HTS_TM_CCC_IDX */
  {HTS_IT_CH_CCC_HDL,     ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_ENC}     /* MEDS_HTP_HTS_IT_CCC_IDX */
};

/**************************************************************************************************
  Local Functions
**************************************************************************************************/

static void medsHtpStart(void);
static void medsHtpProcMsg(wsfMsgHdr_t *pMsg);
static void medsHtpBtn(dmConnId_t connId, uint8_t btn);

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! profile interface pointer */
medsIf_t medsHtpIf =
{
  NULL,
  medsHtpStart,
  medsHtpProcMsg,
  medsHtpBtn
};

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! application control block */
static struct
{
  bool_t            measuring;
  bool_t            storedMeasurement;
} medsHtpCb;

/*************************************************************************************************/
/*!
 *  \brief  Set temperature measurement units.
 *
 *  \param  units   CH_TM_FLAG_UNITS_C or CH_TM_FLAG_UNITS_F.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void medsHtpSetUnits(uint8_t units)
{
  HtpsSetTmFlags(units | CH_TM_FLAG_TIMESTAMP);
  HtpsSetItFlags(units);
  AppHwTmSetUnits(units);
}

/*************************************************************************************************/
/*!
 *  \brief  Perform UI actions on connection close.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void medsHtpClose(wsfMsgHdr_t *pMsg)
{
  /* stop health thermometer measurement */
  HtpsMeasStop();
  medsHtpCb.measuring = FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void medsHtpStart(void)
{
  /* set up CCCD table and callback */
  AttsCccRegister(MEDS_HTP_NUM_CCC_IDX, (attsCccSet_t *) medsHtpCccSet, medsCccCback);

  /* add health thermometer service */
  SvcHtsAddGroup();

  /* Set Service Changed CCCD index. */
  GattSetSvcChangedIdx(MEDS_HTP_GATT_SC_CCC_IDX);

  /* initialize health thermometer profile sensor */
  HtpsInit(medsCb.handlerId, (htpsCfg_t *) &medsHtpsCfg);
  medsHtpSetUnits(CH_TM_FLAG_UNITS_C);

  /* set advertising data */
  AppAdvSetAdValue(APP_ADV_DATA_DISCOVERABLE, DM_ADV_TYPE_16_UUID, sizeof(medsSvcUuidList),
                   (uint8_t *) medsSvcUuidList);
}

/*************************************************************************************************/
/*!
*  \brief  Send stored temperature measurement.
*
*  \param  connId    connection ID to send stored measurement to.
*
*  \return None.
*/
/*************************************************************************************************/
static void medsHtpSendStoredMeasurement(dmConnId_t connId)
{
  medsHtpCb.storedMeasurement = FALSE;

  HtpsMeasComplete((dmConnId_t)connId, MEDS_HTP_HTS_TM_CCC_IDX);
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
static void medsHtpProcMsg(wsfMsgHdr_t *pMsg)
{
  switch(pMsg->event)
  {
    case MEDS_TIMER_IND:
      HtpsProcMsg(pMsg);
      break;

    case ATTS_CCC_STATE_IND:
      /* Check if stored measurement exists and indications on temperature measurement enabled */
      if (medsHtpCb.storedMeasurement && AttsCccEnabled((dmConnId_t)pMsg->param, MEDS_HTP_HTS_TM_CCC_IDX))
      {
        medsHtpSendStoredMeasurement((dmConnId_t)pMsg->param);
      }
      break;

    case DM_CONN_CLOSE_IND:
      medsHtpClose(pMsg);
      break;

    default:
      break;
  }
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
static void medsHtpBtn(dmConnId_t connId, uint8_t btn)
{
  static bool_t advNonconn = FALSE;

  switch (btn)
  {
    case APP_UI_BTN_1_SHORT:
      /* if connected */
      if (connId != DM_CONN_ID_NONE)
      {
        /* start or complete measurement */
        if (!medsHtpCb.measuring)
        {
          HtpsMeasStart(connId, MEDS_TIMER_IND, MEDS_HTP_HTS_IT_CCC_IDX);
          medsHtpCb.measuring = TRUE;
        }
        else
        {
          HtpsMeasComplete(connId, MEDS_HTP_HTS_TM_CCC_IDX);
          medsHtpCb.measuring = FALSE;
        }
      }
      break;

    case APP_UI_BTN_2_SHORT:
      /* set units */
      medsHtpSetUnits(CH_TM_FLAG_UNITS_F);
      break;

    case APP_UI_BTN_2_MED:
      /* set units */
      medsHtpSetUnits(CH_TM_FLAG_UNITS_C);
      break;

    case APP_UI_BTN_2_LONG:
      /* set new advertising type */
      advNonconn = !advNonconn;
      AppSetAdvType(advNonconn ? DM_ADV_NONCONN_UNDIRECT : DM_ADV_CONN_UNDIRECT);
      break;

    case APP_UI_BTN_2_EX_LONG:
      if (connId == DM_CONN_ID_NONE)
      {
        /* Store a temperature measurement to be sent when connected */
        medsHtpCb.storedMeasurement = TRUE;
      }
      break;

    default:
      break;
  }
}
