/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Medical sensor sample, blood pressure profile
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
#include "svc_bps.h"
#include "svc_dis.h"
#include "svc_core.h"
#include "blps/blps_api.h"
#include "gatt/gatt_api.h"
#include "meds/meds_main.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! enumeration of client characteristic configuration descriptors */
enum
{
  MEDS_BLP_GATT_SC_CCC_IDX,                /*! GATT service, service changed characteristic */
  MEDS_BLP_BPS_BPM_CCC_IDX,                /*! Blood pressure service, blood pressure measurement characteristic */
  MEDS_BLP_BPS_ICP_CCC_IDX,                /*! Blood pressure service, intermediate cuff pressure characteristic */
  MEDS_BLP_NUM_CCC_IDX
};

/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/

/*! blood pressure measurement configuration */
static const blpsCfg_t medsBlpsCfg =
{
  2000      /*! Measurement timer expiration period in ms */
};

/**************************************************************************************************
  Advertising Data
**************************************************************************************************/

/*! Service UUID list */
static const uint8_t medsSvcUuidList[] =
{
  UINT16_TO_BYTES(ATT_UUID_BLOOD_PRESSURE_SERVICE),
  UINT16_TO_BYTES(ATT_UUID_DEVICE_INFO_SERVICE)
};

/**************************************************************************************************
  Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t medsBlpCccSet[MEDS_BLP_NUM_CCC_IDX] =
{
  /* cccd handle          value range               security level */
  {GATT_SC_CH_CCC_HDL,    ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_ENC},    /* MEDS_BLP_GATT_SC_CCC_IDX */
  {BPS_BPM_CH_CCC_HDL,    ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_ENC},    /* MEDS_BLP_BPS_BPM_CCC_IDX */
  {BPS_ICP_CH_CCC_HDL,    ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_ENC}     /* MEDS_BLP_BPS_ICP_CCC_IDX */
};

/**************************************************************************************************
  Local Functions
**************************************************************************************************/

static void medsBlpStart(void);
static void medsBlpProcMsg(wsfMsgHdr_t *pMsg);
static void medsBlpBtn(dmConnId_t connId, uint8_t btn);

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! profile interface pointer */
medsIf_t medsBlpIf =
{
  NULL,
  medsBlpStart,
  medsBlpProcMsg,
  medsBlpBtn
};

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! application control block */
static struct
{
  bool_t            measuring;
  bool_t            storedMeasurement;
} medsBlpCb;

static uint8_t medsBpmFlags = CH_BPM_FLAG_UNITS_MMHG | CH_BPM_FLAG_TIMESTAMP |
                              CH_BPM_FLAG_PULSE_RATE | CH_BPM_FLAG_MEAS_STATUS;
static uint8_t medsIcpFlags = CH_BPM_FLAG_UNITS_MMHG | CH_BPM_FLAG_PULSE_RATE |
                              CH_BPM_FLAG_MEAS_STATUS;

/*************************************************************************************************/
/*!
 *  \brief  Perform UI actions on connection close.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void medsBlpClose(wsfMsgHdr_t *pMsg)
{
  /* stop blood pressure measurement */
  BlpsMeasStop();
  medsBlpCb.measuring = FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void medsBlpStart(void)
{
  /* set up CCCD table and callback */
  AttsCccRegister(MEDS_BLP_NUM_CCC_IDX, (attsCccSet_t *) medsBlpCccSet, medsCccCback);

  /* add blood pressure service */
  SvcBpsAddGroup();

  /* Set Service Changed CCCD index. */
  GattSetSvcChangedIdx(MEDS_BLP_GATT_SC_CCC_IDX);

  /* initialize blood pressure profile sensor */
  BlpsInit(medsCb.handlerId, (blpsCfg_t *) &medsBlpsCfg);
  BlpsSetBpmFlags(medsBpmFlags);
  BlpsSetIcpFlags(medsIcpFlags);

  /* set advertising data */
  AppAdvSetAdValue(APP_ADV_DATA_DISCOVERABLE, DM_ADV_TYPE_16_UUID, sizeof(medsSvcUuidList),
                   (uint8_t *) medsSvcUuidList);

}

/*************************************************************************************************/
/*!
*  \brief  Send stored blood pressure measurement.
*
*  \param  connId    Connection ID to send stored measurement to.
*
*  \return None.
*/
/*************************************************************************************************/
static void medsBlpSendStoredMeasurement(dmConnId_t connId)
{
  medsBlpCb.storedMeasurement = FALSE;

  BlpsMeasComplete((dmConnId_t)connId, MEDS_BLP_BPS_BPM_CCC_IDX);
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
static void medsBlpProcMsg(wsfMsgHdr_t *pMsg)
{
  switch(pMsg->event)
  {
    case MEDS_TIMER_IND:
      BlpsProcMsg(pMsg);
      break;

    case ATTS_CCC_STATE_IND:
      /* Check if stored measurement exists and indications on measurements enabled */
      if (medsBlpCb.storedMeasurement &&
          AttsCccEnabled((dmConnId_t)pMsg->param, MEDS_BLP_BPS_BPM_CCC_IDX))
      {
        medsBlpSendStoredMeasurement((dmConnId_t)pMsg->param);
      }
      break;

    case DM_CONN_CLOSE_IND:
      medsBlpClose(pMsg);
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
static void medsBlpBtn(dmConnId_t connId, uint8_t btn)
{
  /* button actions when connected */
  if (connId != DM_CONN_ID_NONE)
  {
    switch (btn)
    {
      case APP_UI_BTN_1_SHORT:
        /* start or complete measurement */
        if (!medsBlpCb.measuring)
        {
          BlpsMeasStart(connId, MEDS_TIMER_IND, MEDS_BLP_BPS_ICP_CCC_IDX);
          medsBlpCb.measuring = TRUE;
        }
        else
        {
          BlpsMeasComplete(connId, MEDS_BLP_BPS_BPM_CCC_IDX);
          medsBlpCb.measuring = FALSE;
        }
        break;

      default:
        break;
    }
  }
  else /* if not connected */
  {
    switch (btn)
    {
      case APP_UI_BTN_1_EX_LONG:
        /* Toggle flags in service for testing purposes */
        SvcBpsToggleFeatureFlags(CH_BPF_FLAG_MULTI_BOND);
        break;

      case APP_UI_BTN_2_SHORT:
        /* Toggle flags in measurements for testing purposes */
        if (!(medsBpmFlags & CH_BPM_FLAG_USER_ID))
        {
          medsBpmFlags |= CH_BPM_FLAG_USER_ID;
        }
        else
        {
          medsBpmFlags &= ~CH_BPM_FLAG_USER_ID;
        }

        BlpsSetBpmFlags(medsBpmFlags);
        break;

      case APP_UI_BTN_2_MED:
        /* Toggle flags in intermediate measurements for testing purposes */
        if (!(medsIcpFlags & CH_BPM_FLAG_USER_ID))
        {
          medsIcpFlags |= CH_BPM_FLAG_USER_ID;
        }
        else
        {
          medsIcpFlags &= ~CH_BPM_FLAG_USER_ID;
        }

        BlpsSetIcpFlags(medsIcpFlags);
        break;

      case APP_UI_BTN_2_LONG:
        /* Toggle flags in intermediate measurements for testing purposes */
        if (!(medsIcpFlags & CH_BPM_FLAG_TIMESTAMP))
        {
          medsIcpFlags |= CH_BPM_FLAG_TIMESTAMP;
        }
        else
        {
          medsIcpFlags &= ~CH_BPM_FLAG_TIMESTAMP;
        }

        BlpsSetIcpFlags(medsIcpFlags);
        break;


      case APP_UI_BTN_2_EX_LONG:
        if (connId == DM_CONN_ID_NONE)
        {
          /* Store a measurement to be sent when connected */
          medsBlpCb.storedMeasurement = TRUE;
        }
        break;

      default:
        break;
    }
  }
}
