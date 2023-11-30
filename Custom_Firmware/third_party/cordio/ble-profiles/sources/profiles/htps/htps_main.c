/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Health Thermometer profile sensor.
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

#include "wsf_types.h"
#include "wsf_assert.h"
#include "wsf_trace.h"
#include "util/bstream.h"
#include "att_api.h"
#include "svc_ch.h"
#include "svc_hts.h"
#include "app_api.h"
#include "app_hw.h"
#include "htps_api.h"

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/* Control block */
static struct
{
  wsfTimer_t    measTimer;            /* periodic measurement timer */
  appTm_t       tm;                   /* temperature measurement */
  htpsCfg_t     cfg;                  /* configurable parameters */
  uint8_t       tmFlags;              /* temperature measurement flags */
  uint8_t       itFlags;              /* intermediate temperature flags */
} htpsCb;

/*************************************************************************************************/
/*!
 *  \brief  Build a temperature measurement characteristic.
 *
 *  \param  pBuf    Pointer to buffer to hold the built temperature measurement characteristic.
 *  \param  pTm     Temperature measurement values.
 *
 *  \return Length of pBuf in bytes.
 */
/*************************************************************************************************/
static uint8_t htpsBuildTm(uint8_t *pBuf, appTm_t *pTm)
{
  uint8_t   *p = pBuf;
  uint8_t   flags = pTm->flags;

  /* flags */
  UINT8_TO_BSTREAM(p, flags);

  /* measurement */
  UINT32_TO_BSTREAM(p, pTm->temperature);

  /* time stamp */
  if (flags & CH_TM_FLAG_TIMESTAMP)
  {
    UINT16_TO_BSTREAM(p, pTm->timestamp.year);
    UINT8_TO_BSTREAM(p, pTm->timestamp.month);
    UINT8_TO_BSTREAM(p, pTm->timestamp.day);
    UINT8_TO_BSTREAM(p, pTm->timestamp.hour);
    UINT8_TO_BSTREAM(p, pTm->timestamp.min);
    UINT8_TO_BSTREAM(p, pTm->timestamp.sec);
  }

  /* temperature type */
  if (flags & CH_TM_FLAG_TEMP_TYPE)
  {
    UINT8_TO_BSTREAM(p, pTm->tempType);
  }

  /* return length */
  return (uint8_t) (p - pBuf);
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize the Health Thermometer profile sensor.
 *
 *  \param  handerId    WSF handler ID of the application using this service.
 *  \param  pCfg        Configurable parameters.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HtpsInit(wsfHandlerId_t handlerId, htpsCfg_t *pCfg)
{
  htpsCb.measTimer.handlerId = handlerId;
  htpsCb.cfg = *pCfg;
}

/*************************************************************************************************/
/*!
 *  \brief  Start periodic temperature measurement.  This function starts a timer to perform
 *          periodic measurements.
 *
 *  \param  connId      DM connection identifier.
 *  \param  timerEvt    WSF event designated by the application for the timer.
 *  \param  itCccIdx    Index of intermediate temperature CCC descriptor in CCC descriptor
 *                      handle table.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HtpsMeasStart(dmConnId_t connId, uint8_t timerEvt, uint8_t itCccIdx)
{
  /* initialize control block */
  htpsCb.measTimer.msg.param = connId;
  htpsCb.measTimer.msg.event = timerEvt;
  htpsCb.measTimer.msg.status = itCccIdx;

  /* start timer */
  WsfTimerStartMs(&htpsCb.measTimer, htpsCb.cfg.period);
}

/*************************************************************************************************/
/*!
 *  \brief  Stop periodic temperature measurement.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HtpsMeasStop(void)
{
  /* stop timer */
  WsfTimerStop(&htpsCb.measTimer);
}

/*************************************************************************************************/
/*!
 *  \brief  Temperature measurement complete.
 *
 *  \param  connId      DM connection identifier.
 *  \param  tmCccIdx    Index of temperature measurement CCC descriptor in CCC descriptor
 *                      handle table.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HtpsMeasComplete(dmConnId_t connId, uint8_t tmCccIdx)
{
  uint8_t buf[ATT_DEFAULT_PAYLOAD_LEN];
  uint8_t len;

  /* stop periodic measurement */
  HtpsMeasStop();

  /* if indications enabled  */
  if (AttsCccEnabled(connId, tmCccIdx))
  {
    /* read temperature measurement sensor data */
    AppHwTmRead(FALSE, &htpsCb.tm);

    /* set flags */
    htpsCb.tm.flags = htpsCb.tmFlags;

    /* build temperature measurement characteristic */
    len = htpsBuildTm(buf, &htpsCb.tm);

    /* send temperature measurement indication */
    AttsHandleValueInd(connId, HTS_TM_HDL, len, buf);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  This function is called by the application when the periodic measurement
 *          timer expires.
 *
 *  \param  pMsg     Event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HtpsProcMsg(wsfMsgHdr_t *pMsg)
{
  uint8_t buf[ATT_DEFAULT_PAYLOAD_LEN];
  uint8_t len;

  /* if notifications enabled (note ccc idx is stored in hdr.status) */
  if (AttsCccEnabled((dmConnId_t) pMsg->param, pMsg->status))
  {
    /* read temperature measurement sensor data */
    AppHwTmRead(TRUE, &htpsCb.tm);

    /* set flags */
    htpsCb.tm.flags = htpsCb.itFlags;

    /* build temperature measurement characteristic */
    len = htpsBuildTm(buf, &htpsCb.tm);

    /* send intermediate temperature notification */
    AttsHandleValueNtf((dmConnId_t) pMsg->param, HTS_IT_HDL, len, buf);

    /* restart timer */
    WsfTimerStartMs(&htpsCb.measTimer, htpsCb.cfg.period);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Set the temperature measurement flags.
 *
 *  \param  flags      Temperature measurement flags.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HtpsSetTmFlags(uint8_t flags)
{
  htpsCb.tmFlags = flags;
}

/*************************************************************************************************/
/*!
 *  \brief  Set the intermediate temperature flags.
 *
 *  \param  flags      Intermediate temperature flags.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HtpsSetItFlags(uint8_t flags)
{
  htpsCb.itFlags = flags;
}
