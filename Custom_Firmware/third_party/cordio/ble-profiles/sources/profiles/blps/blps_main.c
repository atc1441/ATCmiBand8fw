/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Blood Pressure profile sensor.
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
#include "svc_bps.h"
#include "app_api.h"
#include "app_hw.h"
#include "blps_api.h"

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/* Control block */
static struct
{
  wsfTimer_t    measTimer;            /* periodic measurement timer */
  appBpm_t      bpm;                  /* blood pressure measurement */
  blpsCfg_t     cfg;                  /* configurable parameters */
  uint8_t       bpmFlags;             /* blood pressure measurement flags */
  uint8_t       icpFlags;             /* intermediate cuff pressure flags */
} blpsCb;

/*************************************************************************************************/
/*!
 *  \brief  Build a blood pressure measurement characteristic.
 *
 *  \param  pBuf     Pointer to buffer to hold the built blood pressure measurement characteristic.
 *  \param  pBpm     Blood pressure measurement values.
 *
 *  \return Length of pBuf in bytes.
 */
/*************************************************************************************************/
static uint8_t blpsBuildBpm(uint8_t *pBuf, appBpm_t *pBpm)
{
  uint8_t   *p = pBuf;
  uint8_t   flags = pBpm->flags;

  /* flags */
  UINT8_TO_BSTREAM(p, flags);

  /* measurement */
  UINT16_TO_BSTREAM(p, pBpm->systolic);
  UINT16_TO_BSTREAM(p, pBpm->diastolic);
  UINT16_TO_BSTREAM(p, pBpm->map);

  /* time stamp */
  if (flags & CH_BPM_FLAG_TIMESTAMP)
  {
    UINT16_TO_BSTREAM(p, pBpm->timestamp.year);
    UINT8_TO_BSTREAM(p, pBpm->timestamp.month);
    UINT8_TO_BSTREAM(p, pBpm->timestamp.day);
    UINT8_TO_BSTREAM(p, pBpm->timestamp.hour);
    UINT8_TO_BSTREAM(p, pBpm->timestamp.min);
    UINT8_TO_BSTREAM(p, pBpm->timestamp.sec);
  }

  /* pulse rate */
  if (flags & CH_BPM_FLAG_PULSE_RATE)
  {
    UINT16_TO_BSTREAM(p, pBpm->pulseRate);
  }

  /* user id */
  if (flags & CH_BPM_FLAG_USER_ID)
  {
    UINT8_TO_BSTREAM(p, pBpm->userId);
  }

  /* measurement status */
  if (flags & CH_BPM_FLAG_MEAS_STATUS)
  {
    UINT16_TO_BSTREAM(p, pBpm->measStatus);
  }

  /* return length */
  return (uint8_t) (p - pBuf);
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize the Blood Pressure profile sensor.
 *
 *  \param  handerId    WSF handler ID of the application using this service.
 *  \param  pCfg        Configurable parameters.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BlpsInit(wsfHandlerId_t handlerId, blpsCfg_t *pCfg)
{
  blpsCb.measTimer.handlerId = handlerId;
  blpsCb.cfg = *pCfg;
}

/*************************************************************************************************/
/*!
 *  \brief  Start periodic blood pressure measurement.  This function starts a timer to perform
 *          periodic measurements.
 *
 *  \param  connId      DM connection identifier.
 *  \param  timerEvt    WSF event designated by the application for the timer.
 *  \param  icpCccIdx   Index of intermediate cuff pressure CCC descriptor in CCC descriptor
 *                      handle table.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BlpsMeasStart(dmConnId_t connId, uint8_t timerEvt, uint8_t icpCccIdx)
{
  /* initialize control block */
  blpsCb.measTimer.msg.param = connId;
  blpsCb.measTimer.msg.event = timerEvt;
  blpsCb.measTimer.msg.status = icpCccIdx;

  /* start timer */
  WsfTimerStartMs(&blpsCb.measTimer, blpsCb.cfg.period);
}

/*************************************************************************************************/
/*!
 *  \brief  Stop periodic blood pressure measurement.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BlpsMeasStop(void)
{
  /* stop timer */
  WsfTimerStop(&blpsCb.measTimer);
}

/*************************************************************************************************/
/*!
 *  \brief  Blood pressure measurement complete.
 *
 *  \param  connId      DM connection identifier.
 *  \param  bpmCccIdx   Index of blood pressure measurement CCC descriptor in CCC descriptor
 *                      handle table.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BlpsMeasComplete(dmConnId_t connId, uint8_t bpmCccIdx)
{
  uint8_t buf[ATT_DEFAULT_PAYLOAD_LEN];
  uint8_t len;

  /* stop periodic measurement */
  BlpsMeasStop();

  /* if indications enabled  */
  if (AttsCccEnabled(connId, bpmCccIdx))
  {
    /* read blood pressure measurement sensor data */
    AppHwBpmRead(FALSE, &blpsCb.bpm);

    /* set flags */
    blpsCb.bpm.flags = blpsCb.bpmFlags;

    /* build blood pressure measurement characteristic */
    len = blpsBuildBpm(buf, &blpsCb.bpm);

    /* send blood pressure measurement indication */
    AttsHandleValueInd(connId, BPS_BPM_HDL, len, buf);
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
void BlpsProcMsg(wsfMsgHdr_t *pMsg)
{
  uint8_t buf[ATT_DEFAULT_PAYLOAD_LEN];
  uint8_t len;

  /* if notifications enabled (note ccc idx is stored in hdr.status) */
  if (AttsCccEnabled((dmConnId_t) pMsg->param, pMsg->status))
  {
    /* read blood pressure measurement sensor data */
    AppHwBpmRead(TRUE, &blpsCb.bpm);

    /* set flags */
    blpsCb.bpm.flags = blpsCb.icpFlags;

    /* build blood pressure measurement characteristic */
    len = blpsBuildBpm(buf, &blpsCb.bpm);

    /* send intermediate cuff pressure notification */
    AttsHandleValueNtf((dmConnId_t) pMsg->param, BPS_ICP_HDL, len, buf);

    /* restart timer */
    WsfTimerStartMs(&blpsCb.measTimer, blpsCb.cfg.period);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Set the blood pressure measurement flags.
 *
 *  \param  flags      Blood pressure measurement flags.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BlpsSetBpmFlags(uint8_t flags)
{
  blpsCb.bpmFlags = flags;
}

/*************************************************************************************************/
/*!
 *  \brief  Set the intermediate cuff pressure flags.
 *
 *  \param  flags      Intermediate cuff pressure flags.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BlpsSetIcpFlags(uint8_t flags)
{
  blpsCb.icpFlags = flags;
}
