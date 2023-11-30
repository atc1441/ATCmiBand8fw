/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Pulse Oximeter profile sensor.
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
#include "wsf_assert.h"
#include "wsf_trace.h"
#include "util/bstream.h"
#include "att_api.h"
#include "svc_ch.h"
#include "svc_plxs.h"
#include "app_api.h"
#include "app_ui.h"
#include "plxps_api.h"
#include "plxps_main.h"

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/* Control block */
static struct
{
  wsfTimer_t    measTimer;                    /* continuous measurement timer */
  plxpsCfg_t    *pCfg;                        /* configurable parameters */
  plxpsRec_t    *pCurrRec;                    /* Pointer to current measurement record */
  plxpCm_t      plxpsCm;                      /* Continuous measurement data */
  bool_t        inProgress;                   /* TRUE if RACP procedure in progress */
  bool_t        cmTxPending;                  /* TRUE if Continuous Measurement tx pending */
  bool_t        txReady;                      /* TRUE if ready to send next notification or indication */
  bool_t        aborting;                     /* TRUE if abort procedure in progress */
  uint8_t       plxscCccIdx;                  /* Pulse Oximeter spot check measurement CCCD index */
  uint8_t       plxcCccIdx;                   /* Pulse Oximeter continuous measurement CCCD index */
  uint8_t       racpCccIdx;                   /* Record access control point CCCD index */
} plxpsCb;

/*************************************************************************************************/
/*!
 *  \brief  Build a spot check measurement characteristic.
 *
 *  \param  pBuf     Pointer to buffer to hold the built characteristic.
 *  \param  pScm     Pulse Oximeter spot check measurement values.
 *
 *  \return Length of pBuf in bytes.
 */
/*************************************************************************************************/
static uint8_t plxpsBuildScm(uint8_t *pBuf, plxpScm_t *pScm)
{
  uint8_t   *p = pBuf;
  uint8_t   flags = pScm->flags;

  /* flags */
  UINT8_TO_BSTREAM(p, flags);

  /* Manditory SpO2 */
  UINT16_TO_BSTREAM(p, pScm->spo2);

  /* Manditory Pulse Rate */
  UINT16_TO_BSTREAM(p, pScm->pulseRate);

  /* Timestamp */
  if (flags & CH_PLXSC_FLAG_TIMESTAMP)
  {
    UINT16_TO_BSTREAM(p, pScm->timestamp.year);
    UINT8_TO_BSTREAM(p, pScm->timestamp.month);
    UINT8_TO_BSTREAM(p, pScm->timestamp.day);
    UINT8_TO_BSTREAM(p, pScm->timestamp.hour);
    UINT8_TO_BSTREAM(p, pScm->timestamp.min);
    UINT8_TO_BSTREAM(p, pScm->timestamp.sec);
  }

  /* Measurement Status */
  if (flags & CH_PLXSC_FLAG_MEASUREMENT_STATUS)
  {
    UINT16_TO_BSTREAM(p, pScm->measStatus);
  }

  /* Device and Sensor Status */
  if (flags & CH_PLXSC_FLAG_SENSOR_STATUS)
  {
    UINT24_TO_BSTREAM(p, pScm->sensorStatus);
  }

  /* Pulse Amplitude Index */
  if (flags & CH_PLXSC_FLAG_PULSE_AMP_INDX)
  {
    UINT16_TO_BSTREAM(p, pScm->pulseAmpIndex);
  }

  /* return length */
  return (uint8_t) (p - pBuf);
}

/*************************************************************************************************/
/*!
 *  \brief  Build a Continuous measurement characteristic.
 *
 *  \param  pBuf     Pointer to buffer to hold the built characteristic.
 *  \param  pCm      Pulse Oximeter continuous measurement values.
 *
 *  \return Length of pBuf in bytes.
 */
/*************************************************************************************************/
static uint8_t plxpsBuildCm(uint8_t *pBuf, plxpCm_t *pCm)
{
  uint8_t   *p = pBuf;
  uint8_t   flags = pCm->flags;

  /* flags */
  UINT8_TO_BSTREAM(p, flags);

  /* Manditory SpO2 */
  UINT16_TO_BSTREAM(p, pCm->spo2);

  /* Manditory Pulse Rate */
  UINT16_TO_BSTREAM(p, pCm->pulseRate);

  /* SpO2PR Fast */
  if (flags & CH_PLXC_FLAG_SPO2PR_FAST)
  {
    UINT16_TO_BSTREAM(p, pCm->spo2Fast);
    UINT16_TO_BSTREAM(p, pCm->pulseRateFast);
  }

  /* SpO2PR Slow */
  if (flags & CH_PLXC_FLAG_SPO2PR_SLOW)
  {
    UINT16_TO_BSTREAM(p, pCm->spo2Slow);
    UINT16_TO_BSTREAM(p, pCm->pulseRateSlow);
  }

  /* Measurement Status */
  if (flags & CH_PLXC_FLAG_MEASUREMENT_STATUS)
  {
    UINT16_TO_BSTREAM(p, pCm->measStatus);
  }

  /* Device and Sensor Status */
  if (flags & CH_PLXC_FLAG_SENSOR_STATUS)
  {
    UINT24_TO_BSTREAM(p, pCm->sensorStatus);
  }

  /* Pulse Amplitude Index */
  if (flags & CH_PLXC_FLAG_PULSE_AMP_INDX)
  {
    UINT16_TO_BSTREAM(p, pCm->pulseAmpIndex);
  }


  /* return length */
  return (uint8_t) (p - pBuf);
}

/*************************************************************************************************/
/*!
 *  \brief  Send a Continuous measurement notification.
 *
 *  \param  connId      Connection ID.
 *  \param  pMeas       Pointer to Pulse Oximiter continuous measurement.
 *
 *  \return None.
 */
/*************************************************************************************************/
void plxpsSendContinuousMeas(dmConnId_t connId, plxpCm_t *pMeas)
{
  uint8_t buf[ATT_DEFAULT_PAYLOAD_LEN];
  uint8_t len;

  /* build continuous measurement characteristic */
  len = plxpsBuildCm(buf, pMeas);

  /* send notification */
  AttsHandleValueNtf(connId, PLXS_CONTINUOUS_HDL, len, buf);
  plxpsCb.txReady = FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Send a spot check measurement indication.
 *
 *  \param  connId      Connection ID.
 *  \param  pMeas       Pointer to pulse oximeter spot check measurement.
 *
 *  \return None.
 */
/*************************************************************************************************/
void plxpsSendSpotCheckMeas(dmConnId_t connId, plxpScm_t *pMeas)
{
  uint8_t buf[ATT_DEFAULT_PAYLOAD_LEN];
  uint8_t len;

  /* build spot check measurement characteristic */
  len = plxpsBuildScm(buf, pMeas);

  /* send indication */
  AttsHandleValueInd(connId, PLXS_SPOT_CHECK_HDL, len, buf);
  plxpsCb.txReady = FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Send a RACP response indication.
 *
 *  \param  connId      Connection ID.
 *  \param  opcode      RACP opcode.
 *  \param  status      RACP status.
 *
 *  \return None.
 */
/*************************************************************************************************/
void plxpsRacpSendRsp(dmConnId_t connId, uint8_t opcode, uint8_t status)
{
  uint8_t buf[PLXPS_RACP_RSP_LEN];

  /* build response */
  buf[0] = CH_RACP_OPCODE_RSP;
  buf[1] = CH_RACP_OPERATOR_NULL;
  buf[2] = opcode;
  buf[3] = status;

  /* send indication */
  AttsHandleValueInd(connId, PLXS_RECORD_ACCESS_HDL, PLXPS_RACP_RSP_LEN, buf);
  plxpsCb.txReady = FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Send a RACP number of records response indication.
 *
 *  \param  connId      Connection ID.
 *  \param  numRec      Number of records.
 *
 *  \return None.
 */
/*************************************************************************************************/
void plxpsRacpSendNumRecRsp(dmConnId_t connId, uint16_t numRec)
{
  uint8_t buf[PLXPS_RACP_NUM_REC_RSP_LEN];

  /* build response */
  buf[0] = CH_RACP_OPCODE_NUM_RSP;
  buf[1] = CH_RACP_OPERATOR_NULL;
  buf[2] = UINT16_TO_BYTE0(numRec);
  buf[3] = UINT16_TO_BYTE1(numRec);

  /* send indication */
  AttsHandleValueInd(connId, PLXS_RECORD_ACCESS_HDL, PLXPS_RACP_RSP_LEN, buf);
  plxpsCb.txReady = FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Handle connection open.
 *
 *  \param  pMsg     Event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void plxpsConnOpen(dmEvt_t *pMsg)
{
  /* initialize */
  plxpsCb.pCurrRec = NULL;
  plxpsCb.aborting = FALSE;
  plxpsCb.inProgress = FALSE;
  plxpsCb.txReady = FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Handle connection close.
 *
 *  \param  pMsg     Event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void plxpsConnClose(dmEvt_t *pMsg)
{
  plxpsCb.pCurrRec = NULL;
  plxpsCb.aborting = FALSE;

  PlxpsMeasStop();
}

/*************************************************************************************************/
/*!
 *  \brief  Handle an ATT handle value confirm.
 *
 *  \param  pMsg     Event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void plxpsHandleValueCnf(attEvt_t *pMsg)
{
  dmConnId_t connId = (dmConnId_t) pMsg->hdr.param;
  plxpsCb.txReady = TRUE;

  /* send continuous measurement if necessary */
  if (plxpsCb.cmTxPending == TRUE)
  {
    plxpsSendContinuousMeas((dmConnId_t) plxpsCb.measTimer.msg.param, &plxpsCb.plxpsCm);
    plxpsCb.txReady = FALSE;
    plxpsCb.cmTxPending = FALSE;
    return;
  }

  /* if aborting finish that up */
  if (plxpsCb.aborting)
  {
    plxpsCb.aborting = FALSE;
    plxpsRacpSendRsp(connId, CH_RACP_OPCODE_ABORT, CH_RACP_RSP_SUCCESS);
  }

  /* if this is for RACP indication */
  if (pMsg->handle == PLXS_RECORD_ACCESS_HDL)
  {
    /* procedure no longer in progress */
    plxpsCb.inProgress = FALSE;
  }
  /* if this is for measurement or continuous notification */
  else if (pMsg->handle == PLXS_SPOT_CHECK_HDL || pMsg->handle == PLXS_CONTINUOUS_HDL)
  {
    if (plxpsCb.pCurrRec != NULL)
    {
      /* if there is another record */
      if (plxpsDbGetNextRecord(CH_RACP_OPERATOR_ALL, plxpsCb.pCurrRec, &plxpsCb.pCurrRec) == CH_RACP_RSP_SUCCESS)
      {
        /* send measurement */
        plxpsSendSpotCheckMeas(connId, (plxpScm_t*) plxpsCb.pCurrRec);
      }
      /* else all records sent; send RACP response */
      else
      {
        plxpsRacpSendRsp(connId, CH_RACP_OPCODE_REPORT, CH_RACP_RSP_SUCCESS);
      }
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Handle a RACP report stored records operation.
 *
 *  \param  connId      Connection ID.
 *  \param  oper        Operator.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void plxpsRacpReport(dmConnId_t connId, uint8_t oper)
{
  uint8_t status;

  /* if record found */
  if ((status = plxpsDbGetNextRecord(oper, NULL, &plxpsCb.pCurrRec)) == CH_RACP_RSP_SUCCESS)
  {
    /* send spot check measurement */
    plxpsSendSpotCheckMeas(connId, (plxpScm_t *) plxpsCb.pCurrRec);
  }
  /* if not successful send response */
  else
  {
    plxpsRacpSendRsp(connId, CH_RACP_OPCODE_REPORT, status);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Handle a RACP delete records operation.
 *
 *  \param  connId      Connection ID.
 *  \param  oper        Operator.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void plxpsRacpDelete(dmConnId_t connId, uint8_t oper)
{
  uint8_t status;

  /* delete records */
  status = plxpsDbDeleteRecords(oper);

  /* send response */
  plxpsRacpSendRsp(connId, CH_RACP_OPCODE_DELETE, status);
}

/*************************************************************************************************/
/*!
 *  \brief  Handle a RACP abort operation.
 *
 *  \param  connId      Connection ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void plxpsRacpAbort(dmConnId_t connId)
{
  /* if operation in progress */
  if (plxpsCb.inProgress)
  {
    /* abort operation and clean up */
    plxpsCb.pCurrRec = NULL;
  }

  /* send response */
  if (plxpsCb.txReady)
  {
    plxpsRacpSendRsp(connId, CH_RACP_OPCODE_ABORT, CH_RACP_RSP_SUCCESS);
  }
  else
  {
    plxpsCb.aborting = TRUE;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Handle a RACP report number of stored records operation.
 *
 *  \param  connId      Connection ID.
 *  \param  oper        Operator.
 *  \param  pOperand    Operand data.

 *
 *  \return None.
 */
/*************************************************************************************************/
static void plxpsRacpReportNum(dmConnId_t connId, uint8_t oper)
{
  uint8_t status;
  uint8_t numRec;

  /* get number of records */
  status = plxpsDbGetNumRecords(oper, &numRec);

  if (status == CH_RACP_RSP_SUCCESS)
  {
    /* send response */
    plxpsRacpSendNumRecRsp(connId, numRec);
  }
  else
  {
    plxpsRacpSendRsp(connId, CH_RACP_OPCODE_REPORT_NUM, status);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize the Pulse Oximeter profile sensor.
 *
 *  \return None.
 */
/*************************************************************************************************/
void PlxpsInit(wsfHandlerId_t handlerId, plxpsCfg_t *pCfg)
{
  plxpsCb.measTimer.handlerId = handlerId;
  plxpsCb.pCfg = pCfg;
  plxpsDbInit();
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
void plxpsMeasTimerExp(wsfMsgHdr_t *pMsg)
{
  /* read pulse oximeter measurement data */
  AppHwPlxcmRead(&plxpsCb.plxpsCm);

  plxpsCb.cmTxPending = TRUE;

  /* if ready to send measurements */
  if (plxpsCb.txReady)
  {
    plxpsSendContinuousMeas((dmConnId_t) plxpsCb.measTimer.msg.param, &plxpsCb.plxpsCm);
    plxpsCb.txReady = FALSE;
    plxpsCb.cmTxPending = FALSE;
  }

  /* restart timer */
  WsfTimerStartMs(&plxpsCb.measTimer, plxpsCb.pCfg->period);
}

/*************************************************************************************************/
/*!
 *  \brief  This function is called by the application when a message that requires
 *          processing by the pulse oximeter profile sensor is received.
 *
 *  \param  pMsg     Event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void PlxpsProcMsg(wsfMsgHdr_t *pMsg)
{
  if (pMsg->event == plxpsCb.measTimer.msg.event)
  {
    plxpsMeasTimerExp(pMsg);
  }
  else
  {
    switch(pMsg->event)
    {
      case DM_CONN_OPEN_IND:
        plxpsConnOpen((dmEvt_t *) pMsg);
        break;

      case DM_CONN_CLOSE_IND:
        plxpsConnClose((dmEvt_t *) pMsg);
        break;

      case ATTS_HANDLE_VALUE_CNF:
        plxpsHandleValueCnf((attEvt_t *) pMsg);
        break;

      default:
        break;
    }
  }
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
void PlxpsBtn(dmConnId_t connId, uint8_t btn)
{
  /* button actions when connected */
  if (connId != DM_CONN_ID_NONE)
  {
    switch (btn)
    {
      case APP_UI_BTN_2_LONG:
        /* generate a new record */
        plxpsDbGenerateRecord();
        break;

      default:
        break;
    }
  }
  /* button actions when not connected */
  else
  {
    switch (btn)
    {
      case APP_UI_BTN_2_LONG:
        /* generate a new record */
        plxpsDbGenerateRecord();
        break;

      case APP_UI_BTN_2_EX_LONG:
        /* delete all records */
        plxpsDbDeleteRecords(CH_RACP_OPERATOR_ALL);
        break;

      default:
        break;
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  ATTS write callback for pulse oximeter service record access control point.  Use this
 *          function as a parameter to SvcPlxCbackRegister().
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t PlxpsWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                        uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr)
{
  uint8_t opcode;
  uint8_t oprator;

  /* sanity check on length */
  if (len < PLXPS_RACP_MIN_WRITE_LEN)
  {
    return ATT_ERR_LENGTH;
  }

  /* if control point not configured for indication */
  if (!AttsCccEnabled(connId, plxpsCb.racpCccIdx))
  {
    return ATT_ERR_CCCD;
  }

  /* parse opcode and operator and adjust remaining parameter length */
  BSTREAM_TO_UINT8(opcode, pValue);
  BSTREAM_TO_UINT8(oprator, pValue);
  len -= 2;

  /* handle a procedure in progress */
  if (opcode != CH_RACP_OPCODE_ABORT && plxpsCb.inProgress)
  {
    return ATT_ERR_IN_PROGRESS;
  }

  /* handle record request when notifications not enabled */
  if (opcode == CH_RACP_OPCODE_REPORT && !AttsCccEnabled(connId, plxpsCb.plxscCccIdx))
  {
    return ATT_ERR_CCCD;
  }

  /* verify opcode */
  if (opcode < CH_RACP_OPCODE_REPORT || opcode > CH_RACP_OPCODE_RSP)
  {
    plxpsRacpSendRsp(connId, opcode, CH_RACP_RSP_OPCODE_NOT_SUP);
  }

  /* verify operator */
  if ((opcode != CH_RACP_OPCODE_ABORT && oprator != CH_RACP_OPERATOR_ALL) ||
      (opcode == CH_RACP_OPCODE_ABORT && oprator != CH_RACP_OPERATOR_NULL))
  {
    if (oprator > CH_RACP_OPERATOR_LAST)
      plxpsRacpSendRsp(connId, opcode, CH_RACP_RSP_OPERATOR_NOT_SUP);
    else
      plxpsRacpSendRsp(connId, opcode, CH_RACP_RSP_INV_OPERATOR);

    return ATT_SUCCESS;
  }

  /* verify operands */
  if (len > 0)
  {
    plxpsRacpSendRsp(connId, opcode, CH_RACP_RSP_INV_OPERAND);
    return ATT_SUCCESS;
  }

  switch (opcode)
  {
    /* report records */
    case CH_RACP_OPCODE_REPORT:
      plxpsRacpReport(connId, oprator);
      break;

    /* delete records */
    case CH_RACP_OPCODE_DELETE:
      plxpsRacpDelete(connId, oprator);
      break;

    /* abort current operation */
    case CH_RACP_OPCODE_ABORT:
      plxpsRacpAbort(connId);
      break;

    /* report number of records */
    case CH_RACP_OPCODE_REPORT_NUM:
      plxpsRacpReportNum(connId, oprator);
      break;

    /* unsupported opcode */
    default:
      plxpsRacpSendRsp(connId, opcode, CH_RACP_RSP_OPCODE_NOT_SUP);
      break;
  }

  /* procedure now in progress */
  plxpsCb.inProgress = TRUE;

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Set the supported features of the pulse oximeter sensor.
 *
 *  \param  feature       Feature bitmask.
 *  \param  measStatus    Measurement status.
 *  \param  sensorStatus  Sensor status.
 *
 *  \return None.
 */
/*************************************************************************************************/
void PlxpsSetFeature(uint16_t feature, uint16_t measStatus, uint32_t sensorStatus)
{
  uint8_t buf[CH_PLXF_MAX_FEATURES_LEN];
  uint8_t *p = buf;

  UINT16_TO_BSTREAM(p, feature);

  if (feature & CH_PLF_FLAG_MEAS_STATUS_SUP)
  {
    UINT16_TO_BSTREAM(p, measStatus);
  }

  if (feature & CH_PLF_FLAG_SENSOR_STATUS_SUP)
  {
    UINT24_TO_BSTREAM(p, sensorStatus);
  }

  AttsSetAttr(PLXS_FEATURES_HDL, (uint16_t)(p - buf), buf);
}

/*************************************************************************************************/
/*!
 *  \brief  Set the CCCD index used by the application for pulse oximeter service characteristics.
 *
 *  \param  plxscCccIdx Pulse Oximeter spot check CCCD index.
 *  \param  plxcCccIdx  Pulse Oximeter continuous CCCD index.
 *  \param  racpCccIdx  Record access control point CCCD index.
 *
 *  \return None.
 */
/*************************************************************************************************/
void PlxpsSetCccIdx(uint8_t plxscCccIdx, uint8_t plxcCccIdx, uint8_t racpCccIdx)
{
  plxpsCb.plxscCccIdx = plxscCccIdx;
  plxpsCb.plxcCccIdx = plxcCccIdx;
  plxpsCb.racpCccIdx = racpCccIdx;
}

/*************************************************************************************************/
/*!
 *  \brief  Start periodic pulse oximeter  measurement.  This function starts a timer to perform
 *          periodic measurements.
 *
 *  \param  connId      DM connection identifier.
 *  \param  timerEvt    WSF event designated by the application for the timer.
 *  \param  plxmCccIdx  Index of pulse oximeter  CCC descriptor in CCC descriptor handle table.
 *
 *  \return None.
 */
/*************************************************************************************************/
void PlxpsMeasStart(dmConnId_t connId, uint8_t timerEvt, uint8_t plxmCccIdx)
{
  /* initialize control block */
  plxpsCb.measTimer.msg.param = connId;
  plxpsCb.measTimer.msg.event = timerEvt;
  plxpsCb.measTimer.msg.status = plxmCccIdx;

  /* start timer */
  WsfTimerStartMs(&plxpsCb.measTimer, plxpsCb.pCfg->period);
}

/*************************************************************************************************/
/*!
 *  \brief  Stop periodic pulse oximeter  measurement.
 *
 *  \return None.
 */
/*************************************************************************************************/
void PlxpsMeasStop(void)
{
  WsfTimerStop(&plxpsCb.measTimer);
}

