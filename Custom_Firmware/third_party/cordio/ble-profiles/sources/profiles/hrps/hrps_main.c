/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Heart Rate profile sensor.
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
#include "wsf_buf.h"
#include "wsf_trace.h"
#include "util/bstream.h"
#include "att_api.h"
#include "svc_ch.h"
#include "svc_hrs.h"
#include "app_api.h"
#include "app_hw.h"
#include "hrps_api.h"
#include "eatt_api.h"
#include "att_eatt.h"

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! \brief Connection control block */
typedef struct
{
  dmConnId_t    connId;               /*! \brief Connection ID */
  bool_t        hrmToSend;            /*! \brief heart rate measurement ready to be sent on this channel */
} hrpsConn_t;

/*! \brief Control block */
static struct
{
  hrpsConn_t    conn[DM_CONN_MAX];    /* \brief connection control block */
  wsfTimer_t    measTimer;            /* \brief periodic measurement timer */
  appHrm_t      hrm;                  /* \brief heart rate measurement */
  hrpsCfg_t     cfg;                  /* \brief configurable parameters */
  uint16_t      energyExp;            /* \brief energy expended value */
  bool_t        txReady;              /* \brief TRUE if ready to send notifications */
  uint8_t       flags;                /* \brief heart rate measurement flags */
} hrpsCb;

/*************************************************************************************************/
/*!
 *  \brief  Return TRUE if no connections with active measurements.
 *
 *  \return TRUE if no connections active.
 */
/*************************************************************************************************/
static bool_t hrpsNoConnActive(void)
{
  hrpsConn_t    *pConn = hrpsCb.conn;
  uint8_t       i;

  for (i = 0; i < DM_CONN_MAX; i++, pConn++)
  {
    if (pConn->connId != DM_CONN_ID_NONE)
    {
      return FALSE;
    }
  }
  return TRUE;
}

/*************************************************************************************************/
/*!
 *  \brief  Setup to send measurements on active connections.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hrpsSetupToSend(void)
{
  hrpsConn_t    *pConn = hrpsCb.conn;
  uint8_t       i;

  for (i = 0; i < DM_CONN_MAX; i++, pConn++)
  {
    if (pConn->connId != DM_CONN_ID_NONE)
    {
      pConn->hrmToSend = TRUE;
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Find next connection with measurement to send.
 *
 *  \param  cccIdx  Heart rate measurement CCC descriptor index.
 *
 *  \return Connection control block.
 */
/*************************************************************************************************/
static hrpsConn_t *hrpsFindNextToSend(uint8_t cccIdx)
{
  hrpsConn_t    *pConn = hrpsCb.conn;
  uint8_t       i;

  for (i = 0; i < DM_CONN_MAX; i++, pConn++)
  {
    if (pConn->connId != DM_CONN_ID_NONE && pConn->hrmToSend)
    {
      if (AttsCccEnabled(pConn->connId, cccIdx))
      {
        return pConn;
      }
    }
  }
  return NULL;
}

/*************************************************************************************************/
/*!
 *  \brief  Build a heart rate measurement characteristic.
 *
 *  \param  connId   DM connection identifier.
 *  \param  pBuf     Pointer to buffer to hold the built heart rate measurement characteristic.
 *  \param  pHrm     Heart rate measurement values.
 *
 *  \return Length of pBuf in bytes.
 */
/*************************************************************************************************/
static uint8_t hrpsBuildHrm(dmConnId_t connId, uint8_t **pBuf, appHrm_t *pHrm)
{
  uint8_t   *pHrpsData;
  uint8_t   flags = pHrm->flags;
  uint8_t   i;
  uint16_t  *pInterval;
  uint8_t   len = 2; /* Start with 2 for flags and 1 Byte Heart Rate measurement */
  uint8_t   maxLen = AttGetMtu(connId) - ATT_VALUE_NTF_LEN;

  /* Calculate Buffer length */
  if (flags & CH_HRM_FLAGS_VALUE_16BIT)
  {
    len += 1;
  }

  if (flags & CH_HRM_FLAGS_ENERGY_EXP)
  {
    len += 2;
  }

  /* rr interval */
  if (flags & CH_HRM_FLAGS_RR_INTERVAL)
  {
    len += pHrm->numIntervals * sizeof(uint16_t);
  }

  /* Adjust length if necessary */
  if (len > maxLen)
  {
    len = maxLen;
  }

  /* Allocate buffer */
  if ((*pBuf = (uint8_t *)WsfBufAlloc(len)) != NULL)
  {
    /* Add data to buffer */
    pHrpsData = *pBuf;

    /* flags */
    UINT8_TO_BSTREAM(pHrpsData, flags);

    /* Subtract 2 for flags and 1 Byte Heart Rate measurement */
    len -= 2;

    /* heart rate measurement */
    if (flags & CH_HRM_FLAGS_VALUE_16BIT)
    {
      UINT16_TO_BSTREAM(pHrpsData, (uint16_t)pHrm->heartRate);

      /* Subtract an additional byte for a 2 byte Heart Rate measurement */
      len -= 1;
    }
    else
    {
      UINT8_TO_BSTREAM(pHrpsData, pHrm->heartRate);
    }

    /* energy expended */
    if (flags & CH_HRM_FLAGS_ENERGY_EXP)
    {
      UINT16_TO_BSTREAM(pHrpsData, pHrm->energyExp);
      len -= 2;
    }

    /* rr interval */
    if (flags & CH_HRM_FLAGS_RR_INTERVAL)
    {
      pInterval = pHrm->pRrInterval;

      /* Use as many rr intervals as will fit in remaining buffer space. */
      i = pHrm->numIntervals < (len / sizeof(uint16_t)) ?
          pHrm->numIntervals : (len / sizeof(uint16_t));

      for (; i > 0; i--, pInterval++)
      {
        UINT16_TO_BSTREAM(pHrpsData, *pInterval);
      }
    }

    /* return length */
    return (uint8_t)(pHrpsData - *pBuf);
  }

  return 0;
}

/*************************************************************************************************/
/*!
 *  \brief  Send heart rate measurement notification
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hrpsSendHrmNtf(dmConnId_t connId)
{
  uint8_t *pBuf;
  uint8_t len;

  /* Build heart rate measurement characteristic */
  if ((len = hrpsBuildHrm(connId, &pBuf, &hrpsCb.hrm)) > 0)
  {
    #ifdef AM_BLE_EATT
    /* Send EATT notification */
    EattsHandleValueNtf(connId, eattCb.ccb->pChanCb->priority, HRS_HRM_HDL, len, pBuf);
    #else
    /* Send ATT notification */
    AttsHandleValueNtf(connId, HRS_HRM_HDL, len, pBuf);
    #endif

    /* Free allocated buffer */
    WsfBufFree(pBuf);
  }
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
static void hrpsConnOpen(dmEvt_t *pMsg)
{
  hrpsCb.txReady = TRUE;
}

/*************************************************************************************************/
/*!
 *  \brief  Handle a received ATT handle value confirm.
 *
 *  \param  pMsg     Event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hrpsHandleValueCnf(attEvt_t *pMsg)
{
  hrpsConn_t  *pConn;

  if (pMsg->hdr.status == ATT_SUCCESS && pMsg->handle == HRS_HRM_HDL)
  {
    hrpsCb.txReady = TRUE;

    /* find next connection to send (note ccc idx is stored in timer status) */
    if ((pConn = hrpsFindNextToSend(hrpsCb.measTimer.msg.status)) != NULL)
    {
      hrpsSendHrmNtf(pConn->connId);
      hrpsCb.txReady = FALSE;
      pConn->hrmToSend = FALSE;
    }
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
void hrpsMeasTimerExp(wsfMsgHdr_t *pMsg)
{
  hrpsConn_t  *pConn;

  /* if there are active connections */
  if (hrpsNoConnActive() == FALSE)
  {

    /* set up heart rate measurement to be sent on all connections */
    hrpsSetupToSend();

    /* read heart rate measurement sensor data */
    AppHwHrmRead(&hrpsCb.hrm);

    /* if ready to send measurements */
    if (hrpsCb.txReady)
    {
      /* find next connection to send (note ccc idx is stored in timer status) */
      if ((pConn = hrpsFindNextToSend(pMsg->status)) != NULL)
      {
        hrpsSendHrmNtf(pConn->connId);
        hrpsCb.txReady = FALSE;
        pConn->hrmToSend = FALSE;
      }
    }

    /* restart timer */
    WsfTimerStartMs(&hrpsCb.measTimer, hrpsCb.cfg.period);

    /* increment energy expended for test/demonstration purposes */
    hrpsCb.hrm.energyExp++;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize the Heart Rate profile sensor.
 *
 *  \param  handerId    WSF handler ID of the application using this service.
 *  \param  pCfg        Configurable parameters.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HrpsInit(wsfHandlerId_t handlerId, hrpsCfg_t *pCfg)
{
  hrpsCb.measTimer.handlerId = handlerId;
  hrpsCb.cfg = *pCfg;
}

/*************************************************************************************************/
/*!
 *  \brief  Start periodic heart rate measurement.  This function starts a timer to perform
 *          periodic measurements.
 *
 *  \param  connId      DM connection identifier.
 *  \param  timerEvt    WSF event designated by the application for the timer.
 *  \param  hrmCccIdx   Index of heart rate CCC descriptor in CCC descriptor handle table.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HrpsMeasStart(dmConnId_t connId, uint8_t timerEvt, uint8_t hrmCccIdx)
{
  /* if this is first connection */
  if (hrpsNoConnActive())
  {
    /* initialize control block */
    hrpsCb.measTimer.msg.event = timerEvt;
    hrpsCb.measTimer.msg.status = hrmCccIdx;

    /* start timer */
    WsfTimerStartMs(&hrpsCb.measTimer, hrpsCb.cfg.period);
  }

  /* set conn id */
  hrpsCb.conn[connId - 1].connId = connId;
}

/*************************************************************************************************/
/*!
 *  \brief  Stop periodic heart rate measurement.
 *
 *  \param  connId      DM connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HrpsMeasStop(dmConnId_t connId)
{
  /* clear connection */
  hrpsCb.conn[connId - 1].connId = DM_CONN_ID_NONE;
  hrpsCb.conn[connId - 1].hrmToSend = FALSE;

  /* if no remaining connections */
  if (hrpsNoConnActive())
  {
    /* stop timer */
    WsfTimerStop(&hrpsCb.measTimer);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Process received WSF message.
 *
 *  \param  pMsg     Event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HrpsProcMsg(wsfMsgHdr_t *pMsg)
{
  if (pMsg->event == DM_CONN_OPEN_IND)
  {
    hrpsConnOpen((dmEvt_t *) pMsg);
  }
  else if (pMsg->event == ATTS_HANDLE_VALUE_CNF)
  {
    hrpsHandleValueCnf((attEvt_t *) pMsg);
  }
  else if (pMsg->event == hrpsCb.measTimer.msg.event)
  {
    hrpsMeasTimerExp(pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  ATTS write callback for heart rate service.  Use this function as a parameter
 *          to SvcHrsCbackRegister().
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t HrpsWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                       uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr)
{
  if (*pValue == CH_HRCP_RESET_ENERGY_EXP)
  {
    /* reset energy expended */
    hrpsCb.hrm.energyExp = 0;
    return ATT_SUCCESS;
  }
  else
  {
    /* else unknown control point command */
    return HRS_ERR_CP_NOT_SUP;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Set the heart rate measurement flags.
 *
 *  \param  flags      Heart rate measurement flags.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HrpsSetFlags(uint8_t flags)
{
  hrpsCb.hrm.flags = flags;
}
