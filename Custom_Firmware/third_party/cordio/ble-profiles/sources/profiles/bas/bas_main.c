/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Battery service server.
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
#include "svc_batt.h"
#include "app_api.h"
#include "app_hw.h"
#include "bas_api.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief battery level initialization value */
#define BAS_BATT_LEVEL_INIT           0xFF

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! \brief Connection control block */
typedef struct
{
  dmConnId_t    connId;               /*! \brief Connection ID */
  bool_t        battToSend;           /*! \brief battery measurement ready to be sent on this channel */
  uint8_t       sentBattLevel;        /*! \brief value of last sent battery level */
} basConn_t;

/*! \brief Control block */
static struct
{
  basConn_t         conn[DM_CONN_MAX];    /*! \brief connection control block */
  wsfTimer_t        measTimer;            /*! \brief periodic measurement timer */
  basCfg_t          cfg;                  /*! \brief configurable parameters */
  uint16_t          currCount;            /*! \brief current measurement period count */
  bool_t            txReady;              /*! \brief TRUE if ready to send notifications */
  uint8_t           measBattLevel;        /*! \brief value of last measured battery level */
} basCb;

/*************************************************************************************************/
/*!
 *  \brief  Return TRUE if no connections with active measurements.
 *
 *  \return TRUE if no connections active.
 */
/*************************************************************************************************/
static bool_t basNoConnActive(void)
{
  basConn_t     *pConn = basCb.conn;
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
static void basSetupToSend(void)
{
  basConn_t     *pConn = basCb.conn;
  uint8_t       i;

  for (i = 0; i < DM_CONN_MAX; i++, pConn++)
  {
    if (pConn->connId != DM_CONN_ID_NONE)
    {
      pConn->battToSend = TRUE;
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Find next connection with measurement to send.
 *
 *  \param  cccIdx  Battery measurement CCC descriptor index.
 *
 *  \return Connection control block.
 */
/*************************************************************************************************/
static basConn_t *basFindNextToSend(uint8_t cccIdx)
{
  basConn_t    *pConn = basCb.conn;
  uint8_t       i;

  for (i = 0; i < DM_CONN_MAX; i++, pConn++)
  {
    if (pConn->connId != DM_CONN_ID_NONE && pConn->battToSend &&
        pConn->sentBattLevel != basCb.measBattLevel)
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
 *  \brief  Send periodic battery measurement.
 *
 *  \param  pConn   Connection control block.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void basSendPeriodicBattlevel(basConn_t *pConn)
{
  BasSendBattLevel(pConn->connId, basCb.measTimer.msg.status, basCb.measBattLevel);
  pConn->sentBattLevel = basCb.measBattLevel;
  pConn->battToSend = FALSE;
  basCb.txReady = FALSE;
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
static void basConnOpen(dmEvt_t *pMsg)
{
  basCb.txReady = TRUE;
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
static void basHandleValueCnf(attEvt_t *pMsg)
{
  basConn_t  *pConn;

  if (pMsg->hdr.status == ATT_SUCCESS && pMsg->handle == BATT_LVL_HDL)
  {
    basCb.txReady = TRUE;

    /* find next connection to send (note ccc idx is stored in timer status) */
    if ((pConn = basFindNextToSend(basCb.measTimer.msg.status)) != NULL)
    {
      basSendPeriodicBattlevel(pConn);
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
void basMeasTimerExp(wsfMsgHdr_t *pMsg)
{
  basConn_t  *pConn;

  /* if there are active connections */
  if (basNoConnActive() == FALSE)
  {
    if (--basCb.currCount == 0)
    {
      /* reset count */
      basCb.currCount = basCb.cfg.count;

      /* set up battery measurement to be sent on all connections */
      basSetupToSend();

      /* read battery measurement sensor data */
      AppHwBattRead(&basCb.measBattLevel);

      /* if ready to send measurements */
      if (basCb.txReady)
      {
        /* find next connection to send (note ccc idx is stored in timer status) */
        if ((pConn = basFindNextToSend(pMsg->status)) != NULL)
        {
          basSendPeriodicBattlevel(pConn);
        }
      }
    }

    /* restart timer */
    WsfTimerStartSec(&basCb.measTimer, basCb.cfg.period);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize the battery service server.
 *
 *  \param  handerId    WSF handler ID of the application using this service.
 *  \param  pCfg        Battery service configurable parameters.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BasInit(wsfHandlerId_t handlerId, basCfg_t *pCfg)
{
  basCb.measTimer.handlerId = handlerId;
  basCb.cfg = *pCfg;
}

/*************************************************************************************************/
/*!
 *  \brief  Start periodic battery level measurement.  This function starts a timer to perform
 *          periodic battery measurements.
 *
 *  \param  connId      DM connection identifier.
 *  \param  timerEvt    WSF event designated by the application for the timer.
 *  \param  battCccIdx  Index of battery level CCC descriptor in CCC descriptor handle table.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BasMeasBattStart(dmConnId_t connId, uint8_t timerEvt, uint8_t battCccIdx)
{
  /* if this is first connection */
  if (basNoConnActive())
  {
    /* initialize control block */
    basCb.measTimer.msg.event = timerEvt;
    basCb.measTimer.msg.status = battCccIdx;
    basCb.measBattLevel = BAS_BATT_LEVEL_INIT;
    basCb.currCount = basCb.cfg.count;

    /* start timer */
    WsfTimerStartSec(&basCb.measTimer, basCb.cfg.period);
  }

  /* set conn id and last sent battery level */
  basCb.conn[connId - 1].connId = connId;
  basCb.conn[connId - 1].sentBattLevel = BAS_BATT_LEVEL_INIT;
}

/*************************************************************************************************/
/*!
 *  \brief  Stop periodic battery level measurement.
 *
 *  \param  connId      DM connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BasMeasBattStop(dmConnId_t connId)
{
  /* clear connection */
  basCb.conn[connId - 1].connId = DM_CONN_ID_NONE;
  basCb.conn[connId - 1].battToSend = FALSE;

  /* if no remaining connections */
  if (basNoConnActive())
  {
    /* stop timer */
    WsfTimerStop(&basCb.measTimer);
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
void BasProcMsg(wsfMsgHdr_t *pMsg)
{
  if (pMsg->event == DM_CONN_OPEN_IND)
  {
    basConnOpen((dmEvt_t *) pMsg);
  }
  else if (pMsg->event == ATTS_HANDLE_VALUE_CNF)
  {
    basHandleValueCnf((attEvt_t *) pMsg);
  }
  else if (pMsg->event == basCb.measTimer.msg.event)
  {
    basMeasTimerExp(pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Send the battery level to the peer device.
 *
 *  \param  connId      DM connection identifier.
 *  \param  idx         Index of battery level CCC descriptor in CCC descriptor handle table.
 *  \param  level       The battery level.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BasSendBattLevel(dmConnId_t connId, uint8_t idx, uint8_t level)
{
  if (AttsCccEnabled(connId, idx))
  {
    AttsHandleValueNtf(connId, BATT_LVL_HDL, CH_BATT_LEVEL_LEN, &level);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  ATTS read callback for battery service used to read the battery level.  Use this
 *          function as a parameter to SvcBattCbackRegister().
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t BasReadCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                     uint16_t offset, attsAttr_t *pAttr)
{
  /* read the battery level and set attribute value */
  AppHwBattRead(pAttr->pValue);

  return ATT_SUCCESS;
}
