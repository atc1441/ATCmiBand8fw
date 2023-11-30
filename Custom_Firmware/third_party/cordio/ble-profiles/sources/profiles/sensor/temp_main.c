/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example temperature service profile.
*
 *  Copyright (c) 2015-2018 Arm Ltd. All Rights Reserved.
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

#include <stdlib.h>

#include "wsf_types.h"
#include "app_api.h"
#include "att_api.h"
#include "wsf_trace.h"
#include "util/bstream.h"

#include "temp_api.h"
#include "svc_temp.h"

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! Control block. */
static struct
{
  wsfTimer_t  measTimer;
  bool_t      measTimerStarted;
} tempCb;

/*************************************************************************************************/
/*!
 *  \brief    Update measurement timer.
 *
 *  \return   None.
 */
/*************************************************************************************************/
static void tempUpdateTimer(void)
{
  uint8_t  config;
  uint8_t *pConfig = NULL;
  uint8_t  period;
  uint8_t *pPeriod = NULL;
  uint16_t attLen = 0;

  /* Get config & period. */
  AttsGetAttr(TEMP_HANDLE_CONFIG, &attLen, &pConfig);
  if (pConfig == NULL)
  {
    WSF_TRACE_ERR0("temp: unable to read config");
    return;
  }
  config = *pConfig;
  AttsGetAttr(TEMP_HANDLE_PERIOD, &attLen, &pPeriod);
  if (pPeriod == NULL)
  {
    WSF_TRACE_ERR0("temp: unable to read period");
    return;
  }
  period = *pPeriod;
  if (period < TEMP_ATT_PERIOD_MIN)
  {
    period = TEMP_ATT_PERIOD_MIN;
  }

  if (config == TEMP_ATT_CONFIG_ENABLE)
  {
    if (!tempCb.measTimerStarted)
    {
      tempCb.measTimerStarted = TRUE;
      WsfTimerStartMs(&tempCb.measTimer, period * 10u);
    }
  }
  else
  {
    if (tempCb.measTimerStarted)
    {
      tempCb.measTimerStarted = FALSE;
      WsfTimerStop(&tempCb.measTimer);
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  ATTS write callback for temperature profile.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t tempWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                              uint16_t offset, uint16_t len, uint8_t *pValue,
                              attsAttr_t *pAttr)
{
  switch (handle)
  {
    case TEMP_HANDLE_CONFIG:
    {
      uint8_t config;

      /* Check attribute value. */
      if (len != 1)
      {
        return ATT_ERR_LENGTH;
      }
      config = *pValue;
      if ((config != TEMP_ATT_CONFIG_DISABLE) && (config != TEMP_ATT_CONFIG_ENABLE))
      {
        return ATT_ERR_RANGE;
      }

      /* Save value. */
      AttsSetAttr(TEMP_HANDLE_CONFIG, len, pValue);

      /* Enable or disable timer. */
      tempUpdateTimer();
      return ATT_SUCCESS;
    }
    case TEMP_HANDLE_PERIOD:
    {
      uint8_t period;

      if (len != 1)
      {
        return ATT_ERR_LENGTH;
      }
      period = *pValue;
      if ((period < TEMP_ATT_PERIOD_MIN) || (period > TEMP_ATT_PERIOD_MAX))
      {
        return ATT_ERR_RANGE;
      }
      AttsSetAttr(TEMP_HANDLE_PERIOD, len, pValue);
      return ATT_SUCCESS;
    }
  }
  return ATT_ERR_NOT_SUP;
}

/*************************************************************************************************/
/*!
 *  \brief  Start service.
 *
 *  \param  handlerId       Handler ID.
 *  \param  timerEvt        Timer message event.
 *
 *  \return None.
 */
/*************************************************************************************************/
void TempStart(wsfHandlerId_t handlerId, uint8_t timerEvt)
{
  SvcTempAddGroup();
  SvcTempCbackRegister(tempWriteCback);

  tempCb.measTimer.handlerId = handlerId;
  tempCb.measTimer.msg.event = timerEvt;
  tempCb.measTimerStarted    = FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Stop service.
 *
 *  \return None.
 */
/*************************************************************************************************/
void TempStop(void)
{
  TempMeasStop();
  SvcTempRemoveGroup();
}

/*************************************************************************************************/
/*!
 *  \brief  Measurement stop handler.
 *
 *  \return None.
 */
/*************************************************************************************************/
void TempMeasStop(void)
{
  tempCb.measTimerStarted = FALSE;
  WsfTimerStop(&tempCb.measTimer);
}

/*************************************************************************************************/
/*!
 *  \brief  Measurement start handler.
 *
 *  \return None.
 */
/*************************************************************************************************/
void TempMeasStart(void)
{
  tempUpdateTimer();
}

/*************************************************************************************************/
/*!
 *  \brief  Measurement complete handler.
 *
 *  \param  connId    Connection ID.
 *  \param  temp      Temperature reading.
 *
 *  \return None.
 */
/*************************************************************************************************/
void TempMeasComplete(dmConnId_t connId, int16_t temp)
{
  tempCb.measTimerStarted = FALSE;

  uint8_t tempData[2] = {UINT16_TO_BYTES(temp)};
  AttsSetAttr(TEMP_HANDLE_DATA, sizeof(tempData), tempData);
  AttsHandleValueNtf(connId, TEMP_HANDLE_DATA, sizeof(tempData), tempData);

  tempUpdateTimer();
}
