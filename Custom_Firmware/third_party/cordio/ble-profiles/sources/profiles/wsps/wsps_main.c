/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Weight Scale profile sensor.
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
#include "svc_wss.h"
#include "app_api.h"
#include "app_hw.h"
#include "wsps_api.h"

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/* Control block */
static struct
{
  appWsm_t      wsm;                  /* weight scale measurement */
  uint8_t       wsmFlags;             /* flags */
} wspsCb;

/*************************************************************************************************/
/*!
 *  \brief  Build a weight scale measurement characteristic.
 *
 *  \param  pBuf     Pointer to buffer to hold the built weight scale measurement characteristic.
 *  \param  pWsm     Weight measurement values.
 *
 *  \return Length of pBuf in bytes.
 */
/*************************************************************************************************/
static uint8_t wspsBuildWsm(uint8_t *pBuf, appWsm_t *pWsm)
{
  uint8_t   *p = pBuf;
  uint8_t   flags = pWsm->flags;

  /* flags */
  UINT8_TO_BSTREAM(p, flags);

  /* measurement */
  UINT16_TO_BSTREAM(p, pWsm->weight);

  /* time stamp */
  if (flags & CH_WSM_FLAG_TIMESTAMP)
  {
    UINT16_TO_BSTREAM(p, pWsm->timestamp.year);
    UINT8_TO_BSTREAM(p, pWsm->timestamp.month);
    UINT8_TO_BSTREAM(p, pWsm->timestamp.day);
    UINT8_TO_BSTREAM(p, pWsm->timestamp.hour);
    UINT8_TO_BSTREAM(p, pWsm->timestamp.min);
    UINT8_TO_BSTREAM(p, pWsm->timestamp.sec);
  }

  /* return length */
  return (uint8_t) (p - pBuf);
}

/*************************************************************************************************/
/*!
 *  \brief  Weight measurement complete.
 *
 *  \param  connId      DM connection identifier.
 *  \param  wsmCccIdx   Index of weight scale measurement CCC descriptor in CCC descriptor
 *                      handle table.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WspsMeasComplete(dmConnId_t connId, uint8_t wsmCccIdx)
{
  uint8_t buf[ATT_DEFAULT_PAYLOAD_LEN];
  uint8_t len;

  /* if indications enabled  */
  if (AttsCccEnabled(connId, wsmCccIdx))
  {
    /* read weight scale measurement sensor data */
    AppHwWsmRead(&wspsCb.wsm);

    /* set flags */
    wspsCb.wsm.flags = wspsCb.wsmFlags;

    /* build weight scale measurement characteristic */
    len = wspsBuildWsm(buf, &wspsCb.wsm);

    /* send weight scale measurement indication */
    AttsHandleValueInd(connId, WSS_WM_HDL, len, buf);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Set the weight scale measurement flags.
 *
 *  \param  flags      Weight measurement flags.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WspsSetWsmFlags(uint8_t flags)
{
  wspsCb.wsmFlags = flags;
}
