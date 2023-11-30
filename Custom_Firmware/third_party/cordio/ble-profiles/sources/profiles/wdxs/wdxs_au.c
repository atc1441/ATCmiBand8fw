/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Wireless Data Exchange profile implementation - Authentication.
 *
 *  Copyright (c) 2013-2018 Arm Ltd. All Rights Reserved.
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
#include "wsf_trace.h"
#include "wsf_assert.h"
#include "wsf_efs.h"
#include "wsf_os.h"
#include "sec_api.h"
#include "svc_wdxs.h"
#include "wdxs_api.h"
#include "wdxs_main.h"
#include "dm_api.h"
#include "att_api.h"
#include "app_api.h"

#if WDXS_AU_ENABLED == TRUE

/* WDXS Authentication Control Block */
wdxsAuCb_t wdxsAuCb;

/*************************************************************************************************/
/*!
 *  \brief  Transmit to authentication characteristic.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
void wdxsAuSend(dmConnId_t connId)
{
  APP_TRACE_INFO0("WDXS: AuSend");

  /* if notification enabled */
  if (AttsCccEnabled(connId, wdxsCb.auCccIdx))
  {
    /* send notification */
    AttsHandleValueNtf(connId, WDXS_AU_HDL, wdxsAuCb.auMsgLen, wdxsAuCb.auMsgBuf);
    wdxsCb.txReadyMask &= ~(WDXS_TX_MASK_AU_BIT | WDXS_TX_MASK_READY_BIT);

    wdxsAuCb.authState = WDXS_AU_STATE_WAIT_REPLY;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Called by WDXS event handler when the WSF Sec operation is complete.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsAuSecComplete(secAes_t *pAes)
{
  uint8_t *p = wdxsAuCb.auMsgBuf;

  /* Record the hash */
  memcpy(wdxsAuCb.auHash, pAes->pCiphertext, WDX_AU_HASH_LEN);

  /* Build challenge message */
  UINT8_TO_BSTREAM(p, WDX_AU_OP_CHALLENGE);
  memcpy(p, wdxsAuCb.auRand, WDX_AU_RAND_LEN);
  wdxsAuCb.auMsgLen = WDX_AU_RAND_LEN + WDX_AU_HDR_LEN;

  /* Update State */
  wdxsAuCb.authState = WDXS_AU_STATE_WAIT_REPLY;

  /* Indicate TX Ready */
  wdxsCb.txReadyMask &= ~(WDXS_TX_MASK_AU_BIT | WDXS_TX_MASK_READY_BIT);
  WsfSetEvent(wdxsCb.handlerId, WDXS_EVT_TX_PATH);
}

/*************************************************************************************************/
/*!
 *  \brief  Process a request to start authentication.
 *
 *  \return None.
 */
/*************************************************************************************************/
static uint8_t wdxsAuProcStart(dmConnId_t connId, uint8_t len, uint8_t *pValue)
{
  /* Verify parameter length */
  if (len != WDX_AU_PARAM_LEN_START)
  {
    return ATT_ERR_LENGTH;
  }

  /* Parse parameters */
  BSTREAM_TO_UINT8(wdxsAuCb.authLevel, pValue);
  BSTREAM_TO_UINT8(wdxsAuCb.authMode, pValue);

  /* Generate random number */
  SecRand(wdxsAuCb.auRand, WDX_AU_RAND_LEN);

  /* Encrypt the random number to create the hash */
  SecAes(wdxsAuCb.sessionKey, wdxsAuCb.auRand, wdxsCb.handlerId, connId, WDXS_EVT_AU_SEC_COMPLETE);

  /* Update State */
  wdxsAuCb.authState = WDXS_AU_STATE_WAIT_SEC;

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Process a reply to the authentication challenge.
 *
 *  \return None.
 */
/*************************************************************************************************/
static uint8_t wdxsAuProcReply(uint8_t len, uint8_t *pValue)
{
  /* Verify parameter length */
  if (len != WDX_AU_PARAM_LEN_REPLY)
  {
    return ATT_ERR_LENGTH;
  }

  if (wdxsAuCb.authState == WDXS_AU_STATE_WAIT_REPLY)
  {
    /* Verify [0-7] bytes of cipher text against what was sent by client */
    if (memcmp(wdxsAuCb.auHash, pValue, WDX_AU_HASH_LEN) == 0)
    {
      /* Successful challenge */
      wdxsAuCb.authState = WDXS_AU_STATE_AUTHORIZED;
      return ATT_SUCCESS;
    }

    return WDX_AU_ST_AUTH_FAILED;
  }

  return WDX_AU_ST_INVALID_STATE;
}

/*************************************************************************************************/
/*!
 *  \brief  Process a write to the authentication characteristic.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t wdxsAuWrite(dmConnId_t connId, uint16_t len, uint8_t *pValue)
{
  uint8_t op;
  uint8_t status;

  /* Sanity check message length */
  if (len < WDX_AU_HDR_LEN)
  {
    return ATT_ERR_LENGTH;
  }

  /* Get Operation ID */
  BSTREAM_TO_UINT8(op, pValue);
  len -= WDX_AU_HDR_LEN;

  APP_TRACE_INFO1("WDXS: AuWrite: op=%d", op);

  switch (op)
  {
    case WDX_AU_OP_START:
      status = wdxsAuProcStart(connId, (uint8_t) len, pValue);
      break;

    case WDX_AU_OP_REPLY:
      status = wdxsAuProcReply((uint8_t) len, pValue);
      break;

    default:
      status = ATT_ERR_RANGE;
      break;
  }

  return status;
}

/*************************************************************************************************/
/*!
 *  \brief  Called at startup to configure WDXS authentication.
 *
 *  \param  reqLevel  Level of authentication that is required for a client to use WDXS
 *  \param  key       Authentication key (set to NULL if no authentication is required)
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsAuthenticationCfg(bool_t reqLevel, uint8_t *pKey)
{
  wdxsAuCb.reqAuthLevel = reqLevel;

  if (pKey)
  {
    memcpy(wdxsAuCb.sessionKey, pKey, WDX_AU_KEY_LEN);
  }
}

#endif /* WDXS_AU_ENABLED */
