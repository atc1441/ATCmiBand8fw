/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Security ECC implementation using uECC.
 *
 *  Copyright (c) 2010-2018 Arm Ltd.
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
#include <time.h>
#include <string.h>
#include "wsf_types.h"
#include "wsf_queue.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "sec_api.h"
#include "sec_main.h"
#include "wsf_buf.h"
#include "hci_api.h"
#include "util/calc128.h"
#include "uECC.h"

#ifndef SEC_ECC_CFG
#define SEC_ECC_CFG SEC_ECC_CFG_UECC
#endif

#if SEC_ECC_CFG == SEC_ECC_CFG_UECC

/**************************************************************************************************
  External Variables
**************************************************************************************************/

extern secCb_t secCb;

/*************************************************************************************************/
/*!
 *  \brief  Random number generator used by uECC.
 *
 *  \param  p_dest      Buffer to hold random number
 *  \param  p_size      Size of p_dest in bytes .
 *
 *  \return TRUE if successful.
 */
/*************************************************************************************************/
static int secEccRng(uint8_t *p_dest, unsigned p_size)
{
  SecRand(p_dest, p_size);
  return TRUE;
}

/*************************************************************************************************/
/*!
 *  \brief  Callback for HCI encryption for ECC operations.
 *
 *  \param  pBuf        Pointer to sec queue element.
 *  \param  pEvent      Pointer to HCI event.
 *  \param  handlerId   WSF handler ID.
 *
 *  \return none.
 */
/*************************************************************************************************/
void SecEccHciCback(secQueueBuf_t *pBuf, hciEvt_t *pEvent, wsfHandlerId_t handlerId)
{
  /* TBD */
}

/*************************************************************************************************/
/*!
 *  \brief  Generate an ECC key.
 *
 *  \param  handlerId   WSF handler ID for client.
 *  \param  param       Optional parameter sent to client's WSF handler.
 *  \param  event       Event for client's WSF handler.
 *
 *  \return TRUE if successful, else FALSE.
 */
/*************************************************************************************************/
bool_t SecEccGenKey(wsfHandlerId_t handlerId, uint16_t param, uint8_t event)
{
  secEccMsg_t *pMsg = WsfMsgAlloc(sizeof(secEccMsg_t));

  if (pMsg)
  {
    /* Generate the keys */
    uECC_make_key(pMsg->data.key.pubKey_x, pMsg->data.key.privKey);

    /* Send shared secret to handler */
    pMsg->hdr.event = event;
    pMsg->hdr.param = param;
    pMsg->hdr.status = HCI_SUCCESS;
    WsfMsgSend(handlerId, pMsg);

    return TRUE;
  }

  return FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Generate an ECC key.
 *
 *  \param  pKey        ECC Key structure.
 *  \param  handlerId   WSF handler ID for client.
 *  \param  param       Optional parameter sent to client's WSF handler.
 *  \param  event       Event for client's WSF handler.
 *
 *  \return TRUE if successful, else FALSE.
 */
/*************************************************************************************************/
bool_t SecEccGenSharedSecret(secEccKey_t *pKey, wsfHandlerId_t handlerId, uint16_t param, uint8_t event)
{
  secEccMsg_t *pMsg = WsfMsgAlloc(sizeof(secEccMsg_t));

  if (pMsg)
  {
    bool_t keyValid = uECC_valid_public_key(pKey->pubKey_x);

    if (keyValid)
    {
      uECC_shared_secret(pKey->pubKey_x, pKey->privKey, pMsg->data.sharedSecret.secret);
    }
    else
    {
      memset(pMsg->data.sharedSecret.secret, 0xFF, SEC_ECC_KEY_LEN);
    }

    /* Send shared secret to handler. */
    pMsg->hdr.event = event;
    pMsg->hdr.param = param;
    pMsg->hdr.status = keyValid ? HCI_SUCCESS : HCI_ERR_INVALID_PARAM;
    WsfMsgSend(handlerId, pMsg);

    return TRUE;
  }

  return FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Called to initialize ECC security.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SecEccInit()
{
  //srand((unsigned int)time(NULL));
  uECC_set_rng(secEccRng);
}

#endif /* SEC_ECC_CFG */
