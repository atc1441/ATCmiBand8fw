/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Alert Notification profile client.
 *
 *  Copyright (c) 2011-2018 Arm Ltd. All Rights Reserved.
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
#include "app_api.h"
#include "anpc_api.h"

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*!
 *  Alert Notification service
 */

/* Characteristics for discovery */

/*! Supported new alert category */
static const attcDiscChar_t anpcAnsSnac =
{
  attSnacChUuid,
  ATTC_SET_REQUIRED
};

/*! New alert */
static const attcDiscChar_t anpcAnsNa =
{
  attNaChUuid,
  ATTC_SET_REQUIRED
};

/*! New alert CCC descriptor */
static const attcDiscChar_t anpcAnsNaCcc =
{
  attCliChCfgUuid,
  ATTC_SET_REQUIRED | ATTC_SET_DESCRIPTOR
};

/*! Supported unread alert category */
static const attcDiscChar_t anpcAnsSuac =
{
  attSuacChUuid,
  ATTC_SET_REQUIRED
};

/*! Unread alert status */
static const attcDiscChar_t anpcAnsUas =
{
  attUasChUuid,
  ATTC_SET_REQUIRED
};
/*! Unread alert status CCC descriptor */
static const attcDiscChar_t anpcAnsUasCcc =
{
  attCliChCfgUuid,
  ATTC_SET_REQUIRED | ATTC_SET_DESCRIPTOR
};

/*! Alert notification control point */
static const attcDiscChar_t anpcAnsAncp =
{
  attAncpChUuid,
  ATTC_SET_REQUIRED
};

/*! List of characteristics to be discovered; order matches handle index enumeration  */
static const attcDiscChar_t *anpcAnsDiscCharList[] =
{
  &anpcAnsSnac,                   /*! Supported new alert category */
  &anpcAnsNa,                     /*! New alert */
  &anpcAnsNaCcc,                  /*! New alert CCC descriptor */
  &anpcAnsSuac,                   /*! Supported unread alert category */
  &anpcAnsUas,                    /*! Unread alert status */
  &anpcAnsUasCcc,                 /*! Unread alert status CCC descriptor */
  &anpcAnsAncp                    /*! Alert notification control point */
};

/* sanity check:  make sure handle list length matches characteristic list length */
WSF_CT_ASSERT(ANPC_ANS_HDL_LIST_LEN == ((sizeof(anpcAnsDiscCharList) / sizeof(attcDiscChar_t *))));

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for Alert Notification service.  Parameter
 *          pHdlList must point to an array of length ANPC_ANS_HDL_LIST_LEN.  If discovery is
 *          successful the handles of discovered characteristics and descriptors will be set
 *          in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AnpcAnsDiscover(dmConnId_t connId, uint16_t *pHdlList)
{
  AppDiscFindService(connId, ATT_16_UUID_LEN, (uint8_t *) attAnsSvcUuid,
                     ANPC_ANS_HDL_LIST_LEN, (attcDiscChar_t **) anpcAnsDiscCharList, pHdlList);
}

/*************************************************************************************************/
/*!
 *  \brief  Send a command to the alert notification control point.
 *
 *  \param  connId    Connection identifier.
 *  \param  handle    Attribute handle.
 *  \param  command   Control point command.
 *  \param  catId     Alert category ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AnpcAnsControl(dmConnId_t connId, uint16_t handle, uint8_t command, uint8_t catId)
{
  uint8_t buf[2];

  if (handle != ATT_HANDLE_NONE)
  {
    buf[0] = command;
    buf[1] = catId;
    AttcWriteReq(connId, handle, sizeof(buf), buf);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Process a value received in an ATT read response, notification, or indication
 *          message.  Parameter pHdlList must point to an array of length ANPC_ANS_HDL_LIST_LEN.
 *          If the attribute handle of the message matches a handle in the handle list the value
 *          is processed, otherwise it is ignored.
 *
 *  \param  pHdlList  Characteristic handle list.
 *  \param  pMsg      ATT callback message.
 *
 *  \return ATT_SUCCESS if handle is found, ATT_ERR_NOT_FOUND otherwise.
 */
/*************************************************************************************************/
uint8_t AnpcAnsValueUpdate(uint16_t *pHdlList, attEvt_t *pMsg)
{
  uint8_t   *p;
  uint16_t  catIdMask;
  uint8_t   catId;
  uint8_t   numAlert;
  uint8_t   status = ATT_SUCCESS;
  uint8_t   buf[19];

#if 1
    // Ambiq: Avoid 'variable is used before its value is set' warnings. Warning
    //        is actually invalid because they're set in BSTREAM_TO_UINTxx().
    catIdMask = catId = numAlert = 0;
#endif

  /* Suppress unused variable compile warning */
  (void)catIdMask; (void)catId; (void)numAlert;

  /* new alert */
  if (pMsg->handle == pHdlList[ANPC_ANS_NA_HDL_IDX])
  {
    /* parse value */
    p = pMsg->pValue;
    BSTREAM_TO_UINT8(catId, p);
    BSTREAM_TO_UINT8(numAlert, p);

    /* null terminate string before printing */
    memcpy(buf, p, pMsg->valueLen - 2);
    buf[pMsg->valueLen - 2] = '\0';

    APP_TRACE_INFO2("New alert cat:%d num:%d", catId, numAlert);
    APP_TRACE_INFO1("Msg:%s", buf);
  }
  /* unread alert status */
  else if (pMsg->handle == pHdlList[ANPC_ANS_UAS_HDL_IDX])
  {
    /* parse value */
    p = pMsg->pValue;
    BSTREAM_TO_UINT8(catId, p);
    BSTREAM_TO_UINT8(numAlert, p);

    APP_TRACE_INFO2("Unread alert status cat:%d num:%d", catId, numAlert);
  }
  /* supported new alert category */
  else if (pMsg->handle == pHdlList[ANPC_ANS_SNAC_HDL_IDX])
  {
    /* parse value */
    p = pMsg->pValue;
    if (pMsg->valueLen == 1)
    {
      BSTREAM_TO_UINT8(catIdMask, p);
    }
    else
    {
      BSTREAM_TO_UINT16(catIdMask, p);
    }

    APP_TRACE_INFO1("Supported new alert category: 0x%04x", catIdMask);
  }
  /* supported unread alert category */
  else if (pMsg->handle == pHdlList[ANPC_ANS_SUAC_HDL_IDX])
  {
    /* parse value */
    p = pMsg->pValue;
    if (pMsg->valueLen == 1)
    {
      BSTREAM_TO_UINT8(catIdMask, p);
    }
    else
    {
      BSTREAM_TO_UINT16(catIdMask, p);
    }

    APP_TRACE_INFO1("Supported unread alert category: 0x%04x", catIdMask);
  }
  /* handle not found in list */
  else
  {
    status = ATT_ERR_NOT_FOUND;
  }

  return status;
}
