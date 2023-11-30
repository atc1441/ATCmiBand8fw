/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Weight Scale profile collector.
 *
 *  Copyright (c) 2012-2019 Arm Ltd. All Rights Reserved.
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
#include "svc_ch.h"
#include "app_api.h"
#include "wspc_api.h"

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*!
 *  Weight Scale service characteristics for discovery
 */

/*! Weight scale measurement */
static const attcDiscChar_t wspcWssWsm =
{
  attWmChUuid,
  ATTC_SET_REQUIRED
};

/*! Weight scale measurement CCC descriptor */
static const attcDiscChar_t wspcWssWsmCcc =
{
  attCliChCfgUuid,
  ATTC_SET_REQUIRED | ATTC_SET_DESCRIPTOR
};

/*! Weight scale feature */
static const attcDiscChar_t wspcWssWsf =
{
  attWsfChUuid,
  ATTC_SET_REQUIRED
};

/*! List of characteristics to be discovered; order matches handle index enumeration  */
static const attcDiscChar_t *wspcWssDiscCharList[] =
{
  &wspcWssWsm,                    /*! Weight scale measurement */
  &wspcWssWsmCcc,                 /*! Weight scale measurement CCC descriptor */
  &wspcWssWsf                     /*! Weight scale feature */
};

/* sanity check:  make sure handle list length matches characteristic list length */
WSF_CT_ASSERT(WSPC_WSS_HDL_LIST_LEN == ((sizeof(wspcWssDiscCharList) / sizeof(attcDiscChar_t *))));

/*************************************************************************************************/
/*!
 *  \brief  Parse a weight scale measurement.
 *
 *  \param  pValue    Pointer to buffer containing value.
 *  \param  len       length of buffer.
 *
 *  \return None.
 */
/*************************************************************************************************/
void wspcWssParseWsm(uint8_t *pValue, uint16_t len)
{
  uint8_t   flags = 0;
  uint16_t  weight;
  uint16_t  year;
  uint8_t   month, day, hour, min, sec;
  uint16_t  minLen = CH_WSM_FLAGS_LEN + CH_WSM_MEAS_LEN;

  /* Suppress unused variable compile warning */
  (void)month; (void)day; (void)hour; (void)min; (void)sec; (void)year; (void)weight;

  if (len > 0)
  {
    /* get flags */
    BSTREAM_TO_UINT8(flags, pValue);

    /* determine expected minimum length based on flags */
    if (flags & CH_WSM_FLAG_TIMESTAMP)
    {
      minLen += CH_WSM_TIMESTAMP_LEN;
    }
  }

  /* verify length */
  if (len < minLen)
  {
    APP_TRACE_INFO2("Weight Scale meas len:%d minLen:%d", len, minLen);
    return;
  }

  /* weight */
  BSTREAM_TO_UINT16(weight, pValue);
  if (flags & CH_WSM_FLAG_UNITS_LBS)
  {
    APP_TRACE_INFO2("  Weight: %d.%02d", (weight / 100), (weight % 100));
  }
  else /* CH_WSM_FLAG_UNITS_KG */
  {
    APP_TRACE_INFO2("  Weight: %d.%03d", (weight / 200), ((weight % 200) * 5));
  }

  /* timestamp */
  if (flags & CH_WSM_FLAG_TIMESTAMP)
  {
    BSTREAM_TO_UINT16(year, pValue);
    BSTREAM_TO_UINT8(month, pValue);
    BSTREAM_TO_UINT8(day, pValue);
    BSTREAM_TO_UINT8(hour, pValue);
    BSTREAM_TO_UINT8(min, pValue);
    BSTREAM_TO_UINT8(sec, pValue);
    APP_TRACE_INFO3("  Date: %d/%d/%d", month, day, year);
    APP_TRACE_INFO3("  Time: %02d:%02d:%02d", hour, min, sec);
  }

  APP_TRACE_INFO1("  Flags:0x%02x", flags);
}

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for Weight Scale service.  Parameter
 *          pHdlList must point to an array of length WSPC_WSS_HDL_LIST_LEN.  If discovery is
 *          successful the handles of discovered characteristics and descriptors will be set
 *          in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WspcWssDiscover(dmConnId_t connId, uint16_t *pHdlList)
{
  AppDiscFindService(connId, ATT_16_UUID_LEN, (uint8_t *) attWssSvcUuid,
                     WSPC_WSS_HDL_LIST_LEN, (attcDiscChar_t **) wspcWssDiscCharList, pHdlList);
}

/*************************************************************************************************/
/*!
 *  \brief  Process a value received in an ATT read response, notification, or indication
 *          message.  Parameter pHdlList must point to an array of length WSPC_WSS_HDL_LIST_LEN.
 *          If the attribute handle of the message matches a handle in the handle list the value
 *          is processed, otherwise it is ignored.
 *
 *  \param  pHdlList  Characteristic handle list.
 *  \param  pMsg      ATT callback message.
 *
 *  \return ATT_SUCCESS if handle is found, ATT_ERR_NOT_FOUND otherwise.
 */
/*************************************************************************************************/
uint8_t WspcWssValueUpdate(uint16_t *pHdlList, attEvt_t *pMsg)
{
  uint8_t   status = ATT_SUCCESS;

  /* weight scale measurement */
  if (pMsg->handle == pHdlList[WSPC_WSS_WSM_HDL_IDX])
  {
    APP_TRACE_INFO0("Weight measurement");

    /* parse value */
    wspcWssParseWsm(pMsg->pValue, pMsg->valueLen);
  }
  else
  {
    status = ATT_ERR_NOT_FOUND;
  }

  return status;
}
