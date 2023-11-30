/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  User Data Service Collector.
 *
 *  Copyright (c) 2017-2018 Arm Ltd. All Rights Reserved.
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
#include "udsc_api.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/* Time to wait for control point response in seconds */
#define UDSC_RESPONSE_TIMEOUT             45

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/* Control block */
static struct
{
  UdsRspCback_t     rspCback;            /* Control Point Response Callback */
  wsfTimer_t        rspTimer;

} UdscCb;

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*!
 *  User data service characteristics for discovery
 */

/*! Database change increment */
static const attcDiscChar_t udscDbci =
{
  attDbciChUuid,
  0
};

/*! Database change increment CCC descriptor */
static const attcDiscChar_t udscDcbiCcc =
{
  attCliChCfgUuid,
  ATTC_SET_DESCRIPTOR
};

/*! User index */
static const attcDiscChar_t udscUi =
{
  attUiChUuid,
  0
};

/*! User control point */
static const attcDiscChar_t udscUcp =
{
  attUcpChUuid,
  0
};

/*! User control point CCC descriptor */
static const attcDiscChar_t udscUcpCcc =
{
  attCliChCfgUuid,
  ATTC_SET_DESCRIPTOR
};

/*! List of characteristics to be discovered; order matches handle index enumeration  */
static const attcDiscChar_t *udscDiscCharList[] =
{
  &udscDbci,                   /*! Database Change Increment */
  &udscDcbiCcc,                /*! Database Change Interval CCC descriptor */
  &udscUi,                     /*! User Index */
  &udscUcp,                    /*! User control point */
  &udscUcpCcc,                 /*! User control point CCC descriptor */
};

/* sanity check:  make sure handle list length matches characteristic list length */
WSF_CT_ASSERT(UDSC_HDL_LIST_LEN == ((sizeof(udscDiscCharList) / sizeof(attcDiscChar_t *))));


/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for User Data service.  Parameter
 *          pHdlList must point to an array of length UDSC_HDL_LIST_LEN.  If discovery is
 *          successful the handles of discovered characteristics and descriptors will be set
 *          in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UdscDiscover(dmConnId_t connId, uint16_t *pHdlList)
{
  AppDiscFindService(connId, ATT_16_UUID_LEN, (uint8_t *) attUdsSvcUuid,
                     UDSC_HDL_LIST_LEN, (attcDiscChar_t **) udscDiscCharList, pHdlList);
}

/*************************************************************************************************/
/*!
 *  \brief  Parse a User Control Point message.
 *
 *  \param  connId    Connection identifier.
 *  \param  pValue    Pointer to buffer containing value.
 *  \param  len       length of buffer.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void udscParseUcp(dmConnId_t connId, uint8_t *pValue, uint16_t len)
{
  uint8_t opcode;

  BSTREAM_TO_INT8(opcode, pValue);

  if (opcode == UDSC_UCP_OPCODE_RESPONSE)
  {
    /* Opcode is a response message */
    uint8_t responseOpcode;
    uint8_t response, index = 0;

    BSTREAM_TO_INT8(responseOpcode, pValue);
    BSTREAM_TO_INT8(response, pValue);

    /* Stop the timer waiting for a control point response */
    WsfTimerStop(&UdscCb.rspTimer);

    if ((response == UDSC_UCP_RSP_SUCCESS) && (responseOpcode == UDSC_UCP_OPCODE_RNU))
    {
      BSTREAM_TO_INT8(index, pValue);
    }

    if (UdscCb.rspCback)
    {
      UdscCb.rspCback(connId, responseOpcode, response, index);
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Process a value received in an ATT read response, notification, or indication
 *          message.  Parameter pHdlList must point to an array of length UDSC_PLXS_HDL_LIST_LEN.
 *          If the attribute handle of the message matches a handle in the handle list the value
 *          is processed, otherwise it is ignored.
 *
 *  \param  pHdlList  Characteristic handle list.
 *  \param  pMsg      ATT callback message.
 *
 *  \return ATT_SUCCESS if handle is found, ATT_ERR_NOT_FOUND otherwise.
 */
/*************************************************************************************************/
uint8_t UdscValueUpdate(uint16_t *pHdlList, attEvt_t *pMsg)
{
  uint8_t status = ATT_SUCCESS;

  /* User Control Point */
  if (pMsg->handle == pHdlList[UDSC_UCP_IDX])
  {
    APP_TRACE_INFO0("UDSC: User Control Point notification");

    /* parse value */
    udscParseUcp((dmConnId_t) pMsg->hdr.param, pMsg->pValue, pMsg->valueLen);
  }
  else
  {
    status = ATT_ERR_NOT_FOUND;
  }

  return status;
}

/*************************************************************************************************/
/*!
 *  \brief  Read the user index characteristic.
 *
 *  \param  connId      Connection identifier.
 *  \param  handle      Attribute handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UdscReadUserIndex(dmConnId_t connId, uint16_t handle)
{
  if (handle != ATT_HANDLE_NONE)
  {
    AttcReadReq(connId, handle);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Read the database change increment characteristic.
 *
 *  \param  connId      Connection identifier.
 *  \param  handle      Attribute handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UdscReadDatabaseChangeIncrement(dmConnId_t connId, uint16_t handle)
{
  if (handle != ATT_HANDLE_NONE)
  {
    AttcReadReq(connId, handle);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Write the database change increment characteristic.
 *
 *  \param  connId      Connection identifier.
 *  \param  handle      Attribute handle.
 *  \param  increment   DB change increment
 *
 *  \return None.
 */
/*************************************************************************************************/
void UdscWriteDatabaseChangeIncrement(dmConnId_t connId, uint16_t handle, uint32_t increment)
{
  uint8_t   buf[ATT_DEFAULT_PAYLOAD_LEN];
  uint8_t   *p = buf;

  if (handle != ATT_HANDLE_NONE)
  {
    /* build command */
    UINT32_TO_BSTREAM(p, increment);

    AttcWriteReq(connId, handle, (uint16_t) (p - buf), buf);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Write to the control point and set a timer waiting for the response.
 *
 *  \param  connId      Connection identifier.
 *  \param  handle      Attribute handle.
 *  \param  valueLen    Size of pValue in bytes
 *  \param  pValue      Value to write
 *
 *  \return None.
 */
/*************************************************************************************************/
static void udscWriteControlPoint(dmConnId_t connId, uint16_t handle, uint16_t valueLen, uint8_t *pValue)
{
  /* Start timer waiting for response */
  WsfTimerStartSec(&UdscCb.rspTimer, UDSC_RESPONSE_TIMEOUT);

  /* Send write request */
  AttcWriteReq(connId, handle, valueLen, pValue);
}

/*************************************************************************************************/
/*!
 *  \brief  Write to the user control point characteristic - Register New User.
 *
 *  \param  connId        Connection identifier.
 *  \param  handle        Attribute handle.
 *  \param  consentCode   Consent code (0-9999)
 *
 *  \return None.
 */
/*************************************************************************************************/
void UdscRegisterNewUser(dmConnId_t connId, uint16_t handle, uint16_t consentCode)
{
  uint8_t   buf[ATT_DEFAULT_PAYLOAD_LEN];
  uint8_t   *p = buf;

  if (handle != ATT_HANDLE_NONE)
  {
    /* build command */
    UINT8_TO_BSTREAM(p, UDSC_UCP_OPCODE_RNU);
    UINT16_TO_BSTREAM(p, consentCode);

    udscWriteControlPoint(connId, handle, (uint16_t) (p - buf), buf);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Write to the user control point characteristic - Consent.
 *
 *  \param  connId        Connection identifier.
 *  \param  handle        Attribute handle.
 *  \param  index         User index
 *  \param  consentCode   Consent code (0-9999 - provided when user was registered)
 *
 *  \return None.
 */
/*************************************************************************************************/
void UdscConsent(dmConnId_t connId, uint16_t handle, uint8_t index, uint16_t consentCode)
{
  uint8_t   buf[ATT_DEFAULT_PAYLOAD_LEN];
  uint8_t   *p = buf;

  if (handle != ATT_HANDLE_NONE)
  {
    /* build command */
    UINT8_TO_BSTREAM(p, UDSC_UCP_OPCODE_CONSENT);
    UINT8_TO_BSTREAM(p, index);
    UINT16_TO_BSTREAM(p, consentCode);

    udscWriteControlPoint(connId, handle, (uint16_t) (p - buf), buf);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Write to the user control point characteristic - Delete User Data.
 *
 *  \param  connId      Connection identifier.
 *  \param  handle      Attribute handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UdscDeleteUserData(dmConnId_t connId, uint16_t handle)
{
  uint8_t   buf[ATT_DEFAULT_PAYLOAD_LEN];
  uint8_t   *p = buf;

  if (handle != ATT_HANDLE_NONE)
  {
    /* build command */
    UINT8_TO_BSTREAM(p, UDSC_UCP_OPCODE_DUD);

    udscWriteControlPoint(connId, handle, (uint16_t) (p - buf), buf);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Called by the application when a connection closes.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UdscClose(void)
{
  /* Stop the timer waiting for a control point response */
  WsfTimerStop(&UdscCb.rspTimer);
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize User Data Service collector callbacks.
 *
 *  \param  handlerId      Application task handler ID.
 *  \param  timerEvent     Application timer event for control point timeout.
 *  \param  rspCback       Callback to receive control point response messages.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UdscInit(wsfHandlerId_t handlerId, uint8_t timerEvent, UdsRspCback_t rspCback)
{
  UdscCb.rspTimer.handlerId = handlerId;
  UdscCb.rspTimer.msg.event = timerEvent;

  UdscCb.rspCback = rspCback;
}
