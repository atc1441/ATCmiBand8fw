/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Phone Alert Status profile client.
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

#include "wsf_types.h"
#include "wsf_assert.h"
#include "wsf_trace.h"
#include "util/bstream.h"
#include "app_api.h"
#include "paspc_api.h"

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*!
 *  Phone Alert Status service
 */

/* Characteristics for discovery */


/*! Alert status */
static const attcDiscChar_t paspcPassAs =
{
  attAsChUuid,
  ATTC_SET_REQUIRED
};

/*! Alert status CCC descriptor */
static const attcDiscChar_t paspcPassAsCcc =
{
  attCliChCfgUuid,
  ATTC_SET_REQUIRED | ATTC_SET_DESCRIPTOR
};

/*! Ringer setting */
static const attcDiscChar_t paspcPassRs =
{
  attRsChUuid,
  ATTC_SET_REQUIRED
};

/*! Ringer setting CCC descriptor */
static const attcDiscChar_t paspcPassRsCcc =
{
  attCliChCfgUuid,
  ATTC_SET_REQUIRED | ATTC_SET_DESCRIPTOR
};

/*! Ringer control point */
static const attcDiscChar_t paspcPassRcp =
{
  attRcpChUuid,
  0
};

/*! List of characteristics to be discovered; order matches handle index enumeration  */
static const attcDiscChar_t *paspcPassDiscCharList[] =
{
  &paspcPassAs,                     /*! Alert status */
  &paspcPassAsCcc,                  /*! Alert status CCC descriptor */
  &paspcPassRs,                     /*! Ringer setting */
  &paspcPassRsCcc,                  /*! Ringer setting CCC descriptor */
  &paspcPassRcp                     /*! Ringer control point */
};

/* sanity check:  make sure handle list length matches characteristic list length */
WSF_CT_ASSERT(PASPC_PASS_HDL_LIST_LEN == ((sizeof(paspcPassDiscCharList) / sizeof(attcDiscChar_t *))));

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for Phone Alert Status service.  Parameter
 *          pHdlList must point to an array of length PASPC_ANS_HDL_LIST_LEN.  If discovery is
 *          successful the handles of discovered characteristics and descriptors will be set
 *          in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void PaspcPassDiscover(dmConnId_t connId, uint16_t *pHdlList)
{
  AppDiscFindService(connId, ATT_16_UUID_LEN, (uint8_t *) attPassSvcUuid,
                     PASPC_PASS_HDL_LIST_LEN, (attcDiscChar_t **) paspcPassDiscCharList, pHdlList);
}

/*************************************************************************************************/
/*!
 *  \brief  Send a command to the ringer control point.
 *
 *  \param  connId    Connection identifier.
 *  \param  handle    Attribute handle.
 *  \param  command   Control point command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void PaspcPassControl(dmConnId_t connId, uint16_t handle, uint8_t command)
{
  uint8_t buf[1];

  if (handle != ATT_HANDLE_NONE)
  {
    buf[0] = command;
    AttcWriteCmd(connId, handle, sizeof(buf), buf);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Process a value received in an ATT read response, notification, or indication
 *          message.  Parameter pHdlList must point to an array of length PASPC_PASS_HDL_LIST_LEN.
 *          If the attribute handle of the message matches a handle in the handle list the value
 *          is processed, otherwise it is ignored.
 *
 *  \param  pHdlList  Characteristic handle list.
 *  \param  pMsg      ATT callback message.
 *
 *  \return ATT_SUCCESS if handle is found, ATT_ERR_NOT_FOUND otherwise.
 */
/*************************************************************************************************/
uint8_t PaspcPassValueUpdate(uint16_t *pHdlList, attEvt_t *pMsg)
{
  uint8_t status = ATT_SUCCESS;

  /* alert status */
  if (pMsg->handle == pHdlList[PASPC_PASS_AS_HDL_IDX])
  {
    APP_TRACE_INFO1("Phone alert status: 0x%02x", *pMsg->pValue);
  }
  /* ringer setting */
  else if (pMsg->handle == pHdlList[PASPC_PASS_RS_HDL_IDX])
  {
    APP_TRACE_INFO1("Ringer setting: 0x%02x", *pMsg->pValue);
  }
  /* handle not found in list */
  else
  {
    status = ATT_ERR_NOT_FOUND;
  }

  return status;
}
