/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  GAP profile.
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

#include "wsf_types.h"
#include "wsf_assert.h"
#include "wsf_trace.h"
#include "app_db.h"
#include "app_api.h"
#include "gap_api.h"

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! GAP service characteristics for discovery */

/*! Central Address Resolution */
static const attcDiscChar_t gapCar =
{
  attCarChUuid,
  0
};

/*! Resolvable Private Address Only */
static const attcDiscChar_t gapRpao =
{
  attRpaoChUuid,
  0
};

/*! List of characteristics to be discovered; order matches handle index enumeration  */
static const attcDiscChar_t *gapDiscCharList[] =
{
  &gapCar,                   /* Central Address Resolution */
  &gapRpao                   /* Resolvable Private Address Only */
};

/* sanity check:  make sure handle list length matches characteristic list length */
WSF_CT_ASSERT(GAP_HDL_LIST_LEN == ((sizeof(gapDiscCharList) / sizeof(attcDiscChar_t *))));

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for GAP service.  Note that pHdlList
 *          must point to an array of handles of length GAP_HDL_LIST_LEN.  If discovery is
 *          successful the handles of discovered characteristics and descriptors will be set
 *          in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void GapDiscover(dmConnId_t connId, uint16_t *pHdlList)
{
  AppDiscFindService(connId, ATT_16_UUID_LEN, (uint8_t *) attGapSvcUuid,
                     GAP_HDL_LIST_LEN, (attcDiscChar_t **) gapDiscCharList, pHdlList);
}

/*************************************************************************************************/
/*!
 *  \brief  Process a value received in an ATT read response, notification, or indication
 *          message.  Parameter pHdlList must point to an array of length GAP_HDL_LIST_LEN.
 *          If the attribute handle of the message matches a handle in the handle list the value
 *          is processed, otherwise it is ignored.
 *
 *  \param  pHdlList  Characteristic handle list.
 *  \param  pMsg      ATT callback message.
 *
 *  \return ATT_SUCCESS if handle is found, ATT_ERR_NOT_FOUND otherwise.
 */
/*************************************************************************************************/
uint8_t GapValueUpdate(uint16_t *pHdlList, attEvt_t *pMsg)
{
  uint8_t status = ATT_SUCCESS;

  /* Central Address Resolution */
  if (pMsg->handle == pHdlList[GAP_CAR_HDL_IDX])
  {
    appDbHdl_t dbHdl;

    /* if there's a device record */
    if ((dbHdl = AppDbGetHdl((dmConnId_t)pMsg->hdr.param)) != APP_DB_HDL_NONE)
    {
      if ((pMsg->pValue[0] == FALSE) || (pMsg->pValue[0] == TRUE))
      {
        /* store value in device database */
        AppDbSetPeerAddrRes(dbHdl, pMsg->pValue[0]);
      }
      else
      {
        /* invalid value */
        status = ATT_ERR_RANGE;
      }

      APP_TRACE_INFO1("Central address resolution: %d", pMsg->pValue[0]);
    }
  }
  /* handle not found in list */
  else
  {
    status = ATT_ERR_NOT_FOUND;
  }

  return status;
}
