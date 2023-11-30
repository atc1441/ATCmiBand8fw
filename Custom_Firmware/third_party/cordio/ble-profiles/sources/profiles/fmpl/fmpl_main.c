/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Find Me profile, locator role.
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
#include "app_api.h"
#include "fmpl_api.h"

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! Immediate Alert service characteristics for discovery */

/*! Alert level */
const attcDiscChar_t fmplIasAl =
{
  attAlChUuid,
  ATTC_SET_REQUIRED
};

/*! List of characteristics to be discovered; order matches handle index enumeration  */
static const attcDiscChar_t *fmplIasDiscCharList[] =
{
  &fmplIasAl                  /* Alert level */
};

/* sanity check:  make sure handle list length matches characteristic list length */
WSF_CT_ASSERT(FMPL_IAS_HDL_LIST_LEN == ((sizeof(fmplIasDiscCharList) / sizeof(attcDiscChar_t *))));

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for Immediate Alert service.  Note
 *          that pHdlList must point to an array of handles of length FMPL_IAS_HDL_LIST_LEN.
 *          If discovery is successful the handles of discovered characteristics and
 *          descriptors will be set in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void FmplIasDiscover(dmConnId_t connId, uint16_t *pHdlList)
{
  AppDiscFindService(connId, ATT_16_UUID_LEN, (uint8_t *) attIasSvcUuid,
                     FMPL_IAS_HDL_LIST_LEN, (attcDiscChar_t **) fmplIasDiscCharList, pHdlList);
}

/*************************************************************************************************/
/*!
 *  \brief  Send an immediate alert to the peer device.
 *
 *  \param  connId DM connection ID.
 *  \param  handle Attribute handle.
 *  \param  alert  Alert value.
 *
 *  \return None.
 */
/*************************************************************************************************/
void FmplSendAlert(dmConnId_t connId, uint16_t handle, uint8_t alert)
{
  uint8_t buf[1];

  if (handle != ATT_HANDLE_NONE)
  {
    buf[0] = alert;
    AttcWriteCmd(connId, handle, 1, buf);
  }
}

