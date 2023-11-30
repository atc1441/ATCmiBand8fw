/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Arm Ltd. proprietary profile client.
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

#include <string.h>
#include "wsf_types.h"
#include "wsf_assert.h"
#include "util/bstream.h"
#include "app_api.h"
#include "wpc_api.h"

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*!
 *  Arm Ltd. proprietary service P1
 */

/* UUIDs */
static const uint8_t wpcP1SvcUuid[] = {ATT_UUID_P1_SERVICE};    /*! Proprietary service P1 */
static const uint8_t wpcD1ChUuid[] = {ATT_UUID_D1_DATA};        /*! Proprietary data D1 */

/* Characteristics for discovery */

/*! Proprietary data */
static const attcDiscChar_t wpcP1Dat =
{
  wpcD1ChUuid,
  ATTC_SET_REQUIRED | ATTC_SET_UUID_128
};

/*! Proprietary data descriptor */
static const attcDiscChar_t wpcP1datCcc =
{
  attCliChCfgUuid,
  ATTC_SET_REQUIRED | ATTC_SET_DESCRIPTOR
};

/*! List of characteristics to be discovered; order matches handle index enumeration  */
static const attcDiscChar_t *wpcP1DiscCharList[] =
{
  &wpcP1Dat,                  /*! Proprietary data */
  &wpcP1datCcc                /*! Proprietary data descriptor */
};

/* sanity check:  make sure handle list length matches characteristic list length */
WSF_CT_ASSERT(WPC_P1_HDL_LIST_LEN == ((sizeof(wpcP1DiscCharList) / sizeof(attcDiscChar_t *))));

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for Arm Ltd. proprietary service P1.
 *          Parameter pHdlList must point to an array of length WPC_P1_HDL_LIST_LEN.
 *          If discovery is successful the handles of discovered characteristics and
 *          descriptors will be set in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WpcP1Discover(dmConnId_t connId, uint16_t *pHdlList)
{
  AppDiscFindService(connId, ATT_128_UUID_LEN, (uint8_t *) wpcP1SvcUuid,
                     WPC_P1_HDL_LIST_LEN, (attcDiscChar_t **) wpcP1DiscCharList, pHdlList);
}

