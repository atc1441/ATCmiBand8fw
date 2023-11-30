/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Scan Parameter Profile Server.
 *
 *  Copyright (c) 2016-2018 Arm Ltd. All Rights Reserved.
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
#include "scpps_api.h"
#include "svc_scpss.h"

/**************************************************************************************************
  Local Variables
**************************************************************************************************/
static ScppsAppCback_t *scppsAppCback;

/*************************************************************************************************/
/*!
*  \brief  Called to register an application callback function
*
*  \param  cback    Application interval window callback
*
*  \return Status
*/
/*************************************************************************************************/
void ScppsRegisterCback(ScppsAppCback_t *cback)
{
  scppsAppCback = cback;
}

/*************************************************************************************************/
/*!
 *  \brief  Called when the peer writes to SCPPS attributes
 *
 *  \return Status
 */
/*************************************************************************************************/
uint8_t ScppsAttsWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                            uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr)
{
  if (scppsAppCback && handle == SCPSS_SIW_HDL)
  {
    uint16_t interval;
    uint16_t window;

    BSTREAM_TO_UINT16(interval, pValue);
    BSTREAM_TO_UINT16(window, pValue);

    /* Call the callback to the application layer */
    (*scppsAppCback)(connId, interval, window);
  }

  return 0;
}
