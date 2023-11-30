/*************************************************************************************************/
/*!
*  \file
*
*  \brief  Scan Parameter Profile Server Application Interface.
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

#ifndef SCPPS_API_H
#define SCPPS_API_H

#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup SCAN_PARAMETER_PROFILE
 *  \{ */

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*! \brief Application interval window callback */
typedef void ScppsAppCback_t(dmConnId_t connId, uint16_t interval, uint16_t window);

/*************************************************************************************************/
/*!
 *  \brief  Called to register an application scan interval window callback function
 *
 *  \param  cback    Application interval window callback
 *
 *  \return Status
 */
/*************************************************************************************************/
void ScppsRegisterCback(ScppsAppCback_t *cback);

/*************************************************************************************************/
/*!
 *  \brief  Called when the peer writes to SCPPS attributes
 *
 *  \param  connId      DM connection identifier.
 *  \param  handle      ATT handle.
 *  \param  operation   ATT operation.
 *  \param  offset      Write offset.
 *  \param  len         Write length.
 *  \param  pValue      Value to write.
 *  \param  pAttr       Attribute to write.
 *
 *  \return Status
 */
/*************************************************************************************************/
uint8_t ScppsAttsWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                            uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr);

/*! \} */    /* SCAN_PARAMETER_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* SCPPS_API_H */
