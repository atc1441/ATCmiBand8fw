/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Asset Tracking profile server.
 *
 *  Copyright (c) 2018 Arm Ltd. All Rights Reserved.
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
#ifndef ATPS_API_H
#define ATPS_API_H

#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup ASSET_TRACKING_PROFILE_SERVER
 *  \{ */

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Asset Tracking Profile server initialization.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpsInit(void);

/*************************************************************************************************/
/*!
 *  \brief  Set the antenna identifiers for a connection ID.
 *
 *  \param  connId        Connection identifier.
 *  \param  numAntenna    Number of antenna and len of pAntennaIds in bytes.
 *  \param  pAntennaIds   Array containing identifiers of antenna for this connection.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpsSetAntennaIds(dmConnId_t connId, uint8_t numAntenna, uint8_t *pAntennaIds);

/*************************************************************************************************/
/*!
 *  \brief  Called by application to notify the Asset Tracking Profile server of DM Events.
 *
 *  \param  pEvt   Pointer to the DM Event.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpsProcDmMsg(dmEvt_t *pEvt);

/*! \} */    /* ASSET_TRACKING_PROFILE_SERVER */

#ifdef __cplusplus
};
#endif

#endif /* ATPS_API_H */
