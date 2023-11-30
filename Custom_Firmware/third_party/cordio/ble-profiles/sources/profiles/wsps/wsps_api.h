/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Weight Scale profile sensor.
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
#ifndef WSPS_API_H
#define WSPS_API_H

#include "wsf_timer.h"
#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup WEIGHT_SCALE_PROFILE
 *  \{ */

/*************************************************************************************************/
/*!
 *  \brief  Weight scale measurement complete.
 *
 *  \param  connId      DM connection identifier.
 *  \param  wsmCccIdx   Index of weight scale measurement CCC descriptor in CCC descriptor
 *                      handle table.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WspsMeasComplete(dmConnId_t connId, uint8_t wsmCccIdx);

/*************************************************************************************************/
/*!
 *  \brief  Set the weight scale measurement flags.
 *
 *  \param  flags      Weight scale measurement flags.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WspsSetWsmFlags(uint8_t flags);

/*! \} */    /* WEIGHT_SCALE_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* WSPS_API_H */
