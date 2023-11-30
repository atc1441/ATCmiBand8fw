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
#ifndef FMPL_API_H
#define FMPL_API_H

#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup FIND_ME_PROFILE
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief Enumeration of handle indexes of characteristics to be discovered for immediate alert service */
enum
{
  FMPL_IAS_AL_HDL_IDX,          /*!< \brief Alert level */
  FMPL_IAS_HDL_LIST_LEN         /*!< \brief Handle list length */
};

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for Immediate Alert service.  Note
 *          that pHdlList must point to an array of handles of length \ref FMPL_IAS_HDL_LIST_LEN.
 *          If discovery is successful the handles of discovered characteristics and
 *          descriptors will be set in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void FmplIasDiscover(dmConnId_t connId, uint16_t *pHdlList);

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
void FmplSendAlert(dmConnId_t connId, uint16_t handle, uint8_t alert);

/*! \} */    /* FIND_ME_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* FMPL_API_H */
