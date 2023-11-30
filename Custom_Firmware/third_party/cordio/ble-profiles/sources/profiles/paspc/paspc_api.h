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
#ifndef PASPC_API_H
#define PASPC_API_H

#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup PHONE_ALERT_STATUS_PROFILE
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief Phone Alert Status service enumeration of handle indexes of characteristics to be discovered */
enum
{
  PASPC_PASS_AS_HDL_IDX,              /*!< \brief Alert status */
  PASPC_PASS_AS_CCC_HDL_IDX,          /*!< \brief Alert status CCC descriptor */
  PASPC_PASS_RS_HDL_IDX,              /*!< \brief Ringer setting */
  PASPC_PASS_RS_CCC_HDL_IDX,          /*!< \brief Ringer setting CCC descriptor */
  PASPC_PASS_RCP_HDL_IDX,             /*!< \brief Ringer control point */
  PASPC_PASS_HDL_LIST_LEN             /*!< \brief Handle list length */
};

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for Phone Alert Status  service.
 *          Parameter pHdlList must point to an array of length \ref PASPC_PASS_HDL_LIST_LEN.
 *          If discovery is successful the handles of discovered characteristics and
 *          descriptors will be set in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void PaspcPassDiscover(dmConnId_t connId, uint16_t *pHdlList);

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
void PaspcPassControl(dmConnId_t connId, uint16_t handle, uint8_t command);

/*************************************************************************************************/
/*!
 *  \brief  Process a value received in an ATT read response, notification, or indication
 *          message.  Parameter pHdlList must point to an array of length \ref PASPC_PASS_HDL_LIST_LEN.
 *          If the attribute handle of the message matches a handle in the handle list the value
 *          is processed, otherwise it is ignored.
 *
 *  \param  pHdlList  Characteristic handle list.
 *  \param  pMsg      ATT callback message.
 *
 *  \return \ref ATT_SUCCESS if handle is found, \ref ATT_ERR_NOT_FOUND otherwise.
 */
/*************************************************************************************************/
uint8_t PaspcPassValueUpdate(uint16_t *pHdlList, attEvt_t *pMsg);

/*! \} */    /* PHONE_ALERT_STATUS_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* PASPC_API_H */
