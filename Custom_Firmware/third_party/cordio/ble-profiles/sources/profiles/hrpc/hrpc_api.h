/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Heart Rate profile client.
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
#ifndef HRPC_API_H
#define HRPC_API_H

#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup HEART_RATE_PROFILE
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief Heart Rate service enumeration of handle indexes of characteristics to be discovered */
enum
{
  HRPC_HRS_HRM_HDL_IDX,           /*!< \brief Heart rate measurement */
  HRPC_HRS_HRM_CCC_HDL_IDX,       /*!< \brief Heart rate measurement CCC descriptor */
  HRPC_HRS_BSL_HDL_IDX,           /*!< \brief Body sensor location */
  HRPC_HRS_HRCP_HDL_IDX,          /*!< \brief Heart rate control point */
  HRPC_HRS_HDL_LIST_LEN           /*!< \brief Handle list length */
};

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for Heart Rate service.
 *          Parameter pHdlList must point to an array of length \ref HRPC_HRS_HDL_LIST_LEN.
 *          If discovery is successful the handles of discovered characteristics and
 *          descriptors will be set in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HrpcHrsDiscover(dmConnId_t connId, uint16_t *pHdlList);

/*************************************************************************************************/
/*!
 *  \brief  Send a command to the heart rate control point.
 *
 *  \param  connId    Connection identifier.
 *  \param  handle    Attribute handle.
 *  \param  command   Control point command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HrpcHrsControl(dmConnId_t connId, uint16_t handle, uint8_t command);

/*************************************************************************************************/
/*!
 *  \brief  Process a value received in an ATT read response, notification, or indication
 *          message.  Parameter pHdlList must point to an array of length \ref HRPC_HRS_HDL_LIST_LEN.
 *          If the ATT handle of the message matches a handle in the handle list the value
 *          is processed, otherwise it is ignored.
 *
 *  \param  pHdlList  Characterist handle list.
 *  \param  pMsg      ATT callback message.
 *
 *  \return \ref ATT_SUCCESS if handle is found, \ref ATT_ERR_NOT_FOUND otherwise.
 */
/*************************************************************************************************/
uint8_t HrpcHrsValueUpdate(uint16_t *pHdlList, attEvt_t *pMsg);

/*! \} */    /* HEART_RATE_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* HRPC_API_H */
