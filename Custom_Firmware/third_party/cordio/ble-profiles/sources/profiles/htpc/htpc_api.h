/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Health Thermometer profile client.
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
#ifndef HTPC_API_H
#define HTPC_API_H

#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup HEALTH_THERMOMETER_PROFILE
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief Health Thermometer service enumeration of handle indexes of characteristics to be discovered */
enum
{
  HTPC_HTS_TM_HDL_IDX,            /*!< \brief Temperature measurement */
  HTPC_HTS_TM_CCC_HDL_IDX,        /*!< \brief Temperature measurement CCC descriptor */
  HTPC_HTS_IT_HDL_IDX,            /*!< \brief Intermediate temperature */
  HTPC_HTS_IT_CCC_HDL_IDX,        /*!< \brief Intermediate temperature CCC descriptor */
  HTPC_HTS_TT_HDL_IDX,            /*!< \brief Temperature type */
  HTPC_HTS_HDL_LIST_LEN           /*!< \brief Handle list length */
};

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for Health Thermometer service.
 *          Parameter pHdlList must point to an array of length \ref HTPC_HTS_HDL_LIST_LEN.
 *          If discovery is successful the handles of discovered characteristics and
 *          descriptors will be set in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HtpcHtsDiscover(dmConnId_t connId, uint16_t *pHdlList);

/*************************************************************************************************/
/*!
 *  \brief  Process a value received in an ATT read response, notification, or indication
 *          message.  Parameter pHdlList must point to an array of length \ref HTPC_HTS_HDL_LIST_LEN.
 *          If the ATT handle of the message matches a handle in the handle list the value
 *          is processed, otherwise it is ignored.
 *
 *  \param  pHdlList  Characteristc handle list.
 *  \param  pMsg      ATT callback message.
 *
 *  \return \ref ATT_SUCCESS if handle is found, \ref ATT_ERR_NOT_FOUND otherwise.
 */
/*************************************************************************************************/
uint8_t HtpcHtsValueUpdate(uint16_t *pHdlList, attEvt_t *pMsg);

/*! \} */    /* HEALTH_THERMOMETER_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* HTPC_API_H */
