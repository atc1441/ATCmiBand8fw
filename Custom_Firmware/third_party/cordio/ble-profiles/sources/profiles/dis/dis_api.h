/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Device information service client.
 *
 *  Copyright (c) 2012-2020 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2019-2020 Packetcraft, Inc.
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
#ifndef DIS_API_H
#define DIS_API_H

#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup DEVICE_INFORMATION_PROFILE
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief Enumeration of handle indexes of characteristics to be discovered */
enum
{
  DIS_MFNS_HDL_IDX,         /*!< \brief Manufacturer name string */
  DIS_MNS_HDL_IDX,          /*!< \brief Model number string */
  DIS_SNS_HDL_IDX,          /*!< \brief Serial number string */
  DIS_HRS_HDL_IDX,          /*!< \brief Hardware revision string */
  DIS_FRS_HDL_IDX,          /*!< \brief Firmware revision string */
  DIS_SRS_HDL_IDX,          /*!< \brief Software revision string */
  DIS_SID_HDL_IDX,          /*!< \brief System ID */
  DIS_RCD_HDL_IDX,          /*!< \brief Registration certificate data */
  DIS_PNP_ID_HDL_IDX,       /*!< \brief PnP ID */
  DIS_HDL_LIST_LEN          /*!< \brief Handle list length */
};

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for DIS service.  Note that pHdlList
 *          must point to an array of handles of length \ref DIS_HDL_LIST_LEN.  If discovery is
 *          successful the handles of discovered characteristics and descriptors will be set
 *          in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void DisDiscover(dmConnId_t connId, uint16_t *pHdlList);

/*************************************************************************************************/
/*!
 *  \brief  Process a value received in an ATT read response, notification, or indication
 *          message.  Parameter pHdlList must point to an array of length \ref DIS_HDL_LIST_LEN.
 *          If the attribute handle of the message matches a handle in the handle list the value
 *          is processed, otherwise it is ignored.
 *
 *  \param  pHdlList  Characteristic handle list.
 *  \param  pMsg      ATT callback message.
 *
 *  \return \ref ATT_SUCCESS if handle is found, \ref ATT_ERR_NOT_FOUND otherwise.
 */
/*************************************************************************************************/
uint8_t DisValueUpdate(uint16_t *pHdlList, attEvt_t *pMsg);

/*! \} */    /* DEVICE_INFORMATION_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* DIS_API_H */
