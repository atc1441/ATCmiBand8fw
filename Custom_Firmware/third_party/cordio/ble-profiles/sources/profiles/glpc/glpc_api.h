/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Glucose profile collector.
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
#ifndef GLPC_API_H
#define GLPC_API_H

#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup GLUCOSE_PROFILE
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief Glucose service enumeration of handle indexes of characteristics to be discovered */
enum
{
  GLPC_GLS_GLM_HDL_IDX,           /*!< \brief Glucose measurement */
  GLPC_GLS_GLM_CCC_HDL_IDX,       /*!< \brief Glucose measurement CCC descriptor */
  GLPC_GLS_GLMC_HDL_IDX,          /*!< \brief Glucose measurement context */
  GLPC_GLS_GLMC_CCC_HDL_IDX,      /*!< \brief Glucose measurement context CCC descriptor */
  GLPC_GLS_GLF_HDL_IDX,           /*!< \brief Glucose feature */
  GLPC_GLS_RACP_HDL_IDX,          /*!< \brief Record access control point */
  GLPC_GLS_RACP_CCC_HDL_IDX,      /*!< \brief Record access control point CCC descriptor */
  GLPC_GLS_HDL_LIST_LEN           /*!< \brief Handle list length */
};

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! \brief Glucose service RACP filter type */
typedef struct
{
  union
  {
    uint16_t      seqNum;         /*!< \brief Sequence number filter */
  } param;                        /*!< \brief Parameter union */
  uint8_t         type;           /*!< \brief Filter type */
} glpcFilter_t;

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for Glucose service.
 *          Parameter pHdlList must point to an array of length \ref GLPC_GLS_HDL_LIST_LEN.
 *          If discovery is successful the handles of discovered characteristics and
 *          descriptors will be set in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void GlpcGlsDiscover(dmConnId_t connId, uint16_t *pHdlList);

/*************************************************************************************************/
/*!
 *  \brief  Process a value received in an ATT read response, notification, or indication
 *          message.  Parameter pHdlList must point to an array of length \ref GLPC_GLS_HDL_LIST_LEN.
 *          If the ATT handle of the message matches a handle in the handle list the value
 *          is processed, otherwise it is ignored.
 *
 *  \param  pHdlList  Characteristic handle list.
 *  \param  pMsg      ATT callback message.
 *
 *  \return \ref ATT_SUCCESS if handle is found, \ref ATT_ERR_NOT_FOUND otherwise.
 */
/*************************************************************************************************/
uint8_t GlpcGlsValueUpdate(uint16_t *pHdlList, attEvt_t *pMsg);

/*************************************************************************************************/
/*!
 *  \brief  Send a command to the glucose service record access control point.
 *
 *  \param  connId  Connection identifier.
 *  \param  handle  Attribute handle.
 *  \param  opcode  Command opcode.
 *  \param  oper    Command operator or 0 if no operator required.
 *  \param  pFilter Command filter parameters or NULL of no parameters required.
 *
 *  \return None.
 */
/*************************************************************************************************/
void GlpcGlsRacpSend(dmConnId_t connId, uint16_t handle, uint8_t opcode, uint8_t oper,
                     glpcFilter_t *pFilter);

/*************************************************************************************************/
/*!
 *  \brief  Set the last received glucose measurement sequence number.
 *
 *  \param  seqNum   Glucose measurement sequence number.
 *
 *  \return None.
 */
/*************************************************************************************************/
void GlpcGlsSetLastSeqNum(uint16_t seqNum);

/*************************************************************************************************/
/*!
 *  \brief  Get the last received glucose measurement sequence number.
 *
 *  \return Last received glucose measurement sequence number.
 */
/*************************************************************************************************/
uint16_t GlpcGlsGetLastSeqNum(void);

/*! \} */    /* GLUCOSE_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* GLPC_API_H */
