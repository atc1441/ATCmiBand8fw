/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  User Data Service Collector.
 *
 *  Copyright (c) 2017-2018 Arm Ltd. All Rights Reserved.
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
#ifndef UDSC_API_H
#define UDSC_API_H

#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup USER_DATA_PROFILE
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief UDSC service enumeration of handle indexes of characteristics to be discovered */
enum
{
  UDSC_DBCI_HDL_IDX,               /*!< \brief Database Change Interval */
  UDSC_DCBI_CCC_HDL_IDX,           /*!< \brief Database Change Interval CCC descriptor */
  UDSC_UI_HDL_IDX,                 /*!< \brief User Index */
  UDSC_UCP_IDX,                    /*!< \brief User Control Point */
  UDSC_UCP_CCC_HDL_IDX,            /*!< \brief User Control Point CCC descriptor */
  UDSC_HDL_LIST_LEN                /*!< \brief Handle list length */
};

/** \name User Control Point Opcodes
 *
 */
/**@{*/
#define UDSC_UCP_OPCODE_RNU              0x01    /*!< \brief Register New User */
#define UDSC_UCP_OPCODE_CONSENT          0x02    /*!< \brief Consent */
#define UDSC_UCP_OPCODE_DUD              0x03    /*!< \brief Delete User Data */
#define UDSC_UCP_OPCODE_RESPONSE         0x20    /*!< \brief Command Response */
/**@}*/

/** \name User Control Point Response Values
 *
 */
/**@{*/
#define UDSC_UCP_RSP_SUCCESS             0x01    /*!< \brief Success */
#define UDSC_UCP_RSP_OP_NOT_SUPPORTED    0x02    /*!< \brief Opcode not supported */
#define UDSC_UCP_RSP_INVALID_PARAMETER   0x03    /*!< \brief Invalid Parameter */
#define UDSC_UCP_RSP_OP_FAILED           0x04    /*!< \brief Operation Failed */
#define UDSC_UCP_RSP_NOT_AUTHORIZED      0x05    /*!< \brief User Not Authorized */
/**@}*/

/**************************************************************************************************
  Callback Function Datatypes
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  UDS Control Point Response Callback.
 *
 *  \param  connId      Connection ID.
 *  \param  opcode      Cmd opcode being responded to.
 *  \param  response    Response code.
 *  \param  index       User index (only set when opcode is \ref UDSC_UCP_OPCODE_RNU)
 *
 *  \return None
 */
/*************************************************************************************************/
typedef void (*UdsRspCback_t)(dmConnId_t connId, uint8_t opcode, uint8_t response, uint8_t index);

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for User Data service.  Parameter
 *          pHdlList must point to an array of length \ref UDSC_HDL_LIST_LEN.  If discovery is
 *          successful the handles of discovered characteristics and descriptors will be set
 *          in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UdscDiscover(dmConnId_t connId, uint16_t *pHdlList);

/*************************************************************************************************/
/*!
 *  \brief  Process a value received in an ATT read response, notification, or indication
 *          message.  Parameter pHdlList must point to an array of length \ref UDSC_HDL_LIST_LEN.
 *          If the ATT handle of the message matches a handle in the handle list the value
 *          is processed, otherwise it is ignored.
 *
 *  \param  pHdlList  Characteristic handle list.
 *  \param  pMsg      ATT callback message.
 *
 *  \return \ref ATT_SUCCESS if handle is found, \ref ATT_ERR_NOT_FOUND otherwise.
 */
/*************************************************************************************************/
uint8_t UdscValueUpdate(uint16_t *pHdlList, attEvt_t *pMsg);

/*************************************************************************************************/
/*!
 *  \brief  Read the user index characteristic.
 *
 *  \param  connId      Connection identifier.
 *  \param  handle      Attribute handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UdscReadUserIndex(dmConnId_t connId, uint16_t handle);

/*************************************************************************************************/
/*!
 *  \brief  Read the database change increment characteristic.
 *
 *  \param  connId      Connection identifier.
 *  \param  handle      Attribute handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UdscReadDatabaseChangeIncrement(dmConnId_t connId, uint16_t handle);

/*************************************************************************************************/
/*!
 *  \brief  Write the database change increment characteristic.
 *
 *  \param  connId      Connection identifier.
 *  \param  handle      Attribute handle.
 *  \param  increment   DB Change Increment
 *
 *  \return None.
 */
/*************************************************************************************************/
void UdscWriteDatabaseChangeIncrement(dmConnId_t connId, uint16_t handle, uint32_t increment);

/*************************************************************************************************/
/*!
 *  \brief  Write to the user control point characteristic - Register New User.
 *
 *  \param  connId        Connection identifier.
 *  \param  handle        Attribute handle.
 *  \param  consentCode   Consent code (0-9999)
 *
 *  \return None.
 */
/*************************************************************************************************/
void UdscRegisterNewUser(dmConnId_t connId, uint16_t handle, uint16_t consentCode);

/*************************************************************************************************/
/*!
 *  \brief  Write to the user control point characteristic - Consent.
 *
 *  \param  connId        Connection identifier.
 *  \param  handle        Attribute handle.
 *  \param  index         User Index
 *  \param  consentCode   Consent code (0-9999 - provided when user was registered)
 *
 *  \return None.
 */
/*************************************************************************************************/
void UdscConsent(dmConnId_t connId, uint16_t handle, uint8_t index, uint16_t consentCode);

/*************************************************************************************************/
/*!
 *  \brief  Write to the user control point characteristic - Delete User Data.
 *
 *  \param  connId        Connection identifier.
 *  \param  handle        Attribute handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UdscDeleteUserData(dmConnId_t connId, uint16_t handle);

/*************************************************************************************************/
/*!
 *  \brief  Called by the application when a connection closes.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UdscClose(void);

/*************************************************************************************************/
/*!
 *  \brief  Initialize User Data Service collector callbacks.
 *
 *  \param  handlerId      Application task handler ID.
 *  \param  timerEvent     Application timer event for control point timeout.
 *  \param  rspCback       Callback to receive control point response messages.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UdscInit(wsfHandlerId_t handlerId, uint8_t timerEvent, UdsRspCback_t rspCback);

/*! \} */    /* USER_DATA_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* UDSC_API_H */
