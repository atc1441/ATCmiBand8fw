/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Asset Tracking profile client.
 *
 *  Copyright (c) 2018-2019 Arm Ltd. All Rights Reserved.
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

#ifndef ATPC_API_H
#define ATPC_API_H

#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup ASSET_TRACKING_PROFILE_CLIENT
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief Enumeration of handle indexes of characteristics to be discovered. */
enum
{
  ATPC_CTE_ENABLE_HDL,             /*!< \brief Constant Tone Extension enable. */
  ATPC_CTE_MIN_LEN_HDL,            /*!< \brief Constant Tone Extension minimum length. */
  ATPC_CTE_ADV_MIN_TX_CNT_HDL,     /*!< \brief Constant Tone Extension minimum transmit count. */
  ATPC_CTE_ADV_TX_DURATION_HDL,    /*!< \brief Constant Tone Extension transmit duration. */
  ATPC_CTE_ADV_INTERVAL_HDL,       /*!< \brief Constant Tone Extension interval. */
  ATPC_CTE_ADV_EXT_PHY_HDL,        /*!< \brief Constant Tone Extension PHY. */
  ATPC_CTE_HDL_LIST_LEN,           /*!< \brief Handle list length. */
};

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for Constant Tone Extension service.
 *          Parameter pHdlList must point to an array of length \ref ATPC_CTE_HDL_LIST_LEN.
 *          If discovery is successful the handles of discovered characteristics and
 *          descriptors will be set in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcCteDiscover(dmConnId_t connId, uint16_t *pHdlList);

/*************************************************************************************************/
/*!
 *  \brief  Write to the Constant Tone Extension enable attribute.
 *
 *  \param  connId    Connection identifier.
 *  \param  handle    Attribute handle.
 *  \param  enable    Enable.
 *  \param  cteType   CTE Type.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcCteWriteEnable(dmConnId_t connId, uint16_t handle, uint8_t enable, uint8_t cteType);

/*************************************************************************************************/
/*!
 *  \brief  Write to the Constant Tone Extension minimum length attribute.
 *
 *  \param  connId    Connection identifier.
 *  \param  handle    Attribute handle.
 *  \param  minLen    Minimum length.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcCteWriteMinLen(dmConnId_t connId, uint16_t handle, uint8_t minLen);

/*************************************************************************************************/
/*!
 *  \brief  Write to the Constant Tone Extension minimum transmit count attribute.
 *
 *  \param  connId    Connection identifier.
 *  \param  handle    Attribute handle.
 *  \param  txCount   Minimum transmit count.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcCteWriteMinTxCount(dmConnId_t connId, uint16_t handle, uint8_t txCount);

/*************************************************************************************************/
/*!
 *  \brief  Write to the Constant Tone Extension transmit duration attribute.
 *
 *  \param  connId    Connection identifier.
 *  \param  handle    Attribute handle.
 *  \param  duration  Transmit duration.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcCteWriteTxDuration(dmConnId_t connId, uint16_t handle, uint8_t duration);

/*************************************************************************************************/
/*!
 *  \brief  Write to the Constant Tone Extension interval attribute.
 *
 *  \param  connId    Connection identifier.
 *  \param  handle    Attribute handle.
 *  \param  interval  Interval.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcCteWriteInterval(dmConnId_t connId, uint16_t handle, uint16_t interval);

/*************************************************************************************************/
/*!
 *  \brief  Write to the Constant Tone Extension PHY attribute.
 *
 *  \param  connId    Connection identifier.
 *  \param  handle    Attribute handle.
 *  \param  phy       PHY.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcCteWritePhy(dmConnId_t connId, uint16_t handle, uint8_t phy);

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
void AtpcSetAntennaIds(dmConnId_t connId, uint8_t numAntenna, uint8_t *pAntennaIds);

/*************************************************************************************************/
/*!
 *  \brief  Enable AoA CTE Rx request over ACL.
 *
 *  \param  connId    Connection identifier.
 *  \param  handle    CTE enable attribute handle.
 *  \param  length    Request length.
 *  \param  interval  Request interval.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcCteAclEnableReq(dmConnId_t connId, uint16_t handle, uint8_t length, uint16_t interval,uint8_t cteType);

/*************************************************************************************************/
/*!
 *  \brief  Disable AoA CTE Rx request over ACL.
 *
 *  \param  connId    Connection identifier.
 *  \param  handle    CTE enable attribute handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcCteAclDisableReq(dmConnId_t connId, uint16_t handle);

/*************************************************************************************************/
/*!
 *  \brief  Called by application to notify the ATPC of System Events.
 *
 *  \param  pEvt   Pointer to the Event.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcProcMsg(wsfMsgHdr_t *pEvt);

/*! \} */    /* ASSET_TRACKING_PROFILE_CLIENT */

#ifdef __cplusplus
};
#endif

#endif /* ATPC_API_H */
