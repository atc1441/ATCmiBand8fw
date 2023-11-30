/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Glucose profile sensor.
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
#ifndef GLPS_API_H
#define GLPS_API_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup GLUCOSE_PROFILE
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief All supported features of the glucose profile */
#define GLP_ALL_SUPPORTED_FEATURES      0x000F

/*************************************************************************************************/
/*!
 *  \brief  Initialize the Glucose profile sensor.
 *
 *  \return None.
 */
/*************************************************************************************************/
void GlpsInit(void);

/*************************************************************************************************/
/*!
 *  \brief  This function is called by the application when a message that requires
 *          processing by the glucose profile sensor is received.
 *
 *  \param  pMsg     Event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void GlpsProcMsg(wsfMsgHdr_t *pMsg);

/*************************************************************************************************/
/*!
*  \brief  Handle a button press.
*
*  \param  connId    Connection identifier.
*  \param  btn       Button press.
*
*  \return None.
*/
/*************************************************************************************************/
void GlpsBtn(dmConnId_t connId, uint8_t btn);

/*************************************************************************************************/
/*!
 *  \brief  ATTS write callback for glucose service record access control point.  Use this
 *          function as a parameter to SvcGlsCbackRegister().
 *
 *  \param  connId       DM connection identifier.
 *  \param  handle       ATT handle.
 *  \param  operation    ATT operation.
 *  \param  offset       Write offset.
 *  \param  len          Write length.
 *  \param  pValue       Value to write.
 *  \param  pAttr        Attribute to write.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t GlpsRacpWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                           uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr);

/*************************************************************************************************/
/*!
 *  \brief  Set the supported features of the glucose sensor.
 *
 *  \param  feature   Feature bitmask.
 *
 *  \return None.
 */
/*************************************************************************************************/
void GlpsSetFeature(uint16_t feature);

/*************************************************************************************************/
/*!
 *  \brief  Set the CCCD index used by the application for glucose service characteristics.
 *
 *  \param  glmCccIdx   Glucose measurement CCCD index.
 *  \param  glmcCccIdx  Glucose measurement context CCCD index.
 *  \param  racpCccIdx  Record access control point CCCD index.
 *
 *  \return None.
 */
/*************************************************************************************************/
void GlpsSetCccIdx(uint8_t glmCccIdx, uint8_t glmcCccIdx, uint8_t racpCccIdx);

/*! \} */    /* GLUCOSE_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* GLPS_API_H */
