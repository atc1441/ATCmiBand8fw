/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Heart Rate profile sensor.
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
#ifndef HRPS_API_H
#define HRPS_API_H

#include "wsf_timer.h"
#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup HEART_RATE_PROFILE
 *  \{ */

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! \brief Configurable parameters */
typedef struct
{
  wsfTimerTicks_t     period;     /*!< \brief Measurement timer expiration period in ms */
} hrpsCfg_t;

/*************************************************************************************************/
/*!
 *  \brief  Initialize the Heart Rate profile sensor.
 *
 *  \param  handlerId    WSF handler ID of the application using this service.
 *  \param  pCfg        Configurable parameters.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HrpsInit(wsfHandlerId_t handlerId, hrpsCfg_t *pCfg);

/*************************************************************************************************/
/*!
 *  \brief  Start periodic heart rate measurement.  This function starts a timer to perform
 *          periodic measurements.
 *
 *  \param  connId      DM connection identifier.
 *  \param  timerEvt    WSF event designated by the application for the timer.
 *  \param  hrmCccIdx   Index of heart rate CCC descriptor in CCC descriptor handle table.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HrpsMeasStart(dmConnId_t connId, uint8_t timerEvt, uint8_t hrmCccIdx);

/*************************************************************************************************/
/*!
 *  \brief  Stop periodic heart rate measurement.
 *
 *  \param  connId      DM connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HrpsMeasStop(dmConnId_t connId);

/*************************************************************************************************/
/*!
 *  \brief  Process received WSF message.
 *
 *  \param  pMsg     Event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HrpsProcMsg(wsfMsgHdr_t *pMsg);

/*************************************************************************************************/
/*!
 *  \brief  ATTS write callback for heart rate service Use this function as a parameter
 *          to SvcHrsCbackRegister().
 *
 *  \param  connId      DM connection identifier.
 *  \param  handle      ATT handle.
 *  \param  operation   ATT operation.
 *  \param  offset      Write offset.
 *  \param  len         Write length.
 *  \param  pValue      Value to write.
 *  \param  pAttr       Attribute to write.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t HrpsWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                       uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr);

/*************************************************************************************************/
/*!
 *  \brief  Set the heart rate measurement flags.
 *
 *  \param  flags      Heart rate measurement flags.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HrpsSetFlags(uint8_t flags);

/*! \} */    /* HEART_RATE_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* HRPS_API_H */
