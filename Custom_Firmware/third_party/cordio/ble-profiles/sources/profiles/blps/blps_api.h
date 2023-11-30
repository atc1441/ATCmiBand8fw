/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Blood Pressure profile sensor.
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
#ifndef BLPS_API_H
#define BLPS_API_H

#include "wsf_timer.h"
#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup BLOOD_PRESSURE_PROFILE
 *  \{ */

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! \brief Configurable parameters */
typedef struct
{
  wsfTimerTicks_t     period;     /*!< \brief Measurement timer expiration period in ms */
} blpsCfg_t;

/*************************************************************************************************/
/*!
 *  \brief  Initialize the Blood Pressure profile sensor.
 *
 *  \param  handlerId   WSF handler ID of the application using this service.
 *  \param  pCfg        Configurable parameters.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BlpsInit(wsfHandlerId_t handlerId, blpsCfg_t *pCfg);

/*************************************************************************************************/
/*!
 *  \brief  Start periodic blood pressure measurement.  This function starts a timer to perform
 *          periodic measurements.
 *
 *  \param  connId      DM connection identifier.
 *  \param  timerEvt    WSF event designated by the application for the timer.
 *  \param  icpCccIdx   Index of intermediate cuff pressure CCC descriptor in CCC descriptor
 *                      handle table.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BlpsMeasStart(dmConnId_t connId, uint8_t timerEvt, uint8_t icpCccIdx);

/*************************************************************************************************/
/*!
 *  \brief  Stop periodic blood pressure measurement.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BlpsMeasStop(void);

/*************************************************************************************************/
/*!
 *  \brief  Blood pressure measurement complete.
 *
 *  \param  connId      DM connection identifier.
 *  \param  bpmCccIdx   Index of blood pressure measurement CCC descriptor in CCC descriptor
 *                      handle table.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BlpsMeasComplete(dmConnId_t connId, uint8_t bpmCccIdx);

/*************************************************************************************************/
/*!
 *  \brief  This function is called by the application when the periodic measurement
 *          timer expires.
 *
 *  \param  pMsg     Event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BlpsProcMsg(wsfMsgHdr_t *pMsg);

/*************************************************************************************************/
/*!
 *  \brief  Set the blood pressure measurement flags.
 *
 *  \param  flags      Blood pressure measurement flags.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BlpsSetBpmFlags(uint8_t flags);

/*************************************************************************************************/
/*!
 *  \brief  Set the intermediate cuff pressure flags.
 *
 *  \param  flags      Intermediate cuff pressure flags.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BlpsSetIcpFlags(uint8_t flags);

/*! \} */    /* BLOOD_PRESSURE_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* BLPS_API_H */
