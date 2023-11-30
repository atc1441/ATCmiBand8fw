/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Health Thermometer profile sensor.
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
#ifndef HTPS_API_H
#define HTPS_API_H

#include "wsf_timer.h"
#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup HEALTH_THERMOMETER_PROFILE
 *  \{ */

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! \brief Configurable parameters */
typedef struct
{
  wsfTimerTicks_t     period;     /*!< \brief Measurement timer expiration period in ms */
} htpsCfg_t;

/*************************************************************************************************/
/*!
 *  \brief  Initialize the Health Thermometer profile sensor.
 *
 *  \param  handlerId    WSF handler ID of the application using this service.
 *  \param  pCfg         Configurable parameters.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HtpsInit(wsfHandlerId_t handlerId, htpsCfg_t *pCfg);

/*************************************************************************************************/
/*!
 *  \brief  Start periodic temperature measurement.  This function starts a timer to perform
 *          periodic measurements.
 *
 *  \param  connId      DM connection identifier.
 *  \param  timerEvt    WSF event designated by the application for the timer.
 *  \param  itCccIdx    Index of intermediate temperature CCC descriptor in CCC descriptor
 *                      handle table.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HtpsMeasStart(dmConnId_t connId, uint8_t timerEvt, uint8_t itCccIdx);

/*************************************************************************************************/
/*!
 *  \brief  Stop periodic temperature measurement.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HtpsMeasStop(void);

/*************************************************************************************************/
/*!
 *  \brief  Temperature measurement complete.
 *
 *  \param  connId      DM connection identifier.
 *  \param  tmCccIdx    Index of temperature measurement CCC descriptor in CCC descriptor
 *                      handle table.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HtpsMeasComplete(dmConnId_t connId, uint8_t tmCccIdx);

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
void HtpsProcMsg(wsfMsgHdr_t *pMsg);

/*************************************************************************************************/
/*!
 *  \brief  Set the temperature measurement flags.
 *
 *  \param  flags      Temperature measurement flags.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HtpsSetTmFlags(uint8_t flags);

/*************************************************************************************************/
/*!
 *  \brief  Set the intermediate temperature flags.
 *
 *  \param  flags      Intermediate temperature flags.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HtpsSetItFlags(uint8_t flags);

/*! \} */    /* HEALTH_THERMOMETER_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* HTPS_API_H */
