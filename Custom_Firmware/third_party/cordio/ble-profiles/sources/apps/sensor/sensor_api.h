/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Sensor sample application.
 *
 *  Copyright (c) 2015-2018 Arm Ltd. All Rights Reserved.
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

#ifndef SENSOR_API_H
#define SENSOR_API_H

#include "wsf_types.h"
#include "wsf_os.h"

/**************************************************************************************************
  Callback Function Types
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  App extension callback.
 *
 *  \param  pMsg    Pointer to message structure.
 *
 *  \return None.
 */
/*************************************************************************************************/
typedef void (*sensorExtCback_t)(wsfMsgHdr_t *pMsg);

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Start sensor application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SensorStart(void);

/*************************************************************************************************/
/*!
 *  \brief  WSF event handler for application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SensorHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);

/*************************************************************************************************/
/*!
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SensorHandlerInit(wsfHandlerId_t handlerId);

/*************************************************************************************************/
/*!
 *  \brief  Register a callback to receive events for the purpose of extending the sensor app.
 *
 *  \param  extCback  Callback function
 *
 *  \return None.
 */
/*************************************************************************************************/
void SensorRegisterExtensionCback(sensorExtCback_t extCback);

/**************************************************************************************************
  Application Functions
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Read gyroscope sensor.
 *
 *  \param  pX        Storage for gyroscope x-axis reading.
 *  \param  pY        Storage for gyroscope y-axis reading.
 *  \param  pZ        Storage for gyroscope z-axis reading.
 *
 *  \return None.
 */
/*************************************************************************************************/
bool_t AppReadGyro(int16_t *pX, int16_t *pY, int16_t *pZ);

/*************************************************************************************************/
/*!
 *  \brief  Read temperature sensor.
 *
 *  \param  pTemp     Storage for temperature reading.
 *
 *  \return None.
 */
/*************************************************************************************************/
bool_t AppReadTemp(int16_t *pTemp);

#endif /* SENSOR_API_H */
