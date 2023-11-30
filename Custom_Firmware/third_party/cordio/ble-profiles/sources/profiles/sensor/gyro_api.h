/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example gyroscope service profile.
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

#ifndef GYRO_API_H
#define GYRO_API_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup GYROSCOPE_SERVICE_PROFILE
 *  \{ */

#include "wsf_types.h"
#include "wsf_os.h"

/*************************************************************************************************/
/*!
 *  \brief  Start service.
 *
 *  \param  handlerId       Handler ID.
 *  \param  timerEvt        Timer message event.
 *
 *  \return None.
 */
/*************************************************************************************************/
void GyroStart(wsfHandlerId_t handlerId, uint8_t timerEvt);

/*************************************************************************************************/
/*!
 *  \brief  Stop service.
 *
 *  \return None.
 */
/*************************************************************************************************/
void GyroStop(void);

/*************************************************************************************************/
/*!
 *  \brief  Measurement stop handler.
 *
 *  \return None.
 */
/*************************************************************************************************/
void GyroMeasStop(void);

/*************************************************************************************************/
/*!
 *  \brief  Measurement start handler.
 *
 *  \return None.
 */
/*************************************************************************************************/
void GyroMeasStart(void);

/*************************************************************************************************/
/*!
 *  \brief  Measurement complete handler.
 *
 *  \param  connId    Connection ID.
 *  \param  x         Gyroscope x-axis reading.
 *  \param  y         Gyroscope y-axis reading.
 *  \param  z         Gyroscope z-axis reading.
 *
 *  \return None.
 */
/*************************************************************************************************/
void GyroMeasComplete(dmConnId_t connId, int16_t x, int16_t y, int16_t z);

/*! \} */    /* GYROSCOPE_SERVICE_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* GYRO_API_H */
