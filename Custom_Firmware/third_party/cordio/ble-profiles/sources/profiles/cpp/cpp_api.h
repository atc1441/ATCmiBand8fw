/*************************************************************************************************/
/*!
*  \file
*
*  \brief  Cycling Power Profile API.
*
*  Copyright (c) 2016-2018 Arm Ltd. All Rights Reserved.
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

#ifndef CPP_API_H
#define CPP_API_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup CYCLING_POWER_PROFILE
 *  \{ */

/**************************************************************************************************
Macros
**************************************************************************************************/

/** \name Cycle Power Measurement Types
 *
 */
/**@{*/
#define CPP_PM_PARAM_INSTANTANEOUS_POWER            0     /* Instantaneous Power Measurement */
#define CPP_PM_PARAM_PEDAL_POWER                    1     /* Pedal Power Balance */
#define CPP_PM_PARAM_ACCUMULATED_TORQUE             2     /* Accumulated Torque */
#define CPP_PM_PARAM_WHEEL_REVOLUTIONS              3     /* Wheel Revolution */
#define CPP_PM_PARAM_LAST_WHEEL_REV_TIME            4     /* Last Wheel Revolution Event Time */
#define CPP_PM_PARAM_CRANK_REVOLUTIONS              5     /* Crank Revolution */
#define CPP_PM_PARAM_LAST_CRANK_TIME                6     /* Last Crank Revolution Event Time */
#define CPP_PM_PARAM_MAX_FORCE_MAGNITUDE            7     /* Max Extreme Force Magnitudes */
#define CPP_PM_PARAM_MIN_FORCE_MAGNITUDE            8     /* Min Extreme Force Magnitudes */
#define CPP_PM_PARAM_MAX_TORQUE_MAGNITUDE           9     /* Max Extreme Torque Magnitudes */
#define CPP_PM_PARAM_MIN_TORQUE_MAGNITUDE           10    /* Min Extreme Torque Magnitudes */
#define CPP_PM_PARAM_MAX_EXTREME_ANGLE              11    /* Max Extreme Angles */
#define CPP_PM_PARAM_MIN_EXTREME_ANGLE              12    /* Min Extreme Angles */
#define CPP_PM_PARAM_TOP_DEAD_SPOT                  13    /* Top Dead Spot Angle */
#define CPP_PM_PARAM_BOTTOM_DEAD_SPOT               14    /* Bottom Dead Spot Angle */
#define CPP_PM_PARAM_ACCUMULATED_ENERGY             15    /* Accumulated Energy */
/**@}*/

/**************************************************************************************************
Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Setup connection specific variables.
 *
 *  \param  connId  Connection ID
 *
 *  \return none
 */
/*************************************************************************************************/
void CppsConnOpen(dmConnId_t connId);

/*************************************************************************************************/
/*!
*  \brief  Notifies the collector of a Cycle Power Measurement.
*
*  \param  connId  Connection ID
*
*  \return none
*/
/*************************************************************************************************/
void CppsSendPowerMeasurement(dmConnId_t connId);

/*************************************************************************************************/
/*!
*  \brief  Set a cycle measurement parameter.
*
*  \param  type   Parameter identifier
*  \param  value  Measurement value.
*
*  \return none
*/
/*************************************************************************************************/
void CppsSetParameter(uint8_t type, uint32_t value);

/*************************************************************************************************/
/*!
*  \brief  Set the sensor location attribute.
*
*  \param  location   Sensor Location.
*
*  \return none
*/
/*************************************************************************************************/
void CppsSetSensorLocation(uint8_t location);

/*************************************************************************************************/
/*!
*  \brief  Set the features attribute.
*
*  \param  features   Features bitmask.
*
*  \return none
*/
/*************************************************************************************************/
void CppsSetFeatures(uint32_t features);

/*! \} */    /* CYCLING_POWER_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* CPP_API_H */
