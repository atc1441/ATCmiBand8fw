/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Application framework hardware interfaces.
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
#ifndef APP_HW_H
#define APP_HW_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup APP_FRAMEWORK_HW_API
 *  \{ */

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! \brief Heart rate measurement structure */
typedef struct
{
  uint16_t          *pRrInterval;         /*!< \brief Array of RR intervals */
  uint8_t           numIntervals;         /*!< \brief Length of RR interval array */
  uint16_t          energyExp;            /*!< \brief Energy expended value */
  uint16_t          heartRate;            /*!< \brief Heart rate */
  uint8_t           flags;                /*!< \brief Heart rate measurement flags */
} appHrm_t;

/*! \brief Date and time structure */
typedef struct
{
  uint16_t          year;                 /*!< \brief Year */
  uint8_t           month;                /*!< \brief Month */
  uint8_t           day;                  /*!< \brief Day */
  uint8_t           hour;                 /*!< \brief Hour */
  uint8_t           min;                  /*!< \brief Minutes */
  uint8_t           sec;                  /*!< \brief Seconds */
} appDateTime_t;

/*! \brief Blood pressure measurement structure */
typedef struct
{
  appDateTime_t     timestamp;            /*!< \brief Date-time */
  uint16_t          systolic;             /*!< \brief Systolic pressure */
  uint16_t          diastolic;            /*!< \brief Diastolic pressure */
  uint16_t          map;                  /*!< \brief Mean arterial pressure */
  uint16_t          pulseRate;            /*!< \brief Pulse rate */
  uint16_t          measStatus;           /*!< \brief Measurement status */
  uint8_t           flags;                /*!< \brief Flags */
  uint8_t           userId;               /*!< \brief User ID */
} appBpm_t;

/*! \brief Weight scale measurement structure */
typedef struct
{
  appDateTime_t     timestamp;            /*!< \brief Date-time */
  uint16_t          weight;               /*!< \brief Weight */
  uint8_t           flags;                /*!< \brief Weight measurement flags */
} appWsm_t;

/*! \brief Temperature measurement structure */
typedef struct
{
  appDateTime_t     timestamp;            /*!< \brief Date-time */
  uint32_t          temperature;          /*!< \brief Temperature */
  uint8_t           flags;                /*!< \brief Flags */
  uint8_t           tempType;             /*!< \brief Temperature type */
} appTm_t;

/*! \brief Pulse Oximeter continuous measurement structure */
typedef struct
{
  uint8_t         flags;            /*!< \brief Flags */
  uint16_t        spo2;             /*!< \brief SpO2PR-Spot-Check - SpO2 */
  uint16_t        pulseRate;        /*!< \brief SpO2PR-Spot-Check - Pulse Rate */
  uint16_t        spo2Fast;         /*!< \brief SpO2PR-Spot-Check Fast - SpO2 */
  uint16_t        pulseRateFast;    /*!< \brief SpO2PR-Spot-Check Fast - Pulse Rate */
  uint16_t        spo2Slow;         /*!< \brief SpO2PR-Spot-Check Slow - SpO2 */
  uint16_t        pulseRateSlow;    /*!< \brief SpO2PR-Spot-Check Slow - Pulse Rate */
  uint16_t        measStatus;       /*!< \brief Measurement Status */
  uint32_t        sensorStatus;     /*!< \brief Device and Sensor Status */
  uint16_t        pulseAmpIndex;    /*!< \brief Pulse Amplitude Index */
} appPlxCm_t;

/*! \brief Pulse Oximeter spot check measurement structure */
typedef struct
{
  uint8_t         flags;            /*!< \brief Flags */
  uint16_t        spo2;             /*!< \brief SpO2PR-Spot-Check - SpO2 */
  uint16_t        pulseRate;        /*!< \brief SpO2PR-Spot-Check - Pulse Rate */
  appDateTime_t   timestamp;        /*!< \brief Timestamp */
  uint16_t        measStatus;       /*!< \brief Measurement Status */
  uint32_t        sensorStatus;     /*!< \brief Device and Sensor Status */
  uint16_t        pulseAmpIndex;    /*!< \brief Pulse Amplitude Index */
} appPlxScm_t;

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/** \name App Hardware Interface
 * Interface to emulated sensor of real world devices (e.g. battery, heart rate monitor,
 * blood pressure sensor, etc.)
 */
/**@{*/

/*************************************************************************************************/
/*!
 *  \brief  Read the battery level.  The battery level value returned in pLevel is the
 *          percentage of remaining battery capacity (0-100%).
 *
 *  \param  pLevel   Battery level return value.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppHwBattRead(uint8_t *pLevel);

/*************************************************************************************************/
/*!
 *  \brief  Set the battery level, for test purposes.
 *
 *  \param  level   Battery level (0-100%).
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppHwBattTest(uint8_t level);

/*************************************************************************************************/
/*!
 *  \brief  Perform a heart rate measurement.  Return the heart rate along with any RR interval
 *          data.
 *
 *  \param  pHrm   Heart rate measurement return value.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppHwHrmRead(appHrm_t *pHrm);

/*************************************************************************************************/
/*!
 *  \brief  Set the heart rate, for test purposes.
 *
 *  \param  heartRate Heart rate.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppHwHrmTest(uint8_t heartRate);

/*************************************************************************************************/
/*!
 *  \brief  Perform a blood pressure measurement.  Return the measurement data.
 *
 *  \param  intermed  TRUE if this is an intermediate measurement.
 *  \param  pBpm      Blood pressure measurement return value.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppHwBpmRead(bool_t intermed, appBpm_t *pBpm);

/*************************************************************************************************/
/*!
 *  \brief  Perform a weight scale measurement.  Return the measurement data.
 *
 *  \param  pWsm      Weight scale measurement return value.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppHwWsmRead(appWsm_t *pWsm);

/*************************************************************************************************/
/*!
 *  \brief  Perform a temperature measurement.  Return the measurement data.
 *
 *  \param  intermed  TRUE if this is an intermediate measurement.
 *  \param  pTm      Temperature measurement return value.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppHwTmRead(bool_t intermed, appTm_t *pTm);

/*************************************************************************************************/
/*!
 *  \brief  Set the temperature measurement units.
 *
 *  \param  units     CH_TM_FLAG_UNITS_C or CH_TM_FLAG_UNITS_F.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppHwTmSetUnits(uint8_t units);


/*************************************************************************************************/
/*!
 *  \brief  Set the weight measurement units.
 *
 *  \param  units     CH_WSM_FLAG_UNITS_KG or CH_WSM_FLAG_UNITS_LBS.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppHwWmSetUnits(uint8_t units);

/*************************************************************************************************/
/*!
 *  \brief  Perform a pulse oximeter measurement.
 *
 *  \param  pPlxcm   Pulse Oximeter measurement return value.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppHwPlxcmRead(appPlxCm_t *pPlxcm);


/*************************************************************************************************/
/*!
 *  \brief  Perform a pulse oximeter spot check measurement.
 *
 *  \param  pPlxscm   Pulse Oximeter measurement return value.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppHwPlxscmRead(appPlxScm_t *pPlxscm);

/**@}*/

/*! \} */    /*! APP_FRAMEWORK_HW_API */

#ifdef __cplusplus
};
#endif

#endif /* APP_HW_H */
