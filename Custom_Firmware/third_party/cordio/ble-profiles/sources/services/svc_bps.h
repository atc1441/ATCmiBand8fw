/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Blood Pressure service implementation.
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

#ifndef SVC_BPS_H
#define SVC_BPS_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup BLOOD_PRESSURE_SERVICE
 *  \{ */

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name Blood Pressure Service Handles
 *
 */
/**@{*/
#define BPS_START_HDL               0x00F0            /*!< \brief Start handle. */
#define BPS_END_HDL                 (BPS_MAX_HDL - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief Blood Pressure Service Handles */
enum
{
  BPS_SVC_HDL = BPS_START_HDL,      /*!< \brief Blood pressure service declaration */
  BPS_BPM_CH_HDL,                   /*!< \brief Blood pressure measurement characteristic */
  BPS_BPM_HDL,                      /*!< \brief Blood pressure measurement */
  BPS_BPM_CH_CCC_HDL,               /*!< \brief Blood pressure measurement client characteristic configuration */
  BPS_ICP_CH_HDL,                   /*!< \brief Intermediate cuff pressure characteristic */
  BPS_ICP_HDL,                      /*!< \brief Intermediate cuff pressure */
  BPS_ICP_CH_CCC_HDL,               /*!< \brief Intermediate cuff pressure client characteristic configuration */
  BPS_BPF_CH_HDL,                   /*!< \brief Blood pressure feature characteristic */
  BPS_BPF_HDL,                      /*!< \brief Blood pressure feature */
  BPS_MAX_HDL                       /*!< \brief Maximum handle. */
};
/**@}*/

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcBpsAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcBpsRemoveGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Register callbacks for the service.
 *
 *  \param  readCback   Read callback function.
 *  \param  writeCback  Write callback function.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcBpsCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback);

/*************************************************************************************************/
/*!
 *  \brief  Toggle flag values.
 *
 *  \param  flag  Flag to manipulate.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcBpsToggleFeatureFlags(uint8_t flag);

/*! \} */    /* BLOOD_PRESSURE_SERVICE */

#ifdef __cplusplus
};
#endif

#endif /* SVC_BPS_H */
