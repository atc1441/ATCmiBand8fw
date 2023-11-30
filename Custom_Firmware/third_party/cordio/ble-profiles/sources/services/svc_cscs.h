/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Cycling Speed and Cadence Service Server implementation.
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

#ifndef SVC_CSCS_H
#define SVC_CSCS_H

#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup CYCLING_SPEED_AND_CADENCE_SERVICE
 *  \{ */

/**************************************************************************************************
Constants
**************************************************************************************************/

/** \name CSC Feature Bits
 * Cycle Speed and Cadence Feature Bits of the Feature Characteristic.
 */
/**@{*/

#define CSCS_WRDS_FEATURE_BIT            (1<<0)  /*!< \brief Wheel Revolution Data Supported Feature Mask Bit */
#define CSCS_CRDS_FEATURE_BIT            (1<<1)  /*!< \brief Crank Revolution Data Supported Feature Mask Bit */
#define CSCS_MSLS_FEATURE_BIT            (1<<2)  /*!< \brief Multiple Sensor Locations Supporte Feature Mask Bit */

#define CSCS_ALL_FEATURES                (0x3)   /*!< \brief All Supported Feature Mask */
/**@}*/

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name Cycling Speed and Cadence Service Handles
 *
 */
/**@{*/
#define CSCS_START_HDL               0x0450             /*!< \brief Start handle. */
#define CSCS_END_HDL                 (CSCS_MAX_HDL - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief Cycling Speed Service Handles */
enum
{
  CSCS_SVC_HDL = CSCS_START_HDL,       /*!< \brief Cycling Speed Server Service declaration */
  CSCS_CSF_CH_HDL,                     /*!< \brief Cycling Speed Feature characteristic */
  CSCS_CSF_HDL,                        /*!< \brief Cycling Speed Feature */
  CSCS_CSM_CH_HDL,                     /*!< \brief Cycling Speed Measurement characteristic */
  CSCS_CSM_HDL,                        /*!< \brief Cycling Speed Measurement */
  CSCS_CSM_CH_CCC_HDL,                 /*!< \brief Cycling Speed Measurement Client Characteristic Configuration Descriptor */
  CSCS_SL_CH_HDL,                      /*!< \brief Cycling Speed Sensor Location characteristic */
  CSCS_SL_HDL,                         /*!< \brief Cycling Speed Sensor Location */
  CSCS_MAX_HDL                         /*!< \brief Maximum handle. */
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
void SvcCscsAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcCscsRemoveGroup(void);

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
void SvcCscsCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback);

/*! \} */    /* CYCLING_SPEED_AND_CADENCE_SERVICE */

#ifdef __cplusplus
};
#endif

#endif /* SVC_CSCS_H */
