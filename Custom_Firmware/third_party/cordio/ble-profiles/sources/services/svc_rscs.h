/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Running Speed and Cadence Service Server implementation.
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

#ifndef SVC_RSCS_H
#define SVC_RSCS_H

#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup RUNNING_SPEED_AND_CADENCE_SERVICE
 *  \{ */

/**************************************************************************************************
Constants
**************************************************************************************************/

/** \name RSC Feature Bits
 * Running Speed and Cadence Feature Bits of the Feature Characteristic
 */
/**@{*/
#define RSCS_ISLMS_FEATURE_BIT           (1<<0)  /*!< \brief Instantaneous Stride Length Measurement Supported Feature Mask Bit */
#define RSCS_TDMS_FEATURE_BIT            (1<<1)  /*!< \brief Total Distance Measurement Supported Feature Mask Bit */
#define RSCS_WRSS_FEATURE_BIT            (1<<2)  /*!< \brief Walking or Running Status Supported Feature Mask Bit */
#define RSCS_CPS_FEATURE_BIT             (1<<3)  /*!< \brief Calibration Procedure Supported Feature Mask Bit */
#define RSCS_MSLS_FEATURE_BIT            (1<<4)  /*!< \brief Multiple Sensor Locations Supporte Feature Mask Bit */

#define RSCS_ALL_FEATURES                (0x7)   /*!< \brief All Supported Feature Mask */
/**@}*/

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name Running Speed and Cadence Service Handles
 *
 */
/**@{*/
#define RSCS_START_HDL               0x04D0             /*!< \brief Start handle. */
#define RSCS_END_HDL                 (RSCS_MAX_HDL - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief Running Speed Service Handles */
enum
{
  RSCS_SVC_HDL = RSCS_START_HDL,       /*!< \brief Running Speed Server Service declaration */
  RSCS_RSF_CH_HDL,                     /*!< \brief Running Speed Feature characteristic */
  RSCS_RSF_HDL,                        /*!< \brief Running Speed Feature */
  RSCS_RSM_CH_HDL,                     /*!< \brief Running Speed Measurement characteristic */
  RSCS_RSM_HDL,                        /*!< \brief Running Speed Measurement */
  RSCS_RSM_CH_CCC_HDL,                 /*!< \brief Running Speed Measurement Client Characteristic Configuration Descriptor */
  RSCS_SL_CH_HDL,                      /*!< \brief Running Speed Sensor Location characteristic */
  RSCS_SL_HDL,                         /*!< \brief Running Speed Sensor Location */
  RSCS_MAX_HDL                         /*!< \brief Maximum handle. */
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
void SvcRscsAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcRscsRemoveGroup(void);

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
void SvcRscsCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback);

/*! \} */    /* RUNNING_SPEED_AND_CADENCE_SERVICE */

#ifdef __cplusplus
};
#endif

#endif /* SVC_RSCS_H */
