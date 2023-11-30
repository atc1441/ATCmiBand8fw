/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Heart Rate service implementation.
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

#ifndef SVC_HRS_H
#define SVC_HRS_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup HEART_RATE_SERVICE
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/

/** \name Heart Rate Error Codes
 *
 */
/**@{*/
#define HRS_ERR_CP_NOT_SUP          0x80    /*!< \brief Control Point value not supported */
/**@}*/

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name Heart Rate Service Handles
 *
 */
/**@{*/
#define HRS_START_HDL               0x0020            /*!< \brief Start handle. */
#define HRS_END_HDL                 (HRS_MAX_HDL - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief Heart Rate Service Handles */
enum
{
  HRS_SVC_HDL = HRS_START_HDL,      /*!< \brief Heart rate service declaration */
  HRS_HRM_CH_HDL,                   /*!< \brief Heart rate measurement characteristic */
  HRS_HRM_HDL,                      /*!< \brief Heart rate measurement */
  HRS_HRM_CH_CCC_HDL,               /*!< \brief Heart rate measurement client characteristic configuration */
  HRS_SL_CH_HDL,                    /*!< \brief Body sensor location characteristic */
  HRS_SL_HDL,                       /*!< \brief Body sensor location */
  HRS_CP_CH_HDL,                    /*!< \brief Heart rate control point characteristic */
  HRS_CP_HDL,                       /*!< \brief Heart rate control point */
  HRS_MAX_HDL                       /*!< \brief Maximum handle. */
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
void SvcHrsAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHrsRemoveGroup(void);

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
void SvcHrsCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback);

/*! \} */    /* HEART_RATE_SERVICE */

#ifdef __cplusplus
};
#endif

#endif /* SVC_HRS_H */
