/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Health Thermometer service implementation.
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

#ifndef SVC_HTS_H
#define SVC_HTS_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup HEALTH_THERMOMETER_SERVICE
 *  \{ */

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name Health Thermometer Service Handles
 *
 */
/**@{*/
#define HTS_START_HDL               0x0120            /*!< \brief Start handle. */
#define HTS_END_HDL                 (HTS_MAX_HDL - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief Health Thermometer Service Handles */
enum
{
  HTS_SVC_HDL = HTS_START_HDL,      /*!< \brief Health thermometer service declaration */
  HTS_TM_CH_HDL,                    /*!< \brief Temperature measurement characteristic */
  HTS_TM_HDL,                       /*!< \brief Temperature measurement */
  HTS_TM_CH_CCC_HDL,                /*!< \brief Temperature measurement client characteristic configuration */
  HTS_IT_CH_HDL,                    /*!< \brief Intermediate temperature characteristic */
  HTS_IT_HDL,                       /*!< \brief Intermediate temperature */
  HTS_IT_CH_CCC_HDL,                /*!< \brief Intermediate temperature client characteristic configuration */
  HTS_TT_CH_HDL,                    /*!< \brief Temperature type characteristic */
  HTS_TT_HDL,                       /*!< \brief Temperature type */
  HTS_MAX_HDL                       /*!< \brief Maximum handle. */
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
void SvcHtsAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHtsRemoveGroup(void);

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
void SvcHtsCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback);

/*! \} */    /* HEALTH_THERMOMETER_SERVICE */

#ifdef __cplusplus
};
#endif

#endif /* SVC_HTS_H */
