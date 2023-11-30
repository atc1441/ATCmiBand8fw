/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Glucose service implementation.
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

#ifndef SVC_GLS_H
#define SVC_GLS_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup GLUCOSE_SERVICE
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/

/** \name Error Codes
 *
 */
/**@{*/
#define GLS_ERR_IN_PROGRESS       0x80    /*!< \brief Procedure already in progress */
#define GLS_ERR_CCCD              0x81    /*!< \brief CCCD improperly configured */
/**@}*/

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name Glucose Service Handles
 *
 */
/**@{*/
#define GLS_START_HDL               0x0100            /*!< \brief Start handle. */
#define GLS_END_HDL                 (GLS_MAX_HDL - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief Glucose Service Handles */
enum
{
  GLS_SVC_HDL = GLS_START_HDL,      /*!< \brief Glucose service declaration */
  GLS_GLM_CH_HDL,                   /*!< \brief Glucose measurement characteristic */
  GLS_GLM_HDL,                      /*!< \brief Glucose measurement */
  GLS_GLM_CH_CCC_HDL,               /*!< \brief Glucose measurement client characteristic configuration */
  GLS_GLMC_CH_HDL,                  /*!< \brief Glucose measurement context characteristic */
  GLS_GLMC_HDL,                     /*!< \brief Glucose measurement context */
  GLS_GLMC_CH_CCC_HDL,              /*!< \brief Glucose measurement context client characteristic configuration */
  GLS_GLF_CH_HDL,                   /*!< \brief Glucose feature characteristic */
  GLS_GLF_HDL,                      /*!< \brief Glucose feature */
  GLS_RACP_CH_HDL,                  /*!< \brief Record access control point characteristic */
  GLS_RACP_HDL,                     /*!< \brief Record access control point */
  GLS_RACP_CH_CCC_HDL,              /*!< \brief Record access control point client characteristic configuration */
  GLS_MAX_HDL                       /*!< \brief Maximum handle. */
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
void SvcGlsAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcGlsRemoveGroup(void);

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
void SvcGlsCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback);

/*! \} */    /* GLUCOSE_SERVICE */

#ifdef __cplusplus
};
#endif

#endif /* SVC_GLS_H */
