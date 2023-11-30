/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Weight Scale service implementation.
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

#ifndef SVC_WSS_H
#define SVC_WSS_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup WEIGHT_SCALE_SERVICE
 *  \{ */

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name Weight Scale Service Handles
 *
 */
/**@{*/
#define WSS_START_HDL               0x0110            /*!< \brief Start handle. */
#define WSS_END_HDL                 (WSS_MAX_HDL - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief Weight Scale Service Handles */
enum
{
  WSS_SVC_HDL = WSS_START_HDL,      /*!< \brief Weight scale service declaration */
  WSS_WM_CH_HDL,                    /*!< \brief Weight measurement characteristic */
  WSS_WM_HDL,                       /*!< \brief Weight measurement */
  WSS_WM_CH_CCC_HDL,                /*!< \brief Weight measurement client characteristic configuration */
  WSS_WSF_CH_HDL,                   /*!< \brief Weight scale feature characteristic */
  WSS_WSF_HDL,                      /*!< \brief Weight scale feature */
  WSS_MAX_HDL                       /*!< \brief Maximum handle. */
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
void SvcWssAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcWssRemoveGroup(void);

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
void SvcWssCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback);

/*! \} */    /* WEIGHT_SCALE_SERVICE */

#ifdef __cplusplus
};
#endif

#endif /* SVC_WSS_H */
