/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Internet Profile Support Service implementation.
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

#ifndef SVC_IPSS_H
#define SVC_IPSS_H

#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup INTERNET_PROFILE_SUPPORT_SERVICE
 *  \{ */

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name IP Support Service Handles
 *
 */
/**@{*/
#define IPSS_START_HDL               0x0600             /*!< \brief Start handle. */
#define IPSS_END_HDL                 (IPSS_MAX_HDL - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief IP Support Service Handles */
enum
{
  IPSS_SVC_HDL = IPSS_START_HDL,       /*!< \brief IP Support Server Service declaration */
  IPSS_MAX_HDL                         /*!< \brief Maximum handle. */
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
void SvcIpssAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcIpssRemoveGroup(void);

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
void SvcIpssCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback);

/*! \} */    /* INTERNET_PROFILE_SUPPORT_SERVICE */

#ifdef __cplusplus
};
#endif

#endif /* SVC_IPSS_H */
