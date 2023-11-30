/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  WP service implementation. Arm Ltd. proprietary servicde.
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

#ifndef SVC_WP_H
#define SVC_WP_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup WP_SERVICE
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name Arm Ltd. Proprietary Service Handles
 *
 */
/**@{*/
/*! \brief Proprietary Service */
#define WP_START_HDL               0x0200           /*!< \brief Start handle. */
#define WP_END_HDL                 (WP_MAX_HDL - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief Proprietary Service Handles */
enum
{
  WP_SVC_HDL = WP_START_HDL,       /*!< \brief Proprietary service declaration */
  WP_DAT_CH_HDL,                   /*!< \brief Proprietary data characteristic */
  WP_DAT_HDL,                      /*!< \brief Proprietary data */
  WP_DAT_CH_CCC_HDL,               /*!< \brief Proprietary data client characteristic configuration */
  WP_MAX_HDL                       /*!< \brief Maximum handle. */
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
void SvcWpAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcWpRemoveGroup(void);

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
void SvcWpCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback);

/*! \} */    /* WP_SERVICE */

#ifdef __cplusplus
};
#endif

#endif /* SVC_WP_H */
