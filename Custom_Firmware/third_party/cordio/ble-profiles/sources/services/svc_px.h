/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Proximity services implementation.
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

#ifndef SVC_PX_H
#define SVC_PX_H

#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup PROXIMITY_SERVICE
 *  \{ */

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name Proximity Service Handles
 *
 */
/**@{*/
#define PX_START_HDL                      0x0180           /*!< \brief Start handle. */
#define PX_END_HDL                        (PX_MAX_HDL - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief Service Handles */
enum
{
  LLS_SVC_HDL = PX_START_HDL,       /*!< \brief Link loss service declaration */
  LLS_AL_CH_HDL,                    /*!< \brief Alert level characteristic */
  LLS_AL_HDL,                       /*!< \brief Alert level */
  IAS_SVC_HDL,                      /*!< \brief Immediate alert service declaration */
  IAS_AL_CH_HDL,                    /*!< \brief Alert level characteristic */
  IAS_AL_HDL,                       /*!< \brief Alert level */
  TXS_SVC_HDL,                      /*!< \brief TX power service declaration */
  TXS_TX_CH_HDL,                    /*!< \brief TX power level characteristic */
  TXS_TX_HDL,                       /*!< \brief TX power level */
  PX_MAX_HDL                        /*!< \brief Maximum handle. */
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
void SvcPxAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcPxRemoveGroup(void);

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
void SvcPxCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback);

/*! \} */    /* PROXIMITY_SERVICE */

#ifdef __cplusplus
};
#endif

#endif /* SVC_PX_H */

