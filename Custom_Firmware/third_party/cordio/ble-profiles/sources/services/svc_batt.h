/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Battery service implementation.
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

#ifndef SVC_BATT_H
#define SVC_BATT_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup BATTERY_SERVICE
 *  \{ */

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name Battery Service Handles
 *
 */
/**@{*/
#define BATT_START_HDL                    0x0088              /*!< \brief Service start handle. */
#define BATT_END_HDL                      (BATT_MAX_HDL - 1)  /*!< \brief Service end handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief Battery Service Handles */
enum
{
  BATT_SVC_HDL = BATT_START_HDL,        /*!< \brief Battery service declaration */
  BATT_LVL_CH_HDL,                      /*!< \brief Battery level characteristic */
  BATT_LVL_HDL,                         /*!< \brief Battery level */
  BATT_LVL_CH_CCC_HDL,                  /*!< \brief Battery level CCCD */
  BATT_MAX_HDL                          /*!< \brief Maximum handle. */
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
void SvcBattAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcBattRemoveGroup(void);

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
void SvcBattCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback);

/*************************************************************************************************/
/*!
 *  \brief  Add the battery service using the dynamic attribute subsystem.
 *
 *  \return None.
 */
/*************************************************************************************************/
void *SvcBattAddGroupDyn(void);

/*! \} */    /* BATTERY_SERVICE */

#ifdef __cplusplus
};
#endif

#endif /* SVC_BATT_H */

