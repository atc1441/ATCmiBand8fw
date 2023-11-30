/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  UriCfg configuration service implementation.
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

#ifndef SVC_URICFG_H
#define SVC_URICFG_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup URICFG_CONFIGURATION_SERVICE
 *  \{ */

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name URI Config Service Handles
 *
 */
/**@{*/
#define URICFG_HANDLE_START  0x00A0                          /*!< \brief Start handle. */
#define URICFG_HANDLE_END   (URICFG_HANDLE_END_PLUS_ONE - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief UriBeacon configuration service handles */
enum
{
  URICFG_HANDLE_SVC = URICFG_HANDLE_START,  /*!< \brief Service declaration. */

  URICFG_HANDLE_LOCKSTATE_CHR,              /*!< \brief Lock state characteristic declaration. */
  URICFG_HANDLE_LOCKSTATE,                  /*!< \brief Lock state characteristic value. */
  URICFG_HANDLE_LOCKSTATE_CHR_USR_DESCR,    /*!< \brief Lock state characteristic user description. */

  URICFG_HANDLE_LOCK_CHR,                   /*!< \brief Lock characteristic declaration. */
  URICFG_HANDLE_LOCK,                       /*!< \brief Lock characteristic value. */
  URICFG_HANDLE_LOCK_CHR_USR_DESCR,         /*!< \brief Lock characteristic user declaration. */

  URICFG_HANDLE_UNLOCK_CHR,                 /*!< \brief Unlock characteristic declaration. */
  URICFG_HANDLE_UNLOCK,                     /*!< \brief Unlock characteristic value. */
  URICFG_HANDLE_UNLOCK_CHR_USR_DESCR,       /*!< \brief Unlock characteristic user description. */

  URICFG_HANDLE_URIDATA_CHR,                /*!< \brief URI data characteristic declaration. */
  URICFG_HANDLE_URIDATA,                    /*!< \brief URI data characteristic value. */
  URICFG_HANDLE_URIDATA_CHR_USR_DESCR,      /*!< \brief URI data characteristic user description. */

  URICFG_HANDLE_URIFLAGS_CHR,               /*!< \brief URI flags characteristic declaration. */
  URICFG_HANDLE_URIFLAGS,                   /*!< \brief URI flags characteristic value. */
  URICFG_HANDLE_URIFLAGS_CHR_USR_DESCR,     /*!< \brief URI flags characteristic user description. */

  URICFG_HANDLE_TXPWRLEVELS_CHR,            /*!< \brief Tx power levels characteristic declaration. */
  URICFG_HANDLE_TXPWRLEVELS,                /*!< \brief Tx power levels characteristic value. */
  URICFG_HANDLE_TXPWRLEVELS_CHR_USR_DESCR,  /*!< \brief Tx power levels characteristic user description. */

  URICFG_HANDLE_TXPWRMODE_CHR,              /*!< \brief Tx power mode characteristic declaration. */
  URICFG_HANDLE_TXPWRMODE,                  /*!< \brief Tx power mode characteristic value. */
  URICFG_HANDLE_TXPWRMODE_CHR_USR_DESCR,    /*!< \brief Tx power mode characteristic user description. */

  URICFG_HANDLE_BEACONPERIOD_CHR,           /*!< \brief Beacon period characteristic declaration. */
  URICFG_HANDLE_BEACONPERIOD,               /*!< \brief Beacon period characteristic value. */
  URICFG_HANDLE_BEACONPERIOD_CHR_USR_DESCR, /*!< \brief Beacon period characteristic user description. */

  URICFG_HANDLE_RESET_CHR,                  /*!< \brief Reset characteristic declaration. */
  URICFG_HANDLE_RESET,                      /*!< \brief Reset characteristic value. */
  URICFG_HANDLE_RESET_CHR_USR_DESCR,        /*!< \brief Reset characteristic user description. */

  URICFG_HANDLE_END_PLUS_ONE                /*!< \brief Maximum handle. */
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
void SvcUriCfgAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcUriCfgRemoveGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Register callbacks for the service.
 *
 *  \param  writeCback  Write callback function.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcUriCfgCbackRegister(attsWriteCback_t writeCback);

/*! \} */    /* URICFG_CONFIGURATION_SERVICE */

#ifdef __cplusplus
}
#endif

#endif /* SVC_URICFG_H */
