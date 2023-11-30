/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  UriBeacon configuration profile.
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

#ifndef URICFG_API_H
#define URICFG_API_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup URIBEACON_CONFIGURATION_PROFILE
 *  \{ */

#include "wsf_types.h"
#include "att_api.h"
#include "svc_uricfg.h"
#include "uricfg_defs.h"

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Attribute write callback.
 *
 *  \param  handle      Attribute handle.
 *  \param  valueLen    Length of value data.
 *  \param  pValue      Pointer to value data.
 *
 *  \return None.
 */
/*************************************************************************************************/
typedef void (*uriCfgAttWriteCback_t)(uint16_t handle, uint16_t valueLen, const uint8_t *pValue);

/*************************************************************************************************/
/*!
 *  \brief  Lock change callback.
 *
 *  \param  lockState   New lock state.
 *  \param  lock        Lock value.
 *
 *  \return None.
 */
/*************************************************************************************************/
typedef void (*uriCfgLockChangeCback_t)(uint8_t lockState, const uint8_t *pLock);

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Start UriBeacon configuration service.
 *
 *  \param  pUriData          Initial URI data value
 *  \param  uriDataLen        Length of URI data value
 *  \param  uriFlags          Initial URI flags value
 *  \param  pAdvTxPwrLevels   Initial advertised tx power levels value
 *  \param  txPwrMode         Initial tx power mode value
 *  \param  beaconPeriod      Initial beacon period value
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriCfgStart(const uint8_t *pUriData, uint8_t uriDataLen, uint8_t uriFlags,
    int8_t *pAdvTxPwrLevels, uint8_t txPwrMode, uint16_t beaconPeriod);

/*************************************************************************************************/
/*!
 *  \brief  Stop UriBeacon configuration service.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriCfgStop(void);

/*************************************************************************************************/
/*!
 *  \brief  Register callback for written UriBeacon attributes.
 *
 *  \param  cback       Callback to invoke when an attribute changes.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriCfgAttWriteCbackRegister(uriCfgAttWriteCback_t cback);

/*************************************************************************************************/
/*!
 *  \brief  Make UriBeacon lockable.
 *
 *  \param  lockState   Initial lock state value.
 *  \param  pLock       Initial lock value.
 *  \param  cback       Callback to invoke when lock changes.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriCfgMakeLockable(uint8_t lockState, uint8_t *pLock, uriCfgLockChangeCback_t cback);

/*************************************************************************************************/
/*!
 *  \brief  Set reset value of URI data.
 *
 *  \param  pUriData    Reset value of URI data.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriCfgSetUriDataResetValue(const uint8_t *pUriData);

/*! \} */    /* URIBEACON_CONFIGURATION_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* URICFG_API_H */
