/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  UriBeacon sample application.
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

#ifndef URIBEACON_API_H
#define URIBEACON_API_H

#include "wsf_types.h"
#include "wsf_os.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! UriBeacon parameter IDs */
enum
{
  URI_BEACON_PARAM_LOCK_STATE                 = 4,  /*!< lock state (for UriBeacon) [1 byte] */
  URI_BEACON_PARAM_URI_DATA                   = 5,  /*!< URI data (for UriBeacon) [18 bytes] */
  URI_BEACON_PARAM_URI_FLAGS                  = 6,  /*!< URI flags (for UriBeacon) [1 byte] */
  URI_BEACON_PARAM_ADVERTISED_TX_POWER_LEVELS = 7,  /*!< advertised tx power levels (for UriBeacon) [4 bytes] */
  URI_BEACON_PARAM_TX_POWER_MODE              = 8,  /*!< tx power mode (for UriBeacon) [1 byte] */
  URI_BEACON_PARAM_BEACON_PERIOD              = 9,  /*!< beacon period (for beacon) [2 bytes] */
  URI_BEACON_PARAM_LOCK                       = 10  /*!< lock [16 bytes] */
};

/**************************************************************************************************
  Callback Function Types
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  App extension callback.
 *
 *  \param  pMsg    Pointer to message structure.
 *
 *  \return None.
 */
/*************************************************************************************************/
typedef void (*uribeaconExtCback_t)(wsfMsgHdr_t *pMsg);

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Start UriBeacon application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriBeaconStart(void);

/*************************************************************************************************/
/*!
 *  \brief  WSF event handler for application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriBeaconHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);

/*************************************************************************************************/
/*!
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriBeaconHandlerInit(wsfHandlerId_t handlerId);

/*************************************************************************************************/
/*!
 *  \brief  Called prior to starting Uribeacon app to override the beacon data in nv memory.
 *
 *  \param  pUriData    Pointer to URI data.
 *  \param  dataLen     Length of pUriData in bytes.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriBeaconSetUriOverride(const uint8_t *pUriData, uint8_t dataLen);

/*************************************************************************************************/
/*!
 *  \brief  Register a callback to receive events for the purpose of extending the URI beacon app.
 *
 *  \param  extCback    Callback function
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriBeaconRegisterExtensionCback(uribeaconExtCback_t extCback);

#endif /* URIBEACON_API_H */
