/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Wireless Data Exchange profile application interface.
 *
 *  Copyright (c) 2013-2018 Arm Ltd. All Rights Reserved.
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

#ifndef WDXS_API_H
#define WDXS_API_H

#include "att_api.h"
#include "wdx_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup WIRELESS_DATA_EXCHANGE_PROFILE
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief Size of RAM Media used by the application */
#define WDXS_APP_RAM_MEDIA_SIZE         256

/*! \brief Device Model Name */
#ifndef WDXS_DEVICE_MODEL
#define WDXS_DEVICE_MODEL               "WDXS App"
#endif

/*************************************************************************************************/
/*!
 *  \brief  Called at startup to configure WDXS authentication.
 *
 *  \param  reqLevel  Level of authentication that is required for a client to use WDXS
 *  \param  pKey      Authentication key (set to NULL if no authentication is required)
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsAuthenticationCfg(bool_t reqLevel, uint8_t *pKey);

/*************************************************************************************************/
/*!
 *  \brief  Handle WSF events for WDXS.
 *
 *  \param  event  event
 *  \param  pMsg   message assiciated with event
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);

/*************************************************************************************************/
/*!
 *  \brief  WSF Task Initialization for WDXS task.
 *
 *  \param  handlerId   ID of the WDXS task
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsHandlerInit(wsfHandlerId_t handlerId);

/*************************************************************************************************/
/*!
 *  \brief  Called by application to notify the WDXS of ATT Events.
 *
 *  \param  pEvt   Pointer to the ATT Event
 *
 *  \return TRUE if the application should ignore the event, else FALSE.
 */
/*************************************************************************************************/
uint8_t WdxsAttCback(attEvt_t *pEvt);

/*************************************************************************************************/
/*!
 *  \brief  Called by application to notify the WDXS of DM Events.
 *
 *  \param  pEvt   Pointer to the DM Event
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsProcDmMsg(dmEvt_t *pEvt);

/*************************************************************************************************/
/*!
 *  \brief  Set the CCCD index used by the application for WDXS service characteristics.
 *
 *  \param  dcCccIdx   Device Control CCCD index.
 *  \param  auCccIdx   Authentication CCCD index.
 *  \param  ftcCccIdx  File Transfer Control CCCD index.
 *  \param  ftdCccIdx  File Transfer Data CCCD index.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsSetCccIdx(uint8_t dcCccIdx, uint8_t auCccIdx, uint8_t ftcCccIdx, uint8_t ftdCccIdx);

/*************************************************************************************************/
/*!
 *  \brief  Registers the platform dependent Flash Media with the Embedded File System (EFS)
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsFlashMediaInit(void);

/*************************************************************************************************/
/*!
 *  \brief  Registers the platform dependent OTA Media with the Embedded File System (EFS)
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsOtaMediaInit(void);

/*************************************************************************************************/
/*!
 *  \brief  Resets the system.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsResetSystem(void);

/*************************************************************************************************/
/*!
 *  \brief  Initialize WDXS Device Configuration PHY.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsPhyInit(void);

/*! \} */    /* WIRELESS_DATA_EXCHANGE_PROFILE */

#ifdef __cplusplus
}
#endif

#endif /* WDXS_API_H */
