/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Medical sensor sample application interface.
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
#ifndef MEDS_API_H
#define MEDS_API_H

#include "wsf_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Profile identifier used for MedsSetProfile() */
enum
{
  MEDS_ID_BLP,      /*! Blood pressure profile */
  MEDS_ID_WSP,      /*! Weight scale profile */
  MEDS_ID_HTP,      /*! Health thermometer profile */
  MEDS_ID_PLX,      /*! Pulse Oximeter profile */
  MEDS_ID_GLP       /*! Glucose profile */
};

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/
/*************************************************************************************************/
/*!
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void MedsStart(void);

/*************************************************************************************************/
/*!
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID for App.
 *
 *  \return None.
 */
/*************************************************************************************************/
void MedsHandlerInit(wsfHandlerId_t handlerId);


/*************************************************************************************************/
/*!
 *  \brief  WSF event handler for the application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void MedsHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);

/*************************************************************************************************/
/*!
 *  \brief  Set the profile to be used by the application.  This function is called internally
 *          by MedsHandlerInit() with a default value.  It may also be called by the system
 *          to configure the profile after executing MedsHandlerInit() and before executing
 *          MedsStart().
 *
 *  \param  profile  Profile identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
void MedsSetProfile(uint8_t profile);

#ifdef __cplusplus
};
#endif

#endif /* MEDS_API_H */
