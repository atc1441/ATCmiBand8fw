/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Health/medical collector sample application.
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
#ifndef MEDC_API_H
#define MEDC_API_H

#include "wsf_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Profile identifier used for MedcSetProfile() */
enum
{
  MEDC_ID_HRP,      /*! Heart rate profile */
  MEDC_ID_BLP,      /*! Blood pressure profile */
  MEDC_ID_GLP,      /*! Glucose profile */
  MEDC_ID_WSP,      /*! Weight scale profile */
  MEDC_ID_HTP,      /*! Health thermometer profile */
  MEDC_ID_PLX       /*! Pulse oximeter profile */
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
void MedcStart(void);

/*************************************************************************************************/
/*!
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID for App.
 *
 *  \return None.
 */
/*************************************************************************************************/
void MedcHandlerInit(wsfHandlerId_t handlerId);


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
void MedcHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);

/*************************************************************************************************/
/*!
 *  \brief  Set the profile to be used by the application.  This function is called internally
 *          by MedcHandlerInit() with a default value.  It may also be called by the system
 *          to configure the profile after executing MedcHandlerInit() and before executing
 *          MedcStart().
 *
 *  \param  profile  Profile identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
void MedcSetProfile(uint8_t profile);

#ifdef __cplusplus
};
#endif

#endif /* MEDC_API_H */
