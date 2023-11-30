/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Health/medical sensor sample application interface file.
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

#ifndef MEDS_MAIN_H
#define MEDS_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! WSF message event starting value */
#define MEDS_MSG_START             0xA0

/*! WSF message event enumeration */
enum
{
  MEDS_TIMER_IND = MEDS_MSG_START,    /*! Timer expired */
};

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! profile interface callback functions */
typedef void (*medsInitCback_t)(void);
typedef void (*medsStartCback_t)(void);
typedef void (*medsProcMsgCback_t)(wsfMsgHdr_t *pMsg);
typedef void (*medsBtnCback_t)(dmConnId_t connId, uint8_t btn);

/*! profile interface structure */
typedef struct
{
  medsInitCback_t       init;
  medsStartCback_t      start;
  medsProcMsgCback_t    procMsg;
  medsBtnCback_t        btn;
} medsIf_t;

/*! application control block */
typedef struct
{
  medsIf_t          *pIf;                           /*! Profile interface */
  wsfHandlerId_t    handlerId;                      /*! WSF hander ID */
} medsCb_t;

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! application control block */
extern medsCb_t medsCb;

/*! profile interface pointers */
extern medsIf_t medsBlpIf;          /* blood pressure profile */
extern medsIf_t medsWspIf;          /* weight scale profile */
extern medsIf_t medsHtpIf;          /* health thermometer profile */
extern medsIf_t medsPlxIf;          /* pulse oximeter profile */
extern medsIf_t medsGlpIf;          /* glucose profile */

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

void medsCccCback(attsCccEvt_t *pEvt);

#ifdef __cplusplus
};
#endif

#endif /* MEDS_MAIN_H */

