/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Health/medical collector sample application interface file.
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

#ifndef MEDC_MAIN_H
#define MEDC_MAIN_H

#include "gatt/gatt_api.h"
#include "dis/dis_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! the Client handle list, medcCb.hdlList[], is set as follows:
 *
 *  ------------------------------- <- MEDC_DISC_GATT_START
 *  | GATT handles                |
 *  |...                          |
 *  ------------------------------- <- MEDC_DISC_DIS_START
 *  | DIS handles                 |
 *  | ...                         |
 *  ------------------------------- <- Configured med service start
 *  | Med service handles         |
 *  | ...                         |
 *  -------------------------------
 */

/*! Maximum number of Service UUID for autoconnect */
#define MEDC_MAX_AUTO_UUID          2

/*! Start of each service's handles in the the handle list */
#define MEDC_DISC_GATT_START        0
#define MEDC_DISC_DIS_START         (MEDC_DISC_GATT_START + GATT_HDL_LIST_LEN)

/*! WSF message event starting value */
#define MEDC_MSG_START             0xA0

/*! WSF message event enumeration */
enum
{
  MEDC_TIMER_IND = MEDC_MSG_START,    /*! Timer expired */
};

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! profile interface callback functions */
typedef void (*medcInitCback_t)(void);
typedef bool_t (*medcDiscoverCback_t)(dmConnId_t connId);
typedef void (*medcConfigureCback_t)(dmConnId_t connId, uint8_t status);
typedef void (*medcProcMsgCback_t)(wsfMsgHdr_t *pMsg);
typedef void (*medcBtnCback_t)(dmConnId_t connId, uint8_t btn);

/*! profile interface structure */
typedef struct
{
  medcInitCback_t       init;
  medcDiscoverCback_t   discover;
  medcConfigureCback_t  configure;
  medcProcMsgCback_t    procMsg;
  medcBtnCback_t        btn;
} medcIf_t;

/*! application control block */
typedef struct
{
  uint16_t          hdlList[APP_DB_HDL_LIST_LEN];   /*! Cached handle list */
  medcIf_t          *pIf;                           /*! Profile interface */
  wsfHandlerId_t    handlerId;                      /*! WSF hander ID */
  uint16_t          autoUuid[MEDC_MAX_AUTO_UUID];   /*! Service UUID for autoconnect */
  bool_t            scanning;                       /*! TRUE if scanning */
  bool_t            autoConnect;                    /*! TRUE if auto-connecting */
  uint8_t           hdlListLen;                     /*! Cached handle list length */
  uint8_t           discState;                      /*! Service discovery state */
  uint8_t           cfgState;                       /*! Service configuration state */
} medcCb_t;

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! Default value for CCC indications and notifications */
extern const uint8_t medcCccIndVal[2];

/*! Default value for CCC notifications */
extern const uint8_t medcCccNtfVal[2];

/*! Pointers into handle list for GATT and DIS service handles */
extern uint16_t *pMedcGattHdlList;
extern uint16_t *pMedcDisHdlList;

/*! application control block */
extern medcCb_t medcCb;

/*! profile interface pointers */
extern medcIf_t medcHrpIf;          /* heart rate profile */
extern medcIf_t medcBlpIf;          /* blood pressure profile */
extern medcIf_t medcGlpIf;          /* glucose profile */
extern medcIf_t medcWspIf;          /* weight scale profile */
extern medcIf_t medcHtpIf;          /* health thermometer profile */
extern medcIf_t medcPlxpIf;         /* pulse oximeter profile */

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

#ifdef __cplusplus
};
#endif

#endif /* MEDC_MAIN_H */

