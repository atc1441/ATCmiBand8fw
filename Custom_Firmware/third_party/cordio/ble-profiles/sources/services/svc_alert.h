/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Alert-related service implementation.
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

#ifndef SVC_ALERT_H
#define SVC_ALERT_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup ALERT-RELATED_SERVICE
 *  \{ */

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name Alert Handles
 *
 */
/**@{*/
#define ALERT_START_HDL               0x00C0              /*!< \brief Start handle. */
#define ALERT_END_HDL                 (ALERT_MAX_HDL - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief Alert-Related Service Handles */
enum
{
  ALERT_ANS_SVC_HDL = ALERT_START_HDL,    /*!< \brief Alert notification service declaration */
  ALERT_ANS_SNA_CH_HDL,                   /*!< \brief Supported new alert category characteristic */
  ALERT_ANS_SNA_HDL,                      /*!< \brief Supported new alert category */
  ALERT_ANS_NEW_CH_HDL,                   /*!< \brief New alert characteristic */
  ALERT_ANS_NEW_HDL,                      /*!< \brief New alert */
  ALERT_ANS_NEW_CH_CCC_HDL,               /*!< \brief New alert client characteristic configuration */
  ALERT_ANS_UNR_CH_HDL,                   /*!< \brief Supported unread alert category characteristic */
  ALERT_ANS_UNR_HDL,                      /*!< \brief Supported unread alert category */
  ALERT_ANS_UAS_CH_HDL,                   /*!< \brief Unread alert status characteristic */
  ALERT_ANS_UAS_HDL,                      /*!< \brief Unread alert status */
  ALERT_ANS_UAS_CH_CCC_HDL,               /*!< \brief Unread alert status client characteristic configuration */
  ALERT_ANS_CP_CH_HDL,                    /*!< \brief Alert notification control point characteristic */
  ALERT_ANS_CP_HDL,                       /*!< \brief Alert notification control point */

  ALERT_PASS_SVC_HDL,                     /*!< \brief Phone alert status service declaration */
  ALERT_PASS_AS_CH_HDL,                   /*!< \brief Alert status characteristic */
  ALERT_PASS_AS_HDL,                      /*!< \brief Alert status */
  ALERT_PASS_AS_CCC_HDL,                  /*!< \brief Alert status client characteristic configuration */
  ALERT_PASS_RS_CH_HDL,                   /*!< \brief Ringer setting characteristic */
  ALERT_PASS_RS_HDL,                      /*!< \brief Ringer setting */
  ALERT_PASS_RS_CCC_HDL,                  /*!< \brief Ringer settting client characteristic configuration */
  ALERT_PASS_RCP_CH_HDL,                  /*!< \brief Ringer control point characteristic */
  ALERT_PASS_RCP_HDL,                     /*!< \brief Ringer control point */

  ALERT_NWS_SVC_HDL,                      /*!< \brief Network availability service declaration */
  ALERT_NWS_NWA_CH_HDL,                   /*!< \brief Network availability characteristic */
  ALERT_NWS_NWA_HDL,                      /*!< \brief Network availability */
  ALERT_NWS_NWA_CH_CCC_HDL,               /*!< \brief Network availability client characteristic configuration */
  ALERT_MAX_HDL                           /*!< \brief Maximum handle. */
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
void SvcAlertAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcAlertRemoveGroup(void);

/*! \} */    /* ALERT-RELATED_SERVICE */

#ifdef __cplusplus
};
#endif

#endif /* SVC_ALERT_H */

