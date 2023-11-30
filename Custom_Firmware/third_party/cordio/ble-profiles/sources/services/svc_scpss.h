/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Scan Parameter Service Server implementation.
 *
 *  Copyright (c) 2016-2018 Arm Ltd. All Rights Reserved.
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

#ifndef SVC_SCPSS_H
#define SVC_SCPSS_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup SCAN_PARAMETER_SERVICE
 *  \{ */

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name Scan Parameter Service Handles
 *
 */
/**@{*/
#define SCPSS_START_HDL               0x0300              /*!< \brief Start handle. */
#define SCPSS_END_HDL                 (SCPSS_MAX_HDL - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief Scan Parameter Service Handles */
enum
{
  SCPSS_SVC_HDL = SCPSS_START_HDL,    /*!< \brief Scan Parameter Server Service declaration */
  SCPSS_SIW_CH_HDL,                   /*!< \brief Scan Interval Window characteristic */
  SCPSS_SIW_HDL,                      /*!< \brief Scan Interval Window */
  SCPSS_MAX_HDL                       /*!< \brief Maximum handle. */
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
void SvcScpssAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcScpssRemoveGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Register callbacks for the service.
 *
 *  \param  writeCback  Write callback function.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcScpssCbackRegister(attsWriteCback_t writeCback);

/*! \} */    /* SCAN_PARAMETER_SERVICE */

#ifdef __cplusplus
};
#endif

#endif /* SVC_SCPSS_H */
