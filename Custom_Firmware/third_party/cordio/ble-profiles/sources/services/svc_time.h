/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Time-related service implementation.
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

#ifndef SVC_TIME_H
#define SVC_TIME_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup TIME-RELATED_SERVICE
 *  \{ */


/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name Time Service Handles
 *
 */
/**@{*/
#define TIME_START_HDL               0x00E0             /*!< \brief Start handle. */
#define TIME_END_HDL                 (TIME_MAX_HDL - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief Time-Related Service Handles */
enum
{
  TIME_CTS_SVC_HDL = TIME_START_HDL,    /*!< \brief Current time service declaration */
  TIME_CTS_CT_CH_HDL,                   /*!< \brief CT time characteristic */
  TIME_CTS_CT_HDL,                      /*!< \brief CT time */
  TIME_CTS_CT_CH_CCC_HDL,               /*!< \brief CT time client characteristic configuration */
  TIME_CTS_LOC_CH_HDL,                  /*!< \brief Local time information characteristic */
  TIME_CTS_LOC_HDL,                     /*!< \brief Local time information */
  TIME_CTS_REF_CH_HDL,                  /*!< \brief Reference time information characteristic */
  TIME_CTS_REF_HDL,                     /*!< \brief Reference time information */

  TIME_DST_SVC_HDL,                     /*!< \brief DST change service declaration */
  TIME_DST_WDST_CH_HDL,                 /*!< \brief Time with DST characteristic */
  TIME_DST_WDST_HDL,                    /*!< \brief Time with DST */

  TIME_RTU_SVC_HDL,                     /*!< \brief Reference time update service declaration */
  TIME_RTU_CP_CH_HDL,                   /*!< \brief Time update control point characteristic */
  TIME_RTU_CP_HDL,                      /*!< \brief Time update control point */
  TIME_RTU_STATE_CH_HDL,                /*!< \brief Time update state characteristic */
  TIME_RTU_STATE_HDL,                   /*!< \brief Time update state */

  TIME_MAX_HDL                          /*!< \brief Maximum handle. */
};
/**@}*/

/*! \brief Indexes of CCC descriptor handle table entries */
enum
{
  TIME_CTS_CT_CH_CCC_IDX /*!< \brief CT time CCCD index. */
};

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
void SvcTimeAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcTimeRemoveGroup(void);

/*! \} */    /* TIME-RELATED_SERVICE */

#ifdef __cplusplus
};
#endif

#endif /* SVC_TIME_H */

