/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Constant Tone Extension Service implementation.
 *
 *  Copyright (c) 2018 Arm Ltd. All Rights Reserved.
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

#ifndef SVC_CTE_H
#define SVC_CTE_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup CTE-RELATED_SERVICE
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Enable bits */
#define CTE_ENABLE_NONE             0x00                /*!< CTE disable bitmask. */
#define CTE_ENABLE_ACL_BIT          (0x01 << 0)         /*!< ACL CTE Enable bit. */
#define CTE_ENABLE_ADV_BIT          (0x01 << 1)         /*!< Advertising CTE Enable bit. */

/*! Minimum length range */
#define CTE_MAX_MIN_LEN             20                  /*!< Maximum CTE min length. */
#define CTE_MIN_MIN_LEN             2                   /*!< Minimum CTE min length. */

/*! Minimum tx count range */
#define CTE_MAX_MIN_TX_CNT          15                  /*!< Maximum CTE min tx count. */
#define CTE_MIN_MIN_TX_CNT          1                   /*!< Minimum CTE min tx count. */

/*! Minimum interval (1.25 sec) */
#define CTE_MIN_INTERVAL            6                   /*!< Minimum CTE interval. */

/*! PHY types */
#define CTE_PHY_1M                  0                   /*!< 1 Mbs CTE PHY type. */
#define CTE_PHY_2M                  1                   /*!< 2 Mbs CTE PHY type. */

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name CTE Handles
 *
 */
/**@{*/
#define CTE_START_HDL               0x0170              /*!< \brief Start handle. */
#define CTE_END_HDL                 (CTE_MAX_HDL - 1)   /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief CTE-Related Service Handles */
enum
{
  CTE_SVC_HDL = CTE_START_HDL,          /*!< \brief Constant Tone Extension service declaration. */
  CTE_ENABLE_CH_HDL,                    /*!< \brief Constant Tone Extension enable characteristic. */
  CTE_ENABLE_HDL,                       /*!< \brief Constant Tone Extension enable. */
  CTE_MIN_LEN_CH_HDL,                   /*!< \brief Constant Tone Extension minimum length characteristic. */
  CTE_MIN_LEN_HDL,                      /*!< \brief Constant Tone Extension minimum length. */
  CTE_ADV_MIN_TX_CNT_CH_HDL,            /*!< \brief Constant Tone Extension minimum transmit count characteristic. */
  CTE_ADV_MIN_TX_CNT_HDL,               /*!< \brief Constant Tone Extension minimum transmit count. */
  CTE_ADV_TX_DURATION_CH_HDL,           /*!< \brief Constant Tone Extension transmit duration characteristic. */
  CTE_ADV_TX_DURATION_HDL,              /*!< \brief Constant Tone Extension transmit duration. */
  CTE_ADV_INTERVAL_CH_HDL,              /*!< \brief Constant Tone Extension interval characteristic. */
  CTE_ADV_INTERVAL_HDL,                 /*!< \brief Constant Tone Extension interval. */
  CTE_ADV_EXT_PHY_CH_HDL,               /*!< \brief Constant Tone Extension PHY characteristic. */
  CTE_ADV_EXT_PHY_HDL,                  /*!< \brief Constant Tone Extension PHY. */
  CTE_MAX_HDL
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
void SvcCteAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcCteRemoveGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Register callbacks for the service.
 *
 *  \param  writeCback  Write callback function.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcCteCbackRegister(attsWriteCback_t writeCback);

/*! \} */    /* CTE-RELATED_SERVICE */

#ifdef __cplusplus
};
#endif

#endif /* SVC_CTE_H */
