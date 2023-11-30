/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example temperature sensor service implementation.
*
 *  Copyright (c) 2015-2018 Arm Ltd. All Rights Reserved.
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

#ifndef SVC_TEMP_H
#define SVC_TEMP_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup TEMPERATURE_SENSOR_SERVICE
 *  \{ */

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name Temperature Service Handles
 *
 */
/**@{*/
#define TEMP_HANDLE_START  0x0090                        /*!< \brief Start handle. */
#define TEMP_HANDLE_END   (TEMP_HANDLE_END_PLUS_ONE - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief Temperature service handles. */
enum
{
  TEMP_HANDLE_SVC = TEMP_HANDLE_START, /*!< \brief Service declaration. */

  TEMP_HANDLE_DATA_CHR,                /*!< \brief Data characteristic declaration. */
  TEMP_HANDLE_DATA,                    /*!< \brief Data characteristic value. */
  TEMP_HANDLE_DATA_CLIENT_CHR_CONFIG,  /*!< \brief Data characteristic CCCD. */
  TEMP_HANDLE_DATA_CHR_USR_DESCR,      /*!< \brief Data characteristic user description. */

  TEMP_HANDLE_CONFIG_CHR,              /*!< \brief Configruation characteristic declaration. */
  TEMP_HANDLE_CONFIG,                  /*!< \brief Configruation characteristic value. */
  TEMP_HANDLE_CONFIG_CHR_USR_DESCR,    /*!< \brief Configruation characteristic user description. */

  TEMP_HANDLE_PERIOD_CHR,              /*!< \brief Period characteristc declaration. */
  TEMP_HANDLE_PERIOD,                  /*!< \brief Period characteristc value. */
  TEMP_HANDLE_PERIOD_CHR_USR_DESCR,    /*!< \brief Period characteristc user description. */

  TEMP_HANDLE_END_PLUS_ONE             /*!< \brief Maximum handle. */
};
/**@}*/

/**************************************************************************************************
 Macros
**************************************************************************************************/

/** \name Configuration Attributes
 *
 */
/**@{*/
#define TEMP_ATT_CONFIG_DISABLE  0x00u  /*!< \brief Disable */
#define TEMP_ATT_CONFIG_ENABLE   0x01u  /*!< \brief Enable */
/**@}*/

/** \name Values for Period Attributes
 *
 */
/**@{*/
#define TEMP_ATT_PERIOD_MAX      250u  /*!< \brief Maximum period */
#define TEMP_ATT_PERIOD_DEFAULT  100u  /*!< \brief Default period */
#define TEMP_ATT_PERIOD_MIN       10u  /*!< \brief Minimum period */
/**@}*/

/** \name Sizes of Attributes
 *
 */
/**@{*/
#define TEMP_SIZE_CONFIG_ATT  1u  /*!< \brief Config Attribute size */
#define TEMP_SIZE_PERIOD_ATT  1u  /*!< \brief Period Attribute size */
#define TEMP_SIZE_DATA_ATT    6u  /*!< \brief Data Attribute size */
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
void SvcTempAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcTempRemoveGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Register callbacks for the service.
 *
 *  \param  writeCback  Write callback function.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcTempCbackRegister(attsWriteCback_t writeCback);

/*! \} */    /* TEMPERATURE_SENSOR_SERVICE */

#ifdef __cplusplus
}
#endif

#endif /* SVC_TEMP_H */
