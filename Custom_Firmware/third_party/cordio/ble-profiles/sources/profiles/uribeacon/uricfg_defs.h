/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  UriBeacon configuration service defines.
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

#ifndef URICFG_DEFS_H
#define URICFG_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup URIBEACON_CONFIGURATION_PROFILE
 *  \{ */

/**************************************************************************************************
 Macros
**************************************************************************************************/

/*! \brief UriBeacon configuration service-related UUIDs */
#define URICFG_UUID_BYTES(id) 0xd8, 0x81, 0xc9, 0x1a, 0xb9, 0x99, \
                              0x96, 0xab, \
                              0xba, 0x40, \
                              0x86, 0x87, \
                              (id & 0xFF), ((id >> 8) & 0xFF), 0x0c, 0xee

/*! \brief UriBeacon configuration service-related UUIDs */
enum
{
  URICFG_UUID_SVC              = 0x2080,
  URICFG_UUID_CHR_LOCKSTATE    = 0x2081,
  URICFG_UUID_CHR_LOCK         = 0x2082,
  URICFG_UUID_CHR_UNLOCK       = 0x2083,
  URICFG_UUID_CHR_URIDATA      = 0x2084,
  URICFG_UUID_CHR_URIFLAGS     = 0x2085,
  URICFG_UUID_CHR_TXPWRLEVELS  = 0x2086,
  URICFG_UUID_CHR_TXPWRMODE    = 0x2087,
  URICFG_UUID_CHR_BEACONPERIOD = 0x2088,
  URICFG_UUID_CHR_RESET        = 0x2089
};

/*! \brief Transmit power modes */
enum
{
  URICFG_ATT_TXPWRMODE_LOWEST = 0,
  URICFG_ATT_TXPWRMODE_LOW    = 1,
  URICFG_ATT_TXPWRMODE_MEDIUM = 2,
  URICFG_ATT_TXPWRMODE_HIGH   = 3
};

/** \name URI Config Attributes Sizes
 *
 */
/**@{*/
#define URICFG_MAXSIZE_URIDATA_ATT       18     /*!< \brief Size of URI data attribute */
#define URICFG_SIZE_TXPWRLEVELS_ATT      4      /*!< \brief Size of transmit power levels attribute */
#define URICFG_SIZE_LOCK_ATT             16     /*!< \brief Size of lock attribute */
/**@}*/

/** \name Beacon period Attribute Values
 *
 */
/**@{*/
#define URICFG_ATT_BEACONPERIOD_MIN      20     /*!< \brief Minimum period */
#define URICFG_ATT_BEACONPERIOD_MAX      10240  /*!< \brief Maximum period */
#define URICFG_ATT_BEACONPERIOD_DISABLE  0      /*!< \brief Value to disable beacon */
/**@}*/

/** \name Default (Reset) Values of Attributes
 *
 */
/**@{*/
#define URICFG_ATT_URIFLAGS_DEFAULT      0x10                     /*!< \brief Default URI flags */
#define URICFG_ATT_TXPWRMODE_DEFAULT     URICFG_ATT_TXPWRMODE_LOW /*!< \brief Default TX power mode */
#define URICFG_ATT_BEACONPERIOD_DEFAULT  1000                     /*!< \brief Default beacon period in milliseconds (1 second) */
#define URICFG_ATT_LOCK_DEFAULT_BYTES    0x00, 0x00, 0x00, 0x00,  \
                                         0x00, 0x00, \
                                         0x00, 0x00, \
                                         0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00 /*!< \brief Default lock bytes */
/**@}*/

/*! \brief UriBeacon service UUID for advertising data */
#define URICFG_SERVICE_UUID              0xFEAA

/*! \} */    /* URIBEACON_CONFIGURATION_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* URICFG_DEFS_H */
