/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Device Information Service implementation.
 *
 *  Copyright (c) 2011-2020 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2019-2020 Packetcraft, Inc.
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

#ifndef SVC_DIS_H
#define SVC_DIS_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup DEVICE_INFORMATION_SERVICE
 *  \{ */

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name Device Information Service Handles
 *
 */
/**@{*/
/*! \brief Device Information Service */
#define DIS_START_HDL               0x0030            /*!< \brief Start handle. */
#define DIS_END_HDL                 (DIS_MAX_HDL - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief Device Information Service Handles */
enum
{
  DIS_SVC_HDL = DIS_START_HDL,      /*!< \brief Information service declaration */
  DIS_MFR_CH_HDL,                   /*!< \brief Manufacturer name string characteristic */
  DIS_MFR_HDL,                      /*!< \brief Manufacturer name string */
  DIS_SID_CH_HDL,                   /*!< \brief System ID characteristic */
  DIS_SID_HDL,                      /*!< \brief System ID */
  DIS_MN_CH_HDL,                    /*!< \brief Model number string characteristic */
  DIS_MN_HDL,                       /*!< \brief Model number string */
  DIS_SN_CH_HDL,                    /*!< \brief Serial number string characteristic */
  DIS_SN_HDL,                       /*!< \brief Serial number string */
  DIS_FWR_CH_HDL,                   /*!< \brief Firmware revision string characteristic */
  DIS_FWR_HDL,                      /*!< \brief Firmware revision string */
  DIS_HWR_CH_HDL,                   /*!< \brief Hardware revision string characteristic */
  DIS_HWR_HDL,                      /*!< \brief Hardware revision string */
  DIS_SWR_CH_HDL,                   /*!< \brief Software revision string characteristic */
  DIS_SWR_HDL,                      /*!< \brief Software revision string */
  DIS_RCD_CH_HDL,                   /*!< \brief IEEE 11073-20601 regulatory certificate data characteristic */
  DIS_RCD_HDL,                      /*!< \brief IEEE 11073-20601 regulatory certificate data */
  DIS_PNP_ID_CH_HDL,                /*!< \brief PnP ID characteristic */
  DIS_PNP_ID_HDL,                   /*!< \brief PnP ID */
  DIS_MAX_HDL                       /*!< \brief Maximum handle. */
};
/**@}*/

/**************************************************************************************************
Macros
**************************************************************************************************/

/** \name Size of Attributes
 *
 */
/**@{*/
#ifndef DIS_MAXSIZE_MFR_ATT
#define DIS_MAXSIZE_MFR_ATT         20  /*!< \brief Size of manufacturer name string attribute */
#endif

#ifndef DIS_MAXSIZE_MN_ATT
#define DIS_MAXSIZE_MN_ATT          25  /*!< \brief Size of model number string attribute */
#endif

#ifndef DIS_MAXSIZE_SN_ATT
#define DIS_MAXSIZE_SN_ATT          25  /*!< \brief Size of serial number string attribute */
#endif

#ifndef DIS_MAXSIZE_FWR_ATT
#define DIS_MAXSIZE_FWR_ATT         21 /*!< \brief Size of firmware revision string attribute */
#endif

#ifndef DIS_MAXSIZE_HWR_ATT
#define DIS_MAXSIZE_HWR_ATT         21  /*!< \brief Size of hardware revision string attribute */
#endif

#ifndef DIS_MAXSIZE_SWR_ATT
#define DIS_MAXSIZE_SWR_ATT         21  /*!< \brief Size of software revision string attribute */
#endif

#ifndef DIS_SIZE_SID_ATT
#define DIS_SIZE_SID_ATT            8   /*!< \brief Size of system id attribute */
#endif

#ifndef DIS_SIZE_RCD_ATT
#define DIS_SIZE_RCD_ATT            6   /*!< \brief Size of registration certificate data attribute */
#endif
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
void SvcDisAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcDisRemoveGroup(void);

/*! \} */    /* DEVICE_INFORMATION_SERVICE */

#ifdef __cplusplus
};
#endif

#endif /* SVC_DIS_H */

