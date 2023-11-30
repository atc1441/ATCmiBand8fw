/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Cycling Power Service Server implementation.
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

#ifndef SVC_CPS_H
#define SVC_CPS_H

#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup CYCLING_POWER_SERVICE
 *  \{ */

/**************************************************************************************************
Constants
**************************************************************************************************/

/** \name Cycling Power Feature Bits
 * Cycle Power Feature Bits of the Feature Characteristic.
 */
/**@{*/
#define CPP_PPBS_FEATURE_BIT            (1<<0)  /*!< \brief Pedal Power Balance Supported Feature Mask Bit */
#define CPP_ATS_FEATURE_BIT             (1<<1)  /*!< \brief Accumulated Torque Supported Feature Mask Bit */
#define CPP_WRDS_FEATURE_BIT            (1<<2)  /*!< \brief Wheel Revolution Data Supported Feature Mask Bit */
#define CPP_CRDS_FEATURE_BIT            (1<<3)  /*!< \brief Crank Revolution Data Supported Feature Mask Bit */
#define CPP_EMS_FEATURE_BIT             (1<<4)  /*!< \brief Extreme Magnitudes Supported Feature Mask Bit */
#define CPP_EAS_FEATURE_BIT             (1<<5)  /*!< \brief Extreme Angles Supported Feature Mask Bit */
#define CPP_TABDSAS_FEATURE_BIT         (1<<6)  /*!< \brief Top and Bottom Dead Spot Angles Supported Feature Mask Bit */
#define CPP_AES_FEATURE_BIT             (1<<7)  /*!< \brief Accumulated Energy Supported Feature Mask Bit */
#define CPP_OCIS_FEATURE_BIT            (1<<8)  /*!< \brief Offset Compensation Indicator Supported Feature Mask Bit */
#define CPP_OCS_FEATURE_BIT             (1<<9)  /*!< \brief Offset Compensation Supported Feature Mask Bit */
#define CPP_CPMCCM_FEATURE_BIT          (1<<10) /*!< \brief Cycling Power Measurement Characteristic Content Masking Supported  Offset Compensation Supported  Feature Mask Bit */
#define CPP_MSLS_FEATURE_BIT            (1<<11) /*!< \brief Multiple Sensor Locations Supported Feature Mask Bit */
#define CPP_CLAS_FEATURE_BIT            (1<<12) /*!< \brief Crank Length Adjustment Supported Feature Mask Bit */
#define CPP_CHLAS_FEATURE_BIT           (1<<13) /*!< \brief Chain Length Adjustment Supported Feature Mask Bit */
#define CPP_CHWAS_FEATURE_BIT           (1<<14) /*!< \brief Chain Weight Adjustment Supported Feature Mask Bit */
#define CPP_SLAS_FEATURE_BIT            (1<<15) /*!< \brief Span Length Adjustment Supported Feature Mask Bit */
#define CPP_SMC_FEATURE_BIT             (1<<16) /*!< \brief Sensor Measurement Contex (0: FORCE, 1: TORQUE) */
#define CPP_IMDS_FEATURE_BIT            (1<<17) /*!< \brief Instantaneous Measurement Direction Supported */
#define CPP_FCDS_FEATURE_BIT            (1<<18) /*!< \brief Factory Calibration Date Supported Feature Mask Bit */
#define CPP_EOCS_FEATURE_BIT            (1<<19) /*!< \brief Enhanced Offset Compensation Supported Feature Mask Bit */
#define CPP_DSS_FEATURE_BIT             (1<<20) /*!< \brief Distribute System Support Feature Mask Bit */

#define CPP_ALL_FEATURES                (0xff)  /*!< \brief All Supported Feature Mask */
/**@}*/

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name Cycling Power Service Handles
 *
 */
/**@{*/
#define CPS_START_HDL               0x0400            /*!< \brief Start handle. */
#define CPS_END_HDL                 (CPS_MAX_HDL - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief Cycling Power Service Handles */
enum
{
  CPS_SVC_HDL = CPS_START_HDL,        /*!< \brief Cycling Power Server Service declaration */
  CPS_CPF_CH_HDL,                     /*!< \brief Cycling Power Feature characteristic */
  CPS_CPF_HDL,                        /*!< \brief Cycling Power Feature */
  CPS_CPM_CH_HDL,                     /*!< \brief Cycling Power Measurement characteristic */
  CPS_CPM_HDL,                        /*!< \brief Cycling Power Measurement */
  CPS_CPM_CH_CCC_HDL,                 /*!< \brief Cycling Power Measurement Client Characteristic Configuration Descriptor */
  CPS_CPSL_CH_HDL,                    /*!< \brief Cycling Power Sensor Location characteristic */
  CPS_CPSL_HDL,                       /*!< \brief Cycling Power Sensor Location */
  CPS_MAX_HDL                         /*!< \brief Maximum handle. */
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
void SvcCpsAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcCpsRemoveGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Register callbacks for the service.
 *
 *  \param  readCback   Read callback function.
 *  \param  writeCback  Write callback function.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcCpsCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback);

/*! \} */    /* CYCLING_POWER_SERVICE */

#ifdef __cplusplus
};
#endif

#endif /* SVC_CPS_H */
