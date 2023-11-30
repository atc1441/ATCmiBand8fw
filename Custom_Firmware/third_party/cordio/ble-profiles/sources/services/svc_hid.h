/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Human Interface Device service implementation.
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

#ifndef SVC_HID_H
#define SVC_HID_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup HUMAN_INTERFACE_DEVICE_SERVICE
 *  \{ */

/**************************************************************************************************
Macros
**************************************************************************************************/

/*! \brief HID Service */
#define HID_SVC_UUID                  ATT_UUID_HID_SERVICE

/** \name HID Spec Version
 *
 */
/**@{*/
/*! \brief HID Spec Version: 1.11 */
#define HID_VERSION                   0x0111
/**@}*/

/** \name HID Report Types
 *
 */
/**@{*/
#define HID_REPORT_TYPE_INPUT         0x01  /*!< \brief Input type. */
#define HID_REPORT_TYPE_OUTPUT        0x02  /*!< \brief Output type. */
#define HID_REPORT_TYPE_FEATURE       0x03  /*!< \brief Feature type. */
/**@}*/

/** \name HID Protocol Mode Types
 *
 */
/**@{*/
#define HID_PROTOCOL_MODE_BOOT        0x00  /*!< \brief Boot mode. */
#define HID_PROTOCOL_MODE_REPORT      0x01  /*!< \brief Report mode. */
/**@}*/

/** \name HID Control Point Values
 *
 */
/**@{*/
#define HID_CONTROL_POINT_SUSPEND     0x00  /*!< \brief Suspend. */
#define HID_CONTROL_POINT_RESUME      0x01  /*!< \brief Resume. */
/**@}*/

/*! \brief Max length of the report map value */
#define HID_MAX_REPORT_MAP_LEN        512

/*! \brief Max length of an output report value */
#define HID_MAX_REPORT_LEN            32

/*! \brief Initial length of the report map value */
#define HID_INIT_REPORT_MAP_LEN       1

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name HID Service Handles
 *
 */
/**@{*/
#define HID_START_HDL                 0x50            /*!< \brief Start handle. */
#define HID_END_HDL                   (HID_MAX_HDL - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/
/*! \brief Proprietary Service Handles Common to HID Devices */
enum
{
  HID_SVC_HDL = HID_START_HDL,        /*!< \brief Proprietary Service Declaration */
  HID_INFO_CH_HDL,                    /*!< \brief HID Information Characteristic Declaration */
  HID_INFO_HDL,                       /*!< \brief HID Information Value */
  HID_REPORT_MAP_CH_HDL,              /*!< \brief HID Report Map Characteristic Declaration */
  HID_REPORT_MAP_HDL,                 /*!< \brief HID Report Map Value */
  HID_EXTERNAL_REPORT_HDL,            /*!< \brief HID External Report Descriptor */
  HID_CONTROL_POINT_CH_HDL,           /*!< \brief HID Control Point Characteristic Declaration */
  HID_CONTROL_POINT_HDL,              /*!< \brief HID Control Point Value */
  HID_KEYBOARD_BOOT_IN_CH_HDL,        /*!< \brief HID Keyboard Boot Input Characteristic Declaration */
  HID_KEYBOARD_BOOT_IN_HDL,           /*!< \brief HID Keyboard Boot Input Value */
  HID_KEYBOARD_BOOT_IN_CH_CCC_HDL,    /*!< \brief HID Keyboard Boot Input CCC Descriptor */
  HID_KEYBOARD_BOOT_OUT_CH_HDL,       /*!< \brief HID Keyboard Boot Output Characteristic Declaration */
  HID_KEYBOARD_BOOT_OUT_HDL,          /*!< \brief HID Keyboard Boot Output Value */
  HID_MOUSE_BOOT_IN_CH_HDL,           /*!< \brief HID Mouse Boot Input Characteristic Declaration */
  HID_MOUSE_BOOT_IN_HDL,              /*!< \brief HID Mouse Boot Input Value */
  HID_MOUSE_BOOT_IN_CH_CCC_HDL,       /*!< \brief HID Mouse Boot Input CCC Descriptor */
  HID_INPUT_REPORT_1_CH_HDL,          /*!< \brief HID Input Report Characteristic Declaration */
  HID_INPUT_REPORT_1_HDL,             /*!< \brief HID Input Report Value */
  HID_INPUT_REPORT_1_CH_CCC_HDL,      /*!< \brief HID Input Report CCC Descriptor */
  HID_INPUT_REPORT_1_REFERENCE_HDL,   /*!< \brief HID Input Report Reference Descriptor */
  HID_INPUT_REPORT_2_CH_HDL,          /*!< \brief HID Input Report Characteristic Declaration */
  HID_INPUT_REPORT_2_HDL,             /*!< \brief HID Input Report Value */
  HID_INPUT_REPORT_2_CH_CCC_HDL,      /*!< \brief HID Input Report CCC Descriptor */
  HID_INPUT_REPORT_2_REFERENCE_HDL,   /*!< \brief HID Input Report Reference Descriptor */
  HID_INPUT_REPORT_3_CH_HDL,          /*!< \brief HID Input Report Characteristic Declaration */
  HID_INPUT_REPORT_3_HDL,             /*!< \brief HID Input Report Value */
  HID_INPUT_REPORT_3_CH_CCC_HDL,      /*!< \brief HID Input Report CCC Descriptor */
  HID_INPUT_REPORT_3_REFERENCE_HDL,   /*!< \brief HID Input Report Reference Descriptor */
  HID_OUTPUT_REPORT_CH_HDL,           /*!< \brief HID Output Report Characteristic Declaration */
  HID_OUTPUT_REPORT_HDL,              /*!< \brief HID Output Report Value */
  HID_OUTPUT_REPORT_REFERENCE_HDL,    /*!< \brief HID Output Report Reference Descriptor */
  HID_FEATURE_REPORT_CH_HDL,          /*!< \brief HID Feature Report Characteristic Declaration */
  HID_FEATURE_REPORT_HDL,             /*!< \brief HID Feature Report Value */
  HID_FEATURE_REPORT_REFERENCE_HDL,   /*!< \brief HID Feature Report Reference Descriptor */
  HID_PROTOCOL_MODE_CH_HDL,           /*!< \brief HID Protocol Mode Characteristic Declaration */
  HID_PROTOCOL_MODE_HDL,              /*!< \brief HID Protocol Mode Value */
  HID_MAX_HDL                         /*!< \brief Maximum handle. */
};
/**@}*/

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidRemoveGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Register a read and write callback functions for the ATT Group.
 *
 *  \param  writeCb   Write callback function
 *  \param  readCb    Read callback function
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidRegister(attsWriteCback_t writeCb, attsReadCback_t readCb);

/*************************************************************************************************/
/*!
 *  \brief  Add the Hid Service using the dynamic attribute subsystem.
 *
 *  \return None.
 */
/*************************************************************************************************/
void *SvcHidAddGroupDyn(void);

/*! \} */    /* HUMAN_INTERFACE_DEVICE_SERVICE */

#ifdef __cplusplus
}
#endif

#endif /* SVC_HID_H */

