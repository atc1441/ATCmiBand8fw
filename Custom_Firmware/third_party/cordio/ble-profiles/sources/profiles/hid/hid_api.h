/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Human Interface Device Profile.
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
#ifndef HID_API_H
#define HID_API_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup HUMAN_INTERFACE_DEVICE_PROFILE
 *  \{ */

/**************************************************************************************************
  Constant Values
**************************************************************************************************/

/** \name Type of HID Information
 *
 */
/**@{*/
#define HID_INFO_CONTROL_POINT                  0  /*!< \brief Control point information. */
#define HID_INFO_PROTOCOL_MODE                  1  /*!< \brief Protocol mode information. */
/**@}*/

/** \name HID Boot Report ID
 *
 */
/**@{*/
#define HID_KEYBOARD_BOOT_ID                    0xFF  /*!< \brief Keyboard boot ID. */
#define HID_MOUSE_BOOT_ID                       0xFE  /*!< \brief Mouse boot ID. */
/**@}*/

/**************************************************************************************************
  Callback Function Types
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  This callback function sends a received Output Report to the application.
 *
 *  \param  connId    The connection identifier.
 *  \param  id        The ID of the report.
 *  \param  len       The length of the report data in pReport.
 *  \param  pReport   A buffer containing the report.
 *
 *  \return None.
 */
/*************************************************************************************************/
typedef void (*hidOutputReportCback_t)(dmConnId_t connId, uint8_t id, uint16_t len, uint8_t *pReport);

/*************************************************************************************************/
/*!
 *  \brief  This callback function sends a received Feature Report to the application.
 *
 *  \param  connId    The connection identifier.
 *  \param  id        The ID of the report.
 *  \param  len       The length of the report data in pReport.
 *  \param  pReport   A buffer containing the report.
 *
 *  \return None.
 */
/*************************************************************************************************/
typedef void (*hidFeatureReportCback_t)(dmConnId_t connId, uint8_t id, uint16_t len, uint8_t *pReport);

/*************************************************************************************************/
/*!
 *  \brief  This callback function notifies the application of a change in the protocol mode or
 *          control point from the host.
 *
 *  \param  connId    The connection identifier.
 *  \param  mode      The type of information (\ref HID_INFO_CONTROL_POINT or \ref HID_INFO_PROTOCOL_MODE)
 *  \param  value     The value of the information
 *
 *  \return None.
 */
/*************************************************************************************************/
typedef void (*hidInfoCback_t)(dmConnId_t connId, uint8_t type, uint8_t value);

/*! \brief HID Report Type/ID to Attribute handle map item */
typedef struct
{
  uint8_t type;     /*!< \brief Type */
  uint8_t id;       /*!< \brief Id */
  uint16_t handle;  /*!< \brief Handle */
} hidReportIdMap_t;

/*! \brief HID Profile Configuration */
typedef struct
{
  hidReportIdMap_t            *pReportIdMap;        /*!< \brief A map between report Type/ID and Attribute handle */
  uint8_t                     reportIdMapSize;      /*!< \brief The number of Reports in the ID map (pReportIdMap) */
  hidOutputReportCback_t      outputCback;          /*!< \brief Callback called on receipt of an Output Report */
  hidFeatureReportCback_t     featureCback;         /*!< \brief Callback called on receipt of a Feature Report */
  hidInfoCback_t              infoCback;            /*!< \brief Callback called on receipt of protocol mode or control point */
} hidConfig_t;

/**************************************************************************************************
  API Functions
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Sends an input report to the host
 *
 *  \param  connId      The connection ID
 *  \param  reportId    The Report ID
 *  \param  len         The length of the report in bytes
 *  \param  pValue      A buffer containing the report
 *
 *  \return none.
 */
/*************************************************************************************************/
void HidSendInputReport(dmConnId_t connId, uint8_t reportId, uint16_t len, uint8_t *pValue);

/*************************************************************************************************/
/*!
 *  \brief  Sets the HID protocol mode for keyboard and mouse devices that support Boot Mode.
 *
 *  \param  protocolMode    The protocol mode (\ref HID_PROTOCOL_MODE_REPORT or \ref HID_PROTOCOL_MODE_BOOT)
 *
 *  \return None.
 */
/*************************************************************************************************/
void HidSetProtocolMode(uint8_t protocolMode);

/*************************************************************************************************/
/*!
 *  \brief  Gets the HID protocol mode value.
 *
 *  \return The protocol mode value (\ref HID_PROTOCOL_MODE_REPORT or \ref HID_PROTOCOL_MODE_BOOT).
 */
/*************************************************************************************************/
uint8_t HidGetProtocolMode(void);

/*************************************************************************************************/
/*!
 *  \brief  Gets the HID control point value.
 *
 *  \return The control point value (\ref HID_CONTROL_POINT_SUSPEND or \ref HID_CONTROL_POINT_RESUME).
 */
/*************************************************************************************************/
uint8_t HidGetControlPoint(void);

/*************************************************************************************************/
/*!
 *  \brief  Initialize the HID profile.
 *
 *  \param  pConfig     HID Configuration structure
 *
 *  \return None.
 */
/*************************************************************************************************/
void HidInit(const hidConfig_t *pConfig);

/*************************************************************************************************/
/*!
 *  \brief  Called on an ATTS Write to the HID Service.
 *
 *  \param  connId      DM connection identifier.
 *  \param  handle      ATT handle.
 *  \param  operation   ATT operation.
 *  \param  offset      Write offset.
 *  \param  len         Write length.
 *  \param  pValue      Value to write.
 *  \param  pAttr       Attribute to write.
 *
 *  \return ATT status.
 *
 */
/*************************************************************************************************/
uint8_t HidAttsWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                          uint16_t offset, uint16_t len, uint8_t *pValue,
                          attsAttr_t *pAttr);


/*! \} */    /* HUMAN_INTERFACE_DEVICE_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* HID_API_H */
