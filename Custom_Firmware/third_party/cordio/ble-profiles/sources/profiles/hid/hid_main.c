/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Human Interface Device Profile.
 *
 *  Copyright (c) 2015-2019 Arm Ltd. All Rights Reserved.
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

#include <string.h>
#include "wsf_types.h"
#include "wsf_assert.h"
#include "wsf_trace.h"
#include "att_api.h"
#include "svc_hid.h"
#include "app_api.h"
#include "hid_api.h"

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! HID control block */
typedef struct
{
  const hidConfig_t         *pConfig;             /* HID Configuration passed in from the application */
} hidCb_t;

hidCb_t hidCb;

/*************************************************************************************************/
/*!
 *  \brief  Gets the attribute handle of a report given the type and id
 *
 *  \param  type    The type of report (HID_REPORT_TYPE_INPUT, HID_REPORT_TYPE_OUTPUT, HID_REPORT_TYPE_FEATURE)
 *  \param  id      The ID of the report
 *
 *  \return The attribute handle for the report or ATT_HANDLE_NONE if the report is not in the map.
 */
/*************************************************************************************************/
static uint16_t hidGetReportHandle(uint8_t type, uint8_t id)
{
  hidReportIdMap_t *pMap;
  uint8_t count;
  uint8_t i;

  WSF_ASSERT(hidCb.pConfig);
  WSF_ASSERT(hidCb.pConfig->pReportIdMap);

  pMap = hidCb.pConfig->pReportIdMap;
  count = hidCb.pConfig->reportIdMapSize;

  for (i = 0; i < count; i++)
  {
    if (pMap[i].type == type && pMap[i].id == id)
    {
      return pMap[i].handle;
    }
  }

  return ATT_HANDLE_NONE;
}

/*************************************************************************************************/
/*!
 *  \brief  Gets the type and ID of a report given a handle
 *
 *  \param  handle      The attribute handle
 *
 *  \return A pointer to the hidReportIdMap_t with the type and ID
 */
/*************************************************************************************************/
static hidReportIdMap_t *hidGetReportIdMap(uint16_t handle)
{
  hidReportIdMap_t *pMap;
  uint8_t count;
  uint8_t i;

  WSF_ASSERT(hidCb.pConfig);
  WSF_ASSERT(hidCb.pConfig->pReportIdMap);

  pMap = hidCb.pConfig->pReportIdMap;
  count = hidCb.pConfig->reportIdMapSize;

  for (i = 0; i < count; i++)
  {
    if (pMap[i].handle == handle)
    {
      return &pMap[i];
    }
  }

  return NULL;
}

/*************************************************************************************************/
/*!
 *  \brief  Gets the HID control point value.
 *
 *  \return The control point value (HID_CONTROL_POINT_SUSPEND or HID_CONTROL_POINT_RESUME).
 */
/*************************************************************************************************/
uint8_t HidGetControlPoint(void)
{
  uint16_t len = 1;
  uint8_t *pValue = NULL;

  AttsGetAttr(HID_CONTROL_POINT_HDL, &len, &pValue);

  return *pValue;
}

/*************************************************************************************************/
/*!
 *  \brief  Gets the HID protocol mode value.
 *
 *  \return The protocol mode value (HID_PROTOCOL_MODE_REPORT or HID_PROTOCOL_MODE_BOOT).
 */
/*************************************************************************************************/
uint8_t HidGetProtocolMode(void)
{
  uint16_t len = 1;
  uint8_t *pValue = NULL;

  AttsGetAttr(HID_PROTOCOL_MODE_HDL, &len, &pValue);

  return *pValue;
}

/*************************************************************************************************/
/*!
 *  \brief  Sets the HID protocol mode for keyboard and mouse devices that support Boot Mode.
 *
 *  \param  protocolMode    The protocol mode (HID_PROTOCOL_MODE_REPORT or HID_PROTOCOL_MODE_BOOT)
 *
 *  \return None.
 */
/*************************************************************************************************/
void HidSetProtocolMode(uint8_t protocolMode)
{
  AttsSetAttr(HID_PROTOCOL_MODE_HDL, 1, &protocolMode);
}

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
void HidSendInputReport(dmConnId_t connId, uint8_t reportId, uint16_t len, uint8_t *pValue)
{
  uint16_t handle = hidGetReportHandle(HID_REPORT_TYPE_INPUT, reportId);

  if (handle != ATT_HANDLE_NONE)
  {
    /* Store the attribute value */
    AttsSetAttr(handle, len, pValue);

    /* send notification */
    AttsHandleValueNtf(connId, handle, len, pValue);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Called on an ATTS Write to the HID Service.
 *
 *  \return ATT status.
 *
 */
/*************************************************************************************************/
uint8_t HidAttsWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                          uint16_t offset, uint16_t len, uint8_t *pValue,
                          attsAttr_t *pAttr)
{
  hidReportIdMap_t *pIdMap;

  WSF_ASSERT(hidCb.pConfig);

  switch (handle)
  {
  case HID_CONTROL_POINT_HDL:

    /* notify the application */
    if (hidCb.pConfig->infoCback != NULL)
    {
      hidCb.pConfig->infoCback(connId, HID_INFO_CONTROL_POINT, *pValue);
    }
    break;

  case HID_PROTOCOL_MODE_HDL:

    /* Record the value of the protocol mode */
    HidSetProtocolMode(pValue[0]);

    if (hidCb.pConfig->infoCback != NULL)
    {
      hidCb.pConfig->infoCback(connId, HID_INFO_PROTOCOL_MODE, *pValue);
    }
    break;

  case HID_KEYBOARD_BOOT_OUT_HDL:

    /* set the attribute value so it can be read by the host */
    AttsSetAttr(handle, len, pValue);

    /* notify the application */
    if (hidCb.pConfig->outputCback != NULL)
    {
      pIdMap = hidGetReportIdMap(handle);

      if (pIdMap != NULL)
      {
        hidCb.pConfig->outputCback(connId, pIdMap->id, len, pValue);
      }
    }
    break;

  default:

    pIdMap = hidGetReportIdMap(handle);

    if (pIdMap != NULL)
    {
      /* set the attribute value so it can be read by the host */
      AttsSetAttr(handle, len, pValue);

      /* notify the application */
      if (pIdMap->type == HID_REPORT_TYPE_FEATURE)
      {
        if (hidCb.pConfig->featureCback != NULL)
        {
          hidCb.pConfig->featureCback(connId, pIdMap->id, len, pValue);
        }
      }
      else if (pIdMap->type == HID_REPORT_TYPE_OUTPUT)
      {
        if (hidCb.pConfig->outputCback != NULL)
        {
          hidCb.pConfig->outputCback(connId, pIdMap->id, len, pValue);
        }
      }
    }
    break;
  }

  return 0;
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize the HID profile.
 *
 *  \param  pConfig     HID Configuration structure
 *
 *  \return None.
 */
/*************************************************************************************************/
void HidInit(const hidConfig_t *pConfig)
{
  WSF_ASSERT(pConfig);

  /* Store the configuration */
  hidCb.pConfig = pConfig;
}
