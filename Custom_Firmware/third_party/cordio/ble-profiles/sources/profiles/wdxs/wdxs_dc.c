/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Wireless Data Exchange profile implementation - Device Configuration.
 *
 *  Copyright (c) 2013-2018 Arm Ltd. All Rights Reserved.
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
#include "util/wstr.h"
#include "wsf_trace.h"
#include "wsf_assert.h"
#include "wsf_efs.h"
#include "util/bstream.h"
#include "svc_wdxs.h"
#include "wdxs_api.h"
#include "wdxs_main.h"
#include "dm_api.h"
#include "app_api.h"
#include "app_hw.h"

#if WDXS_DC_ENABLED == TRUE

/* WDXS Device Configuration Control Block */
wdxsDcCb_t wdxsDcCb;

/*************************************************************************************************/
/*!
 *  \brief  Send device configuration notification
 *
 *  \return None.
 */
/*************************************************************************************************/
void wdxsDcSend(dmConnId_t connId)
{
  APP_TRACE_INFO0("WDXS: DcSend");

  /* if notification enabled */
  if (AttsCccEnabled(connId, wdxsCb.dcCccIdx))
  {
    /* send notification */
    AttsHandleValueNtf(connId, WDXS_DC_HDL, wdxsDcCb.dcMsgLen, wdxsDcCb.dcMsgBuf);
    wdxsCb.txReadyMask &= ~(WDXS_TX_MASK_DC_BIT | WDXS_TX_MASK_READY_BIT);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Send update message for connection parameters.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t wdxsDcUpdateConnParam(dmConnId_t connId, uint8_t status)
{
  uint8_t *p;

  /* if update already waiting to be sent */
  if (wdxsCb.txReadyMask & WDXS_TX_MASK_DC_BIT)
  {
    return ATT_ERR_IN_PROGRESS;
  }

  /* build update to global buffer */
  p = wdxsDcCb.dcMsgBuf;
  UINT8_TO_BSTREAM(p, WDX_DC_OP_UPDATE);
  UINT8_TO_BSTREAM(p, WDX_DC_ID_CONN_PARAM);
  UINT8_TO_BSTREAM(p, status);
  UINT16_TO_BSTREAM(p, wdxsCb.connInterval);
  UINT16_TO_BSTREAM(p, wdxsCb.connLatency);
  UINT16_TO_BSTREAM(p, wdxsCb.supTimeout);
  wdxsDcCb.dcMsgLen = WDX_DC_LEN_CONN_PARAM + WDX_DC_HDR_LEN;

  /* Indicate TX Ready */
  wdxsCb.txReadyMask |= WDXS_TX_MASK_DC_BIT;
  WsfSetEvent(wdxsCb.handlerId, WDXS_EVT_TX_PATH);

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Send update message for PHY.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t wdxsDcUpdatePhy(dmConnId_t connId, uint8_t status)
{
  uint8_t *p;

  /* if update already waiting to be sent */
  if (wdxsCb.txReadyMask & WDXS_TX_MASK_DC_BIT)
  {
    return ATT_ERR_IN_PROGRESS;
  }

  /* build update to global buffer */
  p = wdxsDcCb.dcMsgBuf;
  UINT8_TO_BSTREAM(p, WDX_DC_OP_UPDATE);
  UINT8_TO_BSTREAM(p, WDX_DC_ID_PHY);
  UINT8_TO_BSTREAM(p, status);
  UINT8_TO_BSTREAM(p, wdxsCb.txPhy);
  UINT8_TO_BSTREAM(p, wdxsCb.rxPhy);
  wdxsDcCb.dcMsgLen = WDX_DC_LEN_PHY + WDX_DC_HDR_LEN;

  /* Indicate TX Ready */
  wdxsCb.txReadyMask |= WDXS_TX_MASK_DC_BIT;
  WsfSetEvent(wdxsCb.handlerId, WDXS_EVT_TX_PATH);

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Process Update Diagnostics Complete.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t wdxsDcUpdateDiagnosticsComplete(dmConnId_t connId)
{
  uint8_t *p;

  /* if update already waiting to be sent */
  if (wdxsCb.txReadyMask & WDXS_TX_MASK_DC_BIT)
  {
    return ATT_ERR_IN_PROGRESS;
  }

  /* build update to global buffer */
  p = wdxsDcCb.dcMsgBuf;
  UINT8_TO_BSTREAM(p, WDX_DC_OP_UPDATE);
  UINT8_TO_BSTREAM(p, WDX_DC_ID_DIAGNOSTICS_COMPLETE);
  wdxsDcCb.dcMsgLen = WDX_DC_LEN_DIAG_COMPLETE + WDX_DC_HDR_LEN;

  /* Indicate TX Ready */
  wdxsCb.txReadyMask |= WDXS_TX_MASK_DC_BIT;
  WsfSetEvent(wdxsCb.handlerId, WDXS_EVT_TX_PATH);

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Process set connection paramter request.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t wdxsDcSetConnParamReq(dmConnId_t connId, uint16_t len, uint8_t *pValue)
{
  hciConnSpec_t connSpec;

  /* verify parameter length */
  if (len != WDX_DC_LEN_CONN_PARAM_REQ)
  {
    return ATT_ERR_LENGTH;
  }

  /* parse parameters */
  BSTREAM_TO_UINT16(connSpec.connIntervalMin, pValue);
  BSTREAM_TO_UINT16(connSpec.connIntervalMax, pValue);
  BSTREAM_TO_UINT16(connSpec.connLatency, pValue);
  BSTREAM_TO_UINT16(connSpec.supTimeout, pValue);

  /* request update to connection parameters */
  DmConnUpdate(connId, &connSpec);

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Process set diagnostics.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t wdxsDcSetEnterDiadnostics(dmConnId_t connId)
{
  return wdxsDcUpdateDiagnosticsComplete(connId);
}

/*************************************************************************************************/
/*!
 *  \brief  Process a Set Disconnect request.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t wdxsDcSetDisconnectReq(dmConnId_t connId, uint16_t len, uint8_t *pValue)
{
  AppConnClose(connId);
  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Process a Set Security request.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t wdxsDcSetSecurityReq(dmConnId_t connId, uint16_t len, uint8_t *pValue)
{
  uint8_t secLevel;

  /* verify parameter length */
  if (len != WDX_DC_LEN_SEC_LEVEL)
  {
    return ATT_ERR_LENGTH;
  }

  /* parse parameters */
  BSTREAM_TO_UINT8(secLevel, pValue);

  /* Enable Security */
  if (DmConnSecLevel(connId) != DM_SEC_LEVEL_NONE)
  {
    DmSecSlaveReq(connId, secLevel);
  }

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Process a Set Service Changed request.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t wdxsDcSetServiceChanged(dmConnId_t connId, uint16_t len, uint8_t *pValue)
{
  /* TBD */
  return ATT_ERR_NOT_SUP;
}

/*************************************************************************************************/
/*!
 *  \brief  Process a Set Delete Bonds request.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t wdxsDcSetDeleteBonds(dmConnId_t connId, uint16_t len, uint8_t *pValue)
{
  /* TBD */
  return ATT_ERR_NOT_SUP;
}

/*************************************************************************************************/
/*!
 *  \brief  Process a Set Disconnect And Reset request.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t wdxsDcSetDisconnectAndReset(dmConnId_t connId, uint16_t len, uint8_t *pValue)
{
  wdxsDcCb.doReset = TRUE;
  AppConnClose(connId);
  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Process a Get Connection Parameter request.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t wdxsDcGetConnParam(dmConnId_t connId, uint16_t len, uint8_t *pValue)
{
  return wdxsDcUpdateConnParam(connId, HCI_SUCCESS);
}

/*************************************************************************************************/
/*!
 *  \brief  Process a Get Security Level request.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t wdxsDcGetSecurityLevel(dmConnId_t connId, uint16_t len, uint8_t *pValue)
{
  uint8_t *p;

  /* if update already waiting to be sent */
  if (wdxsCb.txReadyMask & WDXS_TX_MASK_DC_BIT)
  {
    return ATT_ERR_IN_PROGRESS;
  }

  /* build update to global buffer */
  p = wdxsDcCb.dcMsgBuf;
  UINT8_TO_BSTREAM(p, WDX_DC_OP_UPDATE);
  UINT8_TO_BSTREAM(p, WDX_DC_ID_CONN_SEC_LEVEL);
  UINT8_TO_BSTREAM(p, DmConnSecLevel(connId));
  wdxsDcCb.dcMsgLen = WDX_DC_LEN_SEC_LEVEL + WDX_DC_HDR_LEN;

  /* Indicate TX Ready */
  wdxsCb.txReadyMask |= WDXS_TX_MASK_DC_BIT;
  WsfSetEvent(wdxsCb.handlerId, WDXS_EVT_TX_PATH);

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Process a Get Current ATT MTU request.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t wdxsDcGetAttMtu(dmConnId_t connId, uint16_t len, uint8_t *pValue)
{
  uint8_t *p;

  /* if update already waiting to be sent */
  if (wdxsCb.txReadyMask & WDXS_TX_MASK_DC_BIT)
  {
    return ATT_ERR_IN_PROGRESS;
  }

  /* build update to global buffer */
  p = wdxsDcCb.dcMsgBuf;
  UINT8_TO_BSTREAM(p, WDX_DC_OP_UPDATE);
  UINT8_TO_BSTREAM(p, WDX_DC_ID_ATT_MTU);
  UINT16_TO_BSTREAM(p, AttGetMtu(connId));
  wdxsDcCb.dcMsgLen = WDX_DC_LEN_ATT_MTU + WDX_DC_HDR_LEN;

  /* Indicate TX Ready */
  wdxsCb.txReadyMask |= WDXS_TX_MASK_DC_BIT;
  WsfSetEvent(wdxsCb.handlerId, WDXS_EVT_TX_PATH);

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Process a Get Battery Level request.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t wdxsDcGetBatteryLevel(dmConnId_t connId, uint16_t len, uint8_t *pValue)
{
  uint8_t *p;

  /* if update already waiting to be sent */
  if (wdxsCb.txReadyMask & WDXS_TX_MASK_DC_BIT)
  {
    return ATT_ERR_IN_PROGRESS;
  }

  /* build update to global buffer */
  p = wdxsDcCb.dcMsgBuf;
  UINT8_TO_BSTREAM(p, WDX_DC_OP_UPDATE);
  UINT8_TO_BSTREAM(p, WDX_DC_ID_BATTERY_LEVEL);

  /* add battery level */
  AppHwBattRead(p);
  wdxsDcCb.dcMsgLen = WDX_DC_LEN_BATTERY_LEVEL + WDX_DC_HDR_LEN;

  /* Indicate TX Ready */
  wdxsCb.txReadyMask |= WDXS_TX_MASK_DC_BIT;
  WsfSetEvent(wdxsCb.handlerId, WDXS_EVT_TX_PATH);

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Process a Get Device Model Number request.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t wdxsDcGetDeviceModel(dmConnId_t connId, uint16_t len, uint8_t *pValue)
{
  uint8_t *p;

  /* TODO: Add Device Model */
  char *pModelTxt = WDXS_DEVICE_MODEL;

  /* if update already waiting to be sent */
  if (wdxsCb.txReadyMask & WDXS_TX_MASK_DC_BIT)
  {
    return ATT_ERR_IN_PROGRESS;
  }

  /* build update to global buffer */
  p = wdxsDcCb.dcMsgBuf;
  UINT8_TO_BSTREAM(p, WDX_DC_OP_UPDATE);
  UINT8_TO_BSTREAM(p, WDX_DC_ID_MODEL_NUMBER);
  /* Potential buffer overrun is intentional to zero out fixed length field */
  /* coverity[overrun-buffer-arg] */
  WstrnCpy((char *)p, pModelTxt, WDX_DC_LEN_DEVICE_MODEL);
  wdxsDcCb.dcMsgLen = WDX_DC_LEN_DEVICE_MODEL + WDX_DC_HDR_LEN;

  /* Indicate TX Ready */
  wdxsCb.txReadyMask |= WDXS_TX_MASK_DC_BIT;
  WsfSetEvent(wdxsCb.handlerId, WDXS_EVT_TX_PATH);

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Process a Get Firmware Revision request.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t wdxsDcGetFirmwareRev(dmConnId_t connId, uint16_t len, uint8_t *pValue)
{
  uint8_t *p;

  /* TODO: Add Firmware Revision */
  char *pFirmwareRev = "1.0";

  /* if update already waiting to be sent */
  if (wdxsCb.txReadyMask & WDXS_TX_MASK_DC_BIT)
  {
    return ATT_ERR_IN_PROGRESS;
  }

  /* build update to global buffer */
  p = wdxsDcCb.dcMsgBuf;
  UINT8_TO_BSTREAM(p, WDX_DC_OP_UPDATE);
  UINT8_TO_BSTREAM(p, WDX_DC_ID_FIRMWARE_REV);

  /* Potential buffer overrun is intentional to zero out fixed length field */
  /* coverity[overrun-buffer-arg] */
  WstrnCpy((char *)p, pFirmwareRev, WDX_DC_LEN_FIRMWARE_REV);
  wdxsDcCb.dcMsgLen = WDX_DC_LEN_FIRMWARE_REV + WDX_DC_HDR_LEN;

  /* Indicate TX Ready */
  wdxsCb.txReadyMask |= WDXS_TX_MASK_DC_BIT;
  WsfSetEvent(wdxsCb.handlerId, WDXS_EVT_TX_PATH);

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Process a write to the device configuration characteristic.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t wdxsDcWrite(dmConnId_t connId, uint16_t len, uint8_t *pValue)
{
  uint8_t op;
  uint8_t id;
  uint8_t status = ATT_SUCCESS;

  /* sanity check on message length */
  if (len < WDX_DC_HDR_LEN)
  {
    return ATT_ERR_LENGTH;
  }

  /* verify notifications are enabled */
  if (!AttsCccEnabled(connId, wdxsCb.dcCccIdx))
  {
    return ATT_ERR_CCCD;
  }

  /* get operation and parameter ID */
  BSTREAM_TO_UINT8(op, pValue);
  BSTREAM_TO_UINT8(id, pValue);

  /* skip over header (note pValue was incremented above) */
  len -= WDX_DC_HDR_LEN;

  /* set operation */
  if (op == WDX_DC_OP_SET)
  {
    switch (id)
    {
      case WDX_DC_ID_CONN_UPDATE_REQ:
        status = wdxsDcSetConnParamReq(connId, len, pValue);
        break;

      case WDX_DC_ID_DISCONNECT_REQ:
        status = wdxsDcSetDisconnectReq(connId, len, pValue);
        break;

      case WDX_DC_ID_SECURITY_REQ:
        status = wdxsDcSetSecurityReq(connId, len, pValue);
        break;

      case WDX_DC_ID_SERVICE_CHANGED:
        status = wdxsDcSetServiceChanged(connId, len, pValue);
        break;

      case WDX_DC_ID_DELETE_BONDS:
        status = wdxsDcSetDeleteBonds(connId, len, pValue);
        break;

      case WDX_DC_ID_ENTER_DIAGNOSTICS:
        status = wdxsDcSetEnterDiadnostics(connId);
        break;

      case WDX_DC_ID_DISCONNECT_AND_RESET:
        status = wdxsDcSetDisconnectAndReset(connId, len, pValue);
        break;

      case WDX_DC_ID_PHY_UPDATE_REQ:
        /* if device configuration phy callback registered */
        if (wdxsDcCb.phyWriteCback != NULL)
        {
          status = (*wdxsDcCb.phyWriteCback)(connId, op, id, len, pValue);
          break;
        }
        /* else fall through */

      default:
        status = ATT_ERR_RANGE;
        break;
    }
  }
  /* get operation */
  else if (op == WDX_DC_OP_GET)
  {
    switch (id)
    {
      case WDX_DC_ID_CONN_PARAM:
        status = wdxsDcGetConnParam(connId, len, pValue);
        break;

      case WDX_DC_ID_CONN_SEC_LEVEL:
        status = wdxsDcGetSecurityLevel(connId, len, pValue);
        break;

      case WDX_DC_ID_ATT_MTU:
        status = wdxsDcGetAttMtu(connId, len, pValue);
        break;

      case WDX_DC_ID_BATTERY_LEVEL:
        status = wdxsDcGetBatteryLevel(connId, len, pValue);
        break;

      case WDX_DC_ID_MODEL_NUMBER:
        status = wdxsDcGetDeviceModel(connId, len, pValue);
        break;

      case WDX_DC_ID_FIRMWARE_REV:
        status = wdxsDcGetFirmwareRev(connId, len, pValue);
        break;

      case WDX_DC_ID_PHY:
        /* if device configuration phy callback registered */
        if (wdxsDcCb.phyWriteCback != NULL)
        {
          status = (*wdxsDcCb.phyWriteCback)(connId, op, id, len, pValue);
          break;
        }
        /* else fall through */

      default:
        status = ATT_ERR_RANGE;
        break;
    }
  }
  else
  {
    status = ATT_ERR_RANGE;
  }

  return status;
}

/*************************************************************************************************/
/*!
 *  \brief  Register a PHY write callback for the device configuration characteristic.
 *
 *  \param  cback  PHY callback function.
 *
 *  \return None.
 */
/*************************************************************************************************/
void wdxsDcPhyRegister(wdxsDcPhyWriteCback_t cback)
{
  wdxsDcCb.phyWriteCback = cback;
}

#endif  /* WDXS_DC_ENABLED */
