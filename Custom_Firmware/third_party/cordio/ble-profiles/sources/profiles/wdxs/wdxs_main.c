/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Wireless Data Exchange profile implementation.
 *
 *  Copyright (c) 2013-2018 Arm Ltd. All Rights Reserved.
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

#include <string.h>
#include "wsf_types.h"
#include "util/wstr.h"
#include "wsf_trace.h"
#include "wsf_assert.h"
#include "wsf_efs.h"
#include "wsf_os.h"
#include "sec_api.h"
#include "util/bstream.h"
#include "svc_wdxs.h"
#include "wdxs_api.h"
#include "wdxs_main.h"
#include "dm_api.h"
#include "att_api.h"
#include "app_api.h"

#if defined(__CC_ARM)
/* The C standard does not permit convert short integer pointer to long integer pointer,
 * or it will generate compile warning "extended constant initialiser used #1296" in Keil.
 * This file needs to do the pointer conversion (uint8_t -> uint32_t)so we need to ignore
 * such warning at this moment to lower the code change risk.
 * Will remove this diag_suppress once find another implement code for the feature.
 */
#pragma diag_suppress 1296
#elif defined(__ICCARM__)
#elif defined(__GNUC__)
#endif

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! application control block */
wdxsCb_t wdxsCb;

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! RAM File Media Configuration */
#define WDXS_RAM_LOCATION         ((uint32_t)WdxsRamBlock)
#define WDXS_RAM_SIZE             (WDX_FLIST_MAX_LEN + WDXS_APP_RAM_MEDIA_SIZE)
#define WDXS_RAM_END              (WDXS_RAM_LOCATION + WDXS_RAM_SIZE)

/**************************************************************************************************
  Local Function Prototypes
**************************************************************************************************/
static uint8_t WdxsRamErase(uint8_t *pAddress, uint32_t size);
static uint8_t WdxsRamRead(uint8_t *pBuf, uint8_t *pAddress, uint32_t size);
static uint8_t WdxsRamWrite(const uint8_t *pBuf, uint8_t *pAddress, uint32_t size);

/**************************************************************************************************
  Function Prototypes
**************************************************************************************************/
void WdxsAuSecComplete(secAes_t *pAes);

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! Reserve RAM for use by the RAM EFS Media */
static uint8_t WdxsRamBlock[WDXS_RAM_SIZE];

/*! EFS RAM Media Control Block */
static wsfEfsMedia_t WDXS_RamMediaCtrl =
{
  0,
  0,
  1,
  NULL,
  WdxsRamErase,
  WdxsRamRead,
  WdxsRamWrite,
  NULL
};

/*************************************************************************************************/
/*!
 *  \brief  Erase function for the EFS RAM media.
 *
 *  \return none.
 *
 */
/*************************************************************************************************/
static uint8_t WdxsRamErase(uint8_t *pAddress, uint32_t size)
{
  memset(pAddress, 0xFF, size);
  return TRUE;
}

/*************************************************************************************************/
/*!
 *  \brief  Read function for the EFS RAM media.
 *
 *  \return none.
 *
 */
/*************************************************************************************************/
static uint8_t WdxsRamRead(uint8_t *pBuf, uint8_t *pAddress, uint32_t size)
{
  memcpy(pBuf, pAddress, size);
  return TRUE;
}

/*************************************************************************************************/
/*!
 *  \brief  Write function for the EFS RAM media.
 *
 *  \return none.
 *
 */
/*************************************************************************************************/
static uint8_t WdxsRamWrite(const uint8_t *pBuf, uint8_t *pAddress, uint32_t size)
{
  memcpy(pAddress, pBuf, size);
  return TRUE;
}

/*************************************************************************************************/
/*!
 *  \brief  Format file list information for the given file.
 *
 *  \return none.
 */
/*************************************************************************************************/
static void wdxsFormatFileResource(uint8_t *pData, wsfEfsHandle_t handle)
{
  UINT16_TO_BSTREAM(pData, handle);
  UINT8_TO_BSTREAM(pData, WsfEfsGetFileType(handle));
  UINT8_TO_BSTREAM(pData, WsfEfsGetFilePermissions(handle) & WSF_EFS_REMOTE_PERMISSIONS_MASK);
  UINT32_TO_BSTREAM(pData, WsfEfsGetFileSize(handle));
  WstrnCpy((char *)pData, WsfEfsGetFileName(handle), WSF_EFS_NAME_LEN);
  WstrnCpy((char *)pData+WSF_EFS_NAME_LEN, WsfEfsGetFileVersion(handle), WSF_EFS_VERSION_LEN);
}

/*************************************************************************************************/
/*!
 *  \brief  Create the file list.
 *
 *  \return none.
 */
/*************************************************************************************************/
void WdxsUpdateListing(void)
{
  uint8_t *pTmp;
  uint8_t header[WDX_FLIST_HDR_SIZE];
  uint8_t record[WDX_FLIST_RECORD_SIZE];
  uint32_t position = 0, totalSize = 0;
  uint32_t fileCount = 0;
  uint8_t i;

  position = WDX_FLIST_HDR_SIZE;

  for (i=0; i<WSF_EFS_MAX_FILES; i++)
  {
    if (WsfEfsGetFileByHandle(i) && (WsfEfsGetFilePermissions(i) & WSF_EFS_REMOTE_VISIBLE))
    {
      /* Update the total size and file count */
      totalSize += WsfEfsGetFileSize(i);
      fileCount++;

      wdxsFormatFileResource(record, i);

      /* Write the record */
      WsfEfsPut(WDX_FLIST_HANDLE, position, record, WDX_FLIST_RECORD_SIZE);
      position += WDX_FLIST_RECORD_SIZE;
    }
  }

  /* Add the header after calculating the total_size and file_count */
  pTmp = header;
  UINT8_TO_BSTREAM(pTmp, WDX_FLIST_FORMAT_VER);
  UINT16_TO_BSTREAM(pTmp, fileCount);
  UINT32_TO_BSTREAM(pTmp, totalSize);

  /* Write the header */
  WsfEfsPut(WDX_FLIST_HANDLE, 0, header, WDX_FLIST_HDR_SIZE);
}

/*************************************************************************************************/
/*!
 *  \brief  Set the CCCD index used by the application for WDXS service characteristics.
 *
 *  \param  dcCccIdx   Device Control CCCD index.
 *  \param  auCccIdx   Authentication CCCD index.
 *  \param  ftcCccIdx  File Transfer Control CCCD index.
 *  \param  ftdCccIdx  File Transfer Data CCCD index.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsSetCccIdx(uint8_t dcCccIdx, uint8_t auCccIdx, uint8_t ftcCccIdx, uint8_t ftdCccIdx)
{
  wdxsCb.dcCccIdx = dcCccIdx;
  wdxsCb.auCccIdx = auCccIdx;
  wdxsCb.ftcCccIdx = ftcCccIdx;
  wdxsCb.ftdCccIdx = ftdCccIdx;
}

/*************************************************************************************************/
/*!
 *  \brief  ATTS write callback for proprietary service.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t wdxsWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                       uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr)
{
  uint8_t status;

#if WDXS_AU_ENABLED == TRUE

  /* Require peer authentication before writing to any characteristic
    (except for the authentication characteristic) */
  if ((wdxsAuCb.reqAuthLevel != WDX_AU_LVL_NONE) && (handle != WDXS_AU_HDL))
  {
    if ((wdxsAuCb.authState != WDXS_AU_STATE_AUTHORIZED) || (wdxsAuCb.authLevel < wdxsAuCb.reqAuthLevel))
    {
      APP_TRACE_INFO1("WDXS: WriteCback unauthorized state=%d", wdxsAuCb.authState);
      return WDX_APP_AUTH_REQUIRED;
    }
  }

#endif /* WDXS_AU_ENABLED */

  switch (handle)
  {
#if WDXS_DC_ENABLED == TRUE
    /* Device configuration */
    case WDXS_DC_HDL:
      status = wdxsDcWrite(connId, len, pValue);
      break;
#endif /* WDXS_DC_ENABLED */

    /* File transfer control */
    case WDXS_FTC_HDL:
      status = wdxsFtcWrite(connId, len, pValue);
      break;

    /* File transfer data */
    case WDXS_FTD_HDL:
      status = wdxsFtdWrite(connId, len, pValue);
      break;

#if WDXS_AU_ENABLED == TRUE
    /* Authentication */
    case WDXS_AU_HDL:
      status = wdxsAuWrite(connId, len, pValue);
      break;
#endif /* WDXS_AU_ENABLED */

    default:
      APP_TRACE_INFO1("WDXS: WriteCback unexpected handle=%d", handle);
      status = ATT_ERR_HANDLE;
      break;
  }


  return status;
}

/*************************************************************************************************/
/*!
 *  \brief  Process TX data path
 *
 *  \return None.
 */
/*************************************************************************************************/
static void wdxsProcTxPath(void)
{
  dmConnId_t  connId;

  /* Check for a connection */
  if ((connId = AppConnIsOpen()) != DM_CONN_ID_NONE)
  {
    /* Check if ready to transmit a message */
    if (wdxsCb.txReadyMask & WDXS_TX_MASK_READY_BIT)
    {

#if WDXS_DC_ENABLED == TRUE
      /* Device configuration */
      if (wdxsCb.txReadyMask & WDXS_TX_MASK_DC_BIT)
      {
        wdxsDcSend(connId);
        return;
      }
#endif /* WDXS_DC_ENABLED */

      /* File Transfer Control */
      if (wdxsCb.txReadyMask & WDXS_TX_MASK_FTC_BIT)
      {
        wdxsFtcSend(connId);
        return;
      }

#if WDXS_AU_ENABLED == TRUE
      /* Authentication */
      if (wdxsCb.txReadyMask & WDXS_TX_MASK_AU_BIT)
      {
        wdxsAuSend(connId);
        return;
      }
#endif /* WDXS_AU_ENABLED */

      /* File Transfer Data */
      if (wdxsCb.txReadyMask & WDXS_TX_MASK_FTD_BIT)
      {
        wdxsFtdSend(connId);
        return;
      }
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  WSF event handler for application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
  APP_TRACE_INFO1("WDXS: Task Handler Evt=%d", event);

  if (event & WDXS_EVT_TX_PATH)
  {
    wdxsProcTxPath();
  }

#if WDXS_AU_ENABLED == TRUE

  if (event & WDXS_EVT_AU_SEC_COMPLETE)
  {
    WdxsAuSecComplete((secAes_t*) pMsg);
  }

#endif /* WDXS_AU_ENABLED */
}

/*************************************************************************************************/
/*!
 *  \brief  Called by application to notify the WDXS of DM Events.
 *
 *  \param  pEvt   Pointer to the DM Event
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsProcDmMsg(dmEvt_t *pEvt)
{
  switch (pEvt->hdr.event)
  {
    case DM_CONN_CLOSE_IND:
      if (wdxsDcCb.doReset)
      {
        WdxsResetSystem();
      }
      break;

    case DM_CONN_OPEN_IND:
      /* Initialize connection parameters */
      wdxsCb.txReadyMask = WDXS_TX_MASK_READY_BIT;
      wdxsCb.ftInProgress = WDX_FTC_OP_NONE;
      wdxsCb.ftLen = 0;
      wdxsCb.ftOffset = 0;
#if WDXS_AU_ENABLED == TRUE
      wdxsAuCb.authLevel = WDX_AU_LVL_NONE;
      wdxsAuCb.authState = WDXS_AU_STATE_UNAUTHORIZED;
#endif /* WDXS_AU_ENABLED */
      wdxsCb.connInterval = pEvt->connOpen.connInterval;
      wdxsCb.connLatency = pEvt->connOpen.connLatency;
      wdxsCb.supTimeout = pEvt->connOpen.supTimeout;
      wdxsCb.txPhy= HCI_PHY_LE_1M_BIT;
      wdxsCb.rxPhy = HCI_PHY_LE_1M_BIT;
      break;

    case DM_CONN_UPDATE_IND:
      if (pEvt->hdr.status == HCI_SUCCESS)
      {
        wdxsCb.connInterval = pEvt->connUpdate.connInterval;
        wdxsCb.connLatency = pEvt->connUpdate.connLatency;
        wdxsCb.supTimeout = pEvt->connUpdate.supTimeout;
      }
      wdxsDcUpdateConnParam((dmConnId_t) pEvt->hdr.param, pEvt->hdr.status);
      break;

    case DM_PHY_UPDATE_IND:
      if (pEvt->hdr.status == HCI_SUCCESS)
      {
        wdxsCb.txPhy = pEvt->phyUpdate.txPhy;
        wdxsCb.rxPhy = pEvt->phyUpdate.rxPhy;
      }
      wdxsDcUpdatePhy((dmConnId_t) pEvt->hdr.param, pEvt->hdr.status);
      break;

    default:
      break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Called by application to notify the WDXS of ATT Events.
 *
 *  \param  pEvt   Pointer to the ATT Event
 *
 *  \return None.
 */
/*************************************************************************************************/
uint8_t WdxsAttCback(attEvt_t *pEvt)
{
  if (pEvt->handle < WDXS_START_HDL || pEvt->handle > WDXS_END_HDL)
  {
    return FALSE;
  }

  APP_TRACE_INFO2("WDXS: AttHook handle=%d event=%d", pEvt->handle, pEvt->hdr.event);

  /* trigger tx data path on confirm */
  if (pEvt->hdr.event == ATTS_HANDLE_VALUE_CNF &&
      pEvt->hdr.status == ATT_SUCCESS)
  {
    wdxsCb.txReadyMask |= WDXS_TX_MASK_READY_BIT;
    WsfSetEvent(wdxsCb.handlerId, WDXS_EVT_TX_PATH);
  }

  return TRUE;
}

/*************************************************************************************************/
/*!
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsHandlerInit(wsfHandlerId_t handlerId)
{
  wsfEsfAttributes_t attr;

  APP_TRACE_INFO0("WDXS: WdxsHandlerInit");

  /* Initialize the EFS RAM Media Control Block */
  WDXS_RamMediaCtrl.startAddress = WDXS_RAM_LOCATION;
  WDXS_RamMediaCtrl.endAddress = WDXS_RAM_LOCATION + WDXS_RAM_SIZE,

  /* Initialize the control block */
  memset(&wdxsCb, 0, sizeof(wdxsCb));
  wdxsCb.txReadyMask = WDXS_TX_MASK_READY_BIT;

  /* Store Handler ID */
  wdxsCb.handlerId = handlerId;

  /* Initialize the device configuration control block */
  memset(&wdxsDcCb, 0, sizeof(wdxsDcCb));

  /* Register the WDXS Service */
  SvcWdxsRegister(wdxsWriteCback);
  SvcWdxsAddGroup();

  /* Initialize the embedded file system */
  WsfEfsInit();

  /* Register the RAM Media */
  memset(WdxsRamBlock, 0xFF, sizeof(WdxsRamBlock));
  WsfEfsRegisterMedia(&WDXS_RamMediaCtrl, WDX_RAM_MEDIA);

  /* Set attributes for the WDXS File List */
  attr.type = WSF_EFS_FILE_TYPE_BULK;
  attr.permissions = WSF_EFS_LOCAL_PUT_PERMITTED | WSF_EFS_REMOTE_GET_PERMITTED;
  WstrnCpy(attr.name, "Listing", WSF_EFS_NAME_LEN);
  WstrnCpy(attr.version, "1.0", WSF_EFS_VERSION_LEN);

  /* Create a file in RAM to contain the list WDXS File List */
  WsfEfsAddFile(WDX_FLIST_MAX_LEN, WDX_RAM_MEDIA, &attr, WSF_EFS_FILE_OFFSET_ANY);
}

/*************************************************************************************************/
/*!
 *  \fn     WdxsOtaMediaInit
 *
 *  \brief  Registers the platform dependent OTA Media with the Embedded File System (EFS)
 *
 *  \param  None
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsOtaMediaInit(void)
{
}

/*************************************************************************************************/
/*!
 *  \fn     WdxsResetSystem
 *
 *  \brief  Resets the system.
 *
 *  \param  None
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsResetSystem(void)
{
}

/*************************************************************************************************/
/*!
 *  \fn     WdxsFlashMediaInit
 *
 *  \brief  Registers the platform dependent Flash Media with the Embedded File System (EFS)
 *
 *  \param  None
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsFlashMediaInit(void)
{
}
