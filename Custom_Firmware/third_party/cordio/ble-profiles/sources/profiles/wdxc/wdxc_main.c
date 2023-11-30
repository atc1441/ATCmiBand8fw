/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Wireless Data Exchange profile client.
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
#include "wsf_os.h"
#include "sec_api.h"
#include "util/bstream.h"
#include "dm_api.h"
#include "att_api.h"
#include "app_api.h"
#include "svc_wdxs.h"
#include "wdx_defs.h"
#include "wdxc_api.h"
#include "wdxc_main.h"

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! WDXC control block */
wdxcCb_t wdxcCb;

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! UUIDs */
static const uint8_t wdxcDcUuid[ATT_128_UUID_LEN] = {WDX_DC_UUID};     /* WDX Device Configuration Characteristic */
static const uint8_t wdxcFtcUuid[ATT_128_UUID_LEN] = {WDX_FTC_UUID};   /* WDX File Transfer Control Characteristic */
static const uint8_t wdxcFtdUuid[ATT_128_UUID_LEN] = {WDX_FTD_UUID};   /* WDX File Transfer Data Characteristic */
static const uint8_t wdxcAuUuid[ATT_128_UUID_LEN] = {WDX_AU_UUID};     /* WDX Authentication Characteristic */

/*! WDXC Device Configuration */
static const attcDiscChar_t wdxcWdxsDc =
{
  wdxcDcUuid,
  ATTC_SET_UUID_128
};

/*! WDXC Device Configuration CCC descriptor */
static const attcDiscChar_t wdxcWdxsDcCcc =
{
  attCliChCfgUuid,
  ATTC_SET_DESCRIPTOR
};

/*! WDXC File Transfer Control */
static const attcDiscChar_t wdxcWdxsFtc =
{
  wdxcFtcUuid,
  ATTC_SET_UUID_128
};

/*! WDXC File Transfer Control CCC descriptor */
static const attcDiscChar_t wdxcWdxsFtcCcc =
{
  attCliChCfgUuid,
  ATTC_SET_DESCRIPTOR
};

/*! WDXC File Transfer Data */
static const attcDiscChar_t wdxcWdxsFtd =
{
  wdxcFtdUuid,
  ATTC_SET_UUID_128
};

/*! WDXC File Transfer Data CCC descriptor */
static const attcDiscChar_t wdxcWdxsFtdCcc =
{
  attCliChCfgUuid,
  ATTC_SET_DESCRIPTOR
};

/*! WDXC Authentication */
static const attcDiscChar_t wdxcWdxsAu =
{
  wdxcAuUuid,
  ATTC_SET_UUID_128
};

/*! WDXC Authentication CCC descriptor */
static const attcDiscChar_t wdxcWdxsAuCcc =
{
  attCliChCfgUuid,
  ATTC_SET_DESCRIPTOR
};

/*! List of characteristics to be discovered; order matches handle index enumeration  */
static const attcDiscChar_t *wdxcWdxsDiscCharList[] =
{
  &wdxcWdxsDc,                      /*! WDXC Device Configuration */
  &wdxcWdxsDcCcc,                   /*! WDXC Device Configuration CCC descriptor */
  &wdxcWdxsFtc,                     /*! WDXC File Transfer Control */
  &wdxcWdxsFtcCcc,                  /*! WDXC File Transfer Control CCC descriptor */
  &wdxcWdxsFtd,                     /*! WDXC File Transfer Data */
  &wdxcWdxsFtdCcc,                  /*! WDXC File Transfer Data CCC descriptor */
  &wdxcWdxsAu,                      /*! WDXC Authentication */
  &wdxcWdxsAuCcc,                   /*! WDXC Authentication CCC descriptor */
};

/*! sanity check:  make sure handle list length matches characteristic list length */
WSF_CT_ASSERT(WDXC_HDL_LIST_LEN == ((sizeof(wdxcWdxsDiscCharList) / sizeof(attcDiscChar_t *))));

const uint8_t wdxcSvcUuid[ATT_16_UUID_LEN] =    {UINT16_TO_BYTES(WDX_SVC_UUID)};

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for Wireless Data Exchange service.
 *          Parameter pHdlList must point to an array of length WDXC_WDX_HDL_LIST_LEN.
 *          If discovery is successful the handles of discovered characteristics and
 *          descriptors will be set in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxcWdxsDiscover(dmConnId_t connId, uint16_t *pHdlList)
{
  wdxcConnCb_t *pConnCb = &wdxcCb.conn[connId - 1];

  /* Store pointer to the attribute handles in the control block */
  pConnCb->pHdlList = pHdlList;

  /* Perform service discovery */
  AppDiscFindService(connId, ATT_16_UUID_LEN, (uint8_t *) wdxcSvcUuid,
                     WDXC_HDL_LIST_LEN, (attcDiscChar_t **) wdxcWdxsDiscCharList, pHdlList);
}

/*************************************************************************************************/
/*!
 *  \brief  Perform File Discovery
 *
 *  \param  connId        Connection ID.
 *  \param  pFileInfo     Buffer to hold information about files
 *  \param  maxFiles      Size of pFileInfo in number of wsfEfsFileInfo_t objects
 *
 *  \note   When discovery is complete, the ftcCallback will be called with op equal to
 *          WDX_FTC_OP_EOF and the file handle equal to WDX_FLIST_HANDLE.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxcDiscoverFiles(dmConnId_t connId, wsfEfsFileInfo_t *pFileInfo, uint8_t maxFiles)
{
  wdxcConnCb_t *pConnCb = &wdxcCb.conn[connId - 1];

  /* Verify WDXC is Idle */
  if (pConnCb->fileHdl == WSF_EFS_INVALID_HANDLE)
  {
    uint16_t len = maxFiles * WDX_FLIST_RECORD_SIZE + WDX_FLIST_HDR_SIZE;

    /* Update control information */
    pConnCb->pFileList = pFileInfo;
    pConnCb->fileCount = 0;
    pConnCb->maxFiles = maxFiles;
    pConnCb->fileHdl = WDX_FLIST_HANDLE;
    pConnCb->fDlPos = 0;

    /* Perform a get on file zero (the file list handle) where the length of the get is
     * the most file information pFileInfo can hold */
    WdxcFtcSendGetReq(connId, WDX_FLIST_HANDLE, 0, len, 0);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Parse File Listing FTD data
 *
 *  \param  pValue        FTD Data.
 *  \param  len           Size of pValue in bytes
 *
 *  \return None.
 */
/*************************************************************************************************/
static void wdxsParseFileList(dmConnId_t connId, uint8_t *pValue, uint16_t len)
{
  wdxcConnCb_t *pConnCb = &wdxcCb.conn[connId - 1];
  uint16_t pos = 0;

  /* Depending on the MTU, blocks of FTD data from the file list may end mid-value.
   * Maintain a global position called wdxcCb.fDlPos, and process FTD data byte by byte */

  while (pos < len)
  {
    if (pConnCb->fDlPos < WDX_FLIST_HDR_SIZE)
    {
      /* Ignore file list header */
    }
    else
    {
      /* Find the file index and the position within the file index (the mark) */
      uint8_t mark = (pConnCb->fDlPos - WDX_FLIST_HDR_SIZE) % WDX_FLIST_RECORD_SIZE;
      uint8_t file = (pConnCb->fDlPos - WDX_FLIST_HDR_SIZE) / WDX_FLIST_RECORD_SIZE;
      wsfEfsFileInfo_t *pInfo;

      /* Ignore data if there is insufficient space in pConnCb->pFileList */
      if (file >= pConnCb->maxFiles)
      {
        return;
      }

      pInfo = &pConnCb->pFileList[file];

      /* Process a byte of data */
      switch (mark)
      {
      case 0: pConnCb->fileCount++; pInfo->handle = pValue[pos];  break;
      case 1: pInfo->handle |= ((uint16_t)pValue[pos]) << 8;    break;
      case 2: pInfo->attributes.type = pValue[pos];             break;
      case 3: pInfo->attributes.permissions = pValue[pos];      break;
      case 4: pInfo->size = pValue[pos];                        break;
      case 5: pInfo->size |= ((uint32_t)pValue[pos]) << 8;      break;
      case 6: pInfo->size |= ((uint32_t)pValue[pos]) << 16;     break;
      case 7: pInfo->size |= ((uint32_t)pValue[pos]) << 24;     break;
      default:
        if (mark > 7 && mark < 8 + WSF_EFS_NAME_LEN)
        {
          pInfo->attributes.name[mark - 8] = pValue[pos];
        }
        else
        {
          pInfo->attributes.version[mark - (8 + WSF_EFS_NAME_LEN)] = pValue[pos];
        }
        break;
      }
    }

    pConnCb->fDlPos++;
    pos++;
  }
}

/*************************************************************************************************/
/*!
*  \brief  Parse a file transfer control message.
*
*  \param  connId    Connection identifier.
*  \param  pValue    Pointer to buffer containing value.
*  \param  len       length of buffer.
*
*  \return None.
*/
/*************************************************************************************************/
static void wdxcParseFtc(dmConnId_t connId, uint8_t *pValue, uint16_t len)
{
  /* Notification on a File Transfer Control (FTC) Attribute */
  wdxcConnCb_t *pConnCb = &wdxcCb.conn[connId - 1];
  uint8_t *p = pValue;
  uint8_t op, status;
  uint16_t handle;

  BSTREAM_TO_UINT8(op, p);
  BSTREAM_TO_UINT16(handle, p);

  if (op == WDX_FTC_OP_ABORT || op == WDX_FTC_OP_EOF)
  {
    pConnCb->fileHdl = WSF_EFS_INVALID_HANDLE;
    status = WDX_FTC_ST_SUCCESS;
  }
  else
  {
    BSTREAM_TO_UINT8(status, p)
  }

  /* Call appliation callback */
  if (wdxcCb.pFtdCallback)
  {
    (*wdxcCb.pFtcCallback)(connId, handle, op, status);
  }
}

/*************************************************************************************************/
/*!
*  \brief  Parse a wdxc file transfer data message.
*
*  \param  pValue    Pointer to buffer containing value.
*  \param  len       length of buffer.
*
*  \return None.
*/
/*************************************************************************************************/
static void wdxcParseFtd(dmConnId_t connId, uint8_t *pValue, uint16_t len)
{
  wdxcConnCb_t *pConnCb = &wdxcCb.conn[connId - 1];

  if (pConnCb->fileHdl == WDX_FLIST_HANDLE)
  {
    /* File Discovery is in progress */
    wdxsParseFileList(connId, pValue, len);
  }
  else if (pConnCb->fileHdl != WSF_EFS_INVALID_HANDLE)
  {
    /* Notify application of File Transfer Data (FTD) */
    if (wdxcCb.pFtdCallback)
    {
      (*wdxcCb.pFtdCallback)(connId, pConnCb->fileHdl, len, pValue);
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Process a value received in an ATT read response, notification, or indication
 *          message.  Parameter pHdlList must point to an array of length WDXC_WDX_HDL_LIST_LEN.
 *          If the attribute handle of the message matches a handle in the handle list the value
 *          is processed, otherwise it is ignored.
 *
 *  \param  pMsg      ATT callback message.
 *
 *  \return ATT_SUCCESS if handle is found, ATT_ERR_NOT_FOUND otherwise.
 */
/*************************************************************************************************/
static uint8_t wdxcWdxsValueUpdate(attEvt_t *pMsg)
{
  uint16_t *pHdlList = wdxcCb.conn[pMsg->hdr.param-1].pHdlList;
  uint8_t   status = ATT_SUCCESS;

  /* device configuration */
  if (pMsg->handle == pHdlList[WDXC_DC_HDL_IDX])
  {
    /* Not Supported */
  }
  /* file transfer control */
  else if (pMsg->handle == pHdlList[WDXC_FTC_HDL_IDX])
  {
    APP_TRACE_INFO0("WDXC file transfer control.");
    wdxcParseFtc((dmConnId_t) pMsg->hdr.param, pMsg->pValue, pMsg->valueLen);
  }
  /* file transfer data */
  else if (pMsg->handle == pHdlList[WDXC_FTD_HDL_IDX])
  {
    wdxcParseFtd((dmConnId_t) pMsg->hdr.param, pMsg->pValue, pMsg->valueLen);
  }
  /* authentication  */
  else if (pMsg->handle == pHdlList[WDXC_AU_HDL_IDX])
  {
    /* Not Supported */
  }
  else
  {
    status = ATT_ERR_NOT_FOUND;
  }

  return status;
}

/*************************************************************************************************/
/*!
 *  \brief  Called by application to notify the WDXC of System Events.
 *
 *  \param  pEvt   Pointer to the Event
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxcProcMsg(wsfMsgHdr_t *pEvt)
{
  switch (pEvt->event)
  {
    case ATTC_HANDLE_VALUE_NTF:
      wdxcWdxsValueUpdate((attEvt_t *)pEvt);
      break;

    case DM_CONN_CLOSE_IND:
      {
        wdxcConnCb_t *pConnCb = &wdxcCb.conn[pEvt->param - 1];

        /* Set file handle to invalid to indicate no file operation is in progress */
        pConnCb->fileHdl = WSF_EFS_INVALID_HANDLE;
      }
      break;

    default:
      break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize the WDXC.
 *
 *  \param  pFtdCallback    File transfer data callback
 *  \param  pFtcCallback    File transfer control callback
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxcInit(WdxcFtdCallback_t *pFtdCallback, WdxcFtcCallback_t *pFtcCallback)
{
  uint8_t i;

  APP_TRACE_INFO0("WDXC: WdxcHandlerInit");

  /* Initialize the control block */
  memset(&wdxcCb, 0, sizeof(wdxcCb));

  for (i = 0; i < DM_CONN_MAX; i++)
  {
    wdxcCb.conn[i].fileHdl = WSF_EFS_INVALID_HANDLE;
  }

  /* Store callbacks */
  wdxcCb.pFtcCallback = pFtcCallback;
  wdxcCb.pFtdCallback = pFtdCallback;
}
