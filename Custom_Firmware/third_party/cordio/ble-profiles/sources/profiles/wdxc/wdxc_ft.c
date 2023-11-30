/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Wireless Data Exchange profile client - File Transfer.
 *
 *  Copyright (c) 2017-2018 Arm Ltd. All Rights Reserved.
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
#include <stddef.h>
#include "wsf_types.h"
#include "wsf_trace.h"
#include "wsf_assert.h"
#include "wsf_efs.h"
#include "util/bstream.h"
#include "svc_wdxs.h"
#include "wdx_defs.h"
#include "wdxc_api.h"
#include "wdxc_main.h"
#include "dm_api.h"
#include "app_api.h"

/**************************************************************************************************
  External Variables
**************************************************************************************************/
extern wdxcCb_t wdxcCb;

/*************************************************************************************************/
/*!
 *  \brief  Send a request to abort a wdx operation.
 *
 *  \param  connId          Connection ID.
 *  \param  fileHdl         Handle of file to abort operation on peer device
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxcFtcSendAbort(dmConnId_t connId, uint16_t fileHdl)
{
  uint8_t   buf[WDX_FTC_ABORT_LEN];
  uint8_t   *p = buf;
  uint16_t  handle;

  WSF_ASSERT(wdxcCb.conn[connId - 1].pHdlList != NULL)

  handle = wdxcCb.conn[connId - 1].pHdlList[WDXC_FTC_HDL_IDX];

  UINT8_TO_BSTREAM(p, WDX_FTC_OP_ABORT);
  UINT16_TO_BSTREAM(p, fileHdl);

  AttcWriteReq(connId, handle, WDX_FTC_ABORT_LEN, buf);
}

/*************************************************************************************************/
/*!
 *  \brief  Send a request to erase a file.
 *
 *  \param  connId          Connection ID.
 *  \param  fileHdl         Handle of file to erase on peer device
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxcFtcSendEraseFile(dmConnId_t connId, uint16_t fileHdl)
{
  uint8_t   buf[WDX_FTC_ERASE_LEN];
  uint8_t   *p = buf;
  uint16_t  handle;

  WSF_ASSERT(wdxcCb.conn[connId - 1].pHdlList != NULL)

  handle = wdxcCb.conn[connId - 1].pHdlList[WDXC_FTC_HDL_IDX];

  UINT8_TO_BSTREAM(p, WDX_FTC_OP_ERASE_REQ);
  UINT16_TO_BSTREAM(p, fileHdl);

  AttcWriteReq(connId, handle, WDX_FTC_ERASE_LEN, buf);
}

/*************************************************************************************************/
/*!
 *  \brief  Send a request to verify a file.
 *
 *  \param  connId          Connection ID.
 *  \param  fileHdl         Handle of file to verify on peer device
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxcFtcSendVerifyFile(dmConnId_t connId, uint16_t fileHdl)
{
  uint8_t   buf[WDX_FTC_VERIFY_LEN];
  uint8_t   *p = buf;
  uint16_t  handle;

  WSF_ASSERT(wdxcCb.conn[connId - 1].pHdlList != NULL)

  handle = wdxcCb.conn[connId - 1].pHdlList[WDXC_FTC_HDL_IDX];

  UINT8_TO_BSTREAM(p, WDX_FTC_OP_VERIFY_REQ);
  UINT16_TO_BSTREAM(p, fileHdl);

  AttcWriteReq(connId, handle, WDX_FTC_VERIFY_LEN, buf);
}

/*************************************************************************************************/
/*!
 *  \brief  Send a request to put a block of data into a file on the peer device
 *
 *  \param  connId          Connection ID.
 *  \param  fileHdl         Handle of file on peer device
 *  \param  offset          The offset from the beginning of the file in bytes
 *  \param  len             The number of bytes to put
 *  \param  fileSize        The size of the file in bytes
 *  \param  type            reserved
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxcFtcSendPutReq(dmConnId_t connId, uint16_t fileHdl, uint32_t offset,
                       uint32_t len, uint32_t fileSize, uint8_t type)
{
  uint8_t   buf[WDX_FTC_PUT_LEN];
  uint8_t   *p = buf;
  uint16_t  handle;

  WSF_ASSERT(wdxcCb.conn[connId - 1].pHdlList != NULL)

  handle = wdxcCb.conn[connId - 1].pHdlList[WDXC_FTC_HDL_IDX];

  UINT8_TO_BSTREAM(p, WDX_FTC_OP_PUT_REQ);
  UINT16_TO_BSTREAM(p, fileHdl);

  UINT32_TO_BSTREAM(p, offset);
  UINT32_TO_BSTREAM(p, len);
  UINT32_TO_BSTREAM(p, fileSize);
  UINT8_TO_BSTREAM(p, type);

  AttcWriteReq(connId, handle, WDX_FTC_PUT_LEN, buf);
}

/*************************************************************************************************/
/*!
 *  \brief  Send a request to perform a get a block of data from a file on the peer device
 *
 *  \param  connId          Connection ID.
 *  \param  fileHdl         Handle of file to verify on peer device
 *  \param  offset          The offset from the beginning of the file in bytes
 *  \param  len             The number of bytes to get
 *  \param  type            reserved
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxcFtcSendGetReq(dmConnId_t connId, uint16_t fileHdl, uint32_t offset, uint32_t len, uint8_t type)
{
  uint8_t   buf[WDX_FTC_GET_LEN];
  uint8_t   *p = buf;
  uint16_t  handle;

  WSF_ASSERT(wdxcCb.conn[connId - 1].pHdlList != NULL)

  handle = wdxcCb.conn[connId - 1].pHdlList[WDXC_FTC_HDL_IDX];

  UINT8_TO_BSTREAM(p, WDX_FTC_OP_GET_REQ);
  UINT16_TO_BSTREAM(p, fileHdl);

  UINT32_TO_BSTREAM(p, offset);
  UINT32_TO_BSTREAM(p, len);
  UINT8_TO_BSTREAM(p, type);

  AttcWriteReq(connId, handle, WDX_FTC_GET_LEN, buf);
}

/*************************************************************************************************/
/*!
 *  \brief  Send a data block to the peer device
 *
 *  \param  connId          Connection ID.
 *  \param  len             Size of pData in bytes
 *  \param  pData           Data to put to the file
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxcFtdSendBlock(dmConnId_t connId, uint32_t len, uint8_t *pData)
{
  uint16_t handle;

  WSF_ASSERT(wdxcCb.conn[connId - 1].pHdlList != NULL)

  handle = wdxcCb.conn[connId - 1].pHdlList[WDXC_FTD_HDL_IDX];

  AttcWriteReq(connId, handle, (uint16_t) len, pData);
}
