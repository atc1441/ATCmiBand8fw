/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Wireless Data Exchange profile client - Stream utility functions.
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
#include "wsf_types.h"
#include "util/wstr.h"
#include "wsf_trace.h"
#include "wsf_assert.h"
#include "wsf_efs.h"
#include "util/bstream.h"
#include "dm_api.h"
#include "att_api.h"
#include "app_api.h"
#include "wdx_defs.h"
#include "wdxc_api.h"
#include "wdxc_main.h"

/**************************************************************************************************
  External Variables
**************************************************************************************************/
extern wdxcCb_t wdxcCb;

/*************************************************************************************************/
/*!
 *  \brief  Send a request to start a stream of a given file handle on the given connection.
 *
 *  \param  connId    Connection ID.
 *  \param  fileHdl   Handle of the file.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxcStreamStart(dmConnId_t connId, uint16_t fileHdl)
{
  wdxcConnCb_t *pConnCb = &wdxcCb.conn[connId - 1];

  /* Verify WDXC is Idle */
  if (pConnCb->fileHdl == WSF_EFS_INVALID_HANDLE)
  {
    pConnCb->fileHdl = fileHdl;

    /* Send a get request with len set the MTU length (offset is ignored for streams) */
    WdxcFtcSendGetReq(connId, fileHdl, 0, AttGetMtu(connId), 0);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Stop the active stream.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxcStreamStop(dmConnId_t connId)
{
  wdxcConnCb_t *pConnCb = &wdxcCb.conn[connId - 1];

  /* Send an abort to stop the stream */
  WdxcFtcSendAbort(connId, pConnCb->fileHdl);

  /* Set file handle to invalid to indicate no file operation is in progress */
  pConnCb->fileHdl = WSF_EFS_INVALID_HANDLE;
}
