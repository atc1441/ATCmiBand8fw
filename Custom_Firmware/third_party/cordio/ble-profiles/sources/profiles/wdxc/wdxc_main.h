/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Wireless Data Exchange profile implementation.
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

#ifndef WDXC_MAIN_H
#define WDXC_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup WIRELESS_DATA_EXCHANGE_PROFILE
 *  \{ */

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! \brief WDXC Connection Control Block */
typedef struct
{
  uint16_t          *pHdlList;              /*!< \brief Attrtibute Handles */

  /* Operation in progress Control */
  uint16_t          fileHdl;                /*!< \brief File Handle */

  /* File Listing Control */
  wsfEfsFileInfo_t  *pFileList;             /*!< \brief File Listing storage */
  uint16_t          maxFiles;               /*!< \brief Size of pFileList in number of wsfEfsFileInfo_t objects */
  uint16_t          fileCount;              /*!< \brief Number of files on peer device */
  uint16_t          fDlPos;                 /*!< \brief Position in the download of file information */
} wdxcConnCb_t;

/*! \brief WDXC Control Block */
typedef struct
{
  wdxcConnCb_t      conn[DM_CONN_MAX];      /*!< \brief Connection control */
  WdxcFtdCallback_t *pFtdCallback;          /*!< \brief File Transfer Data Application Callback */
  WdxcFtcCallback_t *pFtcCallback;          /*!< \brief File Transfer Control Application Callback */
} wdxcCb_t;


/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

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
void WdxcFtcSendAbort(dmConnId_t connId, uint16_t fileHdl);

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
void WdxcFtcSendEraseFile(dmConnId_t connId, uint16_t fileHdl);

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
void WdxcFtcSendVerifyFile(dmConnId_t connId, uint16_t fileHdl);

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
                       uint32_t len, uint32_t fileSize, uint8_t type);

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
void WdxcFtcSendGetReq(dmConnId_t connId, uint16_t fileHdl, uint32_t offset, uint32_t len, uint8_t type);

/*! \} */    /* WIRELESS_DATA_EXCHANGE_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* WDXC_MAIN_H */
