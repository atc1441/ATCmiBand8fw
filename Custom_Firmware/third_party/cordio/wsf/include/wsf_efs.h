/*************************************************************************************************/
/*!
 *  \file   wsf_efs.h
 *
 *  \brief  Embedded File System service.
 *
 *          $Date: 2017-03-09 12:18:38 -0600 (Thu, 09 Mar 2017) $
 *          $Revision: 11460 $
 *
 *  Copyright (c) 2014-2017 ARM Ltd., all rights reserved.
 *  ARM Ltd. confidential and proprietary.
 *
 *  IMPORTANT.  Your use of this file is governed by a Software License Agreement
 *  ("Agreement") that must be accepted in order to download or otherwise receive a
 *  copy of this file.  You may not use or copy this file for any purpose other than
 *  as described in the Agreement.  If you do not agree to all of the terms of the
 *  Agreement do not use this file and delete all copies in your possession or control;
 *  if you do not have a copy of the Agreement, you must contact ARM Ltd. prior
 *  to any use, copying or further distribution of this software.
 */
/*************************************************************************************************/

#ifndef WSF_EFS_H
#define WSF_EFS_H

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/

/* Max Number of Files and Media */
#ifndef WSF_EFS_MAX_FILES
#define WSF_EFS_MAX_FILES                   6
#endif

/* Max Number of Media */
#ifndef WSF_EFS_MAX_MEDIA
#define WSF_EFS_MAX_MEDIA                   4
#endif

/* Status codes */
#define WSF_EFS_SUCCESS                     0             /*! Success */
#define WSF_EFS_FAILURE                     1             /*! Failure */
#define WSF_EFS_GET_FAILED                  0xFFFF        /*! Get operation failure */
#define WSF_EFS_PUT_FAILED                  0xFFFF        /*! PUT operation failure */

/* Invalid parameter Identifiers */
#define WSF_EFS_INVALID_HANDLE              0xFFFF        /*! Invalid Handle */
#define WSF_EFS_INVALID_OFFSET              0xFFFFFFFF    /*! Invalid Offset */
#define WSF_EFS_INVALID_SIZE                0xFFFFFFFF    /*! Invalid Size */
#define WSF_EFS_INVALID_MEDIA               0xFF          /*! Invalid Media */

/* File Types */
#define WSF_EFS_FILE_TYPE_BULK              0             /*! Bulk File Type */
#define WSF_EFS_FILE_TYPE_STREAM            1             /*! Stream File Type */

/* Offset to WsfEfsAddFile indicating any file offset can be used */
#define WSF_EFS_FILE_OFFSET_ANY             0xFFFFFFFF

/* File Permissions */
#define WSF_EFS_REMOTE_PERMISSIONS_MASK     0xFF          /*! Remote Permissions */
#define WSF_EFS_REMOTE_GET_PERMITTED        0x01          /*! Remote Get Permitted */
#define WSF_EFS_REMOTE_PUT_PERMITTED        0x02          /*! Remote Put Permitted */
#define WSF_EFS_REMOTE_ERASE_PERMITTED      0x04          /*! Remote Erase Permitted */
#define WSF_EFS_REMOTE_VERIFY_PERMITTED     0x08          /*! Remote Verify Permitted */
#define WSF_EFS_LOCAL_GET_PERMITTED         0x0100        /*! Local Get Permitted */
#define WSF_EFS_LOCAL_PUT_PERMITTED         0x0200        /*! Local Put Permitted */
#define WSF_EFS_LOCAL_ERASE_PERMITTED       0x0400        /*! Local Erase Permitted */
#define WSF_EFS_REMOTE_VISIBLE              0x0800        /*! File Visible via Remote WDXS */

/* File name length in bytes */
#define WSF_EFS_NAME_LEN                    16

/* File version length in bytes */
#define WSF_EFS_VERSION_LEN                 16

/* Standard Media Specific Command Identifiers */
#define WSF_EFS_WDXS_PUT_COMPLETE_CMD       0x00          /*! Put Complete */
#define WSF_EFS_VALIDATE_CMD                0x01          /*! Validate Req for the file */

/* Media Specific Command Identifiers reserved for applications begin at 0x80 */
#define WSF_EFS_USER_CMD                    0x80

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/* File handle data type */
typedef uint16_t wsfEfsHandle_t;

/* File attributes data type */
typedef struct
{
  char     name[WSF_EFS_NAME_LEN];
  char     version[WSF_EFS_VERSION_LEN];
  uint16_t permissions;
  uint8_t  type;
} wsfEsfAttributes_t;

/* File control block data type */
typedef struct
{
  uint32_t           maxSize;
  uint32_t           address;
  uint8_t            media;
  uint32_t           size;
  wsfEsfAttributes_t attributes;
} wsfEfsControl_t;

/* File Listing Information */
typedef struct
{
  wsfEfsHandle_t     handle;
  uint32_t           size;
  wsfEsfAttributes_t attributes;
} wsfEfsFileInfo_t;

/*************************************************************************************************/
/*!
 *  \fn     wsfMediaInitFunc_t
 *
 *  \brief  Media Init function, called when media is registered.
 *
 *  \param  none.
 *
 *  \return Status of the operation.
 */
/*************************************************************************************************/
typedef uint8_t wsfMediaInitFunc_t(void);

/*************************************************************************************************/
/*!
 *  \fn     wsfMediaEraseFunc_t
 *
 *  \brief  Media Erase function.
 *
 *  \param  address  Address in media to start erasing.
 *  \param  size     Number of bytes to erase.
 *
 *  \return Status of the operation.
 */
/*************************************************************************************************/
typedef uint8_t wsfMediaEraseFunc_t(uint8_t *pAddress, uint32_t size);

/*************************************************************************************************/
/*!
 *  \fn     wsfMediaReadFunc_t
 *
 *  \brief  Media Read function.
 *
 *  \param  pBuf     Buffer to hold data.
 *  \param  address  Address in media to read from.
 *  \param  size     Size of pBuf in bytes.
 *
 *  \return Status of the operation.
 */
/*************************************************************************************************/
typedef uint8_t wsfMediaReadFunc_t(uint8_t *pBuf, uint8_t *pAddress, uint32_t size);

/*************************************************************************************************/
/*!
 *  \fn     wsfMediaWriteFunc_t
 *
 *  \brief  Media Write function.
 *
 *  \param  pBuf     Buffer with data to be written.
 *  \param  address  Address in media to write to.
 *  \param  size     Size of pBuf in bytes.
 *
 *  \return Status of the operation.
 */
/*************************************************************************************************/
typedef uint8_t wsfMediaWriteFunc_t(const uint8_t *pBuf, uint8_t *pAddress, uint32_t size);

/*************************************************************************************************/
/*!
 *  \fn     wsfMediaHandleCmdFunc_t
 *
 *  \brief  Media Specific Command handler.
 *
 *  \param  cmd    Identifier of the media specific command.
 *  \param  param  Optional Parameter to the command.
 *
 *  \return Status of the operation.
 */
/*************************************************************************************************/
typedef uint8_t wsfMediaHandleCmdFunc_t(uint8_t cmd, uint32_t param);

/* Media Control data type */
typedef struct
{
  uint32_t                startAddress;
  uint32_t                endAddress;
  uint32_t                pageSize;
  wsfMediaInitFunc_t      *init;
  wsfMediaEraseFunc_t     *erase;
  wsfMediaReadFunc_t      *read;
  wsfMediaWriteFunc_t     *write;
  wsfMediaHandleCmdFunc_t *handleCmd;
} wsfEfsMedia_t;

/* Pointer to Media Control data type */
typedef const wsfEfsMedia_t *pWsfEfsMedia_t;

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \fn     WsfEfsInit
 *
 *  \brief  Initialise the embedded file system.
 *
 *  \return none.
 */
/*************************************************************************************************/
void WsfEfsInit(void);

/*************************************************************************************************/
/*!
 *  \fn     WsfEfsAddFile
 *
 *  \brief  Create a file in the embedded file system.
 *
 *  \param  maxSize   Max length in bytes of of the file.
 *  \param  media     Identifier of the media where the file is stored.
 *  \param  pAttr     Attributes of the file
 *  \param  offset    Offset address of the file in the memory space.
 *
 *  \return File Handle, or WSF_EFS_INVALID_HANDLE.
 */
/*************************************************************************************************/
wsfEfsHandle_t WsfEfsAddFile(uint32_t maxSize, uint8_t media, wsfEsfAttributes_t *pAttr, uint32_t offset);

/*************************************************************************************************/
/*!
 *  \fn     WsfEfsRemoveFile
 *
 *  \brief  Deletes a file in the embedded file system.
 *
 *  \param  handle    Handle identifying the file.
 *
 *  \return WSF_EFS_SUCCESS or WSF_EFS_FAILURE.
 */
/*************************************************************************************************/
uint8_t WsfEfsRemoveFile(wsfEfsHandle_t handle);

/*************************************************************************************************/
/*!
 *  \fn     WsfEfsErase
 *
 *  \brief  Clears the contents of a file without deleting the file.
 *
 *  \param  handle    Handle identifying the file.
 *
 *  \return WSF_EFS_SUCCESS or WSF_EFS_FAILURE.
 */
/*************************************************************************************************/
uint8_t WsfEfsErase(wsfEfsHandle_t handle);

/*************************************************************************************************/
/*!
 *  \fn     WsfEfsGetAttributes
 *
 *  \brief  Gets the attributes of a file.
 *
 *  \param  handle    Handle identifying the file.
 *  \param  pAttr     Pointer to memory to store the attributes.
 *
 *  \return WSF_EFS_SUCCESS or WSF_EFS_FAILURE.
 */
/*************************************************************************************************/
uint8_t WsfEfsGetAttributes(wsfEfsHandle_t handle, wsfEsfAttributes_t *pAttr);

/*************************************************************************************************/
/*!
 *  \fn     WsfEfsSetAttributes
 *
 *  \brief  Updates the attributes of a file.
 *
 *  \param  handle    Handle identifying the file.
 *  \param  pAttr     Pointer to memory to with the updated attributes.
 *
 *  \return WSF_EFS_SUCCESS or WSF_EFS_FAILURE.
 */
/*************************************************************************************************/
uint8_t WsfEfsSetAttributes(wsfEfsHandle_t handle, wsfEsfAttributes_t *pInfo);

/*************************************************************************************************/
/*!
 *  \fn     WsfEfsGet
 *
 *  \brief  Copies data from a file.
 *
 *  \param  handle    Handle identifying the file.
 *  \param  offset    Offset into the file to begin copying from.
 *  \param  pBuffer   Location to copy the data to.
 *  \param  len       Number of bytes to copy into pBuffer.
 *
 *  \return The number of bytes read from the file
 */
/*************************************************************************************************/
uint16_t WsfEfsGet(wsfEfsHandle_t handle, uint32_t offset, uint8_t *pBuffer, uint16_t len);

/*************************************************************************************************/
/*!
 *  \fn     WsfEfsPut
 *
 *  \brief  Writes data to a file.
 *
 *  \param  handle    Handle identifying the file.
 *  \param  offset    Offset into the file to begin writing to.
 *  \param  pBuffer   Data to write to the file.
 *  \param  len       Number of bytes to write to the file.
 *
 *  \return The number of bytes written to the file
 */
/*************************************************************************************************/
uint16_t WsfEfsPut(wsfEfsHandle_t handle, uint32_t offset, const uint8_t *pBuffer, uint16_t len);

/*************************************************************************************************/
/*!
 *  \fn     WsfEfsRegisterMedia
 *
 *  \brief  Registers a File Storage Medium with the Embedded File System.
 *
 *  \param  pMedia    Pointer to the media control structure.
 *  \param  mediaID   User specified identifier of the media.
 *
 *  \return WSF_EFS_SUCCESS or WSF_EFS_FAILURE.
 */
/*************************************************************************************************/
uint8_t WsfEfsRegisterMedia(const wsfEfsMedia_t *pMediaCtrl, uint8_t mediaID);

/*************************************************************************************************/
/*!
 *  \fn     wsfEfsGetFileByHandle
 *
 *  \brief  Returns the file control block for the given handle.
 *
 *  \param  handle   Handle of the file
 *
 *  \return File control block, or NULL.
 */
/*************************************************************************************************/
wsfEfsControl_t *WsfEfsGetFileByHandle(wsfEfsHandle_t handle);

/*************************************************************************************************/
/*!
 *  \fn     WsfEfsGetFileName
 *
 *  \brief  Get the name of a file.
 *
 *  \param  handle    File Handle.
 *
 *  \return Filename string of a file.
 */
/*************************************************************************************************/
char *WsfEfsGetFileName(wsfEfsHandle_t handle);

/*************************************************************************************************/
/*!
 *  \fn     WsfEfsGetFileVersion
 *
 *  \brief  Get the version of a file.
 *
 *  \param  handle    File Handle.
 *
 *  \return Version string of a file.
 */
/*************************************************************************************************/
char *WsfEfsGetFileVersion(wsfEfsHandle_t handle);

/*************************************************************************************************/
/*!
 *  \fn     WsfEfsGetFileSize
 *
 *  \brief  Get the size of a file.
 *
 *  \param  handle    File Handle.
 *
 *  \return Size of the file.
 */
/*************************************************************************************************/
uint32_t WsfEfsGetFileSize(wsfEfsHandle_t handle);

/*************************************************************************************************/
/*!
 *  \fn     WsfEfsGetFileMaxSize
 *
 *  \brief  Get the number of bytes of memory reserved for use by a file.
 *
 *  \param  handle    File Handle.
 *
 *  \return Max size of the file.
 */
/*************************************************************************************************/
uint32_t WsfEfsGetFileMaxSize(wsfEfsHandle_t handle);

/*************************************************************************************************/
/*!
 *  \fn     WsfEfsGetFileType
 *
 *  \brief  Get the type of a file.
 *
 *  \param  handle    File Handle.
 *
 *  \return Type of file (bulk or stream).
 */
/*************************************************************************************************/
uint8_t WsfEfsGetFileType(wsfEfsHandle_t handle);

/*************************************************************************************************/
/*!
 *  \fn     WsfEfsGetFilePermissions
 *
 *  \brief  Get the permissions of a file.
 *
 *  \param  handle    File Handle.
 *
 *  \return Permissions of the file.
 */
/*************************************************************************************************/
uint16_t WsfEfsGetFilePermissions(wsfEfsHandle_t handle);

/*************************************************************************************************/
/*!
 *  \fn     WsfEfsMediaSpecificCommand
 *
 *  \brief  Execute a media specific command on a file.
 *
 *  \param  handle    File Handle.
 *  \param  cmd       Media specific command identifier.
 *  \param  param     Command specific parameter.
 *
 *  \return Status of the operation.
 */
/*************************************************************************************************/
uint8_t WsfEfsMediaSpecificCommand(wsfEfsHandle_t handle, uint8_t cmd, uint32_t param);

#ifdef __cplusplus
}
#endif

#endif /* WSF_EFS_H */
