/*************************************************************************************************/
/*!
 *  \file   wsf_buf.h
 *
 *  \brief  Buffer pool service.
 *
 *          $Date: 2017-03-04 13:45:34 -0600 (Sat, 04 Mar 2017) $
 *          $Revision: 11379 $
 *
 *  Copyright (c) 2009-2017 ARM Ltd., all rights reserved.
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
#ifndef WSF_BUF_H
#define WSF_BUF_H

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Length of the buffer statistics array */
#define WSF_BUF_STATS_MAX_LEN       128

/*! Failure Codes */
#define WSF_BUF_ALLOC_FAILED        0x01

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! Buffer pool descriptor structure */
typedef struct
{
  uint16_t   len;                  /*! length of buffers in pool */
  uint8_t    num;                  /*! number of buffers in pool */
} wsfBufPoolDesc_t;

/*! Pool statistics */
typedef struct
{
  uint16_t   bufSize;              /*!< Pool buffer size. */
  uint8_t    numBuf;               /*!< Total number of buffers. */
  uint8_t    numAlloc;             /*!< Number of outstanding allocations. */
  uint8_t    maxAlloc;             /*!< High allocation watermark. */
  uint16_t   maxReqLen;            /*!< Maximum requested buffer length. */
} WsfBufPoolStat_t;

/*! WSF buffer diagnostics - buffer allocation failure */
typedef struct
{
  uint8_t    taskId;               /*! Task handler ID where failure occured */
  uint16_t   len;                  /*! Length of buffer being allocated */
} wsfBufDiagAllocFail_t;

/*! WSF buffer diagnostics message */
typedef struct
{
  union
  {
    wsfBufDiagAllocFail_t alloc;   /*! Buffer allocation failure */
  } param;

  uint8_t type;                    /*! Type of error */
} WsfBufDiag_t;

/**************************************************************************************************
  Callback Function Datatypes
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \fn     wsfBufDiagCback_t
 *
 *  \brief  Callback providing WSF buffer diagnostic messages.
 *
 *  \param  pInfo     Diagnostics message
 *
 *  \return None.
 */
/*************************************************************************************************/
typedef void (*wsfBufDiagCback_t)(WsfBufDiag_t *pInfo);

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \fn     WsfBufInit
 *
 *  \brief  Initialize the buffer pool service.  This function should only be called once
 *          upon system initialization.
 *
 *  \param  bufMemLen Length in bytes of memory pointed to by pBufMem.
 *  \param  pBufMem   Memory in which to store the pools used by the buffer pool service.
 *  \param  numPools  Number of buffer pools.
 *  \param  pDesc     Array of buffer pool descriptors, one for each pool.
 *
 *  \return Amount of pBufMem used or 0 for failures.
 */
/*************************************************************************************************/
uint16_t WsfBufInit(uint16_t bufMemLen, uint8_t *pBufMem, uint8_t numPools, wsfBufPoolDesc_t *pDesc);

/*************************************************************************************************/
/*!
 *  \fn     WsfBufAlloc
 *
 *  \brief  Allocate a buffer.
 *
 *  \param  len     Length of buffer to allocate.
 *
 *  \return Pointer to allocated buffer or NULL if allocation fails.
 */
/*************************************************************************************************/
void *WsfBufAlloc(uint16_t len);

/*************************************************************************************************/
/*!
 *  \fn     WsfBufFree
 *
 *  \brief  Free a buffer.
 *
 *  \param  pBuf    Buffer to free.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfBufFree(void *pBuf);

/*************************************************************************************************/
/*!
 *  \fn     WsfBufGetAllocStats
 *
 *  \brief  Diagnostic function to get the buffer allocation statistics.
 *
 *  \return Buffer allocation statistics array.
 */
/*************************************************************************************************/
uint8_t *WsfBufGetAllocStats(void);

/*************************************************************************************************/
/*!
 *  \fn     WsfBufGetNumPool
 *
 *  \brief  Get number of pools.
 *
 *  \return Number of pools.
 */
/*************************************************************************************************/
uint8_t WsfBufGetNumPool(void);

/*************************************************************************************************/
/*!
 *  \fn     WsfBufGetPoolStats
 *
 *  \brief  Get statistics for each pool.
 *
 *  \param  pStat   Buffer to store the statistics.
 *  \param  numPool Number of pool elements.
 *
 *  \return Pool statistics.
 */
/*************************************************************************************************/
void WsfBufGetPoolStats(WsfBufPoolStat_t *pStat, uint8_t numPool);

/*************************************************************************************************/
/*!
 *  \fn     WsfBufDiagRegister
 *
 *  \brief  Called to register the buffer diagnostics callback function.
 *
 *  \param  pCallback   Pointer to the callback function.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfBufDiagRegister(wsfBufDiagCback_t callback);


#ifdef __cplusplus
};
#endif

#endif /* WSF_BUF_H */
