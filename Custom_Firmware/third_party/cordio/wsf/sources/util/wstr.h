/*************************************************************************************************/
/*!
 *  \file   wstr.h
 *
 *  \brief  String manipulation functions.
 *
 *          $Date: 2017-02-14 15:31:43 -0600 (Tue, 14 Feb 2017) $
 *          $Revision: 11184 $
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

#ifndef WSTR_H
#define WSTR_H

#ifdef __cplusplus
extern "C" {
#endif

/*************************************************************************************************/
/*!
 *  \fn     WstrnCpy
 *
 *  \brief  Copies a string up to a given length.
 *
 *  \param  pBuf    Pointer to buffer to copy to.
 *  \param  pData   Pointer to the string to copy.
 *  \param  n       Size of pBuf in bytes.
 *
 *  \return none.
 */
/*************************************************************************************************/
void WstrnCpy(char *pBuf, const char *pData, uint8_t n);

/*************************************************************************************************/
/*!
 *  \fn     WStrReverseCpy
 *
 *  \brief  Byte by byte reverse and copy a buffer.
 *
 *  \param  pBuf1   Buffer to hold reversed copy.
 *  \param  pBuf2   Buffer to copy.
 *  \param  len     Size of pBuf1 and pBuf2 in bytes.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WStrReverseCpy(uint8_t *pBuf1, const uint8_t *pBuf2, uint16_t len);

/*************************************************************************************************/
/*!
 *  \fn     WStrReverse
 *
 *  \brief  Byte by byte reverse a buffer.
 *
 *  \param  pBuf   Buffer to reverse.
 *  \param  len    size of pBuf in bytes.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WStrReverse(uint8_t *pBuf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* WSTR_H */
