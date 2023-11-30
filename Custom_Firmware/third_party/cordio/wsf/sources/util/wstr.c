/*************************************************************************************************/
/*!
 *  \file   wstr.c
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

#include <string.h>
#include "wsf_types.h"
#include "wstr.h"

/*************************************************************************************************/
/*!
 *  \fn     WstrnCpy
 *
 *  \brief  Copy a string and zero out space after the string length.
 *
 *  \return none.
 */
/*************************************************************************************************/
void WstrnCpy(char *pBuf, const char *pData, uint8_t n)
{
  uint8_t i;
  uint8_t zeroing = FALSE;

  for (i=0; i<n; i++)
  {
    if (!zeroing && pData[i] == '\0')
      zeroing = TRUE;

    if (zeroing)
      *pBuf++ = 0;
    else
      *pBuf++ = pData[i];
  }
}

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
void WStrReverseCpy(uint8_t *pBuf1, const uint8_t *pBuf2, uint16_t len)
{
  int16_t i;

  for (i=0; i<len; i++)
  {
    pBuf1[len-1-i] = pBuf2[i];
  }
}

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
void WStrReverse(uint8_t *pBuf, uint8_t len)
{
  uint8_t i, temp;

  for (i=0; i<len/2; i++)
  {
    temp = pBuf[len-i-1];
    pBuf[len-i-1] = pBuf[i];
    pBuf[i] = temp;
  }
}
