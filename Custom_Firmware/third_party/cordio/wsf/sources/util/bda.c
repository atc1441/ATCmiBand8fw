/*************************************************************************************************/
/*!
 *  \file   bda.c
 *
 *  \brief  Bluetooth device address utilities.
 *
 *          $Date: 2016-12-28 16:12:14 -0600 (Wed, 28 Dec 2016) $
 *          $Revision: 10805 $
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

#include <string.h>
#include "wsf_types.h"
#include "bda.h"

/*************************************************************************************************/
/*!
 *  \fn     BdaCpy
 *
 *  \brief  Copy a BD address from source to destination.
 *
 *  \param  pDst    Pointer to destination.
 *  \param  pSrc    Pointer to source.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BdaCpy(uint8_t *pDst, const uint8_t *pSrc)
{
  memcpy(pDst, pSrc, BDA_ADDR_LEN);
}

/*************************************************************************************************/
/*!
 *  \fn     BdaCmp
 *
 *  \brief  Compare two BD addresses.
 *
 *  \param  pAddr1  First address.
 *  \param  pAddr2  Second address.
 *
 *  \return TRUE if addresses match, FALSE otherwise.
 */
/*************************************************************************************************/
bool_t BdaCmp(const uint8_t *pAddr1, const uint8_t *pAddr2)
{
  return (memcmp(pAddr1, pAddr2, BDA_ADDR_LEN) == 0);
}

/*************************************************************************************************/
/*!
 *  \fn     BdaClr
 *
 *  \brief  Set a BD address to all zeros.
 *
 *  \param  pDst    Pointer to destination.
 *
 *  \return pDst + BDA_ADDR_LEN
 */
/*************************************************************************************************/
uint8_t *BdaClr(uint8_t *pDst)
{
  memset(pDst, 0, BDA_ADDR_LEN);

  return (pDst + BDA_ADDR_LEN);
}

/*************************************************************************************************/
/*!
*  \fn     BdaIsZeros
*
*  \brief  Check if a BD address is all zeros.
*
*  \param  pAddr    Pointer to address.
*
*  \return TRUE if address is all zeros, FALSE otherwise.
*/
/*************************************************************************************************/
bool_t BdaIsZeros(const uint8_t *pAddr)
{
  uint8_t addrZeros[BDA_ADDR_LEN] = { 0 };

  return (memcmp(pAddr, addrZeros, BDA_ADDR_LEN) == 0);
}

/*************************************************************************************************/
/*!
 *  \fn     Bda2Str
 *
 *  \brief  Convert a BD address to a string.
 *
 *  \param  pAddr    Pointer to BD address.
 *
 *  \return Pointer to string.
 */
/*************************************************************************************************/
char *Bda2Str(const uint8_t *pAddr)
{
  static const char hex[] = "0123456789ABCDEF";
  static char       str[BDA_ADDR_STR_LEN + 1];
  char              *pStr = str;

  /* address is little endian so copy in reverse */
  pAddr += BDA_ADDR_LEN;

  while (pStr < &str[BDA_ADDR_STR_LEN])
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  /* null terminate string */
  *pStr = 0;

  return str;
}
