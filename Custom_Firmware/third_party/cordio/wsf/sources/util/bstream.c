/*************************************************************************************************/
/*!
 *  \file   bstream.h
 *
 *  \brief  Byte stream to integer conversion functions.
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

#include "wsf_types.h"
#include "bstream.h"

/*************************************************************************************************/
/*!
 *  \brief  Convert bstream to uint64_t.
 *
 *  \param  p       Bstream pointer.
 *
 *  \return Resulting uint64_t number.
 */
/*************************************************************************************************/
uint64_t BstreamToUint64(const uint8_t *p)
{
  return ((uint64_t)p[0] <<  0) |
         ((uint64_t)p[1] <<  8) |
         ((uint64_t)p[2] << 16) |
         ((uint64_t)p[3] << 24) |
         ((uint64_t)p[4] << 32) |
         ((uint64_t)p[5] << 40) |
         ((uint64_t)p[6] << 48) |
         ((uint64_t)p[7] << 56);
}

/*************************************************************************************************/
/*!
 *  \brief  Convert uint64_t to bstream.
 *
 *  \param  p       Bstream pointer.
 *  \param  n       uint64_t number.
 *
 *  \return None.
 */
/*************************************************************************************************/
void Uint64ToBstream(uint8_t *p, uint64_t n)
{
  p[0] = (uint8_t)(n >>  0);
  p[1] = (uint8_t)(n >>  8);
  p[2] = (uint8_t)(n >> 16);
  p[3] = (uint8_t)(n >> 24);
  p[4] = (uint8_t)(n >> 32);
  p[5] = (uint8_t)(n >> 40);
  p[6] = (uint8_t)(n >> 48);
  p[7] = (uint8_t)(n >> 56);
}

/*************************************************************************************************/
/*!
 *  \brief  Convert bstream to BDA64.
 *
 *  \param  p       Bstream pointer.
 *
 *  \return Resulting BDA64 number.
 */
/*************************************************************************************************/
uint64_t BstreamToBda64(const uint8_t *p)
{
  return (uint64_t)p[0] <<  0 |
         (uint64_t)p[1] <<  8 |
         (uint64_t)p[2] << 16 |
         (uint64_t)p[3] << 24 |
         (uint64_t)p[4] << 32 |
         (uint64_t)p[5] << 40;
}

/*************************************************************************************************/
/*!
 *  \brief  Convert BDA64 to bstream.
 *
 *  \param  p       Bstream pointer.
 *  \param  bda     uint64_t BDA.
 *
 *  \return None.
 */
/*************************************************************************************************/
void Bda64ToBstream(uint8_t *p, uint64_t bda)
{
  p[0] = (uint8_t)(bda >>  0);
  p[1] = (uint8_t)(bda >>  8);
  p[2] = (uint8_t)(bda >> 16);
  p[3] = (uint8_t)(bda >> 24);
  p[4] = (uint8_t)(bda >> 32);
  p[5] = (uint8_t)(bda >> 40);
}
