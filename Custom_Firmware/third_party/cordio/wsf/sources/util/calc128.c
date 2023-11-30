/*************************************************************************************************/
/*!
 *  \file   calc128.c
 *
 *  \brief  128-bit integer utilities.
 *
 *          $Date: 2016-12-28 16:12:14 -0600 (Wed, 28 Dec 2016) $
 *          $Revision: 10805 $
 *
 *  Copyright (c) 2010-2017 ARM Ltd., all rights reserved.
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
#include "calc128.h"

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/* 128-bit zero value */
const uint8_t calc128Zeros[CALC128_LEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \fn     Calc128Cpy
 *        
 *  \brief  Copy a 128-bit integer from source to destination.
 *
 *  \param  pDst    Pointer to destination.
 *  \param  pSrc    Pointer to source.
 *
 *  \return None.
 */
/*************************************************************************************************/
void Calc128Cpy(uint8_t *pDst, uint8_t *pSrc)
{
  memcpy(pDst, pSrc, CALC128_LEN);
}

/*************************************************************************************************/
/*!
 *  \fn     Calc128Cpy64
 *        
 *  \brief  Copy a 64-bit integer from source to destination.
 *
 *  \param  pDst    Pointer to destination.
 *  \param  pSrc    Pointer to source.
 *
 *  \return None.
 */
/*************************************************************************************************/
void Calc128Cpy64(uint8_t *pDst, uint8_t *pSrc)
{
  memcpy(pDst, pSrc, CALC128_LEN/2);
}

/*************************************************************************************************/
/*!
 *  \fn     Calc128Xor
 *        
 *  \brief  Exclusive-or two 128-bit integers and return the result in pDst.
 *
 *  \param  pDst    Pointer to destination.
 *  \param  pSrc    Pointer to source.
 *
 *  \return None.
 */
/*************************************************************************************************/
void Calc128Xor(uint8_t *pDst, uint8_t *pSrc)
{
  uint8_t i;
  
  for (i = CALC128_LEN; i > 0; i--)
  {
    *pDst++ ^= *pSrc++;
  }
}
