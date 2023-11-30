/*************************************************************************************************/
/*!
 *  \file   wsf_math.c
 *
 *  \brief  Common math utilities generic implementation file.
 *
 *          $Date: 2014-09-16 13:19:18 -0700 (Tue, 16 Sep 2014) $
 *          $Revision: 1816 $
 *
 *  Copyright (c) 2013 Wicentric, Inc., all rights reserved.
 *  Wicentric confidential and proprietary.
 *
 *  IMPORTANT.  Your use of this file is governed by a Software License Agreement
 *  ("Agreement") that must be accepted in order to download or otherwise receive a
 *  copy of this file.  You may not use or copy this file for any purpose other than
 *  as described in the Agreement.  If you do not agree to all of the terms of the
 *  Agreement do not use this file and delete all copies in your possession or control;
 *  if you do not have a copy of the Agreement, you must contact Wicentric, Inc. prior
 *  to any use, copying or further distribution of this software.
 */
/*************************************************************************************************/

#include "wsf_math.h"
#include <string.h>

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

static uint32_t wsfRngW = 88675123;
static uint32_t wsfRngX = 123456789;
static uint32_t wsfRngY = 362436069;
static uint32_t wsfRngZ = 521288629;

/*************************************************************************************************/
/*!
 *  \fn     WsfMathInit
 *
 *  \brief  Initialize math routines.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfMathInit(void)
{
  /* Seed PRNG. */
  wsfRngW = 88675123;
  wsfRngX = 123456789;
  wsfRngY = 362436069;
  wsfRngZ = 521288629;
}

/*************************************************************************************************/
/*!
 *  \fn     WsfRandNum
 *
 *  \brief  Generate random number.
 *
 *  \return 32-bit random number.
 *
 *  This software implementation uses a xorshift random number generator.
 *      George Marsaglia (2003), "Xorshift RNGs", Journal of Statistical Software
 *
 *  \note   This routine is not a cryptographic grade random number generator.
 */
/*************************************************************************************************/
uint32_t WsfRandNum(void)
{
  uint32_t t;

  t = wsfRngX ^ (wsfRngX << 11);
  wsfRngX = wsfRngY;
  wsfRngY = wsfRngZ;
  wsfRngZ = wsfRngW;
  wsfRngW = wsfRngW ^ (wsfRngW >> 19) ^ (t ^ (t >> 8));
  return wsfRngW;
}

/*************************************************************************************************/
/*!
 *  \fn     WsfAesEcb
 *
 *  \brief  Calculate AES ECB.
 *
 *  \param  pKey        Encryption key.
 *  \param  pOut        Output data.
 *  \param  pIn         Input data.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfAesEcb(const uint8_t *pKey, uint8_t *pOut, const uint8_t *pIn)
{
  const unsigned int KEY_LEN = 16;
  memcpy(pOut, pIn, KEY_LEN);
}
