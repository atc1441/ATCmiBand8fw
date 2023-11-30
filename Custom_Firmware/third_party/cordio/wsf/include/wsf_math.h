/*************************************************************************************************/
/*!
 *  \file   wsf_math.h
 *
 *  \brief  Common math utilities.
 *
 *          $Date: 2016-12-28 16:12:14 -0600 (Wed, 28 Dec 2016) $
 *          $Revision: 10805 $
 *
 *  Copyright (c) 2013-2017 ARM Ltd., all rights reserved.
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
#ifndef WSF_MATH_H
#define WSF_MATH_H

#include "wsf_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief      Returns the minimum of two values. */
#define WSF_MIN(a,b)        ((a) < (b) ? (a) : (b))

/*! \brief      Returns the maximum of two values. */
#define WSF_MAX(a,b)        ((a) > (b) ? (a) : (b))

#ifdef __cplusplus
};
#endif

#endif /* WSF_MATH_H */
