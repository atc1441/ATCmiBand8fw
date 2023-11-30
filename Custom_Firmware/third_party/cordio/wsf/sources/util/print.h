/*************************************************************************************************/
/*!
 *  \file   print.h
 *
 *  \brief  Print functions.
 *
 *          $Date: 2016-12-28 16:12:14 -0600 (Wed, 28 Dec 2016) $
 *          $Revision: 10805 $
 *
 *  Copyright (c) 2015-2017 ARM Ltd., all rights reserved.
 *  ARM confidential and proprietary.
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

#ifndef PRINT_H
#define PRINT_H

#include <stdarg.h>

#include "wsf_types.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief  Print function attributes. */
#if defined(__GNUC__) || defined(__CC_ARM)
#define PRINT_ATTRIBUTE(a, b) __attribute__((format(printf, a, b)))
#else
#define PRINT_ATTRIBUTE(a, b)
#endif

/*************************************************************************************************/
/*!
 *  \fn     UtilVsnPrintf
 *
 *  \brief  Print a trace message.
 *
 *  \param  pStr      Storage for formatted string.
 *  \param  size      Maximum number of characters to store.
 *  \param  pFmt      Format string.
 *  \param  ap        Arguments.
 *
 *  \return Number of characters stored.
 */
/*************************************************************************************************/
uint32_t PrintVsn(char *pStr, uint32_t size, const char *pFmt, va_list ap) PRINT_ATTRIBUTE(3, 0);

#endif /* PRINT_H */
