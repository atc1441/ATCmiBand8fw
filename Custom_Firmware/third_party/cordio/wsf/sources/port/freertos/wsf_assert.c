/*************************************************************************************************/
/*!
 *  \file   wsf_assert.c
 *
 *  \brief  Assert implementation.
 *
 *          $Date: 2014-04-17 16:13:59 -0700 (Thu, 17 Apr 2014) $
 *          $Revision: 1398 $
 *
 *  Copyright (c) 2009 Wicentric, Inc., all rights reserved.
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

#include "wsf_types.h"
#include "wsf_assert.h"
#include "wsf_os.h"

/*************************************************************************************************/
/*!
 *  \def    WsfAssert
 *
 *  \brief  Perform an assert action.
 *
 *  \param  pFile   Name of file originating assert.
 *  \param  line    Line number of assert statement.
 */
/*************************************************************************************************/
void WsfAssert(const char *pFile, uint16_t line)
{
  volatile uint8_t escape=0;

  /* spin forever if fatal error occurred */
  for(;;)
  {
    /*
     *  However, you can exit with a debugger by setting variable 'escape'.
     *  Handy to see where the assert happened if you cannot view the call stack.
    */
    if (escape)
    {
      break;
    }
  }
}
