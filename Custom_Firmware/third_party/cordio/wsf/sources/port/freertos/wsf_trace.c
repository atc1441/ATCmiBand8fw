/*************************************************************************************************/
/*!
 *  \file   wsf_trace.c
 *
 *  \brief  Trace message implementation.
 *
 *          $Date: 2014-08-06 15:13:15 -0700 (Wed, 06 Aug 2014) $
 *          $Revision: 1708 $
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
#include "wsf_trace.h"
#include "wsf_assert.h"
#include "wsf_cs.h"

#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>

#include "am_util_debug.h"
#include "am_util_stdio.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

#ifndef WSF_RING_BUF_SIZE
/*! \brief      Size of token ring buffer (multiple of 2^N). */
#define WSF_RING_BUF_SIZE               32
#endif

/*! \brief      Ring buffer flow control condition detected. */
#define WSF_TOKEN_FLAG_FLOW_CTRL        (1 << 28)

/**************************************************************************************************
  Data types
**************************************************************************************************/

#if WSF_TOKEN_ENABLED

/*! \brief      Trace control block. */
struct
{
  struct
  {
    uint32_t token;             /*!< Token. */
    uint32_t param;             /*!< Parameter. */
  } ringBuf[WSF_RING_BUF_SIZE]; /*!< Tokenized tracing ring buffer. */

  volatile uint32_t prodIdx;    /*!< Ring buffer producer index. */
  volatile uint32_t consIdx;    /*!< Ring buffer consumer index. */

  WsfTokenHandler_t pendCback;  /*!< Pending event handler. */

  bool_t ringBufEmpty;          /*!< Ring buffer state. */
} wsfTraceCb;

#endif

#if WSF_TRACE_ENABLED == TRUE

/*************************************************************************************************/
/*!
 *  \fn     WsfPacketTrace
 *
 *  \brief  Print raw HCI data as a trace message.
 *
 *  \param  ui8Type      HCI packet type byte
 *  \param  ui32Len      Length of the HCI packet
 *  \param  pui8Buf      Pointer to the buffer of HCI data
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfPacketTrace(uint8_t ui8Type, uint32_t ui32Len, uint8_t *pui8Buf)
{
  uint32_t i;

  am_util_debug_printf("%02X ", ui8Type);

  for(i = 0; i < ui32Len; i++)
  {
    if ((i % 8) == 0)
    {
      am_util_debug_printf("\n");
    }

    am_util_debug_printf("%02X ", *pui8Buf++);
  }

  am_util_debug_printf("\n\n");
}

/*************************************************************************************************/
/*!
 *  \fn     WsfTrace
 *
 *  \brief  Print a trace message.
 *
 *  \param  pStr      Message format string
 *  \param  ...       Additional aguments, printf-style
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfTrace(const char *pStr, ...)
{
  char pTraceMsg[AM_PRINTF_BUFSIZE];
  uint32_t ui32NumChars;
  va_list           args;

  va_start(args, pStr);
  am_util_stdio_vsprintf(pTraceMsg, pStr, args);
  //vprintf(pStr, args);
  va_end(args);
  ui32NumChars = am_util_debug_printf(pTraceMsg);
  if (!(ui32NumChars < AM_PRINTF_BUFSIZE))
    WsfAssert(__FILE__, (uint16_t) __LINE__);
  am_util_debug_printf("\n");
}

#elif WSF_TOKEN_ENABLED == TRUE

/*************************************************************************************************/
/*!
 *  \fn     WsfToken
 *
 *  \brief  Output tokenized message.
 *
 *  \param  tok       Token
 *  \param  var       Variable
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfToken(uint32_t tok, uint32_t var)
{
  static uint32_t flags = 0;

  WSF_CS_INIT(cs);
  WSF_CS_ENTER(cs);

  uint32_t prodIdx = (wsfTraceCb.prodIdx + 1) & (WSF_RING_BUF_SIZE - 1);

  if (prodIdx != wsfTraceCb.consIdx)
  {
    wsfTraceCb.ringBuf[wsfTraceCb.prodIdx].token = tok | flags;
    wsfTraceCb.ringBuf[wsfTraceCb.prodIdx].param = var;
    wsfTraceCb.prodIdx = prodIdx;
    flags = 0;
  }
  else
  {
    flags = WSF_TOKEN_FLAG_FLOW_CTRL;
  }

  WSF_CS_EXIT(cs);

  if (wsfTraceCb.pendCback && wsfTraceCb.ringBufEmpty)
  {
    wsfTraceCb.ringBufEmpty = FALSE;
    wsfTraceCb.pendCback();
  }
}

/*************************************************************************************************/
/*!
 *  \fn     wsfTokenService
 *
 *  \brief  Service the trace ring buffer.
 *
 *  \return TRUE if trace messages pending, FALSE otherwise.
 *
 *  This routine is called in the main loop for a "push" type trace systems.
 */
/*************************************************************************************************/
bool_t WsfTokenService(void)
{
  static uint8_t outBuf[sizeof(wsfTraceCb.ringBuf[0])];
  static uint8_t outBufIdx = sizeof(wsfTraceCb.ringBuf[0]);

  if (outBufIdx < sizeof(wsfTraceCb.ringBuf[0]))
  {
    outBufIdx += WsfTokenIOWrite(outBuf + outBufIdx, sizeof(wsfTraceCb.ringBuf[0]) - outBufIdx);

    if (outBufIdx < sizeof(wsfTraceCb.ringBuf[0]))
    {
      /* I/O device is flow controlled. */
      return TRUE;
    }
  }

  if (wsfTraceCb.consIdx != wsfTraceCb.prodIdx)
  {
    memcpy(&outBuf, &wsfTraceCb.ringBuf[wsfTraceCb.consIdx], sizeof(wsfTraceCb.ringBuf[0]));
    outBufIdx = 0;

    wsfTraceCb.consIdx = (wsfTraceCb.consIdx + 1) & (WSF_RING_BUF_SIZE - 1);

    return TRUE;
  }

  return FALSE;
}

#endif
