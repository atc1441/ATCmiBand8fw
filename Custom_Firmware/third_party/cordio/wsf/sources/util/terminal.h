/*************************************************************************************************/
/*!
 *  \file       terminal.h
 *
 *  \brief      Terminal handler.
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

#ifndef TERMINAL_H
#define TERMINAL_H

#include <stdarg.h>

#include "wsf_types.h"
#include "wsf_os.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

#define TERMINAL_MAX_ARGC           8u          /*!< Maximum number of arguments to any command. */
#define TERMINAL_MAX_COMMAND_LEN    100u        /*!< Maximum length of command line. */
#define TERMINAL_PRINTF_MAX_LEN     128u        /*!< Maximum length of any printed output. */
#define TERMINAL_STRING_PROMPT      "> "        /*!< Prompt string. */
#define TERMINAL_STRING_ERROR       "ERROR: "   /*!< Error prefix. */
#define TERMINAL_STRING_USAGE       "USAGE: "   /*!< Usage prefix. */
#define TERMINAL_STRING_NEW_LINE    "\r\n"      /*!< New line string. */

/*! \brief    Terminal command error codes. */
enum
{
  TERMINAL_ERROR_OK                 = 0,  /*!< Command completed. */
  TERMINAL_ERROR_BAD_ARGUMENTS      = 1,  /*!< ERROR: Invalid argument(s) */
  TERMINAL_ERROR_TOO_FEW_ARGUMENTS  = 2,  /*!< ERROR: Too few arguments */
  TERMINAL_ERROR_TOO_MANY_ARGUMENTS = 3,  /*!< ERROR: Too many arguments */
  TERMINAL_ERROR_EXEC               = 4   /*!< Command completed with execution error. */
};

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Handler for a terminal command.
 *
 *  \param  argc      The number of arguments passed to the command.
 *  \param  argv      The array of arguments; the 0th argument is the command.
 *
 *  \return Error code.
 */
/*************************************************************************************************/
typedef uint8_t (*terminalHandler_t)(uint32_t argc, char **argv);

/*************************************************************************************************/
/*!
 *  \brief  Handler for transmit.
 *
 *  \param  pBuf      Buffer to transmit.
 *  \param  len       Number of bytes to transmit.
 *
 *  \return None.
 */
/*************************************************************************************************/
typedef void (*terminalUartTx_t)(const uint8_t *pBuf, uint32_t len);

/*! \brief  Terminal command. */
typedef struct terminalCommand_tag
{
  struct terminalCommand_tag   *pNext;     /*!< Pointer to next command in list. */
  const char                   *pName;     /*!< Name of command. */
  const char                   *pHelpStr;  /*!< Help String for command. */
  terminalHandler_t            handler;    /*!< Handler for command. */
} terminalCommand_t;

/**************************************************************************************************
  Function Prototypes
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \fn     TerminalInit
 *
 *  \brief  Initialize terminal.
 *
 *  \param  handlerId   Handler ID for TerminalHandler().
 *
 *  \return None.
 */
/*************************************************************************************************/
void TerminalInit(wsfHandlerId_t handlerId);

/*************************************************************************************************/
/*!
 *  \fn     TerminalRegisterUartTxFunc
 *
 *  \brief  Register the UART Tx Function for the platform.
 *
 *  \return None.
 */
/*************************************************************************************************/
void TerminalRegisterUartTxFunc(terminalUartTx_t uartTxFunc);

/*************************************************************************************************/
/*!
 *  \fn     TerminalRegisterCommand
 *
 *  \brief  Register command with terminal.
 *
 *  \param  pCommand    Command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void TerminalRegisterCommand(terminalCommand_t *pCommand);

/*************************************************************************************************/
/*!
 *  \fn     TerminalHandler
 *
 *  \brief  Handler for terminal messages.
 *
 *  \param  event       WSF event mask.
 *  \param  pMsg        WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void TerminalHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);

/*************************************************************************************************/
/*!
 *  \fn     TerminalRx
 *
 *  \brief  Called by application when a data byte is received.
 *
 *  \param  dataByte    received byte
 *
 *  \return None.
 */
/*************************************************************************************************/
void TerminalRx(uint8_t dataByte);

/*************************************************************************************************/
/*!
 *  \fn     TerminalTxStr
 *
 *  \brief  Called by application to transmit string.
 *
 *  \param  pStr      String.
 *
 *  \return None.
 */
/*************************************************************************************************/
void TerminalTxStr(const char *pStr);

/*************************************************************************************************/
/*!
 *  \fn     TerminalTxChar
 *
 *  \brief  Called by application to transmit character.
 *
 *  \param  c         Character.
 *
 *  \return None.
 */
/*************************************************************************************************/
void TerminalTxChar(char c);

/*************************************************************************************************/
/*!
 *  \fn     TerminalTxPrint
 *
 *  \brief  Called by application to print formatted data.
 *
 *  \param  pStr      Message format string
 *  \param  ...       Additional arguments, printf-style
 *
 *  \return None.
 */
/*************************************************************************************************/
void TerminalTxPrint(const char *pStr, ...);

/*************************************************************************************************/
/*!
 *  \fn     TerminalTx
 *
 *  \brief  Application function to transmit data..
 *
 *  \param  pData     Data.
 *  \param  len       Length of data, in bytes.
 *
 *  \return None.
 */
/*************************************************************************************************/
void TerminalTx(const uint8_t *pData, uint16_t len);

#endif /* TERMINAL_H */
