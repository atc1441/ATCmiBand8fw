/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief      Terminal handler.
 *
 *  Copyright (c) 2015-2019 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2019 Packetcraft, Inc.
 *  
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *  
 *      http://www.apache.org/licenses/LICENSE-2.0
 *  
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
/*************************************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "util/terminal.h"
#include "app_ui.h"
#include "util/print.h"

#include "wsf_types.h"
#include "wsf_assert.h"
#include "wsf_trace.h"
#include "wsf_bufio.h"
#include "util/bstream.h"

#include "dm_api.h"

/**************************************************************************************************
  Local Function Prototypes
**************************************************************************************************/

/*! \brief    Button Command Handler */
static uint8_t appTerminalCommandBtnHandler(uint32_t argc, char **argv);

/*! \brief    Security Pin Code Command Handler. */
static uint8_t appTerminalPinCodeHandler(uint32_t argc, char **argv);

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! \brief    Button command. */
static terminalCommand_t appTerminalButtonPress = { NULL, "btn", "btn <ID> <s|m|l|x>", appTerminalCommandBtnHandler };

/*! \brief    Security Pin Code commands. */
static terminalCommand_t appTerminalPinCode = { NULL, "pin", "pin <ConnID> <Pin Code>", appTerminalPinCodeHandler };

/*************************************************************************************************/
/*!
 *  \brief  Initialize terminal.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppTerminalInit(void)
{
  wsfHandlerId_t handlerId;

  /* Initialize Serial Communication. */
  // WsfBufIoUartRegister(TerminalRx);
  // TerminalRegisterUartTxFunc(WsfBufIoWrite);
  handlerId = WsfOsSetNextHandler(TerminalHandler);
  TerminalInit(handlerId);

  /* Register commands. */
  TerminalRegisterCommand(&appTerminalButtonPress);
  TerminalRegisterCommand(&appTerminalPinCode);
}

/*************************************************************************************************/
/*!
 *  \brief  Handler for a button press terminal command.
 *
 *  \param  argc      The number of arguments passed to the command.
 *  \param  argv      The array of arguments; the 0th argument is the command.
 *
 *  \return Error code.
 */
/*************************************************************************************************/
static uint8_t appTerminalCommandBtnHandler(uint32_t argc, char **argv)
{
  if (argc < 3)
  {
    return TERMINAL_ERROR_TOO_FEW_ARGUMENTS;
  }
  else if (argc == 3)
  {
    uint8_t btnIndx;
    uint8_t event;

    if (strcmp(argv[1], "1") == 0)
    {
      btnIndx = 1;
    }
    else if (strcmp(argv[1], "2") == 0)
    {
      btnIndx = 2;
    }
    else
    {
      return TERMINAL_ERROR_BAD_ARGUMENTS;
    }

    if (strcmp(argv[2], "d") == 0)
    {
      TerminalTxPrint("Button %s Press" TERMINAL_STRING_NEW_LINE, argv[1]);
      event = (btnIndx == 1? APP_UI_BTN_1_DOWN : APP_UI_BTN_2_DOWN);
    }
    else if (strcmp(argv[2], "s") == 0)
    {
      TerminalTxPrint("Short Button %s Press" TERMINAL_STRING_NEW_LINE, argv[1]);
      event = (btnIndx == 1? APP_UI_BTN_1_SHORT : APP_UI_BTN_2_SHORT);
    }
    else if (strcmp(argv[2], "m") == 0)
    {
      TerminalTxPrint("Medium Button %s Press" TERMINAL_STRING_NEW_LINE, argv[1]);
      event = (btnIndx == 1? APP_UI_BTN_1_MED : APP_UI_BTN_2_MED);
    }
    else if (strcmp(argv[2], "l") == 0)
    {
      TerminalTxPrint("Long Button %s Press" TERMINAL_STRING_NEW_LINE, argv[1]);
      event = (btnIndx == 1? APP_UI_BTN_1_LONG : APP_UI_BTN_2_LONG);
    }
    else if (strcmp(argv[2], "x") == 0)
    {
      TerminalTxPrint("Medium Button %s Press" TERMINAL_STRING_NEW_LINE, argv[1]);
      event = (btnIndx == 1? APP_UI_BTN_1_EX_LONG : APP_UI_BTN_2_EX_LONG);
    }
    else
    {
      return TERMINAL_ERROR_BAD_ARGUMENTS;
    }

    AppUiBtnTest(event);
  }
  else
  {
    return TERMINAL_ERROR_TOO_MANY_ARGUMENTS;
  }

  return TERMINAL_ERROR_OK;
}

/*************************************************************************************************/
/*!
 *  \brief  Handler for a pin code terminal command.
 *
 *  \param  argc      The number of arguments passed to the command.
 *  \param  argv      The array of arguments; the 0th argument is the command.
 *
 *  \return Error code.
 */
/*************************************************************************************************/
static uint8_t appTerminalPinCodeHandler(uint32_t argc, char **argv)
{
  if (argc < 2)
  {
    return TERMINAL_ERROR_TOO_FEW_ARGUMENTS;
  }
  else if (argc == 3)
  {
    uint32_t passkey;
    uint8_t  buf[SMP_PIN_LEN];
    uint8_t connId;

    passkey = atoi(argv[2]);
    connId = atoi(argv[1]);

    if (connId < 1 || connId > DM_CONN_MAX)
    {
      TerminalTxPrint("ConnID must be in the range [1 .. %d]\n", DM_CONN_MAX);
      return TERMINAL_ERROR_BAD_ARGUMENTS;
    }

    passkey %= 1000000;

    /* convert to byte buffer */
    buf[0] = UINT32_TO_BYTE0(passkey);
    buf[1] = UINT32_TO_BYTE1(passkey);
    buf[2] = UINT32_TO_BYTE2(passkey);

    /* send authentication response to DM */
    DmSecAuthRsp((dmConnId_t) connId, SMP_PIN_LEN, buf);
  }
  else
  {
    return TERMINAL_ERROR_TOO_MANY_ARGUMENTS;
  }

  return TERMINAL_ERROR_OK;
}
