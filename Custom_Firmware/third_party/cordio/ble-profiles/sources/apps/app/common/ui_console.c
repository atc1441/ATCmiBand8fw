/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  User Interface - Console
 *
 *  Copyright (c) 2017-2019 Arm Ltd. All Rights Reserved.
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

#include <string.h>
#include "wsf_types.h"
#include "wsf_assert.h"
#include "util/bda.h"
#include "app_api.h"
#include "app_main.h"
#include "app_db.h"
#include "app_cfg.h"
#include "ui_api.h"
#include "wsf_trace.h"

/**************************************************************************************************
  Function Prototypes
**************************************************************************************************/

/*! Action Function Prototypes */
static void uiConsoleDispSplash(const UiSplashScreen_t *pSplash);
static void uiConsoleDispMenu(const UiMenu_t *pMenu);
static void uiConsoleDispDialog(const UiDialog_t *pDialog);
static void uiConsoleProcessKey(uint8_t input);

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/* Console Control Block */
struct
{
  int8_t            alphaNumOffset;
} uiConsoleCb;

/* Console display action functions */
UiActionTbl_t uiConsoleActionTbl =
{
  uiConsoleDispSplash,
  uiConsoleDispMenu,
  uiConsoleDispDialog,
  uiConsoleProcessKey
};

/*************************************************************************************************/
/*!
 *  \brief  Process a key press from the user
 *
 *  \param  key     User keypress.
 *
 *  \return None
 */
/*************************************************************************************************/
static void uiConsoleProcessKey(uint8_t key)
{
  UiDialog_t *pDialog;

  switch(UiCb.activeScreenType)
  {
  case UI_SCREEN_SPLASH:
    if ((key >= '0' && key <= '9') || (key == '\r'))
    {
      /* Jump to main menu on any key */
      UiLoadMenu(UiCb.pMainMenu);
    }
    break;

  case UI_SCREEN_MENU:
    if (key >= '0' && key <= '9')
    {
      UiSelection(key - '0');
    }
    break;

  case UI_SCREEN_DIALOG:
    pDialog = (UiDialog_t*) UiCb.pActiveScreen;

    if (pDialog->type == UI_DLG_TYPE_INPUT_SELECT)
    {
      if (key > '0' && key <= '9')
      {
        /* User pressed a number from 1 - 9 for selection */
        UiSelection(key - '0');
      }
      else if (key == '\r')
      {
        /* User pressed enter on pause screen */
        UiSelection(0);
      }
    }
    else
    {
      if (uiConsoleCb.alphaNumOffset < pDialog->entryMaxLen)
      {
        /* User in process of entering alpha numeric input */
        pDialog->pEntry[uiConsoleCb.alphaNumOffset++] = key;
      }
      else
      {
        pDialog->pEntry[uiConsoleCb.alphaNumOffset] = '\0';
        uiConsoleCb.alphaNumOffset = 0;

        /* Notify callback of selection */
        UiSelection(0);

        /* Exit to parent */
        UiLoadMenu(pDialog->base.pParentMenu);
      }
    }
    break;

  default:
    break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Display a splash screen on a Console
 *
 *  \param  pSplash   Pointer to the splash screen object to display.
 *
 *  \return None
 */
/*************************************************************************************************/
static void uiConsoleDispSplash(const UiSplashScreen_t *pSplash)
{
  /* Print the splash widget identifier */
  UiConsolePrintLn("{Splash}");

  /* Print splash screen */
  UiConsolePrint(pSplash->pAppName);
  UiConsolePrint(", ");
  UiConsolePrintLn(pSplash->pAppVer);
  UiConsolePrintLn(pSplash->pCopyright);

  UiConsoleFlush();
}

/*************************************************************************************************/
/*!
 *  \brief  Display a menu on a Console
 *
 *  \param  pMenu     Pointer to the menu object to display.
 *
 *  \return None
 */
/*************************************************************************************************/
static void uiConsoleDispMenu(const UiMenu_t *pMenu)
{
  int8_t i;
  char ch[2];

  /* Print the menu widget identifier */
  UiConsolePrint("\r\n");
  UiConsolePrintLn("{Menu}");

  /* Print the title to the Console */
  UiConsolePrintLn(pMenu->pTitle);

  /* Print the menu items */
  for (i = 0; i < pMenu->numItems; i++)
  {
    UiConsolePrint("  ");
    ch[0] = '1' + i;
    ch[1] = '\0';
    UiConsolePrint(ch);
    UiConsolePrint(". ");
    UiConsolePrintLn(pMenu->pItems[i]);
  }

  UiConsolePrint("\r\n");
  UiConsolePrint("Choice? ");

  UiConsoleFlush();
}

/*************************************************************************************************/
/*!
 *  \brief  Display a dialog on a Console
 *
 *  \param  pDialog     Pointer to the dialog object to display.
 *
 *  \return None
 */
/*************************************************************************************************/
static void uiConsoleDispDialog(const UiDialog_t *pDialog)
{
  int8_t i;
  char ch[2];

  /* Print the dialog widget identifier */
  UiConsolePrint("\r\n");
  UiConsolePrintLn("{Dialog}");

  /* Print the title to the Console */
  UiConsolePrintLn(pDialog->pTitle);

  /* Print the message to the Console */
  UiConsolePrintLn(pDialog->pMsg);

  if (pDialog->type == UI_DLG_TYPE_INPUT_SELECT)
  {
    if (pDialog->numSelectItems == 0)
    {
      UiConsolePrintLn("ENTER to continue");
    }
    else
    {
      /* Print the dialog items */
      for (i = 0; i < pDialog->numSelectItems; i++)
      {
        UiConsolePrint("  ");
        ch[0] = '1' + i;
        ch[1] = '\0';
        UiConsolePrint(ch);
        UiConsolePrint(". ");
        UiConsolePrintLn(pDialog->pSelectItems[i]);
      }
    }
  }
  else
  {
    /* Print prompt */
    UiConsolePrint("> ");
    UiConsolePrint(pDialog->pEntry);
  }

  UiConsoleFlush();
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize the Console User Interface
 *
 *  \return None
 */
/*************************************************************************************************/
void UiConsoleInit(void)
{
  uiConsoleCb.alphaNumOffset = 0;
  UiRegisterDisplay(uiConsoleActionTbl, UI_DISPLAY_CONSOLE);
}
