/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  User Interface main module
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

/***************************************************************************************************
 Macros
***************************************************************************************************/

/* Keyboard ASCII codes for up, down, select keys */
#define UI_MAIN_UP_CHAR                 0x48
#define UI_MAIN_DOWN_CHAR               0x50
#define UI_MAIN_SELECT_CHAR             0x4d

/***************************************************************************************************
 Global Variables
***************************************************************************************************/

/* Control block */
UiCb_t UiCb;

/***************************************************************************************************
 Local Variables
***************************************************************************************************/

/* Registerable callback for events from a running demo app */
static UiProcEvent_t uiAppEventCallback;

/* Timer to track duration on duration dialog */
static uint32_t uiDuration;

/* Buffer to hold the duration string */
static char uiDurationStr[LCD_LINE_LEN];

/* Buffer to hold the extended text field on the duration dialog */
static char uiDurationExtText[LCD_EXT_LINE_LEN];

/* Duration Dialog selections */
static constStr uiDurationSelectList[] =
{
  "OK"
};

/* Duration Dialog selections with extended text */
static constStr uiDurationSelectListExt[] =
{
  uiDurationExtText,
  "OK"
};

/* Duration Dialog */
static UiDialog_t uiDurationDlg =
{
  {NULL, NULL},
  "",
  uiDurationStr,
  UI_DLG_TYPE_INPUT_SELECT,
  NULL,
  0,
  0,
  NULL,
  1,
  uiDurationSelectList
};

/* Message Selection */
static constStr uiMessageSelectList[] =
{
  "OK",
  "Exit",
  "Abort",
  "Stop"
};

/* Message Dialog */
static UiDialog_t uiMessageDlg =
{
  {NULL, NULL},
  "",
  "",
  UI_DLG_TYPE_INPUT_SELECT,
  NULL,
  0,
  0,
  NULL,
  1,
  NULL
};

/*************************************************************************************************/
/*!
 *  \brief  Display a menu on the active UI
 *
 *  \param  pMenu     Pointer to the menu object to display.
 *
 *  \return None
 */
/*************************************************************************************************/
void UiLoadMenu(const UiMenu_t *pMenu)
{
  int8_t i;

  /* Store the active screen */
  UiCb.pActiveScreen = (UiBase_t*) pMenu;
  UiCb.activeScreenType = UI_SCREEN_MENU;

  if (pMenu->selectCback)
  {
    pMenu->selectCback(pMenu, UI_MENU_ITEM_ON_OPEN);
  }

  for (i = 0; i < UI_DISPLAY_MAX; i++)
  {
    if (UiCb.uiActionTbl[i].displayMenu)
    {
      UiCb.uiActionTbl[i].displayMenu(pMenu);
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Display a splash screen on the active UI
 *
 *  \param  pSplash   Pointer to the splash screen object to display.
 *
 *  \return None
 */
/*************************************************************************************************/
void UiLoadSplash(const UiSplashScreen_t *pSplash)
{
  int8_t i;

  /* Store the active screen */
  UiCb.pActiveScreen = (UiBase_t*) pSplash;
  UiCb.activeScreenType = UI_SCREEN_SPLASH;

  for (i = 0; i < UI_DISPLAY_MAX; i++)
  {
    if (UiCb.uiActionTbl[i].displaySplash)
    {
      UiCb.uiActionTbl[i].displaySplash(pSplash);
    }
  }

  if (pSplash->durMs)
  {
    UiTimerStart(UI_SPLASH_TIMER_IND, pSplash->durMs);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Display a dialog on the active UI
 *
 *  \param  pDialog     Pointer to the dialog object to display.
 *
 *  \return None
 */
/*************************************************************************************************/
void UiLoadDialog(const UiDialog_t *pDialog)
{
  int8_t i;

  /* Store the active screen */
  UiCb.pActiveScreen = (UiBase_t*) pDialog;
  UiCb.activeScreenType = UI_SCREEN_DIALOG;

  if (pDialog->pEntry && pDialog->pEntry[0] == '\0')
  {
    char ch = (pDialog->type == UI_DLG_TYPE_INPUT_NUM) ? '0' : ' ';

    memset(pDialog->pEntry, ch, pDialog->entryMaxLen);
  }

  for (i = 0; i < UI_DISPLAY_MAX; i++)
  {
    if (UiCb.uiActionTbl[i].displayDialog)
    {
      UiCb.uiActionTbl[i].displayDialog(pDialog);
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Called when the user inputs a value into a dialog
 *
 *  \param  pInput        Input string
 *
 *  \return None
 */
/*************************************************************************************************/
void UiInput(char *pInput)
{

}

/*************************************************************************************************/
/*!
 *  \brief  Called when the user selects an item in a menu or dialog
 *
 *  \param  selection     The number of the selection (1-9) or 0 for single select dialog input
 *
 *  \return None
 */
/*************************************************************************************************/
void UiSelection(uint8_t selection)
{
  if (UiCb.pActiveScreen)
  {
    if (UiCb.activeScreenType == UI_SCREEN_MENU)
    {
      UiMenu_t *pMenu = (UiMenu_t*) UiCb.pActiveScreen;

      /* Check if the selected item is read-only */
      if (pMenu->readOnlyMask & (1 << (selection-1)))
      {
        return;
      }

      if (pMenu->selectCback)
      {
        pMenu->selectCback(pMenu, selection);
      }
    }
    else if (UiCb.activeScreenType == UI_SCREEN_DIALOG)
    {
      UiDialog_t *pDialog = (UiDialog_t*) UiCb.pActiveScreen;

      if (pDialog->selectCback)
      {
        pDialog->selectCback(pDialog, selection);
      }
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Refresh the active Display
 *
 *  \return None
 */
/*************************************************************************************************/
void UiRefresh(void)
{
  int8_t i;

  switch(UiCb.activeScreenType)
  {
  case UI_SCREEN_SPLASH:
    for (i = 0; i < UI_DISPLAY_MAX; i++)
    {
      if (UiCb.uiActionTbl[i].displaySplash)
      {
        UiCb.uiActionTbl[i].displaySplash((UiSplashScreen_t*) UiCb.pActiveScreen);
      }
    }
    break;

  case UI_SCREEN_MENU:
    for (i = 0; i < UI_DISPLAY_MAX; i++)
    {
      if (UiCb.uiActionTbl[i].displayMenu)
      {
        UiCb.uiActionTbl[i].displayMenu((UiMenu_t*) UiCb.pActiveScreen);
      }
    }
    break;

  case UI_SCREEN_DIALOG:
    for (i = 0; i < UI_DISPLAY_MAX; i++)
    {
      if (UiCb.uiActionTbl[i].displayDialog)
      {
        UiCb.uiActionTbl[i].displayDialog((UiDialog_t*) UiCb.pActiveScreen);
      }
    }
    break;

  default:
    break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Start the scroll timer
 *
 *  \param  scrollCback     Callback called on timer timeout
 *  \param  ms              Period between timeout in milliseconds
 *
 *  \return None
 */
/*************************************************************************************************/
void UiScrollTimerStart(const UiScrollCback_t scrollCback, uint16_t ms)
{
  /* Record control information */
  UiCb.scrollCback = scrollCback;

  /* Start timer */
  UiTimerStart(UI_SCROLL_TIMER_IND, ms);
}

/*************************************************************************************************/
/*!
 *  \brief  Stop the scroll timer
 *
 *  \return None
 */
/*************************************************************************************************/
void UiScrollTimerStop(void)
{
  UiTimerStop(UI_SCROLL_TIMER_IND);
}

/*************************************************************************************************/
/*!
 *  \brief  Called when an item is selected on the duration dialog
 *
 *  \param  pRef        The referring object
 *  \param  selection   The item selected by the user
 *
 *  \return None
 */
/*************************************************************************************************/
static void uiDurationDlgSelect(const void *pRef, uint8_t selection)
{
  UiDialog_t *pDialog = (UiDialog_t  *) pRef;

  if (pDialog->pSelectItems == uiDurationSelectListExt)
  {
    if (selection == 0)
    {
      return;
    }
  }

  /* Stop the timer */
  UiTimerStop(UI_DURATION_TIMER_IND);

  /* Call the exit dialog callback registered by the application */
  if (UiCb.durationDlgCback)
  {
    UiCb.durationDlgCback(pDialog, selection);
  }

  /* Load the parent menu */
  UiLoadMenu(pDialog->base.pParentMenu);
}

/*************************************************************************************************/
/*!
 *  \brief  Called to fill the duration string in format "Duration: HH:MM:SS"
 *
 *  \param  pBuf      Buffer to put the duration string (19 byte min)
 *  \param  duration  Duration in seconds
 *
 *  \return None
 */
/*************************************************************************************************/
static void uiFillDurationStr(char *pBuf, uint32_t duration)
{
  uint8_t seconds, minutes, hours;

  strcpy(pBuf, "Duration:   :  :  ");

  seconds = (uint8_t) (duration % 60);
  minutes = (uint8_t) ((duration / 60) % 60);
  hours = (uint8_t) (duration / 3600);

  pBuf[10] = '0' + (hours / 10) % 10;
  pBuf[11] = '0' + hours % 10;

  pBuf[13] = '0' + (minutes / 10) % 10;
  pBuf[14] = '0' + minutes % 10;

  pBuf[16] = '0' + (seconds / 10) % 10;
  pBuf[17] = '0' + seconds % 10;
}

/*************************************************************************************************/
/*!
 *  \brief  (Optional) Set the extended text field in the duration dialog.
 *
 *  \param  pText     Title for the dialog
 *
 *  \return None
 */
/*************************************************************************************************/
void UiDurationDialogSetExtText(char *pText)
{
  if (pText == NULL || strlen(pText) == 0)
  {
    /* Remove the extended text from the dialog */
    uiDurationDlg.pSelectItems = uiDurationSelectList;
  }
  else
  {
    /* Set the extended text in the dialog */
    strncpy(uiDurationExtText, pText, sizeof(uiDurationExtText) - 1);
    uiDurationDlg.pSelectItems = uiDurationSelectListExt;

    UiRefresh();
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Display a dialog that shows the amount of time the application has been running
 *          and a selectable dialog item to exit the application.
 *
 *  \param  pTitle    Title for the dialog
 *  \param  pParent   Parent menu to load after running dialog exits
 *  \param  cback     Callback called when dialog exits
 *
 *  \return None
 */
/*************************************************************************************************/
void UiLoadDurationDialog(const char *pTitle, void *pParent, UiDialogSelCback_t cback)
{
  /* Configure the dialog */
  uiDurationDlg.pTitle = pTitle;
  uiDurationDlg.base.pParentMenu = pParent;
  uiDurationDlg.selectCback = uiDurationDlgSelect;

  /* Register the callback */
  UiCb.durationDlgCback = cback;

  /* Setup and start a timer to display the duration of the test */
  uiDuration = 0;
  uiFillDurationStr(uiDurationStr, uiDuration);

  UiTimerStart(UI_DURATION_TIMER_IND, 1000);

  /* Load the dialog */
  UiLoadDialog((const UiDialog_t*)  &uiDurationDlg);
}

/*************************************************************************************************/
/*!
 *  \brief  Called when an item is selected on the Message dialog
 *
 *  \param  pRef        The referring object
 *  \param  selection   The item selected by the user
 *
 *  \return None
 */
/*************************************************************************************************/
void uiMessageDlgSelect(const void *pRef, uint8_t selection)
{
  UiDialog_t *pDialog = (UiDialog_t *) pRef;

  /* Call the exit dialog callback registered by the application */
  if (UiCb.messageDlgCback)
  {
    UiCb.messageDlgCback(pDialog, selection);
  }

  /* Load the parent menu */
  UiLoadMenu(pDialog->base.pParentMenu);
}

/*************************************************************************************************/
/*!
 *  \brief  Display a message dialog.
 *
 *  \param  pTitle    Title for the dialog
 *  \param  pMsg      Message to display
 *  \param  pExitBtn  Index for the confirm ("OK", "Exit", "Abort", etc...) button (See Macros)
 *  \param  pParent   Parent menu to load after running dialog exits
 *  \param  cback     Callback called when dialog exits
 *
 *  \return None
 */
/*************************************************************************************************/
void UiLoadMessageDialog(const char *pTitle, const char *pMsg, uint8_t pExitBtn,  void *pParent, UiDialogSelCback_t cback)
{
  /* Configure the dialog */
  uiMessageDlg.pTitle = pTitle;
  uiMessageDlg.pMsg = pMsg;
  uiMessageDlg.pSelectItems = uiMessageSelectList + pExitBtn;
  uiMessageDlg.base.pParentMenu = pParent;
  uiMessageDlg.selectCback = uiMessageDlgSelect;

  /* Register the callback */
  UiCb.messageDlgCback = cback;

  /* Load the dialog */
  UiLoadDialog(&uiMessageDlg);
}

/*************************************************************************************************/
/*!
 *  \brief  Called to process user input.
 *
 *  \param  input    Input from user
 *
 *  \return None
 */
/*************************************************************************************************/
void UiProcessUserInput(uint8_t input)
{
  int8_t i;

  for (i = 0; i < UI_DISPLAY_MAX; i++)
  {
    if (UiCb.uiActionTbl[i].keyPress)
    {
      UiCb.uiActionTbl[i].keyPress(input);
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Register a callback function to receive UI events
 *
 *  \param  cback     Callback function to receive events
 *
 *  \return None
 */
/*************************************************************************************************/
void UiRegisterAppEvtCback(UiProcEvent_t cback)
{
  uiAppEventCallback = cback;
}

/*************************************************************************************************/
/*!
 *  \brief  Process a UI event
 *
 *  \param  event     Event to process
 *
 *  \return None
 */
/*************************************************************************************************/
void UiProcEvent(uint8_t event)
{
  switch (event)
  {
  case UI_SPLASH_TIMER_IND:
    UiLoadMenu(UiCb.pMainMenu);
    break;

  case UI_SCROLL_TIMER_IND:
    if (UiCb.scrollCback)
    {
      UiCb.scrollCback();
    }
    break;

  case UI_BUTTON_UP_IND:
    UiProcessUserInput(UI_INPUT_BTN_UP);
    break;

  case UI_BUTTON_DOWN_IND:
    UiProcessUserInput(UI_INPUT_BTN_DOWN);
    break;

  case UI_BUTTON_SELECT_IND:
    UiProcessUserInput(UI_INPUT_BTN_SELECT);
    break;

  case UI_DURATION_TIMER_IND:

    /* Update the duration */
    uiDuration++;
    uiFillDurationStr(uiDurationStr, uiDuration);

    /* Restart the timer */
    UiTimerStart(UI_DURATION_TIMER_IND, 1000);

    /* Refresh the UI */
    UiRefresh();
    break;

  default:
    break;
  }

  if (uiAppEventCallback)
  {
    uiAppEventCallback(event);
  }
}

/*************************************************************************************************/
/*!
 *  \brief   Format an integer string.
 *
 *  \param   pBuf         String buffer.
 *  \param   value        Integer to format.
 *  \param   units        Units appended to after the integer.
 *
 *  \return  Pointer to the end of the formatted string.
 */
/*************************************************************************************************/
static char *uiFormatInt(char *pBuf, uint32_t value)
{
  uint32_t divider = 1;

  while (value > divider * 10)
  {
    divider *= 10;
  }

  while (divider >= 1)
  {
    *pBuf++ = '0' + (uint8_t) (value / divider);
    value -= (value / divider) * divider;
    divider /= 10;
  }

  *pBuf = '\0';

  return pBuf;
}

/*************************************************************************************************/
/*!
 *  \brief   Format an integer string.
 *
 *  \param   pBuf         String buffer.
 *  \param   value        Integer to format.
 *  \param   units        Units appended to after the integer.
 *
 *  \return  Pointer to the end of the formatted string.
 */
/*************************************************************************************************/
static char *uiFormatHexByte(char *pBuf, uint8_t value)
{
  static const char hex[] = "0123456789ABCDEF";

  *pBuf++ = hex[value >> 4];
  *pBuf++ = hex[value & 0x0F];
  *pBuf = '\0';

  return pBuf;
}

/*************************************************************************************************/
/*!
 *  \brief   Format an integer string delimited with commas every third order of magnitude.
 *
 *  \param   pBuf         String buffer.
 *  \param   value        Integer to format.
 *  \param   units        Units appended to after the integer.
 *
 *  \return  Pointer to the end of the formatted string.
 */
/*************************************************************************************************/
static char *uiFormatIntDelimited(char *pBuf, uint32_t value, uint8_t units)
{
  int8_t digit=0;
  uint32_t divider = 1;
  static constStr uiUnitStrTbl[] = {"s", "ms", "Mbs"};

  while (value >= divider * 10)
  {
    divider *= 10;
    digit++;
  }

  while (divider >= 1)
  {
    *pBuf++ = '0' + (uint8_t) (value / divider);

    if ((digit > 0) && (digit-- % 3 == 0))
    {
      *pBuf++ = ',';
    }

    value -= (value / divider) * divider;
    divider /= 10;
  }

  if (units < UI_FORMAT_INTEGER)
  {
    strcpy(pBuf, uiUnitStrTbl[units]);
    pBuf += strlen(uiUnitStrTbl[units]);
  }

  *pBuf = '\0';

  return pBuf;
}

/*************************************************************************************************/
/*!
 *  \brief   Format a time period as HH:MM:SS.
 *
 *  \param   pBuf         String buffer.
 *  \param   time         Time in seconds.
 *
 *  \return  Pointer to the end of the formatted string.
 */
/*************************************************************************************************/
static char *uiFormatTime(char *pBuf, uint32_t time)
{
  uint8_t seconds, minutes, hours;

  seconds = (uint8_t) (time % 60);
  minutes = (uint8_t) ((time / 60) % 60);
  hours = (uint8_t) (time / 3600);

  *pBuf++ = '0' + (hours / 10) % 10;
  *pBuf++ = '0' + hours % 10;
  *pBuf++ = ':';
  *pBuf++ = '0' + (minutes / 10) % 10;
  *pBuf++ = '0' + minutes % 10;
  *pBuf++ = ':';
  *pBuf++ = '0' + (seconds / 10) % 10;
  *pBuf++ = '0' + seconds % 10;

  *pBuf = '\0';

  return pBuf;
}

/*************************************************************************************************/
/*!
 *  \brief   Format a version as 0.0.0.0.
 *
 *  \param   pBuf         String buffer.
 *  \param   value        Value to format.
 *
 *  \return  Pointer to the end of the formatted string.
 */
/*************************************************************************************************/
static char *uiFormatVersion(char *pBuf, uint32_t verCode)
{
  char *pStr = pBuf;

  uint8_t major, minor, release;
  uint16_t build;

  major = (uint8_t) ((verCode & 0xF0000000) >> 28);
  minor = (uint8_t) ((verCode & 0x0F000000) >> 24);
  release = (uint8_t) ((verCode & 0xFF0000) >> 16);
  build = (uint16_t) (verCode & 0xFFFF);

  pStr = uiFormatInt(pStr, major);
  *pStr++ = '.';

  pStr = uiFormatInt(pStr, minor);
  *pStr++ = '.';

  pStr = uiFormatInt(pStr, release);
  *pStr++ = '.';

  pStr = uiFormatInt(pStr, build);

  return pStr;
}

/*************************************************************************************************/
/*!
 *  \brief  Convert an array of bytes to a hex string.
 *
 *  \param  pValue   Pointer to the array of bytes.
 *  \param  len      Size of pVal.
 *
 *  \return Pointer to string.
 */
/*************************************************************************************************/
static char *array2HexStr(const uint8_t *pValue, uint8_t len)
{
  static const char hex[] = "0123456789ABCDEF";
  static char       str[(UI_MAX_A2HS_LEN*2) + 1];
  char              *pStr = str;

  /* Address is little endian so copy in reverse. */
  pValue += len;

  while (pStr < &str[len*2])
  {
    *pStr++ = hex[*--pValue >> 4];
    *pStr++ = hex[*pValue & 0x0F];
  }

  /* NULL terminate string. */
  *pStr = 0;

  return str;
}

/*************************************************************************************************/
/*!
 *  \brief   Format a delimited hex string in the format <hex 1><delimiter><hex 2><delimiter>...<hex len>
 *
 *  \param   pBuf         String buffer.
 *  \param   pValue       Pointer to the value to convert.
 *  \param   len          Length of pValue in bytes.
 *
 *  \return  Pointer to the end of the formatted string.
 */
/*************************************************************************************************/
static char *uiFormatDelimitedHex(char *pBuf, uint8_t *pValue, char delimiter, uint8_t len)
{
  uint8_t i, pos=0;
  char *pAddrStr = array2HexStr(pValue, len);

  /* Copy hex string and add colons. */
  for (i = 0; i < len; i++)
  {
    pBuf[pos++] = pAddrStr[i*2];
    pBuf[pos++] = pAddrStr[i*2 + 1];

    if (i < len-1)
    {
      pBuf[pos++] = delimiter;
    }
  }

  return pBuf;
}

/*************************************************************************************************/
/*!
 *  \brief   Copy a string and return a pointer to the end of the new string.
 *
 *  \param   pBuf         String buffer.
 *  \param   pStr         String to copy.
 *
 *  \return  Pointer to the end of the formatted string.
 */
/*************************************************************************************************/
uint8_t UiCpyStr(char *pBuf, char *pStr)
{
  strcpy(pBuf, pStr);

  return (uint8_t) strlen(pStr);
}

/*************************************************************************************************/
/*!
 *  \brief   Append a formatted value to a string.
 *
 *  \param   pBuf          String buffer.
 *  \param   value         Value to format.
 *  \param   format        Type of formatting to use.
 *
 *  \return  Pointer to the end of the formatted string.
 */
/*************************************************************************************************/
char *UiFormatValue(char *pBuf, uint32_t value, uint8_t format)
{
  switch (format)
  {
  case UI_FORMAT_TIME:
    return uiFormatTime(pBuf, value);

  case UI_FORMAT_VERSION:
    return uiFormatVersion(pBuf, value);

  case UI_FORMAT_HEX_BYTE:
    return uiFormatHexByte(pBuf, (uint8_t) value);

  default:
    return uiFormatIntDelimited(pBuf, value, format);
  }
}

/*************************************************************************************************/
/*!
 *  \brief   Append a formatted array of values to a string.
 *
 *  \param   pBuf          String buffer.
 *  \param   pValue        Pointer to array.
 *  \param   format        Type of formatting to use.
 *
 *  \return  Pointer to the end of the formatted string.
 */
/*************************************************************************************************/
char *UiFormatArray(char *pBuf, uint8_t *pAddr, uint8_t format)
{
  switch (format)
  {
  case UI_FORMAT_BD_ADDR:
     return uiFormatDelimitedHex(pBuf, pAddr, ':', UI_BD_ADDR_LEN);

  case UI_FORMAT_MAC154_ADDR:
     return uiFormatDelimitedHex(pBuf, pAddr, ':', UI_MAC_ADDR_LEN);

  case UI_FORMAT_MESH_UUID:
     return uiFormatDelimitedHex(pBuf, pAddr, ':', UI_UUID_ADDR_LEN);

  default:
    break;
  }

  return pBuf;
}

/*************************************************************************************************/
/*!
 *  \brief  Register a display.
 *
 *  \param  actionTbl     Table of action functions.
 *  \param  id            Display identifier.
 *
 *  \return None
 */
/*************************************************************************************************/
void UiRegisterDisplay(UiActionTbl_t actionTbl, uint8_t id)
{
  WSF_ASSERT(id < UI_DISPLAY_MAX);

  UiCb.uiActionTbl[id] = actionTbl;
}

/*************************************************************************************************/
/*!
 *  \brief  Called to process user input from a PC keyboard.
 *
 *  \param  ch      input
 *
 *  \return None
 */
/*************************************************************************************************/
void UiProcessKeyboardInput(uint8_t ch)
{
  static bool_t extended = FALSE;

  if (extended)
  {
    /* Process input from a keyboard (up, down, right) into one of three
       keys (up, down, select) on the dev board */

    extended = FALSE;

    if (ch == UI_MAIN_UP_CHAR)
    {
      /* Up button */
      UiProcessUserInput(UI_INPUT_BTN_UP);
    }
    else if (ch == UI_MAIN_SELECT_CHAR)
    {
      /* Right button */
      UiProcessUserInput(UI_INPUT_BTN_SELECT);
    }
    else if (ch == UI_MAIN_DOWN_CHAR)
    {
      /* Down button */
      UiProcessUserInput(UI_INPUT_BTN_DOWN);
    }
    else if (ch == 75)
    {
      UiProcEvent(UI_BUTTON_WAKE_IND);
    }
  }
  else
  {
    if ((uint8_t) ch == 0xe0)
    {
      extended = TRUE;
    }
    else
    {
      UiProcessUserInput((uint8_t) ch);
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief      Initialize the UI.
 *
 *  \param      pSplash       Splash screen to display at startup.
 *  \param      pMenu         Main Menu.
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiInit(const UiSplashScreen_t *pSplash, const UiMenu_t *pMain)
{
  /* Store main meny and start splash screen. */
  UiCb.pMainMenu = pMain;
  UiLoadSplash(pSplash);
}
