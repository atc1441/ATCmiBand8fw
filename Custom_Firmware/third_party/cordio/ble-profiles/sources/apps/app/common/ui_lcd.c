/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  User Interface - LCD
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

/**************************************************************************************************
  Macros
**************************************************************************************************/

/* Get these values from driver? */
#define LCD_NUM_LINES                 4
#define LCD_LINE_LEN                  20
#define LCD_SEL_COL_WIDTH             2

/* Map button press to LCD actions */
#define LCD_KEYPRESS_UP               UI_INPUT_BTN_UP
#define LCD_KEYPRESS_DOWN             UI_INPUT_BTN_DOWN
#define LCD_KEYPRESS_SELECT           UI_INPUT_BTN_SELECT

/* Scroll timeout in ms */
#define LCD_SCROLL_LONG_TO            1000
#define LCD_SCROLL_SHORT_TO           250

#define LCD_PROMPT_STR                ">>"
#define LCD_RO_PROMPT_STR             "*>"

/**************************************************************************************************
  Function Prototypes
**************************************************************************************************/

/*! LCD display action function prototypes */
static void uiLcdDispSplash(const UiSplashScreen_t *pSplash);
static void uiLcdDispMenu(const UiMenu_t *pMenu);
static void uiLcdDispDialog(const UiDialog_t *pDialog);
static void uiLcdProcessKey(uint8_t key);

/*! Process Key action functions */
static void uiLcdProcessSplashKey(uint8_t key);
static void uiLcdProcessMenuKey(uint8_t key);
static void uiLcdProcessDialogKey(uint8_t key);

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/* LCD Control Block */
struct
{
  int8_t  scrollOffset;
  int8_t  alphaNumOffset;
} uiLcdCb;

/* LCD Process Key action functions */
UiKeyPress_t uiLcdProcessKeyTbl[] =
{
  uiLcdProcessSplashKey,
  uiLcdProcessMenuKey,
  uiLcdProcessDialogKey
};

/* LCD display action functions */
static UiActionTbl_t uiLcdActionTbl =
{
  uiLcdDispSplash,
  uiLcdDispMenu,
  uiLcdDispDialog,
  uiLcdProcessKey
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
static void uiLcdProcessKey(uint8_t key)
{
  uiLcdProcessKeyTbl[UiCb.activeScreenType](key);
}

/*************************************************************************************************/
/*!
 *  \brief  Process a key press from the user - Splash screens
 *
 *  \param  key     User keypress.
 *
 *  \return None
 */
/*************************************************************************************************/
static void uiLcdProcessSplashKey(uint8_t key)
{
  switch(key)
  {
  case LCD_KEYPRESS_UP:
  case LCD_KEYPRESS_DOWN:
  case LCD_KEYPRESS_SELECT:
  default:
    /* Proceed to main menu */
    UiLoadMenu(UiCb.pMainMenu);
    break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Process a key press from the user - Menu screens
 *
 *  \param  key     User keypress.
 *
 *  \return None
 */
/*************************************************************************************************/
static void uiLcdProcessMenuKey(uint8_t key)
{
  UiMenu_t *pMenu = (UiMenu_t*) UiCb.pActiveScreen;

  switch(key)
  {
  case LCD_KEYPRESS_UP:
    /* Move the highlight up or wrap down if at top */
    uiLcdCb.scrollOffset = 0;
    pMenu->highlight = (pMenu->highlight > 0) ? (pMenu->highlight - 1) : (pMenu->numItems - 1);
    UiRefresh();
    break;

  case LCD_KEYPRESS_DOWN:
    /* Move the highlight down or wrap up if at bottom */
    uiLcdCb.scrollOffset = 0;
    pMenu->highlight = (pMenu->highlight + 1) % (pMenu->numItems);
    UiRefresh();
    break;

  case LCD_KEYPRESS_SELECT:
    /* Select the highlighted item */
    uiLcdCb.scrollOffset = 0;
    UiSelection(pMenu->highlight + 1);
    break;

  default:
    break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Select the next character depending on the type of dialog and if the up or down button
 *          was pressed.
 *
 *  \param  pDialog     Pointer to the dialog
 *  \param  up          TRUE if up key is pressed, else the down key
 *
 *  \return The next character to display
 */
/*************************************************************************************************/
static char uiLcdAlphaNumNextChar(const UiDialog_t *pDialog, bool_t up)
{
  uint8_t offset = uiLcdCb.alphaNumOffset;
  char ch = pDialog->pEntry[offset];

  if (pDialog->type == UI_DLG_TYPE_INPUT_NUM)
  {
    /* Cycle through 0-9 */
    if (up)
    {
      ch = (ch < '9') ? ((ch >= '0') ? ch + 1 : '0'): '0';
    }
    else
    {
      ch = (ch > '0') ? ((ch <= '9') ? ch - 1 : '9') : '9';
    }
  }
  else
  {
    /* Cycle through space, 0-9, underscore, A-Z */
    if (up)
    {
      ch = (ch == ' ') ? '0' : (ch == '9') ? '_' : (ch == '_') ? 'A' : (ch == 'Z') ? ' ' : (ch < ' ' || ch > '_') ? ' ' : ch + 1;
    }
    else
    {
      ch = (ch == ' ') ? 'Z' : (ch == 'A') ? '_' : (ch == '_') ? '9' : (ch == '0') ? ' ' : (ch < ' ' || ch > '_') ? 'Z' :ch - 1;
    }
  }

  return ch;
}

/*************************************************************************************************/
/*!
 *  \brief  Process a key press from the user - Numeric or alpha-numeric dialog screens
 *
 *  \param  key     User keypress.
 *
 *  \return None
 */
/*************************************************************************************************/
static void uiLcdProcessDialogAlphaNum(uint8_t key)
{
  UiDialog_t *pDialog = (UiDialog_t*) UiCb.pActiveScreen;
  uint8_t offset = uiLcdCb.alphaNumOffset;

  switch(key)
  {
  case LCD_KEYPRESS_UP:
    if (offset < pDialog->entryMaxLen)
    {
      /* offset in range, change character */
      pDialog->pEntry[offset] = uiLcdAlphaNumNextChar(pDialog, TRUE);
    }
    else if (pDialog->highlight > 0)
    {
      /* offset out of range, move selection */
      pDialog->highlight--;
    }
    break;

  case LCD_KEYPRESS_DOWN:
    if (offset < pDialog->entryMaxLen)
    {
      /* offset in range, change character */
      pDialog->pEntry[offset] = uiLcdAlphaNumNextChar(pDialog, FALSE);
    }
    else if (pDialog->highlight < 1)
    {
      /* offset out of range, move selection */
      pDialog->highlight++;
    }
    break;

  case LCD_KEYPRESS_SELECT:
    if (offset < pDialog->entryMaxLen)
    {
      /* offset in range, move lower carrot to next character */
      uiLcdCb.alphaNumOffset++;
    }
    else
    {
      /* offset out of range, select the highlight */
      if (pDialog->highlight == 0)
      {
        uiLcdCb.alphaNumOffset = 0;
      }
      else
      {
        /* User is done */
        uiLcdCb.alphaNumOffset = 0;
        pDialog->highlight = 0;

        /* Notify callback of selection */
        UiSelection(0);

        /* Exit to parent */
        UiLoadMenu(pDialog->base.pParentMenu);
      }
    }
    break;
  }

  UiRefresh();
}

/*************************************************************************************************/
/*!
 *  \brief  Process a key press from the user - Select Dialog Screens
 *
 *  \param  key     User keypress.
 *
 *  \return None
 */
/*************************************************************************************************/
static void uiLcdProcessDialogSelect(uint8_t key)
{
  UiDialog_t *pDialog = (UiDialog_t*) UiCb.pActiveScreen;

  switch(key)
  {
  case LCD_KEYPRESS_UP:
    /* Move the highlight up */
    if (pDialog->highlight > 0)
    {
      uiLcdCb.scrollOffset = 0;
      pDialog->highlight--;
      UiRefresh();
    }
    break;

  case LCD_KEYPRESS_DOWN:
    /* Move the highlight down */
    if (pDialog->highlight < pDialog->numSelectItems - 1)
    {
      uiLcdCb.scrollOffset = 0;
      pDialog->highlight++;
      UiRefresh();
    }
    break;

  case LCD_KEYPRESS_SELECT:
    /* Select the highlighted item */
    uiLcdCb.scrollOffset = 0;
    UiSelection(pDialog->highlight + 1);
    break;

  default:
    break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Process a key press from the user - Dialog Screens
 *
 *  \param  key     User keypress.
 *
 *  \return None
 */
/*************************************************************************************************/
static void uiLcdProcessDialogKey(uint8_t key)
{
  UiDialog_t *pDialog = (UiDialog_t*) UiCb.pActiveScreen;

  /* process input based on dialog type (select vs alpha numeric input) */
  switch(pDialog->type)
  {
  case UI_DLG_TYPE_INPUT_NUM:
  case UI_DLG_TYPE_INPUT_ALPHANUM:
    uiLcdProcessDialogAlphaNum(key);
    break;

  case UI_DLG_TYPE_INPUT_SELECT:
    uiLcdProcessDialogSelect(key);
    break;

  default:
    break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Process a timer tick from the UI layer
 *
 *  \return None
 */
/*************************************************************************************************/
static void uiLcdScrollCallback(void)
{
  if (UiCb.activeScreenType == UI_SCREEN_MENU)
  {
    UiMenu_t *pMenu = (UiMenu_t *) UiCb.pActiveScreen;

    /* scroll the higlighted menu item if necessary */
    if (++uiLcdCb.scrollOffset > (int8_t) (strlen(pMenu->pItems[pMenu->highlight]) - (LCD_LINE_LEN - LCD_SEL_COL_WIDTH)))
    {
      uiLcdCb.scrollOffset = 0;
    }

    UiRefresh();
  }
  else if (UiCb.activeScreenType == UI_SCREEN_DIALOG)
  {
    UiDialog_t *pDialog = (UiDialog_t *) UiCb.pActiveScreen;

    /* Scroll the highlighted select items if necessary */
    if (++uiLcdCb.scrollOffset > (int8_t) (strlen(pDialog->pSelectItems[pDialog->highlight]) - (LCD_LINE_LEN - LCD_SEL_COL_WIDTH)))
    {
      uiLcdCb.scrollOffset = 0;
    }

    UiRefresh();
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Display a splash screen on an LCD
 *
 *  \param  pSplash   Pointer to the splash screen object to display.
 *
 *  \return None
 */
/*************************************************************************************************/
static void uiLcdDispSplash(const UiSplashScreen_t *pSplash)
{
  /* Display the splash screen */
  UiLcdWriteLine(0, pSplash->pAppName);
  UiLcdWriteLine(1, pSplash->pAppVer);
  UiLcdWriteLine(2, "");
  UiLcdWriteLine(3, pSplash->pCopyright);

  UiLcdSetDataPrepared();
  UiLcdFlush();
}

/*************************************************************************************************/
/*!
 *  \brief  Display a title on an LCD with brackets around it
 *
 *  \param  title     Title to display.
 *
 *  \return None
 */
/*************************************************************************************************/
static void uiLcdDispTitle(const char *title)
{
  char line[LCD_LINE_LEN + 1];

  memset(line, '\0', LCD_LINE_LEN+1);

  /* Put brackets around the title */
  line[0] = '[';
  strncpy(line+1, title, LCD_LINE_LEN-2);
  strcat(line, "]");

  /* Print the title to the LCD */
  UiLcdWriteLine(0, line);
}

/*************************************************************************************************/
/*!
 *  \brief  Calculate first and last item displayed on the screen
 *
 *  \param  pStart      Pointer hold the first item ID
 *  \param  pEnd        Pointer hold the last item ID (one greater than last)
 *  \param  numLines    Number of lines to display items on the screen
 *  \param  highlight   ID of the highlighted item
 *  \param  numItems    Total number of items
 *
 *  \return None
 */
/*************************************************************************************************/
static void appCalcFirstAndLastItem(int8_t *pStart, int8_t *pEnd, uint8_t numLines,
                                    uint8_t highlight, uint8_t numItems)
{
  int8_t start, end;

  /* Assume the highligh will be in the center */
  start = highlight - (numLines - 1) / 2;

  /* If centering the highlight puts the start below zero, start at the beginning */
  if (start < 0)
  {
    start = 0;
  }

  /* Assume the end is the start plus the number of screen lines */
  end = start + numLines;

  /* Check to see if the end overflows */
  if (end > numItems)
  {
    end = numItems;

    /* The last item should be as close to the bottom of the screen as possible */
    /* Adjust the start to accomodate this */
    start = end - numLines;

    if (start < 0)
    {
      start = 0;
    }
  }

  /* Save the result */
  *pStart = start;
  *pEnd = end;
}

/*************************************************************************************************/
/*!
 *  \brief  process the scrolling of long highlight items
 *
 *  \param  pText     Pointer to the highlighted text.
 *  \param  offset    The offset of the scroll.
 *
 *  \return None
 */
/*************************************************************************************************/
static bool_t uiLcdProcessScroll(const char *pText, uint8_t offset)
{
  bool_t scrolling = FALSE;
  uint8_t len = (uint8_t) strlen(pText);

  /* Check if the item length is grater than the LCD (minus space for the select carrot) */
  if (len > LCD_LINE_LEN - LCD_SEL_COL_WIDTH)
  {
    uint16_t timeout = LCD_SCROLL_SHORT_TO;

    scrolling = TRUE;

    /* Delay longer at the beginning and the end of the scroll */
    if ((offset == 0) || (offset == (len - (LCD_LINE_LEN - LCD_SEL_COL_WIDTH))))
    {
      timeout = LCD_SCROLL_LONG_TO;
    }

    /* Start the tick timer */
    UiScrollTimerStart(uiLcdScrollCallback, timeout);
  }

  return scrolling;
}

/*************************************************************************************************/
/*!
 *  \brief  Display a menu on an LCD
 *
 *  \param  pMenu     Pointer to the menu object to display.
 *
 *  \return None
 */
/*************************************************************************************************/
static void uiLcdDispMenu(const UiMenu_t *pMenu)
{
  int8_t i, start, end, pos = 1;
  bool_t scrolling = FALSE;

  WSF_ASSERT(pMenu);
  WSF_ASSERT(pMenu->pTitle);

  /* Print the title to the LCD */
  uiLcdDispTitle(pMenu->pTitle);

  /* Calculate first and last item displayed on the screen */
  appCalcFirstAndLastItem(&start, &end, LCD_NUM_LINES-1, pMenu->highlight, pMenu->numItems);

  /* Display menu items */
  for (i = start; i < end; i++)
  {
    int8_t offset = 0;
    char line[LCD_LINE_LEN+1];

    line[0] = ' ';
    line[1] = ' ';

    if (i == pMenu->highlight)
    {
      /* Add the highlight to the line */
      if (pMenu->readOnlyMask & (1<<i))
      {
        strncpy(line, LCD_RO_PROMPT_STR, sizeof(line));
      }
      else
      {
        strncpy(line, LCD_PROMPT_STR, sizeof(line));
      }

      /* Check if we need to scroll through a long item */
      offset = uiLcdCb.scrollOffset;
      scrolling = uiLcdProcessScroll(pMenu->pItems[i], offset);
    }

    strncpy(line+LCD_SEL_COL_WIDTH, pMenu->pItems[i] + offset, LCD_LINE_LEN-LCD_SEL_COL_WIDTH);
    UiLcdWriteLine(pos++, line);
  }

  /* Clear unused lines */
  for (i = pos; i < LCD_NUM_LINES; i++)
  {
    UiLcdWriteLine(i, "");
  }

  if (!scrolling)
  {
    UiScrollTimerStop();
  }

  UiLcdSetDataPrepared();
  UiLcdFlush();
}

/*************************************************************************************************/
/*!
 *  \brief  Complete display of dialog on an LCD - Dialog type alphanumeric or numeric
 *
 *  \param  pDialog     Pointer to the dialog object to display.
 *
 *  \return None
 */
/*************************************************************************************************/
static void uiLcdDispDialogAlphaNum(const UiDialog_t *pDialog)
{
  char line[LCD_LINE_LEN+1];
  char cmdLine[LCD_LINE_LEN+1];

  line[0] = ' ';
  line[1] = ' ';

  WSF_ASSERT(pDialog->pEntry);

  strncpy(line + LCD_SEL_COL_WIDTH, pDialog->pEntry, LCD_LINE_LEN - LCD_SEL_COL_WIDTH);

  memset(cmdLine, ' ', LCD_LINE_LEN);
  cmdLine[LCD_LINE_LEN] = '\0';

  if (uiLcdCb.alphaNumOffset < pDialog->entryMaxLen)
  {
    /* Print a carrot under the active character */
    cmdLine[uiLcdCb.alphaNumOffset + LCD_SEL_COL_WIDTH] = '^';
  }
  else
  {
    /* After the user presses select on the last character, add a 'Done' entry item */
    strcpy(cmdLine, "  Done");

    if (pDialog->highlight == 0)
    {
      strncpy(line, LCD_PROMPT_STR, sizeof(line));
    }
    else
    {
      strncpy(cmdLine, LCD_PROMPT_STR, sizeof(line));
    }
  }

  /* Print to the LCD */
  UiLcdWriteLine(2, line);
  UiLcdWriteLine(3, cmdLine);
}

/*************************************************************************************************/
/*!
 *  \brief  Complete display of dialog on an LCD - Dialog type select
 *
 *  \param  pDialog     Pointer to the dialog object to display.
 *
 *  \return None
 */
/*************************************************************************************************/
static void uiLcdDispDialogSelect(const UiDialog_t *pDialog)
{
  int8_t i, start, end, pos = 2;
  bool_t scrolling = FALSE;

  /* Calculate first and last item displayed on the screen */
  appCalcFirstAndLastItem(&start, &end, LCD_NUM_LINES-2, pDialog->highlight, pDialog->numSelectItems);

  /* Display dialog items */
  for (i = start; i < end; i++)
  {
    int8_t offset = 0;
    char line[LCD_LINE_LEN+1];

    line[0] = ' ';
    line[1] = ' ';

    if (i == pDialog->highlight)
    {
      /* Add the highlight to the line */
      strncpy(line, LCD_PROMPT_STR, sizeof(line));

      /* Check if we need to scroll through a long item */
      offset = uiLcdCb.scrollOffset;
      scrolling = uiLcdProcessScroll(pDialog->pSelectItems[i], offset);
    }

    strncpy(line+LCD_SEL_COL_WIDTH, pDialog->pSelectItems[i] + offset, LCD_LINE_LEN - LCD_SEL_COL_WIDTH);
    UiLcdWriteLine(pos++, line);
  }

  if (!scrolling)
  {
    UiScrollTimerStop();
  }

  /* Clear unused lines */
  for (i=pos; i<LCD_NUM_LINES; i++)
  {
    UiLcdWriteLine(i, "");
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Display a dialog on an LCD
 *
 *  \param  pDialog     Pointer to the dialog object to display.
 *
 *  \return None
 */
/*************************************************************************************************/
static void uiLcdDispDialog(const UiDialog_t *pDialog)
{
  /* Print the title to the LCD */
  uiLcdDispTitle(pDialog->pTitle);

  /* Print the message to the LCD */
  UiLcdWriteLine(1, pDialog->pMsg);

  if (pDialog->type == UI_DLG_TYPE_INPUT_SELECT)
  {
    uiLcdDispDialogSelect(pDialog);
  }
  else
  {
    uiLcdDispDialogAlphaNum(pDialog);
  }

  UiLcdSetDataPrepared();
  UiLcdFlush();
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize the LCD User Interface
 *
 *  \return None
 */
/*************************************************************************************************/
void UiLcdInit(void)
{
  UiRegisterDisplay(uiLcdActionTbl, UI_DISPLAY_LCD);
}
