/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  User Interface API.
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

#ifndef UI_API_H
#define UI_API_H

#include "wsf_os.h"
#include "app_ui.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief Display types */
#define UI_DISPLAY_CONSOLE              0         /*!< Console display type */
#define UI_DISPLAY_LCD                  1         /*!< LCD display type */
#define UI_DISPLAY_MAX                  2         /*!< Number of display types */

/*! \brief LCD configuration */
#define LCD_NUM_LINES                   4         /*!< Number of lines on the LCD */
#define LCD_LINE_LEN                    20        /*!< Number of characters per line on the LCD */
#define LCD_EXT_LINE_LEN                35        /*!< Number of characters for extended text fields */
#define LCD_SEL_COL_WIDTH               2         /*!< Number of characters for line selection icon */

/*! \brief Dialog types */
#define UI_DLG_TYPE_INPUT_NUM           0         /*!< Type of Dialog - Numeric input */
#define UI_DLG_TYPE_INPUT_ALPHANUM      1         /*!< Type of Dialog - Alpha numeric input */
#define UI_DLG_TYPE_INPUT_SELECT        2         /*!< Type of Dialog - List of selections */

/*! \brief UI events */
#define UI_SPLASH_TIMER_IND             1         /*!< Splash screen timer expired */
#define UI_SCROLL_TIMER_IND             2         /*!< Tick timer expired */
#define UI_DURATION_TIMER_IND           3         /*!< Running dialog duration timer expired */
#define UI_BUTTON_UP_IND                4         /*!< Button up pressed */
#define UI_BUTTON_DOWN_IND              5         /*!< Button down pressed */
#define UI_BUTTON_SELECT_IND            6         /*!< Button selected pressed */
#define UI_BUTTON_WAKE_IND              7         /*!< Wake button pressed */
#define HCI_DEV_CMD_CNT_TIMER_IND       8         /*!< HCI Device cmd/evt counter timer */
#define UI_RESET_TIMER_IND              9         /*!< Timer to wait for DM reset to complete */
#define UI_SLEEP_TIMER_IND              10        /*!< Timer to put application to sleep */
#define UI_STATISTICS_TIMER_IND         11        /*!< Timer to poll and display app statistics */

/*! \brief User key inputs */
#define UI_INPUT_0                      0         /*!< '0' key pressed - For console only */
#define UI_INPUT_1                      1         /*!< '1' key pressed - For console only */
#define UI_INPUT_2                      2         /*!< '2' key pressed - For console only */
#define UI_INPUT_3                      3         /*!< '3' key pressed - For console only */
#define UI_INPUT_4                      4         /*!< '4' key pressed - For console only */
#define UI_INPUT_5                      5         /*!< '5' key pressed - For console only */
#define UI_INPUT_6                      6         /*!< '6' key pressed - For console only */
#define UI_INPUT_7                      7         /*!< '7' key pressed - For console only */
#define UI_INPUT_8                      8         /*!< '8' key pressed - For console only */
#define UI_INPUT_9                      9         /*!< '9' key pressed - For console only */
#define UI_INPUT_BTN_UP                 10        /*!< Up key pressed - For LCD only */
#define UI_INPUT_BTN_DOWN               11        /*!< Down key pressed - For LCD only */
#define UI_INPUT_BTN_SELECT             12        /*!< Select key pressed - For LCD only */

/*! \brief Screen types */
#define UI_SCREEN_SPLASH                0         /*!< Splash screen type */
#define UI_SCREEN_MENU                  1         /*!< Menu screen type */
#define UI_SCREEN_DIALOG                2         /*!< Dialog screen type */
#define UI_SCREEN_INVALID               3         /*!< Invalid screen type */

/*! \brief Message Button Macros */
#define UI_MESSAGE_OK                   0         /*!< OK button type */
#define UI_MESSAGE_EXIT                 1         /*!< Exit button type */
#define UI_MESSAGE_ABORT                2         /*!< Abort button type */
#define UI_MESSAGE_STOP                 3         /*!< Stop button type */

/*! \brief Text format types */
#define UI_FORMAT_SECONDS               0         /*!< Seconds foramt */
#define UI_FORMAT_MILLISECONDS          1         /*!< Milliseconds format */
#define UI_FORMAT_MBS                   2         /*!< Mega bits per second format */
#define UI_FORMAT_INTEGER               3         /*!< Integer format */
#define UI_FORMAT_TIME                  4         /*!< Time format */
#define UI_FORMAT_BD_ADDR               5         /*!< BLE device address */
#define UI_FORMAT_MAC154_ADDR           6         /*!< 15.4 MAC device address */
#define UI_FORMAT_MESH_UUID             7         /*!< Mesh device UUID */
#define UI_FORMAT_VERSION               8         /*!< Version code */
#define UI_FORMAT_HEX_BYTE              9         /*!< Byte in hex */

/*! \brief Text formatting lengths */
#define UI_MAX_A2HS_LEN                 16        /* Max length of array2HexStr conversion. */
#define UI_BD_ADDR_LEN                  6         /* BLE address length. */
#define UI_MAC_ADDR_LEN                 8         /* 802.15.4 MAC address length. */
#define UI_UUID_ADDR_LEN                16        /* Mesh device UUID length. */

/*! \brief Menu open selection */
#define UI_MENU_ITEM_ON_OPEN            0         /*!< Event to selection handler when menu opens */

/*************************************************************************************************/
/*!
 *  \brief      Datatype for menu item select callback function
 *
 *  \param      pMenu       Pointer to the active menu object.
 *  \param      selection   User selection.
 *
 *  \return     None
 */
/*************************************************************************************************/
typedef void (*UiSelCback_t)(const void *pMenu, uint8_t selection);

/*************************************************************************************************/
/*!
 *  \brief      Datatype for dialog item select callback function
 *
 *  \param      pDialog     Pointer to the active dialog object.
 *  \param      selection   User selection.
 *
 *  \return     None
 */
/*************************************************************************************************/
typedef void (*UiDialogSelCback_t)(const void *pDialog, uint8_t selection);

/*************************************************************************************************/
/*!
 *  \brief      Datatype for the LCD scrolling item text timer callback function
 *
 *  \return     None
 */
/*************************************************************************************************/
typedef void (*UiScrollCback_t)(void);

/*************************************************************************************************/
/*!
 *  \brief      Datatype for processing event callback functions
 *
 *  \param      event     Ebent identifier.
 *
 *  \return     None
 */
/*************************************************************************************************/
typedef void (*UiProcEvent_t)(uint8_t event);

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! \brief Pointer to a const string */
typedef const char *constStr;

/*! \brief Screen type */
typedef uint8_t UiScreenType_t;

/*! \brief Input type */
typedef uint8_t UiInputType_t;

/*! \brief Display object base type */
typedef struct
{
  void                *pParentMenu;               /*!< Parent menu */
  void                *pCtx;                      /*!< Opaque context pointer */
} UiBase_t;

/*! \brief Display menu object */
typedef struct
{
  UiBase_t            base;                       /*!< Base object */
  const char          *pTitle;                    /*!< Title of the menu */
  uint8_t             numItems;                   /*!< Size of pItems in number of strings */
  uint8_t             highlight;                  /*!< Index of pItems highlighted on an LCD display */
  UiSelCback_t        selectCback;                /*!< Callback called when a menu item is selected */
  constStr            *pItems;                    /*!< Array of strings containing menu items */
  uint32_t            readOnlyMask;               /*!< Read only mask. If bit is set, menu item at index is RO */
} UiMenu_t;

/*! \brief Display splash screen object */
typedef struct
{
  const char          *pAppName;                  /*!< Name of the application */
  const char          *pAppVer;                   /*!< Version of the application */
  const char          *pCopyright;                /*!< Copyright string */
  uint16_t            durMs;                      /*!< Time in milliseconds splash screen is displayed */
} UiSplashScreen_t;

/*! \brief Display dialog object */
typedef struct
{
  UiBase_t            base;                       /*!< Base object */
  const char          *pTitle;                    /*!< Title of the dialog */
  const char          *pMsg;                      /*!< Message displayed on the dialog */
  uint8_t             type;                       /*!< Type of dialog (UI_DLG_TYPE_INPUT_NUM, UI_DLG_TYPE_INPUT_SELECT, ...) */
  char                *pEntry;                    /*!< Pointer to char array containing input from a user */
  uint8_t             entryMaxLen;                /*!< Size of pEntry in bytes */
  uint8_t             highlight;                  /*!< Index of pSelectItems highlighted on an LCD display */
  UiDialogSelCback_t  selectCback;                /*!< Callback called when a dialog item is selected */
  uint8_t             numSelectItems;             /*!< Size of pSelectItems in number of strings */
  constStr            *pSelectItems;              /*!< Array of strings containing dialog select items */
} UiDialog_t;

/*! \brief Power supply mode. */
typedef enum
{
  UI_PS_MODE_BATT,                                /*!< Battery mode. */
  UI_PS_MODE_LINE,                                /*!< USB line mode. */
} UiPsMode_t;

/*! \brief Display Splash Screen action function */
typedef void (*UiDispSplash_t)(const UiSplashScreen_t *pSplash);

/*! \brief Display Splash Screen action function */
typedef void (*UiDispMenu_t)(const UiMenu_t *pMenu);

/*! \brief Display Splash Screen action function */
typedef void (*UiDispDialog_t)(const UiDialog_t *pDialog);

/*! \brief Process key press from the user */
typedef void (*UiKeyPress_t)(uint8_t input);

/*! \brief Table of display action functions */
typedef struct
{
  UiDispSplash_t     displaySplash;               /*!< Display splash screen action function */
  UiDispMenu_t       displayMenu;                 /*!< Display menu screen action function */
  UiDispDialog_t     displayDialog;               /*!< Display dialog screen action function */
  UiKeyPress_t       keyPress;                    /*!< Process key input from user action function */
} UiActionTbl_t;

/*! \brief UI Control Block */
typedef struct
{
  wsfHandlerId_t     handlerId;                   /*!< UI task handler identifier */
  UiActionTbl_t      uiActionTbl[UI_DISPLAY_MAX]; /*!< Action function table */
  const UiMenu_t     *pMainMenu;                  /*!< Main menu */
  const UiBase_t     *pActiveScreen;              /*!< Pointer to the active screen */
  UiScreenType_t     activeScreenType;            /*!< Type of active screen (splash, menu, or dialog) */
  UiDialogSelCback_t durationDlgCback;            /*!< Callback called when the duration dialog closes */
  UiDialogSelCback_t messageDlgCback;             /*!< Callback called when the message dialog closes */
  UiScrollCback_t    scrollCback;                 /*!< Callback called on scroll timer timeout */
} UiCb_t;

/*! \brief External declaration of display control block */
extern UiCb_t UiCb;

/**************************************************************************************************
  Function API
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief      Display a menu on the active UI
 *
 *  \param      pMenu         Pointer to the menu object to display.
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiLoadMenu(const UiMenu_t *pMenu);

/*************************************************************************************************/
/*!
 *  \brief      Display a dialog on the active UI
 *
 *  \param      pDialog       Pointer to the dialog object to display.
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiLoadDialog(const UiDialog_t *pDialog);

/*************************************************************************************************/
/*!
 *  \brief      Start the scroll timer
 *
 *  \param      scrollCback     Callback called on timer timeout
 *  \param      ms              Period between timeout in milliseconds
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiScrollTimerStart(const UiScrollCback_t scrollCback, uint16_t ms);

/*************************************************************************************************/
/*!
 *  \brief      Stop the scroll timer
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiScrollTimerStop(void);

/*************************************************************************************************/
/*!
 *  \brief      Called when the user selects an item in a menu or dialog
 *
 *  \param      selection     Number of  menu or dialog item selected by user {1 ... numSelectItems}
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiSelection(uint8_t selection);

/*************************************************************************************************/
/*!
 *  \brief      Refresh the active menu or dialog
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiRefresh(void);

/*************************************************************************************************/
/*!
 *  \brief   Copy a string and return a pointer to the end of the new string.
 *
 *  \param   pBuf         String buffer.
 *  \param   pStr         String to copy.
 *
 *  \return  Length of the formatted string.
 */
/*************************************************************************************************/
uint8_t UiCpyStr(char *pBuf, char *pStr);

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
char *UiFormatValue(char *pBuf, uint32_t value, uint8_t format);

/*************************************************************************************************/
/*!
 *  \brief   Append a formatted array of values to a string.
 *
 *  \param   pBuf          String buffer.
 *  \param   pValue        Pointer to array to format.
 *  \param   format        Type of formatting to use.
 *
 *  \return  Pointer to the end of the formatted string.
 */
/*************************************************************************************************/
char *UiFormatArray(char *pBuf, uint8_t *pValue, uint8_t format);

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
void UiInit(const UiSplashScreen_t *pSplash, const UiMenu_t *pMenu);

/*************************************************************************************************/
/*!
 *  \brief      Register a display.
 *
 *  \param      actionTbl     Table of action functions.
 *  \param      id            Display identifier.
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiRegisterDisplay(UiActionTbl_t actionTbl, uint8_t id);

/*************************************************************************************************/
/*!
 *  \brief      Initialize the Console Display
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiConsoleInit(void);

/*************************************************************************************************/
/*!
 *  \brief      Initialize the LCD Display
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiLcdInit(void);

/*************************************************************************************************/
/*!
 *  \brief      Display a dialog that shows the amount of time the application has been running
 *              and a selectable dialog item to exit the application.
 *
 *  \param      pTitle    Title for the dialog
 *  \param      pParent   Parent menu to load after running dialog exits
 *  \param      cback     Callback called when dialog exits
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiLoadDurationDialog(const char *pTitle, void *pParent, UiDialogSelCback_t cback);

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
void UiLoadMessageDialog(const char *pTitle, const char *pMsg, uint8_t pExitBtn,  void *pParent, UiDialogSelCback_t cback);

/*************************************************************************************************/
/*!
 *  \brief      Called to notify the UI subsystem of an event
 *
 *  \param      event    UI Event.
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiProcEvent(uint8_t event);

/*************************************************************************************************/
/*!
 *  \brief      Write a line on the LCD
 *
 *  \param      line         Line number
 *  \param      pLine        String with text to write to LCD.
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiLcdWriteLine(uint8_t line, const char *pLine);

/*************************************************************************************************/
/*!
 *  \brief      Flush the contents of the LCD buffer to the display
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiLcdFlush(void);

/*************************************************************************************************/
/*!
*  \brief  Set flag UiDataPrepared to TRUE
*
*  \return None
*/
/*************************************************************************************************/
void UiLcdSetDataPrepared(void);

/*************************************************************************************************/
/*!
 *  \brief      Print a string to the console
 *
 *  \param      pLine        String with text to print
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiConsolePrint(const char *pLine);

/*************************************************************************************************/
/*!
 *  \brief      Print a string to the console followed by a new line
 *
 *  \param      pLine        String with text to print
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiConsolePrintLn(const char *pLine);

/*************************************************************************************************/
/*!
 *  \brief      Flush the contents of the Console buffer to the display
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiConsoleFlush(void);

/*************************************************************************************************/
/*!
 *  \brief      Start the time in milliseconds until a UI Timer expires.
 *
 *  \param      event   Event to pass to UiProcEvent on timer expiration.
 *  \param      ms      Time in milliseconds until timer expiration.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void UiTimerStart(uint8_t event, uint32_t ms);

/*************************************************************************************************/
/*!
 *  \brief      Stop a UI timer.
 *
 *  \param      event   UI timer event.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void UiTimerStop(uint8_t event);

/*************************************************************************************************/
/*!
 *  \brief      Initialize the UI Timer subsystem.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void UiTimerInit(void);

/*************************************************************************************************/
/*!
 *  \brief      Called to process user input.
 *
 *  \param      input   Input from user
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiProcessUserInput(uint8_t input);

/*************************************************************************************************/
/*!
 *  \brief      Called to process user input from a PC keyboard.
 *
 *  \param      ch      input
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiProcessKeyboardInput(uint8_t ch);

/*************************************************************************************************/
/*!
 *  \brief  Register a callback function to receive UI events
 *
 *  \param  cback     Callback function to receive events
 *
 *  \return None
 */
/*************************************************************************************************/
void UiRegisterAppEvtCback(UiProcEvent_t cback);

/*************************************************************************************************/
/*!
 *  \brief      Initialize the main menu
 *
 *  \return     None.
 */
/*************************************************************************************************/
void MenuMainInit(void);

/*************************************************************************************************/
/*!
 *  \brief  Initialize About Menu.
 *
 *  \return None.
 */
/*************************************************************************************************/
void MenuAboutInit(void);

/*************************************************************************************************/
/*!
*  \brief  Get platform boot strap state.
*
*  \param  pin    GPIO pin.
*
*  \return Enable automation or not.
*/
/*************************************************************************************************/
bool_t UiPlatformGetBootStrapConfig(uint8_t pin);

/*************************************************************************************************/
/*!
*  \brief  Get platform power supply mode.
*
*  \return Power supply mode.
*/
/*************************************************************************************************/
UiPsMode_t UiPlatformGetPowerSupplyMode(void);

/*************************************************************************************************/
/*!
 *  \brief  Get the number of milliseconds since the device was started
 *
 *  \return Time in ms
 */
/*************************************************************************************************/
uint32_t UiPlatformGetTime(void);

#ifdef __cplusplus
}
#endif

#endif /* UI_API_H */
