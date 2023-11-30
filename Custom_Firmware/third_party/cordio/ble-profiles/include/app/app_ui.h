/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Application framework user interface.
 *
 *  Copyright (c) 2011-2019 Arm Ltd. All Rights Reserved.
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
#ifndef APP_UI_H
#define APP_UI_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup APP_FRAMEWORK_UI_API
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief UI event enumeration  */
enum
{
  APP_UI_NONE,                            /*!< \brief No event */
  APP_UI_RESET_CMPL,                      /*!< \brief Reset complete */
  APP_UI_DISCOVERABLE,                    /*!< \brief Enter discoverable mode */
  APP_UI_ADV_START,                       /*!< \brief Advertising started */
  APP_UI_ADV_STOP,                        /*!< \brief Advertising stopped */
  APP_UI_SCAN_START,                      /*!< \brief Scanning started */
  APP_UI_SCAN_STOP,                       /*!< \brief Scanning stopped */
  APP_UI_SCAN_REPORT,                     /*!< \brief Scan data received from peer device */
  APP_UI_CONN_OPEN,                       /*!< \brief Connection opened */
  APP_UI_CONN_CLOSE,                      /*!< \brief Connection closed */
  APP_UI_SEC_PAIR_CMPL,                   /*!< \brief Pairing completed successfully */
  APP_UI_SEC_PAIR_FAIL,                   /*!< \brief Pairing failed or other security failure */
  APP_UI_SEC_ENCRYPT,                     /*!< \brief Connection encrypted */
  APP_UI_SEC_ENCRYPT_FAIL,                /*!< \brief Encryption failed */
  APP_UI_PASSKEY_PROMPT,                  /*!< \brief Prompt user to enter passkey */
  APP_UI_DISPLAY_PASSKEY,                 /*!< \brief Display passkey */
  APP_UI_DISPLAY_CONFIRM,                 /*!< \brief Display confirm value */
  APP_UI_DISPLAY_RSSI,                    /*!< \brief Display rssi */
  APP_UI_ALERT_CANCEL,                    /*!< \brief Cancel a low or high alert */
  APP_UI_ALERT_LOW,                       /*!< \brief Low alert */
  APP_UI_ALERT_HIGH,                      /*!< \brief High alert */
  APP_UI_ADV_SET_START_IND,               /*!< \brief Advertising set(s) started */
  APP_UI_ADV_SET_STOP_IND,                /*!< \brief Advertising set(s) stopped */
  APP_UI_SCAN_REQ_RCVD_IND,               /*!< \brief Scan request received */
  APP_UI_EXT_SCAN_START_IND,              /*!< \brief Extended scanning started */
  APP_UI_EXT_SCAN_STOP_IND,               /*!< \brief Extended scanning stopped */
  APP_UI_PER_ADV_SET_START_IND,           /*!< \brief Periodic advertising set started */
  APP_UI_PER_ADV_SET_STOP_IND,            /*!< \brief Periodic advertising set stopped */
  APP_UI_PER_ADV_SYNC_EST_IND,            /*!< \brief Periodic advertising sync established */
  APP_UI_PER_ADV_SYNC_LOST_IND,           /*!< \brief Periodic advertising sync lost */
  APP_UI_HW_ERROR                         /*!< \brief Hardware error */
};

/*! \brief Button press enumeration */
enum
{
  APP_UI_BTN_NONE,                        /*!< \brief No button press */
  APP_UI_BTN_1_DOWN,                      /*!< \brief Button 1 on down press */
  APP_UI_BTN_1_SHORT,                     /*!< \brief Button 1 short press */
  APP_UI_BTN_1_MED,                       /*!< \brief Button 1 medium press */
  APP_UI_BTN_1_LONG,                      /*!< \brief Button 1 long press */
  APP_UI_BTN_1_EX_LONG,                   /*!< \brief Button 1 extra long press */
  APP_UI_BTN_2_DOWN,                      /*!< \brief Button 2 on down press */
  APP_UI_BTN_2_SHORT,                     /*!< \brief Button 2 short press */
  APP_UI_BTN_2_MED,                       /*!< \brief Button 2 medium press */
  APP_UI_BTN_2_LONG,                      /*!< \brief Button 2 long press */
  APP_UI_BTN_2_EX_LONG                    /*!< \brief Button 2 extra long press */
};

/*! \brief LED values */
#define APP_UI_LED_NONE     0x00          /*!< \brief No LED */
#define APP_UI_LED_1        0x01          /*!< \brief LED 1 */
#define APP_UI_LED_2        0x02          /*!< \brief LED 2 */
#define APP_UI_LED_3        0x04          /*!< \brief LED 3 */
#define APP_UI_LED_4        0x08          /*!< \brief LED 4 */
#define APP_UI_LED_WRAP     0xFF          /*!< \brief Wrap to beginning of sequence */

/*! \brief Sound tone value for wrap/repeat */
#define APP_UI_SOUND_WRAP   0xFFFF

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! \brief Button press callback */
typedef void (*appUiBtnCback_t)(uint8_t btn);

/*! \brief Action event callback */
typedef void (*appUiActionCback_t)(uint8_t event, uint32_t param);

/*! \brief Button Poll callback */
typedef void (*appUiBtnPollCback_t)(void);

/*! \brief Print callback */
typedef void (*appUiPrintFunc_t)(const char *txt);

/*! \brief Sound data structure */
typedef struct
{
  uint16_t      tone;                     /*!< \brief Sound tone in Hz.  Use 0 for silence. */
  uint16_t      duration;                 /*!< \brief Sound duration in milliseconds */
} appUiSound_t;

/*! \brief LED data structure */
typedef struct
{
  uint8_t       led;                      /*!< \brief LED to control */
  uint8_t       state;                    /*!< \brief On or off */
  uint16_t      duration;                 /*!< \brief duration in milliseconds */
} appUiLed_t;

/*! \brief Callback structure */
typedef struct
{
  appUiBtnCback_t       btnCback;         /*!< \brief Called when button pressed */
  appUiActionCback_t    actionCback;      /*!< \brief Called when action event received */
  appUiBtnPollCback_t   btnPollCback;     /*!< \brief Called to poll button hardware */
} appUiCback_t;

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/** \name APP User Interface
 * Commands that may be sent via terminal to the application.
 */
/**@{*/

/*************************************************************************************************/
/*!
 *  \brief  Perform a user interface action based on the event value passed to the function.
 *
 *  \param  event   User interface event value.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppUiAction(uint8_t event);

/*************************************************************************************************/
/*!
 *  \brief  Display a passkey.
 *
 *  \param  passkey   Passkey to display.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppUiDisplayPasskey(uint32_t passkey);

/*************************************************************************************************/
/*!
*  \brief  Display a confirmation value.
*
*  \param  confirm    Confirm value to display.
*
*  \return None.
*/
/*************************************************************************************************/
void AppUiDisplayConfirmValue(uint32_t confirm);

/*************************************************************************************************/
/*!
 *  \brief  Display an RSSI value.
 *
 *  \param  rssi   Rssi value to display.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppUiDisplayRssi(int8_t rssi);

/*************************************************************************************************/
/*!
 *  \brief  Register a callback function to receive button presses.
 *
 *  \param  btnCback    Callback function.
 *
 *  \return None.
 *
 *  \note   Registered by application to receive button events
 */
/*************************************************************************************************/
void AppUiBtnRegister(appUiBtnCback_t btnCback);

/*************************************************************************************************/
/*!
 *  \brief  Register a callback function to receive action events.
 *
 *  \param  actionCback   Callback function.
 *
 *  \return None.
 *
 *  \note   Registered by platform
 */
/*************************************************************************************************/
void AppUiActionRegister(appUiActionCback_t actionCback);

/*************************************************************************************************/
/*!
 *  \brief  Register a callback function to receive APP_BTN_POLL_IND events.
 *
 *  \param  btnPollCback   Callback function.
 *
 *  \return None.
 *
 *  \note   Registered by platform
 */
/*************************************************************************************************/
void AppUiBtnPollRegister(appUiBtnPollCback_t btnPollCback);

/*************************************************************************************************/
/*!
 *  \brief  Handle a hardware button press.  This function is called to handle WSF
 *          event APP_BTN_DOWN_EVT.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppUiBtnPressed(void);

/*************************************************************************************************/
/*!
 *  \brief  Play a sound.
 *
 *  \param  pSound   Pointer to sound tone/duration array.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppUiSoundPlay(const appUiSound_t *pSound);

/*************************************************************************************************/
/*!
 *  \brief  Stop the sound that is currently playing.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppUiSoundStop(void);

/*************************************************************************************************/
/*!
 *  \brief  Start LED blinking.
 *
 *  \param  pLed   Pointer to LED data structure.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppUiLedStart(const appUiLed_t *pLed);

/*************************************************************************************************/
/*!
 *  \brief  Stop LED blinking.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppUiLedStop(void);

/*************************************************************************************************/
/*!
 *  \brief  Button test function-- for test purposes only.
 *
 *  \param  btn   button press
 *  \return None.
 */
/*************************************************************************************************/
void AppUiBtnTest(uint8_t btn);

/**@}*/

/*! \} */    /*! APP_FRAMEWORK_UI_API */

#ifdef __cplusplus
};
#endif

#endif /* APP_UI_H */
