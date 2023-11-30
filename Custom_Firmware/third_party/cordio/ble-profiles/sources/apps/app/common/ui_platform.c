/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Interface between UI and platform drivers.
 *
 *  Copyright (c) 2016-2019 Arm Ltd. All Rights Reserved.
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
#include "wsf_trace.h"
#include "wsf_msg.h"
#include "wsf_bufio.h"

/**************************************************************************************************
  External Functions
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Print a string to the console.
 *
 *  \param  pLine        String with text to print
 *
 *  \return None
 */
/*************************************************************************************************/
void UiConsolePrint(const char *pLine)
{
  // WsfBufIoWrite((const uint8_t *) pLine, strlen(pLine));
}

/*************************************************************************************************/
/*!
 *  \brief  Print a string to the console followed by a new line.
 *
 *  \param  pLine        String with text to print
 *
 *  \return None
 */
/*************************************************************************************************/
void UiConsolePrintLn(const char *pLine)
{
  UiConsolePrint(pLine);
  UiConsolePrint("\r\n");
}

/*************************************************************************************************/
/*!
 *  \brief  Flush the contents of the Console buffer to the display.
 *
 *  \return None
 */
/*************************************************************************************************/
void UiConsoleFlush(void)
{
  /* Take no action. */
}

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
void UiLcdWriteLine(uint8_t line, const char *pLine)
{
  /* Take no action. */
}

/*************************************************************************************************/
/*!
 *  \brief      Flush the contents of the LCD buffer to the display
 *
 *  \return     None
 */
/*************************************************************************************************/
void UiLcdFlush(void)
{
  /* Take no action. */
}

/*************************************************************************************************/
/*!
*  \brief  Set flag UiDataPrepared to TRUE
*
*  \return None
*/
/*************************************************************************************************/
void UiLcdSetDataPrepared(void)
{
  /* Take no action. */
}
