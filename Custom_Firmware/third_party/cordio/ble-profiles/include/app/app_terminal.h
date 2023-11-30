/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief      App Terminal handler.
 *
 *  Copyright (c) 2015-2018 Arm Ltd. All Rights Reserved.
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

#ifndef APP_TERMINAL_H
#define APP_TERMINAL_H

/*! \addtogroup APP_FRAMEWORK_UI_API
 *  \{ */

/**************************************************************************************************
  Function Prototypes
**************************************************************************************************/

/** \name APP Terminal Functions
 * Open a terminal interface to the application.
 */
/**@{*/

/*************************************************************************************************/
/*!
 *  \brief  Initialize terminal.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppTerminalInit(void);

/**@}*/

/*! \} */    /*! APP_FRAMEWORK_UI_API */

#endif /* APP_TERMINAL_H */
