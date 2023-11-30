/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Application framework configuration.
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
#ifndef APP_CFG_H
#define APP_CFG_H


#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \addtogroup APP_FRAMEWORK_DB_API
 *  \{ */

/** \name App Configuration
 * Build-time configuration constants
 */
/**@{*/

/*! \brief Number of application database device records */
#ifndef APP_DB_NUM_RECS
#define APP_DB_NUM_RECS 3
#endif

/*! \brief Number of client characteristic configuration descriptor handles per record */
#ifndef APP_DB_NUM_CCCD
#define APP_DB_NUM_CCCD 10
#endif

/*! \brief Number of ATT client cached handles per record */
#ifndef APP_DB_HDL_LIST_LEN
#define APP_DB_HDL_LIST_LEN 21
#endif

/*! \} */    /* APP_FRAMEWORK_DB_API */

/*! \addtogroup APP_FRAMEWORK_API
 * \{ */

/*! \brief Number of scan results to store (used only when operating as master) */
#ifndef APP_SCAN_RESULT_MAX
#define APP_SCAN_RESULT_MAX 10
#endif

/**@}*/

/*! \} */    /*! APP_FRAMEWORK_API */

#ifdef __cplusplus
};
#endif

#endif /* APP_CFG_H */
