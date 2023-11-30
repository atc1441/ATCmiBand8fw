/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Application framework parameter database.
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
#ifndef APP_PARAM_H
#define APP_PARAM_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup APP_FRAMEWORK_PARAM_API
 *  \{ */

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/** \name App Parameter Database
 * Interface to read and write parameter data in a file system.
 */
/**@{*/

/*************************************************************************************************/
/*!
 *  \brief  Initialize the parameter database.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppParamInit(void);

/*************************************************************************************************/
/*!
 *  \brief  Clear the parameter database.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppParamClear(void);

/*************************************************************************************************/
/*!
 *  \brief  Write parameter value.
 *
 *  \param  id          Identifier.
 *  \param  valueLen    Value length in bytes.
 *  \param  pValue      Value data.
 *
 *  \return Number of bytes written.
 */
/*************************************************************************************************/
uint16_t AppParamWrite(uint16_t id, uint16_t valueLen, const uint8_t *pValue);

/*************************************************************************************************/
/*!
 *  \brief  Read parameter value.
 *
 *  \param  id          Identifier.
 *  \param  valueLen    Maximum value length in bytes.
 *  \param  pValue      Storage value data.
 *
 *  \return Number of bytes read.
 */
/*************************************************************************************************/
uint16_t AppParamRead(uint16_t id, uint16_t valueLen, uint8_t *pValue);

/**@}*/

/*! \} */    /*! APP_FRAMEWORK_PARAM_API */

#ifdef __cplusplus
};
#endif

#endif /* APP_PARAM_H */
