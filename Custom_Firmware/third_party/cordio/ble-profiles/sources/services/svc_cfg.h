/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Service configuration.
 *
 *  Copyright (c) 2011-2018 Arm Ltd. All Rights Reserved.
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
#ifndef SVC_CFG_H
#define SVC_CFG_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup SERVICE_CONFIGURATION
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief Default read security permissions for service characteristics */
#ifndef SVC_SEC_PERMIT_READ
#define SVC_SEC_PERMIT_READ ATTS_PERMIT_READ
#endif

/*! \brief Default write security permissions for service characteristics */
#ifndef SVC_SEC_PERMIT_WRITE
#define SVC_SEC_PERMIT_WRITE ATTS_PERMIT_WRITE
#endif

/*! \} */    /* SERVICE_CONFIGURATION */

#ifdef __cplusplus
};
#endif

#endif /* SVC_CFG_H */
