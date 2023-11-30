//*****************************************************************************
//
//! @file svc_cust.h
//!
//! @brief Customerized Service Definition
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2023, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision release_sdk_4_4_1-7498c7b770 of the AmbiqSuite Development Package.
//
//*****************************************************************************
#ifndef SVC_CUST_H
#define SVC_CUST_H

//
// Put additional includes here if necessary.
//
#include "bstream.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
#define INCLUDE_USER_DESCR

/*! Base UUID:  00002760-08C2-11E1-9073-0E8AC72EXXXX */
#define ATT_UUID_AMBIQ_BASE             0x2E, 0xC7, 0x8A, 0x0E, 0x73, 0x90, \
                                            0xE1, 0x11, 0xC2, 0x08, 0x60, 0x27, 0x00, 0x00

/*! Macro for building Ambiq UUIDs */
#define ATT_UUID_AMBIQ_BUILD(part)      UINT16_TO_BYTES(part), ATT_UUID_AMBIQ_BASE

/* Partial Customized Service UUID */
#define ATT_UUID_CUST_SERVICE_PART          0x2000
#define ATT_UUID_CUST_SERVICE               ATT_UUID_AMBIQ_BUILD(ATT_UUID_CUST_SERVICE_PART)

/*! Partial WRITEONLY characteristic UUID */
#define ATT_UUID_CUSTS_WRITEONLY_PART       0x2010
#define ATT_UUID_CUSTS_WRITEONLY            ATT_UUID_AMBIQ_BUILD(ATT_UUID_CUSTS_WRITEONLY_PART)

/*! Partial READONLY characteristic UUID */
#define ATT_UUID_CUSTS_READONLY_PART        0x2011
#define ATT_UUID_CUSTS_READONLY             ATT_UUID_AMBIQ_BUILD(ATT_UUID_CUSTS_READONLY_PART)

/*! Partial NOTIFYONLY characteristic UUID */
#define ATT_UUID_CUSTS_NOTIFYONLY_PART      0x2012
#define ATT_UUID_CUSTS_NOTIFYONLY           ATT_UUID_AMBIQ_BUILD(ATT_UUID_CUSTS_NOTIFYONLY_PART)

/*! Partial NOTIFYONLY characteristic UUID */
#define ATT_UUID_CUSTS_INDICATEONLY_PART    0x2013
#define ATT_UUID_CUSTS_INDICATEONLY         ATT_UUID_AMBIQ_BUILD(ATT_UUID_CUSTS_INDICATEONLY_PART)

/* Customized Service UUID */

#define CUSTSVC_HANDLE_START                0x01A0
#define CUSTSVC_HANDLE_END                  (CUSTS_HANDLE_LAST - 1)

/* AMDTP Service Handles */
enum
{
    CUSTS_HANDLE_SVC                        = CUSTSVC_HANDLE_START,
    CUSTS_HANDLE_WRITEONLY_CH,
    CUSTS_HANDLE_WRITEONLY,
#ifdef INCLUDE_USER_DESCR
    CUSTS_HANDLE_WRITEONLY_USR_DESCR,
#endif
    CUSTS_HANDLE_READONLY_CH,
    CUSTS_HANDLE_READONLY,
#ifdef INCLUDE_USER_DESCR
    CUSTS_HANDLE_READONLY_USR_DESCR,
#endif
    CUSTS_HANDLE_NOTIFYONLY_CH,
    CUSTS_HANDLE_NOTIFYONLY,
    CUSTS_HANDLE_NOTIFYONLY_CCC,
#ifdef INCLUDE_USER_DESCR
    CUSTS_HANDLE_NOTIFYONLY_USR_DESCR,
#endif
    CUSTS_HANDLE_INDICATEONLY_CH,
    CUSTS_HANDLE_INDICATEONLY,
    CUSTS_HANDLE_INDICATEONLY_CCC,
#ifdef INCLUDE_USER_DESCR
    CUSTS_HANDLE_INDICATEONLY_USR_DESCR,
#endif
    CUSTS_HANDLE_LAST,

};


//*****************************************************************************
//
// External variable definitions
//
//*****************************************************************************

//*****************************************************************************
//
// Function definitions.
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Add the customized service to the attribute server.
//!
//! @param none
//!
//! This function adds the customized service to the attribute server.
//!
//! @return none
//
//*****************************************************************************
void SvcCustAddGroup(void);

//*****************************************************************************
//
//! @brief Remove the customized service from the attribute server.
//!
//! @param none
//!
//! This function removes the customized service to the attribute server.
//!
//! @return none
//
//*****************************************************************************
void SvcCustRemoveGroup(void);

//*****************************************************************************
//
//! @brief Register callbacks for the customized service.
//!
//! @param readCback    - attribute read callback function pointer.
//! @param writeCback   - attribute write callback function pointer.
//!
//! This function registers callback functions for the customized servcie..
//!
//! @return none
//
//*****************************************************************************
void SvcCustCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback);

#ifdef __cplusplus
}
#endif

#endif // SVC_CUST_H
