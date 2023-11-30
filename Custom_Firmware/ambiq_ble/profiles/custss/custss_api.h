//*****************************************************************************
//
//  custss_api.h
//! @file
//!
//! @brief Customized Serice Server API header file
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

#ifndef CUSTSS_API_H
#define CUSTSS_API_H

#include "wsf_timer.h"
#include "att_api.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************

//*****************************************************************************
//
// function definitions
//
//*****************************************************************************


//*****************************************************************************
//
//! @fn     CustssReadCback
//!
//! @brief  Attribute read callback function.
//!
//! @param  connId       DM connection ID.
//! @param  handler      Attribute handle.
//! @param  operation    Attribute operation.
//! @param  offset       Attribute offset.
//! @param  pAttr        Pointer to attribute
//!
//! @return none
//
//*****************************************************************************
static uint8_t CustssReadCback(dmConnId_t connId, uint16_t handle, \
                                    uint8_t operation, uint16_t offset, attsAttr_t *pAttr);

//*****************************************************************************
//
//! @fn     CustssWriteCback
//!
//! @brief  Attribute write callback function.
//!
//! @param  connId       DM connection ID.
//! @param  handler      Attribute handle.
//! @param  operation    Attribute operation.
//! @param  offset       Attribute offset.
//! @param  len          Datalen written to attribute.
//! @param  pValue       Pointer to data
//! @param  pAttr        Pointer to attribute
//!
//! @return none
//
//*****************************************************************************
static uint8_t CustssWriteCback(dmConnId_t connId, uint16_t handle, \
                                    uint8_t operation, uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr);

//*****************************************************************************
//
//! @fn     CustssScheduledActionOnTimesUp
//!
//! @brief  Start the scheduled action.
//!
//! @param  pMsg       Pointer to wsf message.
//!
//! @return none
//
//*****************************************************************************
static void CustssScheduledActionOnTimesUp(wsfMsgHdr_t *pMsg);

//*****************************************************************************
//
//! @fn     CustssScheduledActionStop
//!
//! @brief  Stop the scheduled action.
//!
//! @param  connId      DM connection ID.
//! @param  timerEvt    Event to be sent when the timer expires.
//! @param  msec        Time delay in milliseconds.
//!
//! @return none
//
//*****************************************************************************
void CustssScheduledActionStart(dmConnId_t connId, uint8_t timerEvt, wsfTimerTicks_t msec);

//*****************************************************************************
//
//! @fn     CustssScheduledActionStop
//!
//! @brief  Stop the scheduled action.
//!
//! @param  connId          DM connection ID.
//!
//! @return none
//
//*****************************************************************************
void CustssScheduledActionStop(dmConnId_t connId);

//*****************************************************************************
//
//! @fn     CustssInit
//!
//! @brief  Initialize the customized service server.
//!
//! @param  handlerId       Handler of application
//!
//! @return none
//
//*****************************************************************************
void CustssInit(wsfHandlerId_t handlerId);

//*****************************************************************************
//
//! @fn     CustssDeinit
//!
//! @brief  De-initialize the customized service server.
//!
//! @param  none
//!
//! @return none
//
//*****************************************************************************
void CustssDeinit(void);

//*****************************************************************************
//
//! @fn     CustssStart
//!
//! @brief  Start the customized service server.
//!
//! @param  none
//!
//! @return none
//
//*****************************************************************************
void CustssStart(void);

//*****************************************************************************
//
//! @fn     CustssStop
//!
//! @brief  Stop the customized service server.
//!
//! @param  none
//!
//! @return none
//
//*****************************************************************************
void CustssStop(void);

//*****************************************************************************
//
//! @fn     CustssSendNtf
//!
//! @brief  Send an attribute protocol Handle Value Notification.
//!
//! @param  connId       DM connection ID.
//! @param  idx          Index of attrribute.
//! @param  handle       Attribute handle.
//! @param  len          Length of data.
//! @param  pVal        Pointer to data.
//!
//! @return none
//
//*****************************************************************************
void CustssSendNtf(dmConnId_t connId, uint8_t idx, uint16_t handle, uint16_t len, uint8_t *pVal);

//*****************************************************************************
//
//! @fn     CustssSendInd
//!
//! @brief  Send an attribute protocol handle value indication.
//!
//! @param  connId       DM connection ID.
//! @param  idx          Index of attrribute.
//! @param  handle       Attribute handle.
//! @param  len          Length of data.
//! @param  pVal         Pointer to data.
//!
//! @return none
//
//*****************************************************************************
void CustssSendInd(dmConnId_t connId, uint8_t idx, uint16_t handle, uint16_t len, uint8_t *pVal);

//*****************************************************************************
//
//! @fn     CustssProcMsg
//!
//! @brief  Process wsf messages.
//!
//! @param  pMsg        Pointer to wsf message.
//!
//! @return none
//
//*****************************************************************************
void CustssProcMsg(wsfMsgHdr_t *pMsg);

#ifdef __cplusplus
}
#endif

#endif // CUSTSS_API_H
