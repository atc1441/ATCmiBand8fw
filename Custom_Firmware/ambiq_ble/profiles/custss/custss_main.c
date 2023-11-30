//*****************************************************************************
//
//  custss_main.c
//! @file
//!
//! @brief This file provides the main application for Customized Serice Server service.
//!
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

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "wsf_types.h"
#include "wsf_assert.h"
#include "wsf_trace.h"
#include "wsf_buf.h"
#include "bstream.h"

#include "app_api.h"
#include "app_hw.h"

#include "att_api.h"

#include "svc_cust.h"
#include "custss_api.h"

#include "am_bsp.h"
#include "am_mcu_apollo.h"
#include "am_util.h"
#include "am_util_debug.h"


//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
#define CUSTSS_MAX_CONN             1

//*****************************************************************************
//
// Global variables
//
//*****************************************************************************

static struct
{
    dmConnId_t          connId[CUSTSS_MAX_CONN];
    wsfHandlerId_t      handlerId;
    wsfTimer_t          timer;
    wsfTimerTicks_t     timerPeriod;
} s_CtrlBlock;


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
                               uint8_t operation, uint16_t offset, attsAttr_t *pAttr)
{
    static uint8_t s_readcnt = 0x00;

    APP_TRACE_INFO1("[%s]", __func__);
    APP_TRACE_INFO1("connId           = 0x%02X", connId);
    APP_TRACE_INFO1("handle           = 0x%02X", handle);
    APP_TRACE_INFO1("operation        = 0x%02X", operation);
    APP_TRACE_INFO1("offset           = 0x%04X", offset);

    APP_TRACE_INFO1("attsAttr_t.pLen  = 0x%04X", *pAttr->pLen);

    // fill in fake data
    memset(pAttr->pValue, s_readcnt++, *pAttr->pLen);

    return ATT_SUCCESS;
}

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
                                uint8_t operation, uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr)
{
    APP_TRACE_INFO1("[%s]", __func__);
    APP_TRACE_INFO1("connId           = 0x%02X", connId);
    APP_TRACE_INFO1("handle           = 0x%02X", handle);
    APP_TRACE_INFO1("operation        = 0x%02X", operation);
    APP_TRACE_INFO1("offset           = 0x%04X", offset);
    APP_TRACE_INFO1("len              = 0x%04X", len);
    APP_TRACE_INFO0("data             = ");

    for (size_t idx = 0; idx < len; idx++)
    {
        APP_TRACE_INFO1("\t\t0x%02X", *(pValue + idx));
    }

    return ATT_SUCCESS;
}

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
static void CustssScheduledActionOnTimesUp(wsfMsgHdr_t *pMsg)
{
    // things to be done when the timer expires.
}

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
void CustssScheduledActionStart(dmConnId_t connId, uint8_t timerEvt, wsfTimerTicks_t msec)
{
    APP_TRACE_INFO1("[%s]", __func__);
    s_CtrlBlock.timer.handlerId = s_CtrlBlock.handlerId;
    s_CtrlBlock.timer.msg.event = timerEvt;
    s_CtrlBlock.timerPeriod     = msec;

    /* start timer */
    WsfTimerStartMs(&s_CtrlBlock.timer, s_CtrlBlock.timerPeriod);
}

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
void CustssScheduledActionStop(dmConnId_t connId)
{
    /* stop timer */
    WsfTimerStop(&s_CtrlBlock.timer);
}

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
void CustssInit(wsfHandlerId_t handlerId)
{
    s_CtrlBlock.handlerId = handlerId;
}

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
void CustssDeinit(void)
{
    s_CtrlBlock.handlerId = WSF_INVALID_TASK_ID;
}

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
void CustssStart(void)
{
    SvcCustCbackRegister(CustssReadCback, CustssWriteCback);
    SvcCustAddGroup();
}

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
void CustssStop(void)
{
    SvcCustRemoveGroup();
}

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
//! @param  pVal         Pointer to data.
//!
//! @return none
//
//*****************************************************************************
void CustssSendNtf(dmConnId_t connId, uint8_t idx, uint16_t handle, uint16_t len, uint8_t *pVal)
{
    if (AttsCccEnabled(connId, idx))
    {
        AttsHandleValueNtf(connId, handle, len, pVal);
    }
}

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
void CustssSendInd(dmConnId_t connId, uint8_t idx, uint16_t handle, uint16_t len, uint8_t *pVal)
{
    if (AttsCccEnabled(connId, idx))
    {
        AttsHandleValueInd(connId, handle, len, pVal);
    }
}

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
void CustssProcMsg(wsfMsgHdr_t *pMsg)
{
    if (pMsg->event == DM_CONN_OPEN_IND)
    {
        // things to be done when connected
    }
    else if (pMsg->event == DM_CONN_CLOSE_IND)
    {
        // things to be done when disconnected
    }
    else if (pMsg->event == ATTS_HANDLE_VALUE_CNF)
    {
        // things to be done when a notification/indication is sent
    }
    else if (pMsg->event == s_CtrlBlock.timer.msg.event)
    {
        CustssScheduledActionOnTimesUp(pMsg);
    }
    else
    {

    }
}
