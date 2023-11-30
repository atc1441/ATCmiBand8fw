//*****************************************************************************
//
//  amsc_api.h
//! @file
//!
//! @brief  apple media service client.
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

#ifndef AMSC_API_H
#define AMSC_API_H

#include <stdbool.h>
#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/
/* Apple Media Service */                   // 89D3502B-0F36-433A-8EF4-C502AD55F8DC
#define ATT_UUID_AMS_SERVICE                0xDC, 0xF8, 0x55, 0xAD, 0x02, 0xC5, 0xF4, 0x8E, \
                                            0x3A, 0x43, 0x36, 0x0F, 0x2B, 0x50, 0xD3, 0x89


// Remote Command: UUID 9B3C81D8-57B1-4A8A-B8DF-0E56F7CA51C2 (writeable, notifiable)

#define ATT_UUID_AMS_REMOTE_COMMAND         0xC2, 0x51, 0xCA, 0xF7, 0x56, 0x0E, 0xDF, 0xB8, \
                                            0x8A, 0x4A, 0xB1, 0x57, 0xD8, 0x81, 0x3C, 0x9B

// Entity Update: UUID 2F7CABCE-808D-411F-9A0C-BB92BA96C102 (writeable with response, notifiable)
#define ATT_UUID_AMS_ENTITY_UPDATE          0x02, 0xC1, 0x96, 0xBA, 0x92, 0xBB, 0x0C, 0x9A, \
                                            0x1F, 0x41, 0x8D, 0x80, 0xCE, 0xAB, 0x7C, 0x2F

// Entity Attribute: UUID C6B2F38C-23AB-46D8-A6AB-A3A870BBD5D7 (readable, writeable)
#define ATT_UUID_AMS_ENTITY_ATTRIBUTE       0xD7, 0xD5, 0xBB, 0x70, 0xA8, 0xA3, 0xAB, 0xA6, \
                                            0xD8, 0x46, 0xAB, 0x23, 0x8C, 0xF3, 0xB2, 0xC6

#define AMS_REMOTECOMMANDID_PLAY                    (0)
#define AMS_REMOTECOMMANDID_PAUSE                   (1)
#define AMS_REMOTECOMMANDID_TOGGLEPLAYPAUSE         (2)
#define AMS_REMOTECOMMANDID_NEXTTRACK               (3)
#define AMS_REMOTECOMMANDID_PREVIOUSTRACK           (4)
#define AMS_REMOTECOMMANDID_VOLUMEUP                (5)
#define AMS_REMOTECOMMANDID_VOLUMEDOWN              (6)
#define AMS_REMOTECOMMANDID_ADVANCEREPEATMODE       (7)
#define AMS_REMOTECOMMANDID_ADVANCESHUFFLEMODE      (8)
#define AMS_REMOTECOMMANDID_SKIPFORWARD             (9)
#define AMS_REMOTECOMMANDID_SKIPBACKWARD            (10)
#define AMS_REMOTECOMMANDID_LIKETRACK               (11)
#define AMS_REMOTECOMMANDID_DISLIKETRACK            (12)
#define AMS_REMOTECOMMANDID_BOOKMARKTRACK           (13)
#define AMS_REMOTECOMMANDID_RESERVED                (14)

typedef enum
{
    AMS_SUPPORT_INVALID = 0,
    AMS_SUPPORT_SUCCESS = 1,
    AMS_SUPPORT_FAIL    = 2
}amsStatus;

enum
{
    AMS_ENTITYID_PLAYER                             = 0,
    AMS_ENTITYID_QUEUE                              = 1,
    AMS_ENTITYID_TRACK                              = 2,
    AMS_ENTITYID_RESERVED                           = 3,
    AMS_ENTITYID_MAX                                = AMS_ENTITYID_RESERVED,
};

#define AMS_ENTITYUPDATEFLAG_TRUNCATED              (1 << 0)
#define AMS_ENTITYUPDATEFLAG_RESERVED               (1 << 1)

#define AMS_ATTRIBUTEID_MAX                         (4)

#define AMS_MAX_ATT_DATA_LEN                        (251)
enum
{
    AMS_PLAYERATTRIBUTEID_NAME                      = 0,
    AMS_PLAYERATTRIBUTEID_PLAYBACKINFO              = 1,
    AMS_PLAYERATTRIBUTEID_VOLUME                    = 2,
    AMS_PLAYERATTRIBUTEID_RESERVED                  = 3,
    AMS_PLAYERATTRIBUTEID_MAX                       = AMS_PLAYERATTRIBUTEID_RESERVED,
};
WSF_CT_ASSERT(AMS_PLAYERATTRIBUTEID_MAX <= AMS_ATTRIBUTEID_MAX);

enum
{
    AMS_QUEUEATTRIBUTEID_INDEX                      = 0,
    AMS_QUEUEATTRIBUTEID_COUNT                      = 1,
    AMS_QUEUEATTRIBUTEID_SHUFFLEMODE                = 2,
    AMS_QUEUEATTRIBUTEID_REPEATMODE                 = 3,
    AMS_QUEUEATTRIBUTEID_RESERVED                   = 4,
    AMS_QUEUEATTRIBUTEID_MAX                        = AMS_QUEUEATTRIBUTEID_RESERVED,
};
WSF_CT_ASSERT(AMS_QUEUEATTRIBUTEID_MAX <= AMS_ATTRIBUTEID_MAX);

enum
{
    AMS_TRACKATTRIBUTEID_ARTIST                     = 0,
    AMS_TRACKATTRIBUTEID_ALBUM                      = 1,
    AMS_TRACKATTRIBUTEID_TITLE                      = 2,
    AMS_TRACKATTRIBUTEID_DURATION                   = 3,
    AMS_TRACKATTRIBUTEID_RESERVED                   = 4,
    AMS_TRACKATTRIBUTEID_MAX                        = AMS_TRACKATTRIBUTEID_RESERVED,
};
WSF_CT_ASSERT(AMS_TRACKATTRIBUTEID_MAX <= AMS_ATTRIBUTEID_MAX);


/*! ams client enumeration of handle indexes of characteristics to be discovered */
enum
{
    AMSC_REMOTE_COMMAND_HDL_IDX,
    AMSC_REMOTE_COMMAND_CCC_HDL_IDX,
    AMSC_ENTITY_UPDATE_HDL_IDX,
    AMSC_ENTITY_UPDATE_CCC_HDL_IDX,
    AMSC_ENTITY_ATTRIBUTE_HDL_IDX,
    AMSC_HDL_LIST_LEN,
};

typedef struct ams_id_to_text
{
    char    *pentity;
    char    *pattr;
    uint8_t *pvalue;
    uint16_t valueLen;
} ams_id_to_text_t;

typedef enum ams_err_code
{
    AMS_ERR_INVALID_STATE                   = 0xA0,
    AMS_ERR_INVALID_COMMAND                 = 0xA1,
    AMS_ERR_ABSENT_ATTRIBUTE                = 0xA2,
} ams_err_code_e;

typedef enum ams_shufflemode
{
    AMS_SHUFFLEMODEOFF                      = 0,
    AMS_SHUFFLEMODEONE                      = 1,
    AMS_SHUFFLEMODEALL                      = 2,
    AMS_SHUFFLEMODERESERVED                 = 3,
} ams_shufflemode_e;

typedef enum ams_repeatmode
{
    AMS_REPEATMODEOFF                       = 0,
    AMS_REPEATMODEONE                       = 1,
    AMS_REPEATMODEALL                       = 2,
    AMS_REPEATMODERESERVED                  = 3,
} ams_repeatmode_e;


typedef struct amscCfg
{
    uint32_t reserved;
} amscCfg_t;

typedef struct amsc_subscription
{
    uint8_t     attrListLen;
    uint8_t     attrList[AMS_ATTRIBUTEID_MAX];
} amsc_subscription_t;

typedef void (*amscIttCback_t)(ams_id_to_text_t *ptext);
typedef void (*amscAttrSubsRegCback_t)(void);
typedef void (*amscDiscCfgCmplCback_t)(void);

typedef struct amsc_cback_fn
{
    amscDiscCfgCmplCback_t  dcc;    // Discovery configuration complete
    amscIttCback_t          itt;    // ID-to-text
    amscAttrSubsRegCback_t  asr;    // Attribute subscription registration
} amsc_cback_fn_t;


void        AmscConnOpen(dmConnId_t connId, uint16_t* pHdlList);
void        AmscConnClose(void);
void        AmscDiscover(dmConnId_t connId, uint16_t *pHdlList);
uint8_t     AmscValueUpdate(uint16_t *phdList, attEvt_t *pMsg);
uint8_t     AmscAttcWriteRsp(uint16_t *phdlList, attEvt_t *pMsg);

void        AmscRegisterSupportedRemoteCommands(uint8_t *pvalue, uint16_t valueLen);
bool        AmscCheckSupportedRemoteCommand(uint8_t rCmd);
bool        AmscSendRemoteCommand(uint8_t rCmd);

void        AmscRemoteCommandToString(uint8_t rCmd, ams_id_to_text_t *ptext);
void        AmscEntityUpdateToString(uint8_t *pvalue, uint16_t valueLen, ams_id_to_text_t *ptext);
void        AmscEntityAttributeToString(uint8_t *pvalue, uint16_t valueLen, ams_id_to_text_t *ptext);

void        AmscProcTruncatedEntityUpdate(uint8_t *pvalue, uint16_t valueLen);
void        AmscReadEntityAttribute(void);

bool        AmscCheckEntityUpdateTruncated(uint8_t *pvalue, uint16_t valueLen);
uint8_t     AmscGetEnityIdFromEntityUpdate(uint8_t *pvalue, uint16_t valueLen);
uint8_t     AmscGetAttributeIdFromEntityUpdate(uint8_t *pvalue, uint16_t valueLen);
uint8_t*    AmscGetValueFromEntityUpdate(uint8_t *pvalue, uint16_t valueLen);
void        AmscEnityUpdate(uint8_t connId);
amsStatus   AmscGetPeerDevAmsStatus(void);
void        AmscSetPeerDevAmsStatus(amsStatus status);
void        AmscOnBtnCback(dmConnId_t connId, uint8_t btn);

#ifdef __cplusplus
};
#endif

#endif /* AMSC_API_H */
