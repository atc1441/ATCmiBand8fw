//*****************************************************************************
//
//  amsc_main.c
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

#include <string.h>
#include "wsf_types.h"
#include "wsf_assert.h"
#include "wsf_buf.h"
#include "bstream.h"
#include "app_api.h"
#include "amsc_api.h"
#include "am_util.h"
#include "wsf_trace.h"
#include "app_ui.h"

#define AMS_ENTITY_UPDATE_BYTEOFFSET_ENTITYID               (0)
#define AMS_ENTITY_UPDATE_BYTEOFFSET_ATTRIBUTEID            (1)
#define AMS_ENTITY_UPDATE_BYTEOFFSET_ENTITYUPDATEFLAGS      (2)
#define AMS_ENTITY_UPDATE_BYTEOFFSET_VALUE                  (3)

#define AMS_SUPPORT_CMD_NUM            8
typedef struct amsc_ctrlblock   amsc_ctrlblock_t;
typedef struct ams_id_meta      ams_id_meta_t;

struct amsc_ctrlblock
{
    dmConnId_t          connId;
    uint8_t             attrSubscriptionEvt;
    uint16_t*           phdlList;
    uint32_t            supportedCmds[AMS_SUPPORT_CMD_NUM];
    uint8_t             currTruncatedEntityID;
    uint8_t             currTruncatedAttrID;
    amscAttrSubsRegCback_t attSubRegCback;
    uint8_t            amsSubsIdx;
    amsStatus          remoteDevSupportAms;
};

struct ams_id_meta
{
    uint8_t         id;
    uint8_t         updateFlag;
    char            *ptext;
    ams_id_meta_t   *pattrMeta;
    uint8_t         attrCnt;
};

/**************************************************************************************************
  Local Variables
**************************************************************************************************/
static amsc_ctrlblock_t s_amscCb;

static const uint8_t amscAmsSvcUuid[]               = {ATT_UUID_AMS_SERVICE};
static const uint8_t amscRemoteCommandChUuid[]      = {ATT_UUID_AMS_REMOTE_COMMAND};
static const uint8_t amscEntityUpdateChUuid[]       = {ATT_UUID_AMS_ENTITY_UPDATE};
static const uint8_t amscEntityAttributeChUuid[]    = {ATT_UUID_AMS_ENTITY_ATTRIBUTE};

static const attcDiscChar_t amscRemoteCommandCh =
{
    amscRemoteCommandChUuid,
    ATTC_SET_REQUIRED | ATTC_SET_UUID_128,
};

static const attcDiscChar_t amscRemoteCommandCcc =
{
    attCliChCfgUuid,
    ATTC_SET_REQUIRED | ATTC_SET_DESCRIPTOR,
};

static const attcDiscChar_t amscEntityUpdateCh =
{
    amscEntityUpdateChUuid,
    ATTC_SET_REQUIRED | ATTC_SET_UUID_128,
};

static const attcDiscChar_t amscEntityUpdateCcc =
{
    attCliChCfgUuid,
    ATTC_SET_REQUIRED | ATTC_SET_DESCRIPTOR,
};

static const attcDiscChar_t amscEntityAttributeCh =
{
    amscEntityAttributeChUuid,
    ATTC_SET_REQUIRED | ATTC_SET_UUID_128,
};

static const attcDiscChar_t *amscSvcDiscCharList[] =
{
    &amscRemoteCommandCh,
    &amscRemoteCommandCcc,
    &amscEntityUpdateCh,
    &amscEntityUpdateCcc,
    &amscEntityAttributeCh,
};

WSF_CT_ASSERT(AMSC_HDL_LIST_LEN == ((sizeof(amscSvcDiscCharList) / sizeof(attcDiscChar_t *))));

static ams_id_meta_t s_amsRemoteCommandMeta[] =
{
    [AMS_REMOTECOMMANDID_PLAY               ] = {.ptext = "Play",            .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_REMOTECOMMANDID_PAUSE              ] = {.ptext = "Pause",           .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_REMOTECOMMANDID_TOGGLEPLAYPAUSE    ] = {.ptext = "Play/Pause",      .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_REMOTECOMMANDID_NEXTTRACK          ] = {.ptext = "Next Track",      .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_REMOTECOMMANDID_PREVIOUSTRACK      ] = {.ptext = "Previous Track",  .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_REMOTECOMMANDID_VOLUMEUP           ] = {.ptext = "Volume Up",       .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_REMOTECOMMANDID_VOLUMEDOWN         ] = {.ptext = "Volume Down",     .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_REMOTECOMMANDID_ADVANCEREPEATMODE  ] = {.ptext = "Repeat Mode",     .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_REMOTECOMMANDID_ADVANCESHUFFLEMODE ] = {.ptext = "Shuffle Mode",    .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_REMOTECOMMANDID_SKIPFORWARD        ] = {.ptext = "Skip Forward",    .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_REMOTECOMMANDID_SKIPBACKWARD       ] = {.ptext = "Skip Backward",   .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_REMOTECOMMANDID_LIKETRACK          ] = {.ptext = "Like Track",      .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_REMOTECOMMANDID_DISLIKETRACK       ] = {.ptext = "Dislike Track",   .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_REMOTECOMMANDID_BOOKMARKTRACK      ] = {.ptext = "Bookmark Track",  .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_REMOTECOMMANDID_RESERVED           ] = {.ptext = "Unknown",         .pattrMeta = NULL,                  .attrCnt = 0x00, },
};
#define AMS_REMOTECOMMANDMETA_LEN (sizeof(s_amsRemoteCommandMeta)/sizeof(ams_id_meta_t))


static ams_id_meta_t s_amsPlayerAttrsMeta[] =
{
    [AMS_PLAYERATTRIBUTEID_NAME             ] = {.ptext = "Name",            .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_PLAYERATTRIBUTEID_PLAYBACKINFO     ] = {.ptext = "Playback Info",   .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_PLAYERATTRIBUTEID_VOLUME           ] = {.ptext = "Volume",          .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_PLAYERATTRIBUTEID_RESERVED         ] = {.ptext = "Unknown",         .pattrMeta = NULL,                  .attrCnt = 0x00, },
};
#define AMS_PLAYERATTRSMETA_LEN (sizeof(s_amsPlayerAttrsMeta)/sizeof(ams_id_meta_t))

static ams_id_meta_t s_amsQueueAttrsMeta[] =
{
    [AMS_QUEUEATTRIBUTEID_INDEX             ] = {.ptext = "Index",           .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_QUEUEATTRIBUTEID_COUNT             ] = {.ptext = "Count",           .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_QUEUEATTRIBUTEID_SHUFFLEMODE       ] = {.ptext = "Shuffle Mode",    .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_QUEUEATTRIBUTEID_REPEATMODE        ] = {.ptext = "Repeat Mode",     .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_QUEUEATTRIBUTEID_RESERVED          ] = {.ptext = "Unknown",         .pattrMeta = NULL,                  .attrCnt = 0x00, },
};
#define AMS_QUEUEATTRSMETA_LEN (sizeof(s_amsQueueAttrsMeta)/sizeof(ams_id_meta_t))

static ams_id_meta_t s_amsTrackAttrsMeta[] =
{
    [AMS_TRACKATTRIBUTEID_ARTIST            ] = {.ptext = "Artist",          .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_TRACKATTRIBUTEID_ALBUM             ] = {.ptext = "Album",           .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_TRACKATTRIBUTEID_TITLE             ] = {.ptext = "Title",           .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_TRACKATTRIBUTEID_DURATION          ] = {.ptext = "Duration",        .pattrMeta = NULL,                  .attrCnt = 0x00, },
    [AMS_TRACKATTRIBUTEID_RESERVED          ] = {.ptext = "Unknown",         .pattrMeta = NULL,                  .attrCnt = 0x00, },
};
#define AMS_TRACKATTRSMETA_LEN (sizeof(s_amsTrackAttrsMeta)/sizeof(ams_id_meta_t))

static ams_id_meta_t s_amsEntityMeta[] =
{
    [AMS_ENTITYID_PLAYER                    ] = {.ptext = "Player",          .pattrMeta = s_amsPlayerAttrsMeta,  .attrCnt = AMS_PLAYERATTRSMETA_LEN, },
    [AMS_ENTITYID_QUEUE                     ] = {.ptext = "Queue",           .pattrMeta = s_amsQueueAttrsMeta,   .attrCnt = AMS_QUEUEATTRSMETA_LEN,  },
    [AMS_ENTITYID_TRACK                     ] = {.ptext = "Track",           .pattrMeta = s_amsTrackAttrsMeta,   .attrCnt = AMS_TRACKATTRSMETA_LEN,  },
    [AMS_ENTITYID_RESERVED                  ] = {.ptext = "Unknown",         .pattrMeta = NULL,                  .attrCnt = 0x00, },
};
#define AMS_ENTITYMETA_LEN (sizeof(s_amsEntityMeta)/sizeof(ams_id_meta_t))


/*************************************************************************************************/
/*!
 *  \fn     AmscConnOpen
 *
 *  \brief  Start a delayed service discovery upon connection establishment.
 *
 *  \param  connId      Connection identifier.
 *  \param  phdlList    Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmscConnOpen(dmConnId_t connId, uint16_t* phdlList)
{
    s_amscCb.connId = connId;
    s_amscCb.phdlList = phdlList;
    s_amscCb.amsSubsIdx = AMS_ENTITYID_PLAYER;
    s_amscCb.currTruncatedAttrID = 0;
    s_amscCb.currTruncatedEntityID = 0;
    s_amscCb.remoteDevSupportAms = AMS_SUPPORT_INVALID;

}

/*************************************************************************************************/
/*!
 *  \fn     AmscConnClose
 *
 *  \brief  Clear the key parameters for control variables when connection close.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmscConnClose(void)
{
    s_amscCb.connId = DM_CONN_ID_NONE;
    memset(s_amscCb.supportedCmds, 0x00, sizeof(s_amscCb.supportedCmds));
}

/*************************************************************************************************/
/*!
 *  \fn     AmscDiscover
 *
 *  \brief  Perform service and characteristic discovery for ams service.
 *          Parameter phdlList must point to an array of length AMSC_HDL_LIST_LEN.
 *          If discovery is successful the handles of discovered characteristics and
 *          descriptors will be set in phdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  phdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmscDiscover(dmConnId_t connId, uint16_t *phdlList)
{
    AppDiscFindService(connId, ATT_128_UUID_LEN, (uint8_t *) amscAmsSvcUuid,    \
                       sizeof(amscSvcDiscCharList) / sizeof(attcDiscChar_t *),  \
                       (attcDiscChar_t **) amscSvcDiscCharList, phdlList);
}

/*************************************************************************************************/
/*!
 *  \fn     AmscIdToText
 *
 *  \brief  AMSC callback registration
 *
 *  \param  ptext   pointer to the text structure
 *
 *  \return None.
 */
/*************************************************************************************************/
static void AmscIdToText(ams_id_to_text_t *ptext)
{
    uint8_t *pvalueStr = NULL;

    pvalueStr = (uint8_t *)WsfBufAlloc(ptext->valueLen + 1);

    memcpy(pvalueStr, ptext->pvalue, ptext->valueLen);

    *(pvalueStr + ptext->valueLen) = '\0';

    ptext->pvalue = pvalueStr;

    APP_TRACE_INFO3("*** %s %s: %s", ptext->pentity, ptext->pattr, ptext->pvalue);

    WsfBufFree(pvalueStr);
}

/*************************************************************************************************/
/*!
 *  \brief  Process a value received in an ATT read response, notification, or indication
 *          message.  Parameter pHdlList must point to an array of length AMSC_HDL_LIST_LEN.
 *          If the attribute handle of the message matches a handle in the handle list the value
 *          is processed, otherwise it is ignored.
 *
 *  \param  pHdlList  Characteristic handle list.
 *  \param  pMsg      ATT callback message.
 *
 *  \return ATT_SUCCESS if handle is found, ATT_ERR_NOT_FOUND otherwise.
 */
/*************************************************************************************************/
uint8_t AmscValueUpdate(uint16_t *pHdlList, attEvt_t *pMsg)
{
    uint8_t status = ATT_SUCCESS;

    if (pMsg->handle == pHdlList[AMSC_REMOTE_COMMAND_HDL_IDX])
    {
        AmscRegisterSupportedRemoteCommands(pMsg->pValue, pMsg->valueLen);
    }
    else if (pMsg->handle == pHdlList[AMSC_ENTITY_UPDATE_HDL_IDX])
    {
        if (AmscCheckEntityUpdateTruncated(pMsg->pValue, pMsg->valueLen))
        {
            // truncated. According to AMS spec, if wanna get more informaiton, write to Entity Attribute.
            AmscProcTruncatedEntityUpdate(pMsg->pValue, pMsg->valueLen);
        }
        else
        {
            ams_id_to_text_t text;
            AmscEntityUpdateToString(pMsg->pValue, pMsg->valueLen, &text);
            AmscIdToText(&text);
        }
    }
    else if (pMsg->handle == pHdlList[AMSC_ENTITY_ATTRIBUTE_HDL_IDX])
    {
        ams_id_to_text_t text;
        AmscEntityAttributeToString(pMsg->pValue, pMsg->valueLen, &text);
        AmscIdToText(&text);
    }
    else
    {
        status = ATT_ERR_NOT_FOUND;
    }

    return status;
}

/*************************************************************************************************/
/*!
 *  \brief  Process a response message of write command/write request.
 *          Parameter pHdlList must point to an array of length AMSC_HDL_LIST_LEN.
 *          If the attribute handle of the message matches a handle in the handle list the value
 *          is processed, otherwise it is ignored.
 *
 *  \param  pHdlList  Characteristic handle list.
 *  \param  pMsg      ATT callback message.
 *
 *  \return ATT_SUCCESS if handle is found, ATT_ERR_NOT_FOUND otherwise.
 */
/*************************************************************************************************/
uint8_t AmscAttcWriteRsp(uint16_t *phdlList, attEvt_t *pMsg)
{
    uint8_t status = ATT_SUCCESS;

    if ( pMsg->handle == phdlList[AMSC_ENTITY_ATTRIBUTE_HDL_IDX] )
    {
        AmscReadEntityAttribute();
    }
    else if ( pMsg->handle == phdlList[AMSC_ENTITY_UPDATE_HDL_IDX] )
    {
        if ( pMsg->hdr.status == ATT_SUCCESS )
        {
            AmscEnityUpdate((uint8_t)pMsg->hdr.param);
        }
    }
    else
    {
        status = ATT_ERR_NOT_FOUND;
    }

    return status;
}

/*************************************************************************************************/
/*!
 *  \fn     AmscRegisterSupportedRemoteCommands
 *
 *  \brief  Register the supported remote commands recevied from AMS Media Source.
 *
 *  \param  pvalue    pointer to the list of supported remote command
 *  \param  valueLen  length of the list
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmscRegisterSupportedRemoteCommands(uint8_t *pvalue, uint16_t valueLen)
{
    memset(s_amscCb.supportedCmds, 0x00, sizeof(s_amscCb.supportedCmds));

    for (uint16_t idx = 0x00; idx < valueLen; idx++)
    {
        uint8_t rCmd        = *(pvalue + idx);
        uint8_t byteoffset  = rCmd / (sizeof(uint32_t) * 8);
        uint8_t bitoffset   = rCmd % (sizeof(uint32_t) * 8);

        if (byteoffset < sizeof(s_amscCb.supportedCmds) / sizeof(s_amscCb.supportedCmds[0]))
        {
            s_amscCb.supportedCmds[byteoffset] |= 1 << bitoffset;
        }
    }
}

/*************************************************************************************************/
/*!
 *  \fn     AmscCheckSupportedRemoteCommand
 *
 *  \brief  Check if a remote command is supported by AMS media source.
 *
 *  \param  rCmd        remote command to check
 *
 *  \return bool        true if the remote command is supported
 */
/*************************************************************************************************/
bool AmscCheckSupportedRemoteCommand(uint8_t rCmd)
{
    uint8_t byteoffset  = rCmd / (sizeof(uint32_t) * 8);
    uint8_t bitoffset   = rCmd % (sizeof(uint32_t) * 8);

    if (byteoffset < sizeof(s_amscCb.supportedCmds) / sizeof(s_amscCb.supportedCmds[0]))
    {
        return s_amscCb.supportedCmds[byteoffset] & (1 << bitoffset) ? true : false;
    }
    else
    {
        return false;
    }
}

/*************************************************************************************************/
/*!
 *  \fn     AmscSendRemoteCommand
 *
 *  \brief  Check if a remote command is supported by AMS Media Source.
 *
 *  \param  rCmd        remote command to check
 *
 *  \return bool        true if the remote command is supported
 */
/*************************************************************************************************/
bool AmscSendRemoteCommand(uint8_t rCmd)
{
    if ((rCmd < AMS_REMOTECOMMANDID_RESERVED) && AmscCheckSupportedRemoteCommand(rCmd))
    {
        AttcWriteReq(s_amscCb.connId, s_amscCb.phdlList[AMSC_REMOTE_COMMAND_HDL_IDX], sizeof(uint8_t), (uint8_t*)&rCmd);
        return true;
    }
    return false;
}

/*************************************************************************************************/
/*!
 *  \fn     AmscRemoteCommandToString
 *
 *  \brief  Get the description of a given remote command.
 *
 *  \param  rCmd        remote command to be looked up
 *
 *  \return char*       a string describing the given remote command
 */
/*************************************************************************************************/
void AmscRemoteCommandToString(uint8_t rCmd, ams_id_to_text_t *ptext)
{
    if (rCmd < AMS_REMOTECOMMANDMETA_LEN)
    {
        ptext->pvalue = (uint8_t *)s_amsRemoteCommandMeta[rCmd].ptext;
    }
    else
    {
        ptext->pvalue = (uint8_t *)s_amsRemoteCommandMeta[AMS_REMOTECOMMANDMETA_LEN - 1].ptext;
    }
}


/*************************************************************************************************/
/*!
 *  \fn     AmscCheckEntityUpdateTruncated
 *
 *  \brief  Check if the value of the Entity Update notification is truncated.
 *
 *  \param  pvalue      pointer to the value.
 *  \param  valueLen    length of the value.
 *
 *  \return true if truncated.
 */
/*************************************************************************************************/
bool AmscCheckEntityUpdateTruncated(uint8_t *pvalue, uint16_t valueLen)
{
    if (*(pvalue + AMS_ENTITY_UPDATE_BYTEOFFSET_ENTITYUPDATEFLAGS) & AMS_ENTITYUPDATEFLAG_TRUNCATED)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*************************************************************************************************/
/*!
 *  \fn     AmscGetEntityMeta
 *
 *  \brief  Get the meta data of a given entity.
 *
 *  \param  entityId    entity ID.
 *
 *  \return pointer to the meta data structure of a given entity.
 */
/*************************************************************************************************/
ams_id_meta_t *AmscGetEntityMeta(uint8_t entityId)
{
    if (entityId < AMS_ENTITYMETA_LEN)
    {
        return &s_amsEntityMeta[entityId];
    }
    else
    {
        return &s_amsEntityMeta[AMS_ENTITYMETA_LEN - 1];
    }
}

/*************************************************************************************************/
/*!
ams_id_meta_t *AmscGetAttributeMeta(uint8_t attrId, ams_id_meta_t *pentityMeta)
 *  \fn     AmscGetEntityMeta
 *
 *  \brief  Get the meta data of a given entity.
 *
 *  \param  attrId      attribute ID.
 *  \param  pentityMeta pointer to the meta data structure to be looked for.
 *
 *  \return pointer to the meta data structure of a given attribute.
 */
/*************************************************************************************************/
ams_id_meta_t *AmscGetAttributeMeta(uint8_t attrId, ams_id_meta_t *pentityMeta)
{
    ams_id_meta_t *pattrMeta = pentityMeta->pattrMeta;

    if (attrId < pentityMeta->attrCnt)
    {
        return (pattrMeta + attrId);
    }
    else
    {
        return (pattrMeta + pentityMeta->attrCnt - 1);
    }
}

/*************************************************************************************************/
/*!
 *  \fn     AmscProcTruncatedEntityUpdate
 *
 *  \brief  Process a truncated Entity Update notification.
 *
 *  \param  pvalue      pointer to the value.
 *  \param  valueLen    length of the value.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmscProcTruncatedEntityUpdate(uint8_t *pvalue, uint16_t valueLen)
{
    uint8_t entityId            = AmscGetEnityIdFromEntityUpdate(pvalue, valueLen);
    uint8_t attrId              = AmscGetAttributeIdFromEntityUpdate(pvalue, valueLen);
    //ams_id_meta_t *pentityMeta  = AmscGetEntityMeta(entityId);
    //ams_id_meta_t *pattrMeta    = AmscGetAttributeMeta(attrId, pentityMeta);

    s_amscCb.currTruncatedEntityID  = entityId;
    s_amscCb.currTruncatedAttrID    = attrId;

    AttcWriteCmd(s_amscCb.connId, s_amscCb.phdlList[AMSC_ENTITY_ATTRIBUTE_HDL_IDX], 2, pvalue);
}

/*************************************************************************************************/
/*!
 *  \fn     AmscReadEntityAttribute
 *
 *  \brief  Read the characteristic Entity Attribute.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmscReadEntityAttribute(void)
{
    AttcReadReq(s_amscCb.connId, s_amscCb.phdlList[AMSC_ENTITY_ATTRIBUTE_HDL_IDX]);
}

/*************************************************************************************************/
/*!
 *  \fn     AmscGetEnityIdFromEntityUpdate
 *
 *  \brief  Get entity id from an Entity Update notification.
 *
 *  \param  pvalue      pointer to the value.
 *  \param  valueLen    length of the value.
 *
 *  \return entity id.
 */
/*************************************************************************************************/
uint8_t AmscGetEnityIdFromEntityUpdate(uint8_t *pvalue, uint16_t valueLen)
{
    return *(pvalue + AMS_ENTITY_UPDATE_BYTEOFFSET_ENTITYID);
}

/*************************************************************************************************/
/*!
 *  \fn     AmscGetAttributeIdFromEntityUpdate
 *
 *  \brief  Get attribute id from an Entity Update notification.
 *
 *  \param  pvalue      pointer to the value.
 *  \param  valueLen    length of the value.
 *
 *  \return attribute id.
 */
/*************************************************************************************************/
uint8_t AmscGetAttributeIdFromEntityUpdate(uint8_t *pvalue, uint16_t valueLen)
{
    return *(pvalue + AMS_ENTITY_UPDATE_BYTEOFFSET_ATTRIBUTEID);
}

/*************************************************************************************************/
/*!
 *  \fn     AmscGetValueFromEntityUpdate
 *
 *  \brief  Get value from an Entity Update notification.
 *
 *  \param  pvalue      pointer to the value.
 *  \param  valueLen    length of the value.
 *
 *  \return pointer to the value.
 */
/*************************************************************************************************/
uint8_t* AmscGetValueFromEntityUpdate(uint8_t *pvalue, uint16_t valueLen)
{
    return pvalue + AMS_ENTITY_UPDATE_BYTEOFFSET_VALUE;
}

/*************************************************************************************************/
/*!
 *  \fn     AmscEntityUpdateToString
 *
 *  \brief  Get the description of a given entity update.
 *
 *  \param  pvalue      pointer to the value.
 *  \param  valueLen    length of the value.
 *  \param  ptext       pointer to the text table to store descriptions.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmscEntityUpdateToString(uint8_t *pvalue, uint16_t valueLen, ams_id_to_text_t *ptext)
{
    uint8_t entityId            = AmscGetEnityIdFromEntityUpdate(pvalue, valueLen);
    uint8_t attrId              = AmscGetAttributeIdFromEntityUpdate(pvalue, valueLen);
    ams_id_meta_t *pentityMeta  = AmscGetEntityMeta(entityId);
    ams_id_meta_t *pattrMeta    = AmscGetAttributeMeta(attrId, pentityMeta);

    ptext->pentity  = pentityMeta->ptext;
    ptext->pattr    = pattrMeta->ptext;
    ptext->pvalue   = AmscGetValueFromEntityUpdate(pvalue, valueLen);
    ptext->valueLen = valueLen - AMS_ENTITY_UPDATE_BYTEOFFSET_VALUE;
}

/*************************************************************************************************/
/*!
 *  \fn     AmscEntityAttributeToString
 *
 *  \brief  Get the description of a given attribute.
 *
 *  \param  pvalue      pointer to the value.
 *  \param  valueLen    length of the value.
 *  \param  ptext       pointer to the text table to store descriptions.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmscEntityAttributeToString(uint8_t *pvalue, uint16_t valueLen, ams_id_to_text_t *ptext)
{
    ams_id_meta_t *pentityMeta      = AmscGetEntityMeta(s_amscCb.currTruncatedEntityID);
    ams_id_meta_t *pattrMeta        = AmscGetAttributeMeta(s_amscCb.currTruncatedAttrID, pentityMeta);
    s_amscCb.currTruncatedEntityID  = 0xFF;
    s_amscCb.currTruncatedAttrID    = 0xFF;

    ptext->pentity  = pentityMeta->ptext;
    ptext->pattr    = pattrMeta->ptext;
    ptext->pvalue   = pvalue;
    ptext->valueLen = valueLen;
}

/*************************************************************************************************/
/*!
 *  \fn     AmscEnityUpdate
 *
 *  \brief  write request for entity update characteristic
 *
 *  \param  connId  connection ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmscEnityUpdate(uint8_t connId)
{
    uint8_t buf[AMS_MAX_ATT_DATA_LEN] = {0};
    uint8_t len = 0;

    APP_TRACE_INFO1("AmscEnityUpdate, entityID=%d", s_amscCb.amsSubsIdx);

    if (s_amscCb.amsSubsIdx < AMS_ENTITYID_RESERVED)
    {
        if ( s_amscCb.amsSubsIdx == AMS_ENTITYID_PLAYER )
        {
            buf[0] = AMS_ENTITYID_PLAYER;
            buf[1] = AMS_PLAYERATTRIBUTEID_NAME;
            buf[2] = AMS_PLAYERATTRIBUTEID_PLAYBACKINFO;
            buf[3] = AMS_PLAYERATTRIBUTEID_VOLUME;
            len = AMS_PLAYERATTRIBUTEID_MAX;
        }
        else if ( s_amscCb.amsSubsIdx == AMS_ENTITYID_QUEUE )
        {
            buf[0] = AMS_ENTITYID_QUEUE;
            buf[1] = AMS_QUEUEATTRIBUTEID_INDEX;
            buf[2] = AMS_QUEUEATTRIBUTEID_COUNT;
            buf[3] = AMS_QUEUEATTRIBUTEID_SHUFFLEMODE;
            buf[4] = AMS_QUEUEATTRIBUTEID_REPEATMODE;
            len = AMS_QUEUEATTRIBUTEID_MAX;
        }
        else if ( s_amscCb.amsSubsIdx == AMS_ENTITYID_TRACK )
        {
            buf[0] = AMS_ENTITYID_TRACK;
            buf[1] = AMS_TRACKATTRIBUTEID_ARTIST;
            buf[2] = AMS_TRACKATTRIBUTEID_ALBUM;
            buf[3] = AMS_TRACKATTRIBUTEID_TITLE;
            buf[4] = AMS_TRACKATTRIBUTEID_DURATION;
            len = AMS_TRACKATTRIBUTEID_MAX;
        }

        s_amscCb.amsSubsIdx++;
        APP_TRACE_INFO1("AmsEntityUpdate, len=%d", len);
        AttcWriteReq(connId, s_amscCb.phdlList[AMSC_ENTITY_UPDATE_HDL_IDX], len + 1, buf);
    }
}


/*************************************************************************************************/
/*!
 *  \fn     AmscOnBtnCback
 *
 *  \brief  AMS client button press callback.
 *
 *  \param  connId  connection ID.
 *  \param  btn     Button press.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmscOnBtnCback(dmConnId_t connId, uint8_t btn)
{
    static uint8_t s_currRemoteCmd = AMS_REMOTECOMMANDID_PLAY;
    ams_id_to_text_t Text;

    /* button actions when connected */
    if (DM_CONN_ID_NONE != connId)
    {
        switch (btn)
        {
            case APP_UI_BTN_1_DOWN:
            {
                s_currRemoteCmd++;
                if (s_currRemoteCmd == AMS_REMOTECOMMANDID_RESERVED)
                {
                    s_currRemoteCmd = AMS_REMOTECOMMANDID_PLAY;
                }

                AmscRemoteCommandToString(s_currRemoteCmd, &Text);

                APP_TRACE_INFO2("*** Current Remote Command: %s (%s)", Text.pvalue,
                                AmscCheckSupportedRemoteCommand(s_currRemoteCmd) ? "Supported" : "Not supported");
            }
            break;
            case APP_UI_BTN_1_SHORT:
            {
                AmscRemoteCommandToString(s_currRemoteCmd, &Text);

                if (AmscSendRemoteCommand(s_currRemoteCmd))
                {
                    APP_TRACE_INFO1("*** << Remote Command: %s", Text.pvalue);
                }
                else
                {
                    APP_TRACE_INFO2("*** Remote Command: %s (%s)", Text.pvalue,
                                AmscCheckSupportedRemoteCommand(s_currRemoteCmd) ? "Supported" : "Not supported");
                }
            }
            break;
            default:
            break;
        }
    }
}

/*************************************************************************************************/
/*!
 *  \fn     AmscPeerDevSupportAms
 *
 *  \brief  Get peer device support AMS status
 *
 *  \param void
 *
 *  \return bool value to indicate if peer device support AMS or not.
 */
/*************************************************************************************************/
amsStatus AmscGetPeerDevAmsStatus(void)
{
    return s_amscCb.remoteDevSupportAms;
}

/*************************************************************************************************/
/*!
 *  \fn     AmscSetPeerDevAmsSupport
 *
 *  \brief  set peer device support AMS or not
 *
 *  \param  status  AMS support status
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmscSetPeerDevAmsStatus(amsStatus status)
{
    s_amscCb.remoteDevSupportAms = status;
}
