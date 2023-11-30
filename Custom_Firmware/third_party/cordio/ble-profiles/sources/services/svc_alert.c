/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Alert-related services implementation.
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

#include "wsf_types.h"
#include "att_api.h"
#include "wsf_trace.h"
#include "util/bstream.h"
#include "svc_alert.h"
#include "svc_cfg.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions */
#ifndef ALERT_SEC_PERMIT_READ
#define ALERT_SEC_PERMIT_READ SVC_SEC_PERMIT_READ
#endif

/*! Characteristic write permissions */
#ifndef ALERT_SEC_PERMIT_WRITE
#define ALERT_SEC_PERMIT_WRITE SVC_SEC_PERMIT_WRITE
#endif

/**************************************************************************************************
 Alert-Related Services group
**************************************************************************************************/

/*!
 * Alert notification service
 */

/* Alert notification declaration */
static const uint8_t alertAnsValSvc[] = {UINT16_TO_BYTES(ATT_UUID_ALERT_NOTIF_SERVICE)};
static const uint16_t alertAnsLenSvc = sizeof(alertAnsValSvc);

/* Supported new alert category characteristic */
static const uint8_t alertAnsValSnaCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(ALERT_ANS_SNA_HDL), UINT16_TO_BYTES(ATT_UUID_SUP_NEW_ALERT_CAT)};
static const uint16_t alertAnsLenSnaCh = sizeof(alertAnsValSnaCh);

/* Supported new alert category */
static const uint8_t alertAnsUuSna[] = {UINT16_TO_BYTES(ATT_UUID_SUP_NEW_ALERT_CAT)};
static uint8_t alertAnsValSna[] = {0xFF,0x03};
static uint16_t alertAnsLenSna = sizeof(alertAnsValSna);

/* New alert characteristic */
static const uint8_t alertAnsValNewCh[] = {ATT_PROP_NOTIFY, UINT16_TO_BYTES(ALERT_ANS_NEW_HDL), UINT16_TO_BYTES(ATT_UUID_NEW_ALERT)};
static const uint16_t alertAnsLenNewCh = sizeof(alertAnsValNewCh);

/* New alert */
static const uint8_t alertAnsUuNew[] = {UINT16_TO_BYTES(ATT_UUID_NEW_ALERT)};
static uint8_t alertAnsValNew[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static uint16_t alertAnsLenNew = sizeof(alertAnsValNew);

/* New alert client characteristic configuration */
static uint8_t alertAnsValNewChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t alertAnsLenNewChCcc = sizeof(alertAnsValNewChCcc);

/* Supported unread alert category characteristic */
static const uint8_t alertAnsValUnrCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(ALERT_ANS_UNR_HDL), UINT16_TO_BYTES(ATT_UUID_SUP_UNREAD_ALERT_CAT)};
static const uint16_t alertAnsLenUnrCh = sizeof(alertAnsValUnrCh);

/* Supported unread alert category */
static const uint8_t alertAnsUuUnr[] = {UINT16_TO_BYTES(ATT_UUID_SUP_UNREAD_ALERT_CAT)};
static uint8_t alertAnsValUnr[] = {0xFF,0x03};
static uint16_t alertAnsLenUnr = sizeof(alertAnsValUnr);

/* Unread alert status characteristic */
static const uint8_t alertAnsValUasCh[] = {ATT_PROP_NOTIFY, UINT16_TO_BYTES(ALERT_ANS_UAS_HDL), UINT16_TO_BYTES(ATT_UUID_UNREAD_ALERT_STATUS)};
static const uint16_t alertAnsLenUasCh = sizeof(alertAnsValUasCh);

/* Unread alert status */
static const uint8_t alertAnsUuUas[] = {UINT16_TO_BYTES(ATT_UUID_UNREAD_ALERT_STATUS)};
static uint8_t alertAnsValUas[] = {0x00,0x00};
static uint16_t alertAnsLenUas = sizeof(alertAnsValUas);

/* Unread alert status client characteristic configuration */
static uint8_t alertAnsValUasChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t alertAnsLenUasChCcc = sizeof(alertAnsValUasChCcc);

/* Alert notification control point characteristic */
static const uint8_t alertAnsValCpCh[] = {ATT_PROP_WRITE, UINT16_TO_BYTES(ALERT_ANS_CP_HDL), UINT16_TO_BYTES(ATT_UUID_ALERT_NOTIF_CP)};
static const uint16_t alertAnsLenCpCh = sizeof(alertAnsValCpCh);

/* Alert notification control point */
static const uint8_t alertAnsUuCp[] = {UINT16_TO_BYTES(ATT_UUID_ALERT_NOTIF_CP)};
static uint8_t alertAnsValCp[] = {0, 0};
static const uint16_t alertAnsLenCp = sizeof(alertAnsValCp);

/*!
 * Phone alert status service
 */

/* Alert status client characteristic configuration */
/* Ringer setting characteristic */
/* Ringer setting */
/* Ringer settting client characteristic configuration */
/* Ringer control point characteristic */
/* Ringer control point */

/* Phone alert status service declaration */
static const uint8_t alertPassValSvc[] = {UINT16_TO_BYTES(ATT_UUID_PHONE_ALERT_SERVICE)};
static const uint16_t alertPassLenSvc = sizeof(alertPassValSvc);

/* Alert status characteristic */
static const uint8_t alertPassValAsCh[] = {ATT_PROP_READ | ATT_PROP_NOTIFY, UINT16_TO_BYTES(ALERT_PASS_AS_HDL), UINT16_TO_BYTES(ATT_UUID_ALERT_STATUS)};
static const uint16_t alertPassLenAsCh = sizeof(alertPassValAsCh);

/* Alert status */
static const uint8_t alertPassUuAs[] = {UINT16_TO_BYTES(ATT_UUID_ALERT_STATUS)};
static uint8_t alertPassValAs[] = {0};
static const uint16_t alertPassLenAs = sizeof(alertPassValAs);

/* Alert status client characteristic configuration */
static uint8_t alertPassValAsChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t alertPassLenAsChCcc = sizeof(alertPassValAsChCcc);

/* Ringer setting characteristic */
static const uint8_t alertPassValRsCh[] = {ATT_PROP_READ | ATT_PROP_NOTIFY, UINT16_TO_BYTES(ALERT_PASS_RS_HDL), UINT16_TO_BYTES(ATT_UUID_RINGER_SETTING)};
static const uint16_t alertPassLenRsCh = sizeof(alertPassValRsCh);

/* Ringer setting */
static const uint8_t alertPassUuRs[] = {UINT16_TO_BYTES(ATT_UUID_RINGER_SETTING)};
static uint8_t alertPassValRs[] = {0};
static const uint16_t alertPassLenRs = sizeof(alertPassValRs);

/* Ringer setting client characteristic configuration */
static uint8_t alertPassValRsChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t alertPassLenRsChCcc = sizeof(alertPassValRsChCcc);

/* Ringer control point characteristic */
static const uint8_t alertPassValRcpCh[] = {ATT_PROP_WRITE_NO_RSP, UINT16_TO_BYTES(ALERT_PASS_RCP_HDL), UINT16_TO_BYTES(ATT_UUID_RINGER_CP)};
static const uint16_t alertPassLenRcpCh = sizeof(alertPassValRcpCh);

/* Ringer control point */
static const uint8_t alertPassUuRcp[] = {UINT16_TO_BYTES(ATT_UUID_RINGER_CP)};
static uint8_t alertPassValRcp[] = {0};
static const uint16_t alertPassLenRcp = sizeof(alertPassValRcp);

/*!
 * Network availability service
 */

/* Network availability service declaration */
static const uint8_t alertNwsValSvc[] = {UINT16_TO_BYTES(ATT_UUID_NETWORK_AVAIL_SERVICE)};
static const uint16_t alertNwsLenSvc = sizeof(alertNwsValSvc);

/* Network availability characteristic */
static const uint8_t alertNwsValNwaCh[] = {ATT_PROP_READ | ATT_PROP_INDICATE, UINT16_TO_BYTES(ALERT_NWS_NWA_HDL), UINT16_TO_BYTES(ATT_UUID_NETWORK_AVAIL)};
static const uint16_t alertNwsLenNwaCh = sizeof(alertNwsValNwaCh);

/* Network availability */
static const uint8_t alertNwsUuNwa[] = {UINT16_TO_BYTES(ATT_UUID_NETWORK_AVAIL)};
static uint8_t alertNwsValNwa[] = {0};
static const uint16_t alertNwsLenNwa = sizeof(alertNwsValNwa);

/* Network availability client characteristic configuration */
static uint8_t alertNwsValNwaChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t alertNwsLenNwaChCcc = sizeof(alertNwsValNwaChCcc);

/* Attribute list for group */
static const attsAttr_t alertList[] =
{
  /* Alert notification service */
  {
    attPrimSvcUuid,
    (uint8_t *) alertAnsValSvc,
    (uint16_t *) &alertAnsLenSvc,
    sizeof(alertAnsValSvc),
    0,
    ATTS_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) alertAnsValSnaCh,
    (uint16_t *) &alertAnsLenSnaCh,
    sizeof(alertAnsValSnaCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    alertAnsUuSna,
    alertAnsValSna,
    &alertAnsLenSna,
    sizeof(alertAnsValSna),
    0,
    ALERT_SEC_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) alertAnsValNewCh,
    (uint16_t *) &alertAnsLenNewCh,
    sizeof(alertAnsValNewCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    alertAnsUuNew,
    alertAnsValNew,
    &alertAnsLenNew,
    sizeof(alertAnsValNew),
    0,
    0
  },
  {
    attCliChCfgUuid,
    alertAnsValNewChCcc,
    (uint16_t *) &alertAnsLenNewChCcc,
    sizeof(alertAnsValNewChCcc),
    0,
    (ATTS_PERMIT_READ | ALERT_SEC_PERMIT_WRITE)
  },
  {
    attChUuid,
    (uint8_t *) alertAnsValUnrCh,
    (uint16_t *) &alertAnsLenUnrCh,
    sizeof(alertAnsValUnrCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    alertAnsUuUnr,
    alertAnsValUnr,
    &alertAnsLenUnr,
    sizeof(alertAnsValUnr),
    0,
    ALERT_SEC_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) alertAnsValUasCh,
    (uint16_t *) &alertAnsLenUasCh,
    sizeof(alertAnsValUasCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    alertAnsUuUas,
    alertAnsValUas,
    &alertAnsLenUas,
    sizeof(alertAnsValUas),
    0,
    0
  },
  {
    attCliChCfgUuid,
    alertAnsValUasChCcc,
    (uint16_t *) &alertAnsLenUasChCcc,
    sizeof(alertAnsValUasChCcc),
    0,
    (ATTS_PERMIT_READ | ALERT_SEC_PERMIT_WRITE)
  },
  {
    attChUuid,
    (uint8_t *) alertAnsValCpCh,
    (uint16_t *) &alertAnsLenCpCh,
    sizeof(alertAnsValCpCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    alertAnsUuCp,
    alertAnsValCp,
    (uint16_t *) &alertAnsLenCp,
    sizeof(alertAnsValCp),
    0,
    ALERT_SEC_PERMIT_WRITE
  },
  /* Phone alert status service */
  {
    attPrimSvcUuid,
    (uint8_t *) alertPassValSvc,
    (uint16_t *) &alertPassLenSvc,
    sizeof(alertPassValSvc),
    0,
    ATTS_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) alertPassValAsCh,
    (uint16_t *) &alertPassLenAsCh,
    sizeof(alertPassValAsCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    alertPassUuAs,
    alertPassValAs,
    (uint16_t *) &alertPassLenAs,
    sizeof(alertPassValAs),
    0,
    ALERT_SEC_PERMIT_READ
  },
  {
    attCliChCfgUuid,
    alertPassValAsChCcc,
    (uint16_t *) &alertPassLenAsChCcc,
    sizeof(alertPassValAsChCcc),
    0,
    (ATTS_PERMIT_READ | ALERT_SEC_PERMIT_WRITE)
  },
  {
    attChUuid,
    (uint8_t *) alertPassValRsCh,
    (uint16_t *) &alertPassLenRsCh,
    sizeof(alertPassValRsCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    alertPassUuRs,
    alertPassValRs,
    (uint16_t *) &alertPassLenRs,
    sizeof(alertPassValRs),
    0,
    ALERT_SEC_PERMIT_READ
  },
  {
    attCliChCfgUuid,
    alertPassValRsChCcc,
    (uint16_t *) &alertPassLenRsChCcc,
    sizeof(alertPassValRsChCcc),
    0,
    (ATTS_PERMIT_READ | ALERT_SEC_PERMIT_WRITE)
  },
  {
    attChUuid,
    (uint8_t *) alertPassValRcpCh,
    (uint16_t *) &alertPassLenRcpCh,
    sizeof(alertPassValRcpCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    alertPassUuRcp,
    alertPassValRcp,
    (uint16_t *) &alertPassLenRcp,
    sizeof(alertPassValRcp),
    0,
    ALERT_SEC_PERMIT_WRITE
  },

  /* Network availability service */
  {
    attPrimSvcUuid,
    (uint8_t *) alertNwsValSvc,
    (uint16_t *) &alertNwsLenSvc,
    sizeof(alertNwsValSvc),
    0,
    ATTS_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) alertNwsValNwaCh,
    (uint16_t *) &alertNwsLenNwaCh,
    sizeof(alertNwsValNwaCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    alertNwsUuNwa,
    alertNwsValNwa,
    (uint16_t *) &alertNwsLenNwa,
    sizeof(alertNwsValNwa),
    0,
    ALERT_SEC_PERMIT_READ
  },
  {
    attCliChCfgUuid,
    alertNwsValNwaChCcc,
    (uint16_t *) &alertNwsLenNwaChCcc,
    sizeof(alertNwsValNwaChCcc),
    0,
    (ATTS_PERMIT_READ | ALERT_SEC_PERMIT_WRITE)
  },
};

/* Alert group structure */
static attsGroup_t svcAlertGroup =
{
  NULL,
  (attsAttr_t *) alertList,
  NULL,
  NULL,
  ALERT_START_HDL,
  ALERT_END_HDL
};

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcAlertAddGroup(void)
{
  AttsAddGroup(&svcAlertGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcAlertRemoveGroup(void)
{
  AttsRemoveGroup(ALERT_START_HDL);
}
