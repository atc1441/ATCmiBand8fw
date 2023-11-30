/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Wireless Data Exchange service implementation.
 *
 *  Copyright (c) 2013-2018 Arm Ltd. All Rights Reserved.
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
#include "svc_wdxs.h"
#include "att_api.h"
#include "wsf_trace.h"
#include "util/bstream.h"
#include "svc_ch.h"
#include "svc_cfg.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions */
#ifndef WDXS_SEC_PERMIT_READ
#define WDXS_SEC_PERMIT_READ ATTS_PERMIT_READ
#endif

/*! Characteristic write permissions */
#ifndef WDXS_SEC_PERMIT_WRITE
#define WDXS_SEC_PERMIT_WRITE ATTS_PERMIT_WRITE
#endif

/**************************************************************************************************
 Global Variables
**************************************************************************************************/

/* UUIDs */
const uint8_t wdxsDcUuid[ATT_128_UUID_LEN] = {WDX_DC_UUID};     /* WDX Device Configuration Characteristic */
const uint8_t wdxsFtcUuid[ATT_128_UUID_LEN] = {WDX_FTC_UUID};   /* WDX File Transfer Control Characteristic */
const uint8_t wdxsFtdUuid[ATT_128_UUID_LEN] = {WDX_FTD_UUID};   /* WDX File Transfer Data Characteristic */
const uint8_t wdxsAuUuid[ATT_128_UUID_LEN] = {WDX_AU_UUID};     /* WDX Authentication Characteristic */

/**************************************************************************************************
 Service variables
**************************************************************************************************/

/* Proprietary Service Declaration */
static const uint8_t wdxsValSvc[] = {UINT16_TO_BYTES(WDX_SVC_UUID)};
static const uint16_t wdxsLenSvc = sizeof(wdxsValSvc);

/* WDX Device Configuration Characteristic Declaration */
static const uint8_t wdxsValDcCh[] = {ATT_PROP_NOTIFY | ATT_PROP_WRITE, UINT16_TO_BYTES(WDXS_DC_HDL), WDX_DC_UUID};
static const uint16_t wdxsLenDcCh = sizeof(wdxsValDcCh);

/* WDX Device Configuration Characteristic Value */
/* Note these are dummy values */
static const uint8_t wdxsValDc[] = {0};
static const uint16_t wdxsLenDc = sizeof(wdxsValDc);

/* WDX Device Configuration CCCD */
static uint8_t wdxsValDcChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t wdxsLenDcChCcc = sizeof(wdxsValDcChCcc);

/* WDX File Transfer Control Characteristic Declaration */
static const uint8_t wdxsValFtcCh[] = {ATT_PROP_NOTIFY | ATT_PROP_WRITE_NO_RSP, UINT16_TO_BYTES(WDXS_FTC_HDL), WDX_FTC_UUID};
static const uint16_t wdxsLenFtcCh = sizeof(wdxsValFtcCh);

/* WDX File Transfer Control Characteristic Value */
/* Note these are dummy values */
static const uint8_t wdxsValFtc[] = {0};
static const uint16_t wdxsLenFtc = sizeof(wdxsValFtc);

/* WDX File Transfer Control CCCD */
static uint8_t wdxsValFtcChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t wdxsLenFtcChCcc = sizeof(wdxsValFtcChCcc);

/* WDX File Transfer Data Characteristic Declaration */
static const uint8_t wdxsValFtdCh[] = {ATT_PROP_NOTIFY | ATT_PROP_WRITE_NO_RSP, UINT16_TO_BYTES(WDXS_FTD_HDL), WDX_FTD_UUID};
static const uint16_t wdxsLenFtdCh = sizeof(wdxsValFtdCh);

/* WDX File Transfer Data Characteristic Value */
/* Note these are dummy values */
static const uint8_t wdxsValFtd[] = {0};
static const uint16_t wdxsLenFtd = sizeof(wdxsValFtd);

/* WDX File Transfer Data CCCD */
static uint8_t wdxsValFtdChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t wdxsLenFtdChCcc = sizeof(wdxsValFtdChCcc);

/* WDX Authentication Characteristic Declaration */
static const uint8_t wdxsValAuCh[] = {ATT_PROP_NOTIFY | ATT_PROP_WRITE, UINT16_TO_BYTES(WDXS_AU_HDL), WDX_AU_UUID};
static const uint16_t wdxsLenAuCh = sizeof(wdxsValAuCh);

/* WDX Authentication Characteristic Value */
/* Note these are dummy values */
static const uint8_t wdxsValAu[] = {0};
static const uint16_t wdxsLenAu = sizeof(wdxsValAu);

/* WDX Authentication CCCD */
static uint8_t wdxsValAuChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t wdxsLenAuChCcc = sizeof(wdxsValAuChCcc);

/* Attribute list for WDX group */
static const attsAttr_t wdxsList[] =
{
  /* Service Delcaration */
  {
    attPrimSvcUuid,
    (uint8_t *) wdxsValSvc,
    (uint16_t *) &wdxsLenSvc,
    sizeof(wdxsValSvc),
    0,
    ATTS_PERMIT_READ
  },
  /* WDX Device Configuration Characteristic Declaration */
  {
    attChUuid,
    (uint8_t *) wdxsValDcCh,
    (uint16_t *) &wdxsLenDcCh,
    sizeof(wdxsValDcCh),
    0,
    ATTS_PERMIT_READ
  },
  /* WDX Device Configuration Characteristic Value */
  {
    wdxsDcUuid,
    (uint8_t *) wdxsValDc,
    (uint16_t *) &wdxsLenDc,
    ATT_DEFAULT_PAYLOAD_LEN,
    (ATTS_SET_UUID_128 | ATTS_SET_VARIABLE_LEN | ATTS_SET_WRITE_CBACK),
    WDXS_SEC_PERMIT_WRITE
  },
  /* WDX Device Configuration CCCD */
  {
    attCliChCfgUuid,
    (uint8_t *) wdxsValDcChCcc,
    (uint16_t *) &wdxsLenDcChCcc,
    sizeof(wdxsValDcChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | WDXS_SEC_PERMIT_WRITE)
  },
  /* WDX File Transfer Control Characteristic Declaration */
  {
    attChUuid,
    (uint8_t *) wdxsValFtcCh,
    (uint16_t *) &wdxsLenFtcCh,
    sizeof(wdxsValFtcCh),
    0,
    ATTS_PERMIT_READ
  },
  /* WDX File Transfer Control Characteristic Value */
  {
    wdxsFtcUuid,
    (uint8_t *) wdxsValFtc,
    (uint16_t *) &wdxsLenFtc,
    ATT_DEFAULT_PAYLOAD_LEN,
    (ATTS_SET_UUID_128 | ATTS_SET_VARIABLE_LEN | ATTS_SET_WRITE_CBACK),
    WDXS_SEC_PERMIT_WRITE
  },
  /* WDX File Transfer Control CCCD */
  {
    attCliChCfgUuid,
    (uint8_t *) wdxsValFtcChCcc,
    (uint16_t *) &wdxsLenFtcChCcc,
    sizeof(wdxsValFtcChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | WDXS_SEC_PERMIT_WRITE)
  },
  /* WDX File Transfer Data Characteristic Declaration */
  {
    attChUuid,
    (uint8_t *) wdxsValFtdCh,
    (uint16_t *) &wdxsLenFtdCh,
    sizeof(wdxsValFtdCh),
    0,
    ATTS_PERMIT_READ
  },
  /* WDX File Transfer Data Characteristic Value */
  {
    wdxsFtdUuid,
    (uint8_t *) wdxsValFtd,
    (uint16_t *) &wdxsLenFtd,
    ATT_DEFAULT_PAYLOAD_LEN,
    (ATTS_SET_UUID_128 | ATTS_SET_VARIABLE_LEN | ATTS_SET_WRITE_CBACK),
    WDXS_SEC_PERMIT_WRITE
  },
  /* WDX File Transfer Data CCCD */
  {
    attCliChCfgUuid,
    (uint8_t *) wdxsValFtdChCcc,
    (uint16_t *) &wdxsLenFtdChCcc,
    sizeof(wdxsValFtdChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | WDXS_SEC_PERMIT_WRITE)
  },
  /* WDX Authentication Characteristic Declaration */
  {
    attChUuid,
    (uint8_t *) wdxsValAuCh,
    (uint16_t *) &wdxsLenAuCh,
    sizeof(wdxsValAuCh),
    0,
    ATTS_PERMIT_READ
  },
  /* WDX Authentication Characteristic Value */
  {
    wdxsAuUuid,
    (uint8_t *) wdxsValAu,
    (uint16_t *) &wdxsLenAu,
    ATT_DEFAULT_PAYLOAD_LEN,
    (ATTS_SET_UUID_128 | ATTS_SET_VARIABLE_LEN | ATTS_SET_WRITE_CBACK),
    WDXS_SEC_PERMIT_WRITE
  },
  /* WDX Authentication CCCD */
  {
    attCliChCfgUuid,
    (uint8_t *) wdxsValAuChCcc,
    (uint16_t *) &wdxsLenAuChCcc,
    sizeof(wdxsValAuChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | WDXS_SEC_PERMIT_WRITE)
  }
};

/* WDX group structure */
static attsGroup_t svcWdxsGroup =
{
  NULL,
  (attsAttr_t *) wdxsList,
  NULL,
  NULL,
  WDXS_START_HDL,
  WDXS_END_HDL
};

/*************************************************************************************************/
/*!
 *  \brief  Add the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcWdxsAddGroup(void)
{
  AttsAddGroup(&svcWdxsGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcWdxsRemoveGroup(void)
{
  AttsRemoveGroup(WDXS_START_HDL);
}

/*************************************************************************************************/
/*!
 *  \brief  Register a write callback functions for the ATT Group.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcWdxsRegister(attsWriteCback_t writeCback)
{
  svcWdxsGroup.writeCback = writeCback;
}
