/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Weight Scale service implementation.
 *
 *  Copyright (c) 2012-2018 Arm Ltd. All Rights Reserved.
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
#include "svc_ch.h"
#include "svc_wss.h"
#include "svc_cfg.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions */
#ifndef WSS_SEC_PERMIT_READ
#define WSS_SEC_PERMIT_READ (ATTS_PERMIT_READ | ATTS_PERMIT_READ_ENC)
#endif

/*! Characteristic write permissions */
#ifndef WSS_SEC_PERMIT_WRITE
#define WSS_SEC_PERMIT_WRITE  (ATTS_PERMIT_WRITE | ATTS_PERMIT_WRITE_ENC)
#endif

/**************************************************************************************************
 Service variables
**************************************************************************************************/

/* Weight scale service declaration */
static const uint8_t wssValSvc[] = {UINT16_TO_BYTES(ATT_UUID_WEIGHT_SCALE_SERVICE)};
static const uint16_t wssLenSvc = sizeof(wssValSvc);

/* Weight measurement characteristic */
static const uint8_t wssValWmCh[] = {ATT_PROP_INDICATE, UINT16_TO_BYTES(WSS_WM_HDL), UINT16_TO_BYTES(ATT_UUID_WEIGHT_MEAS)};
static const uint16_t wssLenWmCh = sizeof(wssValWmCh);

/* Weight measurement */
/* Note these are dummy values */
static const uint8_t wssValWm[] = {0};
static const uint16_t wssLenWm = sizeof(wssValWm);

/* Weight measurement client characteristic configuration */
static uint8_t wssValWmChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t wssLenWmChCcc = sizeof(wssValWmChCcc);

/* Weight scale feature characteristic */
static const uint8_t wssValWsfCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(WSS_WSF_HDL), UINT16_TO_BYTES(ATT_UUID_WEIGHT_SCALE_FEATURE)};
static const uint16_t wssLenWsfCh = sizeof(wssValWsfCh);

/* Weight scale feature */
static uint8_t wssValWsf[] = {UINT16_TO_BYTES(CH_WSF_FLAG_TIMESTAMP), 0x00, 0x00};
static const uint16_t wssLenWsf = sizeof(wssValWsf);

/* Attribute list for WSS group */
static const attsAttr_t wssList[] =
{
  /* Weight scale service declaration */
  {
    attPrimSvcUuid,
    (uint8_t *) wssValSvc,
    (uint16_t *) &wssLenSvc,
    sizeof(wssValSvc),
    0,
    ATTS_PERMIT_READ
  },
  /* Weight measurement characteristic */
  {
    attChUuid,
    (uint8_t *) wssValWmCh,
    (uint16_t *) &wssLenWmCh,
    sizeof(wssValWmCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Weight measurement */
  {
    attWmChUuid,
    (uint8_t *) wssValWm,
    (uint16_t *) &wssLenWm,
    sizeof(wssValWm),
    0,
    0
  },
  /* Weight measurement client characteristic configuration */
  {
    attCliChCfgUuid,
    (uint8_t *) wssValWmChCcc,
    (uint16_t *) &wssLenWmChCcc,
    sizeof(wssValWmChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | WSS_SEC_PERMIT_WRITE)
  },
  /* Weight scale feature characteristic */
  {
    attChUuid,
    (uint8_t *) wssValWsfCh,
    (uint16_t *) &wssLenWsfCh,
    sizeof(wssValWsfCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Weight scale feature */
  {
    attWsfChUuid,
    wssValWsf,
    (uint16_t *) &wssLenWsf,
    sizeof(wssValWsf),
    0,
    WSS_SEC_PERMIT_READ
  }
};

/* WSS group structure */
static attsGroup_t svcWssGroup =
{
  NULL,
  (attsAttr_t *) wssList,
  NULL,
  NULL,
  WSS_START_HDL,
  WSS_END_HDL
};

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcWssAddGroup(void)
{
  AttsAddGroup(&svcWssGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcWssRemoveGroup(void)
{
  AttsRemoveGroup(WSS_START_HDL);
}

/*************************************************************************************************/
/*!
 *  \brief  Register callbacks for the service.
 *
 *  \param  readCback   Read callback function.
 *  \param  writeCback  Write callback function.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcWssCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback)
{
  svcWssGroup.readCback = readCback;
  svcWssGroup.writeCback = writeCback;
}
