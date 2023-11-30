/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  WP service implementation. Arm Ltd. proprietary servicde.
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
#include "svc_ch.h"
#include "svc_wp.h"
#include "svc_cfg.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions */
#ifndef WP_SEC_PERMIT_READ
#define WP_SEC_PERMIT_READ SVC_SEC_PERMIT_READ
#endif

/*! Characteristic write permissions */
#ifndef WP_SEC_PERMIT_WRITE
#define WP_SEC_PERMIT_WRITE SVC_SEC_PERMIT_WRITE
#endif

/**************************************************************************************************
 Static Variables
**************************************************************************************************/

/* UUIDs */
static const uint8_t svcDatUuid[] = {ATT_UUID_D1_DATA};

/**************************************************************************************************
 Service variables
**************************************************************************************************/

/* Proprietary service declaration */
static const uint8_t wpValSvc[] = {ATT_UUID_P1_SERVICE};
static const uint16_t wpLenSvc = sizeof(wpValSvc);

/* Proprietary data characteristic */
static const uint8_t wpValDatCh[] = {ATT_PROP_NOTIFY | ATT_PROP_WRITE_NO_RSP, UINT16_TO_BYTES(WP_DAT_HDL), ATT_UUID_D1_DATA};
static const uint16_t wpLenDatCh = sizeof(wpValDatCh);

/* Proprietary data */
/* Note these are dummy values */
static const uint8_t wpValDat[] = {0};
static const uint16_t wpLenDat = sizeof(wpValDat);

/* Proprietary data client characteristic configuration */
static uint8_t wpValDatChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t wpLenDatChCcc = sizeof(wpValDatChCcc);

/* Attribute list for WP group */
static const attsAttr_t wpList[] =
{
  {
    attPrimSvcUuid,
    (uint8_t *) wpValSvc,
    (uint16_t *) &wpLenSvc,
    sizeof(wpValSvc),
    0,
    ATTS_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) wpValDatCh,
    (uint16_t *) &wpLenDatCh,
    sizeof(wpValDatCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    svcDatUuid,
    (uint8_t *) wpValDat,
    (uint16_t *) &wpLenDat,
    ATT_VALUE_MAX_LEN,
    (ATTS_SET_UUID_128 | ATTS_SET_VARIABLE_LEN | ATTS_SET_WRITE_CBACK),
    WP_SEC_PERMIT_WRITE
  },
  {
    attCliChCfgUuid,
    (uint8_t *) wpValDatChCcc,
    (uint16_t *) &wpLenDatChCcc,
    sizeof(wpValDatChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | WP_SEC_PERMIT_WRITE)
  }
};

/* WP group structure */
static attsGroup_t svcWpGroup =
{
  NULL,
  (attsAttr_t *) wpList,
  NULL,
  NULL,
  WP_START_HDL,
  WP_END_HDL
};

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcWpAddGroup(void)
{
  AttsAddGroup(&svcWpGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcWpRemoveGroup(void)
{
  AttsRemoveGroup(WP_START_HDL);
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
void SvcWpCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback)
{
  svcWpGroup.readCback = readCback;
  svcWpGroup.writeCback = writeCback;
}
