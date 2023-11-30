/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Health Thermometer service implementation.
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
#include "svc_hts.h"
#include "svc_cfg.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions */
#ifndef HTS_SEC_PERMIT_READ
#define HTS_SEC_PERMIT_READ (ATTS_PERMIT_READ | ATTS_PERMIT_READ_ENC)
#endif

/*! Characteristic write permissions */
#ifndef HTS_SEC_PERMIT_WRITE
#define HTS_SEC_PERMIT_WRITE  (ATTS_PERMIT_WRITE | ATTS_PERMIT_WRITE_ENC)
#endif

/**************************************************************************************************
 Service variables
**************************************************************************************************/

/* Health thermometer service declaration */
static const uint8_t htsValSvc[] = {UINT16_TO_BYTES(ATT_UUID_HEALTH_THERM_SERVICE)};
static const uint16_t htsLenSvc = sizeof(htsValSvc);

/* Temperature measurement characteristic */
static const uint8_t htsValTmCh[] = {ATT_PROP_INDICATE, UINT16_TO_BYTES(HTS_TM_HDL), UINT16_TO_BYTES(ATT_UUID_TEMP_MEAS)};
static const uint16_t htsLenTmCh = sizeof(htsValTmCh);

/* Temperature measurement */
/* Note these are dummy values */
static const uint8_t htsValTm[] = {0};
static const uint16_t htsLenTm = sizeof(htsValTm);

/* Temperature measurement client characteristic configuration */
static uint8_t htsValTmChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t htsLenTmChCcc = sizeof(htsValTmChCcc);

/* Intermediate temperature characteristic */
static const uint8_t htsValItCh[] = {ATT_PROP_NOTIFY, UINT16_TO_BYTES(HTS_IT_HDL), UINT16_TO_BYTES(ATT_UUID_INTERMEDIATE_TEMP)};
static const uint16_t htsLenItCh = sizeof(htsValItCh);

/* Intermediate temperature */
/* Note these are dummy values */
static const uint8_t htsValIt[] = {0};
static const uint16_t htsLenIt = sizeof(htsValIt);

/* Intermediate temperature client characteristic configuration */
static uint8_t htsValItChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t htsLenItChCcc = sizeof(htsValItChCcc);

/* Temperature type characteristic */
static const uint8_t htsValTtCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(HTS_TT_HDL), UINT16_TO_BYTES(ATT_UUID_TEMP_TYPE)};
static const uint16_t htsLenTtCh = sizeof(htsValTtCh);

/* Temperature type */
static uint8_t htsValTt[] = {CH_TT_BODY};
static const uint16_t htsLenTt = sizeof(htsValTt);

/* Attribute list for HTS group */
static const attsAttr_t htsList[] =
{
  /* Health thermometer service declaration */
  {
    attPrimSvcUuid,
    (uint8_t *) htsValSvc,
    (uint16_t *) &htsLenSvc,
    sizeof(htsValSvc),
    0,
    ATTS_PERMIT_READ
  },
  /* Temperature measurement characteristic */
  {
    attChUuid,
    (uint8_t *) htsValTmCh,
    (uint16_t *) &htsLenTmCh,
    sizeof(htsValTmCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Temperature measurement */
  {
    attTmChUuid,
    (uint8_t *) htsValTm,
    (uint16_t *) &htsLenTm,
    sizeof(htsValTm),
    0,
    0
  },
  /* Temperature measurement client characteristic configuration */
  {
    attCliChCfgUuid,
    (uint8_t *) htsValTmChCcc,
    (uint16_t *) &htsLenTmChCcc,
    sizeof(htsValTmChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | HTS_SEC_PERMIT_WRITE)
  },
  /* Intermediate temperature characteristic */
  {
    attChUuid,
    (uint8_t *) htsValItCh,
    (uint16_t *) &htsLenItCh,
    sizeof(htsValItCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Intermediate temperature */
  {
    attItChUuid,
    (uint8_t *) htsValIt,
    (uint16_t *) &htsLenIt,
    sizeof(htsValIt),
    0,
    0
  },
  /* Intermediate temperature client characteristic configuration */
  {
    attCliChCfgUuid,
    (uint8_t *) htsValItChCcc,
    (uint16_t *) &htsLenItChCcc,
    sizeof(htsValItChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | HTS_SEC_PERMIT_WRITE)
  },
  /* Temperature type characteristic */
  {
    attChUuid,
    (uint8_t *) htsValTtCh,
    (uint16_t *) &htsLenTtCh,
    sizeof(htsValTtCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Temperature type */
  {
    attTtChUuid,
    (uint8_t *) htsValTt,
    (uint16_t *) &htsLenTt,
    sizeof(htsValTt),
    0,
    HTS_SEC_PERMIT_READ
  }
};

/* HTS group structure */
static attsGroup_t svcHtsGroup =
{
  NULL,
  (attsAttr_t *) htsList,
  NULL,
  NULL,
  HTS_START_HDL,
  HTS_END_HDL
};

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHtsAddGroup(void)
{
  AttsAddGroup(&svcHtsGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHtsRemoveGroup(void)
{
  AttsRemoveGroup(HTS_START_HDL);
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
void SvcHtsCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback)
{
  svcHtsGroup.readCback = readCback;
  svcHtsGroup.writeCback = writeCback;
}
