/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Glucose service implementation.
 *
 *  Copyright (c) 2012-2019 Arm Ltd. All Rights Reserved.
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
#include "svc_gls.h"
#include "svc_cfg.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions */
#ifndef GLS_SEC_PERMIT_READ
#define GLS_SEC_PERMIT_READ (ATTS_PERMIT_READ | ATTS_PERMIT_READ_ENC)
#endif

/*! Characteristic write permissions */
#ifndef GLS_SEC_PERMIT_WRITE
#define GLS_SEC_PERMIT_WRITE (ATTS_PERMIT_WRITE | ATTS_PERMIT_WRITE_ENC)
#define GLS_SEC_PERMIT_WRITE_AUTH (ATTS_PERMIT_WRITE | ATTS_PERMIT_WRITE_ENC | ATTS_PERMIT_WRITE_AUTH)
#endif

/**************************************************************************************************
 Service variables
**************************************************************************************************/

/* Glucose service declaration */
static const uint8_t glsValSvc[] = {UINT16_TO_BYTES(ATT_UUID_GLUCOSE_SERVICE)};
static const uint16_t glsLenSvc = sizeof(glsValSvc);

/* Glucose measurement characteristic */
static const uint8_t glsValGlmCh[] = {ATT_PROP_NOTIFY, UINT16_TO_BYTES(GLS_GLM_HDL), UINT16_TO_BYTES(ATT_UUID_GLUCOSE_MEAS)};
static const uint16_t glsLenGlmCh = sizeof(glsValGlmCh);

/* Glucose measurement */
/* Note these are dummy values */
static const uint8_t glsValGlm[] = {0};
static const uint16_t glsLenGlm = sizeof(glsValGlm);

/* Glucose measurement client characteristic configuration */
static uint8_t glsValGlmChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t glsLenGlmChCcc = sizeof(glsValGlmChCcc);

/* Glucose measurement context characteristic */
static const uint8_t glsValGlmcCh[] = {ATT_PROP_NOTIFY, UINT16_TO_BYTES(GLS_GLMC_HDL), UINT16_TO_BYTES(ATT_UUID_GLUCOSE_MEAS_CONTEXT)};
static const uint16_t glsLenGlmcCh = sizeof(glsValGlmcCh);

/* Glucose measurement context */
/* Note these are dummy values */
static const uint8_t glsValGlmc[] = {0};
static const uint16_t glsLenGlmc = sizeof(glsValGlmc);

/* Glucose measurement context client characteristic configuration */
static uint8_t glsValGlmcChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t glsLenGlmcChCcc = sizeof(glsValGlmcChCcc);

/* Glucose feature characteristic */
static const uint8_t glsValGlfCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(GLS_GLF_HDL), UINT16_TO_BYTES(ATT_UUID_GLUCOSE_FEATURE)};
static const uint16_t glsLenGlfCh = sizeof(glsValGlfCh);

/* Glucose feature */
static uint8_t glsValGlf[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t glsLenGlf = sizeof(glsValGlf);

/* Control point characteristic */
static const uint8_t glsValRacpCh[] = {(ATT_PROP_INDICATE | ATT_PROP_WRITE), UINT16_TO_BYTES(GLS_RACP_HDL), UINT16_TO_BYTES(ATT_UUID_RACP)};
static const uint16_t glsLenRacpCh = sizeof(glsValRacpCh);

/* Record access control point */
/* Note these are dummy values */
static const uint8_t glsValRacp[] = {0};
static const uint16_t glsLenRacp = sizeof(glsValRacp);

/* Record access control point client characteristic configuration */
static uint8_t glsValRacpChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t glsLenRacpChCcc = sizeof(glsValRacpChCcc);

/* Attribute list for GLS group */
static const attsAttr_t glsList[] =
{
  /* Service declaration */
  {
    attPrimSvcUuid,
    (uint8_t *) glsValSvc,
    (uint16_t *) &glsLenSvc,
    sizeof(glsValSvc),
    0,
    ATTS_PERMIT_READ
  },
  /* Glucose measurement characteristic declaration */
  {
    attChUuid,
    (uint8_t *) glsValGlmCh,
    (uint16_t *) &glsLenGlmCh,
    sizeof(glsValGlmCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value */
  {
    attGlmChUuid,
    (uint8_t *) glsValGlm,
    (uint16_t *) &glsLenGlm,
    sizeof(glsValGlm),
    0,
    0
  },
  /* Client characteristic configuration descriptor */
  {
    attCliChCfgUuid,
    (uint8_t *) glsValGlmChCcc,
    (uint16_t *) &glsLenGlmChCcc,
    sizeof(glsValGlmChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | GLS_SEC_PERMIT_WRITE)
  },
  /* Glucose measurement context characteristic declaration */
  {
    attChUuid,
    (uint8_t *) glsValGlmcCh,
    (uint16_t *) &glsLenGlmcCh,
    sizeof(glsValGlmcCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value */
  {
    attGlmcChUuid,
    (uint8_t *) glsValGlmc,
    (uint16_t *) &glsLenGlmc,
    sizeof(glsValGlmc),
    0,
    0
  },
  /* Client characteristic configuration descriptor */
  {
    attCliChCfgUuid,
    (uint8_t *) glsValGlmcChCcc,
    (uint16_t *) &glsLenGlmcChCcc,
    sizeof(glsValGlmcChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | GLS_SEC_PERMIT_WRITE)
  },
  /* Glucose feature characteristic declaration */
  {
    attChUuid,
    (uint8_t *) glsValGlfCh,
    (uint16_t *) &glsLenGlfCh,
    sizeof(glsValGlfCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value */
  {
    attGlfChUuid,
    glsValGlf,
    (uint16_t *) &glsLenGlf,
    sizeof(glsValGlf),
    0,
    GLS_SEC_PERMIT_READ
  },
  /* Record access control point characteristic delclaration */
  {
    attChUuid,
    (uint8_t *) glsValRacpCh,
    (uint16_t *) &glsLenRacpCh,
    sizeof(glsValRacpCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value */
  {
    attRacpChUuid,
    (uint8_t *) glsValRacp,
    (uint16_t *) &glsLenRacp,
    ATT_DEFAULT_PAYLOAD_LEN,
    (ATTS_SET_VARIABLE_LEN | ATTS_SET_WRITE_CBACK),
    GLS_SEC_PERMIT_WRITE_AUTH
  },
  /* Client characteristic configuration descriptor */
  {
    attCliChCfgUuid,
    (uint8_t *) glsValRacpChCcc,
    (uint16_t *) &glsLenRacpChCcc,
    sizeof(glsValRacpChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | GLS_SEC_PERMIT_WRITE)
  },
};

/* GLS group structure */
static attsGroup_t svcGlsGroup =
{
  NULL,
  (attsAttr_t *) glsList,
  NULL,
  NULL,
  GLS_START_HDL,
  GLS_END_HDL
};

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcGlsAddGroup(void)
{
  AttsAddGroup(&svcGlsGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcGlsRemoveGroup(void)
{
  AttsRemoveGroup(GLS_START_HDL);
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
void SvcGlsCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback)
{
  svcGlsGroup.readCback = readCback;
  svcGlsGroup.writeCback = writeCback;
}
