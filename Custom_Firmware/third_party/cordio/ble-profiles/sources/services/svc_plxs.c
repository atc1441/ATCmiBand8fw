/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Pulse Oximeter Service Server implementation.
 *
 *  Copyright (c) 2016-2018 Arm Ltd. All Rights Reserved.
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
#include "att_uuid.h"
#include "wsf_trace.h"
#include "util/bstream.h"
#include "svc_ch.h"
#include "svc_plxs.h"
#include "svc_cfg.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions */
#ifndef PLXS_SEC_PERMIT_READ
#define PLXS_SEC_PERMIT_READ (ATTS_PERMIT_READ | ATTS_PERMIT_READ_ENC)
#endif

/*! Characteristic write permissions */
#ifndef PLXS_SEC_PERMIT_WRITE
#define PLXS_SEC_PERMIT_WRITE  (ATTS_PERMIT_WRITE | ATTS_PERMIT_READ_ENC)
#endif

/**************************************************************************************************
 Service variables
**************************************************************************************************/

/* Pulse Oximeter service declaration */
static const uint8_t plxsValSvc[] = {UINT16_TO_BYTES(ATT_UUID_PULSE_OXIMITER_SERVICE)};
static const uint16_t plxsLenSvc = sizeof(plxsValSvc);


/* Pulse Oximeter Feature characteristic */
static const uint8_t plxsValFeatureCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(PLXS_FEATURES_HDL), UINT16_TO_BYTES(ATT_UUID_PULSE_OX_FEATURES)};
static const uint16_t plxsLenFeatureCh = sizeof(plxsValFeatureCh);

/* Pulse Oximeter Feature */
static uint8_t plxsValFeature[CH_PLXF_MAX_FEATURES_LEN] = { 0 };
static uint16_t plxsLenFeature = CH_PLXF_MIN_FEATURES_LEN;


/* Pulse Oximeter Spot Check Measurement characteristic */
static const uint8_t plxsValSpotCheckCh[] = {ATT_PROP_INDICATE, UINT16_TO_BYTES(PLXS_SPOT_CHECK_HDL), UINT16_TO_BYTES(ATT_UUID_PULSE_OX_SPOT_CHECK)};
static const uint16_t plxsLenSpotCheckCh = sizeof(plxsValSpotCheckCh);

/* Pulse Oximeter Spot Check Measurement */
/* Note these are dummy values */
static const uint8_t plxsValSpotCheck[] = { 0 };
static const uint16_t plxsLenSpotCheck = 0;

/* Pulse Oximeter Spot Check Measurement client characteristic configuration */
static uint8_t plxsValSpotCheckChCcc[] = { UINT16_TO_BYTES(0x0000) };
static const uint16_t plxsLenSpotCheckChCcc = sizeof(plxsValSpotCheckChCcc);


/* Pulse Oximeter Continuous Measurement characteristic */
static const uint8_t plxsValContinuousCh[] = {ATT_PROP_NOTIFY, UINT16_TO_BYTES(PLXS_CONTINUOUS_HDL), UINT16_TO_BYTES(ATT_UUID_PULSE_OX_CONTINUOUS)};
static const uint16_t plxsLenContinuousCh = sizeof(plxsValContinuousCh);

/* Pulse Oximeter Continuous Measurement */
/* Note these are dummy values */
static const uint8_t plxsValContinuous[] = { 0 };
static const uint16_t plxsLenContinuous = 0;

/* Pulse Oximeter Continuous Measurement client characteristic configuration */
static uint8_t plxsValContinuousChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t plxsLenContinuousChCcc = sizeof(plxsValContinuousChCcc);


/* Pulse Oximeter Record Access Control Point characteristic */
static const uint8_t plxsValRecordAccessCh[] = {ATT_PROP_INDICATE | ATT_PROP_WRITE, UINT16_TO_BYTES(PLXS_RECORD_ACCESS_HDL), UINT16_TO_BYTES(ATT_UUID_RACP)};
static const uint16_t plxsLenRecordAccessCh = sizeof(plxsValRecordAccessCh);

/* Pulse Oximeter Record Access Control Point */
/* Note these are dummy values */
static const uint8_t plxsValRecordAccess[] = { 0 };
static const uint16_t plxsLenRecordAccess = 0;

/* Pulse Oximeter Record Access Control Point client characteristic configuration */
static uint8_t plxsValRecordAccessChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t plxsLenRecordAccessChCcc = sizeof(plxsValRecordAccessChCcc);


/* Attribute list for PLXS group */
static const attsAttr_t plxsList[] =
{
  /* Pulse Oximeter Service declaration */
  {
    attPrimSvcUuid,
    (uint8_t *) plxsValSvc,
    (uint16_t *) &plxsLenSvc,
    sizeof(plxsValSvc),
    0,
    ATTS_PERMIT_READ
  },
  /* Pulse Oximeter Feature characteristic */
  {
    attChUuid,
    (uint8_t *) plxsValFeatureCh,
    (uint16_t *) &plxsLenFeatureCh,
    sizeof(plxsValFeatureCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Pulse Oximeter Feature value */
  {
    attPlxfChUuid,
    (uint8_t *) plxsValFeature,
    (uint16_t *) &plxsLenFeature,
    sizeof(plxsValFeature),
    ATTS_SET_VARIABLE_LEN,
    PLXS_SEC_PERMIT_READ
  },
  /* Pulse Oximeter Spot Check characteristic */
  {
    attChUuid,
    (uint8_t *) plxsValSpotCheckCh,
    (uint16_t *) &plxsLenSpotCheckCh,
    sizeof(plxsValSpotCheckCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Pulse Oximeter Spot Check value */
  {
    attPlxscmChUuid,
    (uint8_t *) plxsValSpotCheck,
    (uint16_t *) &plxsLenSpotCheck,
    sizeof(plxsValSpotCheck),
    0,
    0
  },
  /* Pulse Oximeter Spot Check Characteristic CCC descriptor */
  {
    attCliChCfgUuid,
    plxsValSpotCheckChCcc,
    (uint16_t *) &plxsLenSpotCheckChCcc,
    sizeof(plxsValSpotCheckChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | PLXS_SEC_PERMIT_WRITE)
  },

  /* Pulse Oximeter Continuous Measurement characteristic */
  {
    attChUuid,
    (uint8_t *)plxsValContinuousCh,
    (uint16_t *)&plxsLenContinuousCh,
    sizeof(plxsValContinuousCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Pulse Oximeter Continuous Measurement value */
  {
    attPlxcmChUuid,
    (uint8_t *) plxsValContinuous,
    (uint16_t *) &plxsLenContinuous,
    sizeof(plxsValContinuous),
    0,
    0
  },
  /* Pulse Oximeter Continuous Measurement Characteristic CCC descriptor */
  {
    attCliChCfgUuid,
    plxsValContinuousChCcc,
    (uint16_t *) &plxsLenContinuousChCcc,
    sizeof(plxsValContinuousChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | PLXS_SEC_PERMIT_WRITE)
  },

  /* Pulse Oximeter Record Access Control Point characteristic */
  {
    attChUuid,
    (uint8_t *)plxsValRecordAccessCh,
    (uint16_t *)&plxsLenRecordAccessCh,
    sizeof(plxsValRecordAccessCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Pulse Oximeter Record Access Control Point value */
  {
    attRacpChUuid,
    (uint8_t *)plxsValRecordAccess,
    (uint16_t *)&plxsLenRecordAccess,
    ATT_DEFAULT_PAYLOAD_LEN,
    (ATTS_SET_VARIABLE_LEN | ATTS_SET_WRITE_CBACK),
    PLXS_SEC_PERMIT_WRITE
  },
  /* Pulse Oximeter Record Access Control Point Characteristic CCC descriptor */
  {
    attCliChCfgUuid,
    plxsValRecordAccessChCcc,
    (uint16_t *)&plxsLenRecordAccessChCcc,
    sizeof(plxsValRecordAccessChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | PLXS_SEC_PERMIT_WRITE)
  },
};

/* PLXS group structure */
static attsGroup_t svcPlxsGroup =
{
  NULL,
  (attsAttr_t *) plxsList,
  NULL,
  NULL,
  PLXS_START_HDL,
  PLXS_END_HDL
};

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcPlxsAddGroup(void)
{
  AttsAddGroup(&svcPlxsGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcPlxsRemoveGroup(void)
{
  AttsRemoveGroup(PLXS_START_HDL);
}

/*************************************************************************************************/
/*!
 *  \brief  Register callbacks for the service.
 *
 *  \param  writeCback  Write callback function.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcPlxsCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback)
{
  svcPlxsGroup.readCback = readCback;
  svcPlxsGroup.writeCback = writeCback;
}
