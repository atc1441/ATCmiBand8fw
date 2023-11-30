/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Cycling Power Service Server implementation.
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
#include "svc_cps.h"
#include "svc_cfg.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions */
#ifndef CPS_SEC_PERMIT_READ
#define CPS_SEC_PERMIT_READ (ATTS_PERMIT_READ | ATTS_PERMIT_READ_ENC)
#endif

/*! Characteristic write permissions */
#ifndef CPS_SEC_PERMIT_WRITE
#define CPS_SEC_PERMIT_WRITE  (ATTS_PERMIT_WRITE | ATTS_PERMIT_READ_ENC)
#endif

/**************************************************************************************************
 Service variables
**************************************************************************************************/

/* Cycling Power service declaration */
static const uint8_t cpsValSvc[] = {UINT16_TO_BYTES(ATT_UUID_CYCLING_POWER_SERVICE)};
static const uint16_t cpsLenSvc = sizeof(cpsValSvc);


/* Cycling Power Feature characteristic */
static const uint8_t cpsValFeatureCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(CPS_CPF_HDL), UINT16_TO_BYTES(ATT_UUID_CYCLING_POWER_FEATURE)};
static const uint16_t cpsLenFeatureCh = sizeof(cpsValFeatureCh);

/* Cycling Power Feature */
static uint32_t cpsValFeature[] = { CPP_PPBS_FEATURE_BIT };
static const uint16_t cpsLenFeature = sizeof(cpsValFeature);


/* Cycling Power Measurement characteristic */
static const uint8_t cpsValMeasurementCh[] = {ATT_PROP_NOTIFY, UINT16_TO_BYTES(CPS_CPM_HDL), UINT16_TO_BYTES(ATT_UUID_CYCLING_POWER_MEASUREMENT)};
static const uint16_t cpsLenMeasurementCh = sizeof(cpsValMeasurementCh);

/* Cycling Power Measurement */
/* Note these are dummy values */
static const uint8_t cpsValMeasurement[] = { 0 };
static const uint16_t cpsLenMeasurement = CH_CPS_MEASUREMENT_LEN;

/* Cycling Power Measurement client characteristic configuration */
static uint8_t cpsValMeasurementChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t cpsLenMeasurementChCcc = sizeof(cpsValMeasurementChCcc);


/* Cycling Power Sensor Location characteristic */
static const uint8_t cpsValLocationCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(CPS_CPSL_HDL), UINT16_TO_BYTES(ATT_UUID_SENSOR_LOCATION)};
static const uint16_t cpsLenLocationCh = sizeof(cpsValLocationCh);

/* Cycling Power Sensor Location */
/* Note these are dummy values */
static uint8_t cpsValLocation[] = { 0 };
static const uint16_t cpsLenLocation = sizeof(cpsValLocation);


/* Attribute list for CPS group */
static const attsAttr_t cpsList[] =
{
  /* Cycling Power Service declaration */
  {
    attPrimSvcUuid,
    (uint8_t *) cpsValSvc,
    (uint16_t *) &cpsLenSvc,
    sizeof(cpsValSvc),
    0,
    ATTS_PERMIT_READ
  },
  /* Cycling Power Feature characteristic */
  {
    attChUuid,
    (uint8_t *)cpsValFeatureCh,
    (uint16_t *) &cpsLenFeatureCh,
    sizeof(cpsValFeatureCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Cycling Power Feature value */
  {
    attCpfChUuid,
    (uint8_t *)cpsValFeature,
    (uint16_t *) &cpsLenFeature,
    sizeof(cpsValFeature),
    0,
    CPS_SEC_PERMIT_READ
  },
  /* Cycling Power Measurement characteristic */
  {
    attChUuid,
    (uint8_t *)cpsValMeasurementCh,
    (uint16_t *)&cpsLenMeasurementCh,
    sizeof(cpsValMeasurementCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Cycling Power Measurement value */
  {
    attCpmChUuid,
    (uint8_t *)cpsValMeasurement,
    (uint16_t *)&cpsLenMeasurement,
    sizeof(cpsValMeasurement),
    0,
    0
  },
  /* Characteristic CCC descriptor */
  {
    attCliChCfgUuid,
    cpsValMeasurementChCcc,
    (uint16_t *)&cpsLenMeasurementChCcc,
    sizeof(cpsValMeasurementChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | CPS_SEC_PERMIT_WRITE)
  },
  /* Cycling Power Sensor Location characteristic */
  {
    attChUuid,
    (uint8_t *)cpsValLocationCh,
    (uint16_t *)&cpsLenLocationCh,
    sizeof(cpsValLocationCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Cycling Power Sensor Location value */
  {
    attSlChUuid,
    (uint8_t *)cpsValLocation,
    (uint16_t *)&cpsLenLocation,
    sizeof(cpsValLocation),
    0,
    CPS_SEC_PERMIT_READ
  },
};

/* CPS group structure */
static attsGroup_t svcCpsGroup =
{
  NULL,
  (attsAttr_t *) cpsList,
  NULL,
  NULL,
  CPS_START_HDL,
  CPS_END_HDL
};

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcCpsAddGroup(void)
{
  AttsAddGroup(&svcCpsGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcCpsRemoveGroup(void)
{
  AttsRemoveGroup(CPS_START_HDL);
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
void SvcCpsCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback)
{
  svcCpsGroup.readCback = readCback;
  svcCpsGroup.writeCback = writeCback;
}
