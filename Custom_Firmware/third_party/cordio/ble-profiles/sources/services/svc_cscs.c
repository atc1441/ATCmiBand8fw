/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Cycling Speed and Cadence Service Server implementation.
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
#include "svc_cscs.h"
#include "svc_cfg.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions */
#ifndef CSCS_SEC_PERMIT_READ
#define CSCS_SEC_PERMIT_READ (ATTS_PERMIT_READ | ATTS_PERMIT_READ_ENC)
#endif

/*! Characteristic write permissions */
#ifndef CSCS_SEC_PERMIT_WRITE
#define CSCS_SEC_PERMIT_WRITE  (ATTS_PERMIT_WRITE | ATTS_PERMIT_WRITE_ENC)
#endif

/**************************************************************************************************
 Service variables
**************************************************************************************************/

/* Cycling Speed service declaration */
static const uint8_t cscsValSvc[] = {UINT16_TO_BYTES(ATT_UUID_CYCLING_SPEED_SERVICE)};
static const uint16_t cscsLenSvc = sizeof(cscsValSvc);


/* Cycling Speed Feature characteristic */
static const uint8_t cscsValFeatureCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(CSCS_CSF_HDL), UINT16_TO_BYTES(ATT_UUID_CYCLING_SPEED_FEATURE)};
static const uint16_t cscsLenFeatureCh = sizeof(cscsValFeatureCh);

/* Cycling Speed Feature */
static uint16_t cscsValFeature[] = {CSCS_WRDS_FEATURE_BIT};
static const uint16_t cscsLenFeature = sizeof(cscsValFeature);


/* Cycling Speed Measurement characteristic */
static const uint8_t cscsValMeasurementCh[] = {ATT_PROP_NOTIFY, UINT16_TO_BYTES(CSCS_CSM_HDL), UINT16_TO_BYTES(ATT_UUID_CYCLING_SPEED_MEASUREMENT)};
static const uint16_t cscsLenMeasurementCh = sizeof(cscsValMeasurementCh);

/* Cycling Speed Measurement */
/* Note these are dummy values */
static const uint8_t cscsValMeasurement[] = { 0 };
static const uint16_t cscsLenMeasurement = CH_CSCS_MEASUREMENT_LEN;

/* Cycling Speed Measurement client characteristic configuration */
static uint8_t cscsValMeasurementChCcc[] = {UINT16_TO_BYTES(0x0000) };
static const uint16_t cscsLenMeasurementChCcc = sizeof(cscsValMeasurementChCcc);


/* Cycling Speed Sensor Location characteristic */
static const uint8_t cscsValLocationCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(CSCS_SL_HDL), UINT16_TO_BYTES(ATT_UUID_SENSOR_LOCATION)};
static const uint16_t cscsLenLocationCh = sizeof(cscsValLocationCh);

/* Cycling Speed Sensor Location */
/* Note these are dummy values */
static uint8_t cscsValLocation[] = { 0 };
static const uint16_t cscsLenLocation = sizeof(cscsValLocation);


/* Attribute list for CSCS group */
static const attsAttr_t cscsList[] =
{
  /* Cycling Speed Service declaration */
  {
    attPrimSvcUuid,
    (uint8_t *) cscsValSvc,
    (uint16_t *) &cscsLenSvc,
    sizeof(cscsValSvc),
    0,
    ATTS_PERMIT_READ
  },
  /* Cycling Speed Feature characteristic */
  {
    attChUuid,
    (uint8_t *)cscsValFeatureCh,
    (uint16_t *) &cscsLenFeatureCh,
    sizeof(cscsValFeatureCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Cycling Speed Feature value */
  {
    attCsfChUuid,
    (uint8_t *)cscsValFeature,
    (uint16_t *) &cscsLenFeature,
    sizeof(cscsValFeature),
    0,
    CSCS_SEC_PERMIT_READ
  },
  /* Cycling Speed Measurement characteristic */
  {
    attChUuid,
    (uint8_t *)cscsValMeasurementCh,
    (uint16_t *)&cscsLenMeasurementCh,
    sizeof(cscsValMeasurementCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Cycling Speed Measurement value */
  {
    attCsmChUuid,
    (uint8_t *)cscsValMeasurement,
    (uint16_t *)&cscsLenMeasurement,
    sizeof(cscsValMeasurement),
    0,
    0
  },
  /* Characteristic CCC descriptor */
  {
    attCliChCfgUuid,
    cscsValMeasurementChCcc,
    (uint16_t *)&cscsLenMeasurementChCcc,
    sizeof(cscsValMeasurementChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | CSCS_SEC_PERMIT_WRITE)
  },
  /* Cycling Speed Sensor Location characteristic */
  {
    attChUuid,
    (uint8_t *)cscsValLocationCh,
    (uint16_t *)&cscsLenLocationCh,
    sizeof(cscsValLocationCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Cycling Speed Sensor Location value */
  {
    attSlChUuid,
    (uint8_t *)cscsValLocation,
    (uint16_t *)&cscsLenLocation,
    sizeof(cscsValLocation),
    0,
    CSCS_SEC_PERMIT_READ
  },
};

/* CSCS group structure */
static attsGroup_t svcCscsGroup =
{
  NULL,
  (attsAttr_t *) cscsList,
  NULL,
  NULL,
  CSCS_START_HDL,
  CSCS_END_HDL
};

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcCscsAddGroup(void)
{
  AttsAddGroup(&svcCscsGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcCscsRemoveGroup(void)
{
  AttsRemoveGroup(CSCS_START_HDL);
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
void SvcCscsCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback)
{
  svcCscsGroup.readCback = readCback;
  svcCscsGroup.writeCback = writeCback;
}
