/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example gyroscope sensor service implementation.
*
 *  Copyright (c) 2015-2018 Arm Ltd. All Rights Reserved.
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

#include <string.h>

#include "wsf_types.h"
#include "att_api.h"
#include "wsf_trace.h"
#include "util/bstream.h"
#include "svc_cfg.h"
#include "svc_gyro.h"

/**************************************************************************************************
 Macros
**************************************************************************************************/

#define GYRO_UUID_SVC          0x3000
#define GYRO_UUID_CHR_DATA     0x3001
#define GYRO_UUID_CHR_TEMPDATA 0x3002
#define GYRO_UUID_CHR_CONFIG   0x3003
#define GYRO_UUID_CHR_PERIOD   0x3004

/**************************************************************************************************
 Service variables
**************************************************************************************************/

/* Gyroscope service declaration. */
static const uint8_t  gyroValSvc[] = {UINT16_TO_BYTES(GYRO_UUID_SVC)};
static const uint16_t gyroLenSvc   = sizeof(gyroValSvc);

/* Gyroscope data characteristic. */
static const uint8_t  gyroValDataChr[] = {ATT_PROP_READ | ATT_PROP_NOTIFY,
                                          UINT16_TO_BYTES(GYRO_HANDLE_DATA),
                                          UINT16_TO_BYTES(GYRO_UUID_CHR_DATA)};
static const uint16_t gyroLenDataChr   = sizeof(gyroValDataChr);

/* Gyroscope data. */
static const uint8_t  gyroUuidData[] = {UINT16_TO_BYTES(GYRO_UUID_CHR_DATA)};
static       uint8_t  gyroValData[]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint16_t gyroLenData    = sizeof(gyroValData);

/* Gyroscope data client characteristic configuration. */
static       uint8_t  gyroValDataClientChrConfig[] = {0x00, 0x00};
static const uint16_t gyroLenDataClientChrConfig   = sizeof(gyroValDataClientChrConfig);

/* Gyroscope data characteristic user description. */
static const uint8_t  gyroValDataChrUsrDescr[] = "SMD Gyro Data";
static const uint16_t gyroLenDataChrUsrDescr   = sizeof(gyroValDataChrUsrDescr) - 1u;

/* Gyroscope tempdata characteristic. */
static const uint8_t  gyroValTempDataChr[] = {ATT_PROP_READ | ATT_PROP_NOTIFY,
                                              UINT16_TO_BYTES(GYRO_HANDLE_TEMPDATA),
                                              UINT16_TO_BYTES(GYRO_UUID_CHR_TEMPDATA)};
static const uint16_t gyroLenTempDataChr   = sizeof(gyroValTempDataChr);

/* Gyroscope tempdata. */
static const uint8_t  gyroUuidTempData[] = {UINT16_TO_BYTES(GYRO_UUID_CHR_TEMPDATA)};
static       uint8_t  gyroValTempData[]  = {0x00, 0x00};
static const uint16_t gyroLenTempData    = sizeof(gyroValTempData);

/* Gyroscope tempdata client characteristic configuration. */
static       uint8_t  gyroValTempDataClientChrConfig[] = {0x00, 0x00};
static const uint16_t gyroLenTempDataClientChrConfig   = sizeof(gyroValTempDataClientChrConfig);

/* Gyroscope tempdata characteristic user description. */
static const uint8_t  gyroValTempDataChrUsrDescr[] = "SMD Gyro Temp Data";
static const uint16_t gyroLenTempDataChrUsrDescr   = sizeof(gyroValTempDataChrUsrDescr) - 1u;

/* Gyroscope config characteristic. */
static const uint8_t  gyroValConfigChr[] = {ATT_PROP_READ | ATT_PROP_WRITE,
                                            UINT16_TO_BYTES(GYRO_HANDLE_CONFIG),
                                            UINT16_TO_BYTES(GYRO_UUID_CHR_CONFIG)};
static const uint16_t gyroLenConfigChr   = sizeof(gyroValConfigChr);

/* Gyroscope config. */
static const uint8_t  gyroUuidConfig[] = {UINT16_TO_BYTES(GYRO_UUID_CHR_CONFIG)};
static       uint8_t  gyroValConfig[]  = {0x00};
static const uint16_t gyroLenConfig    = sizeof(gyroValConfig);

/* Gyroscope config characteristic user description. */
static const uint8_t  gyroValConfigChrUsrDescr[] = "SMD Gyro Config";
static const uint16_t gyroLenConfigChrUsrDescr   = sizeof(gyroValConfigChrUsrDescr) - 1u;

/* Gyroscope period characteristic. */
static const uint8_t  gyroValPeriodChr[] = {ATT_PROP_READ | ATT_PROP_WRITE,
                                            UINT16_TO_BYTES(GYRO_HANDLE_PERIOD),
                                            UINT16_TO_BYTES(GYRO_UUID_CHR_PERIOD)};
static const uint16_t gyroLenPeriodChr   = sizeof(gyroValPeriodChr);

/* Gyroscope period. */
static const uint8_t  gyroUuidPeriod[] = {UINT16_TO_BYTES(GYRO_UUID_CHR_PERIOD)};
static       uint8_t  gyroValPeriod[]  = {GYRO_ATT_PERIOD_DEFAULT};
static const uint16_t gyroLenPeriod    = sizeof(gyroValPeriod);

/* Gyroscope period characteristic user description. */
static const uint8_t  gyroValPeriodChrUsrDescr[] = "SMD Gyro Period";
static const uint16_t gyroLenPeriodChrUsrDescr   = sizeof(gyroValPeriodChrUsrDescr) - 1u;

/* Attribute list for gyro group. */
static const attsAttr_t gyroList[] =
{
  /* Service declaration. */
  {
    attPrimSvcUuid,
    (uint8_t *) gyroValSvc,
    (uint16_t *) &gyroLenSvc,
    sizeof(gyroValSvc),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic declaration. */
  {
    attChUuid,
    (uint8_t *) gyroValDataChr,
    (uint16_t *) &gyroLenDataChr,
    sizeof(gyroValDataChr),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value. */
  {
    gyroUuidData,
    (uint8_t *) gyroValData,
    (uint16_t *) &gyroLenData,
    sizeof(gyroValData),
    0,
    ATTS_PERMIT_READ
  },
  /* Client characteristic configuration. */
  {
    attCliChCfgUuid,
    (uint8_t *) gyroValDataClientChrConfig,
    (uint16_t *) &gyroLenDataClientChrConfig,
    sizeof(gyroValDataClientChrConfig),
    ATTS_SET_CCC,
    ATTS_PERMIT_READ | ATTS_PERMIT_WRITE
  },
  /* Characteristic user description. */
  {
    attChUserDescUuid,
    (uint8_t *) gyroValDataChrUsrDescr,
    (uint16_t *) &gyroLenDataChrUsrDescr,
    sizeof(gyroValDataChrUsrDescr) - 1,
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic declaration. */
  {
    attChUuid,
    (uint8_t *) gyroValTempDataChr,
    (uint16_t *) &gyroLenTempDataChr,
    sizeof(gyroValTempDataChr),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value. */
  {
    gyroUuidTempData,
    (uint8_t *) gyroValTempData,
    (uint16_t *) &gyroLenTempData,
    sizeof(gyroValTempData),
    0, /*ATTS_SET_UUID_128, */
    ATTS_PERMIT_READ
  },
  /* Client characteristic configuration. */
  {
    attCliChCfgUuid,
    (uint8_t *) gyroValTempDataClientChrConfig,
    (uint16_t *) &gyroLenTempDataClientChrConfig,
    sizeof(gyroValTempDataClientChrConfig),
    ATTS_SET_CCC,
    ATTS_PERMIT_READ | ATTS_PERMIT_WRITE
  },
  /* Characteristic user description. */
  {
    attChUserDescUuid,
    (uint8_t *) gyroValTempDataChrUsrDescr,
    (uint16_t *) &gyroLenTempDataChrUsrDescr,
    sizeof(gyroValTempDataChrUsrDescr),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic declaration. */
  {
    attChUuid,
    (uint8_t *) gyroValConfigChr,
    (uint16_t *) &gyroLenConfigChr,
    sizeof(gyroValConfigChr),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value. */
  {
    gyroUuidConfig,
    (uint8_t *) gyroValConfig,
    (uint16_t *) &gyroLenConfig,
    sizeof(gyroValConfig),
    ATTS_SET_WRITE_CBACK,
    ATTS_PERMIT_READ | ATTS_PERMIT_WRITE
  },
  /* Characteristic user description. */
  {
    attChUserDescUuid,
    (uint8_t *) gyroValConfigChrUsrDescr,
    (uint16_t *) &gyroLenConfigChrUsrDescr,
    sizeof(gyroValConfigChrUsrDescr),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic declaration. */
  {
    attChUuid,
    (uint8_t *) gyroValPeriodChr,
    (uint16_t *) &gyroLenPeriodChr,
    sizeof(gyroValPeriodChr),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value. */
  {
    gyroUuidPeriod,
    (uint8_t *) gyroValPeriod,
    (uint16_t *) &gyroLenPeriod,
    sizeof(gyroValPeriod),
    ATTS_SET_WRITE_CBACK,
    ATTS_PERMIT_READ | ATTS_PERMIT_WRITE
  },
  /* Characteristic user description. */
  {
    attChUserDescUuid,
    (uint8_t *) gyroValPeriodChrUsrDescr,
    (uint16_t *) &gyroLenPeriodChrUsrDescr,
    sizeof(gyroValPeriodChrUsrDescr),
    0,
    ATTS_PERMIT_READ
  }
};

/* Gyro group structure. */
static attsGroup_t gyroGroup =
{
  NULL,
  (attsAttr_t *) gyroList,
  NULL,
  NULL,
  GYRO_HANDLE_START,
  GYRO_HANDLE_END
};

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcGyroAddGroup(void)
{
  AttsAddGroup(&gyroGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcGyroRemoveGroup(void)
{
  AttsRemoveGroup(GYRO_HANDLE_START);
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
void SvcGyroCbackRegister(attsWriteCback_t writeCback)
{
  gyroGroup.writeCback = writeCback;
}
