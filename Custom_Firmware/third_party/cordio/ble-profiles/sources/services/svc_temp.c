/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example temperature sensor service implementation.
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
#include "svc_temp.h"

/**************************************************************************************************
 Macros
**************************************************************************************************/

#define TEMP_UUID_SVC           0x3010
#define TEMP_UUID_CHR_DATA      0x3011
#define TEMP_UUID_CHR_CONFIG    0x3012
#define TEMP_UUID_CHR_PERIOD    0x3013

/**************************************************************************************************
 Service variables
**************************************************************************************************/

/* Temperature service declaration. */
static const uint8_t  tempValSvc[] = {UINT16_TO_BYTES(TEMP_UUID_SVC)};
static const uint16_t tempLenSvc   = sizeof(tempValSvc);

/* Temperature data characteristic. */
static const uint8_t  tempValDataChr[] = {ATT_PROP_READ | ATT_PROP_NOTIFY,
                                          UINT16_TO_BYTES(TEMP_HANDLE_DATA),
                                          UINT16_TO_BYTES(TEMP_UUID_CHR_DATA)};
static const uint16_t tempLenDataChr   = sizeof(tempValDataChr);

/* Temperature data. */
static const uint8_t  tempUuidData[] = {UINT16_TO_BYTES(TEMP_UUID_CHR_DATA)};
static       uint8_t  tempValData[]  = {0x00, 0x00};
static const uint16_t tempLenData    = sizeof(tempValData);

/* Temperature data client characteristic configuration. */
static       uint8_t  tempValDataClientChrConfig[] = {0x00, 0x00};
static const uint16_t tempLenDataClientChrConfig   = sizeof(tempValDataClientChrConfig);

/* Temperature data characteristic user description. */
static const uint8_t  tempValDataChrUsrDescr[] = "SMD Temp Data";
static const uint16_t tempLenDataChrUsrDescr   = sizeof(tempValDataChrUsrDescr) - 1u;

/* Temperature config characteristic. */
static const uint8_t  tempValConfigChr[] = {ATT_PROP_READ | ATT_PROP_WRITE,
                                            UINT16_TO_BYTES(TEMP_HANDLE_CONFIG),
                                            UINT16_TO_BYTES(TEMP_UUID_CHR_CONFIG)};
static const uint16_t tempLenConfigChr   = sizeof(tempValConfigChr);

/* Temperature config. */
static const uint8_t  tempUuidConfig[] = {UINT16_TO_BYTES(TEMP_UUID_CHR_CONFIG)};
static       uint8_t  tempValConfig[]  = {0x00};
static const uint16_t tempLenConfig    = sizeof(tempValConfig);

/* Temperature config characteristic user description. */
static const uint8_t  tempValConfigChrUsrDescr[] = "SMD Temp Config";
static const uint16_t tempLenConfigChrUsrDescr   = sizeof(tempValConfigChrUsrDescr) - 1u;

/* Temperature period characteristic. */
static const uint8_t  tempValPeriodChr[] = {ATT_PROP_READ | ATT_PROP_WRITE,
                                            UINT16_TO_BYTES(TEMP_HANDLE_PERIOD),
                                            UINT16_TO_BYTES(TEMP_UUID_CHR_PERIOD)};
static const uint16_t tempLenPeriodChr   = sizeof(tempValPeriodChr);

/* Temperature period. */
static const uint8_t  tempUuidPeriod[] = {UINT16_TO_BYTES(TEMP_UUID_CHR_PERIOD)};
static       uint8_t  tempValPeriod[]  = {TEMP_ATT_PERIOD_DEFAULT};
static const uint16_t tempLenPeriod    = sizeof(tempValPeriod);

/* Temperature period characteristic user description. */
static const uint8_t  tempValPeriodChrUsrDescr[] = "SMD Temp Period";
static const uint16_t tempLenPeriodChrUsrDescr   = sizeof(tempValPeriodChrUsrDescr) - 1u;

/* Attribute list for temp group. */
static const attsAttr_t tempList[] =
{
  /* Service declaration. */
  {
    attPrimSvcUuid,
    (uint8_t *) tempValSvc,
    (uint16_t *) &tempLenSvc,
    sizeof(tempValSvc),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic declaration. */
  {
    attChUuid,
    (uint8_t *) tempValDataChr,
    (uint16_t *) &tempLenDataChr,
    sizeof(tempValDataChr),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value. */
  {
    tempUuidData,
    (uint8_t *) tempValData,
    (uint16_t *) &tempLenData,
    sizeof(tempValData),
    0,
    ATTS_PERMIT_READ
  },
  /* Client characteristic configuration. */
  {
    attCliChCfgUuid,
    (uint8_t *) tempValDataClientChrConfig,
    (uint16_t *) &tempLenDataClientChrConfig,
    sizeof(tempValDataClientChrConfig),
    ATTS_SET_CCC,
    ATTS_PERMIT_READ | ATTS_PERMIT_WRITE
  },
  /* Characteristic user description. */
  {
    attChUserDescUuid,
    (uint8_t *) tempValDataChrUsrDescr,
    (uint16_t *) &tempLenDataChrUsrDescr,
    sizeof(tempValDataChrUsrDescr) - 1,
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic declaration. */
  {
    attChUuid,
    (uint8_t *) tempValConfigChr,
    (uint16_t *) &tempLenConfigChr,
    sizeof(tempValConfigChr),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value. */
  {
    tempUuidConfig,
    (uint8_t *) tempValConfig,
    (uint16_t *) &tempLenConfig,
    sizeof(tempValConfig),
    ATTS_SET_WRITE_CBACK,
    ATTS_PERMIT_READ | ATTS_PERMIT_WRITE
  },
  /* Characteristic user description. */
  {
    attChUserDescUuid,
    (uint8_t *) tempValConfigChrUsrDescr,
    (uint16_t *) &tempLenConfigChrUsrDescr,
    sizeof(tempValConfigChrUsrDescr),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic declaration. */
  {
    attChUuid,
    (uint8_t *) tempValPeriodChr,
    (uint16_t *) &tempLenPeriodChr,
    sizeof(tempValPeriodChr),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value. */
  {
    tempUuidPeriod,
    (uint8_t *) tempValPeriod,
    (uint16_t *) &tempLenPeriod,
    sizeof(tempValPeriod),
    ATTS_SET_WRITE_CBACK,
    ATTS_PERMIT_READ | ATTS_PERMIT_WRITE
  },
  /* Characteristic user description. */
  {
    attChUserDescUuid,
    (uint8_t *) tempValPeriodChrUsrDescr,
    (uint16_t *) &tempLenPeriodChrUsrDescr,
    sizeof(tempValPeriodChrUsrDescr),
    0,
    ATTS_PERMIT_READ
  }
};

/* Gyro group structure. */
static attsGroup_t tempGroup =
{
  NULL,
  (attsAttr_t *) tempList,
  NULL,
  NULL,
  TEMP_HANDLE_START,
  TEMP_HANDLE_END
};

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcTempAddGroup(void)
{
  AttsAddGroup(&tempGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcTempRemoveGroup(void)
{
  AttsRemoveGroup(TEMP_HANDLE_START);
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
void SvcTempCbackRegister(attsWriteCback_t writeCback)
{
  tempGroup.writeCback = writeCback;
}
