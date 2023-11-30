/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Time-related services implementation.
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
#include "svc_time.h"
#include "svc_cfg.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions */
#ifndef TIME_SEC_PERMIT_READ
#define TIME_SEC_PERMIT_READ SVC_SEC_PERMIT_READ
#endif

/*! Characteristic write permissions */
#ifndef TIME_SEC_PERMIT_WRITE
#define TIME_SEC_PERMIT_WRITE SVC_SEC_PERMIT_WRITE
#endif

/**************************************************************************************************
 Time-Related Services group
**************************************************************************************************/

/*!
 * Current time service
 */

/* Current time service declaration */
static const uint8_t timeCtsValSvc[] = {UINT16_TO_BYTES(ATT_UUID_CURRENT_TIME_SERVICE)};
static const uint16_t timeCtsLenSvc = sizeof(timeCtsValSvc);

/* CT time characteristic */
static const uint8_t timeCtsValCtCh[] = {ATT_PROP_READ | ATT_PROP_NOTIFY, UINT16_TO_BYTES(TIME_CTS_CT_HDL), UINT16_TO_BYTES(ATT_UUID_CURRENT_TIME)};
static const uint16_t timeCtsLenCtCh = sizeof(timeCtsValCtCh);

/* CT time */
static const uint8_t timeCtsUuCt[] = {UINT16_TO_BYTES(ATT_UUID_CURRENT_TIME)};
static uint8_t timeCtsValCt[] = {UINT16_TO_BYTES(2011), 3, 17, 14, 55, 57, 0, 0, 0};
static const uint16_t timeCtsLenCt = sizeof(timeCtsValCt);

/* CT time client characteristic configuration */
static uint8_t timeCtsValCtChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t timeCtsLenCtChCcc = sizeof(timeCtsValCtChCcc);

/* Local time information characteristic  */
static const uint8_t timeCtsValLocCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(TIME_CTS_LOC_HDL), UINT16_TO_BYTES(ATT_UUID_LOCAL_TIME_INFO)};
static const uint16_t timeCtsLenLocCh = sizeof(timeCtsValLocCh);

/* Local time information */
static const uint8_t timeCtsUuLoc[] = {UINT16_TO_BYTES(ATT_UUID_LOCAL_TIME_INFO)};
static uint8_t timeCtsValLoc[] = {(uint8_t) -32, 0};
static const uint16_t timeCtsLenLoc = sizeof(timeCtsValLoc);

/* Reference time information characteristic  */
static const uint8_t timeCtsValRefCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(TIME_CTS_REF_HDL), UINT16_TO_BYTES(ATT_UUID_REFERENCE_TIME_INFO)};
static const uint16_t timeCtsLenRefCh = sizeof(timeCtsValRefCh);

/* Reference time information */
static const uint8_t timeCtsUuRef[] = {UINT16_TO_BYTES(ATT_UUID_REFERENCE_TIME_INFO)};
static uint8_t timeCtsValRef[] = {4, 255, 15, 4};
static const uint16_t timeCtsLenRef = sizeof(timeCtsValRef);

/*!
 * DST change service
 */

/* DST change service declaration */
static const uint8_t timeDstValSvc[] = {UINT16_TO_BYTES(ATT_UUID_DST_CHANGE_SERVICE)};
static const uint16_t timeDstLenSvc = sizeof(timeDstValSvc);

/* Time with DST characteristic  */
static const uint8_t timeDstValWdstCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(TIME_DST_WDST_HDL), UINT16_TO_BYTES(ATT_UUID_TIME_WITH_DST)};
static const uint16_t timeDstLenWdstCh = sizeof(timeDstValWdstCh);

/* Time with DST */
static const uint8_t timeDstUuWdst[] = {UINT16_TO_BYTES(ATT_UUID_TIME_WITH_DST)};
static uint8_t timeDstValWdst[] = {0, 0, 0, 0, 0, 0, 0, 0};
static const uint16_t timeDstLenWdst = sizeof(timeDstValWdst);

/*!
 * Reference time update service
 */

/* Reference time update service declaration */
static const uint8_t timeRtuValSvc[] = {UINT16_TO_BYTES(ATT_UUID_REF_TIME_UPDATE_SERVICE)};
static const uint16_t timeRtuLenSvc = sizeof(timeRtuValSvc);

/* Time update control point characteristic  */
static const uint8_t timeRtuValCpCh[] = {ATT_PROP_WRITE_NO_RSP, UINT16_TO_BYTES(TIME_RTU_CP_HDL), UINT16_TO_BYTES(ATT_UUID_TIME_UPDATE_CP)};
static const uint16_t timeRtuLenCpCh = sizeof(timeRtuValCpCh);

/* Time update control point */
static const uint8_t timeRtuUuCp[] = {UINT16_TO_BYTES(ATT_UUID_TIME_UPDATE_CP)};
static uint8_t timeRtuValCp[] = {0};
static const uint16_t timeRtuLenCp = sizeof(timeRtuValCp);

/* Time update state characteristic  */
static const uint8_t timeRtuValStateCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(TIME_RTU_STATE_HDL), UINT16_TO_BYTES(ATT_UUID_TIME_UPDATE_STATE)};
static const uint16_t timeRtuLenStateCh = sizeof(timeRtuValStateCh);

/* Time update state */
static const uint8_t timeRtuUuState[] = {UINT16_TO_BYTES(ATT_UUID_TIME_UPDATE_STATE)};
static uint8_t timeRtuValState[] = {0, 0};
static const uint16_t timeRtuLenState = sizeof(timeRtuValState);

/* Attribute list for group */
static const attsAttr_t timeList[] =
{
  /* CT time service */
  {
    attPrimSvcUuid,
    (uint8_t *) timeCtsValSvc,
    (uint16_t *) &timeCtsLenSvc,
    sizeof(timeCtsValSvc),
    0,
    ATTS_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) timeCtsValCtCh,
    (uint16_t *) &timeCtsLenCtCh,
    sizeof(timeCtsValCtCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    timeCtsUuCt,
    timeCtsValCt,
    (uint16_t *) &timeCtsLenCt,
    sizeof(timeCtsValCt),
    0,
    TIME_SEC_PERMIT_READ
  },
  {
    attCliChCfgUuid,
    timeCtsValCtChCcc,
    (uint16_t *) &timeCtsLenCtChCcc,
    sizeof(timeCtsValCtChCcc),
    0,
    (ATTS_PERMIT_READ | TIME_SEC_PERMIT_WRITE)
  },
  {
    attChUuid,
    (uint8_t *) timeCtsValLocCh,
    (uint16_t *) &timeCtsLenLocCh,
    sizeof(timeCtsValLocCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    timeCtsUuLoc,
    timeCtsValLoc,
    (uint16_t *) &timeCtsLenLoc,
    sizeof(timeCtsValLoc),
    0,
    TIME_SEC_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) timeCtsValRefCh,
    (uint16_t *) &timeCtsLenRefCh,
    sizeof(timeCtsValRefCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    timeCtsUuRef,
    timeCtsValRef,
    (uint16_t *) &timeCtsLenRef,
    sizeof(timeCtsValRef),
    0,
    TIME_SEC_PERMIT_READ
  },
  /* DST change service */
  {
    attPrimSvcUuid,
    (uint8_t *) timeDstValSvc,
    (uint16_t *) &timeDstLenSvc,
    sizeof(timeDstValSvc),
    0,
    ATTS_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) timeDstValWdstCh,
    (uint16_t *) &timeDstLenWdstCh,
    sizeof(timeDstValWdstCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    timeDstUuWdst,
    timeDstValWdst,
    (uint16_t *) &timeDstLenWdst,
    sizeof(timeDstValWdst),
    0,
    TIME_SEC_PERMIT_READ
  },
  /* Reference time update service */
  {
    attPrimSvcUuid,
    (uint8_t *) timeRtuValSvc,
    (uint16_t *) &timeRtuLenSvc,
    sizeof(timeRtuValSvc),
    0,
    ATTS_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) timeRtuValCpCh,
    (uint16_t *) &timeRtuLenCpCh,
    sizeof(timeRtuValCpCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    timeRtuUuCp,
    timeRtuValCp,
    (uint16_t *) &timeRtuLenCp,
    sizeof(timeRtuValCp),
    0,
    TIME_SEC_PERMIT_WRITE
  },
  {
    attChUuid,
    (uint8_t *) timeRtuValStateCh,
    (uint16_t *) &timeRtuLenStateCh,
    sizeof(timeRtuValStateCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    timeRtuUuState,
    timeRtuValState,
    (uint16_t *) &timeRtuLenState,
    sizeof(timeRtuValState),
    0,
    TIME_SEC_PERMIT_READ
  }
};

/* Time group structure */
static attsGroup_t svcTimeGroup =
{
  NULL,
  (attsAttr_t *) timeList,
  NULL,
  NULL,
  TIME_START_HDL,
  TIME_END_HDL
};

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcTimeAddGroup(void)
{
  AttsAddGroup(&svcTimeGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcTimeRemoveGroup(void)
{
  AttsRemoveGroup(TIME_START_HDL);
}
