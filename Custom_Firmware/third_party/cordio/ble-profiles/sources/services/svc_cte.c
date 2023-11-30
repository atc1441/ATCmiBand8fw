/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Constant Tone Extension Service implementation.
 *
 *  Copyright (c) 2018 Arm Ltd. All Rights Reserved.
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
#include "svc_cte.h"
#include "svc_cfg.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions. */
#ifndef CTE_SEC_PERMIT_READ
#define CTE_SEC_PERMIT_READ SVC_SEC_PERMIT_READ
#endif

/*! Characteristic write permissions. */
#ifndef CTE_SEC_PERMIT_WRITE
#define CTE_SEC_PERMIT_WRITE SVC_SEC_PERMIT_WRITE
#endif

/**************************************************************************************************
 Static Variables
**************************************************************************************************/

/*! Constant Tone Extension service declaration. */
static const uint8_t cteValSvc[] = {UINT16_TO_BYTES(ATT_UUID_CONSTANT_TONE_SERVICE)};
static const uint16_t cteLenSvc = sizeof(cteValSvc);

/*! Constant Tone Extension enable characteristic. */
static const uint8_t cteValEnableCh[] = {ATT_PROP_WRITE, UINT16_TO_BYTES(CTE_ENABLE_HDL), UINT16_TO_BYTES(ATT_UUID_CTE_ENABLE)};
static const uint16_t cteLenEnableCh = sizeof(cteValEnableCh);

/*! Constant Tone Extension enable value. */
static uint8_t cteValEnable[2] = {0};
static const uint16_t cteLenEnable = sizeof(cteValEnable);

/*! Constant Tone Extension minimum length characteristic. */
static const uint8_t cteValMinLenCh[] = {ATT_PROP_WRITE, UINT16_TO_BYTES(CTE_MIN_LEN_HDL), UINT16_TO_BYTES(ATT_UUID_CTE_MIN_LEN)};
static const uint16_t cteLenMinLenCh = sizeof(cteValMinLenCh);

/*! Constant Tone Extension minimum length value. */
static uint8_t cteValMinLen[] = {0};
static const uint16_t cteLenMinLen = sizeof(cteValMinLen);

/*! Constant Tone Extension minimum transmit count characteristic. */
static const uint8_t cteValMinTxCntCh[] = {ATT_PROP_WRITE, UINT16_TO_BYTES(CTE_ADV_MIN_TX_CNT_HDL), UINT16_TO_BYTES(ATT_UUID_CTE_TX_CNT)};
static const uint16_t cteLenMinTxCntCh = sizeof(cteValMinTxCntCh);

/*! Constant Tone Extension minimum transmit count value. */
static uint8_t cteValMinTxCnt[] = {0};
static const uint16_t cteLenMinTxCnt = sizeof(cteValMinTxCnt);

/*! Constant Tone Extension transmit duration characteristic */
static const uint8_t cteValTxDurCh[] = {ATT_PROP_WRITE, UINT16_TO_BYTES(CTE_ADV_TX_DURATION_HDL), UINT16_TO_BYTES(ATT_UUID_CTE_TX_DURATION)};
static const uint16_t cteLenTxDurCh = sizeof(cteValTxDurCh);

/*! Constant Tone Extension transmit duration value. */
static uint8_t cteValTxDur[] = {0};
static const uint16_t cteLenTxDur = sizeof(cteValTxDur);

/*! Constant Tone Extension interval characteristic. */
static const uint8_t cteValIntervalCh[] = {ATT_PROP_WRITE, UINT16_TO_BYTES(CTE_ADV_INTERVAL_HDL), UINT16_TO_BYTES(ATT_UUID_CTE_INTERVAL)};
static const uint16_t cteLenIntervalCh = sizeof(cteValIntervalCh);

/*! Constant Tone Extension interval value. */
static uint16_t cteValInterval[] = {0};
static const uint16_t cteLenInterval = sizeof(cteValInterval);

/*! Constant Tone Extension PHY characteristic. */
static const uint8_t cteValPhyCh[] = {ATT_PROP_WRITE, UINT16_TO_BYTES(CTE_ADV_EXT_PHY_HDL), UINT16_TO_BYTES(ATT_UUID_CTE_PHY)};
static const uint16_t cteLenPhyCh = sizeof(cteValPhyCh);

/*! Constant Tone Extension PHY value. */
static uint8_t cteValPhy[] = {0};
static const uint16_t cteLenPhy = sizeof(cteValPhy);

/*! Attribute list for CTE group. */
static const attsAttr_t cteList[] =
{
  /* Constant Tone Extension service declaration. */
  {
    attPrimSvcUuid,
    (uint8_t *) cteValSvc,
    (uint16_t *) &cteLenSvc,
    sizeof(cteValSvc),
    0,
    ATTS_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) cteValEnableCh,
    (uint16_t *) &cteLenEnableCh,
    sizeof(cteValEnableCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attCteEnChUuid,
    (uint8_t *) cteValEnable,
    (uint16_t *) &cteLenEnable,
    sizeof(cteValEnable),
    ATTS_SET_WRITE_CBACK,
    SVC_SEC_PERMIT_WRITE
  },
  {
    attChUuid,
    (uint8_t *) cteValMinLenCh,
    (uint16_t *) &cteLenMinLenCh,
    sizeof(cteValMinLenCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attCteMinLenChUuid,
    (uint8_t *) cteValMinLen,
    (uint16_t *) &cteLenMinLen,
    sizeof(cteValMinLen),
    ATTS_SET_WRITE_CBACK,
    SVC_SEC_PERMIT_WRITE
  },
  {
    attChUuid,
    (uint8_t *) cteValMinTxCntCh,
    (uint16_t *) &cteLenMinTxCntCh,
    sizeof(cteValMinTxCntCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attCteTxCntChUuid,
    (uint8_t *) cteValMinTxCnt,
    (uint16_t *) &cteLenMinTxCnt,
    sizeof(cteValMinTxCnt),
    ATTS_SET_WRITE_CBACK,
    SVC_SEC_PERMIT_WRITE
  },
  {
    attChUuid,
    (uint8_t *) cteValTxDurCh,
    (uint16_t *) &cteLenTxDurCh,
    sizeof(cteValTxDurCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attCteTxDurChUuid,
    (uint8_t *) cteValTxDur,
    (uint16_t *) &cteLenTxDur,
    sizeof(cteValTxDur),
    ATTS_SET_WRITE_CBACK,
    SVC_SEC_PERMIT_WRITE
  },
  {
    attChUuid,
    (uint8_t *) cteValIntervalCh,
    (uint16_t *) &cteLenIntervalCh,
    sizeof(cteValIntervalCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attCteIntChUuid,
    (uint8_t *) cteValInterval,
    (uint16_t *) &cteLenInterval,
    sizeof(cteValInterval),
    ATTS_SET_WRITE_CBACK,
    SVC_SEC_PERMIT_WRITE
  },
  {
    attChUuid,
    (uint8_t *) cteValPhyCh,
    (uint16_t *) &cteLenPhyCh,
    sizeof(cteValPhyCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attCtePhyChUuid,
    (uint8_t *) cteValPhy,
    (uint16_t *) &cteLenPhy,
    sizeof(cteValPhy),
    ATTS_SET_WRITE_CBACK,
    SVC_SEC_PERMIT_WRITE
  },
};

/*! CTE group structure. */
static attsGroup_t svcCteGroup =
{
  NULL,
  (attsAttr_t *) cteList,
  NULL,
  NULL,
  CTE_START_HDL,
  CTE_END_HDL
};

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcCteAddGroup(void)
{
  AttsAddGroup(&svcCteGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcCteRemoveGroup(void)
{
  AttsRemoveGroup(CTE_START_HDL);
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
void SvcCteCbackRegister(attsWriteCback_t writeCback)
{
  svcCteGroup.writeCback = writeCback;
}
