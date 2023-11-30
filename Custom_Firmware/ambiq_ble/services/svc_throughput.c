//*****************************************************************************
//
//! @file svc_throughput.c
//!
//! @brief Ambiq throughput service implementation
//!
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2023, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision release_sdk_4_4_1-7498c7b770 of the AmbiqSuite Development Package.
//
//*****************************************************************************
#include "wsf_types.h"
#include "att_api.h"
#include "wsf_trace.h"
#include "bstream.h"
#include "svc_ch.h"
#include "svc_throughput.h"
#include "svc_cfg.h"

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************


//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************

/**************************************************************************************************
 Static Variables
**************************************************************************************************/
/* UUIDs */
static const uint8_t svcRxUuid[] = {ATT_UUID_THROUGHPUT_RX};
static const uint8_t svcTxUuid[] = {ATT_UUID_THROUGHPUT_TX};
static const uint8_t svcConUptUuid[] = {ATT_UUID_THROUGHPUT_CON_UPT};
/**************************************************************************************************
 Service variables
**************************************************************************************************/

/* Throughput service declaration */
static const uint8_t throughputSvc[] = {ATT_UUID_THROUGHPUT_SERVICE};
static const uint16_t throughputLenSvc = sizeof(throughputSvc);

/* Throughput RX characteristic */
static const uint8_t throughputRxCh[] = {ATT_PROP_WRITE_NO_RSP, UINT16_TO_BYTES(THROUGHPUT_RX_HDL), ATT_UUID_THROUGHPUT_RX};
static const uint16_t throughputLenRxCh = sizeof(throughputRxCh);

/* Throughput TX characteristic */
static const uint8_t throughputTxCh[] = {ATT_PROP_NOTIFY, UINT16_TO_BYTES(THROUGHPUT_TX_HDL), ATT_UUID_THROUGHPUT_TX};
static const uint16_t throughputLenTxCh = sizeof(throughputTxCh);

/* Throughput connection update characteristic */
static const uint8_t throughputConUptCh[] = {(ATT_PROP_READ | ATT_PROP_WRITE_NO_RSP | ATT_PROP_NOTIFY), UINT16_TO_BYTES(THROUGHPUT_CON_UPT_HDL), ATT_UUID_THROUGHPUT_CON_UPT};
static const uint16_t throughputLenConUptCh = sizeof(throughputConUptCh);
/* Throughput RX data */
/* Note these are dummy values */
static const uint8_t throughputRx[] = {0};
static const uint16_t throughputLenRx = sizeof(throughputRx);

/* Throughput TX data */
/* Note these are dummy values */
static const uint8_t throughputTx[] = {0};
static const uint16_t throughputLenTx = sizeof(throughputTx);

/* Proprietary data client characteristic configuration */
static uint8_t throughputTxChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t throughputLenTxChCcc = sizeof(throughputTxChCcc);

/* Throughput Set connection parameters data */
/* Note these are dummy values */
static uint8_t throughputConUpt[6] = {0};
static const uint16_t throughputLenConUpt = sizeof(throughputConUpt);

/* Proprietary data client characteristic configuration */
static uint8_t throughputConUptChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t throughputLenConUptChCcc = sizeof(throughputConUptChCcc);

/* Attribute list for Throughput group */
static const attsAttr_t throughputList[] =
{
  {
    attPrimSvcUuid,
    (uint8_t *) throughputSvc,
    (uint16_t *) &throughputLenSvc,
    sizeof(throughputSvc),
    0,
    ATTS_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) throughputRxCh,
    (uint16_t *) &throughputLenRxCh,
    sizeof(throughputRxCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    svcRxUuid,
    (uint8_t *) throughputRx,
    (uint16_t *) &throughputLenRx,
    ATT_VALUE_MAX_LEN,
    (ATTS_SET_UUID_128 | ATTS_SET_VARIABLE_LEN | ATTS_SET_WRITE_CBACK),
    ATTS_PERMIT_WRITE
  },
  {
    attChUuid,
    (uint8_t *) throughputTxCh,
    (uint16_t *) &throughputLenTxCh,
    sizeof(throughputTxCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    svcTxUuid,
    (uint8_t *) throughputTx,
    (uint16_t *) &throughputLenTx,
    ATT_VALUE_MAX_LEN,
    (ATTS_SET_UUID_128 | ATTS_SET_VARIABLE_LEN),
    ATTS_PERMIT_READ
  },
  {
    attCliChCfgUuid,
    (uint8_t *) throughputTxChCcc,
    (uint16_t *) &throughputLenTxChCcc,
    sizeof(throughputTxChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | ATTS_PERMIT_WRITE)
  },
  /* Characteristic declaration */
  {
    attChUuid,
    (uint8_t *) throughputConUptCh,
    (uint16_t *) &throughputLenConUptCh,
    sizeof(throughputConUptCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value */
  {
    svcConUptUuid,
    (uint8_t *) throughputConUpt,
    (uint16_t *) &throughputLenConUpt,
    sizeof(throughputConUpt),
    (ATTS_SET_UUID_128 | ATTS_SET_WRITE_CBACK | ATTS_SET_READ_CBACK),
    (ATTS_PERMIT_READ | ATTS_PERMIT_WRITE)
  },
  {
    attCliChCfgUuid,
    (uint8_t *) throughputConUptChCcc,
    (uint16_t *) &throughputLenConUptChCcc,
    sizeof(throughputConUptChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | ATTS_PERMIT_WRITE)
  },
};

/* Throughput group structure */
static attsGroup_t svcThroughputGroup =
{
  NULL,
  (attsAttr_t *) throughputList,
  NULL,
  NULL,
  THROUGHPUT_START_HDL,
  THROUGHPUT_END_HDL
};

/*************************************************************************************************/
/*!
 *  \fn     SvcThroughputAddGroup
 *
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcThroughputAddGroup(void)
{
  AttsAddGroup(&svcThroughputGroup);
}

/*************************************************************************************************/
/*!
 *  \fn     SvcThroughputRemoveGroup
 *
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcThroughputRemoveGroup(void)
{
  AttsRemoveGroup(THROUGHPUT_START_HDL);
}

/*************************************************************************************************/
/*!
 *  \fn     SvcThroughputCbackRegister
 *
 *  \brief  Register callbacks for the service.
 *
 *  \param  readCback   Read callback function.
 *  \param  writeCback  Write callback function.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcThroughputCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback)
{
  svcThroughputGroup.readCback = readCback;
  svcThroughputGroup.writeCback = writeCback;
}
