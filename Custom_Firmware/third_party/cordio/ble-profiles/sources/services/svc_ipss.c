/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Internet Profile Support Service implementation.
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
#include "svc_ipss.h"
#include "svc_cfg.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions */
#ifndef IPSS_SEC_PERMIT_READ
#define IPSS_SEC_PERMIT_READ (ATTS_PERMIT_READ | ATTS_PERMIT_READ_ENC)
#endif

/*! Characteristic write permissions */
#ifndef IPSS_SEC_PERMIT_WRITE
#define IPSS_SEC_PERMIT_WRITE  (ATTS_PERMIT_WRITE | ATTS_PERMIT_WRITE_ENC)
#endif

/**************************************************************************************************
 Service variables
**************************************************************************************************/

/* IP Support service declaration */
static const uint8_t ipssValSvc[] = {UINT16_TO_BYTES(ATT_UUID_IP_SUPPORT_SERVICE)};
static const uint16_t ipssLenSvc = sizeof(ipssValSvc);

/* Attribute list for IPSS group */
static const attsAttr_t ipssList[] =
{
  /* IP Support Service declaration */
  {
    attPrimSvcUuid,
    (uint8_t *) ipssValSvc,
    (uint16_t *) &ipssLenSvc,
    sizeof(ipssValSvc),
    0,
    ATTS_PERMIT_READ
  },
};

/* IPSS group structure */
static attsGroup_t svcIpssGroup =
{
  NULL,
  (attsAttr_t *) ipssList,
  NULL,
  NULL,
  IPSS_START_HDL,
  IPSS_END_HDL
};

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcIpssAddGroup(void)
{
  AttsAddGroup(&svcIpssGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcIpssRemoveGroup(void)
{
  AttsRemoveGroup(IPSS_START_HDL);
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
void SvcIpssCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback)
{
  svcIpssGroup.readCback = readCback;
  svcIpssGroup.writeCback = writeCback;
}
