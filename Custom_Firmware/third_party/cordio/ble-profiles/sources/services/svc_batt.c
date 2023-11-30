/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Battery service implementation.
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
#include "util/bstream.h"
#include "svc_batt.h"
#include "svc_cfg.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions */
#ifndef BATT_SEC_PERMIT_READ
#define BATT_SEC_PERMIT_READ SVC_SEC_PERMIT_READ
#endif

/*! Characteristic write permissions */
#ifndef BATT_SEC_PERMIT_WRITE
#define BATT_SEC_PERMIT_WRITE SVC_SEC_PERMIT_WRITE
#endif

/**************************************************************************************************
 Battery Service group
**************************************************************************************************/

/*!
 * Battery service
 */

/* Battery service declaration */
static const uint8_t battValSvc[] = {UINT16_TO_BYTES(ATT_UUID_BATTERY_SERVICE)};
static const uint16_t battLenSvc = sizeof(battValSvc);

/* Battery level characteristic */
static const uint8_t battValLvlCh[] = {ATT_PROP_READ | ATT_PROP_NOTIFY, UINT16_TO_BYTES(BATT_LVL_HDL), UINT16_TO_BYTES(ATT_UUID_BATTERY_LEVEL)};
static const uint16_t battLenLvlCh = sizeof(battValLvlCh);

/* Battery level */
static uint8_t battValLvl[] = {0};
static const uint16_t battLenLvl = sizeof(battValLvl);

/* Battery level client characteristic configuration */
static uint8_t battValLvlChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t battLenLvlChCcc = sizeof(battValLvlChCcc);

/* Attribute list for group */
static const attsAttr_t battList[] =
{
  /* Service declaration */
  {
    attPrimSvcUuid,
    (uint8_t *) battValSvc,
    (uint16_t *) &battLenSvc,
    sizeof(battValSvc),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic declaration */
  {
    attChUuid,
    (uint8_t *) battValLvlCh,
    (uint16_t *) &battLenLvlCh,
    sizeof(battValLvlCh),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value */
  {
    attBlChUuid,
    battValLvl,
    (uint16_t *) &battLenLvl,
    sizeof(battValLvl),
    ATTS_SET_READ_CBACK,
    BATT_SEC_PERMIT_READ
  },
  /* Characteristic CCC descriptor */
  {
    attCliChCfgUuid,
    battValLvlChCcc,
    (uint16_t *) &battLenLvlChCcc,
    sizeof(battValLvlChCcc),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | BATT_SEC_PERMIT_WRITE)
  }
};

/* Battery group structure */
static attsGroup_t svcBattGroup =
{
  NULL,
  (attsAttr_t *) battList,
  NULL,
  NULL,
  BATT_START_HDL,
  BATT_END_HDL
};

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcBattAddGroup(void)
{
  AttsAddGroup(&svcBattGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcBattRemoveGroup(void)
{
  AttsRemoveGroup(BATT_START_HDL);
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
void SvcBattCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback)
{
  svcBattGroup.readCback = readCback;
  svcBattGroup.writeCback = writeCback;
}

/*************************************************************************************************/
/*!
 *  \brief  Add the battery service using the dynamic attribute subsystem.
 *
 *  \return None.
 */
/*************************************************************************************************/
void *SvcBattAddGroupDyn()
{
  void *pSHdl;
  uint8_t initCcc[] = {UINT16_TO_BYTES(0x0000)};
  uint8_t initBatVal[] = {0};


  /* Create the service */
  pSHdl = AttsDynCreateGroup(BATT_START_HDL, BATT_END_HDL);

  if (pSHdl != NULL)
  {
    /* Primary service */
    AttsDynAddAttrConst(pSHdl, attPrimSvcUuid, battValSvc, sizeof(battValSvc), 0, ATTS_PERMIT_READ);

    /* Battery level characteristic */
    AttsDynAddAttrConst(pSHdl, attChUuid, battValLvlCh, sizeof(battValLvlCh), 0, ATTS_PERMIT_READ);

    /* Battery level value */
    AttsDynAddAttr(pSHdl, attBlChUuid, initBatVal, sizeof(uint8_t), sizeof(uint8_t),
                   ATTS_SET_READ_CBACK, ATTS_PERMIT_READ);

    /* Battery level CCC descriptor */
    AttsDynAddAttr(pSHdl, attCliChCfgUuid, initCcc, sizeof(uint16_t), sizeof(uint16_t),
                   ATTS_SET_CCC, ATTS_PERMIT_READ | ATTS_PERMIT_WRITE);
  }

  return pSHdl;
}
