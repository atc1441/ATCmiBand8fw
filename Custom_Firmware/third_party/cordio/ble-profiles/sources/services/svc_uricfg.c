/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  UriCfg configuration service implementation.
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

#include <string.h>

#include "wsf_types.h"
#include "att_api.h"
#include "wsf_trace.h"
#include "util/bstream.h"
#include "svc_cfg.h"
#include "svc_uricfg.h"
#include "uribeacon/uricfg_defs.h"

/**************************************************************************************************
 Service variables
**************************************************************************************************/

/* Beacon service declaration */
static const uint8_t  uriCfgValSvc[] = {URICFG_UUID_BYTES(URICFG_UUID_SVC)};
static const uint16_t uriCfgLenSvc   = sizeof(uriCfgValSvc);

/* Beacon lock state characteristic */
static const uint8_t  uriCfgValLockStateChr[] = {ATT_PROP_READ,
                                                 UINT16_TO_BYTES(URICFG_HANDLE_LOCKSTATE),
                                                 URICFG_UUID_BYTES(URICFG_UUID_CHR_LOCKSTATE)};
static const uint16_t uriCfgLenLockStateChr   = sizeof(uriCfgValLockStateChr);

/* Beacon lock state */
static const uint8_t  uriCfgUuidLockState[] = {URICFG_UUID_BYTES(URICFG_UUID_CHR_LOCKSTATE)};
static       uint8_t  uriCfgValLockState[]  = {0x00};
static const uint16_t uriCfgLenLockState    = sizeof(uriCfgValLockState);

/* Beacon lock state characteristic user description */
static const uint8_t  uriCfgValLockStateChrUsrDescr[] = "Lock State";
static const uint16_t uriCfgLenLockStateChrUsrDescr   = sizeof(uriCfgValLockStateChrUsrDescr) - 1u;

/* Beacon lock characteristic */
static const uint8_t  uriCfgValLockChr[] = {ATT_PROP_WRITE,
                                            UINT16_TO_BYTES(URICFG_HANDLE_LOCK),
                                            URICFG_UUID_BYTES(URICFG_UUID_CHR_LOCK)};
static const uint16_t uriCfgLenLockChr   = sizeof(uriCfgValLockChr);

/* Beacon lock */
static const uint8_t  uriCfgUuidLock[] = {URICFG_UUID_BYTES(URICFG_UUID_CHR_LOCK)};
static       uint8_t  uriCfgValLock[]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint16_t uriCfgLenLock    = sizeof(uriCfgValLock);

/* Beacon lock characteristic user description */
static const uint8_t  uriCfgValLockChrUsrDescr[] = "Lock";
static const uint16_t uriCfgLenLockChrUsrDescr   = sizeof(uriCfgValLockChrUsrDescr) - 1u;

/* Beacon unlock characteristic */
static const uint8_t  uriCfgValUnlockChr[] = {ATT_PROP_WRITE,
                                              UINT16_TO_BYTES(URICFG_HANDLE_UNLOCK),
                                              URICFG_UUID_BYTES(URICFG_UUID_CHR_UNLOCK)};
static const uint16_t uriCfgLenUnlockChr   = sizeof(uriCfgValUnlockChr);

/* Beacon unlock */
static const uint8_t  uriCfgUuidUnlock[] = {URICFG_UUID_BYTES(URICFG_UUID_CHR_UNLOCK)};
static       uint8_t  uriCfgValUnlock[]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint16_t uriCfgLenUnlock    = sizeof(uriCfgValUnlock);

/* Beacon unlock characteristic user description */
static const uint8_t  uriCfgValUnlockChrUsrDescr[] = "Unlock";
static const uint16_t uriCfgLenUnlockChrUsrDescr   = sizeof(uriCfgValUnlockChrUsrDescr) - 1u;

/* Beacon URI data characteristic */
static const uint8_t  uriCfgValUriDataChr[] = {ATT_PROP_READ | ATT_PROP_WRITE,
                                               UINT16_TO_BYTES(URICFG_HANDLE_URIDATA),
                                               URICFG_UUID_BYTES(URICFG_UUID_CHR_URIDATA)};
static const uint16_t uriCfgLenUriDataChr   = sizeof(uriCfgValUriDataChr);

/* Beacon URI data */
static const uint8_t  uriCfgUuidUriData[] = {URICFG_UUID_BYTES(URICFG_UUID_CHR_URIDATA)};
static       uint8_t  uriCfgValUriData[]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x00};
static       uint16_t uriCfgLenUriData    = 0u;

/* Beacon URI data characteristic user description */
static const uint8_t  uriCfgValUriDataChrUsrDescr[] = "URI Data";
static const uint16_t uriCfgLenUriDataChrUsrDescr   = sizeof(uriCfgValUriDataChrUsrDescr) - 1u;

/* Beacon URI flags characteristic */
static const uint8_t  uriCfgValUriFlagsChr[] = {ATT_PROP_READ | ATT_PROP_WRITE,
                                                UINT16_TO_BYTES(URICFG_HANDLE_URIFLAGS),
                                                URICFG_UUID_BYTES(URICFG_UUID_CHR_URIFLAGS)};
static const uint16_t uriCfgLenUriFlagsChr   = sizeof(uriCfgValUriFlagsChr);

/* Beacon URI flags */
static const uint8_t  uriCfgUuidUriFlags[] = {URICFG_UUID_BYTES(URICFG_UUID_CHR_URIFLAGS)};
static       uint8_t  uriCfgValUriFlags[]  = {0x00};
static const uint16_t uriCfgLenUriFlags    = sizeof(uriCfgValUriFlags);

/* Beacon URI flags characteristic user description */
static const uint8_t  uriCfgValUriFlagsChrUsrDescr[] = "URI Flags";
static const uint16_t uriCfgLenUriFlagsChrUsrDescr   = sizeof(uriCfgValUriFlagsChrUsrDescr) - 1u;

/* Beacon advertised tx power levels characteristic */
static const uint8_t  uriCfgValAdvertisedTxPwrLevelsChr[] = {ATT_PROP_READ | ATT_PROP_WRITE,
                                                             UINT16_TO_BYTES(URICFG_HANDLE_TXPWRLEVELS),
                                                             URICFG_UUID_BYTES(URICFG_UUID_CHR_TXPWRLEVELS)};
static const uint16_t uriCfgLenAdvertisedTxPwrLevelsChr   = sizeof(uriCfgValAdvertisedTxPwrLevelsChr);

/* Beacon advertised tx power levels */
static const uint8_t  uriCfgUuidAdvertisedTxPwrLevels[] = {URICFG_UUID_BYTES(URICFG_UUID_CHR_TXPWRLEVELS)};
static       uint8_t  uriCfgValAdvertisedTxPwrLevels[]  = {0x00, 0x00, 0x00, 0x00};
static const uint16_t uriCfgLenAdvertisedTxPwrLevels    = sizeof(uriCfgValAdvertisedTxPwrLevels);

/* Beacon advertised tx power levels characteristic user description */
static const uint8_t  uriCfgValAdvertisedTxPwrLevelsChrUsrDescr[] = "Advertised Tx Power Levels";
static const uint16_t uriCfgLenAdvertisedTxPwrLevelsChrUsrDescr   = sizeof(uriCfgValAdvertisedTxPwrLevelsChrUsrDescr) - 1u;

/* Beacon tx power mode characteristic */
static const uint8_t  uriCfgValTxPwrModeChr[] = {ATT_PROP_READ | ATT_PROP_WRITE,
                                                 UINT16_TO_BYTES(URICFG_HANDLE_TXPWRMODE),
                                                 URICFG_UUID_BYTES(URICFG_UUID_CHR_TXPWRMODE)};
static const uint16_t uriCfgLenTxPwrModeChr   = sizeof(uriCfgValTxPwrModeChr);

/* Beacon tx power mode */
static const uint8_t  uriCfgUuidTxPwrMode[] = {URICFG_UUID_BYTES(URICFG_UUID_CHR_TXPWRMODE)};
static       uint8_t  uriCfgValTxPwrMode[]  = {0x00};
static const uint16_t uriCfgLenTxPwrMode    = sizeof(uriCfgValTxPwrMode);

/* Beacon tx power mode characteristic user description */
static const uint8_t  uriCfgValTxPwrModeChrUsrDescr[] = "Tx Power Mode";
static const uint16_t uriCfgLenTxPwrModeChrUsrDescr   = sizeof(uriCfgValTxPwrModeChrUsrDescr) - 1u;

/* Beacon period characteristic */
static const uint8_t  uriCfgValBeaconPeriodChr[] = {ATT_PROP_READ | ATT_PROP_WRITE,
                                                    UINT16_TO_BYTES(URICFG_HANDLE_BEACONPERIOD),
                                                    URICFG_UUID_BYTES(URICFG_UUID_CHR_BEACONPERIOD)};
static const uint16_t uriCfgLenBeaconPeriodChr   = sizeof(uriCfgValBeaconPeriodChr);

/* Beacon period */
static const uint8_t  uriCfgUuidBeaconPeriod[] = {URICFG_UUID_BYTES(URICFG_UUID_CHR_BEACONPERIOD)};
static       uint8_t  uriCfgValBeaconPeriod[]  = {0x00, 0x00};
static const uint16_t uriCfgLenBeaconPeriod    = sizeof(uriCfgValBeaconPeriod);

/* Beacon period characteristic user description */
static const uint8_t  uriCfgValBeaconPeriodChrUsrDescr[] = "Beacon Period";
static const uint16_t uriCfgLenBeaconPeriodChrUsrDescr   = sizeof(uriCfgValBeaconPeriodChrUsrDescr) - 1u;

/* Beacon reset characteristic */
static const uint8_t  uriCfgValResetChr[] = {ATT_PROP_WRITE,
                                             UINT16_TO_BYTES(URICFG_HANDLE_RESET),
                                             URICFG_UUID_BYTES(URICFG_UUID_CHR_RESET)};
static const uint16_t uriCfgLenResetChr   = sizeof(uriCfgValResetChr);

/* Beacon reset */
static const uint8_t  uriCfgUuidReset[] = {URICFG_UUID_BYTES(URICFG_UUID_CHR_RESET)};
static       uint8_t  uriCfgValReset[]  = {0x00};
static const uint16_t uriCfgLenReset    = sizeof(uriCfgValReset);

/* Beacon reset characteristic user description */
static const uint8_t  uriCfgValResetChrUsrDescr[] = "Reset";
static const uint16_t uriCfgLenResetChrUsrDescr   = sizeof(uriCfgValResetChrUsrDescr) - 1u;


/* Attribute list for uriCfg group */
static const attsAttr_t uriCfgList[] =
{
  /* Service declaration */
  {
    attPrimSvcUuid,
    (uint8_t *) uriCfgValSvc,
    (uint16_t *) &uriCfgLenSvc,
    sizeof(uriCfgValSvc),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic declaration */
  {
    attChUuid,
    (uint8_t *) uriCfgValLockStateChr,
    (uint16_t *) &uriCfgLenLockStateChr,
    sizeof(uriCfgValLockStateChr),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value */
  {
    uriCfgUuidLockState,
    (uint8_t *) uriCfgValLockState,
    (uint16_t *) &uriCfgLenLockState,
    sizeof(uriCfgValLockState),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic user description */
  {
    attChUserDescUuid,
    (uint8_t *) uriCfgValLockStateChrUsrDescr,
    (uint16_t *) &uriCfgLenLockStateChrUsrDescr,
    sizeof(uriCfgValLockStateChrUsrDescr) - 1,
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic declaration */
  {
    attChUuid,
    (uint8_t *) uriCfgValLockChr,
    (uint16_t *) &uriCfgLenLockChr,
    sizeof(uriCfgValLockChr),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value */
  {
    uriCfgUuidLock,
    (uint8_t *) uriCfgValLock,
    (uint16_t *) &uriCfgLenLock,
    sizeof(uriCfgValLock),
    ATTS_SET_WRITE_CBACK,
    ATTS_PERMIT_WRITE
  },
  /* Characteristic user description */
  {
    attChUserDescUuid,
    (uint8_t *) uriCfgValLockChrUsrDescr,
    (uint16_t *) &uriCfgLenLockChrUsrDescr,
    sizeof(uriCfgValLockChrUsrDescr) - 1,
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic declaration */
  {
    attChUuid,
    (uint8_t *) uriCfgValUnlockChr,
    (uint16_t *) &uriCfgLenUnlockChr,
    sizeof(uriCfgValUnlockChr),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value */
  {
    uriCfgUuidUnlock,
    (uint8_t *) uriCfgValUnlock,
    (uint16_t *) &uriCfgLenUnlock,
    sizeof(uriCfgValUnlock),
    ATTS_SET_WRITE_CBACK,
    ATTS_PERMIT_WRITE
  },
  /* Characteristic user description */
  {
    attChUserDescUuid,
    (uint8_t *) uriCfgValUnlockChrUsrDescr,
    (uint16_t *) &uriCfgLenUnlockChrUsrDescr,
    sizeof(uriCfgValUnlockChrUsrDescr) - 1,
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic declaration */
  {
    attChUuid,
    (uint8_t *) uriCfgValUriDataChr,
    (uint16_t *) &uriCfgLenUriDataChr,
    sizeof(uriCfgValUriDataChr),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value */
  {
    uriCfgUuidUriData,
    (uint8_t *) uriCfgValUriData,
    (uint16_t *) &uriCfgLenUriData,
    sizeof(uriCfgValUriData),
    ATTS_SET_WRITE_CBACK | ATTS_SET_VARIABLE_LEN,
    ATTS_PERMIT_READ | ATTS_PERMIT_WRITE
  },
  /* Characteristic user description */
  {
    attChUserDescUuid,
    (uint8_t *) uriCfgValUriDataChrUsrDescr,
    (uint16_t *) &uriCfgLenUriDataChrUsrDescr,
    sizeof(uriCfgValUriDataChrUsrDescr) - 1,
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic declaration */
  {
    attChUuid,
    (uint8_t *) uriCfgValUriFlagsChr,
    (uint16_t *) &uriCfgLenUriFlagsChr,
    sizeof(uriCfgValUriFlagsChr),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value */
  {
    uriCfgUuidUriFlags,
    (uint8_t *) uriCfgValUriFlags,
    (uint16_t *) &uriCfgLenUriFlags,
    sizeof(uriCfgValUriFlags),
    ATTS_SET_WRITE_CBACK,
    ATTS_PERMIT_READ | ATTS_PERMIT_WRITE
  },
  /* Characteristic user description */
  {
    attChUserDescUuid,
    (uint8_t *) uriCfgValUriFlagsChrUsrDescr,
    (uint16_t *) &uriCfgLenUriFlagsChrUsrDescr,
    sizeof(uriCfgValUriFlagsChrUsrDescr) - 1,
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic declaration */
  {
    attChUuid,
    (uint8_t *) uriCfgValAdvertisedTxPwrLevelsChr,
    (uint16_t *) &uriCfgLenAdvertisedTxPwrLevelsChr,
    sizeof(uriCfgValAdvertisedTxPwrLevelsChr),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value */
  {
    uriCfgUuidAdvertisedTxPwrLevels,
    (uint8_t *) uriCfgValAdvertisedTxPwrLevels,
    (uint16_t *) &uriCfgLenAdvertisedTxPwrLevels,
    sizeof(uriCfgValAdvertisedTxPwrLevels),
    ATTS_SET_WRITE_CBACK,
    ATTS_PERMIT_READ | ATTS_PERMIT_WRITE
  },
  /* Characteristic user description */
  {
    attChUserDescUuid,
    (uint8_t *) uriCfgValAdvertisedTxPwrLevelsChrUsrDescr,
    (uint16_t *) &uriCfgLenAdvertisedTxPwrLevelsChrUsrDescr,
    sizeof(uriCfgValAdvertisedTxPwrLevelsChrUsrDescr) - 1,
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic declaration */
  {
    attChUuid,
    (uint8_t *) uriCfgValTxPwrModeChr,
    (uint16_t *) &uriCfgLenTxPwrModeChr,
    sizeof(uriCfgValTxPwrModeChr),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value */
  {
    uriCfgUuidTxPwrMode,
    (uint8_t *) uriCfgValTxPwrMode,
    (uint16_t *) &uriCfgLenTxPwrMode,
    sizeof(uriCfgValTxPwrMode),
    ATTS_SET_WRITE_CBACK,
    ATTS_PERMIT_READ | ATTS_PERMIT_WRITE
  },
  /* Characteristic user description */
  {
    attChUserDescUuid,
    (uint8_t *) uriCfgValTxPwrModeChrUsrDescr,
    (uint16_t *) &uriCfgLenTxPwrModeChrUsrDescr,
    sizeof(uriCfgValTxPwrModeChrUsrDescr) - 1,
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic declaration */
  {
    attChUuid,
    (uint8_t *) uriCfgValBeaconPeriodChr,
    (uint16_t *) &uriCfgLenBeaconPeriodChr,
    sizeof(uriCfgValBeaconPeriodChr),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value */
  {
    uriCfgUuidBeaconPeriod,
    (uint8_t *) uriCfgValBeaconPeriod,
    (uint16_t *) &uriCfgLenBeaconPeriod,
    sizeof(uriCfgValBeaconPeriod),
    ATTS_SET_WRITE_CBACK,
    ATTS_PERMIT_READ | ATTS_PERMIT_WRITE
  },
  /* Characteristic user description */
  {
    attChUserDescUuid,
    (uint8_t *) uriCfgValBeaconPeriodChrUsrDescr,
    (uint16_t *) &uriCfgLenBeaconPeriodChrUsrDescr,
    sizeof(uriCfgValBeaconPeriodChrUsrDescr) - 1,
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic declaration */
  {
    attChUuid,
    (uint8_t *) uriCfgValResetChr,
    (uint16_t *) &uriCfgLenResetChr,
    sizeof(uriCfgValResetChr),
    0,
    ATTS_PERMIT_READ
  },
  /* Characteristic value */
  {
    uriCfgUuidReset,
    (uint8_t *) uriCfgValReset,
    (uint16_t *) &uriCfgLenReset,
    sizeof(uriCfgValReset),
    ATTS_SET_WRITE_CBACK,
    ATTS_PERMIT_WRITE
  },
  /* Characteristic user description */
  {
    attChUserDescUuid,
    (uint8_t *) uriCfgValResetChrUsrDescr,
    (uint16_t *) &uriCfgLenResetChrUsrDescr,
    sizeof(uriCfgValResetChrUsrDescr) - 1,
    0,
    ATTS_PERMIT_READ
  },
};

/* UriCfg group structure */
static attsGroup_t uriCfgGroup =
{
  NULL,
  (attsAttr_t *) uriCfgList,
  NULL,
  NULL,
  URICFG_HANDLE_START,
  URICFG_HANDLE_END
};

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcUriCfgAddGroup(void)
{;
  AttsAddGroup(&uriCfgGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcUriCfgRemoveGroup(void)
{
  AttsRemoveGroup(URICFG_HANDLE_START);
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
void SvcUriCfgCbackRegister(attsWriteCback_t writeCback)
{
  uriCfgGroup.writeCback = writeCback;
}
