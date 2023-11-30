/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  UriBeacon configuration profile.
 *
 *  Copyright (c) 2011-2019 Arm Ltd. All Rights Reserved.
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

#include "uricfg_api.h"
#include "app_api.h"
#include "att_api.h"
#include "wsf_trace.h"
#include "util/bstream.h"
#include "uricfg_defs.h"

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! \brief      callback when attributes are written */
static uriCfgAttWriteCback_t uriCfgAttWriteCback;

/*! \brief      callback when lock status changes */
static uriCfgLockChangeCback_t uriCfgLockChangeCback;

/*! \brief      reset value of URI data */
static uint8_t uriCfgDataResetValue[URICFG_MAXSIZE_URIDATA_ATT];

/*************************************************************************************************/
/*!
 *  \brief  ATTS write callback for UriBeacon service.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t uriCfgBeaconWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
  uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr)
{
  uint8_t  lockState = 0;
  uint8_t *pLockState = NULL;
  uint16_t attLen;

  if (AttsGetAttr(URICFG_HANDLE_LOCKSTATE, &attLen, &pLockState) == ATT_SUCCESS)
  {
    lockState = *pLockState;
  }

  switch (handle)
  {
    case URICFG_HANDLE_LOCK:
    {
      if (uriCfgLockChangeCback != NULL)
      {
        if (!lockState)
        {
          APP_TRACE_INFO0("URI profile locking");
          lockState = 1u;
          AttsSetAttr(URICFG_HANDLE_LOCK,      URICFG_SIZE_LOCK_ATT, pValue);
          AttsSetAttr(URICFG_HANDLE_LOCKSTATE, sizeof(lockState),    &lockState);
          uriCfgLockChangeCback(lockState, pValue);
        }
        else
        {
          APP_TRACE_INFO0("URI profile already locked");
          return ATT_ERR_AUTHOR;
        }
      }
      break;
    }

    case URICFG_HANDLE_UNLOCK:
    {
      if (uriCfgLockChangeCback != NULL)
      {
        uint8_t *pLock = NULL;

        if (lockState)
        {
          if (AttsGetAttr(URICFG_HANDLE_LOCK, &attLen, &pLock) != ATT_SUCCESS)
          {
            APP_TRACE_INFO0("URI profile failed to get lock value");
            return ATT_ERR_AUTHOR;
          }
          if (memcmp(pLock, pValue, len) != 0)
          {
            APP_TRACE_INFO0("URI profile lock mismatch");
            return ATT_ERR_AUTHOR;
          }
          APP_TRACE_INFO0("URI profile unlocking");
          lockState = 0u;
          AttsSetAttr(URICFG_HANDLE_LOCKSTATE, sizeof(lockState), &lockState);
          uriCfgLockChangeCback(lockState, pValue);
        }
        else
        {
          APP_TRACE_INFO0("URI profile already unlocked");
        }
      }
      break;
    }

    case URICFG_HANDLE_URIDATA:
    {
      /* check lock */
      if (lockState)
      {
        return ATT_ERR_AUTHOR;
      }

      /* save value */
      AttsSetAttr(handle, len, pValue);
      if (uriCfgAttWriteCback != NULL)
      {
        uriCfgAttWriteCback(handle, len, pValue);
      }
      break;
    }

    case URICFG_HANDLE_URIFLAGS:
    case URICFG_HANDLE_TXPWRLEVELS:
    {
      /* check lock */
      if (lockState)
      {
        return ATT_ERR_AUTHOR;
      }

      /* save value */
      AttsSetAttr(handle, len, pValue);
      if (uriCfgAttWriteCback != NULL)
      {
        uriCfgAttWriteCback(handle, len, pValue);
      }
      break;
    }

    case URICFG_HANDLE_TXPWRMODE:
    {
      uint8_t txPwrMode;

      /* check lock */
      if (lockState)
      {
        return ATT_ERR_AUTHOR;
      }

      /* check value */
      txPwrMode = (uint8_t)*pValue;
      if (txPwrMode > URICFG_ATT_TXPWRMODE_HIGH)
      {
        return ATT_ERR_WRITE_REJ;
      }

      /* save value */
      AttsSetAttr(handle, sizeof(txPwrMode), &txPwrMode);
      if (uriCfgAttWriteCback != NULL)
      {
        uriCfgAttWriteCback(handle, sizeof(txPwrMode), &txPwrMode);
      }
      break;
    }

    case URICFG_HANDLE_BEACONPERIOD:
    {
      uint16_t beaconPeriod;

      if (lockState)
      {
        return ATT_ERR_AUTHOR;
      }

      /* check value */
      BYTES_TO_UINT16(beaconPeriod, pValue);

      /* save value */
      if (beaconPeriod != URICFG_ATT_BEACONPERIOD_DISABLE)
      {
        if (beaconPeriod < URICFG_ATT_BEACONPERIOD_MIN)
        {
          beaconPeriod = URICFG_ATT_BEACONPERIOD_MIN;
        }
      }

      uint8_t beaconPeriodVal[2] = {UINT16_TO_BYTES(beaconPeriod)};
      AttsSetAttr(handle, sizeof(beaconPeriod), beaconPeriodVal);
      if (uriCfgAttWriteCback != NULL)
      {
        uriCfgAttWriteCback(handle, sizeof(beaconPeriod), (uint8_t *)&beaconPeriod);
      }
      break;
    }

    case URICFG_HANDLE_RESET:
    {
      uint8_t resetValue;

      if (lockState)
      {
        return ATT_ERR_AUTHOR;
      }

      /* check value */
      resetValue = *pValue;

      if (resetValue != 0)
      {
        uint8_t  uriFlags;
        uint8_t  txPwrMode;
        uint16_t beaconPeriod;
        uint8_t  lock[] = {URICFG_ATT_LOCK_DEFAULT_BYTES};

        /* reset URI data */
        AttsSetAttr(URICFG_HANDLE_URIDATA, sizeof(uriCfgDataResetValue), uriCfgDataResetValue);
        if (uriCfgAttWriteCback != NULL)
        {
          uriCfgAttWriteCback(URICFG_HANDLE_URIDATA, sizeof(uriCfgDataResetValue), uriCfgDataResetValue);
        }

        /* reset URI flags */
        uriFlags = URICFG_ATT_URIFLAGS_DEFAULT;
        AttsSetAttr(URICFG_HANDLE_URIFLAGS, sizeof(uriFlags), &uriFlags);
        if (uriCfgAttWriteCback != NULL)
        {
          uriCfgAttWriteCback(URICFG_HANDLE_URIFLAGS, sizeof(uriFlags), &uriFlags);
        }

        /* reset tx power mode */
        txPwrMode = URICFG_ATT_TXPWRMODE_DEFAULT;
        AttsSetAttr(URICFG_HANDLE_TXPWRMODE, sizeof(txPwrMode), &txPwrMode);
        if (uriCfgAttWriteCback != NULL)
        {
          uriCfgAttWriteCback(URICFG_HANDLE_TXPWRMODE, sizeof(txPwrMode), &txPwrMode);
        }

        /* reset beacon period */
        beaconPeriod = URICFG_ATT_BEACONPERIOD_DEFAULT;
        uint8_t beaconPeriodVal[2] = {UINT16_TO_BYTES(beaconPeriod)};
        AttsSetAttr(URICFG_HANDLE_BEACONPERIOD, sizeof(beaconPeriod), beaconPeriodVal);
        if (uriCfgAttWriteCback != NULL)
        {
          uriCfgAttWriteCback(URICFG_HANDLE_BEACONPERIOD, sizeof(beaconPeriod), (uint8_t *)&beaconPeriod);
        }

        /* reset lock */
        AttsSetAttr(URICFG_HANDLE_LOCK, URICFG_SIZE_LOCK_ATT, lock);
        if (uriCfgAttWriteCback != NULL)
        {
          uriCfgAttWriteCback(URICFG_HANDLE_LOCK, sizeof(lock), lock);
        }
      }
      break;
    }

    default:
      return ATT_ERR_NOT_SUP;
  }
  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Start UriBeacon configuration service.
 *
 *  \param  pUriData          Initial URI data value
 *  \param  uriDataLen        Length of URI data value
 *  \param  uriFlags          Initial URI flags value
 *  \param  advTxPwrLevels    Initial advertised tx power levels value
 *  \param  txPwrMode         Initial tx power mode value
 *  \param  beaconPeriod      Initial beacon period value
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriCfgStart(const uint8_t *pUriData, uint8_t uriDataLen, uint8_t uriFlags,
    int8_t *pAdvTxPwrLevels, uint8_t txPwrMode, uint16_t beaconPeriod)
{
  SvcUriCfgAddGroup();
  SvcUriCfgCbackRegister(uriCfgBeaconWriteCback);

  uint8_t lockState = 0;
  AttsSetAttr(URICFG_HANDLE_LOCKSTATE,    sizeof(lockState),          &lockState);
  AttsSetAttr(URICFG_HANDLE_URIDATA,      uriDataLen,                 (uint8_t *)pUriData);
  AttsSetAttr(URICFG_HANDLE_URIFLAGS,     sizeof(uriFlags),           &uriFlags);
  AttsSetAttr(URICFG_HANDLE_TXPWRLEVELS,  URICFG_SIZE_TXPWRLEVELS_ATT,(uint8_t *)pAdvTxPwrLevels);
  AttsSetAttr(URICFG_HANDLE_TXPWRMODE,    sizeof(txPwrMode),          &txPwrMode);
  uint8_t beaconPeriodVal[2] = {UINT16_TO_BYTES(beaconPeriod)};
  AttsSetAttr(URICFG_HANDLE_BEACONPERIOD, sizeof(beaconPeriod),       beaconPeriodVal);

  uriCfgAttWriteCback = NULL;
  uriCfgLockChangeCback = NULL;
  memset(uriCfgDataResetValue, 0xFFu, URICFG_MAXSIZE_URIDATA_ATT);
}

/*************************************************************************************************/
/*!
 *  \brief  Stop UriBeacon service.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriCfgStop(void)
{
  SvcUriCfgRemoveGroup();
}

/*************************************************************************************************/
/*!
 *  \brief  Register callback for written UriBeacon attributes.
 *
 *  \param  cback       Callback to invoke when an attribute changes.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriCfgAttWriteCbackRegister(uriCfgAttWriteCback_t cback)
{
  uriCfgAttWriteCback = cback;
}

/*************************************************************************************************/
/*!
 *  \brief  Make UriBeacon lockable.
 *
 *  \param  lockState   Initial lock state value.
 *  \param  lock        Initial lock value.
 *  \param  cback       Callback to invoke when lock changes.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriCfgMakeLockable(uint8_t lockState, uint8_t *pLock, uriCfgLockChangeCback_t cback)
{
  AttsSetAttr(URICFG_HANDLE_LOCKSTATE, sizeof(lockState),    &lockState);
  AttsSetAttr(URICFG_HANDLE_LOCK,      URICFG_SIZE_LOCK_ATT, pLock);

  uriCfgLockChangeCback = cback;
}

/*************************************************************************************************/
/*!
 *  \brief  Set reset value of URI data.
 *
 *  \param  pUriData    Reset value of URI data.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriCfgSetUriDataResetValue(const uint8_t *pUriData)
{
  memcpy(uriCfgDataResetValue, pUriData, URICFG_MAXSIZE_URIDATA_ATT);
}
