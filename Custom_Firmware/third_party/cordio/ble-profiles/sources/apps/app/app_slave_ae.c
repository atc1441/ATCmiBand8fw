/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Application framework module for extended slave.
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

#include <string.h>
#include "wsf_types.h"
#include "wsf_assert.h"
#include "wsf_trace.h"
#include "dm_api.h"
#include "app_api.h"
#include "app_main.h"

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! Extended slave control block */
typedef struct
{
  uint8_t     *pPerAdvData[DM_NUM_ADV_SETS];     /*! Periodic advertising data pointers */
  uint16_t    perAdvDataLen[DM_NUM_ADV_SETS];    /*! Periodic advertising data lengths */
  uint16_t    perAdvDataBufLen[DM_NUM_ADV_SETS]; /*! Length of periodic advertising data buffer maintained by Application */
  uint16_t    perAdvDataOffset[DM_NUM_ADV_SETS]; /*! Periodic advertising data offsets */
  bool_t      perAdvDataSynced[DM_NUM_ADV_SETS]; /*! TRUE if periodic advertising data is synced */
  bool_t      perAdvParamsCfg[DM_NUM_ADV_SETS];  /*! TRUE if periodic advertising parameters are configured */
  uint8_t     perAdvState[DM_NUM_ADV_SETS];      /*! Periodic advertising state */
} appExtSlaveCb_t;

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/* Extended slave control block */
static appExtSlaveCb_t appExtSlaveCb;

/*************************************************************************************************/
/*!
 *  \brief  Utility function to start extended advertising.
 *
 *  \param  advHandle    Advertising handle.
 *  \param  cfgAdvParam  Whether to configure advertising parameters
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appSlaveExtAdvStart(uint8_t advHandle, bool_t setAdvParam)
{
  appAdvStart(1, &advHandle, pAppExtAdvCfg->advInterval, pAppExtAdvCfg->advDuration,
              pAppExtAdvCfg->maxEaEvents, setAdvParam);
}

/*************************************************************************************************/
/*!
 *  \brief  Utility function to handle change in advertising type.
 *
 *  \param  pMsg      Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appSlaveExtAdvTypeChanged(dmEvt_t *pMsg)
{
  /* clear advertising type changed flag */
  appSlaveCb.advTypeChanged[pMsg->advSetStop.advHandle] = FALSE;

  /* set advertising state */
  appSlaveCb.advState[pMsg->advSetStop.advHandle] = APP_ADV_STATE1;

  /* restart advertising */
  appSlaveExtAdvStart(pMsg->advSetStop.advHandle, TRUE);
}

/*************************************************************************************************/
/*!
 *  \brief  Set the next advertising state.
 *
 *  \param  pMsg       Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appSlaveNextExtAdvState(dmEvt_t *pMsg)
{
  /* if adv hasn't been stopped and all adv/scan data haven't been sent */
  if ((appSlaveCb.advState[pMsg->advSetStop.advHandle] != APP_ADV_STOPPED) &&
      !appSlaveCb.advDataSynced[pMsg->advSetStop.advHandle])
  {
    /* set advertising state */
    appSlaveCb.advState[pMsg->advSetStop.advHandle] = APP_ADV_STATE1;

    /* restart advertising with rest of adv/scan data */
    appSlaveExtAdvStart(pMsg->advSetStop.advHandle, FALSE);

    return;
  }

  /* done with this advertising set */
  appSlaveCb.advState[pMsg->advSetStop.advHandle] = APP_ADV_STOPPED;
}

/*************************************************************************************************/
/*!
 *  \brief  Handle a DM_ADV_SET_STOP_IND event.
 *
 *  \param  pMsg      Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appSlaveExtAdvStop(dmEvt_t *pMsg)
{
  /* if advertising set was terminated */
  if (pMsg->hdr.event == DM_ADV_SET_STOP_IND)
  {
    /* if advertising was stopped for change to advertising type */
    if (appSlaveCb.advTypeChanged[pMsg->advSetStop.advHandle])
    {
      appSlaveExtAdvTypeChanged(pMsg);
    }
    /* else if advertising successfully ended with connection being created */
    else if (pMsg->hdr.status == HCI_SUCCESS)
    {
      /* advertising is stopped once a connection is opened */
      appSlaveCb.advState[pMsg->advSetStop.advHandle] = APP_ADV_STOPPED;
    }
    /* else advertising ended for other reasons */
    else
    {
      appSlaveNextExtAdvState(pMsg);
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Set periodic advertising data fragment.
 *
 *  \param  advHandle Advertising handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appSetPerAdvDataFrag(uint8_t advHandle)
{
  uint8_t  op;
  uint16_t fragLen;
  uint16_t remainLen;
  uint8_t  *pAdvData;
  bool_t   firstFrag = TRUE;

  /* get data pointer and remaining data length */
  pAdvData = appExtSlaveCb.pPerAdvData[advHandle];
  remainLen = appExtSlaveCb.perAdvDataLen[advHandle] - appExtSlaveCb.perAdvDataOffset[advHandle];

  /* if remaing data length > max adv data length supported by Controller */
  if (remainLen > appSlaveCb.maxAdvDataLen[advHandle])
  {
    remainLen = appSlaveCb.maxAdvDataLen[advHandle];
  }

  /* while there remains data to be sent */
  while (remainLen > 0)
  {
    /* if remaing data length > max length of periodic advertising data (per set adv data command) */
    if (remainLen > HCI_PER_ADV_DATA_LEN)
    {
      /* data needs to be fragmented */
      fragLen = HCI_PER_ADV_DATA_LEN;
      op = firstFrag ? HCI_ADV_DATA_OP_FRAG_FIRST : HCI_ADV_DATA_OP_FRAG_INTER;

    }
    else
    {
      /* no fragmentation needed */
      fragLen = remainLen;
      op = firstFrag ? HCI_ADV_DATA_OP_COMP_FRAG : HCI_ADV_DATA_OP_FRAG_LAST;
    }

    /* send periodic adv data */
    DmPerAdvSetData(advHandle, op, (uint8_t)fragLen,
                    &(pAdvData[appExtSlaveCb.perAdvDataOffset[advHandle]]));

    /* store periodic adv data offset */
    appExtSlaveCb.perAdvDataOffset[advHandle] += fragLen;

    /* update remaining data length */
    remainLen -= fragLen;
    firstFrag = FALSE;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Set periodic advertising data.
 *
 *  \param  advHandle Advertising handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void appSetPerAdvData(uint8_t advHandle)
{
  /* set periodic advertising data */
  if (appExtSlaveCb.perAdvDataOffset[advHandle] < appExtSlaveCb.perAdvDataLen[advHandle])
  {
    appSetPerAdvDataFrag(advHandle);
  }

  /* if all periodic advertising data have been sent */
  if ((appExtSlaveCb.perAdvDataOffset[advHandle] >= appExtSlaveCb.perAdvDataLen[advHandle]))
  {
    appExtSlaveCb.perAdvDataSynced[advHandle] = TRUE;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Utility function to start periodic advertising.
 *
 *  \param  avHandle      Advertising handles array.
 *  \param  advInterval   Advertising interval (in 0.625 ms units).
 *
 *  \return None.
 */
/*************************************************************************************************/
void appPerAdvStart(uint8_t advHandle, uint16_t advInterval)
{
  /* if advertising parameters to be configured */
  if (!appExtSlaveCb.perAdvParamsCfg[advHandle])
  {
    /* set min and max interval */
    DmPerAdvSetInterval(advHandle, advInterval, advInterval);

    /* set advertising parameters */
    DmPerAdvConfig(advHandle);
    appExtSlaveCb.perAdvParamsCfg[advHandle] = TRUE;
  }

  /* if periodic adv data to be synced */
  if (!appExtSlaveCb.perAdvDataSynced[advHandle])
  {
    /* set periodic advertising data */
    appSetPerAdvData(advHandle);
  }

  /* start periodic advertising */
  DmPerAdvStart(advHandle);
}

/*************************************************************************************************/
/*!
 *  \brief  Set periodic advertising data for a given advertising set.
 *
 *  \param  advHandle Advertising handle.
 *  \param  len       Length of the data.
 *  \param  pData     Pointer to the data.
 *  \param  bufLen    Length of the data buffer maintained by Application.  Minimum length is
 *                    31 bytes.
 *
 *  \return None.
 */
/*************************************************************************************************/
void appPerAdvSetData(uint8_t advHandle, uint16_t len, uint8_t *pData, uint16_t bufLen)
{
  /* store data for location */
  appExtSlaveCb.pPerAdvData[advHandle] = pData;
  appExtSlaveCb.perAdvDataLen[advHandle] = len;

  /* set length of advertising data buffer maintained by Application */
  appExtSlaveCb.perAdvDataBufLen[advHandle] = bufLen;

  /* set maximum advertising data length supported by Controller */
  appSlaveCb.maxAdvDataLen[advHandle] = HciGetMaxAdvDataLen();

  /* reset data offset */
  appExtSlaveCb.perAdvDataOffset[advHandle] = 0;

  /* Set the data now if we are in the right mode and the data is complete (no fragmentation's required) */
  if ((appExtSlaveCb.perAdvState[advHandle] != APP_ADV_STOPPED) &&
      (appExtSlaveCb.perAdvParamsCfg[advHandle] == TRUE) &&
      (len <= HCI_PER_ADV_DATA_LEN) &&
      (len <= appSlaveCb.maxAdvDataLen[advHandle]))
  {
    appSetPerAdvData(advHandle);
  }
  /* Otherwise set it when advertising is started or mode changes */
  else
  {
    appExtSlaveCb.perAdvDataSynced[advHandle] = FALSE;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Set the value of an advertising data element in the peroidic advertising data.
 *          If the element already exists in the data then it is replaced with the new value.
 *          If the element does not exist in the data it is appended to it, space permitting.
 *
 *          There is special handling for the device name (AD type DM_ADV_TYPE_LOCAL_NAME).
 *          If the name can only fit in the data if it is shortened, the name is shortened
 *          and the AD type is changed to DM_ADV_TYPE_SHORT_NAME.
 *
 *  \param  advHandle Advertising handle.
 *  \param  adType    Advertising data element type.
 *  \param  len       Length of the value.  Maximum length is 31 bytes.
 *  \param  pValue    Pointer to the value.
 *
 *  \return TRUE if the element was successfully added to the data, FALSE otherwise.
 */
/*************************************************************************************************/
bool_t appPerAdvSetAdValue(uint8_t advHandle, uint8_t adType, uint8_t len, uint8_t *pValue)
{
  uint8_t *pAdvData;
  uint16_t advDataLen;
  uint16_t advDataBufLen;
  bool_t  valueSet;

  /* get pointer and length for location */
  pAdvData = appExtSlaveCb.pPerAdvData[advHandle];
  advDataLen = appExtSlaveCb.perAdvDataLen[advHandle];
  advDataBufLen = appExtSlaveCb.perAdvDataBufLen[advHandle];

  if (pAdvData != NULL)
  {
    /* set the new element value in the advertising data */
    if (adType == DM_ADV_TYPE_LOCAL_NAME)
    {
      valueSet = DmAdvSetName(len, pValue, &advDataLen, pAdvData, advDataBufLen);
    }
    else
    {
      valueSet = DmAdvSetAdValue(adType, len, pValue, &advDataLen, pAdvData, advDataBufLen);
    }

    if (valueSet)
    {
      /* if new value set update periodic advertising data */
      appPerAdvSetData(advHandle, advDataLen, pAdvData, advDataBufLen);

      return TRUE;
    }
  }

  return FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Check if current advertising mode is extended advertising.
 *
 *  \return TRUE if extended advertising mode. FALSE, otherwise.
 */
/*************************************************************************************************/
static bool_t appSlaveExtAdvMode(void)
{
  uint8_t i;

  /* if DM extended advertising */
  if (DmAdvModeExt())
  {
    /* if first time since last power-on or reset */
    if (appSlaveCb.advStopCback == NULL)
    {
      appSlaveCb.advStopCback = appSlaveExtAdvStop;
      appSlaveCb.advRestartCback = NULL;

      /* for each advertising set */
      for (i = 0; i < DM_NUM_ADV_SETS; i++)
      {
        /* configure whether to use legacy advertising PDUs */
        DmAdvUseLegacyPdu(i, pAppExtAdvCfg->useLegacyPdu[i]);
      }

      return TRUE;
    }
  }

  if (appSlaveCb.advStopCback == appSlaveExtAdvStop)
  {
    return TRUE;
  }

  APP_TRACE_WARN0("Invalid DM advertising mode; mode configured as legacy");

  return FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Set extended advertising data.
 *
 *  \param  advHandle Advertising handle.
 *  \param  location  Data location.
 *  \param  len       Length of the data.  Maximum length is 31 bytes if advertising set uses
 *                    legacy advertising PDUs with extended advertising.
 *  \param  pData     Pointer to the data.
 *  \param  bufLen    Length of the data buffer maintained by Application.  Minimum length is
 *                    31 bytes.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppExtAdvSetData(uint8_t advHandle, uint8_t location, uint16_t len, uint8_t *pData, uint16_t bufLen)
{
  uint16_t maxLen;

  WSF_ASSERT(advHandle < DM_NUM_ADV_SETS);

  if (appSlaveExtAdvMode())
  {
    /* if advertising set uses legacy advertising PDUs with extended advertising */
    if (pAppExtAdvCfg->useLegacyPdu[advHandle])
    {
      /* maximum advertising data length supported by Controller is 31 bytes */
      maxLen = HCI_ADV_DATA_LEN;

      /* legacy advertising data length cannot exceed 31 bytes */
      if (len > HCI_ADV_DATA_LEN)
      {
        len = HCI_ADV_DATA_LEN;
      }
    }
    else
    {
      /* get maximum advertising data length supported by Controller */
      maxLen = HciGetMaxAdvDataLen();
    }

    appAdvSetData(advHandle, location, len, pData, bufLen, maxLen);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Start extended advertising using the parameters for the given advertising set and mode.
 *
 *  \param  numSets     Number of advertising sets.
 *  \param  pAdvHandles Advertising handles array.
 *  \param  mode        Discoverable/connectable mode.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppExtAdvStart(uint8_t numSets, uint8_t *pAdvHandles, uint8_t mode)
{
  uint8_t  i;
  uint16_t advInterval[DM_NUM_ADV_SETS] = {0};
  uint16_t advDuration[DM_NUM_ADV_SETS] = {0};
  uint8_t  maxEaEvents[DM_NUM_ADV_SETS] = {0};

  WSF_ASSERT(numSets <= DM_NUM_ADV_SETS);

  if (appSlaveExtAdvMode())
  {
    for (i = 0; i < numSets; i++)
    {
      uint8_t advHandle = pAdvHandles[i];

      WSF_ASSERT(advHandle < DM_NUM_ADV_SETS);

      /* initialize advertising state */
      appSlaveCb.advState[advHandle] = APP_ADV_STATE1;

      if (appSlaveCb.discMode != APP_MODE_NONE)
      {
        /* start advertising from beginning of advertising data */
        appSlaveResetAdvDataOffset(advHandle);
      }

      /* set maximum advertising data length allowed by Controller for this advertising type */
      appSlaveCb.maxAdvDataLen[advHandle] = DmExtMaxAdvDataLen(appSlaveCb.advType[advHandle],
                                              pAppExtAdvCfg->useLegacyPdu[advHandle]);

      /* build advertising parameters */
      advInterval[i] = pAppExtAdvCfg->advInterval[advHandle];
      advDuration[i] = pAppExtAdvCfg->advDuration[advHandle];
      maxEaEvents[i] = pAppExtAdvCfg->maxEaEvents[advHandle];
    }

    appSlaveAdvStart(numSets, pAdvHandles, advInterval, advDuration, maxEaEvents, TRUE, mode);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Stop extended advertising.  If the number of sets is set to 0 then all advertising
 *          sets are disabled.
 *
 *  \param  numSets   Number of advertising sets.
 *  \param  advHandle Advertising handle array.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppExtAdvStop(uint8_t numSets, uint8_t *pAdvHandles)
{
  WSF_ASSERT(numSets <= DM_NUM_ADV_SETS);

  if (appSlaveExtAdvMode())
  {
    appAdvStop(numSets, pAdvHandles);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Set the value of an advertising data element in the extended advertising or scan
 *          response data.  If the element already exists in the data then it is replaced
 *          with the new value.  If the element does not exist in the data it is appended
 *          to it, space permitting.
 *
 *          There is special handling for the device name (AD type DM_ADV_TYPE_LOCAL_NAME).
 *          If the name can only fit in the data if it is shortened, the name is shortened
 *          and the AD type is changed to DM_ADV_TYPE_SHORT_NAME.
 *
 *  \param  advHandle Advertising handle.
 *  \param  location  Data location.
 *  \param  adType    Advertising data element type.
 *  \param  len       Length of the value.  Maximum length is 31 bytes if advertising set uses
 *                    legacy advertising PDUs with extended advertising.
 *  \param  pValue    Pointer to the value.
 *
 *  \return TRUE if the element was successfully added to the data, FALSE otherwise.
 */
/*************************************************************************************************/
bool_t AppExtAdvSetAdValue(uint8_t advHandle, uint8_t location, uint8_t adType, uint8_t len,
                           uint8_t *pValue)
{
  WSF_ASSERT(advHandle < DM_NUM_ADV_SETS);

  if (appSlaveExtAdvMode())
  {
    return appAdvSetAdValue(advHandle, location, adType, len, pValue);
  }

  return FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Set extended advertising type.
 *
 *  \param  advHandle Advertising handle.
 *  \param  advType   Advertising type.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppExtSetAdvType(uint8_t advHandle, uint8_t advType)
{
  WSF_ASSERT(advHandle < DM_NUM_ADV_SETS);

  if (appSlaveExtAdvMode())
  {
    /* set maximum advertising data length allowed by Controller for this advertising type */
    appSlaveCb.maxAdvDataLen[advHandle] = DmExtMaxAdvDataLen(advType,
                                            pAppExtAdvCfg->useLegacyPdu[advHandle]);
    appSlaveCb.advState[advHandle] = APP_ADV_STATE1;
    appSetAdvType(advHandle, advType, pAppExtAdvCfg->advInterval[advHandle],
                  pAppExtAdvCfg->advDuration[advHandle], pAppExtAdvCfg->maxEaEvents[advHandle],
                  TRUE);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Set advertising peer address and its type for the given advertising set.
 *
 *  \param  advHandle     Advertising handle.
 *  \param  peerAddrType  Peer address type.
 *  \param  pPeerAddr     Peer address.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppExtSetAdvPeerAddr(uint8_t advHandle, uint8_t peerAddrType, uint8_t *pPeerAddr)
{
  WSF_ASSERT(advHandle < DM_NUM_ADV_SETS);

  appSlaveCb.peerAddrType[advHandle] = peerAddrType;
  BdaCpy(appSlaveCb.peerAddr[advHandle], pPeerAddr);
}

/*************************************************************************************************/
/*!
 *  \brief  Accept a connection to a peer device with the given address using a given advertising
 *          set.
 *
 *  \param  advHandle Advertising handle.
 *  \param  advType   Advertising type.
 *  \param  addrType  Address type.
 *  \param  pAddr     Peer device address.
 *  \param  dbHdl     Device database handle.
 *
 *  \return Connection identifier.
 */
/*************************************************************************************************/
dmConnId_t AppExtConnAccept(uint8_t advHandle, uint8_t advType, uint8_t addrType, uint8_t *pAddr,
                            appDbHdl_t dbHdl)
{
  WSF_ASSERT(advHandle < DM_NUM_ADV_SETS);

  if (appSlaveExtAdvMode())
  {
    /* set maximum advertising data length allowed by Controller for this advertising type */
    appSlaveCb.maxAdvDataLen[advHandle] = DmExtMaxAdvDataLen(advType,
                                            pAppExtAdvCfg->useLegacyPdu[advHandle]);

    /* start connectable directed advertising (advertising data is supported only with extended
       low-duty cycle connectable directed advertising) */
    return appConnAccept(advHandle, advType, pAppExtAdvCfg->advInterval[advHandle],
                         pAppExtAdvCfg->advDuration[advHandle],
                         pAppExtAdvCfg->maxEaEvents[advHandle], addrType, pAddr, dbHdl,
                         (pAppExtAdvCfg->useLegacyPdu[advHandle] ? FALSE : \
                          (advType == DM_ADV_CONN_DIRECT) ? FALSE : TRUE));
  }

  /* wrong advertising mode */
  return DM_CONN_ID_NONE;
}

/*************************************************************************************************/
/*!
 *  \brief  Set periodic advertising data.
 *
 *  \param  advHandle Advertising handle.
 *  \param  len       Length of the data.
 *  \param  pData     Pointer to the data.
 *  \param  bufLen    Length of the data buffer maintained by Application.  Minimum length is
 *                    31 bytes.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppPerAdvSetData(uint8_t advHandle, uint16_t len, uint8_t *pData, uint16_t bufLen)
{
  WSF_ASSERT(advHandle < DM_NUM_ADV_SETS);

  if (appSlaveExtAdvMode())
  {
    // set periodic advertising data
    appPerAdvSetData(advHandle, len, pData, bufLen);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Start periodic advertising for the given advertising handle.
 *
 *  \param  advHandle Advertising handle.
 *
 *  \return None.
 */
 /*************************************************************************************************/
void AppPerAdvStart(uint8_t advHandle)
{
  WSF_ASSERT(advHandle < DM_NUM_ADV_SETS);

  if (appSlaveExtAdvMode())
  {
    /* initialize periodic advertising state */
    appExtSlaveCb.perAdvState[advHandle] = APP_ADV_STATE1;

    /* if entire periodic adv data has been sent to LL */
    if (appExtSlaveCb.perAdvDataSynced[advHandle])
    {
      /* start advertising from beginning of advertising data */
      appExtSlaveCb.perAdvDataOffset[advHandle] = 0;

      /* force update of advertising data */
      appExtSlaveCb.perAdvDataSynced[advHandle] = FALSE;
    }

    /* start periodic advertising */
    appPerAdvStart(advHandle, pAppExtAdvCfg->perAdvInterval[advHandle]);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Stop periodic advertising for the given advertising handle.
 *
 *  \param  advHandle Advertising handle.
 *
 *  \return None.
 */
 /*************************************************************************************************/
void AppPerAdvStop(uint8_t advHandle)
{
  WSF_ASSERT(advHandle < DM_NUM_ADV_SETS);

  if (appSlaveExtAdvMode())
  {
    /* stop periodic advertising */
    DmPerAdvStop(advHandle);
    appExtSlaveCb.perAdvState[advHandle] = APP_ADV_STOPPED;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Set the value of an advertising data element in the periodic advertising data.
 *          If the element already exists in the data then it is replaced with the new value.
 *          If the element does not exist in the data it is appended to it, space permitting.
 *
 *          There is special handling for the device name (AD type DM_ADV_TYPE_LOCAL_NAME).
 *          If the name can only fit in the data if it is shortened, the name is shortened
 *          and the AD type is changed to DM_ADV_TYPE_SHORT_NAME.
 *
 *  \param  advHandle Advertising handle.
 *  \param  adType    Advertising data element type.
 *  \param  len       Length of the value.  Maximum length is 31 bytes if advertising set uses
 *                    legacy advertising PDUs with extended advertising.
 *  \param  pValue    Pointer to the value.
 *
 *  \return TRUE if the element was successfully added to the data, FALSE otherwise.
 */
/*************************************************************************************************/
bool_t AppPerAdvSetAdValue(uint8_t advHandle, uint8_t adType, uint8_t len, uint8_t *pValue)
{
  WSF_ASSERT(advHandle < DM_NUM_ADV_SETS);

  if (appSlaveExtAdvMode())
  {
    return appPerAdvSetAdValue(advHandle, adType, len, pValue);
  }

  return FALSE;
}
