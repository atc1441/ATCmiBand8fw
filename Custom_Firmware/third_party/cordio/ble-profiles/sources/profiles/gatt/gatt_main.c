/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  GATT profile.
 *
 *  Copyright (c) 2011-2019 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2019-2020 Packetcraft, Inc.
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
#include "wsf_trace.h"
#include "wsf_assert.h"
#include "util/bstream.h"
#include "app_api.h"
#include "gatt_api.h"
#include "svc_core.h"
#include "att_api.h"

/**************************************************************************************************
Data Types
**************************************************************************************************/

/* Control block. */
typedef struct
{
  bool_t  svcChangedCccdIdxSet; /* Check if Service Changed CCCD index has been initialized. */
  uint8_t svcChangedCccdIdx;    /* Stored index of Service Changed CCCD. */
} gattServCb_t;

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/* Control block. */
gattServCb_t gattServCb;

/*! GATT service characteristics for discovery */

/*! Service changed */
static const attcDiscChar_t gattSc =
{
  attScChUuid,
  0
};

/*! Service changed client characteristic configuration descriptor */
static const attcDiscChar_t gattScCcc =
{
  attCliChCfgUuid,
  ATTC_SET_DESCRIPTOR
};

/*! Client supported features */
static const attcDiscChar_t gattCsf =
{
  attGattCsfChUuid,
  0
};

/*! List of characteristics to be discovered; order matches handle index enumeration  */
static const attcDiscChar_t *gattDiscCharList[] =
{
  &gattSc,                    /* Service changed */
  &gattScCcc,                 /* Service changed client characteristic configuration descriptor */
  &gattCsf                    /* Client supported features */
};

/* sanity check:  make sure handle list length matches characteristic list length */
WSF_CT_ASSERT(GATT_HDL_LIST_LEN == ((sizeof(gattDiscCharList) / sizeof(attcDiscChar_t *))));

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for GATT service.  Parameter pHdlList
 *          must point to an array of length GATT_HDL_LIST_LEN.  If discovery is successful
 *          the handles of discovered characteristics and descriptors will be set in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void GattDiscover(dmConnId_t connId, uint16_t *pHdlList)
{
  AppDiscFindService(connId, ATT_16_UUID_LEN, (uint8_t *) attGattSvcUuid,
                     GATT_HDL_LIST_LEN, (attcDiscChar_t **) gattDiscCharList, pHdlList);
}

/*************************************************************************************************/
/*!
 *  \brief  Process a value received in an ATT read response, notification, or indication
 *          message.  Parameter pHdlList must point to an array of length GATT_HDL_LIST_LEN.
 *          If the attribute handle of the message matches a handle in the handle list the value
 *          is processed, otherwise it is ignored.
 *
 *  \param  pHdlList  Characteristic handle list.
 *  \param  pMsg      ATT callback message.
 *
 *  \return ATT_SUCCESS if handle is found, ATT_ERR_NOT_FOUND otherwise.
 */
/*************************************************************************************************/
uint8_t GattValueUpdate(uint16_t *pHdlList, attEvt_t *pMsg)
{
  uint8_t status = ATT_SUCCESS;

  /* service changed */
  if (pMsg->handle == pHdlList[GATT_SC_HDL_IDX])
  {
    /* perform service changed */
    AppDiscServiceChanged(pMsg);
  }
  /* handle not found in list */
  else
  {
    status = ATT_ERR_NOT_FOUND;
  }

  return status;
}

/*************************************************************************************************/
/*!
 *  \brief  Set Index of the Service Changed CCCD in the ATT Server.
 *
 *  \param  idx  Index of the Service Changed CCCD in the ATT Server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void GattSetSvcChangedIdx(uint8_t idx)
{
  gattServCb.svcChangedCccdIdxSet = TRUE;
  gattServCb.svcChangedCccdIdx = idx;
}

/*************************************************************************************************/
/*!
 *  \brief  Send Service Change Indications to the specified connections if they are configured to
 *          do so.
 *
 *  \param  connId    DM Connection identifier or \ref DM_CONN_ID_NONE to send to all connections.
 *  \param  start     start handle for service changed value.
 *  \param  end       end handle for service changed value.
 *
 *  \return None.
 */
/*************************************************************************************************/
void GattSendServiceChangedInd(dmConnId_t connId, uint16_t start, uint16_t end)
{
  uint8_t svcChangedValues[4];
  uint8_t *p;

  if (!gattServCb.svcChangedCccdIdxSet)
  {
    return;
  }

  p = svcChangedValues;
  UINT16_TO_BSTREAM(p, start);
  UINT16_TO_BSTREAM(p, end);

  /* If connection is not specified */
  if (connId == DM_CONN_ID_NONE)
  {
    /* Send to all. */
    for (connId = 1; connId <= DM_CONN_MAX; connId++)
    {
      if (AttsCccEnabled(connId, gattServCb.svcChangedCccdIdx))
      {
        AttsHandleValueInd(connId, GATT_SC_HDL, sizeof(svcChangedValues), svcChangedValues);
      }
    }
  }
  else
  {
    /* Send to only this one. */
    if (AttsCccEnabled(connId, gattServCb.svcChangedCccdIdx))
    {
      AttsHandleValueInd(connId, GATT_SC_HDL, sizeof(svcChangedValues), svcChangedValues);
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  ATTS read callback for gatt service.
 *
 *  \param  connId     Connection identifier.
 *  \param  handle     ATT handle.
 *  \param  operation  Operation selected.
 *  \param  offset     Offset to begin read from.
 *  \param  pAttr      Attribute to read from.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t GattReadCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                      uint16_t offset, attsAttr_t *pAttr)
{
  switch (handle)
  {
    case GATT_CSF_HDL:
    {
      uint8_t csf[ATT_CSF_LEN];

      AttsCsfGetFeatures(connId, csf, sizeof(csf));
      memcpy(pAttr->pValue, csf, ATT_CSF_LEN);
    }
    break;

    default:
      break;
  }

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  ATTS write callback for gatt service.
 *
 *  \param  connId     Connection identifier.
 *  \param  handle     ATT handle.
 *  \param  operation  Operation selected.
 *  \param  offset     Offset to begin write.
 *  \param  len        Length of write.
 *  \param  pValue     Pointer to buffer to write.
 *  \param  pAttr      Attribute to write to.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t GattWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                       uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr)
{
  uint8_t status;

  switch (handle)
  {
    case GATT_CSF_HDL:
      status = AttsCsfWriteFeatures(connId, offset, len, pValue);
      break;

    default:
      status = ATT_SUCCESS;
      break;
  }

  return status;
}
