/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Device information service client.
 *
 *  Copyright (c) 2012-2020 Arm Ltd. All Rights Reserved.
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
#include "wsf_assert.h"
#include "wsf_trace.h"
#include "svc_ch.h"
#include "app_api.h"
#include "dis_api.h"

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! DIS characteristics for discovery */

/*! Manufacturer name string */
static const attcDiscChar_t disMfns =
{
  attMfnsChUuid,
  0
};

/*! Model number string */
static const attcDiscChar_t disMns =
{
  attMnsChUuid,
  0
};

/*! Serial number string */
static const attcDiscChar_t disSns =
{
  attSnsChUuid,
  0
};

/*! Hardware revision string */
static const attcDiscChar_t disHrs =
{
  attHrsChUuid,
  0
};

/*! Firmware revision string */
static const attcDiscChar_t disFrs =
{
  attFrsChUuid,
  0
};

/*! Software revision string */
static const attcDiscChar_t disSrs =
{
  attSrsChUuid,
  0
};

/*! System ID */
static const attcDiscChar_t disSid =
{
  attSidChUuid,
  0
};

/*! IEEE 11073-20601 regulatory certificate data  */
static const attcDiscChar_t disRcd =
{
  attIeeeChUuid,
  0
};

/*! PnP ID */
static const attcDiscChar_t disPnpId =
{
  attPnpChUuid,
  0
};

/*! List of characteristics to be discovered; order matches handle index enumeration  */
static const attcDiscChar_t *disDiscCharList[] =
{
  &disMfns,             /*! Manufacturer name string */
  &disMns,              /*! Model number string */
  &disSns,              /*! Serial number string */
  &disHrs,              /*! Hardware revision string */
  &disFrs,              /*! Firmware revision string */
  &disSrs,              /*! Software revision string */
  &disSid,              /*! System ID */
  &disRcd,              /*! IEEE 11073-20601 regulatory certificate data */
  &disPnpId             /*! PnP ID */
};

/* sanity check:  make sure handle list length matches characteristic list length */
WSF_CT_ASSERT(DIS_HDL_LIST_LEN == ((sizeof(disDiscCharList) / sizeof(attcDiscChar_t *))));

/*************************************************************************************************/
/*!
 *  \brief  Format a string for printing.
 *
 *  \param  pValue    Pointer to buffer containing value.
 *  \param  len       length of buffer.
 *
 *  \return Buffer containing string.
 */
/*************************************************************************************************/
char *disFmtString(uint8_t *pValue, uint16_t len)
{
  static char buf[ATT_DEFAULT_PAYLOAD_LEN + 1];

  len = (len < (sizeof(buf) - 1)) ? len : (sizeof(buf) - 1);

  memcpy(buf, pValue, len);
  buf[len] = '\0';

  return buf;
}

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for DIS service.  Parameter pHdlList
 *          must point to an array of length DIS_HDL_LIST_LEN.  If discovery is successful
 *          the handles of discovered characteristics and descriptors will be set in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void DisDiscover(dmConnId_t connId, uint16_t *pHdlList)
{
  AppDiscFindService(connId, ATT_16_UUID_LEN, (uint8_t *) attDisSvcUuid,
                     DIS_HDL_LIST_LEN, (attcDiscChar_t **) disDiscCharList, pHdlList);
}

/*************************************************************************************************/
/*!
 *  \brief  Process a value received in an ATT read response, notification, or indication
 *          message.  Parameter pHdlList must point to an array of length DIS_HDL_LIST_LEN.
 *          If the attribute handle of the message matches a handle in the handle list the value
 *          is processed, otherwise it is ignored.
 *
 *  \param  pHdlList  Characteristic handle list.
 *  \param  pMsg      ATT callback message.
 *
 *  \return ATT_SUCCESS if handle is found, ATT_ERR_NOT_FOUND otherwise.
 */
/*************************************************************************************************/
uint8_t DisValueUpdate(uint16_t *pHdlList, attEvt_t *pMsg)
{
  uint8_t status = ATT_SUCCESS;

  /* manufacturer name string */
  if (pMsg->handle == pHdlList[DIS_MFNS_HDL_IDX])
  {
    APP_TRACE_INFO1("Mfgr: %s", disFmtString(pMsg->pValue, pMsg->valueLen));
  }
  /* model number string */
  else if (pMsg->handle == pHdlList[DIS_MNS_HDL_IDX])
  {
    APP_TRACE_INFO1("Model num: %s", disFmtString(pMsg->pValue, pMsg->valueLen));
  }
  /* serial number string */
  else if (pMsg->handle == pHdlList[DIS_SNS_HDL_IDX])
  {
    APP_TRACE_INFO1("Serial num: %s", disFmtString(pMsg->pValue, pMsg->valueLen));
  }
  /* hardware revision string */
  else if (pMsg->handle == pHdlList[DIS_HRS_HDL_IDX])
  {
    APP_TRACE_INFO1("Hardware rev: %s", disFmtString(pMsg->pValue, pMsg->valueLen));
  }
  /* firmware revision string */
  else if (pMsg->handle == pHdlList[DIS_FRS_HDL_IDX])
  {
    APP_TRACE_INFO1("Firmware rev: %s", disFmtString(pMsg->pValue, pMsg->valueLen));
  }
  /* software revision string */
  else if (pMsg->handle == pHdlList[DIS_SRS_HDL_IDX])
  {
    APP_TRACE_INFO1("Software rev: %s", disFmtString(pMsg->pValue, pMsg->valueLen));
  }
  /* system id */
  else if (pMsg->handle == pHdlList[DIS_SID_HDL_IDX])
  {
    if (pMsg->valueLen == CH_SYSTEM_ID_LEN)
    {
      APP_TRACE_INFO0("System ID read ok");
    }
  }
  /* IEEE 11073-20601 regulatory certificate data */
  else if (pMsg->handle == pHdlList[DIS_RCD_HDL_IDX])
  {
    APP_TRACE_INFO1("Regulatory certificate data read (length: %d)", pMsg->valueLen);
  }
  /* PnP ID */
  else if (pMsg->handle == pHdlList[DIS_PNP_ID_HDL_IDX])
  {
    if (pMsg->valueLen == CH_PNP_ID_LEN)
    {
      APP_TRACE_INFO0("PnP ID read ok");
    }
  }
  /* handle not found in list */
  else
  {
    status = ATT_ERR_NOT_FOUND;
  }

  return status;
}
