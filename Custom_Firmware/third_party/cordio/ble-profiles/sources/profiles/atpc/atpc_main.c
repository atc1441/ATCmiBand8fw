/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Asset Tracking profile client.
 *
 *  Copyright (c) 2018-2019 Arm Ltd. All Rights Reserved.
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
#include "util/bstream.h"
#include "app_api.h"
#include "atpc_api.h"
#include "svc_cte.h"

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! Asset Tracking Profile Client connection control block. */
typedef struct
{
  uint16_t enableHandle;              /*! CTE enable attribute handle. */
  uint8_t  state;                     /*! Connection state. */
  uint8_t  length;                    /*! Min CTE length. */
  uint16_t interval;                  /*! CTE interval. */
  uint8_t cteType;                    /*! CTE type. */
  bool_t   numAntenna;                /*! Number of antenna and len of pAntennaIds in bytes. */
  uint8_t  *pAntennaIds;              /*! Array containing identifiers of antenna for this connection. */
} atpcConnCb_t;

/*! Asset Tracking Profile Server control block. */
typedef struct
{
  atpcConnCb_t connCb[DM_CONN_MAX];   /*! Connection control block. */
  uint8_t      switchSampleRates;     /*! Supported Switching Sampling Rates. */
  uint8_t      switchPatternMaxLen;   /*! Max Length of Switching Pattern. */
  uint8_t      numAntennae;           /*! Number of Antennae. */
  uint8_t      cteMaxLen;             /*! Max CTE Length. */
} atpcCb_t;

static atpcCb_t atpcCb;

/*! Constant Tone Extension enable. */
static const attcDiscChar_t atpcCteEnable =
{
  attCteEnChUuid,
  ATTC_SET_REQUIRED
};

/*! Constant Tone Extension minimum length. */
static const attcDiscChar_t atpcCteMinLen =
{
  attCteMinLenChUuid,
  ATTC_SET_REQUIRED
};

/*! Constant Tone Extension minimum transmit count. */
static const attcDiscChar_t atpcCteTxCnt =
{
  attCteTxCntChUuid,
  ATTC_SET_REQUIRED
};

/*! Constant Tone Extension transmit duration. */
static const attcDiscChar_t atpcCteDurationc =
{
  attCteTxDurChUuid,
  ATTC_SET_REQUIRED
};

/*! Constant Tone Extension interval. */
static const attcDiscChar_t atpcCteInterval =
{
  attCteIntChUuid,
  ATTC_SET_REQUIRED
};

/*! Constant Tone Extension PHY. */
static const attcDiscChar_t atpcCtePhy =
{
  attCtePhyChUuid,
  ATTC_SET_REQUIRED
};

/*! List of characteristics to be discovered; order matches handle index enumeration. */
static const attcDiscChar_t *atpcCteDiscCharList[] =
{
  &atpcCteEnable,                   /*! Constant Tone Extension enable. */
  &atpcCteMinLen,                   /*! Constant Tone Extension minimum length. */
  &atpcCteTxCnt,                    /*! Constant Tone Extension minimum transmit count. */
  &atpcCteDurationc,                /*! Constant Tone Extension transmit duration. */
  &atpcCteInterval,                 /*! Constant Tone Extension interval. */
  &atpcCtePhy,                      /*! Constant Tone Extension PHY. */
};

/*! sanity check:  make sure handle list length matches characteristic list length. */
WSF_CT_ASSERT(ATPC_CTE_HDL_LIST_LEN == ((sizeof(atpcCteDiscCharList) / sizeof(attcDiscChar_t *))));

/*************************************************************************************************/
/*!
 *  \brief  Called by application to notify the profile of System Events.
 *
 *  \param  pEvt   Pointer to the Event.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void atpcProcWriteRsp(attEvt_t *pEvt)
{
  dmConnId_t connId = (dmConnId_t) pEvt->hdr.param;

  if ((pEvt->hdr.status == ATT_SUCCESS) && (pEvt->handle == atpcCb.connCb[connId-1].enableHandle))
  {
    atpcConnCb_t *pCcb = &atpcCb.connCb[connId-1];

    switch (DmConnCteGetReqState(connId))
    {
      case DM_CONN_CTE_STATE_IDLE:
        DmConnCteRxSampleStart(connId, HCI_CTE_SLOT_DURATION_2_US, pCcb->numAntenna, pCcb->pAntennaIds);
        break;

      case DM_CONN_CTE_STATE_INITIATING:
        DmConnCteRxSampleStop(connId);
        break;

      default:
        APP_TRACE_WARN0("ATPC CTE enable response in unexpected state.");
        break;
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for Constant Tone Extension service.
 *          Parameter pHdlList must point to an array of length \ref ATPC_CTE_HDL_LIST_LEN.
 *          If discovery is successful the handles of discovered characteristics and
 *          descriptors will be set in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcCteDiscover(dmConnId_t connId, uint16_t *pHdlList)
{
  AppDiscFindService(connId, ATT_16_UUID_LEN, (uint8_t *) attCteSvcUuid,
                     ATPC_CTE_HDL_LIST_LEN, (attcDiscChar_t **) atpcCteDiscCharList, pHdlList);
}

/*************************************************************************************************/
/*!
 *  \brief  Write to the Constant Tone Extension enable attribute.
 *
 *  \param  connId    Connection identifier.
 *  \param  handle    Attribute handle.
 *  \param  enable    Enable.
 *  \param  cteType   CTE Type.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcCteWriteEnable(dmConnId_t connId, uint16_t handle, uint8_t enable, uint8_t cteType)
{
  WSF_ASSERT(handle != ATT_HANDLE_NONE);

  uint8_t data[2] = {0};

  data[0] = enable;
  data[1] = cteType;

  AttcWriteReq(connId, handle, 2, data);
}

/*************************************************************************************************/
/*!
 *  \brief  Write to the Constant Tone Extension minimum length attribute.
 *
 *  \param  connId    Connection identifier.
 *  \param  handle    Attribute handle.
 *  \param  minLen    Minimum length.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcCteWriteMinLen(dmConnId_t connId, uint16_t handle, uint8_t minLen)
{
  WSF_ASSERT(handle != ATT_HANDLE_NONE);

  AttcWriteReq(connId, handle, sizeof(uint8_t), &minLen);
}

/*************************************************************************************************/
/*!
 *  \brief  Write to the Constant Tone Extension minimum transmit count attribute.
 *
 *  \param  connId    Connection identifier.
 *  \param  handle    Attribute handle.
 *  \param  txCount   Minimum transmit count.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcCteWriteMinTxCount(dmConnId_t connId, uint16_t handle, uint8_t txCount)
{
  WSF_ASSERT(handle != ATT_HANDLE_NONE);

  AttcWriteReq(connId, handle, sizeof(uint8_t), &txCount);
}

/*************************************************************************************************/
/*!
 *  \brief  Write to the Constant Tone Extension transmit duration attribute.
 *
 *  \param  connId    Connection identifier.
 *  \param  handle    Attribute handle.
 *  \param  duration  Transmit duration.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcCteWriteTxDuration(dmConnId_t connId, uint16_t handle, uint8_t duration)
{
  WSF_ASSERT(handle != ATT_HANDLE_NONE);

  AttcWriteReq(connId, handle, sizeof(uint8_t), &duration);
}

/*************************************************************************************************/
/*!
 *  \brief  Write to the Constant Tone Extension interval attribute.
 *
 *  \param  connId    Connection identifier.
 *  \param  handle    Attribute handle.
 *  \param  interval  Interval.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcCteWriteInterval(dmConnId_t connId, uint16_t handle, uint16_t interval)
{
  WSF_ASSERT(handle != ATT_HANDLE_NONE);

  uint8_t buf[ATT_DEFAULT_PAYLOAD_LEN];
  uint8_t *p = buf;

  UINT16_TO_BSTREAM(p, interval);

  AttcWriteReq(connId, handle, sizeof(uint16_t), buf);
}

/*************************************************************************************************/
/*!
 *  \brief  Write to the Constant Tone Extension PHY attribute.
 *
 *  \param  connId    Connection identifier.
 *  \param  handle    Attribute handle.
 *  \param  phy       PHY.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcCteWritePhy(dmConnId_t connId, uint16_t handle, uint8_t phy)
{
  WSF_ASSERT(handle != ATT_HANDLE_NONE)

  AttcWriteReq(connId, handle, sizeof(uint8_t), &phy);
}

/*************************************************************************************************/
/*!
 *  \brief  Enable AoA CTE Rx request over ACL.
 *
 *  \param  connId    Connection identifier.
 *  \param  handle    CTE enable attribute handle.
 *  \param  length    Request length.
 *  \param  interval  Request interval.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcCteAclEnableReq(dmConnId_t connId, uint16_t handle, uint8_t length, uint16_t interval,
                           uint8_t cteType)
{
  atpcConnCb_t *pCcb;

  WSF_ASSERT((connId > DM_CONN_ID_NONE) && (connId <= DM_CONN_MAX));
  WSF_ASSERT(handle != ATT_HANDLE_NONE)

  if (DmConnCteGetReqState(connId) == DM_CONN_CTE_STATE_IDLE)
  {
    /* Store the enable handle and CTE configuration */
    pCcb = &atpcCb.connCb[connId-1];
    pCcb->enableHandle = handle;
    pCcb->length = length;
    pCcb->interval = interval;
    pCcb->cteType = cteType;

  AtpcCteWriteEnable(connId, handle, CTE_ENABLE_ACL_BIT, cteType);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Disable AoA CTE Rx request over ACL.
 *
 *  \param  connId    Connection identifier.
 *  \param  handle    CTE enable attribute handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcCteAclDisableReq(dmConnId_t connId, uint16_t handle)
{
  atpcConnCb_t *pCcb;

  WSF_ASSERT((connId > DM_CONN_ID_NONE) && (connId <= DM_CONN_MAX));
  WSF_ASSERT(handle != ATT_HANDLE_NONE)

  if (DmConnCteGetReqState(connId) == DM_CONN_CTE_STATE_INITIATING)
  {
    /* Store the enable handle */
    pCcb = &atpcCb.connCb[connId-1];
    pCcb->enableHandle = handle;

  AtpcCteWriteEnable(connId, handle, CTE_ENABLE_NONE, 0);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Set the antenna identifiers for a connection ID.
 *
 *  \param  connId        Connection identifier.
 *  \param  numAntenna    Number of antenna and len of pAntennaIds in bytes.
 *  \param  pAntennaIds   Array containing identifiers of antenna for this connection.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcSetAntennaIds(dmConnId_t connId, uint8_t numAntenna, uint8_t *pAntennaIds)
{
  atpcConnCb_t *pCcb;

  WSF_ASSERT((connId > DM_CONN_ID_NONE) && (connId <= DM_CONN_MAX));
  WSF_ASSERT(numAntenna <= atpcCb.numAntennae);

  pCcb = &atpcCb.connCb[connId-1];
  pCcb->numAntenna = numAntenna;
  pCcb->pAntennaIds = pAntennaIds;
}

/*************************************************************************************************/
/*!
 *  \brief  Called by application to notify the ATPC of System Events.
 *
 *  \param  pEvt   Pointer to the Event.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpcProcMsg(wsfMsgHdr_t *pEvt)
{
  atpcConnCb_t *pCcb = &atpcCb.connCb[pEvt->param - 1];
  dmEvt_t *pDmEvt = (dmEvt_t*) pEvt;

  switch (pEvt->event)
  {
    case DM_RESET_CMPL_IND:
      DmReadAntennaInfo();
      break;

    case DM_READ_ANTENNA_INFO_IND:
      atpcCb.cteMaxLen = pDmEvt->readAntennaInfo.cteMaxLen;
      atpcCb.switchSampleRates = pDmEvt->readAntennaInfo.switchSampleRates;
      atpcCb.numAntennae = pDmEvt->readAntennaInfo.numAntennae;
      atpcCb.switchPatternMaxLen = pDmEvt->readAntennaInfo.switchPatternMaxLen;
      break;

    case ATTC_WRITE_RSP:
      atpcProcWriteRsp((attEvt_t *)pEvt);
      break;

    case DM_CONN_CTE_RX_SAMPLE_START_IND:
      if (pEvt->status == HCI_SUCCESS)
      {
        //DmConnCteReqStart((dmConnId_t) pEvt->param, pCcb->interval, pCcb->length, HCI_CTE_TYPE_REQ_AOA);
        DmConnCteReqStart((dmConnId_t) pEvt->param, pCcb->interval, pCcb->length, pCcb->cteType);
      }
      break;

    case DM_CONN_CTE_RX_SAMPLE_STOP_IND:
      if (pEvt->status == HCI_SUCCESS)
      {
        DmConnCteReqStop((dmConnId_t) pEvt->param);
      }
      break;

    case DM_CTE_REQ_FAIL_IND:
      break;

    default:
      break;
  }
}
