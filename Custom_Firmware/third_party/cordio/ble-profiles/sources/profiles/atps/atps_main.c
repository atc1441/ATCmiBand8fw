/*!
 *  \file
 *
 *  \brief  Asset Tracking profile server.
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
#include "hci_defs.h"
#include "app_api.h"
#include "atps_api.h"
#include "svc_cte.h"

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! Asset Tracking Profile Server connection control block. */
typedef struct
{
  uint8_t  state;                     /*! Connection state. */
  uint8_t  enableBits;                /*! Constant Tone Extension enable. */
  uint8_t  minLen;                    /*! Constant Tone Extension minimum length. */
  uint8_t  minTxCount;                /*! Constant Tone Extension minimum transmit count. */
  uint8_t  duration;                  /*! Constant Tone Extension transmit duration. */
  uint16_t interval;                  /*! Constant Tone Extension interval. */
  uint8_t  phyType;                   /*! Constant Tone Extension PHY type. */
  bool_t   numAntenna;                /*! Number of antenna and len of pAntennaIds in bytes. */
  uint8_t  *pAntennaIds;              /*! Array containing identifiers of antenna for this connection. */
} atpsConnCb_t;

/*! Asset Tracking Profile Server control block. */
typedef struct
{
  atpsConnCb_t connCb[DM_CONN_MAX];   /*! Connection control block. */
  uint8_t      switchSampleRates;     /*! Supported Switching Sampling Rates. */
  uint8_t      switchPatternMaxLen;   /*! Max Length of Switching Pattern. */
  uint8_t      numAntennae;           /*! Number of Antennae. */
  uint8_t      cteMaxLen;             /*! Max CTE Length. */
} atpsCb_t;

static atpsCb_t atpsCb;

/*************************************************************************************************/
/*!
 *  \brief  Process write enable attribute from locator.
 *
 *  \param  connId        Connection identifier.
 *  \param  enableBits    Enable bitfield value from client.
 *  \param  cteType       CTE type for transmission
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t atpsCteSetEnable(dmConnId_t connId, uint8_t enableBits, uint8_t cteType)
{
  atpsConnCb_t *pCcb;
  bool_t enableAcl = !!(enableBits & CTE_ENABLE_ACL_BIT);

  if (enableBits & CTE_ENABLE_ADV_BIT)
  {
    /* Connectionless advertising CTE not supported. */
    return ATT_ERR_WRITE_REJ;
  }

  WSF_ASSERT((connId > DM_CONN_ID_NONE) && (connId <= DM_CONN_MAX));

  pCcb = &atpsCb.connCb[connId-1];

  if (enableAcl)
  {
    if (DmConnCteGetRspState(connId) == DM_CONN_CTE_STATE_IDLE)
    {
      //DmConnCteTxConfig(connId, HCI_CTE_TYPE_PERMIT_AOA_RSP_BIT, pCcb->numAntenna, pCcb->pAntennaIds);
      DmConnCteTxConfig(connId, 1<<cteType, pCcb->numAntenna, pCcb->pAntennaIds);

      /* Delay write response */
      return ATT_RSP_PENDING;
    }
  }
  else
  {
    if (DmConnCteGetRspState(connId) == DM_CONN_CTE_STATE_RESPONDING)
    {
      DmConnCteRspStop(connId);

      /* Delay write response */
      return ATT_RSP_PENDING;
    }
  }

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Process write minimum length attribute from locator.
 *
 *  \param  connId        Connection identifier.
 *  \param  minLen        Minimum length value.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t atpsCteSetMinLen(dmConnId_t connId, uint8_t minLen)
{
  atpsConnCb_t *pCcb;

  WSF_ASSERT((connId > DM_CONN_ID_NONE) && (connId <= DM_CONN_MAX));

  if ((minLen < CTE_MIN_MIN_LEN) || (minLen > CTE_MAX_MIN_LEN) || (minLen > atpsCb.cteMaxLen))
  {
    return ATT_ERR_RANGE;
  }

  pCcb = &atpsCb.connCb[connId-1];
  pCcb->minLen = minLen;

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Process write advertising minimum tx count attribute from locator.
 *
 *  \param  connId        Connection identifier.
 *  \param  minLen        Minimum tx count value.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t atpsCteSetAdvMinTxCnt(dmConnId_t connId, uint8_t minTxCount)
{
  atpsConnCb_t *pCcb;

  WSF_ASSERT((connId > DM_CONN_ID_NONE) && (connId <= DM_CONN_MAX));

  if ((minTxCount < CTE_MIN_MIN_TX_CNT) || (minTxCount > CTE_MAX_MIN_TX_CNT))
  {
    return ATT_ERR_RANGE;
  }

  pCcb = &atpsCb.connCb[connId-1];
  pCcb->minTxCount = minTxCount;

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Process write advertising minimum tx duration attribute from locator.
 *
 *  \param  connId        Connection identifier.
 *  \param  duration      Minimum tx count value.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t atpsCteSetAdvTxDuration(dmConnId_t connId, uint8_t duration)
{
  atpsConnCb_t *pCcb;

  WSF_ASSERT((connId > DM_CONN_ID_NONE) && (connId <= DM_CONN_MAX));

  pCcb = &atpsCb.connCb[connId-1];
  pCcb->duration = duration;

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Process write advertising intergval attribute from locator.
 *
 *  \param  connId        Connection identifier.
 *  \param  interval      Interval value.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t atpsCteSetAdvInterval(dmConnId_t connId, uint8_t interval)
{
  atpsConnCb_t *pCcb;

  WSF_ASSERT((connId > DM_CONN_ID_NONE) && (connId <= DM_CONN_MAX));

  if (interval < CTE_MIN_INTERVAL)
  {
    return ATT_ERR_RANGE;
  }

  pCcb = &atpsCb.connCb[connId-1];
  pCcb->interval = interval;

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Process write advertising PHY attribute from locator.
 *
 *  \param  connId        Connection identifier.
 *  \param  phyType       PHY type.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t atpsCteSetAdvPhy(dmConnId_t connId, uint8_t phyType)
{
  atpsConnCb_t *pCcb;

  WSF_ASSERT((connId > DM_CONN_ID_NONE) && (connId <= DM_CONN_MAX));

  if (phyType > CTE_PHY_2M)
  {
    return ATT_ERR_RANGE;
  }

  pCcb = &atpsCb.connCb[connId-1];
  pCcb->phyType = phyType;

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  ATTS write callback for Continuous Tone Extension, CTE, service.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t atpsCteWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                                 uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr)
{
  uint8_t status;

  /* Only individual write requests with response are supported. */
  if (operation != ATT_PDU_WRITE_REQ)
  {
    return ATT_ERR_NOT_SUP;
  }

  switch (handle)
  {
    case CTE_ENABLE_HDL:
      status = atpsCteSetEnable(connId, *pValue, *(pValue+1));
      break;

    case CTE_MIN_LEN_HDL:
      status = atpsCteSetMinLen(connId, *pValue);
      break;

    case CTE_ADV_MIN_TX_CNT_HDL:
      status = atpsCteSetAdvMinTxCnt(connId, *pValue);
      break;

    case CTE_ADV_TX_DURATION_HDL:
      status = atpsCteSetAdvTxDuration(connId, *pValue);
      break;

    case CTE_ADV_INTERVAL_HDL:
      status = atpsCteSetAdvInterval(connId, *pValue);
      break;

    case CTE_ADV_EXT_PHY_HDL:
      status = atpsCteSetAdvPhy(connId, *pValue);
      break;

    default:
      status = ATT_ERR_HANDLE;
      break;
  }

  return status;
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
void AtpsSetAntennaIds(dmConnId_t connId, uint8_t numAntenna, uint8_t *pAntennaIds)
{
  atpsConnCb_t *pCcb;

  WSF_ASSERT((connId > DM_CONN_ID_NONE) && (connId <= DM_CONN_MAX));
  WSF_ASSERT(numAntenna <= atpsCb.numAntennae);

  pCcb = &atpsCb.connCb[connId-1];
  pCcb->numAntenna = numAntenna;
  pCcb->pAntennaIds = pAntennaIds;
}

/*************************************************************************************************/
/*!
 *  \brief  Called by application to notify the Asset Tracking Profile server of DM Events.
 *
 *  \param  pEvt   Pointer to the DM Event.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpsProcDmMsg(dmEvt_t *pEvt)
{
  uint8_t status;
  atpsConnCb_t *pCcb = &atpsCb.connCb[pEvt->hdr.param - 1];

  switch (pEvt->hdr.event)
  {
    case DM_RESET_CMPL_IND:
      DmReadAntennaInfo();
      break;

    case DM_READ_ANTENNA_INFO_IND:
      atpsCb.cteMaxLen = pEvt->readAntennaInfo.cteMaxLen;
      atpsCb.switchSampleRates = pEvt->readAntennaInfo.switchSampleRates;
      atpsCb.numAntennae = pEvt->readAntennaInfo.numAntennae;
      atpsCb.switchPatternMaxLen = pEvt->readAntennaInfo.switchPatternMaxLen;
      break;

    case DM_CONN_CTE_TX_CFG_IND:
      if (pEvt->hdr.status == HCI_SUCCESS)
      {
        DmConnCteRspStart((dmConnId_t) pEvt->hdr.param);
      }
      else
      {
        AttsContinueWriteReq((dmConnId_t) pEvt->hdr.param, CTE_ENABLE_HDL, ATT_ERR_WRITE_REJ);
      }
      break;

    case DM_CONN_CTE_RSP_START_IND:
      status = (pEvt->hdr.status == HCI_SUCCESS) ? ATT_SUCCESS : ATT_ERR_WRITE_REJ;
      AttsContinueWriteReq((dmConnId_t) pEvt->hdr.param, CTE_ENABLE_HDL, status);

      if (status == ATT_SUCCESS)
      {
        pCcb->enableBits |= CTE_ENABLE_ACL_BIT;
      }
      break;

    case DM_CONN_CTE_RSP_STOP_IND:
      status = (pEvt->hdr.status == HCI_SUCCESS) ? ATT_SUCCESS : ATT_ERR_WRITE_REJ;
      AttsContinueWriteReq((dmConnId_t) pEvt->hdr.param, CTE_ENABLE_HDL, status);

      if (status == ATT_SUCCESS)
      {
        pCcb->enableBits &= ~CTE_ENABLE_ACL_BIT;
      }
      break;

    case DM_CTE_REQ_FAIL_IND:
      break;

    default:
      break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Asset Tracking Profile server initialization.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AtpsInit(void)
{
  SvcCteCbackRegister(atpsCteWriteCback);
  SvcCteAddGroup();
}
