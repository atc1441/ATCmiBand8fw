/*************************************************************************************************/
/*!
 *  \file   hci_core_ps.c
 *
 *  \brief  HCI core platform-specific module for dual-chip.
 *
 *          $Date: 2016-12-28 16:12:14 -0600 (Wed, 28 Dec 2016) $
 *          $Revision: 10805 $
 *
 *  Copyright (c) 2009-2017 ARM Ltd., all rights reserved.
 *  ARM Ltd. confidential and proprietary.
 *
 *  IMPORTANT.  Your use of this file is governed by a Software License Agreement
 *  ("Agreement") that must be accepted in order to download or otherwise receive a
 *  copy of this file.  You may not use or copy this file for any purpose other than
 *  as described in the Agreement.  If you do not agree to all of the terms of the
 *  Agreement do not use this file and delete all copies in your possession or control;
 *  if you do not have a copy of the Agreement, you must contact ARM Ltd. prior
 *  to any use, copying or further distribution of this software.
 */
/*************************************************************************************************/

#include <string.h>
#include "wsf_types.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "bda.h"
#include "bstream.h"
#include "hci_core.h"
#include "hci_tr.h"
#include "hci_cmd.h"
#include "hci_evt.h"
#include "hci_api.h"
#include "hci_main.h"

/*************************************************************************************************/
/*!
 *  \fn     hciCoreInit
 *
 *  \brief  HCI core initialization.
 *
 *  \return None.
 */
/*************************************************************************************************/
void hciCoreInit(void)
{
  hciCmdInit();
}

/*************************************************************************************************/
/*!
 *  \fn     hciCoreNumCmplPkts
 *
 *  \brief  Handle an HCI Number of Completed Packets event.
 *
 *  \param  pMsg    Message containing the HCI Number of Completed Packets event.
 *
 *  \return None.
 */
/*************************************************************************************************/
void hciCoreNumCmplPkts(uint8_t *pMsg)
{
  uint8_t         numHandles;
  uint16_t        bufs;
  uint16_t        handle;
  uint8_t         availBufs = 0;
  hciCoreConn_t   *pConn;

  /* parse number of handles */
  BSTREAM_TO_UINT8(numHandles, pMsg);

  /* for each handle in event */
  while (numHandles-- > 0)
  {
    /* parse handle and number of buffers */
    BSTREAM_TO_UINT16(handle, pMsg);
    BSTREAM_TO_UINT16(bufs, pMsg);

    if ((pConn = hciCoreConnByHandle(handle)) != NULL)
    {
      /* decrement outstanding buffer count to controller */
      pConn->outBufs -= (uint8_t) bufs;

      /* decrement queued buffer count for this connection */
      pConn->queuedBufs -= (uint8_t) bufs;

      /* increment available buffer count */
      availBufs += (uint8_t) bufs;

      /* call flow control callback */
      if (pConn->flowDisabled && pConn->queuedBufs <= hciCoreCb.aclQueueLo)
      {
        pConn->flowDisabled = FALSE;
        (*hciCb.flowCback)(handle, FALSE);
      }
    }
  }

  /* service TX data path */
  hciCoreTxReady(availBufs);
}

/*************************************************************************************************/
/*!
 *  \fn     hciCoreRecv
 *
 *  \brief  Send a received HCI event or ACL packet to the HCI event handler.
 *
 *  \param  msgType       Message type:  HCI_ACL_TYPE or HCI_EVT_TYPE.
 *  \param  pCoreRecvMsg  Pointer to received message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void hciCoreRecv(uint8_t msgType, uint8_t *pCoreRecvMsg)
{
  /* dump event for protocol analysis */
  if (msgType == HCI_EVT_TYPE)
  {
    HCI_PDUMP_EVT(*(pCoreRecvMsg + 1) + HCI_EVT_HDR_LEN, pCoreRecvMsg);
  }
  else if (msgType == HCI_ACL_TYPE)
  {
    HCI_PDUMP_RX_ACL(*(pCoreRecvMsg + 2) + HCI_ACL_HDR_LEN, pCoreRecvMsg);
  }
  else if (msgType == HCI_ISO_TYPE)
  {
    HCI_PDUMP_RX_ISO(*(pCoreRecvMsg + 2) + HCI_ACL_HDR_LEN, pCoreRecvMsg);
  }

  /* queue buffer */
  WsfMsgEnq(&hciCb.rxQueue, (wsfHandlerId_t) msgType, pCoreRecvMsg);

  /* set event */
  WsfSetEvent(hciCb.handlerId, HCI_EVT_RX);
}

/*************************************************************************************************/
/*!
 *  \fn     HciCoreHandler
 *        
 *  \brief  WSF event handler for core HCI.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciCoreHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
  uint8_t         *pBuf;
  wsfHandlerId_t  handlerId;
  
  /* Handle message */
  if (pMsg != NULL)
  {
    /* Handle HCI command timeout */
    if (pMsg->event == HCI_MSG_CMD_TIMEOUT)
    {
      hciCmdTimeout(pMsg);
    }
  }
  /* Handle events */
  else if (event & HCI_EVT_RX)
  {
    /* Process rx queue */
    while ((pBuf = WsfMsgDeq(&hciCb.rxQueue, &handlerId)) != NULL)
    {
      /* Handle incoming HCI events */
      if (handlerId == HCI_EVT_TYPE)
      {
        /* Parse/process events */
        hciEvtProcessMsg(pBuf);

        /* Handle events during reset sequence */
        if (hciCb.resetting)
        {
          hciCoreResetSequence(pBuf);
        }
        
        /* Free buffer */
        WsfMsgFree(pBuf);
      }
      /* Handle ACL data */
      else if (handlerId == HCI_ACL_TYPE)
      {
        /* Reassemble */
        if ((pBuf = hciCoreAclReassembly(pBuf)) != NULL)
        {
          /* Call ACL callback; client will free buffer */
          hciCb.aclCback(pBuf);
        }
      }
      /* Handle ISO data */
      else
      {
        if (hciCb.isoCback)
        {
          /* Call ISO callback; client will free buffer */
          hciCb.isoCback(pBuf);
        }
        else
        {
          /* free buffer */
          WsfMsgFree(pBuf);
        }
      }
    }
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciGetBdAddr
 *
 *  \brief  Return a pointer to the BD address of this device.
 *
 *  \return Pointer to the BD address.
 */
/*************************************************************************************************/
uint8_t *HciGetBdAddr(void)
{
  return hciCoreCb.bdAddr;
}

/*************************************************************************************************/
/*!
 *  \fn     HciGetWhiteListSize
 *
 *  \brief  Return the white list size.
 *
 *  \return White list size.
 */
/*************************************************************************************************/
uint8_t HciGetWhiteListSize(void)
{
  return hciCoreCb.whiteListSize;
}

/*************************************************************************************************/
/*!
 *  \fn     HciGetAdvTxPwr
 *
 *  \brief  Return the advertising transmit power.
 *
 *  \return Advertising transmit power.
 */
/*************************************************************************************************/
int8_t HciGetAdvTxPwr(void)
{
  return hciCoreCb.advTxPwr;
}

/*************************************************************************************************/
/*!
 *  \fn     HciGetBufSize
 *
 *  \brief  Return the ACL buffer size supported by the controller.
 *
 *  \return ACL buffer size.
 */
/*************************************************************************************************/
uint16_t HciGetBufSize(void)
{
  return hciCoreCb.bufSize;
}

/*************************************************************************************************/
/*!
 *  \fn     HciGetNumBufs
 *
 *  \brief  Return the number of ACL buffers supported by the controller.
 *
 *  \return Number of ACL buffers.
 */
/*************************************************************************************************/
uint8_t HciGetNumBufs(void)
{
  return hciCoreCb.numBufs;
}

/*************************************************************************************************/
/*!
 *  \fn     HciGetSupStates
 *
 *  \brief  Return the states supported by the controller.
 *
 *  \return Pointer to the supported states array.
 */
/*************************************************************************************************/
uint8_t *HciGetSupStates(void)
{
  return hciCoreCb.leStates;
}

/*************************************************************************************************/
/*!
 *  \fn     HciGetLeSupFeat
 *
 *  \brief  Return the LE supported features supported by the controller.
 *
 *  \return Supported features.
 */
/*************************************************************************************************/
uint64_t HciGetLeSupFeat(void)
{
  // disable LL connection parameter update feature for a better
  // interoperability with Android phones (especially older Android OS).
  return hciCoreCb.leSupFeat & ~HCI_LE_SUP_FEAT_CONN_PARAM_REQ_PROC;
}

/*************************************************************************************************/
/*!
 *  \fn     HciGetMaxRxAclLen
 *
 *  \brief  Get the maximum reassembled RX ACL packet length.
 *
 *  \return ACL packet length.
 */
/*************************************************************************************************/
uint16_t HciGetMaxRxAclLen(void)
{
  return hciCoreCb.maxRxAclLen;
}

/*************************************************************************************************/
/*!
 *  \fn     HciGetResolvingListSize
 *
 *  \brief  Return the resolving list size.
 *
 *  \return resolving list size.
 */
/*************************************************************************************************/
uint8_t HciGetResolvingListSize(void)
{
  return hciCoreCb.resListSize;
}

/*************************************************************************************************/
/*!
 *  \fn     HciLlPrivacySupported
 *
 *  \brief  Whether LL Privacy is supported.
 *
 *  \return TRUE if LL Privacy is supported. FALSE, otherwise.
 */
/*************************************************************************************************/
bool_t HciLlPrivacySupported(void)
{
  return (hciCoreCb.resListSize > 0) ? TRUE : FALSE;
}

/*************************************************************************************************/
/*!
 *  \fn     HciGetMaxAdvDataLen
 *
 *  \brief  Get the maximum advertisement (or scan response) data length supported by the Controller.
 *
 *  \return Maximum advertisement data length.
 */
/*************************************************************************************************/
uint16_t HciGetMaxAdvDataLen(void)
{
  return hciCoreCb.maxAdvDataLen;
}

/*************************************************************************************************/
/*!
 *  \fn     HciGetNumSupAdvSets
 *
 *  \brief  Get the maximum number of advertising sets supported by the Controller.
 *
 *  \return Maximum number of advertising sets.
 */
/*************************************************************************************************/
uint8_t HciGetNumSupAdvSets(void)
{
  return hciCoreCb.numSupAdvSets;
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeAdvExtSupported
 *
 *  \brief  Whether LE Advertising Extensions is supported.
 *
 *  \return TRUE if LE Advertising Extensions is supported. FALSE, otherwise.
 */
/*************************************************************************************************/
bool_t HciLeAdvExtSupported(void)
{
  return (hciCoreCb.numSupAdvSets > 0) ? TRUE : FALSE;
}

/*************************************************************************************************/
/*!
 *  \fn     HciGetPerAdvListSize
 *
 *  \brief  Return the periodic advertising list size.
 *
 *  \return periodic advertising list size.
 */
/*************************************************************************************************/
uint8_t HciGetPerAdvListSize(void)
{
  return hciCoreCb.perAdvListSize;
}

/*************************************************************************************************/
/*!
 *  \brief  Return a pointer to the local version information.
 *
 *  \return Pointer to the local version information.
 */
/*************************************************************************************************/
hciLocalVerInfo_t *HciGetLocalVerInfo(void)
{
  return &hciCoreCb.locVerInfo;
}

/*************************************************************************************************/
/*!
 *  \brief  Return the LE supported features supported by the controller.
 *
 *  \return Supported features.
 */
/*************************************************************************************************/
uint32_t HciGetLeSupFeat32(void)
{
  return (uint32_t) hciCoreCb.leSupFeat;
}

