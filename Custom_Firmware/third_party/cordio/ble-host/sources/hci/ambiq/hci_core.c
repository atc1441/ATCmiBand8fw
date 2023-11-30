/*************************************************************************************************/
/*!
 *  \file   hci_core.c
 *
 *  \brief  HCI core module, platform independent functions.
 *
 *          $Date: 2017-03-10 14:08:37 -0600 (Fri, 10 Mar 2017) $
 *          $Revision: 11501 $
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
#include "wsf_assert.h"
#include "bda.h"
#include "bstream.h"
#include "hci_core.h"
#include "hci_tr.h"
#include "hci_cmd.h"
#include "hci_api.h"
#include "hci_main.h"
#include "l2c_defs.h"
#include "am_mcu_apollo.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/* Default ACL buffer flow control watermark levels */
#ifndef HCI_ACL_QUEUE_HI
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
#define HCI_ACL_QUEUE_HI          5             /* Disable flow when this many buffers queued */
#else
#define HCI_ACL_QUEUE_HI          14
#endif
#endif
#ifndef HCI_ACL_QUEUE_LO
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
#define HCI_ACL_QUEUE_LO          3             /* Enable flow when this many buffers queued */
#else
#define HCI_ACL_QUEUE_LO          13
#endif
#endif

/* Default maximum ACL packet size for reassembly */
#ifndef HCI_MAX_RX_ACL_LEN
#define HCI_MAX_RX_ACL_LEN        HCI_ACL_DEFAULT_LEN
#endif

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/* Event mask */
const uint8_t hciEventMask[HCI_EVT_MASK_LEN] =
{
  HCI_EVT_MASK_DISCONNECT_CMPL |                  /* Byte 0 */
  HCI_EVT_MASK_ENC_CHANGE,                        /* Byte 0 */
  HCI_EVT_MASK_READ_REMOTE_VER_INFO_CMPL |        /* Byte 1 */
  HCI_EVT_MASK_HW_ERROR,                          /* Byte 1 */
  0,                                              /* Byte 2 */
  HCI_EVT_MASK_DATA_BUF_OVERFLOW,                 /* Byte 3 */
  0,                                              /* Byte 4 */
  HCI_EVT_MASK_ENC_KEY_REFRESH_CMPL,              /* Byte 5 */
  0,                                              /* Byte 6 */
  HCI_EVT_MASK_LE_META                            /* Byte 7 */
};

/* LE event mask */
const uint8_t hciLeEventMask[HCI_LE_EVT_MASK_LEN] =
{
  HCI_EVT_MASK_LE_CONN_CMPL_EVT |                 /* Byte 0 */
  HCI_EVT_MASK_LE_ADV_REPORT_EVT |                /* Byte 0 */
  HCI_EVT_MASK_LE_CONN_UPDATE_CMPL_EVT |          /* Byte 0 */
  HCI_EVT_MASK_LE_READ_REMOTE_FEAT_CMPL_EVT |     /* Byte 0 */
  HCI_EVT_MASK_LE_LTK_REQ_EVT |                   /* Byte 0 */
  HCI_EVT_MASK_LE_REMOTE_CONN_PARAM_REQ_EVT |     /* Byte 0 */
  HCI_EVT_MASK_LE_DATA_LEN_CHANGE_EVT |           /* Byte 0 */
  HCI_EVT_MASK_LE_READ_LOCAL_P256_PUB_KEY_CMPL,   /* Byte 0 */
  HCI_EVT_MASK_LE_GENERATE_DHKEY_CMPL |           /* Byte 1 */
  HCI_EVT_MASK_LE_ENHANCED_CONN_CMPL_EVT |        /* Byte 1 */
  HCI_EVT_MASK_LE_DIRECT_ADV_REPORT_EVT |         /* Byte 1 */
  HCI_EVT_MASK_LE_PHY_UPDATE_CMPL_EVT |           /* Byte 1 */
  HCI_EVT_MASK_LE_EXT_ADV_REPORT_EVT |            /* Byte 1 */
  HCI_EVT_MASK_LE_PER_ADV_SYNC_EST_EVT |          /* Byte 1 */
  HCI_EVT_MASK_LE_PER_ADV_REPORT_EVT |            /* Byte 1 */
  HCI_EVT_MASK_LE_PER_ADV_SYNC_LOST_EVT,          /* Byte 1 */
  HCI_EVT_MASK_LE_SCAN_TIMEOUT_EVT |              /* Byte 2 */
  HCI_EVT_MASK_LE_ADV_SET_TERM_EVT |              /* Byte 2 */
  HCI_EVT_MASK_LE_SCAN_REQ_RCVD_EVT |             /* Byte 2 */
  HCI_EVT_MASK_LE_CH_SEL_ALGO_EVT|                /* Byte 2 */
  HCI_EVT_MASK_LE_CONNLESS_IQ_REPORT_EVT|         /* Byte 2 */
  HCI_EVT_MASK_LE_CONN_IQ_REPORT_EVT|             /* Byte 2 */
  HCI_EVT_MASK_LE_CTE_REQ_FAILED_EVT,             /* Byte 2 */
  0,                                              /* Byte 3 */
  0,                                              /* Byte 4 */
  0,                                              /* Byte 5 */
  0,                                              /* Byte 6 */
  0                                               /* Byte 7 */
};

/* event mask page 2 */
const uint8_t hciEventMaskPage2[HCI_EVT_MASK_PAGE_2_LEN] =
{
  0,                                              /* Byte 0 */
  0,                                              /* Byte 1 */
  HCI_EVT_MASK_AUTH_PAYLOAD_TIMEOUT,              /* Byte 2 */
  0,                                              /* Byte 3 */
  0,                                              /* Byte 4 */
  0,                                              /* Byte 5 */
  0,                                              /* Byte 6 */
  0                                               /* Byte 7 */
};

/* LE supported features configuration mask */
uint64_t hciLeSupFeatCfg =
  HCI_LE_SUP_FEAT_ENCRYPTION                 |    /* LE Encryption */
  HCI_LE_SUP_FEAT_CONN_PARAM_REQ_PROC        |    /* Connection Parameters Request Procedure */
  HCI_LE_SUP_FEAT_EXT_REJECT_IND             |    /* Extended Reject Indication */
  HCI_LE_SUP_FEAT_SLV_INIT_FEAT_EXCH         |    /* Slave-initiated Features Exchange */
  HCI_LE_SUP_FEAT_LE_PING                    |    /* LE Ping */
  HCI_LE_SUP_FEAT_DATA_LEN_EXT               |    /* LE Data Packet Length Extension */
  HCI_LE_SUP_FEAT_PRIVACY                    |    /* LL Privacy */
  HCI_LE_SUP_FEAT_EXT_SCAN_FILT_POLICY       |    /* Extended Scanner Filter Policies */
  HCI_LE_SUP_FEAT_LE_2M_PHY                  |    /* LE 2M PHY supported */
  HCI_LE_SUP_FEAT_STABLE_MOD_IDX_TRANSMITTER |    /* Stable Modulation Index - Transmitter supported */
  HCI_LE_SUP_FEAT_STABLE_MOD_IDX_RECEIVER    |    /* Stable Modulation Index - Receiver supported */
  HCI_LE_SUP_FEAT_LE_EXT_ADV                 |    /* LE Extended Advertising */
  HCI_LE_SUP_FEAT_LE_PER_ADV;                     /* LE Periodic Advertising */

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/* Control block */
hciCoreCb_t hciCoreCb;

/*************************************************************************************************/
/*!
 *  \fn     hciCoreConnAlloc
 *
 *  \brief  Allocate a connection structure.
 *
 *  \param  handle  Connection handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hciCoreConnAlloc(uint16_t handle)
{
  uint8_t         i;
  hciCoreConn_t   *pConn = hciCoreCb.conn;

  /* find available connection struct */
  for (i = DM_CONN_MAX; i > 0; i--, pConn++)
  {
    if (pConn->handle == HCI_HANDLE_NONE)
    {
      /* allocate and initialize */
      pConn->handle = handle;
      pConn->flowDisabled = FALSE;
      pConn->outBufs = 0;
      pConn->queuedBufs = 0;

      return;
    }
  }

  HCI_TRACE_WARN0("HCI conn struct alloc failure");
}

/*************************************************************************************************/
/*!
 *  \fn     hciCoreConnFree
 *
 *  \brief  Free a connection structure.
 *
 *  \param  handle  Connection handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hciCoreConnFree(uint16_t handle)
{
  uint8_t         i;
  hciCoreConn_t   *pConn = hciCoreCb.conn;

  /* find connection struct */
  for (i = DM_CONN_MAX; i > 0; i--, pConn++)
  {
    if (pConn->handle == handle)
    {
      /* free any fragmenting ACL packet */
      if (pConn->pTxAclPkt != NULL)
      {
        WsfMsgFree(pConn->pTxAclPkt);
        pConn->pTxAclPkt = NULL;
      }
      pConn->fragmenting = FALSE;

      if (pConn->pRxAclPkt != NULL)
      {
        WsfMsgFree(pConn->pRxAclPkt);
        pConn->pRxAclPkt = NULL;
      }

      /* free structure */
      pConn->handle = HCI_HANDLE_NONE;

      /* optional: iterate through tx ACL queue and free any buffers with this handle */

      /* outstanding buffers are now available; service TX data path */
      hciCoreTxReady(pConn->outBufs);

      return;
    }
  }

  HCI_TRACE_WARN1("hciCoreConnFree handle not found:%u", handle);
}

/*************************************************************************************************/
/*!
 *  \fn     hciCoreConnByHandle
 *
 *  \brief  Get a connection structure by handle
 *
 *  \param  handle  Connection handle.
 *
 *  \return Pointer to connection structure or NULL if not found.
 */
/*************************************************************************************************/
hciCoreConn_t *hciCoreConnByHandle(uint16_t handle)
{
  uint8_t         i;
  hciCoreConn_t   *pConn = hciCoreCb.conn;

  /* find available connection struct */
  for (i = DM_CONN_MAX; i > 0; i--, pConn++)
  {
    if (pConn->handle == handle)
    {
      return pConn;
    }
  }

  return NULL;
}

/*************************************************************************************************/
/*!
 *  \fn     hciCoreNextConnFragment
 *
 *  \brief  Get the next connection structure with a packet fragment to send.
 *
 *  \return Pointer to connection structure or NULL if not found.
 */
/*************************************************************************************************/
static hciCoreConn_t *hciCoreNextConnFragment(void)
{
  uint8_t         i;
  hciCoreConn_t   *pConn = hciCoreCb.conn;

  /* find connection struct */
  for (i = DM_CONN_MAX; i > 0; i--, pConn++)
  {
    if (pConn->handle != HCI_HANDLE_NONE && pConn->fragmenting)
    {
      return pConn;
    }
  }

  return NULL;
}

/*************************************************************************************************/
/*!
 *  \fn     hciCoreConnOpen
 *
 *  \brief  Perform internal processing on HCI connection open.
 *
 *  \param  handle  Connection handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
void hciCoreConnOpen(uint16_t handle)
{
  /* allocate connection structure */
  hciCoreConnAlloc(handle);
}

/*************************************************************************************************/
/*!
 *  \fn     hciCoreConnClose
 *
 *  \brief  Perform internal processing on HCI connection close.
 *
 *  \param  handle  Connection handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
void hciCoreConnClose(uint16_t handle)
{
  /* free connection structure */
  hciCoreConnFree(handle);
}


/*************************************************************************************************/
/*!
 *  \fn     hciCoreSendAclData
 *
 *  \brief  Send ACL data to transport.
 *
 *  \param  pConn    Pointer to connection structure.
 *  \param  pData    WSF buffer containing an ACL packet.
 *
 *  \return TRUE if packet sent, FALSE otherwise.
 */
/*************************************************************************************************/
bool_t hciCoreSendAclData(hciCoreConn_t *pConn, uint8_t *pData)
{
  /* send to transport */
  if ( hciTrSendAclData(pConn, pData) > 0)
  {
    /* increment outstanding buf count for handle */
    pConn->outBufs++;

    /* decrement available buffer count */
    if (hciCoreCb.availBufs > 0)
    {
      hciCoreCb.availBufs--;
    }
    else
    {
      HCI_TRACE_WARN0("hciCoreSendAclData availBufs=0");
    }
    return TRUE;
  }
  return FALSE;
}

/*************************************************************************************************/
/*!
 *  \fn     hciCoreTxReady
 *
 *  \brief  Service the TX data path.
 *
 *  \param  bufs    Number of new buffers now available.
 *
 *  \return TRUE if any pending hci ACL packet sent successfully.
 */
/*************************************************************************************************/
bool_t hciCoreTxReady(uint8_t bufs)
{
  uint8_t         *pData;
  wsfHandlerId_t  handlerId;
  uint16_t        handle;
  uint16_t        len;
  hciCoreConn_t   *pConn;
  bool_t          pkt_sent = FALSE;

  /* increment available buffers, with ceiling */
  if (bufs > 0)
  {
    hciCoreCb.availBufs += bufs;
    if (hciCoreCb.availBufs > hciCoreCb.numBufs)
    {
      hciCoreCb.availBufs = hciCoreCb.numBufs;
    }
  }

  /* service ACL data queue and send as many buffers as we can */
  while (hciCoreCb.availBufs > 0)
  {
    /* send continuation of any fragments first */
    if (hciCoreTxAclContinue(NULL) == FALSE)
    {
      /* if no fragments then check for any queued ACL data */
      if ((pData = WsfMsgPeek(&hciCoreCb.aclQueue, &handlerId)) != NULL)
      {
        /* parse handle and length */
        BYTES_TO_UINT16(handle, pData);
        BYTES_TO_UINT16(len, &pData[2]);

        /* look up conn structure and send data */
        if ((pConn = hciCoreConnByHandle(handle)) != NULL)
        {
          if (hciCoreTxAclStart(pConn, len, pData) == TRUE)
          {
            WsfMsgDeq(&hciCoreCb.aclQueue, &handlerId);
            hciCoreTxAclComplete(pConn, pData);
            pkt_sent = TRUE;
          }
          else
          {
            // Wait for Cooper to be ready.
            break;
          }
        }
        /* handle not found, connection must be closed */
        else
        {
          /* Dequeue */
          pData = WsfMsgDeq(&hciCoreCb.aclQueue, &handlerId);
          /* discard buffer */
          WsfMsgFree(pData);

          HCI_TRACE_WARN1("hciCoreTxReady discarding buffer, handle=%u", handle);
        }
      }
      else
      {
        /* no fragments or queued data to send; we're done */
        break;
      }
    }
  }
  return pkt_sent;
}

/*************************************************************************************************/
/*!
 *  \fn     hciCoreTxAclStart
 *
 *  \brief  Send ACL packets, start of packet.
 *
 *  \param  pConn    Pointer to connection structure.
 *  \param  len      ACL packet length.
 *  \param  pData    WSF buffer containing an ACL packet.
 *
 *  \return TRUE if packet sent, FALSE otherwise.
 */
/*************************************************************************************************/
bool_t hciCoreTxAclStart(hciCoreConn_t *pConn, uint16_t len, uint8_t *pData)
{
  uint16_t hciLen;

  /* make sure not already fragmenting on this connection */
  WSF_ASSERT(pConn->fragmenting == FALSE);

  hciLen = HciGetBufSize();

  HCI_TRACE_INFO1("hciCoreTxAclStart len=%u", len);

  /* if acl len > controller acl buf len */
  if (len > hciLen)
  {
    /* store remaining acl len = acl len - hci acl buf len */
    pConn->txAclRemLen = len - hciLen;

    /* store position for next fragment */
    pConn->pNextTxFrag = pData + hciLen;

    /* store information required for fragmentation */
    pConn->pTxAclPkt = pData;
    pConn->fragmenting = TRUE;

    /* set acl len in packet to hci acl buf len */
    UINT16_TO_BUF(&pData[2], hciLen);

    /* send the packet */
    if (hciCoreSendAclData(pConn, pData) == TRUE)
    {
      /* send additional fragments while there are HCI buffers available */
      // while ((hciCoreCb.availBufs > 0) && hciCoreTxAclContinue(pConn));

      // Return True since we save the buffer pointer.
      return TRUE;
    }
    else
    {
      /* clear previously stored information required for fragmentation */
      pConn->pTxAclPkt = NULL;
      pConn->fragmenting = FALSE;

      /* Restore original acl len in packet */
      UINT16_TO_BUF(&pData[2], len);

      // Start over for the same packet
      return FALSE;
    }
  }
  else
  {
    /* no fragmentation, just send the packet */
    return hciCoreSendAclData(pConn, pData);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     hciCoreTxAclContinue
 *
 *  \brief  Send ACL packets, continuation of fragmented packets.
 *
 *  \param  pConn    Pointer to connection structure.  If set non-NULL, then a fragment is
 *                   sent from this connection structure.  If NULL the function finds the next
 *                   connection structure with a fragment to be sent.
 *
 *  \return TRUE if packet sent, FALSE otherwise.
 */
/*************************************************************************************************/
bool_t hciCoreTxAclContinue(hciCoreConn_t *pConn)
{
  uint16_t aclLen;

  if (pConn == NULL)
  {
    pConn = hciCoreNextConnFragment();
  }

  if (pConn != NULL)
  {
    /* get next fragment length */
    aclLen = (pConn->txAclRemLen < HciGetBufSize()) ? pConn->txAclRemLen : HciGetBufSize();

    if (aclLen > 0)
    {
      /* set handle in packet with continuation bit set */
      UINT16_TO_BUF(pConn->pNextTxFrag, (pConn->handle | HCI_PB_CONTINUE));

      /* set acl len in packet */
      UINT16_TO_BUF(&(pConn->pNextTxFrag[2]), aclLen);

      /* send the packet */
      if ( hciCoreSendAclData(pConn, pConn->pNextTxFrag) == TRUE)
      {
        /* decrement remaining length */
        pConn->txAclRemLen -= aclLen;

        HCI_TRACE_INFO2("hciCoreTxAclContinue aclLen=%u remLen=%u", aclLen, pConn->txAclRemLen);

        /* set up pointer to next fragment */
        if (pConn->txAclRemLen > 0)
        {
          pConn->pNextTxFrag += aclLen;
        }
        hciCoreTxAclComplete(pConn, pConn->pNextTxFrag);
      }
      return TRUE;
    }
  }

  return FALSE;
}

/*************************************************************************************************/
/*!
 *  \fn     hciCoreTxAclComplete
 *
 *  \brief  This function is called from the HCI transport layer when transmission of an ACL
 *          packet is complete.
 *
 *  \param  pConn    Pointer to connection structure.
 *  \param  pData    WSF buffer containing an ACL packet.
 *
 *  \return None.
 */
/*************************************************************************************************/
void hciCoreTxAclComplete(hciCoreConn_t *pConn, uint8_t *pData)
{
  /* if fragmenting */
  if (pConn->fragmenting)
  {
    /* check if all fragments sent */
    if (pConn->txAclRemLen == 0)
    {
      /* free original buffer */
      WsfMsgFree(pConn->pTxAclPkt);
      pConn->pTxAclPkt = NULL;
      pConn->fragmenting = FALSE;
      HCI_TRACE_INFO0("hciCoreTxAclComplete free pTxAclPkt");
    }
  }
  else if (pData != NULL)
  {
    WsfMsgFree(pData);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     hciCoreAclReassembly
 *
 *  \brief  Reassemble an ACL packet.
 *
 *  \param  pData   Input ACL packet.
 *
 *  \return pointer to ACL packet to send, or NULL if no packet to send.
 */
/*************************************************************************************************/
uint8_t *hciCoreAclReassembly(uint8_t *pData)
{
  hciCoreConn_t *pConn;
  uint8_t       *pDataRtn = NULL;
  uint16_t      handle;
  uint16_t      aclLen;
  uint16_t      l2cLen;
  uint16_t      pbf;
  bool_t        freeData = TRUE;

  BYTES_TO_UINT16(handle, pData);
  pbf = handle & HCI_PB_FLAG_MASK;
  handle &= HCI_HANDLE_MASK;
  BYTES_TO_UINT16(aclLen, &pData[2]);

  /* look up connection */
  if ((pConn = hciCoreConnByHandle(handle)) != NULL)
  {
    /* if this is a start packet */
    if (pbf == HCI_PB_START_C2H)
    {
      /* if currently reassembled packet not complete */
      if (pConn->pRxAclPkt != NULL)
      {
        /* discard currently reassembled packet */
        WsfMsgFree(pConn->pRxAclPkt);
        pConn->pRxAclPkt = NULL;
        HCI_TRACE_WARN1("disarded hci rx pkt handle=0x%04x", handle);
      }

      /* read l2cap length */
      if (aclLen >= L2C_HDR_LEN_SEG_LEN)
      {
        BYTES_TO_UINT16(l2cLen, &pData[4]);

        /* check length vs. configured maximum */
        if ((l2cLen + L2C_HDR_LEN) > hciCoreCb.maxRxAclLen)
        {
          HCI_TRACE_WARN1("l2c len=0x%04x to large for reassembly", l2cLen);
        }
        /* if reassembly required */
        else if ((l2cLen + L2C_HDR_LEN) > aclLen)
        {
          /* allocate buffer to store complete l2cap packet */
          if ((pConn->pRxAclPkt = WsfMsgDataAlloc(l2cLen + L2C_HDR_LEN + HCI_ACL_HDR_LEN, 0)) != NULL)
          {
            /* store buffer for reassembly */
            pConn->pNextRxFrag = pConn->pRxAclPkt;

            /* build acl header and copy data */
            UINT16_TO_BSTREAM(pConn->pNextRxFrag, handle);
            UINT16_TO_BSTREAM(pConn->pNextRxFrag, l2cLen + L2C_HDR_LEN);
            /* if the start packet includes data beside the l2cap length segment */
            if (aclLen > L2C_HDR_LEN_SEG_LEN)
            {
              memcpy(pConn->pNextRxFrag, &pData[4], aclLen);
            }
            pConn->pNextRxFrag += aclLen;

            /* store remaining length */
            pConn->rxAclRemLen = l2cLen + L2C_HDR_LEN - aclLen;
          }
          else
          {
            /* alloc failed; discard */
            HCI_TRACE_WARN1("reassembly alloc failed len=%u", (l2cLen + L2C_HDR_LEN + HCI_ACL_HDR_LEN));
          }
        }
        else
        {
          /* no reassembly required, pData is ready to go */
          pDataRtn = pData;
          freeData = FALSE;
        }
      }
      else if (aclLen > 0)
      {
        /* Incompelete l2cap length segment is included in this packet, allocate the
           maximum acl size buffer to store complete l2cap packet */
        if ((pConn->pRxAclPkt = WsfMsgDataAlloc(hciCoreCb.maxRxAclLen + HCI_ACL_HDR_LEN, 0)) != NULL)
        {
            /* store buffer for reassembly */
            pConn->pNextRxFrag = pConn->pRxAclPkt;

            /* build acl header and copy data */
            UINT16_TO_BSTREAM(pConn->pNextRxFrag, handle);
            /* have not gotten l2cap length yet, write zero to the length segment of complete packet */
            UINT16_TO_BSTREAM(pConn->pNextRxFrag, 0);
            memcpy(pConn->pNextRxFrag, &pData[4], aclLen);
            pConn->pNextRxFrag += aclLen;

            /* store remaining length */
            pConn->rxAclRemLen = hciCoreCb.maxRxAclLen - aclLen;
        }
        else
        {
          /* alloc failed; discard */
          HCI_TRACE_WARN1("reassembly alloc failed len=%u", hciCoreCb.maxRxAclLen + HCI_ACL_HDR_LEN);
        }
      }
      else
      {
        /* invalid l2cap packet; discard */
        HCI_TRACE_WARN1("invalid l2c pkt aclLen=%u", aclLen);
      }
    }
    /* else if this is a continuation packet */
    else if (pbf == HCI_PB_CONTINUE)
    {
      /* if expecting a continuation */
      if (pConn->pRxAclPkt != NULL)
      {
        if (aclLen <= pConn->rxAclRemLen)
        {
          /* copy data to start of next fragment */
          memcpy(pConn->pNextRxFrag, &pData[HCI_ACL_HDR_LEN], aclLen);

          /* if the l2cap length segment has not received in the start packet completely */
          if ((pConn->pNextRxFrag - pConn->pRxAclPkt) < (HCI_ACL_HDR_LEN + L2C_HDR_LEN_SEG_LEN))
          {
            /* read l2cap length */
            BYTES_TO_UINT16(l2cLen, (pConn->pRxAclPkt + HCI_ACL_HDR_LEN));

            /* check length vs. configured maximum */
            if ((l2cLen + L2C_HDR_LEN) > hciCoreCb.maxRxAclLen)
            {
              HCI_TRACE_WARN1("l2c len=0x%04x to large for reassembly", l2cLen);
              /* discard currently reassembled packet */
              WsfMsgFree(pConn->pRxAclPkt);
              pConn->pRxAclPkt = NULL;
            }
            else
            {
              /* update the length segment of reassembled packet */
              UINT16_TO_BUF((pConn->pRxAclPkt + 2), (l2cLen + L2C_HDR_LEN));
              /* update remaining length of buffer which was allocated by the maximum acl size */
              pConn->rxAclRemLen -= (hciCoreCb.maxRxAclLen - (l2cLen + L2C_HDR_LEN));
            }
          }

          pConn->pNextRxFrag += aclLen;

          /* update remaining length */
          pConn->rxAclRemLen -= aclLen;

          /* if reassembly complete return reassembled packet */
          if (pConn->rxAclRemLen == 0)
          {
            pDataRtn = pConn->pRxAclPkt;
            pConn->pRxAclPkt = NULL;
          }
        }
        else
        {
          HCI_TRACE_WARN2("continuation pkt too long len=%u RemLen=%u", aclLen, pConn->rxAclRemLen);
          /* discard currently reassembled packet */
          WsfMsgFree(pConn->pRxAclPkt);
          pConn->pRxAclPkt = NULL;
        }
      }
      else
      {
        HCI_TRACE_WARN1("unexpected continuation pkt handle=0x%04x", handle);
      }
    }
    /* else unknown packet type */
    else
    {
      HCI_TRACE_WARN1("unknown pb flags=0x%04x", pbf);
    }
  }
  else
  {
    /* connection not found */
    HCI_TRACE_WARN1("pkt rcvd on unknown handle=0x%04x", (handle & HCI_HANDLE_MASK));
  }

  if (freeData)
  {
    WsfMsgFree(pData);
  }

  return pDataRtn;
}

/*************************************************************************************************/
/*!
 *  \fn     hciCoreTxAclDataFragmented
 *
 *  \brief  Check if a TX ACL packet is being fragmented.
 *
 *  \param  pContext Connection context.
 *
 *  \return TRUE if fragmenting a TX ACL packet, FALSE otherwise.
 */
/*************************************************************************************************/
bool_t hciCoreTxAclDataFragmented(hciCoreConn_t *pConn)
{
  return pConn->fragmenting;
}

/*************************************************************************************************/
/*!
 *  \fn     HciCoreInit
 *
 *  \brief  HCI core initialization.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciCoreInit(void)
{
  uint8_t   i;

  WSF_QUEUE_INIT(&hciCoreCb.aclQueue);

  for (i = 0; i < DM_CONN_MAX; i++)
  {
    hciCoreCb.conn[i].handle = HCI_HANDLE_NONE;
  }


  for (i = 0; i < DM_CIS_MAX; i++)
  {
    hciCoreCb.cis[i].handle = HCI_HANDLE_NONE;
  }

  hciCoreCb.maxRxAclLen = HCI_MAX_RX_ACL_LEN;
  hciCoreCb.aclQueueHi = HCI_ACL_QUEUE_HI;
  hciCoreCb.aclQueueLo = HCI_ACL_QUEUE_LO;

  #if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
    if (APOLLO3_GE_B0)
    {
      // B0 has only less internal ACL buffers
      hciCoreCb.aclQueueHi--;
      hciCoreCb.aclQueueLo--;
    }
  #endif
  hciCoreCb.extResetSeq = NULL;

  hciCoreInit();
}

/*************************************************************************************************/
/*!
 *  \fn     HciResetSequence
 *
 *  \brief  Initiate an HCI reset sequence.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciResetSequence(void)
{
  uint8_t         *pBuf;
  wsfHandlerId_t    handlerId;

  uint8_t         i;
  hciCoreConn_t   *pConn = hciCoreCb.conn;

  // free any pending incoming packets
  while ((pBuf = WsfMsgDeq(&hciCb.rxQueue, &handlerId)) != NULL)
  {
    /* Free buffer */
    WsfMsgFree(pBuf);
  }

  HCI_TRACE_INFO0("reset sequence");
  // free any pending tx packets
  /* find connection struct */
  for (i = DM_CONN_MAX; i > 0; i--, pConn++)
  {
    /* free any fragmenting ACL packet */
    if (pConn->pTxAclPkt != NULL)
    {
      WsfMsgFree(pConn->pTxAclPkt);
      pConn->pTxAclPkt = NULL;
    }
    pConn->fragmenting = FALSE;

    if (pConn->pRxAclPkt != NULL)
    {
      WsfMsgFree(pConn->pRxAclPkt);
      pConn->pRxAclPkt = NULL;
    }

    /* free structure */
    pConn->handle = HCI_HANDLE_NONE;

    /* optional: iterate through tx ACL queue and free any buffers with this handle */

    /* outstanding buffers are now available; service TX data path */
    hciCoreTxReady(pConn->outBufs);

  }

  /* set resetting state */
  hciCb.resetting = TRUE;

  /* start the reset sequence */
  hciCoreResetStart();
}

/*************************************************************************************************/
/*!
 *  \fn     HciSetMaxRxAclLen
 *
 *  \brief  Set the maximum reassembled RX ACL packet length.  Minimum value is 27.
 *
 *  \param  len     ACL packet length.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciSetMaxRxAclLen(uint16_t len)
{
  hciCoreCb.maxRxAclLen = len;
}

/*************************************************************************************************/
/*!
 *  \fn     HciSetAclQueueWatermarks
 *
 *  \brief  Set TX ACL queue high and low watermarks.
 *
 *  \param  queueHi   Disable flow on a connection when this many ACL buffers are queued.
 *          queueLo   Disable flow on a connection when this many ACL buffers are queued.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciSetAclQueueWatermarks(uint8_t queueHi, uint8_t queueLo)
{
  hciCoreCb.aclQueueHi = queueHi;
  hciCoreCb.aclQueueLo = queueLo;
}

/*************************************************************************************************/
/*!
*  \fn      HciSetLeSupFeat
*
*  \brief   Set LE supported features configuration mask.
*
*  \param   feat    Feature bit to set or clear
*  \param   flag    TRUE to set feature bit and FALSE to clear it
*
*  \return None.
*/
/*************************************************************************************************/
void HciSetLeSupFeat(uint64_t feat, bool_t flag)
{
  /* if asked to include feature */
  if (flag)
  {
    /* set feature bit */
    hciLeSupFeatCfg |= feat;
  }
  else
  {
    /* clear feature bit */
    hciLeSupFeatCfg &= ~feat;
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciSendAclData
 *
 *  \brief  Send data from the stack to HCI.
 *
 *  \param  pData    WSF buffer containing an ACL packet
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciSendAclData(uint8_t *pData)
{
  uint16_t        handle;
  uint16_t        len;
  hciCoreConn_t   *pConn;

  /* parse handle and length */
  BYTES_TO_UINT16(handle, pData);
  BYTES_TO_UINT16(len, &pData[2]);

  /* look up connection structure */
  if ((pConn = hciCoreConnByHandle(handle)) != NULL)
  {
    /* queue data - message handler ID 'handerId' not used */
    WsfMsgEnq(&hciCoreCb.aclQueue, 0, pData);

    HCI_TRACE_WARN1("enq acl pkt %x", pData);

    /* if queue not empty and buffers available */
    if ((WsfQueueCount(&hciCoreCb.aclQueue) == 1) && hciCoreCb.availBufs > 0)
    {
      /* send data */
      hciCoreTxReady(0);
    }

    /* increment buffer queue count for this connection with consideration for HCI fragmentation */
    pConn->queuedBufs += ((len - 1) / HciGetBufSize()) + 1;

    /* manage flow control to stack */
    if (pConn->queuedBufs >= hciCoreCb.aclQueueHi && pConn->flowDisabled == FALSE)
    {
      pConn->flowDisabled = TRUE;
      (*hciCb.flowCback)(handle, TRUE);
    }
  }
  /* connection not found, connection must be closed */
  else
  {
    /* discard buffer */
    WsfMsgFree(pData);

    HCI_TRACE_WARN1("HciSendAclData discarding buffer, handle=%u", handle);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Allocate a CIS connection structure.
 *
 *  \param  handle  Connection handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hciCoreCisAlloc(uint16_t handle)
{
  uint8_t         i;
  hciCoreCis_t   *pCis = hciCoreCb.cis;

  /* find available connection struct */
  for (i = DM_CIS_MAX; i > 0; i--, pCis++)
  {
    if (pCis->handle == HCI_HANDLE_NONE)
    {
      /* allocate and initialize */
      pCis->handle = handle;

      return;
    }
  }

  HCI_TRACE_WARN0("HCI cis struct alloc failure");
}

/*************************************************************************************************/
/*!
 *  \brief  Free a CIS connection structure.
 *
 *  \param  handle  Connection handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hciCoreCisFree(uint16_t handle)
{
  uint8_t         i;
  hciCoreCis_t   *pCis = hciCoreCb.cis;

  /* find connection struct */
  for (i = DM_CIS_MAX; i > 0; i--, pCis++)
  {
    if (pCis->handle == handle)
    {
      /* free structure */
      pCis->handle = HCI_HANDLE_NONE;

      return;
    }
  }

  HCI_TRACE_WARN1("hciCoreCisFree handle not found:%u", handle);
}

/*************************************************************************************************/
/*!
 *  \brief  Get a CIS connection structure by handle
 *
 *  \param  handle  Connection handle.
 *
 *  \return Pointer to CIS connection structure or NULL if not found.
 */
/*************************************************************************************************/
hciCoreCis_t *hciCoreCisByHandle(uint16_t handle)
{
  uint8_t         i;
  hciCoreCis_t   *pCis = hciCoreCb.cis;

  /* find available connection struct */
  for (i = DM_CIS_MAX; i > 0; i--, pCis++)
  {
    if (pCis->handle == handle)
    {
      return pCis;
    }
  }

  return NULL;
}

/*************************************************************************************************/
/*!
 *  \brief  Perform internal processing on HCI CIS connection open.
 *
 *  \param  handle  Connection handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
void hciCoreCisOpen(uint16_t handle)
{
  /* allocate CIS connection structure */
  hciCoreCisAlloc(handle);
}

/*************************************************************************************************/
/*!
 *  \brief  Perform internal processing on HCI CIS connection close.
 *
 *  \param  handle  Connection handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
void hciCoreCisClose(uint16_t handle)
{
  /* free CIS connection structure */
  hciCoreCisFree(handle);
}

