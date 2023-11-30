/*************************************************************************************************/
/*!
 *  \file   hci_cmd.c
 *
 *  \brief  HCI command module.
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
#include "wsf_queue.h"
#include "wsf_timer.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "bstream.h"
#include "hci_cmd.h"
#include "hci_tr.h"
#include "hci_api.h"
#include "hci_main.h"
#include <stdbool.h>
#include "hci_drv_apollo.h"
#include "dm_api.h"

#include "am_mcu_apollo.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/* HCI command timeout in seconds */
#define HCI_CMD_TIMEOUT           10

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/* Local control block type */
typedef struct
{
  wsfTimer_t      cmdTimer;       /* HCI command timeout timer */
  wsfQueue_t      cmdQueue;       /* HCI command queue */
  uint16_t        cmdOpcode;      /* Opcode of last HCI command sent */
  uint8_t         numCmdPkts;     /* Number of outstanding HCI commands that can be sent */
} hciCmdCb_t;

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/* Local control block */
hciCmdCb_t hciCmdCb;

/*************************************************************************************************/
/*!
 *  \fn     hciCmdAlloc
 *
 *  \brief  Allocate an HCI command buffer and set the command header fields.
 *
 *  \param  opcode  Command opcode.
 *  \param  len     length of command parameters.
 *
 *  \return Pointer to WSF msg buffer.
 */
/*************************************************************************************************/
uint8_t *hciCmdAlloc(uint16_t opcode, uint16_t len)
{
  uint8_t   *p;

  /* allocate buffer */
  if ((p = WsfMsgAlloc(len + HCI_CMD_HDR_LEN)) != NULL)
  {
    /* set HCI command header */
    UINT16_TO_BSTREAM(p, opcode);
    UINT8_TO_BSTREAM(p, len);
    p -= HCI_CMD_HDR_LEN;
  }

  return p;
}

/*************************************************************************************************/
/*!
 *  \fn     hciCmdSend
 *
 *  \brief  Send an HCI command and service the HCI command queue.
 *
 *  \param  pData  Buffer containing HCI command to send or NULL.
 *
 *  \return TRUE if any new or pending hci command sent successfully.
 */
/*************************************************************************************************/
bool_t hciCmdSend(uint8_t *pData)
{
  uint8_t         *p;
  wsfHandlerId_t  handlerId;

  /* queue command if present */
  if (pData != NULL)
  {
    /* queue data - message handler ID 'handerId' not used */
    WsfMsgEnq(&hciCmdCb.cmdQueue, 0, pData);
  }

  /* service the HCI command queue; first check if controller can accept any commands */
  if (hciCmdCb.numCmdPkts > 0)
  {
    /* if queue not empty */
    if ((p = WsfMsgPeek(&hciCmdCb.cmdQueue, &handlerId)) != NULL)
    {
      // Ambiq: there is chance that BLE core crashes
      // and no response to HCI command anymore,
      // so start the timer whenever writing HCI command to transport.
      /* store opcode of command we're sending */
      BYTES_TO_UINT16(hciCmdCb.cmdOpcode, p);

      /* start command timeout */
      WsfTimerStartSec(&hciCmdCb.cmdTimer, HCI_CMD_TIMEOUT);

      /* send command to transport */
      if (hciTrSendCmd(p) == TRUE)
      {

        /* remove from the queue*/
        WsfMsgDeq(&hciCmdCb.cmdQueue, &handlerId);

        /* decrement controller command packet count */
        hciCmdCb.numCmdPkts--;

        /* Free buffer here after dequeue */
        WsfMsgFree(p);

        return TRUE;
      }
    }
  }
  return FALSE;
}

/*************************************************************************************************/
/*!
 *  \fn     hciCmdInit
 *
 *  \brief  Initialize the HCI cmd module.
 *
 *  \return None.
 */
/*************************************************************************************************/
void hciCmdInit(void)
{
  WSF_QUEUE_INIT(&hciCmdCb.cmdQueue);

  /* initialize numCmdPkts for special case of first command */
  hciCmdCb.numCmdPkts = 1;

  /* initialize timer */
  hciCmdCb.cmdTimer.msg.event = HCI_MSG_CMD_TIMEOUT;
  hciCmdCb.cmdTimer.handlerId = hciCb.handlerId;
}

/*************************************************************************************************/
/*!
 *  \fn     hciCmdTimeout
 *
 *  \brief  Process an HCI command timeout.
 *
 *  \return None.
 */
/*************************************************************************************************/
void hciCmdTimeout(wsfMsgHdr_t *pMsg)
{
  HCI_TRACE_INFO1("hciCmdTimeout, opcode=0x%x", hciCmdCb.cmdOpcode);
  // When it times out, pretty much we have to
  // reset/reboot controller and initialize HCI
  // layer and SPI transport layer again.

  HciDrvRadioShutdown();
  HciDrvRadioBoot(0);
  DmDevReset();
}

/*************************************************************************************************/
/*!
 *  \fn     hciCmdRecvCmpl
 *
 *  \brief  Process an HCI Command Complete or Command Status event.
 *
 *  \param  numCmdPkts  Number of commands that can be sent to the controller.
 *
 *  \return None.
 */
/*************************************************************************************************/
void hciCmdRecvCmpl(uint8_t numCmdPkts)
{
  /* stop the command timeout timer */
  WsfTimerStop(&hciCmdCb.cmdTimer);

  /*
   * Set the number of commands that can be sent to the controller.  Setting this
   * to 1 rather than incrementing by numCmdPkts allows only one command at a time to
   * be sent to the controller and simplifies the code.
   */
  hciCmdCb.numCmdPkts = 1;

  /* send the next queued command */
  hciCmdSend(NULL);
}

/*************************************************************************************************/
/*!
 *  \fn     HciDisconnectCmd
 *
 *  \brief  HCI disconnect command.
 */
/*************************************************************************************************/
void HciDisconnectCmd(uint16_t handle, uint8_t reason)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_DISCONNECT, HCI_LEN_DISCONNECT)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, handle);
    UINT8_TO_BSTREAM(p, reason);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeAddDevWhiteListCmd
 *
 *  \brief  HCI LE add device white list command.
 */
/*************************************************************************************************/
void HciLeAddDevWhiteListCmd(uint8_t addrType, uint8_t *pAddr)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_ADD_DEV_WHITE_LIST, HCI_LEN_LE_ADD_DEV_WHITE_LIST)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT8_TO_BSTREAM(p, addrType);
    BDA_TO_BSTREAM(p, pAddr);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeClearWhiteListCmd
 *
 *  \brief  HCI LE clear white list command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeClearWhiteListCmd(void)
{
  uint8_t *pBuf;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_CLEAR_WHITE_LIST, HCI_LEN_LE_CLEAR_WHITE_LIST)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeConnUpdateCmd
 *
 *  \brief  HCI connection update command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeConnUpdateCmd(uint16_t handle, hciConnSpec_t *pConnSpec)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_CONN_UPDATE, HCI_LEN_LE_CONN_UPDATE)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, handle);
    UINT16_TO_BSTREAM(p, pConnSpec->connIntervalMin);
    UINT16_TO_BSTREAM(p, pConnSpec->connIntervalMax);
    UINT16_TO_BSTREAM(p, pConnSpec->connLatency);
    UINT16_TO_BSTREAM(p, pConnSpec->supTimeout);
    UINT16_TO_BSTREAM(p, pConnSpec->minCeLen);
    UINT16_TO_BSTREAM(p, pConnSpec->maxCeLen);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeCreateConnCmd
 *
 *  \brief  HCI LE create connection command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeCreateConnCmd(uint16_t scanInterval, uint16_t scanWindow, uint8_t filterPolicy,
                        uint8_t peerAddrType, uint8_t *pPeerAddr, uint8_t ownAddrType,
                        hciConnSpec_t *pConnSpec)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_CREATE_CONN, HCI_LEN_LE_CREATE_CONN)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, scanInterval);
    UINT16_TO_BSTREAM(p, scanWindow);
    UINT8_TO_BSTREAM(p, filterPolicy);
    UINT8_TO_BSTREAM(p, peerAddrType);
    BDA_TO_BSTREAM(p, pPeerAddr);
    UINT8_TO_BSTREAM(p, ownAddrType);
    UINT16_TO_BSTREAM(p, pConnSpec->connIntervalMin);
    UINT16_TO_BSTREAM(p, pConnSpec->connIntervalMax);
    UINT16_TO_BSTREAM(p, pConnSpec->connLatency);
    UINT16_TO_BSTREAM(p, pConnSpec->supTimeout);
    UINT16_TO_BSTREAM(p, pConnSpec->minCeLen);
    UINT16_TO_BSTREAM(p, pConnSpec->maxCeLen);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeCreateConnCancelCmd
 *
 *  \brief  HCI LE create connection cancel command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeCreateConnCancelCmd(void)
{
  uint8_t *pBuf;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_CREATE_CONN_CANCEL, HCI_LEN_LE_CREATE_CONN_CANCEL)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
*  \fn     HciLeRemoteConnParamReqReply
*
*  \brief  HCI LE remote connection parameter request reply command.
*
*  \return None.
*/
/*************************************************************************************************/
void HciLeRemoteConnParamReqReply(uint16_t handle, uint16_t intervalMin, uint16_t intervalMax, uint16_t latency,
                                  uint16_t timeout, uint16_t minCeLen, uint16_t maxCeLen)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_REM_CONN_PARAM_REP, HCI_LEN_LE_REM_CONN_PARAM_REP)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, handle);
    UINT16_TO_BSTREAM(p, intervalMin);
    UINT16_TO_BSTREAM(p, intervalMax);
    UINT16_TO_BSTREAM(p, latency);
    UINT16_TO_BSTREAM(p, timeout);
    UINT16_TO_BSTREAM(p, minCeLen);
    UINT16_TO_BSTREAM(p, maxCeLen);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
*  \fn     HciLeRemoteConnParamReqNegReply
*
*  \brief  HCI LE remote connection parameter request negative reply command.
*
*  \return None.
*/
/*************************************************************************************************/
void HciLeRemoteConnParamReqNegReply(uint16_t handle, uint8_t reason)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_REM_CONN_PARAM_NEG_REP, HCI_LEN_LE_REM_CONN_PARAM_NEG_REP)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, handle);
    UINT8_TO_BSTREAM(p, reason);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
*  \fn     HciLeSetDataLen
*
*  \brief  HCI LE set data len command.
*
*  \return None.
*/
/*************************************************************************************************/
void HciLeSetDataLen(uint16_t handle, uint16_t txOctets, uint16_t txTime)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_SET_DATA_LEN, HCI_LEN_LE_SET_DATA_LEN)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, handle);
    UINT16_TO_BSTREAM(p, txOctets);
    UINT16_TO_BSTREAM(p, txTime);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
*  \fn     HciLeReadDefDataLen
*
*  \brief  HCI LE read suggested default data len command.
*
*  \return None.
*/
/*************************************************************************************************/
void HciLeReadDefDataLen(void)
{
  uint8_t *pBuf;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_READ_DEF_DATA_LEN, HCI_LEN_LE_READ_DEF_DATA_LEN)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
*  \fn     HciLeWriteDefDataLen
*
*  \brief  HCI LE write suggested default data len command.
*
*  \return None.
*/
/*************************************************************************************************/
void HciLeWriteDefDataLen(uint16_t suggestedMaxTxOctets, uint16_t suggestedMaxTxTime)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_WRITE_DEF_DATA_LEN, HCI_LEN_LE_WRITE_DEF_DATA_LEN)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, suggestedMaxTxOctets);
    UINT16_TO_BSTREAM(p, suggestedMaxTxTime);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
*  \fn     HciLeReadLocalP256PubKey
*
*  \brief  HCI LE read local P-256 public key command.
*
*  \return None.
*/
/*************************************************************************************************/
void HciLeReadLocalP256PubKey(void)
{
  uint8_t *pBuf;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_READ_LOCAL_P256_PUB_KEY, HCI_LEN_LE_READ_LOCAL_P256_PUB_KEY)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
*  \fn     HciLeGenerateDHKey
*
*  \brief  HCI LE generate DHKey command.
*
*  \return None.
*/
/*************************************************************************************************/
void HciLeGenerateDHKey(uint8_t *pPubKeyX, uint8_t *pPubKeyY)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_GENERATE_DHKEY, HCI_LEN_LE_GENERATE_DHKEY)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    memcpy(p, pPubKeyX, HCI_DH_KEY_LEN);
    memcpy(p + HCI_DH_KEY_LEN, pPubKeyY, HCI_DH_KEY_LEN);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
*  \fn     HciLeReadMaxDataLen
*
*  \brief  HCI LE read maximum data len command.
*
*  \return None.
*/
/*************************************************************************************************/
void HciLeReadMaxDataLen(void)
{
  uint8_t *pBuf;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_READ_MAX_DATA_LEN, HCI_LEN_LE_READ_MAX_DATA_LEN)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeEncryptCmd
 *
 *  \brief  HCI LE encrypt command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeEncryptCmd(uint8_t *pKey, uint8_t *pData)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_ENCRYPT, HCI_LEN_LE_ENCRYPT)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    memcpy(p, pKey, HCI_KEY_LEN);
    p += HCI_KEY_LEN;
    memcpy(p, pData, HCI_ENCRYPT_DATA_LEN);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeLtkReqNegReplCmd
 *
 *  \brief  HCI LE long term key request negative reply command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeLtkReqNegReplCmd(uint16_t handle)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_LTK_REQ_NEG_REPL, HCI_LEN_LE_LTK_REQ_NEG_REPL)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, handle);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeLtkReqReplCmd
 *
 *  \brief  HCI LE long term key request reply command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeLtkReqReplCmd(uint16_t handle, uint8_t *pKey)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_LTK_REQ_REPL, HCI_LEN_LE_LTK_REQ_REPL)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, handle);
    memcpy(p, pKey, HCI_KEY_LEN);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeRandCmd
 *
 *  \brief  HCI LE random command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeRandCmd(void)
{
  uint8_t *pBuf;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_RAND, HCI_LEN_LE_RAND)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeReadAdvTXPowerCmd
 *
 *  \brief  HCI LE read advertising TX power command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeReadAdvTXPowerCmd(void)
{
  uint8_t *pBuf;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_READ_ADV_TX_POWER, HCI_LEN_LE_READ_ADV_TX_POWER)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeReadBufSizeCmd
 *
 *  \brief  HCI LE read buffer size command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeReadBufSizeCmd(void)
{
  uint8_t *pBuf;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_READ_BUF_SIZE, HCI_LEN_LE_READ_BUF_SIZE)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeReadChanMapCmd
 *
 *  \brief  HCI LE read channel map command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeReadChanMapCmd(uint16_t handle)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_READ_CHAN_MAP, HCI_LEN_LE_READ_CHAN_MAP)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, handle);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeReadLocalSupFeatCmd
 *
 *  \brief  HCI LE read local supported feautre command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeReadLocalSupFeatCmd(void)
{
  uint8_t *pBuf;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_READ_LOCAL_SUP_FEAT, HCI_LEN_LE_READ_LOCAL_SUP_FEAT)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeReadRemoteFeatCmd
 *
 *  \brief  HCI LE read remote feature command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeReadRemoteFeatCmd(uint16_t handle)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_READ_REMOTE_FEAT, HCI_LEN_LE_READ_REMOTE_FEAT)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, handle);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeReadSupStatesCmd
 *
 *  \brief  HCI LE read supported states command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeReadSupStatesCmd(void)
{
  uint8_t *pBuf;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_READ_SUP_STATES, HCI_LEN_LE_READ_SUP_STATES)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeReadWhiteListSizeCmd
 *
 *  \brief  HCI LE read white list size command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeReadWhiteListSizeCmd(void)
{
  uint8_t *pBuf;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_READ_WHITE_LIST_SIZE, HCI_LEN_LE_READ_WHITE_LIST_SIZE)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeRemoveDevWhiteListCmd
 *
 *  \brief  HCI LE remove device white list command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeRemoveDevWhiteListCmd(uint8_t addrType, uint8_t *pAddr)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_REMOVE_DEV_WHITE_LIST, HCI_LEN_LE_REMOVE_DEV_WHITE_LIST)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT8_TO_BSTREAM(p, addrType);
    BDA_TO_BSTREAM(p, pAddr);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetAdvEnableCmd
 *
 *  \brief  HCI LE set advanced enable command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetAdvEnableCmd(uint8_t enable)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_SET_ADV_ENABLE, HCI_LEN_LE_SET_ADV_ENABLE)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT8_TO_BSTREAM(p, enable);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetAdvDataCmd
 *
 *  \brief  HCI LE set advertising data command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetAdvDataCmd(uint8_t len, uint8_t *pData)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_SET_ADV_DATA, HCI_LEN_LE_SET_ADV_DATA)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT8_TO_BSTREAM(p, len);
    memcpy(p, pData, len);
    p += len;
    memset(p, 0, (HCI_ADV_DATA_LEN - len));
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetAdvParamCmd
 *
 *  \brief  HCI LE set advertising parameters command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetAdvParamCmd(uint16_t advIntervalMin, uint16_t advIntervalMax, uint8_t advType,
                         uint8_t ownAddrType, uint8_t peerAddrType, uint8_t *pPeerAddr,
                         uint8_t advChanMap, uint8_t advFiltPolicy)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_SET_ADV_PARAM, HCI_LEN_LE_SET_ADV_PARAM)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, advIntervalMin);
    UINT16_TO_BSTREAM(p, advIntervalMax);
    UINT8_TO_BSTREAM(p, advType);
    UINT8_TO_BSTREAM(p, ownAddrType);
    UINT8_TO_BSTREAM(p, peerAddrType);
    if (pPeerAddr != NULL)
    {
      BDA_TO_BSTREAM(p, pPeerAddr);
    }
    else
    {
      p = BdaClr(p);
    }
    UINT8_TO_BSTREAM(p, advChanMap);
    UINT8_TO_BSTREAM(p, advFiltPolicy);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetEventMaskCmd
 *
 *  \brief  HCI LE set event mask command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetEventMaskCmd(uint8_t *pLeEventMask)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_SET_EVENT_MASK, HCI_LEN_LE_SET_EVENT_MASK)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    memcpy(p, pLeEventMask, HCI_LE_EVT_MASK_LEN);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetHostChanClassCmd
 *
 *  \brief  HCI set host channel class command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetHostChanClassCmd(uint8_t *pChanMap)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_SET_HOST_CHAN_CLASS, HCI_LEN_LE_SET_HOST_CHAN_CLASS)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    memcpy(p, pChanMap, HCI_CHAN_MAP_LEN);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetRandAddrCmd
 *
 *  \brief  HCI LE set random address command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetRandAddrCmd(uint8_t *pAddr)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_SET_RAND_ADDR, HCI_LEN_LE_SET_RAND_ADDR)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    BDA_TO_BSTREAM(p, pAddr);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetScanEnableCmd
 *
 *  \brief  HCI LE set scan enable command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetScanEnableCmd(uint8_t enable, uint8_t filterDup)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_SET_SCAN_ENABLE, HCI_LEN_LE_SET_SCAN_ENABLE)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT8_TO_BSTREAM(p, enable);
    UINT8_TO_BSTREAM(p, filterDup);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetScanParamCmd
 *
 *  \brief  HCI set scan parameters command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetScanParamCmd(uint8_t scanType, uint16_t scanInterval, uint16_t scanWindow,
                          uint8_t ownAddrType, uint8_t scanFiltPolicy)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_SET_SCAN_PARAM, HCI_LEN_LE_SET_SCAN_PARAM)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT8_TO_BSTREAM(p, scanType);
    UINT16_TO_BSTREAM(p, scanInterval);
    UINT16_TO_BSTREAM(p, scanWindow);
    UINT8_TO_BSTREAM(p, ownAddrType);
    UINT8_TO_BSTREAM(p, scanFiltPolicy);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetScanRespDataCmd
 *
 *  \brief  HCI LE set scan response data.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetScanRespDataCmd(uint8_t len, uint8_t *pData)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_SET_SCAN_RESP_DATA, HCI_LEN_LE_SET_SCAN_RESP_DATA)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT8_TO_BSTREAM(p, len);
    memcpy(p, pData, len);
    p += len;
    memset(p, 0, (HCI_SCAN_DATA_LEN - len));
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeStartEncryptionCmd
 *
 *  \brief  HCI LE start encryption command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeStartEncryptionCmd(uint16_t handle, uint8_t *pRand, uint16_t diversifier, uint8_t *pKey)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_START_ENCRYPTION, HCI_LEN_LE_START_ENCRYPTION)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, handle);
    memcpy(p, pRand, HCI_RAND_LEN);
    p += HCI_RAND_LEN;
    UINT16_TO_BSTREAM(p, diversifier);
    memcpy(p, pKey, HCI_KEY_LEN);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciReadBdAddrCmd
 *
 *  \brief  HCI read BD address command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciReadBdAddrCmd(void)
{
  uint8_t *pBuf;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_READ_BD_ADDR, HCI_LEN_READ_BD_ADDR)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciReadBufSizeCmd
 *
 *  \brief  HCI read buffer size command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciReadBufSizeCmd(void)
{
  uint8_t *pBuf;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_READ_BUF_SIZE, HCI_LEN_READ_BUF_SIZE)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciReadLocalSupFeatCmd
 *
 *  \brief  HCI read local supported feature command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciReadLocalSupFeatCmd(void)
{
  uint8_t *pBuf;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_READ_LOCAL_SUP_FEAT, HCI_LEN_READ_LOCAL_SUP_FEAT)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciReadLocalVerInfoCmd
 *
 *  \brief  HCI read local version info command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciReadLocalVerInfoCmd(void)
{
  uint8_t *pBuf;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_READ_LOCAL_VER_INFO, HCI_LEN_READ_LOCAL_VER_INFO)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciReadRemoteVerInfoCmd
 *
 *  \brief  HCI read remote version info command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciReadRemoteVerInfoCmd(uint16_t handle)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_READ_REMOTE_VER_INFO, HCI_LEN_READ_REMOTE_VER_INFO)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, handle);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciReadRssiCmd
 *
 *  \brief  HCI read RSSI command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciReadRssiCmd(uint16_t handle)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_READ_RSSI, HCI_LEN_READ_RSSI)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, handle);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciReadTxPwrLvlCmd
 *
 *  \brief  HCI read Tx power level command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciReadTxPwrLvlCmd(uint16_t handle, uint8_t type)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_READ_TX_PWR_LVL, HCI_LEN_READ_TX_PWR_LVL)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, handle);
    UINT8_TO_BSTREAM(p, type);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     hciClearCmdQueue
 *
 *  \brief  Clears the command queue
 *
 *  \return None.
 */
/*************************************************************************************************/
void hciClearCmdQueue(void)
{
  uint8_t *pBuf;
  wsfHandlerId_t  handlerId;

  // Free up any unsent HCI commandss
  while((pBuf = WsfMsgDeq(&hciCmdCb.cmdQueue, &handlerId)) != NULL)
  {
    WsfMsgFree(pBuf);
  }

  /* initialize numCmdPkts for special case of start the reset sequence */
  hciCmdCb.numCmdPkts = 1;
}

/*************************************************************************************************/
/*!
 *  \fn     HciResetCmd
 *
 *  \brief  HCI reset command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciResetCmd(void)
{
  uint8_t *pBuf;
  hciHwErrorEvt_t evt;

  // let security module to clean up pending request/command
  evt.hdr.event = HCI_HW_ERROR_CBACK_EVT;

  hciCb.secCback((hciEvt_t *)&evt);

  hciClearCmdQueue();

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_RESET, HCI_LEN_RESET)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciSetEventMaskCmd
 *
 *  \brief  HCI set event mask command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciSetEventMaskCmd(uint8_t *pEventMask)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_SET_EVENT_MASK, HCI_LEN_SET_EVENT_MASK)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    memcpy(p, pEventMask, HCI_EVT_MASK_LEN);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
*  \fn     HciSetEventMaskPage2Cmd
*
*  \brief  HCI set event mask page 2 command.
*
*  \return None.
*/
/*************************************************************************************************/
void HciSetEventMaskPage2Cmd(uint8_t *pEventMask)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_SET_EVENT_MASK_PAGE2, HCI_LEN_SET_EVENT_MASK_PAGE2)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    memcpy(p, pEventMask, HCI_EVT_MASK_PAGE_2_LEN);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
*  \fn     HciReadAuthPayloadTimeout
*
*  \brief  HCI read authenticated payload timeout command.
*
*  \param  handle    Connection handle.
*
*  \return None.
*/
/*************************************************************************************************/
void HciReadAuthPayloadTimeout(uint16_t handle)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_READ_AUTH_PAYLOAD_TO, HCI_LEN_READ_AUTH_PAYLOAD_TO)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, handle);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
*  \fn     HciWriteAuthPayloadTimeout
*
*  \brief  HCI write authenticated payload timeout command.
*
*  \param  handle    Connection handle.
*  \param  timeout   Timeout value.
*
*  \return None.
*/
/*************************************************************************************************/
void HciWriteAuthPayloadTimeout(uint16_t handle, uint16_t timeout)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_WRITE_AUTH_PAYLOAD_TO, HCI_LEN_WRITE_AUTH_PAYLOAD_TO)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, handle);
    UINT16_TO_BSTREAM(p, timeout);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeAddDeviceToResolvingListCmd
 *
 *  \brief  HCI add device to resolving list command.
 *
 *  \param  peerAddrType        Peer identity address type.
 *  \param  pPeerIdentityAddr   Peer identity address.
 *  \param  pPeerIrk            Peer IRK.
 *  \param  pLocalIrk           Local IRK.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeAddDeviceToResolvingListCmd(uint8_t peerAddrType, const uint8_t *pPeerIdentityAddr,
                                      const uint8_t *pPeerIrk, const uint8_t *pLocalIrk)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_ADD_DEV_RES_LIST, HCI_LEN_LE_ADD_DEV_RES_LIST)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT8_TO_BSTREAM(p, peerAddrType);
    BDA_TO_BSTREAM(p, pPeerIdentityAddr);
    memcpy(p, pPeerIrk, HCI_KEY_LEN);
    p += HCI_KEY_LEN;
    memcpy(p, pLocalIrk, HCI_KEY_LEN);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeRemoveDeviceFromResolvingList
 *
 *  \brief  HCI remove device from resolving list command.
 *
 *  \param  peerAddrType        Peer identity address type.
 *  \param  pPeerIdentityAddr   Peer identity address.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeRemoveDeviceFromResolvingList(uint8_t peerAddrType, const uint8_t *pPeerIdentityAddr)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_REMOVE_DEV_RES_LIST, HCI_LEN_LE_REMOVE_DEV_RES_LIST)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT8_TO_BSTREAM(p, peerAddrType);
    BDA_TO_BSTREAM(p, pPeerIdentityAddr);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeClearResolvingList
 *
 *  \brief  HCI clear resolving list command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeClearResolvingList(void)
{
  uint8_t *pBuf;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_CLEAR_RES_LIST, HCI_LEN_LE_CLEAR_RES_LIST)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeReadResolvingListSize
 *
 *  \brief  HCI read resolving list command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeReadResolvingListSize(void)
{
  uint8_t *pBuf;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_READ_RES_LIST_SIZE, HCI_LEN_LE_READ_RES_LIST_SIZE)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeReadPeerResolvableAddr
 *
 *  \brief  HCI read peer resolvable address command.
 *
 *  \param  addrType        Peer identity address type.
 *  \param  pIdentityAddr   Peer identity address.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeReadPeerResolvableAddr(uint8_t addrType, const uint8_t *pIdentityAddr)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_READ_PEER_RES_ADDR, HCI_LEN_LE_READ_PEER_RES_ADDR)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT8_TO_BSTREAM(p, addrType);
    BDA_TO_BSTREAM(p, pIdentityAddr);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeReadLocalResolvableAddr
 *
 *  \brief  HCI read local resolvable address command.
 *
 *  \param  addrType        Peer identity address type.
 *  \param  pIdentityAddr   Peer identity address.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeReadLocalResolvableAddr(uint8_t addrType, const uint8_t *pIdentityAddr)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_READ_LOCAL_RES_ADDR, HCI_LEN_LE_READ_LOCAL_RES_ADDR)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT8_TO_BSTREAM(p, addrType);
    BDA_TO_BSTREAM(p, pIdentityAddr);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetAddrResolutionEnable
 *
 *  \brief  HCI enable or disable address resolution command.
 *
 *  \param  enable      Set to TRUE to enable address resolution or FALSE to disable address
 *                      resolution.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetAddrResolutionEnable(uint8_t enable)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_SET_ADDR_RES_ENABLE, HCI_LEN_LE_SET_ADDR_RES_ENABLE)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT8_TO_BSTREAM(p, enable);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetResolvablePrivateAddrTimeout
 *
 *  \brief  HCI set resolvable private address timeout command.
 *
 *  \param  rpaTimeout    Timeout measured in seconds.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetResolvablePrivateAddrTimeout(uint16_t rpaTimeout)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_SET_RES_PRIV_ADDR_TO, HCI_LEN_LE_SET_RES_PRIV_ADDR_TO)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, rpaTimeout);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \fn         HciLeSetPrivacyModeCmd
 *
 *  \brief      HCI LE set privacy mode command.
 *
 *  \param      peerAddrType    Peer identity address type.
 *  \param      pPeerAddr       Peer identity address.
 *  \param      mode            Privacy mode.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void HciLeSetPrivacyModeCmd(uint8_t addrType, uint8_t *pAddr, uint8_t mode)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_SET_PRIVACY_MODE, HCI_LEN_LE_SET_PRIVACY_MODE)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT8_TO_BSTREAM(p, addrType);
    BDA_TO_BSTREAM(p, pAddr);
    UINT8_TO_BSTREAM(p, mode);
    hciCmdSend(pBuf);
  }
}


/*************************************************************************************************/
/*!
 *  \fn         HciLeReceiverTestCmd
 *
 *  \brief      HCI LE receiver test command.
 *
 *  \param      RX_Channel    Radio Channel range from 0 to 27
 *
 *  \return     None.
 */
/*************************************************************************************************/
void HciLeReceiverTestCmd(uint8_t RX_Channel)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_RECEIVER_TEST, HCI_LEN_LE_RECEIVER_TEST)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT8_TO_BSTREAM(p, RX_Channel);
    hciCmdSend(pBuf);
  }
}


/*************************************************************************************************/
/*!
 *  \fn         HciLeTransmitterTestCmd
 *
 *  \brief      HCI LE transmitter test command.
 *
 *  \param      TX_Channel      Radio Channel range from 0 to 27
 *  \param      len_of_test_data range from 0 to 0xff.
 *  \param      packet_payload  Range 0-7, refer to BT spec.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void HciLeTransmitterTestCmd(uint8_t TX_Channel, uint8_t len_of_test_data, uint8_t packet_payload)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_TRANSMITTER_TEST, HCI_LEN_LE_TRANSMITTER_TEST)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT8_TO_BSTREAM(p, TX_Channel);
    UINT8_TO_BSTREAM(p, len_of_test_data);
    UINT8_TO_BSTREAM(p, packet_payload);
    hciCmdSend(pBuf);
  }
}


/*************************************************************************************************/
/*!
 *  \fn         HciLeTestEndCmd
 *
 *  \brief      HCI LE test end command.
 *
 *  \param      None
 *
 *  \return     None.
 */
/*************************************************************************************************/
void HciLeTestEndCmd(void)
{
  uint8_t *pBuf;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_TEST_END, HCI_LEN_LE_TEST_END)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}


/*************************************************************************************************/
/*!
 *  \fn         HciLeReceiverTestCmdV3
 *
 *  \brief      HCI LE Receiver test command[V3].
 *
 *  \param      hciLeRxTestV3Cmd_t
 *  \return     None.
 */
/*************************************************************************************************/
void HciLeReceiverTestCmdV3(hciLeRxTestV3Cmd_t *rx_test_v3)
{
    uint8_t *pBuf;
    uint8_t *p;

    if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_RECEIVER_TEST_V3, HCI_LEN_LE_RECEIVER_TEST_V3)) != NULL)
    {
        p = pBuf + HCI_CMD_HDR_LEN;
        UINT8_TO_BSTREAM(p, rx_test_v3->rx_channel);
        UINT8_TO_BSTREAM(p, rx_test_v3->phy);
        UINT8_TO_BSTREAM(p, rx_test_v3->mod_idx);
        UINT8_TO_BSTREAM(p, rx_test_v3->exp_cte_len);
        UINT8_TO_BSTREAM(p, rx_test_v3->exp_cte_type);
        UINT8_TO_BSTREAM(p, rx_test_v3->slot_dur);
        UINT8_TO_BSTREAM(p, rx_test_v3->switching_pattern_len);
        memcpy(p, rx_test_v3->antenna_id, MAX_SWITCHING_PATTERN_LEN);

        hciCmdSend(pBuf);
    }
}

/*************************************************************************************************/
/*!
 *  \fn         HciLeTransmitterTestCmdV3
 *
 *  \brief      HCI LE transmitter test command[V3].
 *
 *  \param      hciLeTxTestV3Cmd_t
 *  \return     None.
 */
/*************************************************************************************************/
void HciLeTransmitterTestCmdV3(hciLeTxTestV3Cmd_t *tx_test_v3)
{
    uint8_t *pBuf;
    uint8_t *p;

    if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_TRANSMITTER_TEST_V3, HCI_LEN_LE_TRANSMITTER_TEST_V3)) != NULL)
    {
        p = pBuf + HCI_CMD_HDR_LEN;
        UINT8_TO_BSTREAM(p, tx_test_v3->tx_channel);
        UINT8_TO_BSTREAM(p, tx_test_v3->test_data_len);
        UINT8_TO_BSTREAM(p, tx_test_v3->pkt_payl);
        UINT8_TO_BSTREAM(p, tx_test_v3->phy);
        UINT8_TO_BSTREAM(p, tx_test_v3->cte_len);
        UINT8_TO_BSTREAM(p, tx_test_v3->cte_type);
        UINT8_TO_BSTREAM(p, tx_test_v3->switching_pattern_len);
        memcpy(p, tx_test_v3->antenna_id, MAX_SWITCHING_PATTERN_LEN);

        hciCmdSend(pBuf);
    }
}

/*************************************************************************************************/
/*!
 *  \fn     HciVendorSpecificCmd
 *
 *  \brief  HCI vencor specific command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciVendorSpecificCmd(uint16_t opcode, uint8_t len, uint8_t *pData)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(opcode, len)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    memcpy(p, pData, len);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \brief      HCI LE request peer SCA command.
 *
 *  \param      handle    Connection handle.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void HciLeRequestPeerScaCmd(uint16_t handle)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_REQUEST_PEER_SCA, HCI_LEN_LE_REQUEST_PEER_SCA)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT16_TO_BSTREAM(p, handle);
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  HCI LE read buffer size version 2 command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeReadBufSizeCmdV2(void)
{
  uint8_t *pBuf;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_READ_BUF_SIZE_V2, HCI_LEN_LE_READ_BUF_SIZE)) != NULL)
  {
    hciCmdSend(pBuf);
  }
}

/*************************************************************************************************/
/*!
 *  \brief      HCI LE set host feature command.
 *
 *  \param      bitNum    Bit position in the FeatureSet.
 *  \param      bitVal    Enable or disable feature.
 *
 *  \return     None.
 *
 *  \note Set or clear a bit in the feature controlled by the Host in the Link Layer FeatureSet
 *  stored in the Controller.
 */
/*************************************************************************************************/
void HciLeSetHostFeatureCmd(uint8_t bitNum, bool_t bitVal)
{
  uint8_t *pBuf;
  uint8_t *p;

  if ((pBuf = hciCmdAlloc(HCI_OPCODE_LE_SET_HOST_FEATURE, HCI_LEN_LE_SET_HOST_FEATURE)) != NULL)
  {
    p = pBuf + HCI_CMD_HDR_LEN;
    UINT8_TO_BSTREAM(p, bitNum);
    UINT8_TO_BSTREAM(p, bitVal);
    hciCmdSend(pBuf);
  }
}

