/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Wireless Data Exchange profile implementation.
 *
 *  Copyright (c) 2013-2018 Arm Ltd. All Rights Reserved.
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

#ifndef WDXS_MAIN_H
#define WDXS_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup WIRELESS_DATA_EXCHANGE_PROFILE
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/

/** \name WDXS Default Feature Set
 *
 */
/**@{*/
#ifndef WDXS_DC_ENABLED
#define WDXS_DC_ENABLED             TRUE
#endif

#ifndef WDXS_AU_ENABLED
#define WDXS_AU_ENABLED             TRUE
#endif

#ifndef WDXS_OTA_ENABLED
#define WDXS_OTA_ENABLED            TRUE
#endif
/**@}*/

/*! \brief Special length for streaming file */
#define WDXS_STREAM_FILE_LEN        0xFFFFFFFF

/** \name WSF event types for application event handler
 *
 */
/**@{*/
#define WDXS_EVT_TX_PATH            0x01      /*!< \brief Trigger tx data path */
#define WDXS_EVT_AU_SEC_COMPLETE    0x02      /*!< \brief AU encryption of challenge ready */
/**@}*/

/** \name TX Ready Mask Bits
 *
 */
/**@{*/
#define WDXS_TX_MASK_READY_BIT    (1<<0) /*!< \brief Ready bit */
#define WDXS_TX_MASK_DC_BIT       (1<<1) /*!< \brief DC bit */
#define WDXS_TX_MASK_FTC_BIT      (1<<2) /*!< \brief FTC bit */
#define WDXS_TX_MASK_FTD_BIT      (1<<3) /*!< \brief FTD bit */
#define WDXS_TX_MASK_AU_BIT       (1<<4) /*!< \brief AU bit */
/**@}*/

/** \name Authentication states
 *
 */
/**@{*/
#define WDXS_AU_STATE_UNAUTHORIZED      0x00        /*!< \brief Authentication has not started */
#define WDXS_AU_STATE_HASHING           0x01        /*!< \brief Authentication hash is being calculated */
#define WDXS_AU_STATE_WAIT_SEC          0x02        /*!< \brief Authentication challenge sent */
#define WDXS_AU_STATE_WAIT_REPLY        0x03        /*!< \brief Authentication waiting for challenge reply */
#define WDXS_AU_STATE_AUTHORIZED        0x04        /*!< \brief Authentication completed successfully */
/**@}*/

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! \brief WDXS Device Configuration PHY Write Callback */
typedef uint8_t (*wdxsDcPhyWriteCback_t)(dmConnId_t connId, uint8_t op, uint8_t id, uint16_t len,
                                         uint8_t *pValue);

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! \brief WDXS profile control block */
typedef struct
{
  wsfHandlerId_t    handlerId;        /*!< \brief WSF handler ID */
  uint8_t           txReadyMask;      /*!< \brief Bits indicate DC, FTC, FTD, and/or AU wish to transmit */

  /* connection parameters */
  uint16_t          connInterval;     /*!< \brief connection interval */
  uint16_t          connLatency;      /*!< \brief connection latency */
  uint16_t          supTimeout;       /*!< \brief supervision timeout */

  /* Phy parameters */
  uint8_t           txPhy;            /*!< \brief transmitter PHY */
  uint8_t           rxPhy;            /*!< \brief receiver PHY */

  /* for file transfer */
  uint32_t          ftOffset;         /*!< \brief file data offset */
  uint32_t          ftLen;            /*!< \brief remaining data length for current operation */
  uint32_t          ftTotalLen;       /*!< \brief file total length */
  uint16_t          ftHandle;         /*!< \brief file handle */
  uint16_t          ftcMsgLen;        /*!< \brief message length */
  uint8_t           ftcMsgBuf[ATT_DEFAULT_PAYLOAD_LEN]; /*!< \brief message buffer */
  uint8_t           ftInProgress;     /*!< \brief operation in progress */
  uint8_t           ftPrefXferType;   /*!< \brief Preferred transport type */

  /* ccc index */
  uint8_t          dcCccIdx;          /*!< \brief device configuration ccc index */
  uint8_t          auCccIdx;          /*!< \brief authentication ccc index */
  uint8_t          ftcCccIdx;         /*!< \brief file transfer control ccc index */
  uint8_t          ftdCccIdx;         /*!< \brief file transfer data ccc index */
} wdxsCb_t;

/*! \brief WDXS Device Configuration Control Block */
typedef struct
{
  uint16_t              dcMsgLen;                           /*!< \brief message length */
  uint8_t               dcMsgBuf[ATT_DEFAULT_PAYLOAD_LEN];  /*!< \brief message buffer */
  bool_t                doReset;                            /*!< \brief Reset device after disconnect */
  wdxsDcPhyWriteCback_t phyWriteCback;                      /*!< \brief Device config PHY write callback */
} wdxsDcCb_t;

/*! \brief WDXS Authentication Control Block */
typedef struct
{
  uint8_t           auMsgBuf[ATT_DEFAULT_PAYLOAD_LEN];  /*!< \brief message buffer */
  uint8_t           auRand[WDX_AU_RAND_LEN];            /*!< \brief random challenge */
  uint8_t           sessionKey[WDX_AU_KEY_LEN];         /*!< \brief session key */
  uint8_t           auHash[WDX_AU_HASH_LEN];            /*!< \brief session key */
  uint16_t          auMsgLen;                           /*!< \brief message length */
  uint8_t           authLevel;                          /*!< \brief current authentication level */
  uint8_t           authMode;                           /*!< \brief current authentication mode */
  uint8_t           reqAuthLevel;                       /*!< \brief requested authentication level */
  uint8_t           authState;                          /*!< \brief authentication protocol state */
} wdxsAuCb_t;

/*! \brief WDXS event message union */
typedef union
{
  wsfMsgHdr_t       hdr; /*!< header */
  dmEvt_t           dm;  /*!< DM event */
  attsCccEvt_t      ccc; /*!< ATT CCC event */
} wdxsMsg_t;

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/** \name WDXS Control Block External Declaration
 *
 */
/**@{*/
extern wdxsCb_t wdxsCb;     /*!< \brief WDXS control block */
extern wdxsAuCb_t wdxsAuCb; /*!< \brief WDXS AU control block */
extern wdxsDcCb_t wdxsDcCb; /*!< \brief WDXS DC control block */
/**@}*/

/**************************************************************************************************
  Global Function Prototypes
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Send device configuration notification
 *
 *  \param  connId   DM connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
void wdxsDcSend(dmConnId_t connId);

/*************************************************************************************************/
/*!
 *  \brief  Send a file transfer control characteristic notification.
 *
 *  \param  connId   DM connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
void wdxsFtcSend(dmConnId_t connId);

/*************************************************************************************************/
/*!
 *  \brief  Send a file transfer data characteristic notification.
 *
 *  \param  connId   DM connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
void wdxsFtdSend(dmConnId_t connId);

/*************************************************************************************************/
/*!
 *  \brief  Transmit to authentication characteristic.
 *
 *  \param  connId   DM connection identifier.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
void wdxsAuSend(dmConnId_t connId);

/*************************************************************************************************/
/*!
 *  \brief  Process a write to the device configuration characteristic.
 *
 *  \param  connId   DM connection identifier.
 *  \param  len      Length to write.
 *  \param  pValue   value to write.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t wdxsDcWrite(dmConnId_t connId, uint16_t len, uint8_t *pValue);

/*************************************************************************************************/
/*!
 *  \brief  Process a write to the file transfer control characteristic.
 *
 *  \param  connId   DM connection identifier.
 *  \param  len      Length to write.
 *  \param  pValue   Value to write.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t wdxsFtcWrite(dmConnId_t connId, uint16_t len, uint8_t *pValue);

/*************************************************************************************************/
/*!
 *  \brief  Process a write to the file transfer data characteristic.
 *
 *  \param  connId   DM connection identifier.
 *  \param  len      Length to write.
 *  \param  pValue   Value to write.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t wdxsFtdWrite(dmConnId_t connId, uint16_t len, uint8_t *pValue);

/*************************************************************************************************/
/*!
 *  \brief  Process a write to the authentication characteristic.
 *
 *  \param  connId   DM connection identifier.
 *  \param  len      Length to write.
 *  \param  pValue   Value to write
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t wdxsAuWrite(dmConnId_t connId, uint16_t len, uint8_t *pValue);

/*************************************************************************************************/
/*!
 *  \brief  Send update message for connection parameters.
 *
 *  \param  connId   DM connection identifier.
 *  \param  status   Update status.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t wdxsDcUpdateConnParam(dmConnId_t connId, uint8_t status);

/*************************************************************************************************/
/*!
 *  \brief  Send update message for PHY.
 *
 *  \param  connId   DM connection identifier.
 *  \param  status   Update status.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t wdxsDcUpdatePhy(dmConnId_t connId, uint8_t status);

/*************************************************************************************************/
/*!
 *  \brief  Register a PHY write callback for the device configuration characteristic.
 *
 *  \param  cback  PHY callback function.
 *
 *  \return None.
 */
/*************************************************************************************************/
void wdxsDcPhyRegister(wdxsDcPhyWriteCback_t cback);

/*************************************************************************************************/
/*!
 *  \brief  Create the file list.
 *
 *  \return none.
 */
/*************************************************************************************************/
void WdxsUpdateListing(void);

/*! \} */    /* WIRELESS_DATA_EXCHANGE_PROFILE */

#ifdef __cplusplus
}
#endif

#endif /* WDXS_MAIN_H */
