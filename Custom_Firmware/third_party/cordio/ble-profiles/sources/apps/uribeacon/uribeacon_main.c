/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  UriBeacon sample application.
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

#include "uribeacon/uricfg_api.h"
#include "uribeacon/uricfg_defs.h"
#include "svc_uricfg.h"
#include "app_api.h"
#include "app_db.h"
#include "app_ui.h"
#include "app_main.h"
#include "att_defs.h"
#include "util/bstream.h"
#include "dm_api.h"
#include "sec_api.h"
#include "svc_core.h"
#include "svc_dis.h"
#include "gatt/gatt_api.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "wsf_types.h"
// #include "wsf_nvm.h"
#include "uribeacon/uribeacon_api.h"
#include "atts_main.h"

#define WsfNvmWriteData(A, B, C, D) 0
#define WsfNvmReadData(A, B, C, D) 0

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! default lock state  */
#define URIBEACON_LOCKSTATE_DEFAULT         0

/*! default URI data value ("http://www.arm.com") */
#define URIBEACON_URIDATA_DEFAULT           0x00, 'a', 'r', 'm', 0x07

/*! clamp beacon period to allowed range */
#define URIBEACON_BEACONPERIOD_CLAMP(bp)        (((bp) < URICFG_ATT_BEACONPERIOD_MIN) ? URICFG_ATT_BEACONPERIOD_MIN : \
                                                 ((bp) > URICFG_ATT_BEACONPERIOD_MAX) ? URICFG_ATT_BEACONPERIOD_MAX : \
                                                  (bp))
/*! convert beacon period from ms to 0.625-ms units */
#define URIBEACON_BEACONPERIOD_TO_INTERVAL(bp) (((URIBEACON_BEACONPERIOD_CLAMP(bp)) * 8u) / 5u)

/*! size of advertising information */
#define URIBEACON_INFO_SIZE                 24

/*! offsets of information within advertising data */
#define URIBEACON_INFO_OFFSET               3  /*! offset of advertising information within advertising data */
#define URIBEACON_INFO_SVCDATASIZE_OFFSET   7  /*! offset of service data size */
#define URIBEACON_INFO_URIFLAGS_OFFSET      11 /*! offset of URI flags. */
#define URIBEACON_INFO_TXPWRLEVEL_OFFSET    12 /*! offset of Tx power level */
#define URIBEACON_INFO_URIDATA_OFFSET       13 /*! offset of URI data */

/*! default adv Tx power levels */
static const int8_t uriAdvertisedTxPwrLevelsDefault[] = {-80, -60, -40, -20};

/*! WSF message event starting value */
#define URIBEACON_MSG_START                 0xA0

/*! WSF message event enumeration */
enum
{
  URIBEACON_ADV_TIMER_IND = URIBEACON_MSG_START   /*! Advertising timer expired */
};

/*! Advertising timer timeout */
#define URIBEACON_ADV_TIMEOUT_SEC           30

/**************************************************************************************************
Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! enumeration of client characteristic configuration descriptors */
enum
{
  URIBEACON_GATT_SC_CCC_IDX,                    /*! GATT service, service changed characteristic */
  URIBEACON_NUM_CCC_IDX
};

/*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t uriBeaconCccSet[URIBEACON_NUM_CCC_IDX] =
{
  /* cccd handle        value range              security level */
  { GATT_SC_CH_CCC_HDL, ATT_CLIENT_CFG_INDICATE, DM_SEC_LEVEL_NONE } /* URIBEACON_GATT_SC_CCC_IDX */
};

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! WSF handler ID */
static wsfHandlerId_t uriBeaconHandlerId;

/*! Advertising mode timer */
static wsfTimer_t uriBeaconAdvTimer;
static uint8_t    uriBeaconAdvType;

/* App extention callback */
static uribeaconExtCback_t uriBeaconExtCback;

/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/

/*! configurable parameters for advertising */
static appAdvCfg_t uriBeaconAdvCfg =
{
  {0, 0, 0},                              /*! Advertising durations in ms */
  {0, 0, 0},                              /*! Advertising intervals in 0.625 ms units */
};

/*! configurable parameters for slave */
static appSlaveCfg_t uriBeaconSlaveCfg =
{
  1                                       /*! Maximum connections */
};

/*! configurable parameters for security */
static const appSecCfg_t uriBeaconSecCfg =
{
  DM_AUTH_BOND_FLAG,                      /*! Authentication and bonding flags */
  0,                                      /*! Initiator key distribution flags */
  DM_KEY_DIST_LTK,                        /*! Responder key distribution flags */
  FALSE,                                  /*! TRUE if Out-of-band pairing data is present */
  FALSE                                   /*! TRUE to initiate security upon connection */
};

/*! configurable parameters for connection parameter update */
static const appUpdateCfg_t uriBeaconUpdateCfg =
{
  0,                                      /*! Connection idle period in ms before attempting
                                             connection parameter update; set to zero to disable */
  640,                                    /*! Minimum connection interval in 1.25ms units */
  800,                                    /*! Maximum connection interval in 1.25ms units */
  0,                                      /*! Connection latency */
  600,                                    /*! Supervision timeout in 10ms units */
  5                                       /*! Number of update attempts before giving up */
};

/**************************************************************************************************
  Advertising Data
**************************************************************************************************/
static int8_t uriBeaconAdvertisedTxPwrLevels[URICFG_SIZE_TXPWRLEVELS_ATT];
static uint8_t uriBeaconTxPwrMode;
static const uint8_t uriBeaconUriData[] = {URIBEACON_URIDATA_DEFAULT};

/* URI data param override */
static uint8_t *uriDataOverride;
static uint8_t uriDataOverrideLen;

/* advertising data, discoverable mode */
static uint8_t uriBeaconAdvDataDisc[HCI_ADV_DATA_LEN] =
{
  /* advertising type flags */
  1u + sizeof(uint8_t),
  DM_ADV_TYPE_FLAGS,
  DM_FLAG_LE_LIMITED_DISC | DM_FLAG_LE_BREDR_NOT_SUP,

  /* UriBeacon service UUID */
  3u,
  DM_ADV_TYPE_16_UUID,
  UINT16_TO_BYTES(URICFG_SERVICE_UUID),

  /* UriBeacon service data */
  5u + sizeof(uriBeaconUriData), /* length (5-23) */
  DM_ADV_TYPE_SERVICE_DATA,
  UINT16_TO_BYTES(URICFG_SERVICE_UUID),
  0u,                                 /* flags */
  0,                                  /* tx power level */
  URIBEACON_URIDATA_DEFAULT           /* URI */
};

/* scan data, discoverable and connectable modes */
static uint8_t uriBeaconScanDataDisc[HCI_ADV_DATA_LEN];
static uint8_t uriBeaconScanDataConn[HCI_ADV_DATA_LEN];
static const uint8_t uriBeaconScanDataLocalName[] = { 'U', 'r', 'i', 'B', 'c', 'n' };
static const uint8_t uriBeaconScanData128Uuid[] = { URICFG_UUID_BYTES(URICFG_UUID_SVC) };

static const char * const uriBeaconPrefixes[] =
{
  "http://www.", "https://www.", "http://", "https://", "urn:uuid:"
};
static const char * const uriBeaconCodes[] =
{
  ".com/", ".org/", ".edu/", ".net/", ".info/", ".biz/", ".gov/",
  ".com",  ".org",  ".edu",  ".net",  ".info",  ".biz",  ".gov",
};

/*************************************************************************************************/
/*!
 *  \brief  Print URI data as URI.
 *
 *  \param  pUriData      URI data.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void uriBeaconPrintUri(const uint8_t *pUriData)
{
  uint8_t c;
  uint8_t len = 0u;
  char    uri[2 * URICFG_MAXSIZE_URIDATA_ATT];
  uint8_t uriOffset;

  memset(uri, 0, sizeof(uri));
  uriOffset = 0;

  c = *pUriData++;
  len++;
  if (c < sizeof(uriBeaconPrefixes) / sizeof(uriBeaconPrefixes[0]))
  {
    memcpy(uri + uriOffset, uriBeaconPrefixes[c], strlen(uriBeaconPrefixes[c]));
    uriOffset += (uint8_t) strlen(uriBeaconPrefixes[c]);
  }
  else
  {
    uri[uriOffset] = '?';
    APP_TRACE_INFO1("URI = %s", uri);
    return;
  }

  while (len < URICFG_MAXSIZE_URIDATA_ATT)
  {
    c = *pUriData++;
    if ((c > 0x20u) && (c < 0x7F))
    {
      uri[uriOffset++] = (char)c;
    }
    else if (c < sizeof(uriBeaconCodes) / sizeof(uriBeaconCodes[0]))
    {
      memcpy(uri + uriOffset, uriBeaconCodes[c], strlen(uriBeaconCodes[c]));
      uriOffset += (uint8_t) strlen(uriBeaconCodes[c]);
    }
    else
    {
      break;
    }
  }
  APP_TRACE_INFO1("URI = %s", uri);
}

/*************************************************************************************************/
/*!
 *  \brief  Set advertising type.
 *
 *  \param  advType       Advertising type
 *
 *  \return None.
 */
/*************************************************************************************************/
static void uriBeaconSetAdvType(uint8_t advType)
{
  uriBeaconAdvType = advType;
  AppSetAdvType(advType);
}

/*************************************************************************************************/
/*!
 *  \brief  Update URI data in advertising data.
 *
 *  \param  pUriData      URI data.
 *  \param  uriDataLen    URI data length.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void uriBeaconUpdateAdvUriData(const uint8_t *pUriData, uint8_t uriDataLen)
{
  uint16_t len;

  for (len = 0u; len < uriDataLen; len++)
  {
    if (pUriData[len] == 0xFFu)
    {
      break;
    }
  }
  memcpy(&uriBeaconAdvDataDisc[URIBEACON_INFO_URIDATA_OFFSET], pUriData, len);
  uriBeaconAdvDataDisc[URIBEACON_INFO_SVCDATASIZE_OFFSET] = 5u + len;
}

/*************************************************************************************************/
/*!
 *  \brief  Update URI flags in advertising data.
 *
 *  \param  uriFlags      URI flags.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void uriBeaconUpdateAdvUriFlags(uint8_t uriFlags)
{
  uriBeaconAdvDataDisc[URIBEACON_INFO_URIFLAGS_OFFSET] = uriFlags;
}

/*************************************************************************************************/
/*!
 *  \brief  Update Tx power level in advertising data.
 *
 *  \param  txPwrLevel    Tx power level.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void uriBeaconUpdateAdvTxPwrLevel(int8_t txPwrLevel)
{
  uriBeaconAdvDataDisc[URIBEACON_INFO_TXPWRLEVEL_OFFSET] = (uint8_t)txPwrLevel;
}

/*************************************************************************************************/
/*!
 *  \brief  Update beacon period (advertising interval).
 *
 *  \param  beaconPeriod  Beacon period in milliseconds.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void uriBeaconUpdateBeaconPeriod(uint16_t beaconPeriod)
{
  /* this never occurs while we're advertising, so we just need to update the datum */
  if (beaconPeriod == URICFG_ATT_BEACONPERIOD_DISABLE)
  {
    uriBeaconAdvCfg.advInterval[0] = 0;
  }
  else
  {
    uriBeaconAdvCfg.advInterval[0] = URIBEACON_BEACONPERIOD_TO_INTERVAL(beaconPeriod);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Set the system ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void uriBeaconSetSystemId(void)
{
  uint8_t *bdaddr = HciGetBdAddr();
  uint8_t  sysId[8];

  /* formatted according to GATT specification for System ID characteristic (0x2A23) */
  sysId[0] = bdaddr[0];
  sysId[1] = bdaddr[1];
  sysId[2] = bdaddr[2];
  sysId[3] = 0xFE;
  sysId[4] = 0xFF;
  sysId[5] = bdaddr[3];
  sysId[6] = bdaddr[4];
  sysId[7] = bdaddr[5];
  AttsSetAttr(DIS_SID_HDL, sizeof(sysId), sysId);
}

/*************************************************************************************************/
/*!
 *  \brief  Assign a random address.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void uriBeaconAssignRandomAddress(void)
{
  uint8_t addr[BDA_ADDR_LEN];
  SecRand(addr, BDA_ADDR_LEN);
  DM_RAND_ADDR_SET(addr, DM_RAND_ADDR_STATIC);
  DmDevSetRandAddr(addr);
  DmAdvSetAddrType(DM_ADDR_RANDOM);
}

/*************************************************************************************************/
/*!
 *  \brief  Application DM callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void uriBeaconDmCback(dmEvt_t *pDmEvt)
{
  dmEvt_t *pMsg;
  uint16_t  len;

  len = DmSizeOfEvt(pDmEvt);

  if (pDmEvt->hdr.event == DM_SEC_ECC_KEY_IND)
  {
    DmSecSetEccKey(&pDmEvt->eccMsg.data.key);

    // Only calculate database hash if the calculating status is in progress
    if( attsCsfGetHashUpdateStatus() )
    {
      AttsCalculateDbHash();
    }
  }

  if ((pMsg = WsfMsgAlloc(len)) != NULL)
  {
    memcpy(pMsg, pDmEvt, len);
    WsfMsgSend(uriBeaconHandlerId, pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Application ATT callback.
 *
 *  \param  pEvt    ATT callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void uriBeaconAttCback(attEvt_t *pEvt)
{
  attEvt_t *pMsg;

  if ((pMsg = WsfMsgAlloc(sizeof(attEvt_t) + pEvt->valueLen)) != NULL)
  {
    memcpy(pMsg, pEvt, sizeof(attEvt_t));
    pMsg->pValue = (uint8_t *) (pMsg + 1);
    memcpy(pMsg->pValue, pEvt->pValue, pEvt->valueLen);
    WsfMsgSend(uriBeaconHandlerId, pMsg);
  }

}

/*************************************************************************************************/
/*!
 *  \brief  Application ATTS client characteristic configuration callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void uriBeaconCccCback(attsCccEvt_t *pEvt)
{
  appDbHdl_t    dbHdl;

  /* If CCC not set from initialization and there's a device record and currently bonded */
  if ((pEvt->handle != ATT_HANDLE_NONE) &&
      ((dbHdl = AppDbGetHdl((dmConnId_t)pEvt->hdr.param)) != APP_DB_HDL_NONE) &&
      AppCheckBonded((dmConnId_t)pEvt->hdr.param))
  {
    /* Store value in device database. */
    AppDbSetCccTblValue(dbHdl, pEvt->idx, pEvt->value);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Attribute write callback.
 *
 *  \param  handle      Attribute handle.
 *  \param  valueLen    Length of value data.
 *  \param  pValue      Pointer to value data.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void uriBeaconAttWriteCback(uint16_t handle, uint16_t valueLen, const uint8_t *pValue)
{
  uint8_t id;
  bool_t  updatedAdvData = FALSE;
  uint8_t uriData[URICFG_MAXSIZE_URIDATA_ATT];

#if 1   // Ambiq added. Avoids the 'set but never used' compiler warning for the 'id' variable
    id = 0;                             // Set id to something (it's normally unitialized at this point)
    updatedAdvData = id ? TRUE : FALSE; // Use id
    updatedAdvData = FALSE;             // Make double sure to initialize to false as expected.
#endif

  switch (handle)
  {
    case URICFG_HANDLE_URIDATA:
    {
      /* pad data to maximum length so setting does not need to resize */
      memset(uriData, 0xFF, sizeof(uriData));
      memcpy(uriData, pValue, valueLen);
      pValue   = uriData;
      valueLen = URICFG_MAXSIZE_URIDATA_ATT;

      APP_TRACE_INFO0("URI upd URI data");
      uriBeaconPrintUri(uriData);
      uriBeaconUpdateAdvUriData(uriData, (uint8_t) valueLen);
      updatedAdvData = TRUE;
      id = URI_BEACON_PARAM_URI_DATA;
      break;
    }
    case URICFG_HANDLE_URIFLAGS:
    {
      APP_TRACE_INFO1("URI upd URI flags %02X", *pValue);
      uriBeaconUpdateAdvUriFlags(*pValue);
      updatedAdvData = TRUE;
      id = URI_BEACON_PARAM_URI_FLAGS;
      break;
    }
    case URICFG_HANDLE_TXPWRLEVELS:
    {
      APP_TRACE_INFO2("URI upd adv tx pwr lvls {%d, %d,", pValue[0], pValue[1]);
      APP_TRACE_INFO2("                         %d, %d}", pValue[2], pValue[3]);

      memcpy(uriBeaconAdvertisedTxPwrLevels, pValue, valueLen);
      uriBeaconUpdateAdvTxPwrLevel(uriBeaconAdvertisedTxPwrLevels[uriBeaconTxPwrMode]);
      updatedAdvData = TRUE;
      id = URI_BEACON_PARAM_ADVERTISED_TX_POWER_LEVELS;
      break;
    }
    case URICFG_HANDLE_TXPWRMODE:
    {
      APP_TRACE_INFO1("URI upd tx pwr mode %u", *pValue);
      uriBeaconTxPwrMode = *pValue;
      uriBeaconUpdateAdvTxPwrLevel(uriBeaconAdvertisedTxPwrLevels[uriBeaconTxPwrMode]);
      updatedAdvData = TRUE;
      id = URI_BEACON_PARAM_TX_POWER_MODE;
      break;
    }
    case URICFG_HANDLE_BEACONPERIOD:
    {
      uint16_t beaconPeriod;

      BYTES_TO_UINT16(beaconPeriod, pValue);
      APP_TRACE_INFO1("URI upd beacon period %u", beaconPeriod);
      uriBeaconUpdateBeaconPeriod(beaconPeriod);
      if (beaconPeriod == URICFG_ATT_BEACONPERIOD_DISABLE)
      {
        /* do not store 'disable' beacon period */
        return;
      }
      id = URI_BEACON_PARAM_BEACON_PERIOD;
      break;
    }
    case URICFG_HANDLE_LOCK:
    {
      /* only called during reset */
      APP_TRACE_INFO0("URI upd lock");
      id = URI_BEACON_PARAM_LOCK;
      break;
    }
    default:
    {
      APP_TRACE_INFO0("URI upd unknown attr");
      return;
    }
  }

  if (!WsfNvmWriteData((uint64_t)id, pValue, valueLen, NULL))
  {
    APP_TRACE_ERR0("URI failed to wr hostcfg");
  }

  if (updatedAdvData)
  {
    /* Set as Connectable so that data is updated after connection closes. */
    AppAdvSetData(APP_ADV_DATA_CONNECTABLE, sizeof(uriBeaconAdvDataDisc), (uint8_t *)uriBeaconAdvDataDisc);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Lock change callback.
 *
 *  \param  lockState   New lock state.
 *  \param  lock        Lock value.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void uriBeaconLockChangeCback(uint8_t lockState, const uint8_t lock[URICFG_SIZE_LOCK_ATT])
{
  APP_TRACE_INFO1("URI upd lock state %u", lockState);

  /* only update lock if locking */
  if (lockState)
  {
    if (!WsfNvmWriteData((uint64_t)URI_BEACON_PARAM_LOCK, lock, URICFG_SIZE_LOCK_ATT, NULL))
    {
      APP_TRACE_ERR0("URI failed to wr lock state");
      return;
    }
  }
  /* always update lock state */
  if (!WsfNvmWriteData((uint64_t)URI_BEACON_PARAM_LOCK_STATE, &lockState, sizeof(lockState), NULL))
  {
    APP_TRACE_ERR0("URI failed to wr lcok state");
    return;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Set up advertising and other procedures that need to be performed after
 *          device reset.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void uriBeaconSetup(wsfMsgHdr_t *pMsg)
{
  /* set advertising and scan response data for discoverable mode */
  AppAdvSetData(APP_ADV_DATA_DISCOVERABLE, sizeof(uriBeaconAdvDataDisc), (uint8_t *)uriBeaconAdvDataDisc);

  /* set scan response data for discoverable mode */
  memset(uriBeaconScanDataDisc, 0, sizeof(uriBeaconScanDataDisc));
  AppAdvSetData(APP_SCAN_DATA_DISCOVERABLE, 0, (uint8_t *) uriBeaconScanDataDisc);
  AppAdvSetAdValue(APP_SCAN_DATA_DISCOVERABLE, DM_ADV_TYPE_LOCAL_NAME, sizeof(uriBeaconScanDataLocalName), (uint8_t *) uriBeaconScanDataLocalName);
  AppAdvSetAdValue(APP_SCAN_DATA_DISCOVERABLE, DM_ADV_TYPE_128_UUID, sizeof(uriBeaconScanData128Uuid), (uint8_t *) uriBeaconScanData128Uuid);

  /* set advertising and scan response data for connectable mode */
  AppAdvSetData(APP_ADV_DATA_CONNECTABLE, sizeof(uriBeaconAdvDataDisc), (uint8_t *)uriBeaconAdvDataDisc);

  /* set scan response data for connectable mode */
  memset(uriBeaconScanDataDisc, 0, sizeof(uriBeaconScanDataDisc));
  AppAdvSetData(APP_SCAN_DATA_CONNECTABLE, 0, (uint8_t *) uriBeaconScanDataConn);
  AppAdvSetAdValue(APP_SCAN_DATA_CONNECTABLE, DM_ADV_TYPE_LOCAL_NAME, sizeof(uriBeaconScanDataLocalName), (uint8_t *) uriBeaconScanDataLocalName);
  AppAdvSetAdValue(APP_SCAN_DATA_CONNECTABLE, DM_ADV_TYPE_128_UUID, sizeof(uriBeaconScanData128Uuid), (uint8_t *) uriBeaconScanData128Uuid);

  /* set system ID according to BDADDR */
  uriBeaconSetSystemId();

  /* assign a random address before we start advertising */
  uriBeaconAssignRandomAddress();

  /* start timer for advertising mode change */
  memset(&uriBeaconAdvTimer, 0, sizeof(wsfTimer_t));
  uriBeaconAdvTimer.handlerId = uriBeaconHandlerId;
  uriBeaconAdvTimer.msg.event = URIBEACON_ADV_TIMER_IND;
  WsfTimerStartSec(&uriBeaconAdvTimer, URIBEACON_ADV_TIMEOUT_SEC);

  /* start advertising */
  uriBeaconSetAdvType(DM_ADV_CONN_UNDIRECT);
  AppAdvStart(APP_MODE_CONNECTABLE);
}

/*************************************************************************************************/
/*!
 *  \brief  Button press callback.
 *
 *  \param  btn    Button press.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void uriBeaconBtnCback(uint8_t btn)
{
  /* button actions when connected */
  if (AppConnIsOpen() != DM_CONN_ID_NONE)
  {
    switch (btn)
    {
      /* ignore button when connected */
      case APP_UI_BTN_1_SHORT:
        break;

      default:
        break;
    }
  }
  /* button actions when not connected */
  else
  {
    switch (btn)
    {
      case APP_UI_BTN_1_SHORT:
        /* start connectable advertising and reset timer */
        uriBeaconSetAdvType(DM_ADV_CONN_UNDIRECT);
        WsfTimerStartSec(&uriBeaconAdvTimer, URIBEACON_ADV_TIMEOUT_SEC);
        break;

      default:
        break;
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Process messages from the event handler.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void uriBeaconProcMsg(wsfMsgHdr_t *pMsg)
{
  uint8_t uiEvent = APP_UI_NONE;

  switch (pMsg->event)
  {
    /* revert to non-connectable advertising */
    case URIBEACON_ADV_TIMER_IND:
      uriBeaconSetAdvType(DM_ADV_NONCONN_UNDIRECT);
      break;

    case DM_ADV_START_IND:
      if (pMsg->status == HCI_SUCCESS)
      {
        uiEvent = (uriBeaconAdvType == DM_ADV_NONCONN_UNDIRECT) ? APP_UI_DISCOVERABLE : APP_UI_ADV_START;
      }
      break;

    case ATT_MTU_UPDATE_IND:
      APP_TRACE_INFO1("Negotiated MTU %d", ((attEvt_t *)pMsg)->mtu);
      break;

    case DM_RESET_CMPL_IND:
      // set database hash calculating status to true until a new hash is generated after reset
      attsCsfSetHashUpdateStatus(TRUE);

      // Generate ECC key if configured support secure connection,
      // else will calcualte ATT database hash
      if( uriBeaconSecCfg.auth & DM_AUTH_SC_FLAG )
      {
          DmSecGenerateEccKeyReq();
      }
      else
      {
          AttsCalculateDbHash();
      }

      uiEvent = APP_UI_RESET_CMPL;
      break;

    case ATTS_DB_HASH_CALC_CMPL_IND:
      uriBeaconSetup(pMsg);
      break;

    /* stop advertising timeout */
    case DM_CONN_OPEN_IND:
      WsfTimerStop(&uriBeaconAdvTimer);
      uiEvent = APP_UI_CONN_OPEN;
      break;

    /* re-start connectable advertising, with timeout */
    case DM_CONN_CLOSE_IND:
      uriBeaconSetAdvType(DM_ADV_CONN_UNDIRECT);
      uriBeaconAdvTimer.handlerId = uriBeaconHandlerId;
      uriBeaconAdvTimer.msg.event = URIBEACON_ADV_TIMER_IND;
      WsfTimerStartSec(&uriBeaconAdvTimer, URIBEACON_ADV_TIMEOUT_SEC);
      uiEvent = APP_UI_ADV_START;
      break;

    default:
      break;
  }

  if (uiEvent != APP_UI_NONE)
  {
    AppUiAction(uiEvent);
  }

  if (uriBeaconExtCback != NULL)
  {
    uriBeaconExtCback(pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriBeaconHandlerInit(wsfHandlerId_t handlerId)
{
  /* store handler ID */
  uriBeaconHandlerId = handlerId;

  /* set configuration pointers */
  pAppAdvCfg    = (appAdvCfg_t *)&uriBeaconAdvCfg;
  pAppSlaveCfg  = (appSlaveCfg_t *)&uriBeaconSlaveCfg;
  pAppSecCfg    = (appSecCfg_t *)&uriBeaconSecCfg;
  pAppUpdateCfg = (appUpdateCfg_t *)&uriBeaconUpdateCfg;

  /* initialize application framework */
  AppSlaveInit();
  AppServerInit();
}

/*************************************************************************************************/
/*!
 *  \brief  WSF event handler for application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriBeaconHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
  if (pMsg != NULL)
  {
    APP_TRACE_INFO1("UriBeacon got evt %d", pMsg->event);

    /* process ATT messages */
    if (pMsg->event >= ATT_CBACK_START && pMsg->event <= ATT_CBACK_END)
    {
      /* process server-related ATT messages */
      AppServerProcAttMsg(pMsg);
    }
    /* process DM messages */
    else if (pMsg->event >= DM_CBACK_START && pMsg->event <= DM_CBACK_END)
    {
      /* process advertising and connection-related messages */
      AppSlaveProcDmMsg((dmEvt_t *) pMsg);

      /* process security-related messages */
      AppSlaveSecProcDmMsg((dmEvt_t *) pMsg);
    }

    /* perform profile and user interface-related operations */
    uriBeaconProcMsg(pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Start UriBeacon application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriBeaconStart(void)
{
  uint8_t  lockState;
  uint8_t  uriData[URICFG_MAXSIZE_URIDATA_ATT];
  uint8_t  uriDataLen = 0;
  uint8_t  uriDataReset[URICFG_MAXSIZE_URIDATA_ATT];
  uint8_t  uriFlags;
  uint16_t beaconPeriod;
  uint8_t  lock[URICFG_SIZE_LOCK_ATT];

  APP_TRACE_INFO0("URI starting app");

  lockState = URIBEACON_LOCKSTATE_DEFAULT;
  /* try to load settings from NV memory */
  if (WsfNvmReadData(URI_BEACON_PARAM_LOCK_STATE, (uint8_t *)&lockState, sizeof(lockState), NULL) )
  {
    APP_TRACE_INFO1("URI rd lock state from param DB %u", lockState);
  }
  else
  {
    lockState = URIBEACON_LOCKSTATE_DEFAULT;
  }

  if (uriDataOverride != NULL)
  {
    uriDataLen = uriDataOverrideLen;
    memcpy(uriData, uriDataOverride, uriDataLen);
  }
  else if (WsfNvmReadData(URI_BEACON_PARAM_URI_DATA, &uriData[0], sizeof(uriData), NULL))
  {
    APP_TRACE_INFO0("URI rd URI data from param DB ");
    uriBeaconPrintUri(uriData);

    uint8_t *pUriData = uriData;
    while ((*pUriData != 0xFF) && (uriDataLen < URICFG_MAXSIZE_URIDATA_ATT))
    {
      pUriData++;
      uriDataLen++;
    }
  }
  else
  {
    memcpy(uriData, uriBeaconUriData, sizeof(uriBeaconUriData));
    uriDataLen = sizeof(uriBeaconUriData);
  }

  uriFlags = URICFG_ATT_URIFLAGS_DEFAULT;
  if (WsfNvmReadData(URI_BEACON_PARAM_URI_FLAGS, (uint8_t *)&uriFlags, sizeof(uriFlags), NULL))
  {
    APP_TRACE_INFO1("URI rd URI flags from param DB %02X", uriFlags);
  }
  else
  {
    uriFlags = URICFG_ATT_URIFLAGS_DEFAULT;
  }

  if (WsfNvmReadData(URI_BEACON_PARAM_ADVERTISED_TX_POWER_LEVELS, (uint8_t *)&uriBeaconAdvertisedTxPwrLevels, sizeof(uriBeaconAdvertisedTxPwrLevels), NULL))
  {
    APP_TRACE_INFO2("URI rd adv tx pwr lvls from param DB {%d, %d,", uriBeaconAdvertisedTxPwrLevels[0], uriBeaconAdvertisedTxPwrLevels[1]);
    APP_TRACE_INFO2("                                      %d, %d}", uriBeaconAdvertisedTxPwrLevels[2], uriBeaconAdvertisedTxPwrLevels[3]);
  }
  else
  {
    memcpy(uriBeaconAdvertisedTxPwrLevels, uriAdvertisedTxPwrLevelsDefault, sizeof(uriBeaconAdvertisedTxPwrLevels));
  }

  if (WsfNvmReadData(URI_BEACON_PARAM_TX_POWER_MODE, (uint8_t *)&uriBeaconTxPwrMode, sizeof(uriBeaconTxPwrMode), NULL))
  {
    APP_TRACE_INFO1("URI rd tx pwr mode from param DB %u", uriBeaconTxPwrMode);
  }
  else
  {
    uriBeaconTxPwrMode = URICFG_ATT_TXPWRMODE_DEFAULT;
  }

  beaconPeriod = URICFG_ATT_BEACONPERIOD_DEFAULT;

  if (WsfNvmReadData(URI_BEACON_PARAM_BEACON_PERIOD, (uint8_t *)&beaconPeriod, sizeof(beaconPeriod), NULL))
  {
    APP_TRACE_INFO1("URI rd bcn per from param DB %u", beaconPeriod);
  }
  else
  {
    beaconPeriod = URICFG_ATT_BEACONPERIOD_DEFAULT;
  }

  if (WsfNvmReadData(URI_BEACON_PARAM_LOCK, &lock[0], sizeof(lock), NULL))
  {
    APP_TRACE_INFO0("URI rd lock from param DB");
  }
  else
  {
    memset(lock, 0x00u, sizeof(lock));
  }

  /* register for stack callbacks */
  DmRegister(uriBeaconDmCback);
  DmConnRegister(DM_CLIENT_ID_APP, uriBeaconDmCback);
  AttRegister(uriBeaconAttCback);
  AttConnRegister(AppServerConnCback);
  AttsCccRegister(URIBEACON_NUM_CCC_IDX, (attsCccSet_t *)uriBeaconCccSet, uriBeaconCccCback);

  /* set advertising and scan response data for discoverable mode */
  uriBeaconUpdateAdvUriData(uriData, uriDataLen);
  uriBeaconUpdateAdvUriFlags(uriFlags);
  uriBeaconUpdateAdvTxPwrLevel(uriBeaconAdvertisedTxPwrLevels[uriBeaconTxPwrMode]);

  /* register for app framework callbacks */
  AppUiBtnRegister(uriBeaconBtnCback);

  /* initialize attribute server database */
  SvcCoreGattCbackRegister(GattReadCback, GattWriteCback);
  SvcCoreAddGroup();
  SvcDisAddGroup();
  UriCfgStart(uriData, uriDataLen, uriFlags, uriBeaconAdvertisedTxPwrLevels, uriBeaconTxPwrMode, beaconPeriod);
  UriCfgAttWriteCbackRegister(uriBeaconAttWriteCback);
  UriCfgMakeLockable(lockState, lock, uriBeaconLockChangeCback);
  memset(uriDataReset, 0xFFu,            sizeof(uriDataReset));
  memcpy(uriDataReset, uriBeaconUriData, sizeof(uriBeaconUriData));
  UriCfgSetUriDataResetValue(uriDataReset);

  GattSetSvcChangedIdx(URIBEACON_GATT_SC_CCC_IDX);

  /* update beacon period (advertising interval) */
  uriBeaconUpdateBeaconPeriod(beaconPeriod);

  /* reset the device */
  DmDevReset();
}

/*************************************************************************************************/
/*!
 *  \brief  Called prior to starting Uribeacon app to override the beacon data in nv memory.
 *
 *  \param  pUriData    Pointer to URI data.
 *  \param  dataLen     Length of pUriData in bytes.
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriBeaconSetUriOverride(const uint8_t *pUriData, uint8_t dataLen)
{
  uriDataOverride = (uint8_t *) pUriData;
  uriDataOverrideLen = dataLen;
}

/*************************************************************************************************/
/*!
 *  \brief  Register a callback to receive events for the purpose of extending the URI beacon app.
 *
 *  \param  extCback   Callback function
 *
 *  \return None.
 */
/*************************************************************************************************/
void UriBeaconRegisterExtensionCback(uribeaconExtCback_t extCback)
{
  uriBeaconExtCback = extCback;
}
