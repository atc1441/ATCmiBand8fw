/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Proprietary asset tracking locator sample application.
 *
 *  Copyright (c) 2018 - 2019 Arm Ltd. All Rights Reserved.
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
#include "util/bstream.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "wsf_assert.h"
#include "wsf_buf.h"
#include "hci_api.h"
#include "dm_api.h"
#include "dm_priv.h"
#include "gap/gap_api.h"
#include "att_api.h"
#include "smp_api.h"
#include "app_cfg.h"
#include "app_api.h"
#include "app_db.h"
#include "app_ui.h"
#include "svc_core.h"
#include "svc_ch.h"
#include "svc_cte.h"
#include "gatt/gatt_api.h"
#include "atpc/atpc_api.h"
#include "locator/locator_api.h"
#include "util/calc128.h"
#include "app_main.h"


/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Locator app minimum length. */
#define LOCATOR_CTE_MIN_LEN               10

/*! CTE buttion actions. */
#define LOCATOR_CTE_ACTION_ENABLE         0
#define LOCATOR_CTE_ACTION_DISABLE        1

/*! CTE Interval. */
#define LOCATOR_CTE_INTERVAL              25

/*! CTE type. */
#define LOCATOR_CTE_TYPE        HCI_CTE_TYPE_REQ_AOA

#define EXT_SCAN_NAME_MAX_LEN   253

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! application control block */
struct
{
  uint16_t          hdlList[DM_CONN_MAX][APP_DB_HDL_LIST_LEN];   /*! Cached handle list */
  wsfHandlerId_t    handlerId;                      /*! WSF hander ID */
  bool_t            scanning;                       /*! TRUE if scanning */
  bool_t            autoConnect;                    /*! TRUE if auto-connecting */
  uint8_t           discState[DM_CONN_MAX];         /*! Service discovery state */
  uint8_t           hdlListLen;                     /*! Cached handle list length */
  uint8_t           btnConnId;                      /*! The index of the connection ID for button presses */
  uint8_t           cteAction[DM_CONN_MAX];         /*! CTE button action */
} locatorCb;

/*! connection control block */
typedef struct {
  appDbHdl_t          dbHdl;                        /*! Device database record handle type */
  uint8_t             addrType;                     /*! Type of address of device to connect to */
  bdAddr_t            addr;                         /*! Address of device to connect to */
  bool_t              doConnect;                    /*! TRUE to issue connect on scan complete */
  uint8_t             cteState;                     /*! State of CTE config process */
  uint8_t             secPhy;                       /*! Secondary Advertising channel from extended advertising rerport event*/
  uint16_t            eventType;                    /*! event type from extended advertising report event*/

} locatorConnInfo_t;

locatorConnInfo_t locatorConnInfo;

/*! Identifiers for antenna */
static uint8_t locatorAntennaIds[] = {0, 1};

/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/

/*! configurable parameters for master extendec scan */
static const appExtMasterCfg_t locatorMasterExtCfg =
{
  {96},                                      /*! The scan interval, in 0.625 ms units */
  {48},                                      /*! The scan window, in 0.625 ms units  */
  0,                                         /*! The scan duration in ms */
  0,                                         /*! Scan period*/
  DM_DISC_MODE_NONE,                       /*! The GAP discovery mode */
  {DM_SCAN_TYPE_ACTIVE}                      /*! The scan type (active or passive) */
};

/*! configurable parameters for security */
static const appSecCfg_t locatorSecCfg =
{
  DM_AUTH_BOND_FLAG | DM_AUTH_SC_FLAG,    /*! Authentication and bonding flags */
  DM_KEY_DIST_IRK,                        /*! Initiator key distribution flags */
  DM_KEY_DIST_LTK | DM_KEY_DIST_IRK,      /*! Responder key distribution flags */
  FALSE,                                  /*! TRUE if Out-of-band pairing data is present */
  FALSE                                    /*! TRUE to initiate security upon connection */
};

/*! TRUE if Out-of-band pairing data is to be sent */
static const bool_t locatorSendOobData = FALSE;

/*! SMP security parameter configuration */
static const smpCfg_t locatorSmpCfg =
{
  500,                                    /*! 'Repeated attempts' timeout in msec */
  SMP_IO_NO_IN_NO_OUT,                    /*! I/O Capability */
  7,                                      /*! Minimum encryption key length */
  16,                                     /*! Maximum encryption key length */
  1,                                      /*! Attempts to trigger 'repeated attempts' timeout */
  0,                                      /*! Device authentication requirements */
  64000,                                  /*! Maximum repeated attempts timeout in msec */
  64000,                                  /*! Time msec before attemptExp decreases */
  2                                       /*! Repeated attempts multiplier exponent */
};

/*! Connection parameters */
static const hciConnSpec_t locatorConnCfg =
{
  40,                                     /*! Minimum connection interval in 1.25ms units */
  40,                                     /*! Maximum connection interval in 1.25ms units */
  0,                                      /*! Connection latency */
  600,                                    /*! Supervision timeout in 10ms units */
  0,                                      /*! Unused */
  0                                       /*! Unused */
};

/*! Configurable parameters for service and characteristic discovery */
static const appDiscCfg_t locatorDiscCfg =
{
  FALSE,                                  /*! TRUE to wait for a secure connection before initiating discovery */
  TRUE                                    /*! TRUE to fall back on database hash to verify handles when no bond exists. */
};

static const appCfg_t locatorAppCfg =
{
  FALSE,                                  /*! TRUE to abort service discovery if service not found */
  TRUE                                    /*! TRUE to disconnect if ATT transaction times out */
};

/*! ATT configurable parameters (increase MTU) */
static const attCfg_t locatorAttCfg =
{
  15,                               /* ATT server service discovery connection idle timeout in seconds */
  23,                               /* desired ATT MTU */
  ATT_MAX_TRANS_TIMEOUT,            /* transcation timeout in seconds */
  4                                 /* number of queued prepare writes supported by server */
};

/*! local IRK */
static uint8_t localIrk[] =
{
  0xA6, 0xD9, 0xFF, 0x70, 0xD6, 0x1E, 0xF0, 0xA4, 0x46, 0x5F, 0x8D, 0x68, 0x19, 0xF3, 0xB4, 0x96
};

/**************************************************************************************************
  ATT Client Discovery Data
**************************************************************************************************/

/*! Discovery states:  enumeration of services to be discovered */
enum
{
  LOCATOR_DISC_GATT_SVC,      /*! GATT service */
  LOCATOR_DISC_GAP_SVC,       /*! GAP service */
  LOCATOR_DISC_CTE_SVC,       /*! Constant Tone Extension */
  LOCATOR_DISC_SVC_MAX        /*! Discovery complete */
};

/*! the Client handle list, locatorCb.hdlList[], is set as follows:
 *
 *  ------------------------------- <- LOCATOR_DISC_GATT_START
 *  | GATT svc changed handle     |
 *  -------------------------------
 *  | GATT svc changed ccc handle |
 *  ------------------------------- <- LOCATOR_DISC_GAP_START
 *  | GAP central addr res handle |
 *  -------------------------------
 *  | GAP RPA Only handle         |
 *  ------------------------------- <- LOCATOR_DISC_CTE_START
 *  | CTE handles                 |
 *  | ...                         |
 *  -------------------------------
 */

/*! Start of each service's handles in the the handle list */
#define LOCATOR_DISC_GATT_START       0
#define LOCATOR_DISC_GAP_START        (LOCATOR_DISC_GATT_START + GATT_HDL_LIST_LEN)
#define LOCATOR_DISC_CTE_START        (LOCATOR_DISC_GAP_START + GAP_HDL_LIST_LEN)
#define LOCATOR_DISC_HDL_LIST_LEN     (LOCATOR_DISC_CTE_START + ATPC_CTE_HDL_LIST_LEN)

/*! Pointers into handle list for each service's handles */
static uint16_t *pLocatorGattHdlList[DM_CONN_MAX];
static uint16_t *pLocatorGapHdlList[DM_CONN_MAX];
static uint16_t *pLocatorCteHdlList[DM_CONN_MAX];

/*! LESC OOB configuration */
static dmSecLescOobCfg_t *locatorOobCfg;

/**************************************************************************************************
  ATT Client Configuration Data
**************************************************************************************************/

/*! Default value for CCC indications */
const uint8_t locatorCccIndVal[2] = {UINT16_TO_BYTES(ATT_CLIENT_CFG_INDICATE)};

/*! Default value for CCC notifications */
const uint8_t locatorCccNtfVal[2] = {UINT16_TO_BYTES(ATT_CLIENT_CFG_NOTIFY)};

/*! Default value for Client Supported Features (enable Robust Caching) */
const uint8_t locatorCsfVal[1] = {ATTS_CSF_ROBUST_CACHING};

/*! List of characteristics to configure after service discovery */
static const attcDiscCfg_t locatorDiscCfgList[] =
{
  /* Write:  GATT service changed ccc descriptor */
  {locatorCccIndVal, sizeof(locatorCccIndVal), (GATT_SC_CCC_HDL_IDX + LOCATOR_DISC_GATT_START)},

  /* Write: GATT clinet supported features */
  {locatorCsfVal, sizeof(locatorCsfVal), (GATT_CSF_HDL_IDX + LOCATOR_DISC_GATT_START)}
};

/*! Characteristic configuration list length */
#define LOCATOR_DISC_CFG_LIST_LEN   (sizeof(locatorDiscCfgList) / sizeof(attcDiscCfg_t))

/*! sanity check:  make sure configuration list length is <= handle list length */
WSF_CT_ASSERT(LOCATOR_DISC_CFG_LIST_LEN <= LOCATOR_DISC_HDL_LIST_LEN);

/*************************************************************************************************/
/*!
 *  \brief  Application DM callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void locatorDmCback(dmEvt_t *pDmEvt)
{
  dmEvt_t   *pMsg;
  uint16_t  len;
  uint16_t  reportLen;

  if (pDmEvt->hdr.event == DM_SEC_ECC_KEY_IND)
  {
    DmSecSetEccKey(&pDmEvt->eccMsg.data.key);

    /* If the local device sends OOB data. */
    if (locatorSendOobData)
    {
      uint8_t oobLocalRandom[SMP_RAND_LEN];
      SecRand(oobLocalRandom, SMP_RAND_LEN);
      DmSecCalcOobReq(oobLocalRandom, pDmEvt->eccMsg.data.key.pubKey_x);
    }
  }
  else if (pDmEvt->hdr.event == DM_SEC_CALC_OOB_IND)
  {
    if (locatorOobCfg == NULL)
    {
      locatorOobCfg = WsfBufAlloc(sizeof(dmSecLescOobCfg_t));
    }

    if (locatorOobCfg)
    {
      Calc128Cpy(locatorOobCfg->localConfirm, pDmEvt->oobCalcInd.confirm);
      Calc128Cpy(locatorOobCfg->localRandom, pDmEvt->oobCalcInd.random);
    }
  }
  else
  {
    len = DmSizeOfEvt(pDmEvt);

    if (pDmEvt->hdr.event == DM_EXT_SCAN_REPORT_IND)
    {
      reportLen = pDmEvt->extScanReport.len;
    }
    else if (pDmEvt->hdr.event == DM_PER_ADV_REPORT_IND)
    {
      reportLen = pDmEvt->perAdvReport.len;
    }
    else if (pDmEvt->hdr.event == DM_CONN_IQ_REPORT_IND)
    {
      reportLen = pDmEvt->connIQReport.sampleCnt * 2;
    }
    else if (pDmEvt->hdr.event == DM_CONNLESS_IQ_REPORT_IND)
    {
        reportLen = pDmEvt->connlessIQReport.sampleCnt * 2;
    }
    else
    {
      reportLen = 0;
    }

    if ((pMsg = WsfMsgAlloc(len + reportLen)) != NULL)
    {
      memcpy(pMsg, pDmEvt, len);

      if (pDmEvt->hdr.event == DM_EXT_SCAN_REPORT_IND)
      {
        pMsg->extScanReport.pData = (uint8_t *)((uint8_t *)pMsg + len);
        memcpy(pMsg->extScanReport.pData, pDmEvt->extScanReport.pData, reportLen);
      }
      else if (pDmEvt->hdr.event == DM_PER_ADV_REPORT_IND)
      {
        pMsg->perAdvReport.pData = (uint8_t *)((uint8_t *)pMsg + len);
        memcpy(pMsg->perAdvReport.pData, pDmEvt->perAdvReport.pData, reportLen);
      }
      else if (pDmEvt->hdr.event == DM_CONN_IQ_REPORT_IND)
      {
        /* Copy I samples to space after end of report struct */
        pMsg->connIQReport.pISample = (int8_t *)((uint8_t *) pMsg + len);
        memcpy(pMsg->connIQReport.pISample, pDmEvt->connIQReport.pISample, pDmEvt->connIQReport.sampleCnt);

        /* Copy Q samples to space after I samples space */
        pMsg->connIQReport.pQSample = (int8_t *)((uint8_t *) pMsg->connIQReport.pISample + pDmEvt->connIQReport.sampleCnt);
        memcpy(pMsg->connIQReport.pQSample, pDmEvt->connIQReport.pQSample, pDmEvt->connIQReport.sampleCnt);
      }
      else if (pDmEvt->hdr.event == DM_CONNLESS_IQ_REPORT_IND)
      {
        /* Copy I samples to space after end of report struct */
        pMsg->connlessIQReport.pISample = (int8_t *)((uint8_t *) pMsg + len);
        memcpy(pMsg->connlessIQReport.pISample, pDmEvt->connlessIQReport.pISample, pDmEvt->connlessIQReport.sampleCnt);

        /* Copy Q samples to space after I samples space */
        pMsg->connlessIQReport.pQSample = (int8_t *)((uint8_t *) pMsg->connlessIQReport.pISample + pDmEvt->connlessIQReport.sampleCnt);
        memcpy(pMsg->connlessIQReport.pQSample, pDmEvt->connlessIQReport.pQSample, pDmEvt->connlessIQReport.sampleCnt);
      }

      WsfMsgSend(locatorCb.handlerId, pMsg);
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Application  ATT callback.
 *
 *  \param  pEvt    ATT callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void locatorAttCback(attEvt_t *pEvt)
{
  attEvt_t *pMsg;

  if ((pMsg = WsfMsgAlloc(sizeof(attEvt_t) + pEvt->valueLen)) != NULL)
  {
    memcpy(pMsg, pEvt, sizeof(attEvt_t));
    pMsg->pValue = (uint8_t *) (pMsg + 1);
    memcpy(pMsg->pValue, pEvt->pValue, pEvt->valueLen);
    WsfMsgSend(locatorCb.handlerId, pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Perform actions on scan start.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void locatorScanStart(dmEvt_t *pMsg)
{
  if (pMsg->hdr.status == HCI_SUCCESS)
  {
    locatorCb.scanning = TRUE;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Perform actions on scan stop.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void locatorScanStop(dmEvt_t *pMsg)
{
  if (pMsg->hdr.status == HCI_SUCCESS)
  {
    locatorCb.scanning = FALSE;
    locatorCb.autoConnect = FALSE;

    /* Open connection */
    if (locatorConnInfo.doConnect)
    {
        if((locatorConnInfo.eventType & HCI_ADV_RPT_LEG_ADV_BIT) == 0)
        {
            APP_TRACE_INFO1("aux connect ,secphy=%d", locatorConnInfo.secPhy);
            AppExtConnOpen(HCI_INIT_PHY_LE_1M_BIT|locatorConnInfo.secPhy, locatorConnInfo.addrType, locatorConnInfo.addr, locatorConnInfo.dbHdl);
        }
        else
        {
            APP_TRACE_INFO0("legacy connect");
            AppExtConnOpen(HCI_INIT_PHY_LE_1M_BIT, locatorConnInfo.addrType, locatorConnInfo.addr, locatorConnInfo.dbHdl);
        }

        locatorConnInfo.doConnect = FALSE;
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Handle a extended scan report.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void locatorExtScanReport(dmEvt_t *pMsg)
{
  uint8_t    *pData;
  appDbHdl_t dbHdl;
  bool_t     connect = FALSE;
  uint16_t syncTimeout = 0x200;
  char tmp_name[EXT_SCAN_NAME_MAX_LEN+1] = {0};

  /* disregard if not scanning or autoconnecting */
    if (!locatorCb.scanning || !locatorCb.autoConnect)
  {
    return;
  }

  /* if we already have a bond with this device then connect to it */
  if ((dbHdl = AppDbFindByAddr(pMsg->extScanReport.addrType, pMsg->extScanReport.addr)) != APP_DB_HDL_NONE)
  {
    /* if this is a directed advertisement where the initiator address is an RPA */
    if (DM_RAND_ADDR_RPA(pMsg->extScanReport.directAddr, pMsg->extScanReport.directAddrType))
    {
      /* resolve direct address to see if it's addressed to us */
      AppMasterResolveAddr(pMsg, dbHdl, APP_RESOLVE_DIRECT_RPA);
    }
    else
    {
      connect = TRUE;
    }
  }
  /* if the peer device uses an RPA */
  else if (DM_RAND_ADDR_RPA(pMsg->extScanReport.addr, pMsg->extScanReport.addrType))
  {
    /* resolve advertiser's RPA to see if we already have a bond with this device */
    AppMasterResolveAddr(pMsg, APP_DB_HDL_NONE, APP_RESOLVE_ADV_RPA);
  }
  /* find vendor-specific advertising data */
  else if ((pData = DmFindAdType(DM_ADV_TYPE_MANUFACTURER, pMsg->extScanReport.len,
                                 pMsg->extScanReport.pData)) != NULL)
  {
    /* check length and vendor ID */
    if (pData[DM_AD_LEN_IDX] >= 3 && BYTES_UINT16_CMP(&pData[DM_AD_DATA_IDX], HCI_ID_PACKETCRAFT))
    {
      //connect = TRUE;
    }
  }
  /* find Local name advertising data */
  else if ((pData = DmFindAdType(DM_ADV_TYPE_LOCAL_NAME, pMsg->extScanReport.len,
                                 pMsg->extScanReport.pData)) != NULL)
  {
    memcpy(tmp_name, (char *)(pData+2),*pData-1);

    APP_TRACE_INFO1("device name:%s", tmp_name);

    /* check length and vendor ID */
    if (!strncmp((char *)(pData+2), "Ambq_Per", *pData-1))
    {
      uint8_t advSid = pMsg->extScanReport.advSid;
      uint8_t advAddrType = pMsg->extScanReport.addrType;
      bdAddr_t advAddr;

      memcpy(advAddr, pMsg->extScanReport.addr, BDA_ADDR_LEN);

      APP_TRACE_INFO0("found periodic adv device");
      AppSyncStart(advSid, advAddrType, advAddr, 0, syncTimeout);
    }
    /* check length and vendor ID */
    else if (!strncmp((char *)(pData+2), "Asset Tag", *pData-1))
    {
      APP_TRACE_INFO0("found Asset Tag device");
      connect = TRUE;
    }
  }

  if (connect)
  {
    /* stop scanning and connect */
    locatorCb.autoConnect = FALSE;
    AppExtScanStop();

    /* Store peer information for connect on scan stop */
    locatorConnInfo.addrType = DmHostAddrType(pMsg->extScanReport.addrType);
    memcpy(locatorConnInfo.addr, pMsg->extScanReport.addr, sizeof(bdAddr_t));
    locatorConnInfo.dbHdl = dbHdl;
    locatorConnInfo.doConnect = TRUE;
    locatorConnInfo.secPhy = pMsg->extScanReport.secPhy;
    locatorConnInfo.eventType = pMsg->extScanReport.eventType;

  }
}

/*************************************************************************************************/
/*!
 *  \brief  Perform UI actions on connection open.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void locatorOpen(dmEvt_t *pMsg)
{
  /* Set the antenna identifiers for the connection */
  AtpcSetAntennaIds((dmConnId_t) pMsg->hdr.param, sizeof(locatorAntennaIds), locatorAntennaIds);

  /* Reset control information. */
  locatorCb.cteAction[pMsg->hdr.param-1] = LOCATOR_CTE_ACTION_ENABLE;
}

/*************************************************************************************************/
/*!
 *  \brief  Process a received ATT notification.
 *
 *  \param  pMsg    Pointer to ATT callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void locatorValueNtf(attEvt_t *pMsg)
{
}

/*************************************************************************************************/
/*!
 *  \brief  Process IQ report from the event handler.
 *
 *  \param  pIqRpt  Pointer to IQ Report message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void locatorProcIqRpt(hciLeConnIQReportEvt_t *pIqRpt)
{
  APP_TRACE_INFO0("IQ Report");
  APP_TRACE_INFO1("  status: %x", pIqRpt->hdr.status);
  APP_TRACE_INFO1("  sampleCnt: %d", pIqRpt->sampleCnt);
}


/*************************************************************************************************/
/*!
 *  \brief  Process connectionless IQ report from the event handler.
 *
 *  \param  pIqRpt  Pointer to IQ Report message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void locatorProcConlessIqRpt(hciLeConlessIQReportEvt_t *pIqRpt)
{
  APP_TRACE_INFO0("connectionless IQ Report");
  APP_TRACE_INFO1("  status: %x", pIqRpt->hdr.status);
  APP_TRACE_INFO1("  sampleCnt: %d", pIqRpt->sampleCnt);
}

/*************************************************************************************************/
/*!
 *  \brief  Set up procedures that need to be performed after device reset.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void locatorSetup(dmEvt_t *pMsg)
{
  locatorCb.scanning = FALSE;
  locatorCb.autoConnect = FALSE;
  locatorConnInfo.doConnect = FALSE;

  DmConnSetConnSpec((hciConnSpec_t *) &locatorConnCfg);
}

/*************************************************************************************************/
/*!
 *  \brief  GAP service discovery has completed.
 *
 *  \param  connId    Connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void locatorDiscGapCmpl(dmConnId_t connId)
{
  appDbHdl_t dbHdl;

  /* if RPA Only attribute found on peer device */
  if ((pLocatorGapHdlList[connId-1][GAP_RPAO_HDL_IDX] != ATT_HANDLE_NONE) &&
      ((dbHdl = AppDbGetHdl(connId)) != APP_DB_HDL_NONE))
  {
    /* update DB */
    AppDbSetPeerRpao(dbHdl, TRUE);
  }
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
static void locatorBtnCback(uint8_t btn)
{
  dmConnId_t  connId = locatorCb.btnConnId;
  dmConnId_t  connIdList[DM_CONN_MAX];
  uint8_t     numConnections = AppConnOpenList(connIdList);

  /* button actions when connected */
  if (numConnections > 0)
  {
    switch (btn)
    {
      case APP_UI_BTN_1_SHORT:
        if (numConnections < DM_CONN_MAX - 1)
        {
          /* if scanning cancel scanning */
          if (locatorCb.scanning)
          {
            AppExtScanStop();
          }
          /* else auto connect */
          else if (!locatorCb.autoConnect)
          {
            locatorCb.autoConnect = TRUE;
            locatorConnInfo.doConnect = FALSE;
            AppExtScanStart(HCI_SCAN_PHY_LE_1M_BIT, locatorMasterExtCfg.discMode,
                locatorMasterExtCfg.scanType, locatorMasterExtCfg.scanDuration, locatorMasterExtCfg.scanPeriod);
          }
        }
        else
        {
          APP_TRACE_INFO0("locatorBtnCback: Max connections reached.");
        }
        break;

      case APP_UI_BTN_1_MED:
        /* Increment which connection ID is used in button presses */
        if (++locatorCb.btnConnId > DM_CONN_MAX)
        {
          locatorCb.btnConnId = 1;
        }
        APP_TRACE_INFO1("ConnId for Button Press: %d", locatorCb.btnConnId);
        break;

      case APP_UI_BTN_1_LONG:
        /* disconnect */
        AppConnClose(connId);
        break;

      case APP_UI_BTN_1_EX_LONG:
        if (locatorCb.cteAction[connId-1] == LOCATOR_CTE_ACTION_ENABLE)
        {
          /* Toggle action. */
          locatorCb.cteAction[connId-1] = LOCATOR_CTE_ACTION_DISABLE;

          /* Perform AoA enable request. */
          AtpcCteAclEnableReq(connId, pLocatorCteHdlList[connId-1][ATPC_CTE_ENABLE_HDL],
                              LOCATOR_CTE_MIN_LEN, LOCATOR_CTE_INTERVAL, LOCATOR_CTE_TYPE);
        }
        else
        {
          /* Toggle action. */
          locatorCb.cteAction[connId-1] = LOCATOR_CTE_ACTION_ENABLE;

          /* Perform AoA disable request. */
          //AtpcCteAclDisableReq(connId, pLocatorCteHdlList[connId-1][ATPC_CTE_ENABLE_HDL]);
          DmConnCteReqStop(connId);
        }
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
        /* if scanning cancel scanning */
        if (locatorCb.scanning)
        {
          AppExtScanStop();
        }
        /* else auto connect */
        else if (!locatorCb.autoConnect)
        {
          locatorCb.autoConnect = TRUE;
          locatorConnInfo.doConnect = FALSE;
          DmSyncInitialRptEnable(TRUE);
          AppExtScanStart(HCI_SCAN_PHY_LE_1M_BIT, locatorMasterExtCfg.discMode,
                locatorMasterExtCfg.scanType, locatorMasterExtCfg.scanDuration, locatorMasterExtCfg.scanPeriod);
        }
        break;

      case APP_UI_BTN_1_MED:
        /* Increment which connection ID is used in button presses */
        if (++locatorCb.btnConnId > DM_CONN_MAX)
        {
          locatorCb.btnConnId = 1;
        }
        APP_TRACE_INFO1("ConnID for Button Press: %d", locatorCb.btnConnId);
        break;

      case APP_UI_BTN_1_LONG:
        /* clear all bonding info */
        AppClearAllBondingInfo();
        break;

      case APP_UI_BTN_1_EX_LONG:
        /* add RPAO characteristic to GAP service -- needed only when DM Privacy enabled */
        SvcCoreGapAddRpaoCh();
        break;

      case APP_UI_BTN_2_EX_LONG:
        /* enable device privacy -- start generating local RPAs every 15 minutes */
        DmDevPrivStart(15 * 60);

        /* set Scanning filter policy to accept directed advertisements with RPAs */
        DmDevSetFilterPolicy(DM_FILT_POLICY_MODE_SCAN, HCI_FILT_RES_INIT);
        break;

      default:
        break;
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Discovery callback.
 *
 *  \param  connId    Connection identifier.
 *  \param  status    Service or configuration status.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void locatorDiscCback(dmConnId_t connId, uint8_t status)
{
  switch(status)
  {
    case APP_DISC_INIT:
      /* set handle list when initialization requested */
      AppDiscSetHdlList(connId, locatorCb.hdlListLen, locatorCb.hdlList[connId-1]);
      break;

    case APP_DISC_READ_DATABASE_HASH:
      /* read peer's database hash */
      AppDiscReadDatabaseHash(connId);
      break;

    case APP_DISC_SEC_REQUIRED:
      /* initiate security */
      AppMasterSecurityReq(connId);
      break;

    case APP_DISC_START:
      /* initialize discovery state */
      locatorCb.discState[connId-1] = LOCATOR_DISC_GATT_SVC;

      /* discover GATT service */
      GattDiscover(connId, pLocatorGattHdlList[connId-1]);
      break;

    case APP_DISC_FAILED:
      if (pAppCfg->abortDisc)
      {
        /* if discovery failed for proprietary data service then disconnect */
        if (locatorCb.discState[connId-1] == LOCATOR_DISC_CTE_SVC)
        {
          AppConnClose(connId);
          break;
        }
      }
      /* Else falls through. */

    case APP_DISC_CMPL:
      /* next discovery state */
      locatorCb.discState[connId-1]++;

      if (locatorCb.discState[connId-1] == LOCATOR_DISC_GAP_SVC)
      {
        /* discover GAP service */
        GapDiscover(connId, pLocatorGapHdlList[connId-1]);
      }
      else if (locatorCb.discState[connId-1] == LOCATOR_DISC_CTE_SVC)
      {
        /* discover proprietary data service */
        AtpcCteDiscover(connId, pLocatorCteHdlList[connId-1]);
      }
      else
      {
        /* discovery complete */
        AppDiscComplete(connId, APP_DISC_CMPL);

        /* GAP service discovery completed */
        locatorDiscGapCmpl(connId);

        /* start configuration */
        AppDiscConfigure(connId, APP_DISC_CFG_START, LOCATOR_DISC_CFG_LIST_LEN,
                         (attcDiscCfg_t *) locatorDiscCfgList, LOCATOR_DISC_HDL_LIST_LEN, locatorCb.hdlList[connId-1]);
      }
      break;

    case APP_DISC_CFG_START:
        /* start configuration */
        AppDiscConfigure(connId, APP_DISC_CFG_START, LOCATOR_DISC_CFG_LIST_LEN,
                         (attcDiscCfg_t *) locatorDiscCfgList, LOCATOR_DISC_HDL_LIST_LEN, locatorCb.hdlList[connId-1]);
      break;

    case APP_DISC_CFG_CMPL:
      AppDiscComplete(connId, status);
      break;

    case APP_DISC_CFG_CONN_START:
      /* no connection setup configuration */
      break;

    default:
      break;
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
static void locatorProcMsg(dmEvt_t *pMsg)
{
  uint8_t uiEvent = APP_UI_NONE;

  switch(pMsg->hdr.event)
  {
    case ATTC_HANDLE_VALUE_NTF:
      locatorValueNtf((attEvt_t *) pMsg);
      break;

    case DM_RESET_CMPL_IND:
      AttsCalculateDbHash();
      DmSecGenerateEccKeyReq();
      locatorSetup(pMsg);
      uiEvent = APP_UI_RESET_CMPL;
      break;

    case DM_EXT_SCAN_START_IND:
      locatorScanStart(pMsg);
      uiEvent = APP_UI_SCAN_START;
      break;

    case DM_EXT_SCAN_STOP_IND:
      locatorScanStop(pMsg);
      uiEvent = APP_UI_SCAN_STOP;
      break;

    case DM_EXT_SCAN_REPORT_IND:
      locatorExtScanReport(pMsg);
      break;

      case DM_PER_ADV_REPORT_IND:
      {
        uint16_t syncHandle = pMsg->perAdvReport.syncHandle;
        uint8_t slotDurations = 0x01;
        uint8_t maxSampleCte = 0x0;
        uint8_t switchPatternLen = 0x2;
        uint8_t pAntennaIDs[HCI_MIN_NUM_ANTENNA_IDS] = {0,1};

        HciLeSetConlessIQSampleEnCmd(syncHandle, TRUE, slotDurations,
                                maxSampleCte,switchPatternLen, pAntennaIDs);

        break;
      }

      case DM_CONNLESS_IQ_REPORT_IND:
      {
        locatorProcConlessIqRpt(&pMsg->connlessIQReport);

        break;
      }

    case DM_CONN_OPEN_IND:
      locatorOpen(pMsg);
      uiEvent = APP_UI_CONN_OPEN;
      break;

    case DM_CONN_CLOSE_IND:
      uiEvent = APP_UI_CONN_CLOSE;
      break;

    case DM_PHY_UPDATE_IND:
      APP_TRACE_INFO3("DM_PHY_UPDATE_IND status: %d, RX: %d, TX: %d", pMsg->phyUpdate.status,pMsg->phyUpdate.rxPhy, pMsg->phyUpdate.txPhy);
      break;

    case DM_SEC_PAIR_CMPL_IND:
      DmSecGenerateEccKeyReq();
      uiEvent = APP_UI_SEC_PAIR_CMPL;
      break;

    case DM_SEC_PAIR_FAIL_IND:
      DmSecGenerateEccKeyReq();
      uiEvent = APP_UI_SEC_PAIR_FAIL;
      break;

    case DM_SEC_ENCRYPT_IND:
      uiEvent = APP_UI_SEC_ENCRYPT;
      break;

    case DM_SEC_ENCRYPT_FAIL_IND:
      uiEvent = APP_UI_SEC_ENCRYPT_FAIL;
      break;

    case DM_SEC_AUTH_REQ_IND:

      if (pMsg->authReq.oob)
      {
        dmConnId_t connId = (dmConnId_t) pMsg->hdr.param;




        if (locatorOobCfg != NULL)
        {
          DmSecSetOob(connId, locatorOobCfg);
        }

        DmSecAuthRsp(connId, 0, NULL);
      }
      else
      {
        AppHandlePasskey(&pMsg->authReq);
      }
      break;

    case DM_SEC_COMPARE_IND:
      AppHandleNumericComparison(&pMsg->cnfInd);
      break;

    case DM_ADV_NEW_ADDR_IND:
      break;

    case DM_PRIV_CLEAR_RES_LIST_IND:
      APP_TRACE_INFO1("Clear resolving list status 0x%02x", pMsg->hdr.status);
      break;

    case DM_CONN_IQ_REPORT_IND:
      locatorProcIqRpt(&pMsg->connIQReport);
      break;

    case DM_CTE_REQ_FAIL_IND:
      APP_TRACE_INFO1("DM_CTE_REQ_FAIL_IND: status: %#x", pMsg->hdr.status);
      break;

    default:
      break;
  }

  if (uiEvent != APP_UI_NONE)
  {
    AppUiAction(uiEvent);
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
void LocatorHandlerInit(wsfHandlerId_t handlerId)
{
  APP_TRACE_INFO0("LocatorHandlerInit");

  /* store handler ID */
  locatorCb.handlerId = handlerId;

  /* set handle list length */
  locatorCb.hdlListLen = LOCATOR_DISC_HDL_LIST_LEN;

  locatorCb.btnConnId = 1;
  /* Set configuration pointers */
  pAppExtMasterCfg = (appExtMasterCfg_t *)&locatorMasterExtCfg;
  pAppSecCfg = (appSecCfg_t *) &locatorSecCfg;
  pAppDiscCfg = (appDiscCfg_t *) &locatorDiscCfg;
  pAppCfg = (appCfg_t *)&locatorAppCfg;
  pSmpCfg = (smpCfg_t *) &locatorSmpCfg;
  pAttCfg = (attCfg_t *) &locatorAttCfg;

  /* Initialize application framework */
  AppMasterInit();
  AppDiscInit();

  /* Set IRK for the local device */
  DmSecSetLocalIrk(localIrk);
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
void LocatorHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
  if (pMsg != NULL)
  {
    APP_TRACE_INFO1("Locator got evt %d", pMsg->event);

    /* process ATT messages */
    if (pMsg->event <= ATT_CBACK_END)
    {
      /* process discovery-related ATT messages */
      AppDiscProcAttMsg((attEvt_t *) pMsg);

      /* process server-related ATT messages */
      AppServerProcAttMsg(pMsg);
    }
    /* process DM messages */
    else if (pMsg->event <= DM_CBACK_END)
    {
      /* process advertising and connection-related messages */
      AppMasterProcDmMsg((dmEvt_t *) pMsg);

      /* process security-related messages */
      AppMasterSecProcDmMsg((dmEvt_t *) pMsg);

      /* process discovery-related messages */
      AppDiscProcDmMsg((dmEvt_t *) pMsg);
    }

    /* process asset tracking profile related messages */
    AtpcProcMsg(pMsg);

    /* perform profile and user interface-related operations */
    locatorProcMsg((dmEvt_t *) pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize the pointers into the handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void locatorInitSvcHdlList()
{
  uint8_t i;

  for (i = 0; i < DM_CONN_MAX; i++)
  {
    /*! Pointers into handle list for each service's handles */
    pLocatorGattHdlList[i] = &locatorCb.hdlList[i][LOCATOR_DISC_GATT_START];
    pLocatorGapHdlList[i] = &locatorCb.hdlList[i][LOCATOR_DISC_GAP_START];
    pLocatorCteHdlList[i] = &locatorCb.hdlList[i][LOCATOR_DISC_CTE_START];
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void LocatorStart(void)
{
  /* Initialize handle pointers */
  locatorInitSvcHdlList();

  /* Register for stack callbacks */
  DmRegister(locatorDmCback);
  DmConnRegister(DM_CLIENT_ID_APP, locatorDmCback);
  AttRegister(locatorAttCback);

  /* Register for app framework button callbacks */
  AppUiBtnRegister(locatorBtnCback);

  /* Register for app framework discovery callbacks */
  AppDiscRegister(locatorDiscCback);

  /* Initialize attribute server database */
  SvcCoreAddGroup();

  /* Reset the device */
  DmDevReset();
}
