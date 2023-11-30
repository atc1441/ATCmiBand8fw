/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Data transmitter sample application.
 *
 *  Copyright (c) 2012-2019 Arm Ltd.
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
#include "wsf_buf.h"
#include "hci_api.h"
#include "sec_api.h"
#include "dm_api.h"
#include "smp_api.h"
#include "att_api.h"
#include "app_api.h"
#include "app_main.h"
#include "app_db.h"
#include "app_ui.h"
#include "svc_ch.h"
#include "svc_core.h"
#include "svc_wp.h"
#include "util/calc128.h"
#include "gatt/gatt_api.h"
#include "dats/dats_api.h"
#if WDXS_INCLUDED  == TRUE
#include "wsf_efs.h"
#include "svc_wdxs.h"
#include "wdxs/wdxs_api.h"
#include "wdxs/wdxs_main.h"
#include "wdxs/wdxs_stream.h"
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/
#if CS50_INCLUDED == TRUE

/* PHY Test Modes */
#define DATS_PHY_1M                       1
#define DATS_PHY_2M                       2
#define DATS_PHY_CODED                    3

#endif /* CS50_INCLUDED */

/*! Enumeration of client characteristic configuration descriptors */
enum
{
#if WDXS_INCLUDED == TRUE
  WDXS_DC_CH_CCC_IDX,             /*! WDXS DC service, service changed characteristic */
  WDXS_FTC_CH_CCC_IDX,            /*! WDXS FTC  service, service changed characteristic */
  WDXS_FTD_CH_CCC_IDX,            /*! WDXS FTD service, service changed characteristic */
  WDXS_AU_CH_CCC_IDX,             /*! WDXS AU service, service changed characteristic */
#endif /* WDXS_INCLUDED */
  DATS_GATT_SC_CCC_IDX,           /*! GATT service, service changed characteristic */
  DATS_WP_DAT_CCC_IDX,            /*! Arm Ltd. proprietary service, data transfer characteristic */
  DATS_NUM_CCC_IDX
};

/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/

/*! configurable parameters for advertising */
static const appAdvCfg_t datsAdvCfg =
{
  {30000,     0,     0},                  /*! Advertising durations in ms */
  {   96,  1600,     0}                   /*! Advertising intervals in 0.625 ms units */
};

/*! configurable parameters for slave */
static const appSlaveCfg_t datsSlaveCfg =
{
  1,                                      /*! Maximum connections */
};

/*! configurable parameters for security */
static const appSecCfg_t datsSecCfg =
{
  DM_AUTH_BOND_FLAG | DM_AUTH_SC_FLAG,    /*! Authentication and bonding flags */
  DM_KEY_DIST_IRK,                        /*! Initiator key distribution flags */
  DM_KEY_DIST_LTK | DM_KEY_DIST_IRK,      /*! Responder key distribution flags */
  FALSE,                                  /*! TRUE if Out-of-band pairing data is present */
  TRUE                                    /*! TRUE to initiate security upon connection */
};

/*! TRUE if Out-of-band pairing data is to be sent */
static const bool_t datsSendOobData = FALSE;

/*! SMP security parameter configuration */
static const smpCfg_t datsSmpCfg =
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

/*! configurable parameters for connection parameter update */
static const appUpdateCfg_t datsUpdateCfg =
{
  0,                                      /*! Connection idle period in ms before attempting
                                              connection parameter update; set to zero to disable */
  640,                                    /*! Minimum connection interval in 1.25ms units */
  800,                                    /*! Maximum connection interval in 1.25ms units */
  3,                                      /*! Connection latency */
  600,                                    /*! Supervision timeout in 10ms units */
  5                                       /*! Number of update attempts before giving up */
};

/*! ATT configurable parameters (increase MTU) */
static const attCfg_t datsAttCfg =
{
  15,                               /* ATT server service discovery connection idle timeout in seconds */
  241,                              /* desired ATT MTU */
  ATT_MAX_TRANS_TIMEOUT,            /* transcation timeout in seconds */
  4                                 /* number of queued prepare writes supported by server */
};

/*! local IRK */
static uint8_t localIrk[] =
{
  0x95, 0xC8, 0xEE, 0x6F, 0xC5, 0x0D, 0xEF, 0x93, 0x35, 0x4E, 0x7C, 0x57, 0x08, 0xE2, 0xA3, 0x85
};

/**************************************************************************************************
  Advertising Data
**************************************************************************************************/

/*! advertising data, discoverable mode */
static const uint8_t datsAdvDataDisc[] =
{
  /*! flags */
  2,                                      /*! length */
  DM_ADV_TYPE_FLAGS,                      /*! AD type */
  DM_FLAG_LE_GENERAL_DISC |               /*! flags */
  DM_FLAG_LE_BREDR_NOT_SUP,

  /*! manufacturer specific data */
  3,                                      /*! length */
  DM_ADV_TYPE_MANUFACTURER,               /*! AD type */
  UINT16_TO_BYTES(HCI_ID_PACKETCRAFT)     /*! company ID */
};

/*! scan data, discoverable mode */
static const uint8_t datsScanDataDisc[] =
{
  /*! device name */
  8,                                      /*! length */
  DM_ADV_TYPE_LOCAL_NAME,                 /*! AD type */
  'D',
  'a',
  't',
  'a',
  ' ',
  'T',
  'X'
};

/**************************************************************************************************
  Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t datsCccSet[DATS_NUM_CCC_IDX] =
{
  /* cccd handle          value range               security level */
#if WDXS_INCLUDED == TRUE
  {WDXS_DC_CH_CCC_HDL,    ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},   /* WDXS_DC_CH_CCC_IDX */
  {WDXS_FTC_CH_CCC_HDL,   ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},   /* WDXS_FTC_CH_CCC_IDX */
  {WDXS_FTD_CH_CCC_HDL,   ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},   /* WDXS_FTD_CH_CCC_IDX */
  {WDXS_AU_CH_CCC_HDL,    ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},   /* WDXS_AU_CH_CCC_IDX */
#endif /* WDXS_INCLUDED */
  {GATT_SC_CH_CCC_HDL,    ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_NONE},   /* DATS_GATT_SC_CCC_IDX */
  {WP_DAT_CH_CCC_HDL,     ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE}    /* DATS_WP_DAT_CCC_IDX */
};

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! application control block */
static struct
{
  wsfHandlerId_t    handlerId;        /* WSF handler ID */
#if CS50_INCLUDED == TRUE
  uint8_t           phyMode;          /*! PHY Test Mode */
#endif /* CS50_INCLUDED */
} datsCb;

/* LESC OOB configuration */
static dmSecLescOobCfg_t *datsOobCfg;

/*************************************************************************************************/
/*!
 *  \brief  Send notification containing data.
 *
 *  \param  connId      DM connection ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void datsSendData(dmConnId_t connId)
{
  uint8_t str[] = "hello hello hello hello hello hello.....from dats";

  if (AttsCccEnabled(connId, DATS_WP_DAT_CCC_IDX))
  {
    /* send notification */
    AttsHandleValueNtf(connId, WP_DAT_HDL, sizeof(str), str);
    AttsHandleValueNtf(connId, WP_DAT_HDL, sizeof(str), str);
    AttsHandleValueNtf(connId, WP_DAT_HDL, sizeof(str), str);
    AttsHandleValueNtf(connId, WP_DAT_HDL, sizeof(str), str);
    AttsHandleValueNtf(connId, WP_DAT_HDL, sizeof(str), str);
  }
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
static void datsDmCback(dmEvt_t *pDmEvt)
{
  dmEvt_t   *pMsg;
  uint16_t  len;

  if (pDmEvt->hdr.event == DM_SEC_ECC_KEY_IND)
  {
    DmSecSetEccKey(&pDmEvt->eccMsg.data.key);

    /* If the local device sends OOB data. */
    if (datsSendOobData)
    {
      uint8_t oobLocalRandom[SMP_RAND_LEN];
      SecRand(oobLocalRandom, SMP_RAND_LEN);
      DmSecCalcOobReq(oobLocalRandom, pDmEvt->eccMsg.data.key.pubKey_x);
    }
  }
  else if (pDmEvt->hdr.event == DM_SEC_CALC_OOB_IND)
  {
    if (datsOobCfg == NULL)
    {
      datsOobCfg = WsfBufAlloc(sizeof(dmSecLescOobCfg_t));
    }

    if (datsOobCfg)
    {
      Calc128Cpy(datsOobCfg->localConfirm, pDmEvt->oobCalcInd.confirm);
      Calc128Cpy(datsOobCfg->localRandom, pDmEvt->oobCalcInd.random);
    }
  }
  else
  {
    len = DmSizeOfEvt(pDmEvt);

    if ((pMsg = WsfMsgAlloc(len)) != NULL)
    {
      memcpy(pMsg, pDmEvt, len);
      WsfMsgSend(datsCb.handlerId, pMsg);
    }
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
static void datsAttCback(attEvt_t *pEvt)
{
#if WDXS_INCLUDED == TRUE
  WdxsAttCback(pEvt);
#endif /* WDXS_INCLUDED */
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
static void datsCccCback(attsCccEvt_t *pEvt)
{
  appDbHdl_t    dbHdl;

  /* If CCC not set from initialization and there's a device record and currently bonded */
  if ((pEvt->handle != ATT_HANDLE_NONE) &&
      ((dbHdl = AppDbGetHdl((dmConnId_t) pEvt->hdr.param)) != APP_DB_HDL_NONE) &&
      AppCheckBonded((dmConnId_t)pEvt->hdr.param))
  {
    /* Store value in device database. */
    AppDbSetCccTblValue(dbHdl, pEvt->idx, pEvt->value);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  ATTS write callback for proprietary data service.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t datsWpWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                          uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr)
{
  /* print received data */
  APP_TRACE_INFO0((const char*) pValue);

  /* send back some data */
  datsSendData(connId);

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
*
*  \brief  Get peer key from a device database record.
*
*  \param  pMsg    Pointer to DM callback event message.
*
*  \return Pointer to peer key if key is valid or NULL if not valid.
*/
/*************************************************************************************************/
static dmSecKey_t *datsGetPeerKey(dmEvt_t *pMsg)
{
  appDbHdl_t dbHdl;

  /* get device database record handle */
  dbHdl = AppDbGetHdl((dmConnId_t) pMsg->hdr.param);

  /* if database record handle valid */
  if (dbHdl != APP_DB_HDL_NONE)
  {
    return AppDbGetKey(dbHdl, DM_KEY_IRK, NULL);
  }

  return NULL;
}

/*************************************************************************************************/
/*!
*
*  \brief  Handle add device to resolving list indication.
*
*  \param  pMsg    Pointer to DM callback event message.
*
*  \return None.
*/
/*************************************************************************************************/
static void datsPrivAddDevToResListInd(dmEvt_t *pMsg)
{
  dmSecKey_t *pPeerKey;

  /* if peer IRK present */
  if ((pPeerKey = datsGetPeerKey(pMsg)) != NULL)
  {
    /* set advertising peer address */
    AppSetAdvPeerAddr(pPeerKey->irk.addrType, pPeerKey->irk.bdAddr);
  }
}

/*************************************************************************************************/
/*!
*
*  \brief  Handle remove device from resolving list indication.
*
*  \param  pMsg    Pointer to DM callback event message.
*
*  \return None.
*/
/*************************************************************************************************/
static void datsPrivRemDevFromResListInd(dmEvt_t *pMsg)
{
  if (pMsg->hdr.status == HCI_SUCCESS)
  {
    if (AppDbGetHdl((dmConnId_t) pMsg->hdr.param) != APP_DB_HDL_NONE)
    {
      uint8_t addrZeros[BDA_ADDR_LEN] = { 0 };

      /* clear advertising peer address and its type */
      AppSetAdvPeerAddr(HCI_ADDR_TYPE_PUBLIC, addrZeros);
    }
  }
}

/*************************************************************************************************/
/*!
 *
 *  \brief  Display stack version.
 *
 *  \param  version    version number.
 *
 *  \return None.
 */
/*************************************************************************************************/
void datsDisplayStackVersion(const char *pVersion)
{
  APP_TRACE_INFO1("Stack Version: %s", pVersion);
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
static void datsSetup(dmEvt_t *pMsg)
{
  /* set advertising and scan response data for discoverable mode */
  AppAdvSetData(APP_ADV_DATA_DISCOVERABLE, sizeof(datsAdvDataDisc), (uint8_t *) datsAdvDataDisc);
  AppAdvSetData(APP_SCAN_DATA_DISCOVERABLE, sizeof(datsScanDataDisc), (uint8_t *) datsScanDataDisc);

  /* set advertising and scan response data for connectable mode */
  AppAdvSetData(APP_ADV_DATA_CONNECTABLE, sizeof(datsAdvDataDisc), (uint8_t *) datsAdvDataDisc);
  AppAdvSetData(APP_SCAN_DATA_CONNECTABLE, sizeof(datsScanDataDisc), (uint8_t *) datsScanDataDisc);

  /* start advertising; automatically set connectable/discoverable mode and bondable mode */
  AppAdvStart(APP_MODE_AUTO_INIT);
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
static void datsProcMsg(dmEvt_t *pMsg)
{
  uint8_t uiEvent = APP_UI_NONE;

  switch(pMsg->hdr.event)
  {
    case ATT_MTU_UPDATE_IND:
      APP_TRACE_INFO1("Negotiated MTU %d", ((attEvt_t *)pMsg)->mtu);
      break;

    case DM_RESET_CMPL_IND:
      AttsCalculateDbHash();
      DmSecGenerateEccKeyReq();
      datsSetup(pMsg);
      uiEvent = APP_UI_RESET_CMPL;
      break;

    case DM_ADV_START_IND:
      uiEvent = APP_UI_ADV_START;
      break;

    case DM_ADV_STOP_IND:
      uiEvent = APP_UI_ADV_STOP;
      break;

    case DM_CONN_OPEN_IND:
      uiEvent = APP_UI_CONN_OPEN;
#if CS50_INCLUDED == TRUE
      datsCb.phyMode = DATS_PHY_1M;
#endif /* CS50_INCLUDED */
      break;

    case DM_CONN_CLOSE_IND:
      uiEvent = APP_UI_CONN_CLOSE;
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




        if (datsOobCfg != NULL)
        {
          DmSecSetOob(connId, datsOobCfg);
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

    case DM_PRIV_ADD_DEV_TO_RES_LIST_IND:
      datsPrivAddDevToResListInd(pMsg);
      break;

    case DM_PRIV_REM_DEV_FROM_RES_LIST_IND:
      datsPrivRemDevFromResListInd(pMsg);
      break;

    case DM_ADV_NEW_ADDR_IND:
      break;

    case DM_PRIV_CLEAR_RES_LIST_IND:
      APP_TRACE_INFO1("Clear resolving list status 0x%02x", pMsg->hdr.status);
      break;

#if CS50_INCLUDED == TRUE
    case DM_PHY_UPDATE_IND:
      APP_TRACE_INFO2("DM_PHY_UPDATE_IND - RX: %d, TX: %d", pMsg->phyUpdate.rxPhy, pMsg->phyUpdate.txPhy);
      datsCb.phyMode = pMsg->phyUpdate.rxPhy;
      break;
#endif /* CS50_INCLUDED */
    case DM_VENDOR_SPEC_CMD_CMPL_IND:
      {
        #if defined(AM_PART_APOLLO) || defined(AM_PART_APOLLO2)

          uint8_t *param_ptr = &pMsg->vendorSpecCmdCmpl.param[0];

          switch (pMsg->vendorSpecCmdCmpl.opcode)
          {
            case 0xFC20: //read at address
            {
              uint32_t read_value;

              BSTREAM_TO_UINT32(read_value, param_ptr);

              APP_TRACE_INFO3("VSC 0x%0x complete status %x param %x",
                pMsg->vendorSpecCmdCmpl.opcode,
                pMsg->hdr.status,
                read_value);
            }

            break;
            default:
                APP_TRACE_INFO2("VSC 0x%0x complete status %x",
                    pMsg->vendorSpecCmdCmpl.opcode,
                    pMsg->hdr.status);
            break;
          }

        #endif
      }
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
void DatsHandlerInit(wsfHandlerId_t handlerId)
{
  APP_TRACE_INFO0("DatsHandlerInit");

  /* store handler ID */
  datsCb.handlerId = handlerId;

  /* Set configuration pointers */
  pAppSlaveCfg = (appSlaveCfg_t *) &datsSlaveCfg;
  pAppAdvCfg = (appAdvCfg_t *) &datsAdvCfg;
  pAppSecCfg = (appSecCfg_t *) &datsSecCfg;
  pAppUpdateCfg = (appUpdateCfg_t *) &datsUpdateCfg;
  pSmpCfg = (smpCfg_t *) &datsSmpCfg;
  pAttCfg = (attCfg_t *) &datsAttCfg;

  /* Initialize application framework */
  AppSlaveInit();
  AppServerInit();

  /* Set IRK for the local device */
  DmSecSetLocalIrk(localIrk);
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
static void datsBtnCback(uint8_t btn)
{
#if WDXS_INCLUDED == TRUE
  static uint8_t waveform = WDXS_STREAM_WAVEFORM_SINE;
#endif /* WDXS_INCLUDED */

#if CS50_INCLUDED == TRUE
  dmConnId_t      connId;
  if ((connId = AppConnIsOpen()) != DM_CONN_ID_NONE)
#else
  if (AppConnIsOpen() != DM_CONN_ID_NONE)
#endif /* CS50_INCLUDED */
  {
    switch (btn)
    {
#if CS50_INCLUDED == TRUE
      case APP_UI_BTN_2_SHORT:

        /* Toggle PHY Test Mode */
        if (datsCb.phyMode == DATS_PHY_1M)
        {
          APP_TRACE_INFO0("2 MBit TX and RX PHY Requested");
          DmSetPhy(connId, HCI_ALL_PHY_ALL_PREFERENCES, HCI_PHY_LE_2M_BIT, HCI_PHY_LE_2M_BIT, HCI_PHY_OPTIONS_NONE);
        }
        else if (datsCb.phyMode == DATS_PHY_2M)
        {
          APP_TRACE_INFO0("LE Coded TX and RX PHY Requested");
          DmSetPhy(connId, HCI_ALL_PHY_ALL_PREFERENCES, HCI_PHY_LE_CODED_BIT, HCI_PHY_LE_CODED_BIT, HCI_PHY_OPTIONS_NONE);
        }
        else
        {
          APP_TRACE_INFO0("1 MBit TX and RX PHY Requested");
          DmSetPhy(connId, HCI_ALL_PHY_ALL_PREFERENCES, HCI_PHY_LE_1M_BIT, HCI_PHY_LE_1M_BIT, HCI_PHY_OPTIONS_NONE);
        }
        break;

#endif /* CS50_INCLUDED */

#if WDXS_INCLUDED == TRUE
      case APP_UI_BTN_1_SHORT:
        /* Change stream waveform */
        waveform++;

        if (waveform > WDXS_STREAM_WAVEFORM_SAWTOOTH)
          waveform = WDXS_STREAM_WAVEFORM_SINE;

        wdxsSetStreamWaveform(waveform);
        break;
#endif /* WDXS_INCLUDED */

      default:
        break;
    }
  }
  else
  {
    switch (btn)
    {
      case APP_UI_BTN_1_SHORT:
        {
          const char *pVersion;

          StackGetVersionNumber(&pVersion);

          datsDisplayStackVersion(pVersion);
        }
        break;

      case APP_UI_BTN_1_MED:
        /* Enter bondable mode */
        AppSetBondable(TRUE);
        break;

      case APP_UI_BTN_1_LONG:
        /* clear all bonding info */
        AppSlaveClearAllBondingInfo();
        break;

      default:
        break;
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Callback for WSF buffer diagnostic messages.
 *
 *  \param  pInfo     Diagnostics message
 *
 *  \return None.
 */
/*************************************************************************************************/
static void datsWsfBufDiagnostics(WsfBufDiag_t *pInfo)
{
  if (pInfo->type == WSF_BUF_ALLOC_FAILED)
  {
    APP_TRACE_INFO2("Dats got WSF Buffer Allocation Failure - Task: %d Len: %d",
                     pInfo->param.alloc.taskId, pInfo->param.alloc.len);
  }
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
void DatsHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
  if (pMsg != NULL)
  {
    APP_TRACE_INFO1("Dats got evt %d", pMsg->event);

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

#if WDXS_INCLUDED == TRUE
      /* process WDXS-related messages */
      WdxsProcDmMsg((dmEvt_t*) pMsg);
#endif /* WDXS_INCLUDED */
    }

    /* perform profile and user interface-related operations */
    datsProcMsg((dmEvt_t *) pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void DatsStart(void)
{
  /* Register for stack callbacks */
  DmRegister(datsDmCback);
  DmConnRegister(DM_CLIENT_ID_APP, datsDmCback);
  AttRegister(datsAttCback);
  AttConnRegister(AppServerConnCback);
  AttsCccRegister(DATS_NUM_CCC_IDX, (attsCccSet_t *) datsCccSet, datsCccCback);

  /* Initialize attribute server database */
  SvcCoreGattCbackRegister(GattReadCback, GattWriteCback);
  SvcCoreAddGroup();
  SvcWpCbackRegister(NULL, datsWpWriteCback);
  SvcWpAddGroup();

  /* Set Service Changed CCCD index. */
  GattSetSvcChangedIdx(DATS_GATT_SC_CCC_IDX);

  /* Register for app framework button callbacks */
  AppUiBtnRegister(datsBtnCback);

#if WDXS_INCLUDED == TRUE

  /* Initialize the OTA File Media */
  WdxsOtaMediaInit();

  /* Initialize the WDXS Stream */
  wdxsStreamInit();

  /* Set the WDXS CCC Identifiers */
  WdxsSetCccIdx(WDXS_DC_CH_CCC_IDX, WDXS_AU_CH_CCC_IDX, WDXS_FTC_CH_CCC_IDX, WDXS_FTD_CH_CCC_IDX);

#if CS50_INCLUDED == TRUE
  WdxsPhyInit();
#endif /* CS50_INCLUDED */

#endif /* WDXS_INCLUDED */

#if CS50_INCLUDED == TRUE
  DmPhyInit();
#endif /* CS50_INCLUDED */

  WsfBufDiagRegister(datsWsfBufDiagnostics);

  /* Reset the device */
  DmDevReset();
}
