// ****************************************************************************
//
//  barebone_main.c
//! @file
//!
//! @brief Ambiq Micro's demonstration of AmbiqMicro BLE Barebone project.
//!
//! @{
//
// ****************************************************************************
//*****************************************************************************
//
// Copyright (c) 2023, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision release_sdk_4_4_1-7498c7b770 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#define TUTORIAL_ADDING_DIS
#define TUTORIAL_ADDING_BAS
#define TUTORIAL_ADDING_CUSTS
#define TUTORIAL_ADDING_AMOTAS

#include <string.h>
#include "wsf_types.h"
#include "bstream.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "hci_api.h"
#include "dm_api.h"
#include "att_api.h"
#include "smp_api.h"
#include "app_api.h"
#include "app_db.h"
#include "app_ui.h"
#include "app_hw.h"
#include "svc_ch.h"
#include "svc_core.h"

#ifdef TUTORIAL_ADDING_DIS
#include "svc_dis.h"
#endif

#ifdef TUTORIAL_ADDING_BAS
#include "svc_batt.h"
#include "bas_api.h"
#endif

#ifdef TUTORIAL_ADDING_CUSTS
#endif

#ifdef TUTORIAL_ADDING_AMOTAS
#include "svc_amotas.h"
#include "amotas_api.h"
#endif

#include "am_bsp.h"
#include "am_util.h"

#include "barebone_api.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! WSF message event enumeration */
enum
{
    /*! WSF message event starting value */
    BAREBONE_MSG_START      = 0xC0,
#ifdef TUTORIAL_ADDING_BAS
    /*! Battery measurement timer expired */
    BAREBONE_BATT_TIMER_IND = BAREBONE_MSG_START,
#endif
#ifdef TUTORIAL_ADDING_CUSTS
#endif
#ifdef TUTORIAL_ADDING_AMOTAS
    /*! AMOTA reset timer expired */
    BAREBONE_AMOTA_RESET_TIMER_IND,
    /*! AMOTA disconnect timer expired */
    BAREBONE_AMOTA_DISCONNECT_TIMER_IND,
#endif
};



/**************************************************************************************************
  Data Types
**************************************************************************************************/
typedef struct BareboneEnv     BareboneEnv_t;

struct BareboneEnv
{
    wsfHandlerId_t      handlerId;
};

/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/

/*! configurable parameters for advertising */
static const appAdvCfg_t AppAdvCfg =
{
    {    0,     0,     0},                  /*! Advertising durations in ms */
    {  800,     0,     0}                   /*! Advertising intervals in 0.625 ms units */
};

/*! configurable parameters for slave */
static const appSlaveCfg_t AppSlaveCfg =
{
    .connMax = 1,                           /*! Maximum connections */
};

/*! configurable parameters for security */
static const appSecCfg_t AppSecCfg =
{
    0,                                      /*! Authentication and bonding flags */
    0,                                      /*! Initiator key distribution flags */
    DM_KEY_DIST_LTK,                        /*! Responder key distribution flags */
    FALSE,                                  /*! TRUE if Out-of-band pairing data is present */
    FALSE                                   /*! TRUE to initiate security upon connection */
};

/*! configurable parameters for connection parameter update */
static const appUpdateCfg_t AppUpdateCfg =
{
    3000,                                   /*! Connection idle period in ms before attempting
                                              connection parameter update; set to zero to disable */
    24,                                     /*! 30ms */
    24,                                     /*! 30ms */
    0,                                      /*! Connection latency */
    600,                                    /*! Supervision timeout in 10ms units */
    5                                       /*! Number of update attempts before giving up */
};

/*! SMP security parameter configuration */
static const smpCfg_t AppSmpCfg =
{
    3000,                                   /*! 'Repeated attempts' timeout in msec */
    SMP_IO_NO_IN_NO_OUT,                    /*! I/O Capability */
    7,                                      /*! Minimum encryption key length */
    16,                                     /*! Maximum encryption key length */
    3,                                      /*! Attempts to trigger 'repeated attempts' timeout */
    0,                                      /*! Device authentication requirements */
};

#ifdef TUTORIAL_ADDING_BAS
/*! battery measurement configuration */
static const basCfg_t BareboneBasCfg =
{
    /*! Battery measurement timer expiration period in seconds */
    .period     = 5,
    /*! Perform battery measurement after this many timer periods */
    .count      = 1,
    /*! Send battery level notification to peer when below this level. Not used for the moment*/
    .threshold  = 100,
};
#endif

#ifdef TUTORIAL_ADDING_AMOTAS
/*! AMOTAS configuration */
static const AmotasCfg_t s_sAmotasCfg =
{
    .reserved   = 0,
};
#endif

/**************************************************************************************************
    Advertising Data
**************************************************************************************************/
/*! advertising data, discoverable mode */
static const uint8_t AppAdvDataDisc[] =
{
    /*! flags */
    2,                                      /*! length */
    DM_ADV_TYPE_FLAGS,                      /*! AD type */
    DM_FLAG_LE_GENERAL_DISC |               /*! flags */
    DM_FLAG_LE_BREDR_NOT_SUP,

#ifdef TUTORIAL_ADDING_DIS
    /*! service UUID list */
    3,                                      /*! length */
    DM_ADV_TYPE_16_UUID,                    /*! AD type */
    UINT16_TO_BYTES(ATT_UUID_DEVICE_INFO_SERVICE),
#endif
};

/*! scan data, discoverable mode */
static const uint8_t AppScanDataDisc[] =
    {
    /*! device name */
    12,                                     /*! length */
    DM_ADV_TYPE_LOCAL_NAME,                 /*! AD type */
    'A',
    'M',
    ' ',
    'B',
    'a',
    'r',
    'e',
    'b',
    'o',
    'n',
    'e',
};

/**************************************************************************************************
    Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! enumeration of client characteristic configuration descriptors */
enum
{
    /*! GATT service, service changed characteristic */
    BAREBONE_GATT_SC_CCC_IDX,
#ifdef TUTORIAL_ADDING_BAS
    /*! Battery service, battery level characteristic */
    BAREBONE_BATT_LVL_CCC_IDX,
#endif
#ifdef TUTORIAL_ADDING_CUSTS
#endif
#ifdef TUTORIAL_ADDING_AMOTAS
    /*! AMOTA service, tx characteristic */
    BAREBONE_AMOTAS_TX_CCC_IDX,
#endif
    BAREBONE_NUM_CCC_IDX
};

// /*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t BareboneCccSet[BAREBONE_NUM_CCC_IDX] =
{
    /* cccd handle                      value range                 security level */
    {GATT_SC_CH_CCC_HDL,                ATT_CLIENT_CFG_INDICATE,    DM_SEC_LEVEL_NONE},
#ifdef TUTORIAL_ADDING_BAS
    {BATT_LVL_CH_CCC_HDL,               ATT_CLIENT_CFG_NOTIFY,      DM_SEC_LEVEL_NONE},
#endif
#ifdef TUTORIAL_ADDING_CUSTS
#endif
#ifdef TUTORIAL_ADDING_AMOTAS
    {AMOTAS_TX_CH_CCC_HDL,              ATT_CLIENT_CFG_NOTIFY,      DM_SEC_LEVEL_NONE},
#endif
};

/**************************************************************************************************
    App Variables.
**************************************************************************************************/
static BareboneEnv_t sBareBoneEnv;


/*************************************************************************************************/
/*!
 *  \fn     BareboneDmCback
 *
 *  \brief  Application DM callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void BareboneDmCback(dmEvt_t *pDmEvt)
{
    dmEvt_t *pMsg;
    uint16_t len;

    len = DmSizeOfEvt(pDmEvt);

    if ((pMsg = WsfMsgAlloc(len)) != NULL)
    {
        memcpy(pMsg, pDmEvt, len);
        WsfMsgSend(sBareBoneEnv.handlerId, pMsg);
    }
}

/*************************************************************************************************/
/*!
 *  \fn     BareboneAttCback
 *
 *  \brief  Application ATT callback.
 *
 *  \param  pEvt    ATT callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void BareboneAttCback(attEvt_t *pEvt)
{
    attEvt_t *pMsg;

    if ((pMsg = WsfMsgAlloc(sizeof(attEvt_t) + pEvt->valueLen)) != NULL)
    {
        memcpy(pMsg, pEvt, sizeof(attEvt_t));
        pMsg->pValue = (uint8_t *) (pMsg + 1);
        memcpy(pMsg->pValue, pEvt->pValue, pEvt->valueLen);
        WsfMsgSend(sBareBoneEnv.handlerId, pMsg);
    }
}

/*************************************************************************************************/
/*!
 *  \fn     BareboneCccCback
 *
 *  \brief  Application ATTS client characteristic configuration callback.
 *
 *  \param  _pEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void BareboneCccCback(attsCccEvt_t *pEvt)
{
    attsCccEvt_t  *pMsg;
    appDbHdl_t    dbHdl;

    /* if CCC not set from initialization and there's a device record */
    if ((pEvt->handle != ATT_HANDLE_NONE) &&
        ((dbHdl = AppDbGetHdl((dmConnId_t) pEvt->hdr.param)) != APP_DB_HDL_NONE))
    {
        /* store value in device database */
        AppDbSetCccTblValue(dbHdl, pEvt->idx, pEvt->value);
    }

    if ((pMsg = WsfMsgAlloc(sizeof(attsCccEvt_t))) != NULL)
    {
        memcpy(pMsg, pEvt, sizeof(attsCccEvt_t));
        WsfMsgSend(sBareBoneEnv.handlerId, pMsg);
    }
}

/*************************************************************************************************/
/*!
 *  \fn     BareboneProcCccState
 *
 *  \brief  Process CCC state change.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void BareboneProcCccState(wsfMsgHdr_t *pMsg)
{
    attsCccEvt_t *pEvt = (attsCccEvt_t *)pMsg;

    APP_TRACE_INFO1("[%s]", __func__);
    APP_TRACE_INFO1("handle           = 0x%X", pEvt->handle);
    APP_TRACE_INFO1("value            = 0x%X", pEvt->value);
    APP_TRACE_INFO1("idx              = 0x%X", pEvt->idx);

    switch(pEvt->idx)
    {
#ifdef TUTORIAL_ADDING_BAS
        /* handle battery level CCC */
        case BAREBONE_BATT_LVL_CCC_IDX:
        {
            if (pEvt->value == ATT_CLIENT_CFG_NOTIFY)
            {
                BasMeasBattStart((dmConnId_t) pEvt->hdr.param, BAREBONE_BATT_TIMER_IND, BAREBONE_BATT_LVL_CCC_IDX);
            }
            else
            {
                BasMeasBattStop((dmConnId_t) pEvt->hdr.param);
            }
        }
        break;
#endif
#ifdef TUTORIAL_ADDING_AMOTAS
        case BAREBONE_AMOTAS_TX_CCC_IDX:
        {
            if (pEvt->value == ATT_CLIENT_CFG_NOTIFY)
            {
                // notify enabled
                amotas_start((dmConnId_t) pEvt->hdr.param, BAREBONE_AMOTA_RESET_TIMER_IND, BAREBONE_AMOTA_DISCONNECT_TIMER_IND, BAREBONE_AMOTAS_TX_CCC_IDX);
            }
            else
            {
                // notify disabled
                amotas_stop((dmConnId_t) pEvt->hdr.param);
            }
        }
#endif
        default:
        break;
    }
}

/*************************************************************************************************/
/*!
 *  \fn     BareboneSetup
 *
 *  \brief  Set up advertising and other procedures that need to be performed after
 *          device reset.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void BareboneSetup(wsfMsgHdr_t *pMsg)
{
    /* set advertising and scan response data for discoverable mode */
    AppAdvSetData(APP_ADV_DATA_DISCOVERABLE, sizeof(AppAdvDataDisc), (uint8_t *) AppAdvDataDisc);
    AppAdvSetData(APP_SCAN_DATA_DISCOVERABLE, sizeof(AppScanDataDisc), (uint8_t *) AppScanDataDisc);

    /* set advertising and scan response data for connectable mode */
    AppAdvSetData(APP_ADV_DATA_CONNECTABLE, sizeof(AppAdvDataDisc), (uint8_t *) AppAdvDataDisc);
    AppAdvSetData(APP_SCAN_DATA_CONNECTABLE, sizeof(AppScanDataDisc), (uint8_t *) AppScanDataDisc);

    /* start advertising; automatically set connectable/discoverable mode and bondable mode */
    AppAdvStart(APP_MODE_AUTO_INIT);
}

/*************************************************************************************************/
/*!
 *  \fn     BareboneBtnCback
 *
 *  \brief  Button press callback.
 *
 *  \param  btn    Button press.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void BareboneBtnCback(uint8_t btn)
{
    dmConnId_t      connId;

    /* button actions when connected */
    if ((connId = AppConnIsOpen()) != DM_CONN_ID_NONE)
    {
        switch (btn)
        {
            case APP_UI_BTN_1_SHORT:
            break;

            case APP_UI_BTN_1_MED:
            break;

            case APP_UI_BTN_1_LONG:
                AppConnClose(connId);
            break;

            case APP_UI_BTN_2_SHORT:
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
                /* start or restart advertising */
                AppAdvStart(APP_MODE_AUTO_INIT);
            break;

            case APP_UI_BTN_1_MED:
                /* enter discoverable and bondable mode mode */
                AppSetBondable(TRUE);
                AppAdvStart(APP_MODE_DISCOVERABLE);
            break;

            case APP_UI_BTN_1_LONG:
                /* clear bonded device info and restart advertising */
                AppDbDeleteAllRecords();
                AppAdvStart(APP_MODE_AUTO_INIT);
            break;

            default:
            break;
        }
    }
}

/*************************************************************************************************/
/*!
 *  \fn     BareboneProcMsg
 *
 *  \brief  Process messages from the event handler.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void BareboneProcMsg(wsfMsgHdr_t *pMsg)
{
    uint8_t uiEvent = APP_UI_NONE;

    switch (pMsg->event)
    {
        case ATTS_HANDLE_VALUE_CNF:
#ifdef TUTORIAL_ADDING_BAS
            BasProcMsg(pMsg);
#endif
#ifdef TUTORIAL_ADDING_CUSTS
#endif
        break;

        case ATTS_CCC_STATE_IND:
            BareboneProcCccState(pMsg);
        break;

        case ATT_MTU_UPDATE_IND:
            APP_TRACE_INFO1("mtu              = %d", ((attEvt_t*)pMsg)->mtu);
        break;

        case DM_RESET_CMPL_IND:
            DmSecGenerateEccKeyReq();
            uiEvent = APP_UI_RESET_CMPL;
        break;
        case DM_ADV_START_IND:
            uiEvent = APP_UI_ADV_START;
        break;

        case DM_ADV_STOP_IND:
            uiEvent = APP_UI_ADV_STOP;
        break;
        case DM_CONN_OPEN_IND:
        {
            APP_TRACE_INFO0("DM_CONN_OPEN_IND");
            APP_TRACE_INFO1("connId           = %d", pMsg->param);
            APP_TRACE_INFO1("handle           = %d", ((dmEvt_t*)pMsg)->connOpen.handle);
            APP_TRACE_INFO1("role             = %d", ((dmEvt_t*)pMsg)->connOpen.role);
            APP_TRACE_INFO3("addrMSB          = %02X:%02X:%02X", \
                                                    ((dmEvt_t*)pMsg)->connOpen.peerAddr[0], \
                                                    ((dmEvt_t*)pMsg)->connOpen.peerAddr[1], \
                                                    ((dmEvt_t*)pMsg)->connOpen.peerAddr[2]);
            APP_TRACE_INFO3("addrLSB          = %02X:%02X:%02X", \
                                                    ((dmEvt_t*)pMsg)->connOpen.peerAddr[3], \
                                                    ((dmEvt_t*)pMsg)->connOpen.peerAddr[4], \
                                                    ((dmEvt_t*)pMsg)->connOpen.peerAddr[5]);
            APP_TRACE_INFO1("connInterval     = %d", ((dmEvt_t*)pMsg)->connOpen.connInterval);
            APP_TRACE_INFO1("connLatency      = %d", ((dmEvt_t*)pMsg)->connOpen.connLatency);
            APP_TRACE_INFO1("supTimeout       = %d", ((dmEvt_t*)pMsg)->connOpen.supTimeout);
            uiEvent = APP_UI_CONN_OPEN;
#ifdef TUTORIAL_ADDING_BAS
            BasProcMsg(pMsg);
#endif
#ifdef TUTORIAL_ADDING_CUSTS
#endif
#ifdef TUTORIAL_ADDING_AMOTAS
            amotas_proc_msg(pMsg);
#endif
        }
        break;

        case DM_CONN_CLOSE_IND:
            APP_TRACE_INFO0("DM_CONN_CLOSE_IND");
            APP_TRACE_INFO1("connID           = 0x%02X", (dmConnId_t) pMsg->param);
            APP_TRACE_INFO1("reason           = 0x%02X", ((dmEvt_t*)pMsg)->connClose.reason);
            uiEvent = APP_UI_CONN_CLOSE;
#ifdef TUTORIAL_ADDING_BAS
            BasMeasBattStop((dmConnId_t) pMsg->param);
#endif
#ifdef TUTORIAL_ADDING_CUSTS
#endif
#ifdef TUTORIAL_ADDING_AMOTAS
            amotas_conn_close((dmConnId_t) pMsg->param);
#endif
        break;

        case DM_CONN_UPDATE_IND:
#ifdef TUTORIAL_ADDING_AMOTAS
            amotas_proc_msg(pMsg);
#endif
        break;
        case DM_SEC_PAIR_CMPL_IND:
            uiEvent = APP_UI_SEC_PAIR_CMPL;
        break;

        case DM_SEC_PAIR_FAIL_IND:
            uiEvent = APP_UI_SEC_PAIR_FAIL;
        break;

        case DM_SEC_ENCRYPT_IND:
            uiEvent = APP_UI_SEC_ENCRYPT;
        break;

        case DM_SEC_ENCRYPT_FAIL_IND:
            uiEvent = APP_UI_SEC_ENCRYPT_FAIL;
        break;

        case DM_SEC_AUTH_REQ_IND:
            AppHandlePasskey(&((dmEvt_t*)pMsg)->authReq);
        break;

        case DM_SEC_ECC_KEY_IND:
            BareboneSetup(pMsg);
            DmSecSetEccKey(&((dmEvt_t*)pMsg)->eccMsg.data.key);
        break;

        case DM_SEC_COMPARE_IND:
            AppHandleNumericComparison(&((dmEvt_t*)pMsg)->cnfInd);
        break;
        case DM_CONN_DATA_LEN_CHANGE_IND:
        break;
#ifdef TUTORIAL_ADDING_BAS
        case BAREBONE_BATT_TIMER_IND:
        {
            static uint8_t s_battlvl = 100;
            AppHwBattTest(s_battlvl--);
            if (s_battlvl == 0x00)
            {
                s_battlvl = 100;
            }
            BasProcMsg(pMsg);
        }
        break;
#endif
#ifdef TUTORIAL_ADDING_CUSTS
#endif
#ifdef TUTORIAL_ADDING_AMOTAS
        case BAREBONE_AMOTA_RESET_TIMER_IND:
        case BAREBONE_AMOTA_DISCONNECT_TIMER_IND:
            amotas_proc_msg(pMsg);
        break;
#endif
        default:
            // do not care this message. just jgnore.
        break;

    };

    if (uiEvent != APP_UI_NONE)
    {
        AppUiAction(uiEvent);
    }

}

/*************************************************************************************************/
/*!
 *  \fn     BareboneInit
 *
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID for App.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BareboneInit(wsfHandlerId_t handlerId)
{
    APP_TRACE_INFO1("[%s]", __func__);
    /* store handler ID */
    sBareBoneEnv.handlerId    = handlerId;

    pAppAdvCfg                  = (appAdvCfg_t *) &AppAdvCfg;
    pAppSlaveCfg                = (appSlaveCfg_t *) &AppSlaveCfg;
    pAppSecCfg                  = (appSecCfg_t *) &AppSecCfg;
    pAppUpdateCfg               = (appUpdateCfg_t *) &AppUpdateCfg;
    pSmpCfg                     = (smpCfg_t *) &AppSmpCfg;

    /* Initialize application framework */
    AppSlaveInit();

#ifdef TUTORIAL_ADDING_BAS
    /* initialize battery service server */
    BasInit(handlerId, (basCfg_t *) &BareboneBasCfg);
#endif

#ifdef TUTORIAL_ADDING_CUSTS
#endif

#ifdef TUTORIAL_ADDING_AMOTAS
    amotas_init(handlerId, (AmotasCfg_t *) &s_sAmotasCfg);
#endif

}

/*************************************************************************************************/
/*!
 *  \fn     BareboneHandler
 *
 *  \brief  WSF event handler for the application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BareboneHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    if (pMsg != NULL)
    {

        if (pMsg->event >= DM_CBACK_START && pMsg->event <= DM_CBACK_END)
        {
            /* process advertising and connection-related messages */
            AppSlaveProcDmMsg((dmEvt_t *) pMsg);

            /* process security-related messages */
            AppSlaveSecProcDmMsg((dmEvt_t *) pMsg);
        }

        /* perform profile and user interface-related operations */
        BareboneProcMsg(pMsg);

    }
}

/*************************************************************************************************/
/*!
 *  \fn     BareboneStart
 *
 *  \brief  Start the barebone application
 *
 *  \return None.
 */
/*************************************************************************************************/
void BareboneStart(void)
{
    /* Register for stack callbacks */
    DmRegister(BareboneDmCback);
    DmConnRegister(DM_CLIENT_ID_APP, BareboneDmCback);
    AttRegister(BareboneAttCback);
    AttConnRegister(AppServerConnCback);
    AttsCccRegister(BAREBONE_NUM_CCC_IDX, (attsCccSet_t *) BareboneCccSet, BareboneCccCback);

    /* Register for app framework callbacks */
    AppUiBtnRegister(BareboneBtnCback);

    /* Initialize attribute server database */
    SvcCoreAddGroup();

#ifdef TUTORIAL_ADDING_DIS
    SvcDisAddGroup();
#endif

#ifdef TUTORIAL_ADDING_BAS
    SvcBattCbackRegister(BasReadCback, NULL);
    SvcBattAddGroup();
#endif

#ifdef TUTORIAL_ADDING_CUSTS
#endif

#ifdef TUTORIAL_ADDING_AMOTAS
    SvcAmotasCbackRegister(NULL, amotas_write_cback);
    SvcAmotasAddGroup();
#endif
    /* Reset the device */
    DmDevReset();

}
