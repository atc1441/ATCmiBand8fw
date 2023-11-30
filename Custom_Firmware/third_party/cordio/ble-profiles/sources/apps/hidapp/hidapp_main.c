/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  HID HidApp sample application.
 *
 *  Copyright (c) 2015-2019 Arm Ltd. All Rights Reserved.
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
#include "wsf_os.h"
#include "hci_api.h"
#include "dm_api.h"
#include "att_api.h"
#include "smp_api.h"
#include "app_cfg.h"
#include "app_api.h"
#include "app_db.h"
#include "app_ui.h"
#include "app_main.h"
#include "svc_ch.h"
#include "svc_hid.h"
#include "svc_core.h"
#include "svc_dis.h"
#include "svc_batt.h"
#include "gatt/gatt_api.h"
#include "bas/bas_api.h"
#include "hid/hid_api.h"
#include "hidapp/hidapp_api.h"
#include "atts_main.h"
/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/

/*! configurable parameters for advertising */
static const appAdvCfg_t hidAppAdvCfg =
{
  {60000,     0,     0},                  /*! Advertising durations in ms */
  {   64,  1600,     0}                   /*! Advertising intervals in 0.625 ms units */
};

/*! configurable parameters for slave */
static const appSlaveCfg_t hidAppSlaveCfg =
{
  1,                                      /*! Maximum connections */
};

/*! configurable parameters for security */
static const appSecCfg_t hidAppSecCfg =
{
  DM_AUTH_BOND_FLAG | DM_AUTH_SC_FLAG,    /*! Authentication and bonding flags */
  0,                                      /*! Initiator key distribution flags */
  DM_KEY_DIST_LTK,                        /*! Responder key distribution flags */
  FALSE,                                  /*! TRUE if Out-of-band pairing data is present */
  TRUE                                    /*! TRUE to initiate security upon connection */
};

/*! configurable parameters for connection parameter update */
static const appUpdateCfg_t hidAppUpdateCfg =
{
  0,                                      /*! Connection idle period in ms before attempting
                                              connection parameter update; set to zero to disable */
  640,                                    /*! Minimum connection interval in 1.25ms units */
  800,                                    /*! Maximum connection interval in 1.25ms units */
  3,                                      /*! Connection latency */
  600,                                    /*! Supervision timeout in 10ms units */
  5                                       /*! Number of update attempts before giving up */
};

/*! battery measurement configuration */
static const basCfg_t hidAppBasCfg =
{
  30,       /*! Battery measurement timer expiration period in seconds */
  1,        /*! Perform battery measurement after this many timer periods */
  100       /*! Send battery level notification to peer when below this level. */
};

/**************************************************************************************************
  Advertising Data
**************************************************************************************************/

/*! advertising data, discoverable mode */
static const uint8_t hidAppAdvDataDisc[] =
{
  /*! flags */
  2,                                      /*! length */
  DM_ADV_TYPE_FLAGS,                      /*! AD type */
  DM_FLAG_LE_GENERAL_DISC |               /*! flags */
  DM_FLAG_LE_BREDR_NOT_SUP,

  /*! manufacturer specific data */
  3,                                      /*! length */
  DM_ADV_TYPE_MANUFACTURER,               /*! AD type */
  UINT16_TO_BYTES(HCI_ID_PACKETCRAFT),    /*! company ID */

  /*! service UUID list */
  3,                                      /*! length */
  DM_ADV_TYPE_16_UUID,                    /*! AD type */
  UINT16_TO_BYTES(ATT_UUID_HID_SERVICE)
};

/*! scan data, discoverable mode */
static const uint8_t hidAppScanDataDisc[] =
{
  /*! device name */
  8,                                      /*! length */
  DM_ADV_TYPE_LOCAL_NAME,                 /*! AD type */
  'H',
  'I',
  'D',
  ' ',
  'A',
  'p',
  'p'
};

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! WSF message event starting value */
#define HIDAPP_MSG_START               0xA0

/*! WSF message event enumeration */
enum
{
  HIDAPP_BATT_TIMER_IND = HIDAPP_MSG_START,       /*! Battery timer expired */
};

/* HidApp TX path flags */
#define HIDAPP_TX_FLAGS_READY             0x01
#define HIDAPP_TX_FLAGS_PENDING           0x02

/* The input report fits in one byte */
#define HIDAPP_KEYBOARD_INPUT_REPORT_LEN  8
#define HIDAPP_MOUSE_INPUT_REPORT_LEN     4
#define HIDAPP_REMOTE_INPUT_REPORT_LEN    1
#define HIDAPP_OUTPUT_REPORT_LEN          1
#define HIDAPP_FEATURE_REPORT_LEN         1

/* Remote Button Identifier bits */
#define REMOTE_USAGE_NONE                 (0<<0)
#define REMOTE_PLAY_PAUSE                 (1<<0)
#define REMOTE_AL_CCC                     (1<<1)
#define REMOTE_SCAN_NEXT                  (1<<2)
#define REMOTE_SCAN_PREVIOUS              (1<<3)
#define REMOTE_VOLUME_DOWN                (1<<4)
#define REMOTE_VOLUME_UP                  (1<<5)
#define REMOTE_AC_FORWARD                 (1<<6)
#define REMOTE_AC_BACKWARD                (1<<7)

/* Keyboard input record message format */
#define KEYBOARD_IR_MODIFIER_POS          0
#define KEYBOARD_IR_RESERVED_POS          1
#define KEYBOARD_IR_KEY_POS               2
#define KEYBOARD_IR_MAX_KEYS              6

/* Keyboard LED Identifier bits */
#define KEYBOARD_LED_NUM_LOCK             (1<<0)
#define KEYBOARD_LED_CAPS_LOCK            (1<<1)
#define KEYBOARD_LED_SCROLL_LOCK          (1<<2)
#define KEYBOARD_LED_COMPOSE              (1<<3)
#define KEYBOARD_LED_KANA                 (1<<4)

/* Keyboard usage definitions */
#define KEYBOARD_USAGE_NONE               0x00
#define KEYBOARD_USAGE_CAPS_LOCK          0x39
#define KEYBOARD_USAGE_RIGHT_ARROW        0x4F
#define KEYBOARD_USAGE_LEFT_ARROW         0x50
#define KEYBOARD_USAGE_DOWN_ARROW         0x51
#define KEYBOARD_USAGE_UP_ARROW           0x52

/* Mouse button bit mask */
#define MOUSE_BUTTON_LEFT                 (1<<0)
#define MOUSE_BUTTON_RIGHT                (1<<1)
#define MOUSE_BUTTON_MIDDLE               (1<<2)

/* Mouse input record message format */
#define MOUSE_BUTTON_POS                  0
#define MOUSE_X_POS                       1
#define MOUSE_Y_POS                       2

/* Mouse usage definitions */
#define MOUSE_USAGE_NONE                  0x00
#define MOUSE_USAGE_UP_ARROW              0x52

/* HID Report IDs */
#define HIDAPP_REMOTE_REPORT_ID           1
#define HIDAPP_KEYBOARD_REPORT_ID         2
#define HIDAPP_MOUSE_REPORT_ID            3

/* Test button identifiers */
enum
{
  HIDAPP_REMOTE_PLAY_PAUSE_BTN,
  HIDAPP_REMOTE_SCAN_NEXT_BTN,
  HIDAPP_REMOTE_SCAN_PREVIOUS_BTN,
  HIDAPP_REMOTE_VOLUME_DOWN_BTN,
  HIDAPP_REMOTE_VOLUME_UP_BTN,
  HIDAPP_MOUSE_LEFT_BTN,
  HIDAPP_MOUSE_RIGHT_BTN,
  HIDAPP_KEYBOARD_UP_BTN,
  HIDAPP_KEYBOARD_DOWN_BTN,
  HIDAPP_KEYBOARD_LEFT_BTN,
  HIDAPP_KEYBOARD_RIGHT_BTN,

  HIDAPP_NUM_TEST_BUTTONS
};

/**************************************************************************************************
  Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! enumeration of client characteristic configuration descriptors */
enum
{
  HIDAPP_GATT_SC_CCC_IDX,               /*! GATT service, service changed characteristic */
  HIDAPP_MBI_CCC_HDL,                   /*! HID Boot Mouse Input characteristic */
  HIDAPP_KBI_CCC_HDL,                   /*! HID Boot Keyboard Input characteristic */
  HIDAPP_IN_REMOTE_CCC_HDL,             /*! HID Input Report characteristic for remote inputs */
  HIDAPP_IN_KEYBOARD_CCC_HDL,           /*! HID Input Report characteristic for keyboard inputs */
  HIDAPP_IN_MOUSE_CCC_HDL,              /*! HID Input Report characteristic for mouse inputs */
  HIDAPP_BATT_LVL_CCC_IDX,              /*! Battery service, battery level characteristic */
  HIDAPP_NUM_CCC_IDX
};

/*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t hidAppCccSet[HIDAPP_NUM_CCC_IDX] =
{
  /* cccd handle                        value range               security level */
  {GATT_SC_CH_CCC_HDL,                  ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_NONE},    /* HIDAPP_GATT_SC_CCC_IDX */
  {HID_MOUSE_BOOT_IN_CH_CCC_HDL,        ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},    /* HIDAPP_MBI_CCC_HDL */
  {HID_KEYBOARD_BOOT_IN_CH_CCC_HDL,     ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},    /* HIDAPP_KBI_CCC_HDL */
  {HID_INPUT_REPORT_1_CH_CCC_HDL,       ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},    /* HIDAPP_IN_REMOTE_CCC_HDL */
  {HID_INPUT_REPORT_2_CH_CCC_HDL,       ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},    /* HIDAPP_IN_KEYBOARD_CCC_HDL */
  {HID_INPUT_REPORT_3_CH_CCC_HDL,       ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},    /* HIDAPP_IN_MOUSE_CCC_HDL */
  {BATT_LVL_CH_CCC_HDL,                 ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE}     /* HIDAPP_BATT_LVL_CCC_IDX */
};

/*! HID Report Type/ID and attribute handle map */
static const hidReportIdMap_t hidAppReportIdSet[] =
{
  /* type                       ID                            handle */
  {HID_REPORT_TYPE_INPUT,       HIDAPP_REMOTE_REPORT_ID,      HID_INPUT_REPORT_1_HDL},     /* Remote Input Report */
  {HID_REPORT_TYPE_INPUT,       HIDAPP_KEYBOARD_REPORT_ID,    HID_INPUT_REPORT_2_HDL},     /* Keyboard Input Report */
  {HID_REPORT_TYPE_OUTPUT,      HIDAPP_KEYBOARD_REPORT_ID,    HID_OUTPUT_REPORT_HDL},      /* Keyboard Output Report */
  {HID_REPORT_TYPE_FEATURE,     HIDAPP_KEYBOARD_REPORT_ID,    HID_FEATURE_REPORT_HDL},     /* Keyboard Feature Report */
  {HID_REPORT_TYPE_INPUT,       HIDAPP_MOUSE_REPORT_ID,       HID_INPUT_REPORT_3_HDL},     /* Mouse Input Report */
  {HID_REPORT_TYPE_INPUT,       HID_KEYBOARD_BOOT_ID,         HID_KEYBOARD_BOOT_IN_HDL},   /* Boot Keyboard Input Report */
  {HID_REPORT_TYPE_OUTPUT,      HID_KEYBOARD_BOOT_ID,         HID_KEYBOARD_BOOT_OUT_HDL},  /* Boot Keyboard Output Report */
  {HID_REPORT_TYPE_INPUT,       HID_MOUSE_BOOT_ID,            HID_MOUSE_BOOT_IN_HDL},      /* Boot Mouse Input Report */
};

/*! HidApp Report Map (Descriptor) */
const uint8_t hidReportMap[] =
{
  0x05, 0x0c,                    /*	Usage Page (Consumer Devices) */
  0x09, 0x01,                    /*	Usage (Consumer Control) */
  0xa1, 0x01,                    /*	Collection (Application) */
  0x85, HIDAPP_REMOTE_REPORT_ID, /*   report ID (HIDAPP_REMOTE_REPORT_ID) */
  0x15, 0x00,       // Logical minimum (0)
  0x25, 0x01,       // Logical maximum (1)
  0x75, 0x01,       // Report Size (1)
  0x95, 0x01,       // Report Count (1)

  0x09, 0xCD,       // Usage (Play/Pause)
  0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
  0x0A, 0x83, 0x01, // Usage (AL Consumer Control Configuration)
  0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
  0x09, 0xB5,       // Usage (Scan Next Track)
  0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
  0x09, 0xB6,       // Usage (Scan Previous Track)
  0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)

  0x09, 0xEA,       // Usage (Volume Down)
  0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
  0x09, 0xE9,       // Usage (Volume Up)
  0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
  0x0A, 0x25, 0x02, // Usage (AC Forward)
  0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
  0x0A, 0x24, 0x02, // Usage (AC Back)
  0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
  0xC0,              // End Collection
  0x05, 0x01,                    /* Usage Page (Generic Desktop) */
  0x09, 0x06,                    /* Usage (Keyboard) */
  0xA1, 0x01,                    /* Collection (Application) */
  0x85, HIDAPP_KEYBOARD_REPORT_ID, /*   report ID (HIDAPP_KEYBOARD_REPORT_ID) */
  0x75, 0x01,                    /*   Report Size (1) */
  0x95, 0x08,                    /*   Report Count (8) */
  0x05, 0x07,                    /*   Usage Page (Key Codes) */
  0x19, 0xe0,                    /*   Usage Minimum (224) */
  0x29, 0xe7,                    /*   Usage Maximum (231) */
  0x15, 0x00,                    /*   Logical Minimum (0) */
  0x25, 0x01,                    /*   Logical Maximum (1) */
  0x81, 0x02,                    /*   Input (Data, Variable, Absolute) */
  0x95, 0x01,                    /*   Report Count (1) */
  0x75, 0x08,                    /*   Report Size (8) */
  0x81, 0x01,                    /*   Input (Constant) reserved byte(1) */
  0x95, 0x05,                    /*   Report Count (5) */
  0x75, 0x01,                    /*   Report Size (1) */
  0x05, 0x08,                    /*   Usage Page (Page# for LEDs) */
  0x19, 0x01,                    /*   Usage Minimum (1) */
  0x29, 0x05,                    /*   Usage Maximum (5) */
  0x91, 0x02,                    /*   Output (Data, Variable, Absolute), Led report */
  0x95, 0x01,                    /*   Report Count (1) */
  0x75, 0x03,                    /*   Report Size (3) */
  0x91, 0x01,                    /*   Output (Constant), Led report padding */
  0x95, 0x06,                    /*   Report Count (6) */
  0x75, 0x08,                    /*   Report Size (8) */
  0x15, 0x00,                    /*   Logical Minimum (0) */
  0x25, 0x65,                    /*   Logical Maximum (101) */
  0x05, 0x07,                    /*   Usage Page (Key codes) */
  0x19, 0x00,                    /*   Usage Minimum (0) */
  0x29, 0x65,                    /*   Usage Maximum (101) */
  0x81, 0x00,                    /*   Input (Data, Array) Key array(6 bytes) */
  0xC0,                          /* End Collection (Application) */
  0x05, 0x01,                    /* USAGE_PAGE (Generic Desktop) */
  0x09, 0x02,                    /* USAGE (Mouse) */
  0xa1, 0x01,                    /* COLLECTION (Application) */
  0x85, HIDAPP_MOUSE_REPORT_ID,  /*   report ID (HIDAPP_MOUSE_REPORT_ID) */
  0x09, 0x01,                    /*   USAGE (Pointer) */
  0xa1, 0x00,                    /*   COLLECTION (Physical) */
  0x95, 0x03,                    /*     REPORT_COUNT (3) */
  0x75, 0x01,                    /*     REPORT_SIZE (1) */
  0x05, 0x09,                    /*     USAGE_PAGE (Button) */
  0x19, 0x01,                    /*     USAGE_MINIMUM (Button 1) */
  0x29, 0x03,                    /*     USAGE_MAXIMUM (Button 3) */
  0x15, 0x00,                    /*     LOGICAL_MINIMUM (0) */
  0x25, 0x01,                    /*     LOGICAL_MAXIMUM (1) */
  0x81, 0x02,                    /*     INPUT (Data, Variable, Absolute) */
  0x95, 0x01,                    /*     REPORT_COUNT (1) */
  0x75, 0x05,                    /*     REPORT_SIZE (5) */
  0x81, 0x01,                    /*     INPUT (Constant) */
  0x75, 0x08,                    /*     REPORT_SIZE (8) */
  0x95, 0x02,                    /*     REPORT_COUNT (2) */
  0x05, 0x01,                    /*     USAGE_PAGE (Generic Desktop) */
  0x09, 0x30,                    /*     USAGE (X) */
  0x09, 0x31,                    /*     USAGE (Y) */
  0x15, 0x81,                    /*     LOGICAL_MINIMUM (-127) */
  0x25, 0x7f,                    /*     LOGICAL_MAXIMUM (127) */
  0x81, 0x06,                    /*     INPUT (Data, Variable, Relative) */
  0x09, 0x38,                    /*     AB usage wheel*/
  0x15, 0x81,                    /*     Logical Minimum (-127) */
  0x25, 0x7F,                    /*     Logical Maximum (127) */
  0x75, 0x08,                    /*     Report Size (8) */
  0x95, 0x03,                    /*     Report Count (2) AB (3) */
  0x81, 0x06,                    /*     Input (Data, Variable, Relative) */
  0xc0,                          /*   End Collection (Physical) */
  0xc0                           /* End Collection (Application) */
};

const uint16_t hidReportMapLen = sizeof(hidReportMap);

/*! HID Callback prototypes */
static void hidAppOutputCback(dmConnId_t connId, uint8_t id, uint16_t len, uint8_t *pReport);
static void hidAppFeatureCback(dmConnId_t connId, uint8_t id, uint16_t len, uint8_t *pReport);
static void hidAppInfoCback(dmConnId_t connId, uint8_t type, uint8_t value);

/*! HID Profile Configuration */
static const hidConfig_t hidAppHidConfig =
{
  (hidReportIdMap_t*) hidAppReportIdSet,                  /* Report ID to Attribute Handle map */
  sizeof(hidAppReportIdSet)/sizeof(hidReportIdMap_t),     /* Size of Report ID to Attribute Handle map */
  &hidAppOutputCback,                                     /* Output Report Callback */
  &hidAppFeatureCback,                                    /* Feature Report Callback */
  &hidAppInfoCback                                        /* Info Callback */
};

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! application control block */
struct
{
  wsfHandlerId_t handlerId;             /* task handle */

  /* Mouse pending event data */
  uint8_t buttonMask;                   /* pending button mask */
  uint8_t xDisplacement;                /* pending X Displacement */
  uint8_t yDisplacement;                /* pending Y Displacement */
  uint8_t scroll;
  uint8_t devSpecific;                  /* pending Device Specific */

  /* Keyboard pending event data */
  uint8_t modifier;                     /* pending key modifiers */
  uint8_t keys[KEYBOARD_IR_MAX_KEYS];   /* pending keys */

  /* Remote pending event data */
  uint8_t btnData;                      /* pending remote button data */

  uint8_t testBtnState;                 /* identifier of test button action */
  uint8_t reportId;                     /* pending reportId button data */
  uint8_t txFlags;                      /* transmit flags */
  uint8_t protocolMode;                 /* current protocol mode */
  uint8_t hostSuspended;                /* TRUE if host suspended */
} hidAppCb;

/*************************************************************************************************/
/*!
 *  \brief  Application DM callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hidAppDmCback(dmEvt_t *pDmEvt)
{
  dmEvt_t   *pMsg;
  uint16_t  len;

  len = DmSizeOfEvt(pDmEvt);

  if ((pMsg = WsfMsgAlloc(len)) != NULL)
  {
    memcpy(pMsg, pDmEvt, len);
    WsfMsgSend(hidAppCb.handlerId, pMsg);
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
static void hidAppAttCback(attEvt_t *pEvt)
{
  attEvt_t *pMsg;

  if ((pMsg = WsfMsgAlloc(sizeof(attEvt_t) + pEvt->valueLen)) != NULL)
  {
    memcpy(pMsg, pEvt, sizeof(attEvt_t));
    pMsg->pValue = (uint8_t *) (pMsg + 1);
    memcpy(pMsg->pValue, pEvt->pValue, pEvt->valueLen);
    WsfMsgSend(hidAppCb.handlerId, pMsg);
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
static void hidAppCccCback(attsCccEvt_t *pEvt)
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
 *  \brief  Perform UI actions on connection open.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hidAppOpen(dmEvt_t *pMsg)
{

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
static void hidAppSetup(dmEvt_t *pMsg)
{
  /* set advertising and scan response data for discoverable mode */
  AppAdvSetData(APP_ADV_DATA_DISCOVERABLE, sizeof(hidAppAdvDataDisc), (uint8_t *) hidAppAdvDataDisc);
  AppAdvSetData(APP_SCAN_DATA_DISCOVERABLE, sizeof(hidAppScanDataDisc), (uint8_t *) hidAppScanDataDisc);

  /* set advertising and scan response data for connectable mode */
  AppAdvSetData(APP_ADV_DATA_CONNECTABLE, sizeof(hidAppAdvDataDisc), (uint8_t *) hidAppAdvDataDisc);
  AppAdvSetData(APP_SCAN_DATA_CONNECTABLE, sizeof(hidAppScanDataDisc), (uint8_t *) hidAppScanDataDisc);

  /* start advertising; automatically set connectable/discoverable mode and bondable mode */
  AppAdvStart(APP_MODE_AUTO_INIT);
}

/*************************************************************************************************/
/*!
 *  \brief  Send example data.
 *
 *  \param  connId    Connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hidAppMouseSendData(dmConnId_t connId)
{
  uint8_t cccHandle = HIDAPP_IN_MOUSE_CCC_HDL;
  uint8_t protocolMode;

  protocolMode = HidGetProtocolMode();

  if (protocolMode == HID_PROTOCOL_MODE_BOOT)
  {
    cccHandle = HIDAPP_MBI_CCC_HDL;
  }

  if (AttsCccEnabled(connId, cccHandle))
  {
    if (hidAppCb.txFlags & HIDAPP_TX_FLAGS_PENDING)
    {
      if (hidAppCb.txFlags & HIDAPP_TX_FLAGS_READY)
      {
        uint8_t buffer[HIDAPP_MOUSE_INPUT_REPORT_LEN];
        uint8_t reportId = HIDAPP_MOUSE_REPORT_ID;

        /* mouse record: button mask, x displacement, y displacement, device specific */
        buffer[MOUSE_BUTTON_POS] = hidAppCb.buttonMask;
        buffer[MOUSE_X_POS] = hidAppCb.xDisplacement;
        buffer[MOUSE_Y_POS] = hidAppCb.yDisplacement;
        buffer[MOUSE_Y_POS+1] = hidAppCb.scroll;

        hidAppCb.txFlags &= ~(HIDAPP_TX_FLAGS_READY | HIDAPP_TX_FLAGS_PENDING);

        if (protocolMode == HID_PROTOCOL_MODE_BOOT)
        {
          reportId = HID_MOUSE_BOOT_ID;
        }

        /* Send the message */
        HidSendInputReport(connId, reportId, HIDAPP_MOUSE_INPUT_REPORT_LEN, buffer);
      }
    }
  }
  else
  {
    hidAppCb.txFlags &= ~HIDAPP_TX_FLAGS_PENDING;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Send example data.
 *
 *  \param  connId    Connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hidAppkeyboardSendData(dmConnId_t connId)
{
  uint8_t cccHandle = HIDAPP_IN_KEYBOARD_CCC_HDL;
  uint8_t protocolMode;

  protocolMode = HidGetProtocolMode();

  if (protocolMode == HID_PROTOCOL_MODE_BOOT)
  {
    cccHandle = HIDAPP_KBI_CCC_HDL;
  }

  if (AttsCccEnabled(connId, cccHandle))
  {
    if (hidAppCb.txFlags & HIDAPP_TX_FLAGS_PENDING)
    {
      if (hidAppCb.txFlags & HIDAPP_TX_FLAGS_READY)
      {
        uint8_t buffer[HIDAPP_KEYBOARD_INPUT_REPORT_LEN];
        uint8_t reportId = HIDAPP_KEYBOARD_REPORT_ID;

        /* modifier, reserved, keys[6] */
        buffer[KEYBOARD_IR_MODIFIER_POS] = hidAppCb.modifier;
        buffer[KEYBOARD_IR_RESERVED_POS] = 0;
        memcpy(buffer + KEYBOARD_IR_KEY_POS, hidAppCb.keys, sizeof(hidAppCb.keys));

        hidAppCb.txFlags &= ~(HIDAPP_TX_FLAGS_READY | HIDAPP_TX_FLAGS_PENDING);

        if (protocolMode == HID_PROTOCOL_MODE_BOOT)
        {
          reportId = HID_KEYBOARD_BOOT_ID;
        }

        /* Send the message */
        HidSendInputReport(connId, reportId, HIDAPP_KEYBOARD_INPUT_REPORT_LEN, buffer);
      }
    }
  }
  else
  {
    hidAppCb.txFlags &= ~HIDAPP_TX_FLAGS_PENDING;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Send example data.
 *
 *  \param  connId    Connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hidAppRemoteSendData(dmConnId_t connId)
{
  if (AttsCccEnabled(connId, HIDAPP_IN_REMOTE_CCC_HDL))
  {
    if (hidAppCb.txFlags & HIDAPP_TX_FLAGS_PENDING)
    {
      if (hidAppCb.txFlags & HIDAPP_TX_FLAGS_READY)
      {
        uint8_t buffer;

        /* bitmask of remote buttons */
        buffer = hidAppCb.btnData;

        hidAppCb.txFlags &= ~(HIDAPP_TX_FLAGS_READY | HIDAPP_TX_FLAGS_PENDING);

        /* Send the message */
        HidSendInputReport(connId, hidAppCb.reportId, HIDAPP_REMOTE_INPUT_REPORT_LEN, &buffer);
      }
    }
  }
  else
  {
    hidAppCb.txFlags &= ~HIDAPP_TX_FLAGS_PENDING;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Send example data.
 *
 *  \param  connId    Connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hidAppSendData(dmConnId_t connId)
{
  /* Send the report based on report ID */
  switch(hidAppCb.reportId)
  {
  case HIDAPP_REMOTE_REPORT_ID:
    hidAppRemoteSendData(connId);
    break;
  case HIDAPP_KEYBOARD_REPORT_ID:
    hidAppkeyboardSendData(connId);
    break;
  case HIDAPP_MOUSE_REPORT_ID:
    hidAppMouseSendData(connId);
    break;
  default:
    break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Send or queue a remote event to the host
 *
 *  \param  button        Remote control depressed button
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hidAppRemoteReportEvent(uint8_t button)
{
  dmConnId_t connId;

  if ((connId = AppConnIsOpen()) != DM_CONN_ID_NONE)
  {
    /* record key data */
    hidAppCb.btnData = button;
    hidAppCb.reportId = HIDAPP_REMOTE_REPORT_ID;

    /* Indicate new data is pending */
    hidAppCb.txFlags |= HIDAPP_TX_FLAGS_PENDING;

    /* send the data */
    hidAppSendData(connId);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Send or queue a keyboard event to the host
 *
 *  \param  modifiers        Keyboard modifiers.
 *  \param  keys             Keyboard depressed keys.
 *  \param  numKeys          Size of keys parameters in bytes
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hidAppKeyboardReportEvent(uint8_t modifiers, uint8_t keys[], uint8_t numKeys)
{
  dmConnId_t connId;

  if ((connId = AppConnIsOpen()) != DM_CONN_ID_NONE)
  {
    /* max 6 keys in report map */
    if (numKeys > KEYBOARD_IR_MAX_KEYS)
    {
      numKeys = KEYBOARD_IR_MAX_KEYS;
    }

    /* record key data */
    hidAppCb.modifier = modifiers;
    memset(hidAppCb.keys, 0, sizeof(hidAppCb.keys));
    memcpy(hidAppCb.keys, keys, numKeys);
    hidAppCb.reportId = HIDAPP_KEYBOARD_REPORT_ID;

    /* indicate new data is pending */
    hidAppCb.txFlags |= HIDAPP_TX_FLAGS_PENDING;

    /* send the data */
    hidAppSendData(connId);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Report or queue a mouse event to the host.
 *
 *  \param  buttonMask        Mouse button mask.
 *  \param  xDisplacement     Mouse displacement in the x direction.
 *  \param  yDisplacement     Mouse displacement in the y direction.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hidAppMouseReportEvent(uint8_t buttonMask, uint8_t xDisplacement, uint8_t yDisplacement, uint8_t scroll)
{
  dmConnId_t connId;

  if ((connId = AppConnIsOpen()) != DM_CONN_ID_NONE)
  {
    /* record mouse data */
    hidAppCb.buttonMask = buttonMask;
    hidAppCb.xDisplacement = xDisplacement;
    hidAppCb.yDisplacement = yDisplacement;
    hidAppCb.scroll= scroll;
    hidAppCb.devSpecific = 0;
    hidAppCb.reportId = HIDAPP_MOUSE_REPORT_ID;

    /* Indicate new data is pending */
    hidAppCb.txFlags |= HIDAPP_TX_FLAGS_PENDING;

    /* send the data */
    hidAppSendData(connId);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Change the test button.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hidAppTestCycleButton(void)
{
  /* Cycle the test button */
  hidAppCb.testBtnState++;

  if (hidAppCb.testBtnState >= HIDAPP_NUM_TEST_BUTTONS)
  {
    hidAppCb.testBtnState = 0;
  }

  /* Display the active button */
  switch(hidAppCb.testBtnState)
  {
  case HIDAPP_REMOTE_PLAY_PAUSE_BTN:
    APP_TRACE_INFO0("HidApp Test Button: Remote Play/Pause");
    break;
  case HIDAPP_REMOTE_SCAN_NEXT_BTN:
    APP_TRACE_INFO0("HidApp Test Button: Remote Scan Next");
    break;
  case HIDAPP_REMOTE_SCAN_PREVIOUS_BTN:
    APP_TRACE_INFO0("HidApp Test Button: Remote Scan Previous");
    break;
  case HIDAPP_REMOTE_VOLUME_DOWN_BTN:
    APP_TRACE_INFO0("HidApp Test Button: Remote Volume Down");
    break;
  case HIDAPP_REMOTE_VOLUME_UP_BTN:
    APP_TRACE_INFO0("HidApp Test Button: Remote Volume Up");
    break;
  case HIDAPP_MOUSE_LEFT_BTN:
    APP_TRACE_INFO0("HidApp Test Button: Mouse Left");
    break;
  case HIDAPP_MOUSE_RIGHT_BTN:
    APP_TRACE_INFO0("HidApp Test Button: Mouse Right");
    break;
  case HIDAPP_KEYBOARD_UP_BTN:
    APP_TRACE_INFO0("HidApp Test Button: Keyboard Up");
    break;
  case HIDAPP_KEYBOARD_DOWN_BTN:
    APP_TRACE_INFO0("HidApp Test Button: Keyboard Down");
    break;
  case HIDAPP_KEYBOARD_LEFT_BTN:
    APP_TRACE_INFO0("HidApp Test Button: Keyboard Left");
    break;
  case HIDAPP_KEYBOARD_RIGHT_BTN:
    APP_TRACE_INFO0("HidApp Test Button: Keyboard Right");
    break;
  default:
    break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Send test button press.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hidAppTestSendButton(void)
{
  uint8_t button;

  /* Send play command */
  switch(hidAppCb.testBtnState)
  {
  case HIDAPP_REMOTE_PLAY_PAUSE_BTN:
    button = REMOTE_PLAY_PAUSE;
    hidAppRemoteReportEvent(button);
    break;
  case HIDAPP_REMOTE_SCAN_NEXT_BTN:
    button = REMOTE_SCAN_NEXT;
    hidAppRemoteReportEvent(button);
    break;
  case HIDAPP_REMOTE_SCAN_PREVIOUS_BTN:
    button = REMOTE_SCAN_PREVIOUS;
    hidAppRemoteReportEvent(button);
    break;
  case HIDAPP_REMOTE_VOLUME_DOWN_BTN:
    button = REMOTE_VOLUME_DOWN;
    hidAppRemoteReportEvent(button);
    break;
  case HIDAPP_REMOTE_VOLUME_UP_BTN:
    button = REMOTE_VOLUME_UP;
    hidAppRemoteReportEvent(button);
    break;
  case HIDAPP_MOUSE_LEFT_BTN:
    hidAppMouseReportEvent(MOUSE_BUTTON_LEFT, 0, 0, 20);
    break;
  case HIDAPP_MOUSE_RIGHT_BTN:
    hidAppMouseReportEvent(0, 0, 0, (uint8_t)-20);
    break;
  case HIDAPP_KEYBOARD_UP_BTN:
    button = KEYBOARD_USAGE_UP_ARROW;
    hidAppKeyboardReportEvent(0, &button, 1);
    break;
  case HIDAPP_KEYBOARD_DOWN_BTN:
    button = KEYBOARD_USAGE_DOWN_ARROW;
    hidAppKeyboardReportEvent(0, &button, 1);
    break;
  case HIDAPP_KEYBOARD_LEFT_BTN:
    button = KEYBOARD_USAGE_LEFT_ARROW;
    hidAppKeyboardReportEvent(0, &button, 1);
    break;
  case HIDAPP_KEYBOARD_RIGHT_BTN:
    button = KEYBOARD_USAGE_RIGHT_ARROW;
    hidAppKeyboardReportEvent(0, &button, 1);
    break;
  default:
    return;
  }

  APP_TRACE_INFO0("HidApp: Test Button Sent");
}

/*************************************************************************************************/
/*!
 *  \brief  Send the corresponding no button.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hidAppTestNoButton(void)
{
  uint8_t button;

  /* Send play command */
  switch(hidAppCb.testBtnState)
  {
  case HIDAPP_REMOTE_PLAY_PAUSE_BTN:
  case HIDAPP_REMOTE_SCAN_NEXT_BTN:
  case HIDAPP_REMOTE_SCAN_PREVIOUS_BTN:
  case HIDAPP_REMOTE_VOLUME_DOWN_BTN:
  case HIDAPP_REMOTE_VOLUME_UP_BTN:
    button = REMOTE_USAGE_NONE;
    hidAppRemoteReportEvent(button);
    break;

  case HIDAPP_MOUSE_LEFT_BTN:
  case HIDAPP_MOUSE_RIGHT_BTN:
    hidAppMouseReportEvent(MOUSE_USAGE_NONE, 0, 0, 0);
    break;

  case HIDAPP_KEYBOARD_UP_BTN:
  case HIDAPP_KEYBOARD_DOWN_BTN:
  case HIDAPP_KEYBOARD_LEFT_BTN:
  case HIDAPP_KEYBOARD_RIGHT_BTN:
    button = KEYBOARD_USAGE_NONE;
    hidAppKeyboardReportEvent(0, &button, 1);
    break;
  default:
    return;
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
static void hidAppBtnCback(uint8_t btn)
{
  dmConnId_t connId = AppConnIsOpen();

  /* button actions when connected */
  if (connId != DM_CONN_ID_NONE)
  {
    switch (btn)
    {
      case APP_UI_BTN_1_SHORT:
        /* Send test command */
        hidAppTestSendButton();
        hidAppTestNoButton();

        break;

      case APP_UI_BTN_2_SHORT:
        /* Change test button */
        hidAppTestCycleButton();
        break;

      case APP_UI_BTN_1_MED:
      case APP_UI_BTN_1_LONG:
      case APP_UI_BTN_1_EX_LONG:
      case APP_UI_BTN_2_MED:
      case APP_UI_BTN_2_LONG:
      case APP_UI_BTN_2_EX_LONG:
        /* Send no button */
        hidAppTestNoButton();
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
        /* clear all bonding info */
        AppSlaveClearAllBondingInfo();

        /* restart advertising */
        AppAdvStart(APP_MODE_AUTO_INIT);
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
static void hidAppProcMsg(dmEvt_t *pMsg)
{
  uint8_t uiEvent = APP_UI_NONE;

  switch(pMsg->hdr.event)
  {
    case ATTS_HANDLE_VALUE_CNF:
      if (pMsg->hdr.status == ATT_SUCCESS)
      {
        hidAppCb.txFlags |= HIDAPP_TX_FLAGS_READY;
        hidAppSendData((dmConnId_t) pMsg->hdr.param);
      }
      break;

    case HIDAPP_BATT_TIMER_IND:
      BasProcMsg(&pMsg->hdr);
      break;

    case ATT_MTU_UPDATE_IND:
      APP_TRACE_INFO1("Negotiated MTU %d", ((attEvt_t *)pMsg)->mtu);
      break;

    case DM_RESET_CMPL_IND:
      // set database hash calculating status to true until a new hash is generated after reset
      attsCsfSetHashUpdateStatus(TRUE);

      // Generate ECC key if configured support secure connection,
      // else will calcualte ATT database hash
      if( hidAppSecCfg.auth & DM_AUTH_SC_FLAG )
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
      hidAppSetup(pMsg);
      break;

    case DM_CONN_OPEN_IND:
      hidAppOpen(pMsg);
      uiEvent = APP_UI_CONN_OPEN;
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
      AppHandlePasskey(&pMsg->authReq);
      break;

     case DM_SEC_ECC_KEY_IND:
      DmSecSetEccKey(&pMsg->eccMsg.data.key);
      // Only calculate database hash if the calculating status is in progress
      if( attsCsfGetHashUpdateStatus() )
      {
        AttsCalculateDbHash();
      }
      break;

    case DM_SEC_COMPARE_IND:
      AppHandleNumericComparison(&pMsg->cnfInd);
      break;

    case DM_PRIV_CLEAR_RES_LIST_IND:
      APP_TRACE_INFO1("Clear resolving list status 0x%02x", pMsg->hdr.status);
      break;

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
 *  \brief  Callback to handle an output report from the host.
 *
 *  \param  connId    The connection identifier.
 *  \param  id        The ID of the report.
 *  \param  len       The length of the report data in pReport.
 *  \param  pReport   A buffer containing the report.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hidAppOutputCback(dmConnId_t connId, uint8_t id, uint16_t len, uint8_t *pReport)
{
}

/*************************************************************************************************/
/*!
 *  \brief  Callback to handle a feature report from the host.
 *
 *  \param  connId    The connection identifier.
 *  \param  id        The ID of the report.
 *  \param  len       The length of the report data in pReport.
 *  \param  pReport   A buffer containing the report.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hidAppFeatureCback(dmConnId_t connId, uint8_t id, uint16_t len, uint8_t *pReport)
{
}

/*************************************************************************************************/
/*!
 *  \brief  Callback to handle a change in protocol mode or control point from the host.
 *
 *  \param  connId    The connection identifier.
 *  \param  mode      The type of information (HID_INFO_CONTROL_POINT or HID_INFO_PROTOCOL_MODE)
 *  \param  value     The value of the information
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hidAppInfoCback(dmConnId_t connId, uint8_t type, uint8_t value)
{
  if (type == HID_INFO_PROTOCOL_MODE)
  {
    hidAppCb.protocolMode = value;
  }
  else if (type == HID_INFO_CONTROL_POINT)
  {
    hidAppCb.hostSuspended = (value == HID_CONTROL_POINT_SUSPEND) ? TRUE : FALSE;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize the report attributes to default values for the hidApp.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hidAppReportInit(void)
{
  uint8_t iKeyboardBuffer[HIDAPP_KEYBOARD_INPUT_REPORT_LEN];
  uint8_t iMouseBuffer[HIDAPP_MOUSE_INPUT_REPORT_LEN];
  uint8_t iRemoteBuffer[HIDAPP_REMOTE_INPUT_REPORT_LEN];
  uint8_t oBuffer[HIDAPP_OUTPUT_REPORT_LEN];
  uint8_t fBuffer[HIDAPP_FEATURE_REPORT_LEN];

  /* Remote Input report */
  memset(iRemoteBuffer, 0, HIDAPP_REMOTE_INPUT_REPORT_LEN);
  AttsSetAttr(HID_INPUT_REPORT_1_HDL, HIDAPP_REMOTE_INPUT_REPORT_LEN, iRemoteBuffer);

  /* Keyboard Input report */
  memset(iKeyboardBuffer, 0, HIDAPP_KEYBOARD_INPUT_REPORT_LEN);
  AttsSetAttr(HID_INPUT_REPORT_2_HDL, HIDAPP_KEYBOARD_INPUT_REPORT_LEN, iKeyboardBuffer);

  /* Mouse Input report */
  memset(iMouseBuffer, 0, HIDAPP_MOUSE_INPUT_REPORT_LEN);
  AttsSetAttr(HID_INPUT_REPORT_3_HDL, HIDAPP_MOUSE_INPUT_REPORT_LEN, iMouseBuffer);

  /* Output report */
  memset(oBuffer, 0, HIDAPP_OUTPUT_REPORT_LEN);
  AttsSetAttr(HID_OUTPUT_REPORT_HDL, HIDAPP_OUTPUT_REPORT_LEN, oBuffer);

  /* Feature report */
  memset(fBuffer, 0, HIDAPP_FEATURE_REPORT_LEN);
  AttsSetAttr(HID_FEATURE_REPORT_HDL, HIDAPP_FEATURE_REPORT_LEN, fBuffer);
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
void HidAppHandlerInit(wsfHandlerId_t handlerId)
{
  APP_TRACE_INFO0("HidAppHandlerInit");

  /* Initialize the control block */
  memset(&hidAppCb, 0, sizeof(hidAppCb));
  hidAppCb.txFlags = HIDAPP_TX_FLAGS_READY;

  /* store handler ID */
  hidAppCb.handlerId = handlerId;

  /* Set configuration pointers */
  pAppSlaveCfg = (appSlaveCfg_t *) &hidAppSlaveCfg;
  pAppAdvCfg = (appAdvCfg_t *) &hidAppAdvCfg;
  pAppSecCfg = (appSecCfg_t *) &hidAppSecCfg;
  pAppUpdateCfg = (appUpdateCfg_t *) &hidAppUpdateCfg;

  /* Initialize application framework */
  AppSlaveInit();
  AppServerInit();

  /* initialize battery service server */
  BasInit(handlerId, (basCfg_t *) &hidAppBasCfg);
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
void HidAppHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
  if (pMsg != NULL)
  {
    APP_TRACE_INFO1("HidApp got evt %d", pMsg->event);

    /* process ATT messages */
    if (pMsg->event >= ATT_CBACK_START && pMsg->event <= ATT_CBACK_END)
    {
      /* process server-related ATT messages */
      AppServerProcAttMsg(pMsg);
    }
    /* process DM messages */
    else if (pMsg->event <= DM_CBACK_END)
    {
      /* process advertising and connection-related messages */
      AppSlaveProcDmMsg((dmEvt_t *) pMsg);

      /* process security-related messages */
      AppSlaveSecProcDmMsg((dmEvt_t *) pMsg);
    }

    /* perform profile and user interface-related operations */
    hidAppProcMsg((dmEvt_t *) pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Test a hidApp button event.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HidAppTest(void)
{
  hidAppBtnCback(APP_UI_BTN_1_DOWN);
  hidAppBtnCback(APP_UI_BTN_1_SHORT);
}

/*************************************************************************************************/
/*!
 *  \brief  Start the application.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HidAppStart(void)
{
#ifdef HID_ATT_DYNAMIC
  void *pSHdl;
#endif /* HID_ATT_DYNAMIC */

  /* Register for stack callbacks */
  DmRegister(hidAppDmCback);
  DmConnRegister(DM_CLIENT_ID_APP, hidAppDmCback);
  AttRegister(hidAppAttCback);
  AttConnRegister(AppServerConnCback);
  AttsCccRegister(HIDAPP_NUM_CCC_IDX, (attsCccSet_t *) hidAppCccSet, hidAppCccCback);

  /* Register for app framework button callbacks */
  AppUiBtnRegister(hidAppBtnCback);

  /* Initialize attribute server database */
  SvcCoreGattCbackRegister(GattReadCback, GattWriteCback);
  SvcCoreAddGroup();
  SvcDisAddGroup();

#ifdef HID_ATT_DYNAMIC

  /* Initialize the dynamic service system */
  AttsDynInit();

  /* Add the HID service dynamically */
  pSHdl = SvcHidAddGroupDyn();
  AttsDynRegister(pSHdl, NULL, HidAttsWriteCback);

  /* Add the Battery service dynamically */
  pSHdl = SvcBattAddGroupDyn();
  AttsDynRegister(pSHdl, BasReadCback, NULL);

#else

  /* Add the HID service statically */
  SvcHidAddGroup();
  SvcHidRegister(HidAttsWriteCback, NULL);

  /* Add the Battery service statically */
  SvcBattAddGroup();
  SvcBattCbackRegister(BasReadCback, NULL);

#endif /* HID_ATT_DYNAMIC */

  /* Set Service Changed CCCD index. */
  GattSetSvcChangedIdx(HIDAPP_GATT_SC_CCC_IDX);

  /* Initialize the HID profile */
  HidInit(&hidAppHidConfig);

  /* Initialize the report attributes */
  hidAppReportInit();

  /* Reset the device */
  DmDevReset();
}
