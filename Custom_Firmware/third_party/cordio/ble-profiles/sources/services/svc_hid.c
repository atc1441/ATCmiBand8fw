/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Human Interface Device service implementation.
 *
 *  Copyright (c) 2015-2018 Arm Ltd. All Rights Reserved.
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
#include "att_api.h"
#include "svc_hid.h"
#include "wsf_trace.h"
#include "svc_ch.h"
#include "svc_cfg.h"
#include "svc_batt.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions */
#ifndef HID_SEC_PERMIT_READ
#define HID_SEC_PERMIT_READ (ATTS_PERMIT_READ | ATTS_PERMIT_READ_ENC)
#endif

/*! Characteristic write permissions */
#ifndef HID_SEC_PERMIT_WRITE
#define HID_SEC_PERMIT_WRITE (ATTS_PERMIT_WRITE | ATTS_PERMIT_WRITE_ENC)
#endif

/**************************************************************************************************
 Static Variables
**************************************************************************************************/

/* UUIDs */

/**************************************************************************************************
 Service variables
**************************************************************************************************/

/* Proprietary Service Declaration */
static const uint8_t hidValSvc[] = {UINT16_TO_BYTES(ATT_UUID_HID_SERVICE)};
static const uint16_t hidLenSvc = sizeof(hidValSvc);


/* HID Info Characteristic */
static const uint8_t hidInfoCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(HID_INFO_HDL), UINT16_TO_BYTES(ATT_UUID_HID_INFORMATION)};
static const uint16_t hidLenInfoCh = sizeof(hidInfoCh);

/* HID Info Value: HID Spec version, country code, flags */
static const uint8_t hidInfoVal[] = {UINT16_TO_BYTES(HID_VERSION), 0x00, 0x00};
static const uint16_t hidLenInfoVal = sizeof(hidInfoVal);


/* HID Report Map Characteristic */
static const uint8_t hidRmCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(HID_REPORT_MAP_HDL), UINT16_TO_BYTES(ATT_UUID_HID_REPORT_MAP)};
static const uint16_t hidLenRmCh = sizeof(hidRmCh);

/* HID Report Map Value */
/* Note: The hidReportMap and hidReportMapLen variables should be defined in the application */
extern const uint8_t hidReportMap[];
extern const uint16_t hidReportMapLen;

/* HID External Report Reference Descriptor */
static const uint8_t hidExtReport[] = {UINT16_TO_BYTES(ATT_UUID_BATTERY_LEVEL)};
static const uint16_t hidLenExtReport = sizeof(hidExtReport);


/* HID Control Point Characteristic */
static const uint8_t hidCpCh[] = {ATT_PROP_WRITE_NO_RSP, UINT16_TO_BYTES(HID_CONTROL_POINT_HDL), UINT16_TO_BYTES(ATT_UUID_HID_CONTROL_POINT)};
static const uint16_t hidLenCpCh = sizeof(hidCpCh);

/* HID Control Point Value */
static uint8_t hidCpVal[] = {0};
static const uint16_t hidLenCpVal = sizeof(hidCpVal);


/* HID Boot Keyboard In Characteristic */
static const uint8_t hidBkiCh[] = {ATT_PROP_READ | ATT_PROP_NOTIFY, UINT16_TO_BYTES(HID_KEYBOARD_BOOT_IN_HDL), UINT16_TO_BYTES(ATT_UUID_HID_BOOT_KEYBOARD_IN)};
static const uint16_t hidLenBkiCh = sizeof(hidBkiCh);

/* HID Boot Keyboard In Value */
static uint8_t hidBkiVal[HID_MAX_REPORT_LEN];
static uint16_t hidLenBkiVal = sizeof(hidBkiVal);

/* HID Boot Keyboard In client characteristic configuration */
static uint8_t hidValBkiChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t hidLenBkiChCcc = sizeof(hidValBkiChCcc);


/* HID Boot Keyboard Out Characteristic */
static const uint8_t hidBkoCh[] = {ATT_PROP_READ | ATT_PROP_WRITE | ATT_PROP_WRITE_NO_RSP, UINT16_TO_BYTES(HID_KEYBOARD_BOOT_OUT_HDL), UINT16_TO_BYTES(ATT_UUID_HID_BOOT_KEYBOARD_OUT)};
static const uint16_t hidLenBkoCh = sizeof(hidBkoCh);

/* HID Boot Keyboard Out Value */
static uint8_t hidBkoVal[HID_MAX_REPORT_LEN];
static uint16_t hidLenBkoVal = sizeof(hidBkoVal);


/* HID Boot Mouse In Characteristic */
static const uint8_t hidBmiCh[] = {ATT_PROP_READ | ATT_PROP_NOTIFY, UINT16_TO_BYTES(HID_MOUSE_BOOT_IN_HDL), UINT16_TO_BYTES(ATT_UUID_HID_BOOT_MOUSE_IN)};
static const uint16_t hidLenBmiCh = sizeof(hidBmiCh);

/* HID Boot Mouse In Value */
static uint8_t hidBmiVal[HID_MAX_REPORT_LEN];
static uint16_t hidLenBmiVal = sizeof(hidBmiVal);

/* HID Boot Mouse In client characteristic configuration */
static uint8_t hidValBmiChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t hidLenBmiChCcc = sizeof(hidValBmiChCcc);


/* HID Input Report #1 Characteristic */
static const uint8_t hidIRep1Ch[] = {ATT_PROP_READ | ATT_PROP_NOTIFY, UINT16_TO_BYTES(HID_INPUT_REPORT_1_HDL), UINT16_TO_BYTES(ATT_UUID_HID_REPORT)};
static const uint16_t hidLenIRep1Ch = sizeof(hidIRep1Ch);

/* HID Input Report Value */
static uint8_t hidIRep1Val[HID_MAX_REPORT_LEN];
static uint16_t hidLenIRep1Val = sizeof(hidIRep1Val);

/* HID Input Report client characteristic configuration */
static uint8_t hidValIRep1ChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t hidLenIRep1ChCcc = sizeof(hidValIRep1ChCcc);

/* HID Input Report Reference - ID, Type */
static const uint8_t hidValIRep1IdMap[] = {0x01, HID_REPORT_TYPE_INPUT};
static const uint16_t hidLenIRep1IdMap = sizeof(hidValIRep1IdMap);


/* HID Input Report #2 Characteristic */
static const uint8_t hidIRep2Ch[] = {ATT_PROP_READ | ATT_PROP_NOTIFY, UINT16_TO_BYTES(HID_INPUT_REPORT_2_HDL), UINT16_TO_BYTES(ATT_UUID_HID_REPORT)};
static const uint16_t hidLenIRep2Ch = sizeof(hidIRep2Ch);

/* HID Input Report Value */
static uint8_t hidIRep2Val[HID_MAX_REPORT_LEN];
static uint16_t hidLenIRep2Val = sizeof(hidIRep2Val);

/* HID Input Report client characteristic configuration */
static uint8_t hidValIRep2ChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t hidLenIRep2ChCcc = sizeof(hidValIRep2ChCcc);

/* HID Input Report Reference - ID, Type */
static const uint8_t hidValIRep2IdMap[] = {0x02, HID_REPORT_TYPE_INPUT};
static const uint16_t hidLenIRep2IdMap = sizeof(hidValIRep2IdMap);


/* HID Input Report #3 Characteristic */
static const uint8_t hidIRep3Ch[] = {ATT_PROP_READ | ATT_PROP_NOTIFY, UINT16_TO_BYTES(HID_INPUT_REPORT_3_HDL), UINT16_TO_BYTES(ATT_UUID_HID_REPORT)};
static const uint16_t hidLenIRep3Ch = sizeof(hidIRep3Ch);

/* HID Input Report Value */
static uint8_t hidIRep3Val[HID_MAX_REPORT_LEN];
static uint16_t hidLenIRep3Val = sizeof(hidIRep3Val);

/* HID Input Report client characteristic configuration */
static uint8_t hidValIRep3ChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t hidLenIRep3ChCcc = sizeof(hidValIRep3ChCcc);

/* HID Input Report Reference - ID, Type */
static const uint8_t hidValIRep3IdMap[] = {0x03, HID_REPORT_TYPE_INPUT};
static const uint16_t hidLenIRep3IdMap = sizeof(hidValIRep3IdMap);


/* HID Output Report Characteristic */
static const uint8_t hidORepCh[] = {ATT_PROP_READ | ATT_PROP_WRITE | ATT_PROP_WRITE_NO_RSP, UINT16_TO_BYTES(HID_OUTPUT_REPORT_HDL), UINT16_TO_BYTES(ATT_UUID_HID_REPORT)};
static const uint16_t hidLenORepCh = sizeof(hidORepCh);

/* HID Output Report Value */
static uint8_t hidORepVal[HID_MAX_REPORT_LEN];
static uint16_t hidLenORepVal = sizeof(hidORepVal);

/* HID Output Report Reference - ID, Type */
static const uint8_t hidValORepIdMap[] = {0x00, HID_REPORT_TYPE_OUTPUT};
static const uint16_t hidLenORepIdMap = sizeof(hidValORepIdMap);


/* HID Feature Report Characteristic */
static const uint8_t hidFRepCh[] = {ATT_PROP_READ | ATT_PROP_WRITE, UINT16_TO_BYTES(HID_FEATURE_REPORT_HDL), UINT16_TO_BYTES(ATT_UUID_HID_REPORT)};
static const uint16_t hidLenFRepCh = sizeof(hidFRepCh);

/* HID Feature Report Value */
static uint8_t hidFRepVal[HID_MAX_REPORT_LEN];
static uint16_t hidLenFRepVal = sizeof(hidFRepVal);

/* HID Feature Report Reference - ID, Type */
static const uint8_t hidValFRepIdMap[] = {0x00, HID_REPORT_TYPE_FEATURE};
static const uint16_t hidLenFRepIdMap = sizeof(hidValFRepIdMap);


/* HID Protocol Mode Characteristic */
static const uint8_t hidPmCh[] = {ATT_PROP_READ | ATT_PROP_WRITE_NO_RSP, UINT16_TO_BYTES(HID_PROTOCOL_MODE_HDL), UINT16_TO_BYTES(ATT_UUID_HID_PROTOCOL_MODE)};
static const uint16_t hidLenPmCh = sizeof(hidPmCh);

/* HID Protocol Mode Value */
static uint8_t hidPmVal[] = {HID_PROTOCOL_MODE_REPORT};
static const uint16_t hidLenPmVal = sizeof(hidPmVal);


/* Attribute list for HID group */
static const attsAttr_t hidList[] =
{
  /* Service Delcaration */
  {
    attPrimSvcUuid,
    (uint8_t *) hidValSvc,
    (uint16_t *) &hidLenSvc,
    sizeof(hidValSvc),
    0,
    ATTS_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidInfoCh,
    (uint16_t *) &hidLenInfoCh,
    sizeof(hidInfoCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidiChUuid,
    (uint8_t *) hidInfoVal,
    (uint16_t *) &hidLenInfoVal,
    sizeof(hidInfoVal),
    0,
    HID_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidRmCh,
    (uint16_t *) &hidLenRmCh,
    sizeof(hidRmCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRmChUuid,
    (uint8_t *) hidReportMap,
    (uint16_t *) &hidReportMapLen,
    HID_MAX_REPORT_MAP_LEN,
    ATTS_SET_VARIABLE_LEN,
    HID_SEC_PERMIT_READ
  },
  {
    attHidErmUuid,
    (uint8_t *) hidExtReport,
    (uint16_t *) &hidLenExtReport,
    sizeof(hidExtReport),
    0,
    HID_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidCpCh,
    (uint16_t *) &hidLenCpCh,
    sizeof(hidCpCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRepChUuid,
    (uint8_t *) hidCpVal,
    (uint16_t *) &hidLenCpVal,
    sizeof(hidCpVal),
    ATTS_SET_WRITE_CBACK,
    HID_SEC_PERMIT_WRITE
  },

  {
    attChUuid,
    (uint8_t *) hidBkiCh,
    (uint16_t *) &hidLenBkiCh,
    sizeof(hidBkiCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidBkiChUuid,
    (uint8_t *) hidBkiVal,
    (uint16_t *) &hidLenBkiVal,
    sizeof(hidBkiVal),
    ATTS_SET_VARIABLE_LEN,
    HID_SEC_PERMIT_READ
  },
  {
    attCliChCfgUuid,
    (uint8_t *) hidValBkiChCcc,
    (uint16_t *) &hidLenBkiChCcc,
    sizeof(hidValBkiChCcc),
    ATTS_SET_CCC,
    (HID_SEC_PERMIT_READ | HID_SEC_PERMIT_WRITE)
  },

  {
    attChUuid,
    (uint8_t *) hidBkoCh,
    (uint16_t *) &hidLenBkoCh,
    sizeof(hidBkoCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidBkoChUuid,
    (uint8_t *) hidBkoVal,
    (uint16_t *) &hidLenBkoVal,
    sizeof(hidBkoVal),
    (ATTS_SET_WRITE_CBACK | ATTS_SET_VARIABLE_LEN),
    (HID_SEC_PERMIT_READ | HID_SEC_PERMIT_WRITE)
  },

  {
    attChUuid,
    (uint8_t *) hidBmiCh,
    (uint16_t *) &hidLenBmiCh,
    sizeof(hidBmiCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidBmiChUuid,
    (uint8_t *) hidBmiVal,
    (uint16_t *) &hidLenBmiVal,
    sizeof(hidBmiVal),
    ATTS_SET_VARIABLE_LEN,
    HID_SEC_PERMIT_READ
  },
  {
    attCliChCfgUuid,
    (uint8_t *) hidValBmiChCcc,
    (uint16_t *) &hidLenBmiChCcc,
    sizeof(hidValBmiChCcc),
    ATTS_SET_CCC,
    (HID_SEC_PERMIT_READ | HID_SEC_PERMIT_WRITE)
  },

  {
    attChUuid,
    (uint8_t *) hidIRep1Ch,
    (uint16_t *) &hidLenIRep1Ch,
    sizeof(hidIRep1Ch),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRepChUuid,
    (uint8_t *) hidIRep1Val,
    (uint16_t *) &hidLenIRep1Val,
    sizeof(hidIRep1Val),
    ATTS_SET_VARIABLE_LEN,
    HID_SEC_PERMIT_READ
  },
  {
    attCliChCfgUuid,
    (uint8_t *) hidValIRep1ChCcc,
    (uint16_t *) &hidLenIRep1ChCcc,
    sizeof(hidValIRep1ChCcc),
    ATTS_SET_CCC,
    (HID_SEC_PERMIT_READ | HID_SEC_PERMIT_WRITE)
  },
  {
    attHidRimUuid,
    (uint8_t *) hidValIRep1IdMap,
    (uint16_t *) &hidLenIRep1IdMap,
    sizeof(hidValIRep1IdMap),
    0,
    HID_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidIRep2Ch,
    (uint16_t *) &hidLenIRep2Ch,
    sizeof(hidIRep2Ch),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRepChUuid,
    (uint8_t *) hidIRep2Val,
    (uint16_t *) &hidLenIRep2Val,
    sizeof(hidIRep2Val),
    ATTS_SET_VARIABLE_LEN,
    HID_SEC_PERMIT_READ
  },
  {
    attCliChCfgUuid,
    (uint8_t *) hidValIRep2ChCcc,
    (uint16_t *) &hidLenIRep2ChCcc,
    sizeof(hidValIRep2ChCcc),
    ATTS_SET_CCC,
    (HID_SEC_PERMIT_READ | HID_SEC_PERMIT_WRITE)
  },
  {
    attHidRimUuid,
    (uint8_t *) hidValIRep2IdMap,
    (uint16_t *) &hidLenIRep2IdMap,
    sizeof(hidValIRep2IdMap),
    0,
    HID_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidIRep3Ch,
    (uint16_t *) &hidLenIRep3Ch,
    sizeof(hidIRep3Ch),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRepChUuid,
    (uint8_t *) hidIRep3Val,
    (uint16_t *) &hidLenIRep3Val,
    sizeof(hidIRep3Val),
    ATTS_SET_VARIABLE_LEN,
    HID_SEC_PERMIT_READ
  },
  {
    attCliChCfgUuid,
    (uint8_t *) hidValIRep3ChCcc,
    (uint16_t *) &hidLenIRep3ChCcc,
    sizeof(hidValIRep3ChCcc),
    ATTS_SET_CCC,
    (HID_SEC_PERMIT_READ | HID_SEC_PERMIT_WRITE)
  },
  {
    attHidRimUuid,
    (uint8_t *) hidValIRep3IdMap,
    (uint16_t *) &hidLenIRep3IdMap,
    sizeof(hidValIRep3IdMap),
    0,
    HID_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidORepCh,
    (uint16_t *) &hidLenORepCh,
    sizeof(hidORepCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRepChUuid,
    (uint8_t *) hidORepVal,
    (uint16_t *) &hidLenORepVal,
    sizeof(hidORepVal),
    (ATTS_SET_WRITE_CBACK | ATTS_SET_VARIABLE_LEN),
    (HID_SEC_PERMIT_READ | HID_SEC_PERMIT_WRITE)
  },
  {
    attHidRimUuid,
    (uint8_t *) hidValORepIdMap,
    (uint16_t *) &hidLenORepIdMap,
    sizeof(hidValORepIdMap),
    0,
    HID_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidFRepCh,
    (uint16_t *) &hidLenFRepCh,
    sizeof(hidFRepCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRepChUuid,
    (uint8_t *) hidFRepVal,
    (uint16_t *) &hidLenFRepVal,
    sizeof(hidFRepVal),
    (ATTS_SET_WRITE_CBACK | ATTS_SET_VARIABLE_LEN),
    (HID_SEC_PERMIT_READ | HID_SEC_PERMIT_WRITE)
  },
  {
    attHidRimUuid,
    (uint8_t *) hidValFRepIdMap,
    (uint16_t *) &hidLenFRepIdMap,
    sizeof(hidValFRepIdMap),
    0,
    HID_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidPmCh,
    (uint16_t *) &hidLenPmCh,
    sizeof(hidPmCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRepChUuid,
    (uint8_t *) hidPmVal,
    (uint16_t *) &hidLenPmVal,
    sizeof(hidPmVal),
    ATTS_SET_WRITE_CBACK,
    (HID_SEC_PERMIT_READ | HID_SEC_PERMIT_WRITE)
  }
};

/* HID group structure */
static attsGroup_t svcHidkbGroup =
{
  NULL,
  (attsAttr_t *) hidList,
  NULL,
  NULL,
  HID_START_HDL,
  HID_END_HDL
};

/*************************************************************************************************/
/*!
 *  \brief  Add the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidAddGroup(void)
{
  AttsAddGroup(&svcHidkbGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidRemoveGroup(void)
{
  AttsRemoveGroup(HID_START_HDL);
}

/*************************************************************************************************/
/*!
 *  \brief  Register a read and write callback functions for the ATT Group.
 *
 *  \param  writeCb   Write callback function
 *  \param  readCb    Read callback function
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidRegister(attsWriteCback_t writeCb, attsReadCback_t readCb)
{
  svcHidkbGroup.writeCback = writeCb;
  svcHidkbGroup.readCback = readCb;
}

/*************************************************************************************************/
/*!
 *  \brief  Add the Hid Service using the dynamic attribute subsystem.
 *
 *  \return None.
 */
/*************************************************************************************************/
void *SvcHidAddGroupDyn()
{
  void *pSHdl;
  uint8_t initCcc[] = {UINT16_TO_BYTES(0x0000)};

  /* Create the service */
  pSHdl = AttsDynCreateGroup(HID_START_HDL, HID_END_HDL);

  if (pSHdl != NULL)
  {
    /* Primary service */
    AttsDynAddAttrConst(pSHdl, attPrimSvcUuid, hidValSvc, sizeof(hidValSvc), 0, ATTS_PERMIT_READ);

    /* HID Info */
    AttsDynAddAttrConst(pSHdl, attChUuid, hidInfoCh, sizeof(hidInfoCh), 0, ATTS_PERMIT_READ);
    AttsDynAddAttrConst(pSHdl, attHidiChUuid, hidInfoVal, sizeof(hidInfoVal), 0, HID_SEC_PERMIT_READ);

    /* HID Report Map */
    AttsDynAddAttrConst(pSHdl, attChUuid, hidRmCh, sizeof(hidRmCh), 0, ATTS_PERMIT_READ);
    AttsDynAddAttrConst(pSHdl, attHidRmChUuid, hidReportMap, hidReportMapLen, 0, HID_SEC_PERMIT_READ);

    /* HID External Report Reference */
    AttsDynAddAttrConst(pSHdl, attHidErmUuid, hidExtReport, sizeof(hidExtReport), 0, HID_SEC_PERMIT_READ);

    /* HID Control Point */
    AttsDynAddAttrConst(pSHdl, attChUuid, hidCpCh, sizeof(hidCpCh), 0, ATTS_PERMIT_READ);
    AttsDynAddAttr(pSHdl, attHidRepChUuid, NULL, 1, 1, ATTS_SET_WRITE_CBACK, HID_SEC_PERMIT_WRITE);

    /* HID Boot Keyboard In */
    AttsDynAddAttrConst(pSHdl, attChUuid, hidBkiCh, sizeof(hidBkiCh), 0, ATTS_PERMIT_READ);
    AttsDynAddAttr(pSHdl, attHidBkiChUuid, NULL, HID_MAX_REPORT_LEN, HID_MAX_REPORT_LEN,
                   ATTS_SET_VARIABLE_LEN, HID_SEC_PERMIT_READ);
    AttsDynAddAttr(pSHdl, attCliChCfgUuid, initCcc, sizeof(uint16_t), sizeof(uint16_t),
                   ATTS_SET_CCC, HID_SEC_PERMIT_READ | HID_SEC_PERMIT_WRITE);

    /* HID Boot Keyboard Out */
    AttsDynAddAttrConst(pSHdl, attChUuid, hidBkoCh, sizeof(hidBkoCh), 0, ATTS_PERMIT_READ);
    AttsDynAddAttr(pSHdl, attHidBkoChUuid, NULL, HID_MAX_REPORT_LEN, HID_MAX_REPORT_LEN,
        ATTS_SET_WRITE_CBACK | ATTS_SET_VARIABLE_LEN, HID_SEC_PERMIT_READ | HID_SEC_PERMIT_WRITE);

    /* HID Boot Mouse In */
    AttsDynAddAttrConst(pSHdl, attChUuid, hidBmiCh, sizeof(hidBmiCh), 0, ATTS_PERMIT_READ);
    AttsDynAddAttr(pSHdl, attHidBmiChUuid, NULL, HID_MAX_REPORT_LEN, HID_MAX_REPORT_LEN,
                   ATTS_SET_VARIABLE_LEN, HID_SEC_PERMIT_READ);
    AttsDynAddAttr(pSHdl, attCliChCfgUuid, initCcc, sizeof(uint16_t), sizeof(uint16_t),
                   ATTS_SET_CCC, HID_SEC_PERMIT_READ | HID_SEC_PERMIT_WRITE);

    /* HID Input Report #1 (HIDAPP_REMOTE_REPORT_ID) */
    AttsDynAddAttrConst(pSHdl, attChUuid, hidIRep1Ch, sizeof(hidIRep1Ch), 0, ATTS_PERMIT_READ);
    AttsDynAddAttr(pSHdl, attHidRepChUuid, NULL, HID_MAX_REPORT_LEN, HID_MAX_REPORT_LEN,
                   ATTS_SET_VARIABLE_LEN, HID_SEC_PERMIT_READ);
    AttsDynAddAttr(pSHdl, attCliChCfgUuid, initCcc, sizeof(uint16_t), sizeof(uint16_t),
                   ATTS_SET_CCC, HID_SEC_PERMIT_READ | HID_SEC_PERMIT_WRITE);
    AttsDynAddAttrConst(pSHdl, attHidRimUuid, hidValIRep1IdMap, sizeof(hidValIRep1IdMap),
                        0, HID_SEC_PERMIT_READ);

    /* HID Input Report #2 (HIDAPP_KEYBOARD_REPORT_ID) */
    AttsDynAddAttrConst(pSHdl, attChUuid, hidIRep2Ch, sizeof(hidIRep2Ch), 0, ATTS_PERMIT_READ);
    AttsDynAddAttr(pSHdl, attHidRepChUuid, NULL, HID_MAX_REPORT_LEN, HID_MAX_REPORT_LEN,
                   ATTS_SET_VARIABLE_LEN, HID_SEC_PERMIT_READ);
    AttsDynAddAttr(pSHdl, attCliChCfgUuid, initCcc, sizeof(uint16_t), sizeof(uint16_t),
                   ATTS_SET_CCC, HID_SEC_PERMIT_READ | HID_SEC_PERMIT_WRITE);
    AttsDynAddAttrConst(pSHdl, attHidRimUuid, hidValIRep2IdMap, sizeof(hidValIRep2IdMap),
                        0, HID_SEC_PERMIT_READ);

    /* HID Input Report #3 (HIDAPP_MOUSE_REPORT_ID) */
    AttsDynAddAttrConst(pSHdl, attChUuid, hidIRep3Ch, sizeof(hidIRep3Ch), 0, ATTS_PERMIT_READ);
    AttsDynAddAttr(pSHdl, attHidRepChUuid, NULL, HID_MAX_REPORT_LEN, HID_MAX_REPORT_LEN,
                   ATTS_SET_VARIABLE_LEN, HID_SEC_PERMIT_READ);
    AttsDynAddAttr(pSHdl, attCliChCfgUuid, initCcc, sizeof(uint16_t), sizeof(uint16_t),
                   ATTS_SET_CCC, HID_SEC_PERMIT_READ | HID_SEC_PERMIT_WRITE);
    AttsDynAddAttrConst(pSHdl, attHidRimUuid, hidValIRep3IdMap, sizeof(hidValIRep3IdMap),
                        0, HID_SEC_PERMIT_READ);

    /* HID Output Report */
    AttsDynAddAttrConst(pSHdl, attChUuid, hidORepCh, sizeof(hidORepCh), 0, ATTS_PERMIT_READ);
    AttsDynAddAttr(pSHdl, attHidRepChUuid, NULL, HID_MAX_REPORT_LEN, HID_MAX_REPORT_LEN,
        ATTS_SET_WRITE_CBACK | ATTS_SET_VARIABLE_LEN, HID_SEC_PERMIT_READ | HID_SEC_PERMIT_WRITE);
    AttsDynAddAttrConst(pSHdl, attHidRimUuid, hidValORepIdMap, sizeof(hidValORepIdMap),
                        0, HID_SEC_PERMIT_READ);

    /* HID Feature Report */
    AttsDynAddAttrConst(pSHdl, attChUuid, hidFRepCh, sizeof(hidFRepCh), 0, ATTS_PERMIT_READ);
    AttsDynAddAttr(pSHdl, attHidRepChUuid, NULL, HID_MAX_REPORT_LEN, HID_MAX_REPORT_LEN,
        ATTS_SET_WRITE_CBACK | ATTS_SET_VARIABLE_LEN, HID_SEC_PERMIT_READ | HID_SEC_PERMIT_WRITE);
    AttsDynAddAttrConst(pSHdl, attHidRimUuid, hidValFRepIdMap, sizeof(hidValFRepIdMap),
                        0, HID_SEC_PERMIT_READ);

    /* HID Protocol Mode */
    AttsDynAddAttrConst(pSHdl, attChUuid, hidPmCh, sizeof(hidPmCh), 0, ATTS_PERMIT_READ);
    AttsDynAddAttr(pSHdl, attHidRepChUuid, hidPmVal, sizeof(hidPmVal),  sizeof(hidPmVal),
                   ATTS_SET_WRITE_CBACK, HID_SEC_PERMIT_READ | HID_SEC_PERMIT_WRITE);
  }

  return pSHdl;
}
