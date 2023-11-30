/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example Device Information Service implementation.
 *
 *  Copyright (c) 2011-2020 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2019-2020 Packetcraft, Inc.
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

#include "wsf_types.h"
#include "att_api.h"
#include "wsf_trace.h"
#include "util/bstream.h"
#include "svc_dis.h"
#include "svc_cfg.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions */
#ifndef DIS_SEC_PERMIT_READ
#define DIS_SEC_PERMIT_READ SVC_SEC_PERMIT_READ
#endif

/*! Default manufacturer name */
#define DIS_DEFAULT_MFR_NAME        "Arm Ltd."

/*! Length of default manufacturer name */
#define DIS_DEFAULT_MFR_NAME_LEN    8

/*! Default model number */
#define DIS_DEFAULT_MODEL_NUM       "Packetcraft model num"

/*! Length of default model number */
#define DIS_DEFAULT_MODEL_NUM_LEN   21

/*! Default serial number */
#define DIS_DEFAULT_SERIAL_NUM      "Packetcraft serial num"

/*! Length of default serial number */
#define DIS_DEFAULT_SERIAL_NUM_LEN  22

/*! Default firmware revision */
#define DIS_DEFAULT_FW_REV          "Packetcraft fw rev"

/*! Length of default firmware revision */
#define DIS_DEFAULT_FW_REV_LEN      18

/*! Default hardware revision */
#define DIS_DEFAULT_HW_REV          "Packetcraft hw rev"

/*! Length of default hardware revision */
#define DIS_DEFAULT_HW_REV_LEN      18

/*! Default software revision */
#define DIS_DEFAULT_SW_REV          "Packetcraft sw rev"

/*! Length of default software revision */
#define DIS_DEFAULT_SW_REV_LEN      18

/*! Vendor ID source */
#define DIS_VENDOR_ID_SRC_BT        0x01    /* Bluetooth SIG - assigned Device ID Vendor ID value from Assigned Numbers document */
#define DIS_VENDOR_ID_SRC_USB       0x02    /* USB Implementer's Forum assigned Vendor ID value */

/*! Vendor ID: Company Identifier from Bluetooth SIG or Vendor ID from USB Implementer's Forum */
#define DIS_VENDOR_ID               HCI_ID_PACKETCRAFT

/*! Product ID */
#define DIS_PRODUCT_ID              0x1234

/*! Product version: 0xJJMN for version JJ.M.N (JJ - major version number, M - minor version number, N - sub-minor version number) */
#define DIS_PRODUCT_VER             0x2030 /* version 20.3.0 */

/**************************************************************************************************
 Service variables
**************************************************************************************************/

/* Device information service declaration */
static const uint8_t disValSvc[] = {UINT16_TO_BYTES(ATT_UUID_DEVICE_INFO_SERVICE)};
static const uint16_t disLenSvc = sizeof(disValSvc);

/* Manufacturer name string characteristic */
static const uint8_t disValMfrCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(DIS_MFR_HDL), UINT16_TO_BYTES(ATT_UUID_MANUFACTURER_NAME)};
static const uint16_t disLenMfrCh = sizeof(disValMfrCh);

/* Manufacturer name string */
static const uint8_t disUuMfr[] = {UINT16_TO_BYTES(ATT_UUID_MANUFACTURER_NAME)};
static uint8_t disValMfr[DIS_MAXSIZE_MFR_ATT] = DIS_DEFAULT_MFR_NAME;
static uint16_t disLenMfr = DIS_DEFAULT_MFR_NAME_LEN;

/* System ID characteristic */
static const uint8_t disValSidCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(DIS_SID_HDL), UINT16_TO_BYTES(ATT_UUID_SYSTEM_ID)};
static const uint16_t disLenSidCh = sizeof(disValSidCh);

/* System ID */
static const uint8_t disUuSid[] = {UINT16_TO_BYTES(ATT_UUID_SYSTEM_ID)};
static uint8_t disValSid[DIS_SIZE_SID_ATT] = {0x01, 0x02, 0x03, 0x04, 0x05, UINT16_TO_BYTE0(HCI_ID_PACKETCRAFT), UINT16_TO_BYTE1(HCI_ID_PACKETCRAFT), 0x00};
static const uint16_t disLenSid = sizeof(disValSid);

/* Model number string characteristic */
static const uint8_t disValMnCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(DIS_MN_HDL), UINT16_TO_BYTES(ATT_UUID_MODEL_NUMBER)};
static const uint16_t disLenMnCh = sizeof(disValMnCh);

/* Model number string */
static const uint8_t disUuMn[] = {UINT16_TO_BYTES(ATT_UUID_MODEL_NUMBER)};
static uint8_t disValMn[DIS_MAXSIZE_MN_ATT] = DIS_DEFAULT_MODEL_NUM;
static uint16_t disLenMn = DIS_DEFAULT_MODEL_NUM_LEN;

/* Serial number string characteristic */
static const uint8_t disValSnCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(DIS_SN_HDL), UINT16_TO_BYTES(ATT_UUID_SERIAL_NUMBER)};
static const uint16_t disLenSnCh = sizeof(disValSnCh);

/* Serial number string */
static const uint8_t disUuSn[] = {UINT16_TO_BYTES(ATT_UUID_SERIAL_NUMBER)};
static uint8_t disValSn[DIS_MAXSIZE_SN_ATT] = DIS_DEFAULT_SERIAL_NUM;
static uint16_t disLenSn = DIS_DEFAULT_SERIAL_NUM_LEN;

/* Firmware revision string characteristic */
static const uint8_t disValFwrCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(DIS_FWR_HDL), UINT16_TO_BYTES(ATT_UUID_FIRMWARE_REV)};
static const uint16_t disLenFwrCh = sizeof(disValFwrCh);

/* Firmware revision string */
static const uint8_t disUuFwr[] = {UINT16_TO_BYTES(ATT_UUID_FIRMWARE_REV)};
static uint8_t disValFwr[DIS_MAXSIZE_FWR_ATT] = DIS_DEFAULT_FW_REV;
static uint16_t disLenFwr = DIS_DEFAULT_FW_REV_LEN;

/* Hardware revision string characteristic */
static const uint8_t disValHwrCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(DIS_HWR_HDL), UINT16_TO_BYTES(ATT_UUID_HARDWARE_REV)};
static const uint16_t disLenHwrCh = sizeof(disValHwrCh);

/* Hardware revision string */
static const uint8_t disUuHwr[] = {UINT16_TO_BYTES(ATT_UUID_HARDWARE_REV)};
static uint8_t disValHwr[DIS_MAXSIZE_HWR_ATT] = DIS_DEFAULT_HW_REV;
static uint16_t disLenHwr = DIS_DEFAULT_HW_REV_LEN;

/* Software revision string characteristic */
static const uint8_t disValSwrCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(DIS_SWR_HDL), UINT16_TO_BYTES(ATT_UUID_SOFTWARE_REV)};
static const uint16_t disLenSwrCh = sizeof(disValSwrCh);

/* Software revision string */
static const uint8_t disUuSwr[] = {UINT16_TO_BYTES(ATT_UUID_SOFTWARE_REV)};
static uint8_t disValSwr[DIS_MAXSIZE_SWR_ATT] = DIS_DEFAULT_SW_REV;
static uint16_t disLenSwr = DIS_DEFAULT_SW_REV_LEN;

/* IEEE 11073-20601 regulatory certificate data characteristic */
static const uint8_t disValRcdCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(DIS_RCD_HDL), UINT16_TO_BYTES(ATT_UUID_11073_CERT_DATA)};
static const uint16_t disLenRcdCh = sizeof(disValRcdCh);

/* IEEE 11073-20601 regulatory certificate data */
static const uint8_t disUuRcd[] = {UINT16_TO_BYTES(ATT_UUID_11073_CERT_DATA)};
static uint8_t disValRcd[DIS_SIZE_RCD_ATT] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint16_t disLenRcd = sizeof(disValRcd);

/* PnP ID characteristic */
static const uint8_t disValPnpIdCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(DIS_PNP_ID_HDL), UINT16_TO_BYTES(ATT_UUID_PNP_ID)};
static const uint16_t disLenPnpIdCh = sizeof(disValPnpIdCh);

/* PnP ID */
static const uint8_t disUuPnpId[] = {UINT16_TO_BYTES(ATT_UUID_PNP_ID)};
static uint8_t disValPnpId[] =
{
  /* Vendor ID source */
  DIS_VENDOR_ID_SRC_BT,

  /* Vendor ID */
  UINT16_TO_BYTE0(DIS_VENDOR_ID), UINT16_TO_BYTE1(DIS_VENDOR_ID),

  /* Product ID */
  UINT16_TO_BYTE0(DIS_PRODUCT_ID), UINT16_TO_BYTE1(DIS_PRODUCT_ID),

  /* Product version */
  UINT16_TO_BYTE0(DIS_PRODUCT_VER), UINT16_TO_BYTE1(DIS_PRODUCT_VER)
};
static const uint16_t disLenPnpId = sizeof(disValPnpId);

/* Attribute list for dis group */
static const attsAttr_t disList[] =
{
  {
    attPrimSvcUuid,
    (uint8_t *) disValSvc,
    (uint16_t *) &disLenSvc,
    sizeof(disValSvc),
    0,
    ATTS_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) disValMfrCh,
    (uint16_t *) &disLenMfrCh,
    sizeof(disValMfrCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    disUuMfr,
    (uint8_t *) disValMfr,
    (uint16_t *) &disLenMfr,
    sizeof(disValMfr),
    ATTS_SET_VARIABLE_LEN,
    DIS_SEC_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) disValSidCh,
    (uint16_t *) &disLenSidCh,
    sizeof(disValSidCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    disUuSid,
    disValSid,
    (uint16_t *) &disLenSid,
    sizeof(disValSid),
    0,
    DIS_SEC_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) disValMnCh,
    (uint16_t *) &disLenMnCh,
    sizeof(disValMnCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    disUuMn,
    (uint8_t *) disValMn,
    (uint16_t *) &disLenMn,
    sizeof(disValMn),
    ATTS_SET_VARIABLE_LEN,
    DIS_SEC_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) disValSnCh,
    (uint16_t *) &disLenSnCh,
    sizeof(disValSnCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    disUuSn,
    (uint8_t *) disValSn,
    (uint16_t *) &disLenSn,
    sizeof(disValSn),
    ATTS_SET_VARIABLE_LEN,
    DIS_SEC_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) disValFwrCh,
    (uint16_t *) &disLenFwrCh,
    sizeof(disValFwrCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    disUuFwr,
    (uint8_t *) disValFwr,
    (uint16_t *) &disLenFwr,
    sizeof(disValFwr),
    ATTS_SET_VARIABLE_LEN,
    DIS_SEC_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) disValHwrCh,
    (uint16_t *) &disLenHwrCh,
    sizeof(disValHwrCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    disUuHwr,
    (uint8_t *) disValHwr,
    (uint16_t *) &disLenHwr,
    sizeof(disValHwr),
    ATTS_SET_VARIABLE_LEN,
    DIS_SEC_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) disValSwrCh,
    (uint16_t *) &disLenSwrCh,
    sizeof(disValSwrCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    disUuSwr,
    (uint8_t *) disValSwr,
    (uint16_t *) &disLenSwr,
    sizeof(disValSwr),
    ATTS_SET_VARIABLE_LEN,
    DIS_SEC_PERMIT_READ
  },
  {
    attChUuid,
    (uint8_t *) disValRcdCh,
    (uint16_t *) &disLenRcdCh,
    sizeof(disValRcdCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    disUuRcd,
    (uint8_t *) disValRcd,
    (uint16_t *) &disLenRcd,
    sizeof(disValRcd),
    0,
    DIS_SEC_PERMIT_READ
  },
  {
      attChUuid,
      (uint8_t *) disValPnpIdCh,
      (uint16_t *) &disLenPnpIdCh,
      sizeof(disValPnpIdCh),
      0,
      ATTS_PERMIT_READ
  },
  {
    disUuPnpId,
    disValPnpId,
    (uint16_t *) &disLenPnpId,
    sizeof(disValPnpId),
    0,
    DIS_SEC_PERMIT_READ
  }
};

/* DIS group structure */
static attsGroup_t svcDisGroup =
{
  NULL,
  (attsAttr_t *) disList,
  NULL,
  NULL,
  DIS_START_HDL,
  DIS_END_HDL
};

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcDisAddGroup(void)
{
  AttsAddGroup(&svcDisGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcDisRemoveGroup(void)
{
  AttsRemoveGroup(DIS_START_HDL);
}
