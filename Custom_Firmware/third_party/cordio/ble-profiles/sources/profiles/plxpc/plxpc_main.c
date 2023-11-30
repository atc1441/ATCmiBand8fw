/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Pulse Oximeter profile collector.
 *
 *  Copyright (c) 2016-2019 Arm Ltd. All Rights Reserved.
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
#include "wsf_assert.h"
#include "wsf_trace.h"
#include "util/bstream.h"
#include "svc_ch.h"
#include "app_api.h"
#include "plxpc_api.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/* Length of response data contained in a received RACP message */
#define PLXPC_PLX_RACP_RSP_LEN         4

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*!
 *  Pulse Oximeter service characteristics for discovery
 */

/*! Pulse Oximeter spot check measurement */
static const attcDiscChar_t plxpcPlxsPlxsc =
{
  attPlxscmChUuid,
  0
};

/*! Pulse Oximeter spot check measurement CCC descriptor */
static const attcDiscChar_t plxpcPlxsPlxscCcc =
{
  attCliChCfgUuid,
  ATTC_SET_DESCRIPTOR
};

/*! Pulse Oximeter continuous measurement */
static const attcDiscChar_t plxpcPlxsPlxc =
{
  attPlxcmChUuid,
  0
};

/*! Pulse Oximeter continuous measurement CCC descriptor */
static const attcDiscChar_t plxpcPlxsPlxcCcc =
{
  attCliChCfgUuid,
  ATTC_SET_DESCRIPTOR
};

/*! Pulse Oximeter features */
static const attcDiscChar_t plxpcPlxsPlxf =
{
  attPlxfChUuid,
  ATTC_SET_REQUIRED
};

/*! Record access control point */
static const attcDiscChar_t plxpcPlxsRacp =
{
  attRacpChUuid,
  0
};

/*! Record access control point CCC descriptor */
static const attcDiscChar_t plxpcPlxsRacpCcc =
{
  attCliChCfgUuid,
  ATTC_SET_DESCRIPTOR
};

/*! List of characteristics to be discovered; order matches handle index enumeration  */
static const attcDiscChar_t *plxpcPlxsDiscCharList[] =
{
  &plxpcPlxsPlxsc,                  /*! Pulse Oximeter measurement */
  &plxpcPlxsPlxscCcc,               /*! Pulse Oximeter measurement CCC descriptor */
  &plxpcPlxsPlxc,                   /*! Pulse Oximeter measurement context */
  &plxpcPlxsPlxcCcc,                /*! Pulse Oximeter measurement context CCC descriptor */
  &plxpcPlxsPlxf,                   /*! Pulse Oximeter feature */
  &plxpcPlxsRacp,                   /*! Record access control point */
  &plxpcPlxsRacpCcc                 /*! Record access control point CCC descriptor */
};

/* sanity check:  make sure handle list length matches characteristic list length */
WSF_CT_ASSERT(PLXPC_PLXS_HDL_LIST_LEN == ((sizeof(plxpcPlxsDiscCharList) / sizeof(attcDiscChar_t *))));

/*************************************************************************************************/
/*!
 *  \brief  Parse a pulse oximeter spot check measurement.
 *
 *  \param  pValue    Pointer to buffer containing value.
 *  \param  len       length of buffer.
 *
 *  \return None.
 */
/*************************************************************************************************/
void plxpcPlxsParsePlxsc(uint8_t *pValue, uint16_t len)
{
  uint8_t   flags = 0;
  uint16_t  year;
  uint8_t   month, day, hour, min, sec;
  int16_t   mantissa;
  int8_t    exponent;
  uint16_t  spo2;
  uint16_t  pulse;
  uint32_t  sensorStatus;
  uint16_t  measurementStatus;
  uint16_t  pulseAmpIndex;
  uint16_t  minLen = CH_PLX_FLAGS_LEN + CH_PLX_SPO2_LEN + CH_PLX_PULSE_LEN;

  /* Suppress unused variable compile warning */
  (void)measurementStatus; (void)sensorStatus; (void)exponent; (void)mantissa;
  (void)year; (void)month; (void)day; (void)hour; (void)min; (void)sec;

  if (len > 0)
  {
    /* get flags */
    BSTREAM_TO_UINT8(flags, pValue);

    /* determine expected minimum length based on flags */
    if (flags & CH_PLXSC_FLAG_TIMESTAMP)
    {
      minLen += CH_PLXSC_TIMESTAMP_LEN;
    }
    if (flags & CH_PLXSC_FLAG_MEASUREMENT_STATUS)
    {
      minLen += CH_PLX_MEASUREMENT_STATUS_LEN;
    }
    if (flags & CH_PLXSC_FLAG_SENSOR_STATUS)
    {
      minLen += CH_PLX_SENSOR_STATUS_LEN;
    }
    if (flags & CH_PLXSC_FLAG_PULSE_AMP_INDX)
    {
      minLen += CH_PLX_PULSE_AMP_INDX_LEN;
    }
  }

  /* verify length */
  if (len < minLen)
  {
    APP_TRACE_INFO2("Pulse Oximeter meas len:%d minLen:%d", len, minLen);
    return;
  }

  /* get SpO2 */
  BSTREAM_TO_UINT16(spo2, pValue);
  UINT16_TO_SFLT(mantissa, exponent, spo2);
  APP_TRACE_INFO2("  Pulse Oximeter SpO2: %de%d", mantissa, exponent);

  /* get pulse */
  BSTREAM_TO_UINT16(pulse, pValue);
  UINT16_TO_SFLT(mantissa, exponent, pulse);
  APP_TRACE_INFO2("  Pulse Oximeter Pulse: %de%d", mantissa, exponent);

  /* timestamp */
  if (flags & CH_PLXSC_FLAG_TIMESTAMP)
  {
    /* base time */
    BSTREAM_TO_UINT16(year, pValue);
    BSTREAM_TO_UINT8(month, pValue);
    BSTREAM_TO_UINT8(day, pValue);
    BSTREAM_TO_UINT8(hour, pValue);
    BSTREAM_TO_UINT8(min, pValue);
    BSTREAM_TO_UINT8(sec, pValue);
    APP_TRACE_INFO3("  Date: %d/%d/%d", month, day, year);
    APP_TRACE_INFO3("  Time: %02d:%02d:%02d", hour, min, sec);
  }

  /* measurement status */
  if (flags & CH_PLXSC_FLAG_MEASUREMENT_STATUS)
  {
    BSTREAM_TO_UINT16(measurementStatus, pValue);
    APP_TRACE_INFO1("  Pulse Oximeter measurement status: 0x%04", measurementStatus);
  }

  /* sensor status */
  if (flags & CH_PLXSC_FLAG_SENSOR_STATUS)
  {
    BSTREAM_TO_UINT24(sensorStatus, pValue);
    APP_TRACE_INFO1("   Pulse Oximeter Sensor status: 0x%04x", sensorStatus);
  }

  /* Pulse amplitde index */
  if (flags & CH_PLXSC_FLAG_PULSE_AMP_INDX)
  {
    BSTREAM_TO_UINT16(pulseAmpIndex, pValue);
    UINT16_TO_SFLT(mantissa, exponent, pulseAmpIndex);
    APP_TRACE_INFO2("  Pulse Oximeter Pulse Amplitude Index: %de%d", mantissa, exponent);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Parse a pulse oximeter continuous measurement.
 *
 *  \param  pValue    Pointer to buffer containing value.
 *  \param  len       length of buffer.
 *
 *  \return None.
 */
/*************************************************************************************************/
void plxpcPlxsParsePlxc(uint8_t *pValue, uint16_t len)
{
  uint8_t   flags = 0;
  int16_t   mantissa;
  int8_t    exponent;
  uint16_t  spo2;
  uint16_t  pulse;
  uint32_t  sensorStatus;
  uint16_t  measurementStatus;
  uint16_t  pulseAmpIndex;
  uint16_t  minLen = CH_PLX_FLAGS_LEN + CH_PLX_SPO2_LEN + CH_PLX_PULSE_LEN;

  /* Suppress unused variable compile warning */
  (void)measurementStatus; (void)sensorStatus; (void)exponent; (void)mantissa;

  if (len > 0)
  {
    /* get flags */
    BSTREAM_TO_UINT8(flags, pValue);

    /* determine expected minimum length based on flags */
    if (flags & CH_PLXC_FLAG_SPO2PR_FAST)
    {
      minLen += CH_PLXC_SPO2PR_FAST_LEN;
    }
    if (flags & CH_PLXC_FLAG_SPO2PR_SLOW)
    {
      minLen += CH_PLXC_SPO2PR_SLOW_LEN;
    }
    if (flags & CH_PLXC_FLAG_MEASUREMENT_STATUS)
    {
      minLen += CH_PLX_MEASUREMENT_STATUS_LEN;
    }
    if (flags & CH_PLXC_FLAG_SENSOR_STATUS)
    {
      minLen += CH_PLX_SENSOR_STATUS_LEN;
    }
    if (flags & CH_PLXC_FLAG_PULSE_AMP_INDX)
    {
      minLen += CH_PLX_PULSE_AMP_INDX_LEN;
    }
  }

  /* verify length */
  if (len < minLen)
  {
    APP_TRACE_INFO2("Pulse Oximeter meas len:%d minLen:%d", len, minLen);
    return;
  }

  /* get SpO2 */
  BSTREAM_TO_UINT16(spo2, pValue);
  UINT16_TO_SFLT(mantissa, exponent, spo2);
  APP_TRACE_INFO2("  Pulse Oximeter SpO2: %de%d", mantissa, exponent);

  /* get pulse */
  BSTREAM_TO_UINT16(pulse, pValue);
  UINT16_TO_SFLT(mantissa, exponent, pulse);
  APP_TRACE_INFO2("  Pulse Oximeter Pulse: %de%d", mantissa, exponent);

  /* SpO2PR-Fast */
  if (flags & CH_PLXC_FLAG_SPO2PR_FAST)
  {
    /* get SpO2 */
    BSTREAM_TO_UINT16(spo2, pValue);
    UINT16_TO_SFLT(mantissa, exponent, spo2);
    APP_TRACE_INFO2("  Pulse Oximeter SpO2-Fast: %de%d", mantissa, exponent);

    /* get pulse */
    BSTREAM_TO_UINT16(pulse, pValue);
    UINT16_TO_SFLT(mantissa, exponent, pulse);
    APP_TRACE_INFO2("  Pulse Oximeter Pulse-Fast: %de%d", mantissa, exponent);
  }

  /* SpO2PR-Slow */
  if (flags & CH_PLXC_FLAG_SPO2PR_SLOW)
  {
    /* get SpO2 */
    BSTREAM_TO_UINT16(spo2, pValue);
    UINT16_TO_SFLT(mantissa, exponent, spo2);
    APP_TRACE_INFO2("  Pulse Oximeter SpO2-Slow: %de%d", mantissa, exponent);

    /* get pulse */
    BSTREAM_TO_UINT16(pulse, pValue);
    UINT16_TO_SFLT(mantissa, exponent, pulse);
    APP_TRACE_INFO2("  Pulse Oximeter Pulse-Slow: %de%d", mantissa, exponent);
  }

  /* measurement status */
  if (flags & CH_PLXC_FLAG_MEASUREMENT_STATUS)
  {
    BSTREAM_TO_UINT16(measurementStatus, pValue);
    APP_TRACE_INFO1("  Pulse Oximeter measurement status: 0x%04", measurementStatus);
  }

  /* sensor status */
  if (flags & CH_PLXC_FLAG_SENSOR_STATUS)
  {
    BSTREAM_TO_UINT24(sensorStatus, pValue);
    APP_TRACE_INFO1("   Pulse Oximeter Sensor status: 0x%04x", sensorStatus);
  }

  /* Pulse amplitde index */
  if (flags & CH_PLXC_FLAG_PULSE_AMP_INDX)
  {
    BSTREAM_TO_UINT16(pulseAmpIndex, pValue);
    UINT16_TO_SFLT(mantissa, exponent, pulseAmpIndex);
    APP_TRACE_INFO2("  Pulse Oximeter Pulse Amplitude Index: %de%d", mantissa, exponent);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Process a message received from the sensor on the record access control point.
 *
 *  \param  pValue    Pointer to buffer containing value.
 *  \param  len       length of buffer.
 *
 *  \return None.
 */
/*************************************************************************************************/
void plxpcPlxsProcRacp(uint8_t *pValue, uint16_t len)
{
  uint8_t   opcode;
  uint16_t  numRecords;
  uint16_t  reqOpcode;
  uint16_t  status;

  /* Suppress unused variable compile warning */
  (void)status; (void)reqOpcode; (void)numRecords;

  /* verify length */
  if (len != PLXPC_PLX_RACP_RSP_LEN)
  {
    APP_TRACE_INFO1("Unexpected RACP message length: %d", len);
    return;
  }

  /* parse message */
  BSTREAM_TO_UINT8(opcode, pValue);

  /* Operator unused */
  pValue++;

  if (opcode == CH_RACP_OPCODE_NUM_RSP)
  {
    BSTREAM_TO_UINT16(numRecords, pValue);
    APP_TRACE_INFO1("Number of records: %d", numRecords);
  }
  else if (opcode == CH_RACP_OPCODE_RSP)
  {
    BSTREAM_TO_UINT8(reqOpcode, pValue);
    BSTREAM_TO_UINT8(status, pValue);
    APP_TRACE_INFO2("Response opcode: %d status: %d", reqOpcode, status);
  }
}

/*************************************************************************************************/
/*!
*  \brief  Parse a pulse oximeter features.
*
*  \param  pValue    Pointer to buffer containing value.
*  \param  len       length of buffer.
*
*  \return None.
*/
/*************************************************************************************************/
void plxpcPlxParsePlxf(uint8_t *pValue, uint16_t len)
{
  int16_t   features;
  int16_t   measStatusSupport;
  int16_t   sensorStatusSupport;

  uint16_t  minLen = CH_PLXF_MIN_FEATURES_LEN;

  /* Suppress unused variable compile warning */
  (void)measStatusSupport; (void)sensorStatusSupport;

  if (len >= minLen)
  {
    /* get flags */
    BSTREAM_TO_UINT16(features, pValue);

    /* determine expected minimum length based on features */
    if (features & CH_PLF_FLAG_MEAS_STATUS_SUP)
    {
      minLen += CH_PLXF_SENSOR_SUPPORT_LEN;
    }

    if (features & CH_PLF_FLAG_SENSOR_STATUS_SUP)
    {
      minLen += CH_PLXF_MEASUREMENT_SUPPORT_LEN;
    }
  }

  /* verify length */
  if (len < minLen)
  {
    APP_TRACE_INFO2("Pulse Oximeter feature len:%d minLen:%d", len, minLen);
    return;
  }

  /* SpO2PR-Fast */
  if (features & CH_PLF_FLAG_MEAS_STATUS_SUP)
  {
    /* get Measurement Status Support */
    BSTREAM_TO_UINT16(measStatusSupport, pValue);
    APP_TRACE_INFO1("  Pulse Oximeter measurement status sypported: 0x%04", measStatusSupport);
  }

  /* SpO2PR-Slow */
  if (features & CH_PLF_FLAG_SENSOR_STATUS_SUP)
  {
    /* get Device and Sensor Status Support */
    BSTREAM_TO_UINT16(sensorStatusSupport, pValue);
    APP_TRACE_INFO1("  Pulse Oximeter sensor status supported: 0x%04", sensorStatusSupport);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for Pulse Oximeter service.  Parameter
 *          pHdlList must point to an array of length PLXPC_PLXS_HDL_LIST_LEN.  If discovery is
 *          successful the handles of discovered characteristics and descriptors will be set
 *          in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void PlxpcPlxsDiscover(dmConnId_t connId, uint16_t *pHdlList)
{
  AppDiscFindService(connId, ATT_16_UUID_LEN, (uint8_t *) attPlxsSvcUuid,
                     PLXPC_PLXS_HDL_LIST_LEN, (attcDiscChar_t **) plxpcPlxsDiscCharList, pHdlList);
}

/*************************************************************************************************/
/*!
 *  \brief  Process a value received in an ATT read response, notification, or indication
 *          message.  Parameter pHdlList must point to an array of length PLXPC_PLXS_HDL_LIST_LEN.
 *          If the attribute handle of the message matches a handle in the handle list the value
 *          is processed, otherwise it is ignored.
 *
 *  \param  pHdlList  Characteristic handle list.
 *  \param  pMsg      ATT callback message.
 *
 *  \return ATT_SUCCESS if handle is found, ATT_ERR_NOT_FOUND otherwise.
 */
/*************************************************************************************************/
uint8_t PlxpcPlxsValueUpdate(uint16_t *pHdlList, attEvt_t *pMsg)
{
  uint8_t   status = ATT_SUCCESS;

  /* spot check measurement */
  if (pMsg->handle == pHdlList[PLXPC_PLXS_PLXSC_HDL_IDX])
  {
    APP_TRACE_INFO0("Pulse Oximeter spot check measurement.");

    /* parse value */
    plxpcPlxsParsePlxsc(pMsg->pValue, pMsg->valueLen);
  }
  /* continuous measurement */
  else if (pMsg->handle == pHdlList[PLXPC_PLXS_PLXC_HDL_IDX])
  {
    APP_TRACE_INFO0("Pulse Oximeter continuous measurement");

    /* parse value */
    plxpcPlxsParsePlxc(pMsg->pValue, pMsg->valueLen);
  }
  /* record access control point */
  else if (pMsg->handle == pHdlList[PLXPC_PLXS_RACP_HDL_IDX])
  {
    plxpcPlxsProcRacp(pMsg->pValue, pMsg->valueLen);
  }
  /* pulse oximeter feature  */
  else if (pMsg->handle == pHdlList[PLXPC_PLXS_PLXF_HDL_IDX])
  {
    APP_TRACE_INFO0("Pulse Oximeter features");

    /* parse value */
    plxpcPlxParsePlxf(pMsg->pValue, pMsg->valueLen);
  }
  else
  {
    status = ATT_ERR_NOT_FOUND;
  }

  return status;
}

/*************************************************************************************************/
/*!
 *  \brief  Send a command to the pulse oximeter service record access control point.
 *
 *  \param  connId  Connection identifier.
 *  \param  handle  Attribute handle.
 *  \param  opcode  Command opcode.
 *  \param  oper    Command operator or 0 if no operator required.
 *
 *  \return None.
 */
/*************************************************************************************************/
void PlxpcPlxsRacpSend(dmConnId_t connId, uint16_t handle, uint8_t opcode, uint8_t oper)
{
  uint8_t   buf[ATT_DEFAULT_PAYLOAD_LEN];
  uint8_t   *p = buf;

  if (handle != ATT_HANDLE_NONE)
  {
    /* build RACP command */
    UINT8_TO_BSTREAM(p, opcode);
    UINT8_TO_BSTREAM(p, oper);

    AttcWriteReq(connId, handle, (uint16_t) (p - buf), buf);
  }
}
