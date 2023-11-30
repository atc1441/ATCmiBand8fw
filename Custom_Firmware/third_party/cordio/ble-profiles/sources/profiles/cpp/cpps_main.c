/*************************************************************************************************/
/*!
*  \file
*
*  \brief  Cycling Power Profile Sensor Implementation.
*
*  Copyright (c) 2016-2018 Arm Ltd. All Rights Reserved.
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
#include "dm_api.h"
#include "svc_cps.h"
#include "svc_ch.h"
#include "cpp_api.h"

/*************************************************************************************************
* Constant Definitions
*************************************************************************************************/

/*! \brief The maximum length of a power measurement */
#define CPPS_PM_MAX_LEN       34

/*! \brief The maximum length of a field in a power measurement */
#define CPPS_PM_MAX_FIELD_LEN 6

/*! \brief Power Measurement Flag Indicies */
enum
{
  CPPS_PPBP_FLAG_INDEX,         /* Pedal Power Balance Present */
  CPPS_PPBR_FLAG_INDEX,         /* Pedal Power Balance Reference */
  CPPS_ATP_FLAG_INDEX,          /* Accumulated Torque Present */
  CPPS_ATS_FLAG_INDEX,          /* Accumulated Torque Source */
  CPPS_WRDP_FLAG_INDEX,         /* Wheel Revolution Data Present */
  CPPS_CRDP_FLAG_INDEX,         /* Crank Revolution Data Present */
  CPPS_EFMP_FLAG_INDEX,         /* Extreme Force Magnitudes Present */
  CPPS_ETMP_FLAG_INDEX,         /* Extreme Torque Magnitudes Present */
  CPPS_EAP_FLAG_INDEX,          /* Extreme Angles Present */
  CPPS_TDSAP_FLAG_INDEX,        /* Top Dead Spot Angle Present */
  CPPS_BDSAP_FLAG_INDEX,        /* Bottom Dead Spot Angle Present */
  CPPS_AEP_FLAG_INDEX,          /* Accumulated Energy Present */
  CPPS_OCI_FLAG_INDEX,          /* Offset Compensation Indicator */
  CPPS_NUM_FLAGS
};

/**************************************************************************************************
Data Types
**************************************************************************************************/

/*! \brief Power Measurement Data */
typedef struct
{
  uint16_t  flags;                      /* Power Measurement Flags */
  uint16_t  insantaneousPower;          /* Instantaneous Power Measurement */
  uint8_t   powerBalance;               /* Pedal Power Balance */
  uint16_t  accumulatedTorque;          /* Accumulated Torque */
  uint32_t  wheelRevolutions;           /* Wheel Revolution */
  uint16_t  wheelEventTime;             /* Last Wheel Revolution Event Time */
  uint16_t  crankRevolutions;           /* Crank Revolution */
  uint16_t  crankEventTime;             /* Last Crank Revolution Event Time */
  uint16_t  maxForceMagnitude;          /* Max Extreme Force Magnitudes */
  uint16_t  minForceMagnitude;          /* Min Extreme Force Magnitudes */
  uint16_t  maxTorqueMagnitude;         /* Max Extreme Torque Magnitudes */
  uint16_t  minTorqueMagnitude;         /* Min Extreme Torque Magnitudes */
  uint16_t  maxAngle;                   /* Max Extreme Angles */
  uint16_t  minAngle;                   /* Min Extreme Angles */
  uint16_t  topDeadSpotAngle;           /* Top Dead Spot Angle */
  uint16_t  btmDeadSpotAngle;           /* Bottom Dead Spot Angle */
  uint16_t  accumulatedEnergy;          /* Accumulated Energy */
} cppPmData_t;

/**************************************************************************************************
Local Variables
**************************************************************************************************/

/*! \brief measurement data */
cppPmData_t cppsPmData;

/*! \brief index of measurement notification when MTU is too small to send all at once */
uint8_t nextMeasFlag[DM_CONN_MAX];

/*************************************************************************************************/
/*!
*  \brief  Set the sensor location attribute.
*
*  \param  location   Sensor Location.
*
*  \return none
*/
/*************************************************************************************************/
void CppsSetSensorLocation(uint8_t location)
{
  AttsSetAttr(CPS_CPSL_HDL, sizeof(uint8_t), &location);
}

/*************************************************************************************************/
/*!
*  \brief  Set the features attribute.
*
*  \param  features   Features bitmask.
*
*  \return none
*/
/*************************************************************************************************/
void CppsSetFeatures(uint32_t features)
{
  uint8_t tempData[4] = {UINT32_TO_BYTES(features)};
  AttsSetAttr(CPS_CPF_HDL, sizeof(tempData), tempData);
}

/*************************************************************************************************/
/*!
*  \brief  Set a cycling power measurement parameter.
*
*  \param  type   Parameter identifier
*  \param  value  Measurement value.
*
*  \return none
*/
/*************************************************************************************************/
void CppsSetParameter(uint8_t type, uint32_t value)
{
  switch (type)
  {
  case CPP_PM_PARAM_INSTANTANEOUS_POWER:
    cppsPmData.insantaneousPower = (uint16_t) value;
    break;

  case CPP_PM_PARAM_PEDAL_POWER:
    cppsPmData.flags |= (1 << CPPS_PPBP_FLAG_INDEX);
    cppsPmData.powerBalance = (uint8_t) value;
    break;

  case CPP_PM_PARAM_ACCUMULATED_TORQUE:
    cppsPmData.flags |= (1 << CPPS_ATP_FLAG_INDEX);
    cppsPmData.accumulatedTorque = (uint16_t) value;
    break;

  case CPP_PM_PARAM_WHEEL_REVOLUTIONS:
    cppsPmData.flags |= (1 << CPPS_WRDP_FLAG_INDEX);
    cppsPmData.wheelRevolutions = value;
    break;

  case CPP_PM_PARAM_LAST_WHEEL_REV_TIME:
    cppsPmData.flags |= (1 << CPPS_WRDP_FLAG_INDEX);
    cppsPmData.wheelEventTime = (uint16_t) value;
    break;

  case CPP_PM_PARAM_CRANK_REVOLUTIONS:
    cppsPmData.flags |= (1 << CPPS_CRDP_FLAG_INDEX);
    cppsPmData.crankRevolutions = (uint16_t) value;
    break;

  case CPP_PM_PARAM_LAST_CRANK_TIME:
    cppsPmData.flags |= (1 << CPPS_CRDP_FLAG_INDEX);
    cppsPmData.crankEventTime = (uint16_t) value;
    break;

  case CPP_PM_PARAM_MAX_FORCE_MAGNITUDE:
    cppsPmData.flags |= (1 << CPPS_EFMP_FLAG_INDEX);
    cppsPmData.maxForceMagnitude = (uint16_t) value;
    break;

  case CPP_PM_PARAM_MIN_FORCE_MAGNITUDE:
    cppsPmData.flags |= (1 << CPPS_EFMP_FLAG_INDEX);
    cppsPmData.minForceMagnitude = (uint16_t) value;
    break;

  case CPP_PM_PARAM_MAX_TORQUE_MAGNITUDE:
    cppsPmData.flags |= (1 << CPPS_ETMP_FLAG_INDEX);
    cppsPmData.maxTorqueMagnitude = (uint16_t) value;
    break;

  case CPP_PM_PARAM_MIN_TORQUE_MAGNITUDE:
    cppsPmData.flags |= (1 << CPPS_ETMP_FLAG_INDEX);
    cppsPmData.minTorqueMagnitude = (uint16_t) value;
    break;

  case CPP_PM_PARAM_MAX_EXTREME_ANGLE:
    cppsPmData.flags |= (1 << CPPS_EAP_FLAG_INDEX);
    cppsPmData.maxAngle = (uint16_t) value;
    break;

  case CPP_PM_PARAM_MIN_EXTREME_ANGLE:
    cppsPmData.flags |= (1 << CPPS_EAP_FLAG_INDEX);
    cppsPmData.minAngle = (uint16_t) value;
    break;

  case CPP_PM_PARAM_TOP_DEAD_SPOT:
    cppsPmData.flags |= (1 << CPPS_TDSAP_FLAG_INDEX);
    cppsPmData.topDeadSpotAngle = (uint16_t) value;
    break;

  case CPP_PM_PARAM_BOTTOM_DEAD_SPOT:
    cppsPmData.flags |= (1 << CPPS_BDSAP_FLAG_INDEX);
    cppsPmData.btmDeadSpotAngle = (uint16_t) value;
    break;

  case CPP_PM_PARAM_ACCUMULATED_ENERGY:
    cppsPmData.flags |= (1 << CPPS_AEP_FLAG_INDEX);
    cppsPmData.accumulatedEnergy = (uint16_t) value;
    break;

  default:
    break;

  }
}

/*************************************************************************************************/
/*!
*  \brief  Notifies the collector of a Cycle Power Measurement.
*
*  \param  connId  Connection ID
*
*  \return none
*/
/*************************************************************************************************/
void CppsSendPowerMeasurement(dmConnId_t connId)
{
  int8_t i;
  uint16_t flags = 0;
  uint16_t len = 0;
  uint8_t msg[CPPS_PM_MAX_LEN];
  uint8_t measChar[CPPS_PM_MAX_FIELD_LEN];
  uint8_t *pMeasChar;
  uint8_t *pMsg = msg;
  uint32_t temp;
  uint16_t maxValueLen;

  /* Get maximum length for a notification (ATT_MTU - 3) */
  maxValueLen = AttGetMtu(connId) - ATT_VALUE_NTF_LEN;

  /* Add manditory parameters. skip flags field for now */
  UINT16_TO_BSTREAM(pMsg, 0);
  UINT16_TO_BSTREAM(pMsg, cppsPmData.insantaneousPower);

  /* Add optional parameters */
  for (i = nextMeasFlag[connId - 1]; i < CPPS_NUM_FLAGS; i++)
  {
    /* Add data if flag is set */
    if (cppsPmData.flags & (1 << i))
    {
      pMeasChar = measChar;

      switch (i)
      {
      case CPPS_PPBP_FLAG_INDEX:
        UINT8_TO_BSTREAM(pMeasChar, cppsPmData.powerBalance);
        break;
      case CPPS_ATP_FLAG_INDEX:
        UINT16_TO_BSTREAM(pMeasChar, cppsPmData.accumulatedTorque);
        break;
      case CPPS_WRDP_FLAG_INDEX:
        UINT32_TO_BSTREAM(pMeasChar, cppsPmData.wheelRevolutions);
        UINT16_TO_BSTREAM(pMeasChar, cppsPmData.wheelEventTime);
        break;
      case CPPS_CRDP_FLAG_INDEX:
        UINT16_TO_BSTREAM(pMeasChar, cppsPmData.crankRevolutions);
        UINT16_TO_BSTREAM(pMeasChar, cppsPmData.crankEventTime);
        break;
      case CPPS_EFMP_FLAG_INDEX:
        UINT16_TO_BSTREAM(pMeasChar, cppsPmData.maxForceMagnitude);
        UINT16_TO_BSTREAM(pMeasChar, cppsPmData.minForceMagnitude);
        break;
      case CPPS_ETMP_FLAG_INDEX:
        UINT16_TO_BSTREAM(pMeasChar, cppsPmData.maxTorqueMagnitude);
        UINT16_TO_BSTREAM(pMeasChar, cppsPmData.minTorqueMagnitude);
        break;
      case CPPS_EAP_FLAG_INDEX:
        temp = ((uint32_t)cppsPmData.maxAngle & 0xfff) | ((uint32_t)cppsPmData.minAngle << 12);
        UINT24_TO_BSTREAM(pMeasChar, temp);
        break;
      case CPPS_TDSAP_FLAG_INDEX:
        UINT16_TO_BSTREAM(pMeasChar, cppsPmData.topDeadSpotAngle);
        break;
      case CPPS_BDSAP_FLAG_INDEX:
        UINT16_TO_BSTREAM(pMeasChar, cppsPmData.btmDeadSpotAngle);
        break;
      case CPPS_AEP_FLAG_INDEX:
        UINT16_TO_BSTREAM(pMeasChar, cppsPmData.accumulatedEnergy);
        break;
      default:
        break;
      }

      /* Add data into message buffer */
      if ((len = pMeasChar - measChar) > 0)
      {
        if ((pMsg + len - msg) <= maxValueLen)
        {
          memcpy(pMsg, measChar, len);
          pMsg += len;

          flags |= cppsPmData.flags & (1 << i);
        }
        else
        {
          /* Buffer is full.  Store current flag offset for next Notification. */
          nextMeasFlag[connId - 1] = i;
          break;
        }
      }
    }
  }

  /* If all flags checked reset flag index */
  if (i == CPPS_NUM_FLAGS)
  {
    nextMeasFlag[connId - 1] = 0;
  }

  /* Set flags */
  UINT16_TO_BUF(msg, flags);

  /* Calculate message length */
  len = (uint16_t) (pMsg - msg);

  /* Transmit notification */
  AttsHandleValueNtf(connId, CPS_CPM_HDL, len, msg);

  /* Clear the measurement data */
  memset(&cppsPmData, 0, sizeof(cppsPmData));
}

/*************************************************************************************************/
/*!
 *  \fn     CppsConnOpen
 *
 *  \brief  Setup connection specific variables.
 *
 *  \param  connId  Connection ID
 *
 *  \return none
 */
/*************************************************************************************************/
void CppsConnOpen(dmConnId_t connId)
{
  nextMeasFlag[connId - 1] = 0;
}
