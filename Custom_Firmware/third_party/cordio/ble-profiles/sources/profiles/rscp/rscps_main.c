/*************************************************************************************************/
/*!
*  \file
*
*  \brief  Running Speed and Cadence Profile Sensor Implementation.
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
#include "svc_rscs.h"
#include "svc_ch.h"
#include "rscp_api.h"

/*************************************************************************************************
* Constant Definitions
*************************************************************************************************/

/*! \brief The maximum length of a power measurement */
#define RSCPS_PM_MAX_LEN     10

/*! \brief Running Speed Measurement Flag Indicies */
enum
{
  RSCPS_ISLP_FLAG_INDEX,          /*! \brief Instantaneous Stride Length Present Present */
  RSCPS_TDP_FLAG_INDEX,           /*! \brief Total Distance Present */
  RSCPS_WRS_FLAG_INDEX,           /*! \brief  Walking or Running Status bits */
  RSCPS_NUM_FLAGS
};

/**************************************************************************************************
Data Types
**************************************************************************************************/

/*! \brief Running Speed Measurement Data */
typedef struct
{
  uint8_t   flags;                  /*! \brief Speed Measurement Flags */
  uint16_t  speed;                  /*! \brief Instantaneous Speed */
  uint8_t   cadence;                /*! \brief Instantaneous Cadence */
  uint16_t  stride;                 /*! \brief Instantaneous Stride Length */
  uint32_t  distance;               /*! \brief Total Distance */
} rscpSmData_t;

/**************************************************************************************************
Local Variables
**************************************************************************************************/

/*! \brief Measurement data */
rscpSmData_t rscpSmData;

/*************************************************************************************************/
/*!
*  \brief  Set the sensor location attribute.
*
*  \param  location   Sensor Location.
*
*  \return none
*/
/*************************************************************************************************/
void RscpsSetSensorLocation(uint8_t location)
{
  AttsSetAttr(RSCS_SL_HDL, sizeof(uint8_t), &location);
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
void RscpsSetFeatures(uint16_t features)
{
  uint8_t tempData[2] = {UINT16_TO_BYTES(features)};
  AttsSetAttr(RSCS_RSF_HDL, sizeof(tempData), tempData);
}

/*************************************************************************************************/
/*!
*  \brief  Set a running speed measurement parameter.
*
*  \param  type   Parameter identifier
*  \param  value  Measurement value.
*
*  \return none
*/
/*************************************************************************************************/
void RscpsSetParameter(uint8_t type, uint32_t value)
{
  switch (type)
  {
  case RSCP_SM_PARAM_SPEED:
    rscpSmData.speed = (uint16_t) value;
    break;

  case RSCP_SM_PARAM_CADENCE:
    rscpSmData.cadence = (uint8_t) value;
    break;

  case RSCP_SM_PARAM_STRIDE_LENGTH:
    rscpSmData.flags |= (1 << RSCPS_ISLP_FLAG_INDEX);
    rscpSmData.stride = (uint16_t) value;
    break;

  case RSCP_SM_PARAM_TOTAL_DISTANCE:
    rscpSmData.flags |= (1 << RSCPS_TDP_FLAG_INDEX);
    rscpSmData.distance = value;
    break;

  case RSCP_SM_PARAM_STATUS:
    if (value)
    {
      rscpSmData.flags |= (1 << RSCPS_WRS_FLAG_INDEX);
    }
    else
    {
      rscpSmData.flags &= ~(1 << RSCPS_WRS_FLAG_INDEX);
    }
    break;

  default:
    break;
  }
}

/*************************************************************************************************/
/*!
*  \brief  Notifies the collector of a Cycle Speed Measurement.
*
*  \param  connId  Connection ID
*
*  \return none
*/
/*************************************************************************************************/
void RscpsSendSpeedMeasurement(dmConnId_t connId)
{
  int8_t i;
  uint16_t len;
  uint8_t msg[RSCPS_PM_MAX_LEN];
  uint8_t *p = msg;

  /* Add manditory parameters */
  UINT8_TO_BSTREAM(p, rscpSmData.flags);
  UINT16_TO_BSTREAM(p, rscpSmData.speed);
  UINT8_TO_BSTREAM(p, rscpSmData.cadence);

  /* Add optional parameters */
  for (i = 0; i < RSCPS_NUM_FLAGS; i++)
  {
    if (rscpSmData.flags & (1 << i))
    {
      switch (i)
      {
      case RSCPS_ISLP_FLAG_INDEX:
        UINT16_TO_BSTREAM(p, rscpSmData.stride);
        break;
      case RSCPS_TDP_FLAG_INDEX:
        UINT32_TO_BSTREAM(p, rscpSmData.distance);
        break;

      default:
        break;
      }
    }
  }

  /* Calculate message length */
  len = (uint16_t) (p - msg);

  /* Transmit notification */
  AttsHandleValueNtf(connId, RSCS_RSM_HDL, len, msg);

  /* Clear the measurement data */
  memset(&rscpSmData, 0, sizeof(rscpSmData));
}
