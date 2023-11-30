/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Wireless Data Exchange profile implementation - Device PHY Configuration.
 *
 *  Copyright (c) 2013-2018 Arm Ltd. All Rights Reserved.
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
#include "util/wstr.h"
#include "wsf_trace.h"
#include "wsf_assert.h"
#include "wsf_efs.h"
#include "util/bstream.h"
#include "svc_wdxs.h"
#include "wdxs_api.h"
#include "wdxs_main.h"
#include "dm_api.h"
#include "app_api.h"
#include "app_hw.h"

#if WDXS_DC_ENABLED == TRUE

/*************************************************************************************************/
/*!
*  \brief  Process set PHY request.
*
*  \return ATT status.
*/
/*************************************************************************************************/
static uint8_t wdxsDcSetPhyReq(dmConnId_t connId, uint16_t len, uint8_t *pValue)
{
  uint8_t allPhys;
  uint8_t txPhys;
  uint8_t rxPhys;
  uint16_t phyOptions;

  /* verify parameter length */
  if (len != WDX_DC_LEN_PHY_UPDATE_REQ)
  {
    return ATT_ERR_LENGTH;
  }

  /* parse parameters */
  BSTREAM_TO_UINT8(allPhys, pValue);
  BSTREAM_TO_UINT8(txPhys, pValue);
  BSTREAM_TO_UINT8(rxPhys, pValue);
  BSTREAM_TO_UINT16(phyOptions, pValue);

  /* request update to PHY */
  DmSetPhy(connId, allPhys, txPhys, rxPhys, phyOptions);

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Process a Get PHY request.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t wdxsDcGetPhy(dmConnId_t connId, uint16_t len, uint8_t *pValue)
{
  return wdxsDcUpdatePhy(connId, HCI_SUCCESS);
}

/*************************************************************************************************/
/*!
 *  \brief  Process a PHY write to the device configuration characteristic.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
static uint8_t wdxsDcPhyWrite(dmConnId_t connId, uint8_t op, uint8_t id, uint16_t len, uint8_t *pValue)
{
  uint8_t status;

  /* set operation */
  if (op == WDX_DC_OP_SET)
  {
    switch (id)
    {
      case WDX_DC_ID_PHY_UPDATE_REQ:
        status = wdxsDcSetPhyReq(connId, len, pValue);
        break;

     default:
        status = ATT_ERR_RANGE;
        break;
     }
  }
  /* get operation */
  else if (op == WDX_DC_OP_GET)
  {
    switch (id)
    {
      case WDX_DC_ID_PHY:
        status = wdxsDcGetPhy(connId, len, pValue);
        break;

      default:
        status = ATT_ERR_RANGE;
        break;
    }
  }
  else
  {
    status = ATT_ERR_RANGE;
  }

  return status;
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize WDXS Device Configuration PHY.
 *
 *  \param  None
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsPhyInit(void)
{
  /* register device configuration phy write callback */
  wdxsDcPhyRegister(wdxsDcPhyWrite);
}

#endif  /* WDXS_DC_ENABLED */
