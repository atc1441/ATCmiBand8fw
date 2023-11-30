/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Application framework module for legacy master.  This module can be used with both
 *          DM legacy and extended scanning and connect.
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

#include "wsf_types.h"
#include "wsf_trace.h"
#include "dm_api.h"
#include "app_api.h"
#include "app_main.h"

/*************************************************************************************************/
/*!
 *  \brief  Check if current scanning mode is legacy scanning.
 *
 *  \return TRUE if legacy scanning mode. FALSE, otherwise.
 */
/*************************************************************************************************/
static bool_t appMasterScanMode(void)
{
  /* legacy master app works with both DM legacy and extended scanning */

  /* if first time since last power-on or reset */
  if (appMasterCb.scanMode == APP_SCAN_MODE_NONE)
  {
    /* set scanning mode to legacy */
    appMasterCb.scanMode = APP_SCAN_MODE_LEG;

    return TRUE;
  }

  if (appMasterCb.scanMode == APP_SCAN_MODE_LEG)
  {
    return TRUE;
  }

  APP_TRACE_WARN0("Invalid DM scanning mode; mode configured as extended");

  return FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Start scanning.   A scan is performed using the given discoverability mode,
 *          scan type, and duration.
 *
 *  \param  mode      Discoverability mode.
 *  \param  scanType  Scan type.
 *  \param  duration  The scan duration, in milliseconds.  If set to zero, scanning will
 *                    continue until AppScanStop() is called.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppScanStart(uint8_t mode, uint8_t scanType, uint16_t duration)
{
  if (appMasterScanMode())
  {
    DmScanSetInterval(HCI_SCAN_PHY_LE_1M_BIT, &pAppMasterCfg->scanInterval, &pAppMasterCfg->scanWindow);

    DmScanStart(HCI_SCAN_PHY_LE_1M_BIT, mode, &scanType, TRUE, duration, 0);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Stop scanning.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppScanStop(void)
{
  if (appMasterScanMode())
  {
    /* stop address resolution */
    appMasterCb.inProgress = FALSE;

    DmScanStop();
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Open a connection to a peer device with the given address.
 *
 *  \param  addrType  Address type.
 *  \param  pAddr     Peer device address.
 *  \param  dbHdl     Device database handle.
 *
 *  \return Connection identifier.
 */
/*************************************************************************************************/
dmConnId_t AppConnOpen(uint8_t addrType, uint8_t *pAddr, appDbHdl_t dbHdl)
{
  if (appMasterScanMode())
  {
    return appConnOpen(HCI_INIT_PHY_LE_1M_BIT, addrType, pAddr, dbHdl);
  }

  /* wrong connect mode */
  return DM_CONN_ID_NONE;
}
