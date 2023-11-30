//*****************************************************************************
//
//! @file am_util_id.c
//!
//! @brief Identification of the Ambiq Micro device.
//!
//! This module contains functions for run time identification of Ambiq Micro
//! devices.
//!
//! @addtogroup id ID - Identification
//! @ingroup utils
//! @{
//
//*****************************************************************************

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
#include <stdint.h>
#include "am_util_id.h"

//*****************************************************************************
//
// Globals.
//
//*****************************************************************************
//
//! Strings for use with pui8VendorName.
//
#if defined(AM_ID_APOLLO)
static const uint8_t g_DeviceNameApollo[]     = "Apollo";
#endif
#if defined(AM_ID_APOLLO2)
static const uint8_t g_DeviceNameApollo2[]    = "Apollo2";
#endif
#if defined(AM_ID_APOLLO3)
static const uint8_t g_DeviceNameApollo3[]    = "Apollo3 Blue";
#endif
#if defined(AM_ID_APOLLO3P)
static const uint8_t g_DeviceNameApollo3p[]   = "Apollo3 Blue Plus";
#endif
#if defined(AM_ID_APOLLO4A)
static const uint8_t g_DeviceNameApollo4[]    = "Apollo4 revA";
#endif
#if defined(AM_ID_APOLLO4B)
static const uint8_t g_DeviceNameApollo4b[]   = "Apollo4b";
#endif
#if defined(AM_ID_APOLLO4P)
static const uint8_t g_DeviceNameApollo4p[]        = "Apollo4 Plus";
static const uint8_t g_DeviceNameApollo4p_blue[]   = "Apollo4 Blue Plus";
static const uint8_t g_PackageType[][4]            = { "SIP", "SIP2", "BGA", "CSP" };
#endif
#if defined(AM_ID_APOLLO4L)
static const uint8_t g_DeviceNameApollo4l[]   = "Apollo4 Lite";
#endif

static const uint8_t g_TempRange[][11] = { "Commercial", "Military", "Automotive", "Industrial" };
static const uint8_t g_ui8VendorNameAmbq[]    = "AMBQ";
static const uint8_t g_ui8VendorNameUnknown[] = "????";
static const uint8_t g_ui8DeviceNameUnknown[] = "Unknown device";

#if !defined(AM_ID_APOLLO) && !defined(AM_ID_APOLLO2)
//*****************************************************************************
// Return the major version of the chip rev.
// Returns: 'A', 'B', 'C', ...
//*****************************************************************************
static uint32_t
revmaj_get(uint32_t ui32ChipRev)
{
    uint32_t ui32ret;

#ifdef _FLD2VAL
    ui32ret = _FLD2VAL(MCUCTRL_CHIPREV_REVMAJ, ui32ChipRev);
#else
    ui32ret = (ui32ChipRev & 0xF0) >> 4;
#endif

    //
    // Major revision is 1=A, 2=B, 3=C, ...
    // Convert to the expected return value.
    //
    return ui32ret + 'A' - 1;

} // revmaj_get()
#endif

//*****************************************************************************
// Update the ID structure with the appropriate ChipRev letter.
// ui32minrevbase should be 0 for Apollo or Apollo2, 1 for Apollo3.
//*****************************************************************************
static void
chiprev_set(am_util_id_t *psIDDevice, uint32_t ui32minrevbase)
{
    uint32_t ui32maj, ui32min;

    ui32maj = ((psIDDevice->sMcuCtrlDevice.ui32ChipRev & 0xF0) >> 4);
    psIDDevice->ui8ChipRevMaj  = (uint8_t)('A' - 1 + ui32maj);

    //
    // For Apollo and Apollo2:  rev0=0, rev1=1, ... (0-based)
    // For Apollo3:             rev0=1, rev1=2, ... (1-based)
    //
    ui32min = ((psIDDevice->sMcuCtrlDevice.ui32ChipRev & 0x0F) >> 0);
    psIDDevice->ui8ChipRevMin = (uint8_t)('0' + ui32min - ui32minrevbase);

} // chiprev_set()

//*****************************************************************************
//
// Device identification.
//
//*****************************************************************************
uint32_t
am_util_id_device(am_util_id_t *psIDDevice)
{
#if !defined(AM_ID_APOLLO) && !defined(AM_ID_APOLLO2)
    uint32_t ui32ChipRev;
#endif
    uint32_t ui32PN;

    //
    // Initialize to default as "unknown".
    //
    psIDDevice->ui32Device = AM_UTIL_ID_UNKNOWN;
    psIDDevice->pui8DeviceName = g_ui8DeviceNameUnknown;
    psIDDevice->pui8VendorName = g_ui8VendorNameUnknown;
    psIDDevice->ui8ChipRevMaj = (uint8_t)'?';
    psIDDevice->ui8ChipRevMin = (uint8_t)' ';

    //
    // Go get all the device (hardware) info from the HAL
    //
#if AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_info_get(AM_HAL_MCUCTRL_INFO_DEVICEID, &psIDDevice->sMcuCtrlDevice);
    am_hal_mcuctrl_info_get(AM_HAL_MCUCTRL_INFO_FEATURES_AVAIL, &psIDDevice->sMcuCtrlFeature);
#else // AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_device_info_get(&psIDDevice->sMcuCtrlDevice);
#endif // AM_APOLLO3_MCUCTRL

    //
    // Device identification
    //
    ui32PN = psIDDevice->sMcuCtrlDevice.ui32ChipPN  &
             AM_UTIL_MCUCTRL_CHIP_INFO_PARTNUM_PN_M;
#if !defined(AM_ID_APOLLO) && !defined(AM_ID_APOLLO2)
    ui32ChipRev = psIDDevice->sMcuCtrlDevice.ui32ChipRev;
#endif

    if ( ( psIDDevice->sMcuCtrlDevice.ui32VendorID ==
            (('A' << 24) | ('M' << 16) | ('B' << 8) | ('Q' << 0)) ) )
    {
        //
        // VENDORID is AMBQ. Set the manufacturer string pointer.
        //
        psIDDevice->pui8VendorName = g_ui8VendorNameAmbq;
    }

#if defined(AM_ID_APOLLO)
    //
    // Apollo1 did not support VENDORID.
    // Do a specific check from JEDEC values to verify Ambiq as the vendor.
    //
    if ( ((psIDDevice->sMcuCtrlDevice.ui32JedecCID   == 0xB105100D)     &&
         (psIDDevice->sMcuCtrlDevice.ui32JedecJEPID == 0x0000009B)      &&
         ((psIDDevice->sMcuCtrlDevice.ui32JedecPN & 0xF00) == 0xE00)) )
    {
        //
        // VENDORID is AMBQ. Set the manufacturer string pointer.
        //
        psIDDevice->pui8VendorName = g_ui8VendorNameAmbq;
    }

    if ( ( ui32PN == AM_UTIL_MCUCTRL_CHIP_INFO_PARTNUM_APOLLO )             &&
         ((psIDDevice->sMcuCtrlDevice.ui32JedecPN & 0x0F0) == 0x0E0) )
    {
        psIDDevice->ui32Device = AM_UTIL_ID_APOLLO;
        psIDDevice->pui8DeviceName = g_DeviceNameApollo;
        chiprev_set(psIDDevice, 0);
    }
#endif // AM_ID_APOLLO

#if defined(AM_ID_APOLLO2)
    if ( ( ui32PN == AM_UTIL_MCUCTRL_CHIP_INFO_PARTNUM_APOLLO2 )            &&
              ((psIDDevice->sMcuCtrlDevice.ui32JedecPN & 0x0F0) == 0x0D0) )
    {
        psIDDevice->ui32Device = AM_UTIL_ID_APOLLO2;
        psIDDevice->pui8DeviceName = g_DeviceNameApollo2;
        chiprev_set(psIDDevice, 0);
    }
#endif // AM_ID_APOLLO2

#if defined(AM_ID_APOLLO3)
    if ( ( ui32PN == AM_UTIL_MCUCTRL_CHIP_INFO_PARTNUM_APOLLO3 )            &&
              ((psIDDevice->sMcuCtrlDevice.ui32JedecPN & 0x0F0) == 0x0C0)   &&
              ( revmaj_get(ui32ChipRev) <= 'B' ) )
    {
        psIDDevice->ui32Device = AM_UTIL_ID_APOLLO3;
        psIDDevice->pui8DeviceName = g_DeviceNameApollo3;
        chiprev_set(psIDDevice, 1);
    }
#endif // AM_ID_APOLLO3

#if defined(AM_ID_APOLLO3P)
    if ( ( ui32PN == AM_UTIL_MCUCTRL_CHIP_INFO_PARTNUM_APOLLO3P)            &&
              ((psIDDevice->sMcuCtrlDevice.ui32JedecPN & 0x0F0) == 0x0C0)   &&
              ( revmaj_get(ui32ChipRev) == 'C' ) )
    {
        psIDDevice->ui32Device = AM_UTIL_ID_APOLLO3P;
        psIDDevice->pui8DeviceName = g_DeviceNameApollo3p;
        chiprev_set(psIDDevice, 1);
    }
#endif // AM_ID_APOLLO3P

#if defined(AM_ID_APOLLO4A)
    if ( ( ui32PN == AM_UTIL_MCUCTRL_CHIP_INFO_PARTNUM_APOLLO4)             &&
              ((psIDDevice->sMcuCtrlDevice.ui32JedecPN & 0x0F0) == 0x0B0)   &&
               ( revmaj_get(ui32ChipRev) == 'A' ) )
    {
        psIDDevice->ui32Device = AM_UTIL_ID_APOLLO4;
        psIDDevice->pui8DeviceName = g_DeviceNameApollo4;
        chiprev_set(psIDDevice, 1);
    }
#endif // AM_ID_APOLLO4A

#if defined(AM_ID_APOLLO4B)
    if ( ( ui32PN == AM_UTIL_MCUCTRL_CHIP_INFO_PARTNUM_APOLLO4)             &&
              ((psIDDevice->sMcuCtrlDevice.ui32JedecPN & 0x0F0) == 0x0B0)   &&
               ( revmaj_get(ui32ChipRev) == 'B' ) )
    {
        psIDDevice->ui32Device = AM_UTIL_ID_APOLLO4;
        psIDDevice->pui8DeviceName = g_DeviceNameApollo4b;
        chiprev_set(psIDDevice, 1);
    }
#endif // AM_ID_APOLLO4B

#if defined(AM_ID_APOLLO4P)
    if ( ( ui32PN == AM_UTIL_MCUCTRL_CHIP_INFO_PARTNUM_APOLLO4)             &&
              ((psIDDevice->sMcuCtrlDevice.ui32JedecPN & 0x0F0) == 0x0B0)   &&
              ( revmaj_get(ui32ChipRev) == 'C' ) )
    {
        psIDDevice->ui32Device = AM_UTIL_ID_APOLLO4P;
        if ( ((psIDDevice->sMcuCtrlDevice.ui32ChipPN & 0xc0) >> 6) >= 2 )
        {
            psIDDevice->pui8DeviceName = g_DeviceNameApollo4p;
        }
        else
        {
            psIDDevice->pui8DeviceName = g_DeviceNameApollo4p_blue;
        }
        chiprev_set(psIDDevice, 1);
    }
#endif // AM_ID_APOLLO4P

#if defined(AM_ID_APOLLO4L)
    if ( ( ui32PN == AM_UTIL_MCUCTRL_CHIP_INFO_PARTNUM_APOLLO4L)             &&
              ((psIDDevice->sMcuCtrlDevice.ui32JedecPN & 0x0F0) == 0x0B0)   &&
              ( revmaj_get(ui32ChipRev) == 'A' ) )
    {
        psIDDevice->ui32Device = AM_UTIL_ID_APOLLO4L;
        psIDDevice->pui8DeviceName = g_DeviceNameApollo4l;
        chiprev_set(psIDDevice, 1);
    }
#endif // AM_ID_APOLLO4L


    //
    // This section defines the package type
    //
    // currently this is only defined for the Apollo4 Plus / Blue Plus
    //
#if defined(AM_PART_APOLLO4P)
    psIDDevice->pui8PackageType = g_PackageType[((psIDDevice->sMcuCtrlDevice.ui32ChipPN & 0xC0) >> 6)];
#else
    psIDDevice->pui8PackageType = NULL;
#endif

    psIDDevice->pui8TempRange = g_TempRange[((psIDDevice->sMcuCtrlDevice.ui32ChipPN & 0x06) >> 1)];

    return psIDDevice->ui32Device;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

