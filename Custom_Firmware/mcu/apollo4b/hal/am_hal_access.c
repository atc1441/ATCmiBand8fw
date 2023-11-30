//*****************************************************************************
//
//! @file am_hal_access.c
//!
//! @brief This file controls peripheral access in Apollo4.
//!
//! @addtogroup access_4b Access - Peripheral Access
//! @ingroup apollo4b_hal
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
#include <stdbool.h>
#include "am_mcu_apollo.h"

//*****************************************************************************
//
// Global definitions.
//
//*****************************************************************************

//
//! @brief Access State Struct
//
typedef struct
{
    const am_hal_access_t *psGlobalAccess;
    const uint32_t *Allowed;
    uint32_t *Claimed;
}
am_hal_access_state_t;

//*****************************************************************************
//
//! @name Helper macros for finding the right bit in an access structure.
//! @{
//
//*****************************************************************************
#define AM_HAL_ACCESS_ALLOWED(psAccessStruct, ePeriph)                        \
    ((bool) (psAccessStruct->Allowed[ePeriph >> 5] & (1 << (ePeriph & 0x1F))))

#define AM_HAL_ACCESS_CLAIMED(psAccessStruct, ePeriph)                        \
    ((bool) (psAccessStruct->Claimed[ePeriph >> 5] & (1 << (ePeriph & 0x1F))))

#define AM_HAL_ACCESS_SHARED(psAccess, ePeriph)                               \
    ((bool) (psAccess->psGlobalAccess->pui32Shared[ePeriph >> 5] &            \
             (1 << (ePeriph & 0x1F))))

#define AM_HAL_ACCESS_AVAILABLE(psAccess, ePeriph)                            \
    (((psAccess->psGlobalAccess->pui32MCUClaimed[ePeriph >> 5] |              \
       psAccess->psGlobalAccess->pui32DSP0Claimed[ePeriph >> 5] |             \
       psAccess->psGlobalAccess->pui32DSP1Claimed[ePeriph >> 5])              \
      & (1 << (ePeriph  & 0x1F))) == 0)

#define AM_HAL_ACCESS_CLAIM(psAccessStruct, ePeriph)                          \
    psAccessStruct->Claimed[ePeriph >> 5] |= (1 << (ePeriph & 0x1F))

#define AM_HAL_ACCESS_RELEASE(psAccessStruct, ePeriph)                        \
    psAccessStruct->Claimed[ePeriph >> 5] &= ~(1 << (ePeriph & 0x1F))
//! @}

//*****************************************************************************
//
// Static function prototypes.
//
//*****************************************************************************

//*****************************************************************************
//
// Future functions
//
//*****************************************************************************
#define delay_us(...)

#define get_allowed_structure(psAccess)                                        \
    psAccess->pui32MCUAllowed

#define get_claimed_structure(psAccess)                                        \
    psAccess->pui32MCUClaimed

//*****************************************************************************
//
// Future function for managing peripheral access.
//
//*****************************************************************************
uint32_t
get_access_mutex(void)
{
    return 1;
}

//*****************************************************************************
//
// Future function for managing peripheral access.
//
//*****************************************************************************
static uint32_t
release_access_mutex(void)
{
    return 1;
}

//*****************************************************************************
//
// Initialize the central access structure.
//
//*****************************************************************************
uint32_t
am_hal_access_initialize(void **pvHandle)
{
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Initialize the central access structure.
//
//*****************************************************************************
uint32_t
am_hal_access_deinitialize(void *pvHandle)
{
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Set up the necessary pointers for the acesss structure.
//
//*****************************************************************************
uint32_t
am_hal_access_config(void *pvHandle, am_hal_access_t *psGlobalAccess)
{
    am_hal_access_state_t *psAccess = pvHandle;

    psAccess->psGlobalAccess = psGlobalAccess;
    psAccess->Allowed = get_allowed_structure(psGlobalAccess);
    psAccess->Claimed = get_claimed_structure(psGlobalAccess);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Obtain access to a peripheral.
//
//*****************************************************************************
uint32_t
am_hal_access_check(void *pvHandle,
                    am_hal_access_periph_e ePeripheral)
{
    am_hal_access_state_t *psAccess = pvHandle;

    if (AM_HAL_ACCESS_ALLOWED(psAccess, ePeripheral) &&
        !AM_HAL_ACCESS_SHARED(psAccess, ePeripheral))
    {
        return AM_HAL_STATUS_SUCCESS;
    }
    else if (AM_HAL_ACCESS_ALLOWED(psAccess, ePeripheral) &&
             AM_HAL_ACCESS_CLAIMED(psAccess, ePeripheral))
    {
        return AM_HAL_STATUS_SUCCESS;
    }
    else
    {
        return AM_HAL_STATUS_FAIL;
    }
}

//*****************************************************************************
//
// Obtain access to a peripheral.
//
//*****************************************************************************
uint32_t
am_hal_access_get(void *pvHandle,
                  am_hal_access_periph_e ePeripheral,
                  uint32_t ui32TimeoutUS)
{
    uint32_t i;
    bool bPeriphClaimed;
    am_hal_access_state_t *psAccess = pvHandle;

    if (am_hal_access_check(psAccess, ePeripheral))
    {
        //
        // If we already have access to the peripheral, we don't need to do
        // anything else.
        //
        return AM_HAL_STATUS_SUCCESS;
    }
    else if (AM_HAL_ACCESS_ALLOWED(psAccess, ePeripheral))
    {
        return AM_HAL_ACCESS_NOT_ALLOWED;
    }
    else
    {
        //
        // If we don't have access, we need to try to obtain it. Initialize our
        // tracking variable, and start a loop to keep track of our timeout
        // condition.
        //
        bPeriphClaimed = false;

        for (i = 0; i < ui32TimeoutUS; i++)
        {
            //
            // Make sure we grab the mutex before accessing the data structure.
            //
            if (get_access_mutex())
            {
                //
                // If the peripheral is free, claim it.
                //
                if (AM_HAL_ACCESS_AVAILABLE(psAccess, ePeripheral))
                {
                    AM_HAL_ACCESS_CLAIM(psAccess, ePeripheral);
                    bPeriphClaimed = true;
                }

                release_access_mutex();
            }

            //
            // If we got access, we can exit the loop.
            //
            if (bPeriphClaimed)
            {
                break;
            }

            delay_us(1);
        }

        //
        // Return a status based on whether we were able to obtain access.
        //
        if (bPeriphClaimed)
        {
            return AM_HAL_STATUS_SUCCESS;
        }
        else
        {
            return AM_HAL_STATUS_TIMEOUT;
        }
    }
}

//*****************************************************************************
//
// Release control of a peripheral.
//
//*****************************************************************************
uint32_t
am_hal_access_release(void *pvHandle,
                      am_hal_access_periph_e ePeripheral,
                      uint32_t ui32TimeoutUS)
{
    uint32_t i;
    bool bPeriphFree;
    am_hal_access_state_t *psAccess = pvHandle;

    //
    // If we don't have access, we need to try to obtain it. Initialize our
    // tracking variable, and start a loop to keep track of our timeout
    // condition.
    //
    bPeriphFree = 0;

    for (i = 0; i < ui32TimeoutUS; i++)
    {
        //
        // Make sure we grabe the mutex before accessing the data structure.
        //
        if (get_access_mutex())
        {
            AM_HAL_ACCESS_RELEASE(psAccess, ePeripheral);
            release_access_mutex();
            bPeriphFree = true;
        }

        //
        // If we got access, we can exit the loop.
        //
        if (bPeriphFree)
        {
            break;
        }

        delay_us(1);
    }

    //
    // Return a status based on whether we were able to release the resource.
    //
    if (bPeriphFree)
    {
        return AM_HAL_STATUS_SUCCESS;
    }
    else
    {
        return AM_HAL_STATUS_TIMEOUT;
    }
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
