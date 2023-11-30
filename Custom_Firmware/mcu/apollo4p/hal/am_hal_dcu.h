//*****************************************************************************
//
//! @file am_hal_dcu.h
//!
//! @brief Implementation for Debug Control Unit functionality
//!
//! @addtogroup dcu_4p DCU - Debug Control Unit
//! @ingroup apollo4p_hal
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

#ifndef AM_HAL_DCU_H
#define AM_HAL_DCU_H

#ifdef __cplusplus
extern "C"
{
#endif

#define AM_HAL_DCU_NUMDCU               21 // Number of valid bits

// Qualified DCU Controls
#define AM_HAL_DCU_CPUDBG_INVASIVE      0x00000002UL
#define AM_HAL_DCU_CPUDBG_NON_INVASIVE  0x00000004UL
#define AM_HAL_DCU_CPUTRC_ETM           0x00000008UL
#define AM_HAL_DCU_CPUTRC_ITM           0x00000004UL // Same as NON_INVASIVE
#define AM_HAL_DCU_CPUTRC_TPIU_SWO      0x00000010UL
#define AM_HAL_DCU_CPUTRC_PERFCNT       0x00000020UL
#define AM_HAL_DCU_CACHEDBG             0x00000040UL
#define AM_HAL_DCU_SWD                  0x00000800UL
#define AM_HAL_DCU_ETB                  0x00002000UL
#define AM_HAL_DCU_TRACE                0x00004000UL


// Following macros define Raw DCU Values as in HOSTDCU registers
// These are provided only for reference
// The HAL APIs work only with Qualified values (AM_HAL_DCU_* above)

// 3 bit encoding for individual DCU fields
#define AM_HAL_DCURAWVAL_ENABLE            0x5
#define AM_HAL_DCURAWVAL_DISABLE           0x2
#define AM_HAL_DCURAWVAL_MASK              0x7

// Raw DCU control masks
#define AM_HAL_DCURAW_CPUDBG_INVASIVE      0x0000000000000007ULL
#define AM_HAL_DCURAW_CPUDBG_NON_INVASIVE  0x0000000000000038ULL
#define AM_HAL_DCURAW_CPUTRC_ETM           0x00000000000001C0ULL
#define AM_HAL_DCURAW_CPUTRC_ITM           0x0000000000000038ULL // Same as NON_INVASIVE
#define AM_HAL_DCURAW_CPUTRC_TPIU_SWO      0x0000000000000E00ULL
#define AM_HAL_DCURAW_CPUTRC_PERFCNT       0x0000000000007000ULL
#define AM_HAL_DCURAW_CACHEDBG             0x0000000000038000ULL
#define AM_HAL_DCURAW_SWD                  0x00000001C0000000ULL
#define AM_HAL_DCURAW_ETB                  0x0000007000000000ULL
#define AM_HAL_DCURAW_TRACE                0x0000038000000000ULL


// All possible controls
#define AM_HAL_DCURAW_MASK             0x000003F1C003FFFFULL
#define AM_HAL_DCURAW_ENABLE           0x000002D14002DB6DULL
#define AM_HAL_DCURAW_DISABLE          0x0000012080012492ULL

// Convenience wrappers
//! @param  ui32Mask -  DCU controls to be modified - OR'ing of AM_HAL_DCU_* masks defined above
#define am_hal_dcu_enable(ui32Mask)         am_hal_dcu_update(true, (ui32Mask))
//! @param  ui32Mask -  DCU controls to be modified - OR'ing of AM_HAL_DCU_* masks defined above
#define am_hal_dcu_disable(ui32Mask)        am_hal_dcu_update(false, (ui32Mask))

//*****************************************************************************
//
//! @brief  Read DCU Lock
//!
//! @param  pui32Val -  Pointer to word for returned data (Qualified DCU Mask)
//!
//! This will retrieve the DCU Lock information
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
extern uint32_t am_hal_dcu_lock_status_get(uint32_t *pui32Val);

//*****************************************************************************
//
//! @brief  Write DCU Lock (Qualified Values)
//!
//! @param  ui32Mask -  Mask for lock values
//!
//! This will lock the DCU from further changes
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
extern uint32_t am_hal_dcu_lock(uint32_t ui32Mask);

//*****************************************************************************
//
//! @brief  Read DCU Enables (Qualified Values)
//!
//! @param  pui32Val -  Pointer to Mask for returned data
//!
//! This will get the current DCU Enable settings
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
extern uint32_t am_hal_dcu_get(uint32_t *pui32Val);

//*****************************************************************************
//
//! @brief  Update DCU Enable (Qualified Values)
//!
//! @param  ui32Mask -  DCU controls to be modified
//! @param  bEnable - Whether to enable or disable
//!
//! This will update the DCU Enable settings, if not locked
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
extern uint32_t am_hal_dcu_update(bool bEnable, uint32_t ui32Mask);

//*****************************************************************************
//
//! @brief  DCU Disable - Using MCUCTRL Override
//!
//! @param  ui32Mask -  DCU controls to be modified (Qualified Values)
//!
//! This will update the MCUCTRL DCU Disable Override settings
//! This can only further lock things if the corresponding DCU Enable was open
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
extern uint32_t am_hal_dcu_mcuctrl_override(uint32_t ui32Mask);


#ifdef __cplusplus
}
#endif

#endif // AM_HAL_DCU_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

