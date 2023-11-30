//*****************************************************************************
//
//! @file am_hal_cachectrl.c
//!
//! @brief Functions for interfacing with the CACHE controller.
//!
//! @addtogroup cachectrl4_4b CACHE - Cache Control
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

// State Variables used to manage DAXI flush
static uint32_t *p64BAxiMem;
static uint32_t ui32AxiBufPwrStatMask;
static uint32_t ui32AxiBufPwrStatReg;

extern uint32_t gMspiXipState;

typedef struct
{
    uint32_t baseAddr;
    uint32_t maxSize;
    uint32_t ui32PwrStatReg;
    uint32_t ui32PwrStatMask;
} am_hal_memrange_t;

static am_hal_memrange_t gAxiMemRange[] =
{
    {SSRAM0_BASEADDR, SSRAM0_MAX_SIZE, AM_REGADDR(PWRCTRL, SSRAMPWRST), 0x1},
    {SSRAM1_BASEADDR, SSRAM1_MAX_SIZE, AM_REGADDR(PWRCTRL, SSRAMPWRST), 0x2},
    {DSP0IRAM_BASEADDR, DSP0IRAM_MAX_SIZE, AM_REGADDR(PWRCTRL, DSP0MEMPWRST), PWRCTRL_DSP0MEMPWRST_PWRSTDSP0ICACHE_Msk},
    {DSP0DRAM_BASEADDR, DSP0DRAM_MAX_SIZE, AM_REGADDR(PWRCTRL, DSP0MEMPWRST), PWRCTRL_DSP0MEMPWRST_PWRSTDSP0RAM_Msk},
    {DSP1IRAM_BASEADDR, DSP1IRAM_MAX_SIZE, AM_REGADDR(PWRCTRL, DSP1MEMPWRST), PWRCTRL_DSP1MEMPWRST_PWRSTDSP1ICACHE_Msk},
    {DSP1DRAM_BASEADDR, DSP1DRAM_MAX_SIZE, AM_REGADDR(PWRCTRL, DSP1MEMPWRST), PWRCTRL_DSP1MEMPWRST_PWRSTDSP1RAM_Msk},
};

//*****************************************************************************
//
//  Default settings for the cache.
//
//*****************************************************************************
const am_hal_cachectrl_config_t am_hal_cachectrl_defaults =
{
    .bLRU                       = 0,
    .eDescript                  = AM_HAL_CACHECTRL_DESCR_1WAY_128B_4096E,
    .eMode                      = AM_HAL_CACHECTRL_CONFIG_MODE_INSTR_DATA,
};

//*****************************************************************************
//
//  Default settings for the DAXI.
//
//*****************************************************************************
const am_hal_daxi_config_t am_hal_daxi_defaults =
{
    .agingCounter             = 2,
    .eNumBuf                  = AM_HAL_DAXI_CONFIG_NUMBUF_1,
    .eNumFreeBuf              = AM_HAL_DAXI_CONFIG_NUMFREEBUF_2,
};

static void
daxi_flush_invalidate(bool bFlush)
{
    // Call DSB
    __DSB();

    if (bFlush)
    {
        // Call DAXI Flush
        CPU->DAXICTRL_b.DAXIFLUSHWRITE = 1;
        // APB_SYNC
        am_hal_sysctrl_sysbus_write_flush();
    }

    // Call DAXI Invalidate
    CPU->DAXICTRL_b.DAXIINVALIDATE = 1;

    // APB_SYNC
    am_hal_sysctrl_sysbus_write_flush();

} // daxi_flush_invalidate()

//*****************************************************************************
//
// Write flush - This function will return once all queued write
// operations have completed, thereby guaranteeing that all
// writes have been flushed.
// This works across all the buses - AXI and APB
//
//*****************************************************************************
static void
daxi_flush_complete(void)
{
    AM_CRITICAL_BEGIN
    daxi_flush_invalidate(true);
    // flush any buffered core and peripheral writes.
    if (p64BAxiMem && ((AM_REGVAL(ui32AxiBufPwrStatReg) & ui32AxiBufPwrStatMask) == ui32AxiBufPwrStatMask))
    {
        if (gMspiXipState)
        {
            //
            // Additional processing if MXPI XIP is enabled
            // Wait for AXI FIFO buffers to be flushed out
            //
            for (uint32_t i = 0; i < 4; i++)
            {
                AM_REGVAL((uint32_t)p64BAxiMem + i*16) = 0;
            }
            daxi_flush_invalidate(true);
            for (uint32_t i = 0; i < 4; i++)
            {
                AM_REGVAL((uint32_t)p64BAxiMem + i*16);
            }
        }

        //
        // Insert 300 cycles worth of NOPs here to handle worse-case SSRAM busy with GFX/Disp/DMA,
        // but do it in such a way so as to keep DAXI clock active, doing repeated reads
        //
        for (uint32_t i = 0; i < 30; i++)
        {
            AM_REGVAL((uint32_t)p64BAxiMem);
            AM_REGVAL((uint32_t)p64BAxiMem);
            AM_REGVAL((uint32_t)p64BAxiMem);
            AM_REGVAL((uint32_t)p64BAxiMem);
            AM_REGVAL((uint32_t)p64BAxiMem);
            AM_REGVAL((uint32_t)p64BAxiMem);
        }
        daxi_flush_invalidate(false);
    }
    AM_CRITICAL_END
} // daxi_flush_complete()

//*****************************************************************************
//
//  Configure the cache with given and recommended settings, but do not enable.
//
//*****************************************************************************
uint32_t
am_hal_cachectrl_config(const am_hal_cachectrl_config_t *psConfig)
{
    //
    // In the case where cache is currently enabled, we need to gracefully
    // bow out of that configuration before reconfiguring.  The best way to
    // accomplish that is to shut down the ID bits, leaving the cache enabled.
    // Once the instr and data caches have been disabled, we can safely set
    // any new configuration, including disabling the controller.
    //
    AM_CRITICAL_BEGIN
    CPU->CACHECFG &=
        ~(CPU_CACHECFG_DENABLE_Msk  |
          CPU_CACHECFG_IENABLE_Msk);
    AM_CRITICAL_END

    CPU->CACHECFG =
        _VAL2FLD(CPU_CACHECFG_ENABLE, 0)                              |
        _VAL2FLD(CPU_CACHECFG_CLKGATE, 1)                             |
        _VAL2FLD(CPU_CACHECFG_LS, 0)                                  |
        _VAL2FLD(CPU_CACHECFG_DATACLKGATE, 1)                         |
        _VAL2FLD(CPU_CACHECFG_ENABLEMONITOR, 0)                       |
        _VAL2FLD(CPU_CACHECFG_LRU, psConfig->bLRU)                    |
        _VAL2FLD(CPU_CACHECFG_CONFIG, psConfig->eDescript)            |
        ((psConfig->eMode << CPU_CACHECFG_IENABLE_Pos) &
            (CPU_CACHECFG_DENABLE_Msk   |
             CPU_CACHECFG_IENABLE_Msk));

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_cachectrl_config()

//*****************************************************************************
//
//  Enable the cache.
//
//*****************************************************************************
uint32_t
am_hal_cachectrl_enable(void)
{
    //
    // Enable the cache
    //
    CPU->CACHECFG |= _VAL2FLD(CPU_CACHECFG_ENABLE, 1);
    CPU->CACHECTRL |= _VAL2FLD(CPU_CACHECTRL_INVALIDATE, 1);

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_cachectrl_enable()

//*****************************************************************************
//
//  Disable the cache.
//
//*****************************************************************************
uint32_t
am_hal_cachectrl_disable(void)
{
    //
    // Shut down as gracefully as possible.
    // Disable the I/D cache enable bits first to allow a little time
    // for any in-flight transactions to hand off to the line buffer.
    // Then clear the enable.
    //
    AM_CRITICAL_BEGIN
    CPU->CACHECFG &= ~(_VAL2FLD(CPU_CACHECFG_IENABLE, 1) |
                       _VAL2FLD(CPU_CACHECFG_DENABLE, 1));
    CPU->CACHECFG &= ~_VAL2FLD(CPU_CACHECFG_ENABLE, 1);
    AM_CRITICAL_END

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_cachectrl_disable()

//*****************************************************************************
//
//  Select the cache configuration type.
//
//*****************************************************************************
uint32_t
am_hal_cachectrl_control(am_hal_cachectrl_control_e eControl, void *pArgs)
{
    //
    // All fields in the CACHECTRL register are write-only or read-only.
    // A write to CACHECTRL acts as a mask-set.  That is, only the bits
    // written as '1' have an effect, any bits written as '0' are unaffected.
    //
    // Important note - setting of an enable and disable simultanously has
    // unpredicable results.
    //
    switch ( eControl )
    {
        case AM_HAL_CACHECTRL_CONTROL_MRAM_CACHE_INVALIDATE:
            CPU->CACHECTRL = CPU_CACHECTRL_INVALIDATE_Msk;
            am_hal_sysctrl_sysbus_write_flush();
            return am_hal_delay_us_status_change(100, (uint32_t)&CPU->CACHECTRL,
                                                 CPU_CACHECTRL_CACHEREADY_Msk,
                                                 CPU_CACHECTRL_CACHEREADY_Msk);
            //break;      // Unreachable statement

        case AM_HAL_CACHECTRL_CONTROL_STATISTICS_RESET:
            if ( !_FLD2VAL(CPU_CACHECFG_ENABLEMONITOR, CPU->CACHECFG) )
            {
                //
                // The monitor must be enabled for the reset to have any affect.
                //
                return AM_HAL_STATUS_INVALID_OPERATION;
            }
            else
            {
                CPU->CACHECTRL = CPU_CACHECTRL_RESETSTAT_Msk;
                am_hal_sysctrl_sysbus_write_flush();
            }
            break;

        case AM_HAL_CACHECTRL_CONTROL_MONITOR_ENABLE:
            AM_CRITICAL_BEGIN
            CPU->CACHECFG |= CPU_CACHECFG_ENABLEMONITOR_Msk;
            am_hal_sysctrl_sysbus_write_flush();
            AM_CRITICAL_END
            break;

        case AM_HAL_CACHECTRL_CONTROL_MONITOR_DISABLE:
            AM_CRITICAL_BEGIN
            CPU->CACHECFG &= ~CPU_CACHECFG_ENABLEMONITOR_Msk;
            am_hal_sysctrl_sysbus_write_flush();
            AM_CRITICAL_END
            break;

        case AM_HAL_CACHECTRL_CONTROL_NC_CFG:
        {
            if ( pArgs == NULL )
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }

            am_hal_cachectrl_nc_cfg_t *pNcCfg;
            pNcCfg = (am_hal_cachectrl_nc_cfg_t *)pArgs;
#ifndef AM_HAL_DISABLE_API_VALIDATION
            // Make sure the addresses are valid
            if ((pNcCfg->ui32StartAddr & ~CPU_NCR0START_ADDR_Msk) ||
                (pNcCfg->ui32EndAddr & ~CPU_NCR0START_ADDR_Msk))
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
#endif // AM_HAL_DISABLE_API_VALIDATION
            if (pNcCfg->eNCRegion == AM_HAL_CACHECTRL_NCR0)
            {
                CPU->NCR0START = pNcCfg->ui32StartAddr;
                CPU->NCR0END   = pNcCfg->ui32EndAddr;
                CPU->CACHECFG_b.NC0ENABLE = pNcCfg->bEnable;
            }
            else if (pNcCfg->eNCRegion == AM_HAL_CACHECTRL_NCR1)
            {
                CPU->NCR1START = pNcCfg->ui32StartAddr;
                CPU->NCR1END   = pNcCfg->ui32EndAddr;
                CPU->CACHECFG_b.NC1ENABLE = pNcCfg->bEnable;
            }
#ifndef AM_HAL_DISABLE_API_VALIDATION
            else
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
#endif // AM_HAL_DISABLE_API_VALIDATION
            am_hal_sysctrl_sysbus_write_flush();
            return AM_HAL_STATUS_SUCCESS;
        }

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_cachectrl_control()

//*****************************************************************************
//
//  Cache controller status function
//
//*****************************************************************************
uint32_t
am_hal_cachectrl_status_get(am_hal_cachectrl_status_t *psStatus)
{
    if ( psStatus == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Cache Ready Status
    //
    psStatus->bCacheReady = CPU->CACHECTRL_b.CACHEREADY;

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_cachectrl_status_get()

//*****************************************************************************
//
//  Configure the DAXI with given settings.
//  psConfig of NULL implies DAXI in pass through mode
//
//*****************************************************************************
uint32_t
am_hal_daxi_config(const am_hal_daxi_config_t *psConfig)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (!psConfig)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    // Only flushlevel 0 along with 1 or 2 buffers is supported on Apollo4
    if ((psConfig->eNumFreeBuf != AM_HAL_DAXI_CONFIG_NUMFREEBUF_2) ||  (psConfig->eNumBuf > AM_HAL_DAXI_CONFIG_NUMBUF_2))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif
    AM_CRITICAL_BEGIN
    am_hal_sysctrl_bus_write_flush();
    CPU->DAXICFG =
        _VAL2FLD(CPU_DAXICFG_FLUSHLEVEL, psConfig->eNumFreeBuf) |
        _VAL2FLD(CPU_DAXICFG_BUFFERENABLE, psConfig->eNumBuf)   |
        _VAL2FLD(CPU_DAXICFG_AGINGCOUNTER, psConfig->agingCounter);
    // APB_SYNC
    am_hal_sysctrl_sysbus_write_flush();
    AM_CRITICAL_END
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Misc. DAXI controls.
//
//*****************************************************************************
uint32_t
am_hal_daxi_control(am_hal_daxi_control_e eControl, void *pArgs)
{
    switch ( eControl )
    {
        case AM_HAL_DAXI_CONTROL_FLUSH:
            daxi_flush_complete();
            break;
        case AM_HAL_DAXI_CONTROL_INVALIDATE:
            daxi_flush_invalidate(false);
            break;
        case AM_HAL_DAXI_CONTROL_AXIMEM:
        {
            uint32_t addr = (uint32_t)pArgs;
            uint32_t i;
            if ( pArgs == NULL )
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            if (addr & 0xF)
            {
                // Need to make sure that the buffer is 16B aligned
                return AM_HAL_STATUS_INVALID_ARG;
            }
            for (i = 0; i < sizeof(gAxiMemRange) / sizeof(am_hal_memrange_t); i++)
            {
                // Determine the corresponding Power Status bitmask
                if ((addr >= gAxiMemRange[i].baseAddr) && ((addr + 64) < (gAxiMemRange[i].baseAddr + gAxiMemRange[i].maxSize)))
                {
                    ui32AxiBufPwrStatReg = gAxiMemRange[i].ui32PwrStatReg;
                    ui32AxiBufPwrStatMask = gAxiMemRange[i].ui32PwrStatMask;
                    p64BAxiMem = (uint32_t *)addr;
                    break;
                }
            }
            if (i == sizeof(gAxiMemRange) / sizeof(am_hal_memrange_t))
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            break;
        }
        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Get the current DAXI settings.
//
//*****************************************************************************
uint32_t
am_hal_daxi_config_get(am_hal_daxi_config_t *psConfig)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (!psConfig)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif
    psConfig->agingCounter = _FLD2VAL(CPU_DAXICFG_AGINGCOUNTER, CPU->DAXICFG);
    psConfig->eNumBuf = (am_hal_daxi_config_numbuf_e)_FLD2VAL(CPU_DAXICFG_BUFFERENABLE, CPU->DAXICFG);
    psConfig->eNumFreeBuf = (am_hal_daxi_config_numfreebuf_e)_FLD2VAL(CPU_DAXICFG_FLUSHLEVEL, CPU->DAXICFG);
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
