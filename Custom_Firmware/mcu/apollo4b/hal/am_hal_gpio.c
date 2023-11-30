//*****************************************************************************
//
//! @file am_hal_gpio.c
//!
//! @brief General Purpose Input Output Functionality
//!
//! @addtogroup gpio_4b GPIO - General Purpose Input Output
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
//! @name Default settings for GPIOs.
//! @{
//
//*****************************************************************************
const am_hal_gpio_pincfg_t am_hal_gpio_pincfg_default           = AM_HAL_GPIO_PINCFG_DEFAULT;
const am_hal_gpio_pincfg_t am_hal_gpio_pincfg_output            = AM_HAL_GPIO_PINCFG_OUTPUT;
const am_hal_gpio_pincfg_t am_hal_gpio_pincfg_input             = AM_HAL_GPIO_PINCFG_INPUT;
const am_hal_gpio_pincfg_t am_hal_gpio_pincfg_tristate          = AM_HAL_GPIO_PINCFG_TRISTATE;
const am_hal_gpio_pincfg_t am_hal_gpio_pincfg_opendrain         = AM_HAL_GPIO_PINCFG_OPENDRAIN;
const am_hal_gpio_pincfg_t am_hal_gpio_pincfg_disabled          = AM_HAL_GPIO_PINCFG_DISABLED;
const am_hal_gpio_pincfg_t am_hal_gpio_pincfg_pulledup_disabled = AM_HAL_GPIO_PINCFG_PULLEDUP_DISABLED;
const am_hal_gpio_pincfg_t am_hal_gpio_pincfg_output_with_read  = AM_HAL_GPIO_PINCFG_OUTPUT_WITH_READ;
//! @}

//*****************************************************************************
//
//! Helper Macro
//
//*****************************************************************************
#define AM_HAL_GPIO_CONFIGn(n) (((uint32_t *)(&GPIO))[n])

//*****************************************************************************
//
//! @name Array of function pointers for handling GPIO interrupts.
//! @{
//
//*****************************************************************************
static am_hal_gpio_handler_t gpio_ppfnHandlers[GPIO_NUM_IRQS][32];
static void *gpio_pppvIrqArgs[GPIO_NUM_IRQS][32];
//! @}

//*****************************************************************************
//
// Returns the actual function select number that corresponds to the requested
// feature on the requested pin.
//
//*****************************************************************************
static uint32_t
gpio_funcsel_find(uint32_t ui32GpioNum, uint32_t ui32FunctionEnum,
                  uint32_t *ui32Fsel)
{
    uint32_t ux = 0;
    uint32_t ui32ReturnValue = AM_HAL_GPIO_PIN_FUNCTION_DOES_NOT_EXIST;
    uint16_t ui16FuncEnum;

    if ( (ui32GpioNum >= AM_HAL_PIN_TOTAL_GPIOS)     ||
         (ui32FunctionEnum >= (1 << 16)) )
    {
        return ui32ReturnValue;
    }

    ui16FuncEnum = ui32FunctionEnum;

    //
    // Search through our list of known functions for the selected pin, and see
    // if any of them match the function that the caller requested.
    //
    ux = 0;
    for ( ux = 0; ux < AM_HAL_PIN_NUMFUNCS; ux++ )
    {
        //
        // If we find a matching enum value for the requested function, we can
        // return its index. If not, we'll just default to returning a "does
        // not exist" error.
        //
        if ( am_hal_pin_fn_list[ui32GpioNum][ux] == ui16FuncEnum )
        {
            *ui32Fsel = ux;
            ui32ReturnValue = AM_HAL_STATUS_SUCCESS;
            break;
        }
    }

    return ui32ReturnValue;
}


//
// This function computes common GPIO interrupt register offsets.
//
static uint32_t
gpionum_intreg_index_get(uint32_t ui32Gpionum,
                         uint32_t *pui32RegIdx,
                         uint32_t *pui32Msk)
{

    *pui32RegIdx = ui32Gpionum / 32;
    *pui32Msk = 1 << (ui32Gpionum & 0x1F);
    return AM_HAL_STATUS_SUCCESS;
} // gpionum_intreg_index_get()

//*****************************************************************************
//
// Return the current configuration of a pin.
//
//*****************************************************************************
uint32_t
am_hal_gpio_pinconfig_get(uint32_t ui32GpioNum, am_hal_gpio_pincfg_t* psGpioCfg)
{
    volatile uint32_t *pui32Config = &GPIO->PINCFG0;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( ui32GpioNum >= AM_HAL_PIN_TOTAL_GPIOS )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    if ( psGpioCfg == (am_hal_gpio_pincfg_t*)0x0 )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif /// AM_HAL_DISABLE_API_VALIDATION

    psGpioCfg->GP.cfg = pui32Config[ui32GpioNum];

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_gpio_pinconfig_get()

//*****************************************************************************
//
// Configure the function of a single pin.
//
//*****************************************************************************
//
// Apollo4 Rev B allows additional drive strengths for the following
// (non-virtual) pins: 5-10, 22-27, 31-49, 51-57, 61-88.
// The last physical pad is 1 less than AM_HAL_PIN_VIRTUAL_FIRST (0-based).
//
#ifndef AM_HAL_DISABLE_API_VALIDATION
static const uint32_t
g_ui32DSpintbl[((AM_HAL_PIN_VIRTUAL_FIRST - 1) + 32) / 32] =
{
    0x8FC007E0,     // [31:0]:   31, 22-27, 5-10
    0xE3FBFFFF,     // [63:32]:  61-63, 51-57, 32-49
    0x01FFFFFF,     // [95:64]:  64-88
    0x00000000      // [104:96]:
};
#endif // AM_HAL_DISABLE_API_VALIDATION

uint32_t
am_hal_gpio_pinconfig(uint32_t ui32GpioNum, const am_hal_gpio_pincfg_t sGpioCfg)
{
    volatile uint32_t *pui32Config = &GPIO->PINCFG0;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( ui32GpioNum >= AM_HAL_PIN_TOTAL_GPIOS )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    if ( sGpioCfg.GP.cfg_b.eDriveStrength > AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P5X )
    {
        //
        // Make sure the given pin supports this drive strength
        //
        if ( (g_ui32DSpintbl[ui32GpioNum / 32] & (1 << (ui32GpioNum % 32))) == 0 )
        {
            return AM_HAL_STATUS_INVALID_OPERATION;
        }
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Set the key to enable GPIO configuration.
    //
    GPIO->PADKEY = GPIO_PADKEY_PADKEY_Key;

    //
    // Write the configuration directly to the register.
    //
    pui32Config[ui32GpioNum] = sGpioCfg.GP.cfg;

    //
    // Lock the GPIO register again.
    //
    GPIO->PADKEY = 0;


    return AM_HAL_STATUS_SUCCESS;

} // am_hal_gpio_pinconfig()

//*****************************************************************************
//
// Configure the function of a single pin.
//
//*****************************************************************************
uint32_t
am_hal_gpio_pinconfig_override(uint32_t ui32GpioNum,
                               am_hal_gpio_pincfg_t sGpioCfg,
                               am_hal_pin_function_e eFunction)
{
    uint32_t ui32Ret;
    uint32_t ui32Fsel = 0;

    if ( ui32GpioNum >= AM_HAL_PIN_TOTAL_GPIOS )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Resolve the function select number using our list of functions.
    //
    ui32Ret = gpio_funcsel_find(ui32GpioNum, eFunction, &ui32Fsel);
    if ( ui32Ret != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Ret;
    }

    //
    // Place the new Fsel into a modified GPIO configuration structure.
    //
    am_hal_gpio_pincfg_t sModifiedConfig = sGpioCfg;
    sModifiedConfig.GP.cfg_b.uFuncSel = ui32Fsel;

    //
    // Run the standard pinconfig function on our modified structure.
    //
    am_hal_gpio_pinconfig(ui32GpioNum, sModifiedConfig);

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_gpio_pinconfig_override()

//*****************************************************************************
//
// Read GPIO state values
//
//*****************************************************************************
uint32_t
am_hal_gpio_state_read(uint32_t ui32GpioNum, am_hal_gpio_read_type_e eReadType,
                       uint32_t *pui32ReadState)
{
    volatile uint32_t *pui32Target;

    //
    // Find the correct register to read based on the read type input. Each of
    // these registers map exactly one bit to each pin, so the calculation for
    // which register to use is simple.
    //
    switch (eReadType)
    {
        case AM_HAL_GPIO_INPUT_READ:
            pui32Target = AM_HAL_GPIO_RDn(ui32GpioNum);
            break;

        case AM_HAL_GPIO_OUTPUT_READ:
            pui32Target = AM_HAL_GPIO_WTn(ui32GpioNum);
            break;

        case AM_HAL_GPIO_ENABLE_READ:
            pui32Target = AM_HAL_GPIO_ENn(ui32GpioNum);
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // We need to do one more shift and mask to get to the specific bit we want
    // for the chosen pin. Return the value to the caller through the read
    // state variable.
    //
    *pui32ReadState = (*pui32Target >> (ui32GpioNum % 32)) & 1;

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_gpio_state_read()

//*****************************************************************************
//
// Write GPIO state values
//
//*****************************************************************************
uint32_t
am_hal_gpio_state_write(uint32_t ui32GpioNum, am_hal_gpio_write_type_e eWriteType)
{
    //
    // Find the correct register to read based on the read type input. Each of
    // these registers map exactly one bit to each pin, so the calculation for
    // which register to use is simple.
    //
    switch (eWriteType)
    {
        case AM_HAL_GPIO_OUTPUT_CLEAR:
            am_hal_gpio_output_clear(ui32GpioNum);
            break;

        case AM_HAL_GPIO_OUTPUT_SET:
            am_hal_gpio_output_set(ui32GpioNum);
            break;

        case AM_HAL_GPIO_OUTPUT_TOGGLE:
            am_hal_gpio_output_toggle(ui32GpioNum);
            break;

        case AM_HAL_GPIO_OUTPUT_TRISTATE_OUTPUT_DIS:
            am_hal_gpio_output_tristate_output_dis(ui32GpioNum);
            break;

        case AM_HAL_GPIO_OUTPUT_TRISTATE_OUTPUT_EN:
            am_hal_gpio_output_tristate_output_en(ui32GpioNum);
            break;

        case AM_HAL_GPIO_OUTPUT_TRISTATE_OUTPUT_TOG:
            am_hal_gpio_output_tristate_output_tog(ui32GpioNum);
            break;
    }

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_gpio_state_write()

//*****************************************************************************
//
// GPIO Interrupt control.
// This function performs interrupt enabling, disabling, clear on
// the various combinations of interrupt priority levels.
//
//*****************************************************************************
uint32_t
am_hal_gpio_interrupt_control(am_hal_gpio_int_channel_e eChannel,
                              am_hal_gpio_int_ctrl_e eControl,
                              void *pGpioIntMaskOrNumber)
{
    uint32_t ui32Gpionum, ui32RegAddr, ui32Idx, ui32Msk;
    uint32_t ui32FuncRet = AM_HAL_STATUS_SUCCESS;
    am_hal_gpio_mask_t *pGpioIntMask = (am_hal_gpio_mask_t*)pGpioIntMaskOrNumber;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // In all cases, the pointer must be non-NULL.
    //
    if ( pGpioIntMaskOrNumber == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ( eControl > AM_HAL_GPIO_INT_CTRL_LAST )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    if ( eControl <= AM_HAL_GPIO_INT_CTRL_INDV_ENABLE )
    {
        ui32Gpionum = *(uint32_t *)pGpioIntMaskOrNumber;
#ifndef AM_HAL_DISABLE_API_VALIDATION
        if ( ui32Gpionum >= AM_HAL_GPIO_MAX_PADS )
        {
            return AM_HAL_STATUS_OUT_OF_RANGE;
        }
#endif // AM_HAL_DISABLE_API_VALIDATION

        //
        // Convert the GPIO number into an index and bitmask.
        // Then use that to obtain the needed register address.
        // These will be used later.
        //
        if ( gpionum_intreg_index_get(ui32Gpionum, &ui32Idx, &ui32Msk) )
        {
            return AM_HAL_STATUS_INVALID_ARG;
        }

        ui32RegAddr = (uint32_t)&GPIO->MCUN0INT0EN + (ui32Idx * GPIO_INTX_DELTA);
        if ( eChannel == AM_HAL_GPIO_INT_CHANNEL_1 )
        {
            ui32RegAddr += GPIO_NXINT_DELTA;
        }
    }

    DIAG_SUPPRESS_VOLATILE_ORDER()
    AM_CRITICAL_BEGIN

    switch ( eControl )
    {
        case AM_HAL_GPIO_INT_CTRL_INDV_DISABLE:
            AM_REGVAL(ui32RegAddr) &= ~ui32Msk;         // Write MCUNxINTxEN
            if ( eChannel == AM_HAL_GPIO_INT_CHANNEL_BOTH )
            {
                ui32RegAddr += GPIO_NXINT_DELTA;        // Get MCUN1INTxEN addr
                AM_REGVAL(ui32RegAddr) &= ~ui32Msk;     // Write MCUN1INTxEN
            }
            break;

        case AM_HAL_GPIO_INT_CTRL_INDV_ENABLE:
            AM_REGVAL(ui32RegAddr) |= ui32Msk;          // Write MCUNnINTxEN
            if ( eChannel == AM_HAL_GPIO_INT_CHANNEL_BOTH )
            {
                ui32RegAddr += GPIO_NXINT_DELTA;        // Get MCUN1INTxEN addr
                AM_REGVAL(ui32RegAddr) |= ui32Msk;      // Write MCUN1INTxEN
            }
            break;

        case AM_HAL_GPIO_INT_CTRL_MASK_DISABLE:
            if ( eChannel != AM_HAL_GPIO_INT_CHANNEL_1)
            {
                GPIO->MCUN0INT0EN &= ~pGpioIntMask->U.Msk[0];
                GPIO->MCUN0INT1EN &= ~pGpioIntMask->U.Msk[1];
                GPIO->MCUN0INT2EN &= ~pGpioIntMask->U.Msk[2];
                GPIO->MCUN0INT3EN &= ~pGpioIntMask->U.Msk[3];
            }
            if ( eChannel != AM_HAL_GPIO_INT_CHANNEL_0)
            {
                GPIO->MCUN1INT0EN &= ~pGpioIntMask->U.Msk[0];
                GPIO->MCUN1INT1EN &= ~pGpioIntMask->U.Msk[1];
                GPIO->MCUN1INT2EN &= ~pGpioIntMask->U.Msk[2];
                GPIO->MCUN1INT3EN &= ~pGpioIntMask->U.Msk[3];
            }
            break;

        case AM_HAL_GPIO_INT_CTRL_MASK_ENABLE:
            if ( eChannel != AM_HAL_GPIO_INT_CHANNEL_1)
            {
                GPIO->MCUN0INT0EN |= pGpioIntMask->U.Msk[0];
                GPIO->MCUN0INT1EN |= pGpioIntMask->U.Msk[1];
                GPIO->MCUN0INT2EN |= pGpioIntMask->U.Msk[2];
                GPIO->MCUN0INT3EN |= pGpioIntMask->U.Msk[3];
            }
            if ( eChannel != AM_HAL_GPIO_INT_CHANNEL_0)
            {
                GPIO->MCUN1INT0EN |= pGpioIntMask->U.Msk[0];
                GPIO->MCUN1INT1EN |= pGpioIntMask->U.Msk[1];
                GPIO->MCUN1INT2EN |= pGpioIntMask->U.Msk[2];
                GPIO->MCUN1INT3EN |= pGpioIntMask->U.Msk[3];
            }
            break;

        default:
            break;
    }

    AM_CRITICAL_END
    DIAG_DEFAULT_VOLATILE_ORDER()

    //
    // Return the status.
    //
    return ui32FuncRet;

} // am_hal_gpio_interrupt_control()

//*****************************************************************************
//
// Read the GPIO interrupt status.
//
//*****************************************************************************
uint32_t
am_hal_gpio_interrupt_status_get(am_hal_gpio_int_channel_e eChannel,
                                 bool bEnabledOnly,
                                 am_hal_gpio_mask_t *pGpioIntMask)
{
    uint32_t ui32FuncRet = AM_HAL_STATUS_SUCCESS;
    volatile uint32_t ui32Mask[AM_HAL_GPIO_NUMWORDS];

    //
    // Initialize mask variable outside critical section
    //
    for (uint32_t ux = 0; ux < AM_HAL_GPIO_NUMWORDS; ux++ )
    {
        ui32Mask[ux] = 0xFFFFFFFF;
    }

    //
    // Combine upper or lower GPIO words and return in the bitmask structure.
    //
    AM_CRITICAL_BEGIN
    DIAG_SUPPRESS_VOLATILE_ORDER()

    if ( eChannel == AM_HAL_GPIO_INT_CHANNEL_0 )
    {
        if ( bEnabledOnly )
        {
            ui32Mask[0] = GPIO->MCUN0INT0EN;
            ui32Mask[1] = GPIO->MCUN0INT1EN;
            ui32Mask[2] = GPIO->MCUN0INT2EN;
            ui32Mask[3] = GPIO->MCUN0INT3EN;
        }
        pGpioIntMask->U.Msk[0] = GPIO->MCUN0INT0STAT & ui32Mask[0];
        pGpioIntMask->U.Msk[1] = GPIO->MCUN0INT1STAT & ui32Mask[1];
        pGpioIntMask->U.Msk[2] = GPIO->MCUN0INT2STAT & ui32Mask[2];
        pGpioIntMask->U.Msk[3] = GPIO->MCUN0INT3STAT & ui32Mask[3];
    }
    else if ( eChannel == AM_HAL_GPIO_INT_CHANNEL_1 )
    {
        if ( bEnabledOnly )
        {
            ui32Mask[0] = GPIO->MCUN1INT0EN;
            ui32Mask[1] = GPIO->MCUN1INT1EN;
            ui32Mask[2] = GPIO->MCUN1INT2EN;
            ui32Mask[3] = GPIO->MCUN1INT3EN;
        }
        pGpioIntMask->U.Msk[0] = GPIO->MCUN1INT0STAT & ui32Mask[0];
        pGpioIntMask->U.Msk[1] = GPIO->MCUN1INT1STAT & ui32Mask[1];
        pGpioIntMask->U.Msk[2] = GPIO->MCUN1INT2STAT & ui32Mask[2];
        pGpioIntMask->U.Msk[3] = GPIO->MCUN1INT3STAT & ui32Mask[3];
    }
    else
    {
        ui32FuncRet = AM_HAL_STATUS_INVALID_ARG;
    }

    DIAG_DEFAULT_VOLATILE_ORDER()
    AM_CRITICAL_END

    //
    // Return the status.
    //
    return ui32FuncRet;

} // am_hal_gpio_interrupt_status_get()

//*****************************************************************************
//
// Clear GPIO interrupts.
//
//*****************************************************************************
uint32_t
am_hal_gpio_interrupt_clear(am_hal_gpio_int_channel_e eChannel,
                            am_hal_gpio_mask_t *pGpioIntMask)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( !pGpioIntMask )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ( pGpioIntMask->U.Msk[AM_HAL_GPIO_NUMWORDS - 1] &
         ~(((uint32_t)1 << ((AM_HAL_GPIO_MAX_PADS - 1) % 32)) - 1) )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    if ( eChannel == AM_HAL_GPIO_INT_CHANNEL_BOTH )
    {
        //
        // Clear all of the interrupt specified in the masks.
        //
        AM_CRITICAL_BEGIN
        GPIO->MCUN0INT0CLR = pGpioIntMask->U.Msk[0];
        GPIO->MCUN0INT1CLR = pGpioIntMask->U.Msk[1];
        GPIO->MCUN0INT2CLR = pGpioIntMask->U.Msk[2];
        GPIO->MCUN0INT3CLR = pGpioIntMask->U.Msk[3];
        GPIO->MCUN1INT0CLR = pGpioIntMask->U.Msk[0];
        GPIO->MCUN1INT1CLR = pGpioIntMask->U.Msk[1];
        GPIO->MCUN1INT2CLR = pGpioIntMask->U.Msk[2];
        GPIO->MCUN1INT3CLR = pGpioIntMask->U.Msk[3];
        AM_CRITICAL_END
    }
    else if ( eChannel == AM_HAL_GPIO_INT_CHANNEL_0 )
    {
        //
        // Clear the N0 interrupts specified in the masks.
        //
        AM_CRITICAL_BEGIN
        GPIO->MCUN0INT0CLR = pGpioIntMask->U.Msk[0];
        GPIO->MCUN0INT1CLR = pGpioIntMask->U.Msk[1];
        GPIO->MCUN0INT2CLR = pGpioIntMask->U.Msk[2];
        GPIO->MCUN0INT3CLR = pGpioIntMask->U.Msk[3];
        AM_CRITICAL_END
    }
    else if ( eChannel == AM_HAL_GPIO_INT_CHANNEL_1 )
    {
        //
        // Clear the N1 interrupts specified in the masks.
        //
        AM_CRITICAL_BEGIN
        GPIO->MCUN1INT0CLR = pGpioIntMask->U.Msk[0];
        GPIO->MCUN1INT1CLR = pGpioIntMask->U.Msk[1];
        GPIO->MCUN1INT2CLR = pGpioIntMask->U.Msk[2];
        GPIO->MCUN1INT3CLR = pGpioIntMask->U.Msk[3];
        AM_CRITICAL_END
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_gpio_interrupt_clear()

//*****************************************************************************
//
// Read the interrupt status of a given GPIO IRQ.
//
//*****************************************************************************
uint32_t
am_hal_gpio_interrupt_irq_status_get(uint32_t ui32GpioIrq,
                                     bool bEnabledOnly,
                                     uint32_t *pui32IntStatus)
{
    uint32_t ui32FuncRet = AM_HAL_STATUS_SUCCESS;
    uint32_t ui32Nx, ui32Idx, ui32EnblAddr, ui32StatAddr;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (  (pui32IntStatus == NULL)          ||
          (ui32GpioIrq < GPIO0_001F_IRQn)   ||
          (ui32GpioIrq > GPIO1_607F_IRQn) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Get the addresses of the required interrupts registers.
    //
    ui32Nx  = (ui32GpioIrq <= GPIO0_607F_IRQn) ? 0 : 1;
    ui32Idx = ui32GpioIrq - GPIO0_001F_IRQn - (ui32Nx * (GPIO0_607F_IRQn - GPIO0_001F_IRQn + 1));

    ui32EnblAddr = (uint32_t)&GPIO->MCUN0INT0EN   + (ui32Nx * GPIO_NXINT_DELTA) + (ui32Idx * GPIO_INTX_DELTA);
    ui32StatAddr = (uint32_t)&GPIO->MCUN0INT0STAT + (ui32Nx * GPIO_NXINT_DELTA) + (ui32Idx * GPIO_INTX_DELTA);

    AM_CRITICAL_BEGIN
    *pui32IntStatus  = bEnabledOnly ? AM_REGVAL(ui32EnblAddr) : 0xFFFFFFFF;

    //
    // Get the GPIO status register we are interested in.
    //
    *pui32IntStatus &= AM_REGVAL(ui32StatAddr);
    AM_CRITICAL_END

    //
    // Return the status.
    //
    return ui32FuncRet;

} // am_hal_gpio_interrupt_individual_status_get()

//*****************************************************************************
//
// Clear the interrupt(s) for the given GPIO IRQ.
//
//*****************************************************************************
uint32_t
am_hal_gpio_interrupt_irq_clear(uint32_t ui32GpioIrq,
                                uint32_t ui32GpioIntMaskStatus)
{
    uint32_t ui32Nx, ui32Idx, ui32RegAddr;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( (ui32GpioIrq < GPIO0_001F_IRQn)   ||
         (ui32GpioIrq > GPIO1_607F_IRQn) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Get the addresses of the required interrupts registers.
    //
    ui32Nx  = (ui32GpioIrq <= GPIO0_607F_IRQn) ? 0 : 1;
    ui32Idx = ui32GpioIrq - GPIO0_001F_IRQn - (ui32Nx * (GPIO0_607F_IRQn - GPIO0_001F_IRQn + 1));
    ui32RegAddr = (uint32_t)&GPIO->MCUN0INT0CLR + (ui32Nx * GPIO_NXINT_DELTA) + (ui32Idx * GPIO_INTX_DELTA);

    //
    // Clear the given interrupt.
    //
    AM_REGVAL(ui32RegAddr) = ui32GpioIntMaskStatus;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_gpio_interrupt_individual_clear()

//*****************************************************************************
//
// Register an interrupt handler for a specific GPIO.
//
//*****************************************************************************
uint32_t
am_hal_gpio_interrupt_register(am_hal_gpio_int_channel_e eChannel,
                               uint32_t ui32GpioNum,
                               am_hal_gpio_handler_t pfnHandler,
                               void *pArg)
{
    //
    // Determine the correct IRQ offset numbers.
    //
    uint32_t ui32Channel0Irq = GPIO_NUM2IDX(ui32GpioNum);
    uint32_t ui32Channel1Irq = ui32Channel0Irq + 4;

    //
    // Store the handler information in the array associated with this GPIO.
    //
    if ( eChannel == AM_HAL_GPIO_INT_CHANNEL_0 )
    {
        gpio_ppfnHandlers[ui32Channel0Irq][ui32GpioNum % 32] = pfnHandler;
        gpio_pppvIrqArgs[ui32Channel0Irq][ui32GpioNum % 32] = pArg;
    }
    else if ( eChannel == AM_HAL_GPIO_INT_CHANNEL_1)
    {
        gpio_ppfnHandlers[ui32Channel1Irq][ui32GpioNum % 32] = pfnHandler;
        gpio_pppvIrqArgs[ui32Channel1Irq][ui32GpioNum % 32] = pArg;
    }
    else if ( eChannel == AM_HAL_GPIO_INT_CHANNEL_BOTH)
    {
        gpio_ppfnHandlers[ui32Channel0Irq][ui32GpioNum % 32] = pfnHandler;
        gpio_pppvIrqArgs[ui32Channel0Irq][ui32GpioNum % 32] = pArg;
        gpio_ppfnHandlers[ui32Channel1Irq][ui32GpioNum % 32] = pfnHandler;
        gpio_pppvIrqArgs[ui32Channel1Irq][ui32GpioNum % 32] = pArg;
    }
    else
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_gpio_interrupt_register()

//*****************************************************************************
//
// Relay interrupts from the main GPIO module to individual handlers.
//
// A typical call sequence to the service routine might look like:
//
// am_hal_gpio_interrupt_irq_status_get(GPIO1_405F_IRQn, true, &ui32IntStatus);
// am_hal_gpio_interrupt_service(GPIO1_405F_IRQn, ui32IntStatus);
//
//
//*****************************************************************************
uint32_t
am_hal_gpio_interrupt_service(uint32_t ui32GpioIrq,
                              uint32_t ui32GpioIntMaskStatus)
{
    uint32_t ui32RetStatus = AM_HAL_STATUS_SUCCESS;
    uint32_t ui32FFS;
    am_hal_gpio_handler_t pfnHandler;
    void *pArg;

    //
    // 0-base the IRQ number.
    //
    ui32GpioIrq -= GPIO0_001F_IRQn;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check parameters
    //
    if ( ui32GpioIrq >= GPIO_NUM_IRQS )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Handle interrupts.
    // Get status word from the caller.
    //
    while ( ui32GpioIntMaskStatus )
    {
        //
        // We need to FFS (Find First Set).  We can easily zero-base FFS
        // since we know that at least 1 bit is set in ui32GpioIntMaskStatus.
        // FFS(x) = 31 - clz(x & -x).       // Zero-based version of FFS.
        //
        ui32FFS = ui32GpioIntMaskStatus & (uint32_t)(-(int32_t)ui32GpioIntMaskStatus);
        ui32FFS = 31 - AM_ASM_CLZ(ui32FFS);

        //
        // Turn off the bit we picked in the working copy
        //
        ui32GpioIntMaskStatus &= ~(0x00000001 << ui32FFS);

        //
        // Check the bit handler table to see if there is an interrupt handler
        // registered for this particular bit.
        //
        pfnHandler = gpio_ppfnHandlers[ui32GpioIrq][ui32FFS];
        pArg = gpio_pppvIrqArgs[ui32GpioIrq][ui32FFS];
        if ( pfnHandler )
        {
            //
            // If we found an interrupt handler routine, call it now.
            //
            pfnHandler(pArg);
        }
        else
        {
            //
            // No handler was registered for the GPIO that interrupted.
            // Return an error.
            //
            ui32RetStatus = AM_HAL_STATUS_INVALID_OPERATION;
        }
    }

    //
    // Return the status.
    //
    return ui32RetStatus;

} // am_hal_gpio_interrupt_service()

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
