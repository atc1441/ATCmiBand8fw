//*****************************************************************************
//
//! @file am_hal_audadc.c
//!
//! @brief Functions for interfacing with the Audio Analog to Digital Converter.
//!
//! @addtogroup audadc4_4b AUDADC - Audio Analog-to-Digital Converter
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
// Private Types.
//
//*****************************************************************************

#define AM_HAL_MAGIC_AUDADC                0xAFAFAF
#define AM_HAL_AUDADC_CHK_HANDLE(h)        ((h) && ((am_hal_handle_prefix_t *)(h))->s.bInit && (((am_hal_handle_prefix_t *)(h))->s.magic == AM_HAL_MAGIC_AUDADC))

// ****************************************************************************
//
//! @name Default coefficients (used when trims not provided):
//!  TEMP_DEFAULT    = Temperature in deg K (e.g. 299.5 - 273.15 = 26.35)
//!  AMBIENT_DEFAULT = Voltage measurement at default temperature.
//!  OFFSET_DEFAULT  = Default AUDADC offset at 1v.
//! @{
//
// ****************************************************************************
#define AM_HAL_AUDADC_CALIB_TEMP_DEFAULT            (299.5F)
#define AM_HAL_AUDADC_CALIB_AMBIENT_DEFAULT         (1.02809F)
#define AM_HAL_AUDADC_CALIB_AUDADC_OFFSET_DEFAULT   (-0.004281F)
//! @}

//
//! @brief AUDADC configuration registers structure.
//
typedef struct
{
    bool          bValid;
    uint32_t      regCFG;
    uint32_t      regSL0CFG;
    uint32_t      regSL1CFG;
    uint32_t      regSL2CFG;
    uint32_t      regSL3CFG;
    uint32_t      regSL4CFG;
    uint32_t      regSL5CFG;
    uint32_t      regSL6CFG;
    uint32_t      regSL7CFG;
    uint32_t      regWULIM;
    uint32_t      regWLLIM;
    uint32_t      regINTEN;
} am_hal_audadc_register_state_t;

//
//! @brief AUDADC State structure.
//
typedef struct
{
    //
    //! Handle validation prefix.
    //
    am_hal_handle_prefix_t      prefix;

    //
    //! Physical module number.
    //
    uint32_t                    ui32Module;

    //
    //! AUDADC Capabilities.
    //
    am_hal_audadc_capabilities_t   capabilities;

    //
    //! Power Save-Restore register state
    //
    am_hal_audadc_register_state_t registerState;

    //
    //! DMA transaction Tranfer Control Buffer.
    //
    uint32_t            ui32BufferPing;
    uint32_t            ui32BufferPong;
    uint32_t            ui32BufferPtr;

} am_hal_audadc_state_t;

//*****************************************************************************
//
//! @brief Private SRAM view of temperature trims.
//!
//! This static SRAM union is private to the AUDADC HAL functions.
//
//*****************************************************************************
static union
{
    //! These trim values are loaded as uint32_t values.
    struct
    {
              //! Temperature of the package test head (in degrees Kelvin)
              uint32_t ui32CalibrationTemperature;

              //! Voltage corresponding to temperature measured on test head.
              uint32_t ui32CalibrationVoltage;

              //! AUDADC offset voltage measured on the package test head.
              uint32_t ui32CalibrationOffset;

              //! Flag if default (guess) or measured.
              bool bMeasured;
    } ui32;
    //! These trim values are accessed as floats when used in temp calculations.
    struct
    {
              //! Temperature of the package test head in degrees Kelvin
              float    fCalibrationTemperature;

              //! Voltage corresponding to temperature measured on test head.
              float    fCalibrationVoltage;

              //! AUDADC offset voltage measured on the package test head.
              float    fCalibrationOffset;

              //! Flag if default (guess) or measured.
              float fMeasuredFlag;
    } flt;
} priv_temp_trims;

//*****************************************************************************
//
// Global Variables.
//
//*****************************************************************************
am_hal_audadc_state_t           g_AUDADCState[AM_REG_AUDADC_NUM_MODULES];

uint32_t                        g_AUDADCSlotsConfigured;

uint32_t                        g_AUDADCSlotTrkCycMin;

uint32_t                        g_AUDADCPgaChConfigured;

//*****************************************************************************
//
//! @brief AUDADC initialization function
//!
//! @param ui32Module - module instance.
//! @param ppHandle   - returns the handle for the module instance.
//!
//! This function accepts a module instance, allocates the interface and then
//! returns a handle to be used by the remaining interface functions.
//!
//! @return status    - generic or interface specific status.
//!
//! @note A return of AM_HAL_STATUS_SUCCESS does not infer that the
//! temperature calibrations are valid. The caller must check the bMeasured
//! structure element in order to determine that.
//
//*****************************************************************************
uint32_t
am_hal_audadc_initialize(uint32_t ui32Module, void **ppHandle)
{
    uint32_t ui32Ret, ui32Val;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check if AUADC is available
    //
    ui32Ret = am_hal_mram_info_read(1, AM_REG_INFO1_AUDADC_BINNING_O / 4, 1, &ui32Val);

    if ( (ui32Ret != 0) || (ui32Val != 0x00000001) )
    {
        //
        // Fail to read info, or AUDADC is not available in this part.
        //
        return AM_HAL_STATUS_FAIL;
    }
    //
    // Validate the module number
    //
    if ( ui32Module >= AM_REG_AUDADC_NUM_MODULES )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    //
    // Check for valid arguements.
    //
    if ( !ppHandle )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Check if the handle is unallocated.
    //
    if ( g_AUDADCState[ui32Module].prefix.s.bInit )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Initialize the handle.
    //
    g_AUDADCState[ui32Module].prefix.s.bInit = true;
    g_AUDADCState[ui32Module].prefix.s.magic = AM_HAL_MAGIC_AUDADC;
    g_AUDADCState[ui32Module].ui32Module = ui32Module;

    //
    // Initialize the number of slots configured.
    //
    g_AUDADCSlotsConfigured = 0;

    //
    // Return the handle.
    //
    *ppHandle = (void *)&g_AUDADCState[ui32Module];

    //
    // Before returning, read the temperature trims from INFO1,
    // which are safely read using am_hal_mram_info_read().
    //
    am_hal_mram_info_read(1, AM_REG_INFO1_TEMP_CAL_ATE_O / 4, 1,    // INFO1, word offset, read 1 word
                          &priv_temp_trims.ui32.ui32CalibrationTemperature);
    am_hal_mram_info_read(1, AM_REG_INFO1_TEMP_CAL_MEASURED_O / 4, 1,
                          &priv_temp_trims.ui32.ui32CalibrationVoltage);
    am_hal_mram_info_read(1, AM_REG_INFO1_TEMP_CAL_ADC_OFFSET_O / 4, 1,
                          &priv_temp_trims.ui32.ui32CalibrationOffset);

    if ( (priv_temp_trims.ui32.ui32CalibrationTemperature == 0xffffffff)    ||
         (priv_temp_trims.ui32.ui32CalibrationVoltage     == 0xffffffff)    ||
         (priv_temp_trims.ui32.ui32CalibrationOffset      == 0xffffffff) )
    {
        //
        // Since the device has not been calibrated on the tester, we'll load
        // default calibration values.  These default values should result
        // in worst-case temperature measurements of +-6 degress C.
        //
        priv_temp_trims.flt.fCalibrationTemperature = AM_HAL_AUDADC_CALIB_TEMP_DEFAULT;
        priv_temp_trims.flt.fCalibrationVoltage     = AM_HAL_AUDADC_CALIB_AMBIENT_DEFAULT;
        priv_temp_trims.flt.fCalibrationOffset      = AM_HAL_AUDADC_CALIB_AUDADC_OFFSET_DEFAULT;
        priv_temp_trims.ui32.bMeasured = false;
    }
    else
    {
        priv_temp_trims.ui32.bMeasured = true;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief AUDADC deinitialization function
//!
//! @param pHandle       - returns the handle for the module instance.
//!
//! This function accepts a handle to an instance and de-initializes the
//! interface.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_deinitialize(void *pHandle)
{
    uint32_t                status = AM_HAL_STATUS_SUCCESS;
    am_hal_audadc_state_t    *pAUDADCState = (am_hal_audadc_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    if ( pAUDADCState->prefix.s.bEnable )
    {
        status = am_hal_audadc_disable(pHandle);
    }

    pAUDADCState->prefix.s.bInit = false;

    //
    // Return the status.
    //
    return status;
}

//*****************************************************************************
//
//! @brief AUDADC configuration function
//!
//! @param pHandle   - handle for the module instance.
//! @param psConfig  - pointer to the configuration structure.
//!
//! This function configures the AUDADC for operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_configure(void *pHandle,
                        am_hal_audadc_config_t *psConfig)
{
    uint32_t                ui32Config;
    am_hal_audadc_state_t   *pAUDADCState = (am_hal_audadc_state_t *)pHandle;
    uint32_t                ui32Module = pAUDADCState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Config = 0;

    //
    // Set the AUDADC clock source.
    //
    ui32Config |= _VAL2FLD(AUDADC_CFG_CLKSEL, psConfig->eClock);

    //
    // set minimum limit of Trkcyc
    //
    if ( psConfig->eClock == AM_HAL_AUDADC_CLKSEL_HFRC_48MHz || psConfig->eClock == AM_HAL_AUDADC_CLKSEL_HFRC2_48MHz )
    {
        g_AUDADCSlotTrkCycMin = AM_HAL_AUDADC_MIN_TRKCYC;
    }
    else if ( psConfig->eClock == AM_HAL_AUDADC_CLKSEL_XTHS_24MHz )
    {
        g_AUDADCSlotTrkCycMin = 19;
    }
    else
    {
        g_AUDADCSlotTrkCycMin = AM_HAL_AUDADC_MIN_TRKCYC;
    }
    //
    // Set the AUDADC periodic trigger source.
    //
    ui32Config |= _VAL2FLD(AUDADC_CFG_RPTTRIGSEL, psConfig->eRepeatTrigger);

    //
    // Set the AUDADC trigger polarity.
    //
    ui32Config |= _VAL2FLD(AUDADC_CFG_TRIGPOL, psConfig->ePolarity);

    //
    // Set the AUDADC trigger.
    //
    ui32Config |= _VAL2FLD(AUDADC_CFG_TRIGSEL, psConfig->eTrigger);

    //
    // Set the sample mode.
    //
    ui32Config |= _VAL2FLD(AUDADC_CFG_SAMPMODE, psConfig->eSampMode);

    //
    // Set the Destructive FIFO read.
    //
    ui32Config |= _VAL2FLD(AUDADC_CFG_DFIFORDEN, 1);

    //
    // Set the AUDADC clock mode.
    //
    ui32Config |= _VAL2FLD(AUDADC_CFG_CKMODE, psConfig->eClockMode);

    //
    // Set the AUDADC low power mode.
    //
    ui32Config |= _VAL2FLD(AUDADC_CFG_LPMODE, psConfig->ePowerMode);

    //
    // Set the AUDADC repetition mode.
    //
    ui32Config |= _VAL2FLD(AUDADC_CFG_RPTEN, psConfig->eRepeat);

    //
    // Set the configuration in the AUDADC peripheral.
    //
    AUDADCn(ui32Module)->CFG = ui32Config;

    //
    // Add this signed offset to data before being written to the FIFO:
    // DATA2's complement (12 bit data)
    //
    AUDADCn(ui32Module)->DATAOFFSET = ((AUDADC->DATAOFFSET & ~AUDADC_DATAOFFSET_OFFSET_Msk) | _VAL2FLD(AUDADC_DATAOFFSET_OFFSET, 0x1800));

    //
    // Return status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief AUDADC slot configuration function
//!
//! @param pHandle - handle for the module instance.
//! @param ui32SlotNumber - AUDADC Slot Number
//! @param pSlotConfig    - pointer to the configuration structure.
//!
//! This function configures the AUDADC slot for operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_configure_slot(void *pHandle,
                             uint32_t ui32SlotNumber,
                             am_hal_audadc_slot_config_t *pSlotConfig)
{
    uint32_t            ui32Config;
    uint32_t            ui32RegOffset;
    am_hal_audadc_state_t  *pAUDADCState = (am_hal_audadc_state_t *)pHandle;
    uint32_t            ui32Module = pAUDADCState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    //
    // Check the slot number.
    //
    if ( ui32SlotNumber >= AM_HAL_AUDADC_MAX_SLOTS )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    //
    // check the Trkcyc value minimum limit
    //
    if ( pSlotConfig->ui32TrkCyc < g_AUDADCSlotTrkCycMin )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Config = 0;

    //
    // Set the measurements to average
    //
    ui32Config |= _VAL2FLD(AUDADC_SL0CFG_ADSEL0, pSlotConfig->eMeasToAvg);

    //
    // Set additional sampling AUDADC clock cycles
    //
    if ( pSlotConfig->ui32TrkCyc < g_AUDADCSlotTrkCycMin )
    {
        pSlotConfig->ui32TrkCyc = g_AUDADCSlotTrkCycMin;
    }

    ui32Config |= _VAL2FLD(AUDADC_SL0CFG_TRKCYC0, pSlotConfig->ui32TrkCyc);

    //
    // Set the precision mode.
    //
    ui32Config |= _VAL2FLD(AUDADC_SL0CFG_PRMODE0, pSlotConfig->ePrecisionMode);

    //
    // Set the ui32ChanNumnel.
    //
    ui32Config |= _VAL2FLD(AUDADC_SL0CFG_CHSEL0, pSlotConfig->eChannel);

    //
    // Enable window comparison if configured.
    //
    ui32Config |= _VAL2FLD(AUDADC_SL0CFG_WCEN0, pSlotConfig->bWindowCompare);

    //
    // Enable the slot if configured.
    //
    ui32Config |= _VAL2FLD(AUDADC_SL0CFG_SLEN0, pSlotConfig->bEnabled);

    //
    // Locate the correct register for this AUDADC slot.
    //
    ui32RegOffset = ((uint32_t)&AUDADCn(ui32Module)->SL0CFG) + (4 * ui32SlotNumber);

    //
    // Write the register with the caller's configuration value.
    //
    AM_REGVAL(ui32RegOffset) = ui32Config;

    //
    // Update the nubmer of slots configured.
    //
    g_AUDADCSlotsConfigured++;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief AUDADC internal repeating trigger timer configuration function
//!
//! @param pHandle - handle for the module instance.
//! @param pConfig - pointer to the configuration structure.
//!
//! This function configures the AUDADC internal trigger timer for operation.
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_configure_irtt(void *pHandle,
                             am_hal_audadc_irtt_config_t *pConfig)
{
    uint32_t    ui32Config;
    uint32_t    ui32Module = ((am_hal_audadc_state_t *)pHandle)->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Config = 0;
    //
    // Disable AUDADC internal repeating trigger timer
    //
    ui32Config |= _VAL2FLD(AUDADC_INTTRIGTIMER_TIMEREN, 0);

    //
    // Set AUDADC internal repeating trigger timer clock division
    //
    ui32Config |= _VAL2FLD(AUDADC_INTTRIGTIMER_CLKDIV, pConfig->eClkDiv);

    //
    // Set AUDADC internal repeating trigger timer count
    //
    ui32Config |= _VAL2FLD(AUDADC_INTTRIGTIMER_TIMERMAX, pConfig->ui32IrttCountMax);

    //
    // Set AUDADC internal repeating trigger timer configuration.
    //
    AUDADCn(ui32Module)->INTTRIGTIMER = ui32Config;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief AUDADC internal repeating trigger timer enable function
//!
//! @param pHandle - handle for the module instance.
//!
//! This function enables internal repeating trigger timer.
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_irtt_enable(void *pHandle)
{
    am_hal_audadc_state_t  *pAUDADCState = (am_hal_audadc_state_t *)pHandle;
    uint32_t            ui32Module = pAUDADCState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Enable the AUDADC.
    //
    AUDADCn(ui32Module)->INTTRIGTIMER_b.TIMEREN = 0x1;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

}

//*****************************************************************************
//
//! @brief AUDADC internal repeating trigger timer disable function
//!
//! @param pHandle - handle for the module instance.
//!
//! This function disables internal repeating trigger timer.
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_irtt_disable(void *pHandle)
{
    am_hal_audadc_state_t  *pAUDADCState = (am_hal_audadc_state_t *)pHandle;
    uint32_t            ui32Module = pAUDADCState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Enable the AUDADC.
    //
    AUDADCn(ui32Module)->INTTRIGTIMER_b.TIMEREN = 0x0;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

}

//*****************************************************************************
//
//! @brief AUDADC DMA configuration function
//!
//! @param pHandle    - handle for the module instance.
//! @param pDMAConfig - pointer to the configuration structure.
//!
//! This function configures the AUDADC DMA for operation.
//!
//! @return status    - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_configure_dma(void *pHandle,
                            am_hal_audadc_dma_config_t *pDMAConfig)
{
    am_hal_audadc_state_t* pAUDADCState = (am_hal_audadc_state_t *)pHandle;
    uint32_t ui32Module = pAUDADCState->ui32Module;
    uint32_t ui32Config;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    pAUDADCState->ui32BufferPtr = pAUDADCState->ui32BufferPing = pDMAConfig->ui32TargetAddress;
    pAUDADCState->ui32BufferPong = pDMAConfig->ui32TargetAddressReverse;

    ui32Config = 0;

    //
    // Configure the DMA complete power-off.
    //
    ui32Config |= _VAL2FLD(AUDADC_DMACFG_DPWROFF, 0);      // DPWROFF not supported!

    //
    // Configure the data to be transferred.
    //
    if ( g_AUDADCSlotsConfigured > 1 )
    {
        // Need slot number to distinguish between slot results.
        ui32Config |= _VAL2FLD(AUDADC_DMACFG_DMAEN, AUDADC_DMACFG_DMAEN_DIS);
    }
    else
    {
        ui32Config |= _VAL2FLD(AUDADC_DMACFG_DMAEN, AUDADC_DMACFG_DMAEN_EN);
    }

    //
    // Enable DMA Halt on Status (DMAERR or DMACPL) by default. This bit is reserved in apollo4
    //
//    ui32Config |= _VAL2FLD(AUDADC_DMACFG_DMAHONSTAT, AUDADC_DMACFG_DMAHONSTAT_EN);

    //
    // Configure the DMA dynamic priority handling.
    //
    ui32Config |= _VAL2FLD(AUDADC_DMACFG_DMADYNPRI, pDMAConfig->bDynamicPriority);

    //
    // Configure the DMA static priority.
    //
    ui32Config |= _VAL2FLD(AUDADC_DMACFG_DMAPRI, pDMAConfig->ePriority);

    //
    // Enable the DMA (does not start until AUDADC is enabled and triggered).
    //
    ui32Config |= _VAL2FLD(AUDADC_DMACFG_DMAEN, AUDADC_DMACFG_DMAEN_EN);

    //
    // Set the DMA configuration.
    //
    AUDADCn(ui32Module)->DMACFG = ui32Config;

    //
    // Set the DMA transfer count.
    //
    AUDADCn(ui32Module)->DMATOTCOUNT_b.TOTCOUNT = pDMAConfig->ui32SampleCount;

    //
    // Set the DMA target address.
    //
    AUDADCn(ui32Module)->DMATARGADDR = pAUDADCState->ui32BufferPtr;

    //
    // Set the DMA trigger on FIFO 75% full.
    //
    AUDADCn(ui32Module)->DMATRIGEN = AUDADC_DMATRIGEN_DFIFO75_Msk;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief AUDADC DMA Buffer function
//!
//! @param pHandle - handle for the module instance.
//!
//! This function to get DMA Buffer.
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_dma_get_buffer(void *pHandle)
{
    uint32_t ui32BufferPtr;
    am_hal_audadc_state_t *pState = (am_hal_audadc_state_t *) pHandle;

    // Invalidate DAXI to make sure CPU sees the new data when loaded.
    am_hal_daxi_control(AM_HAL_DAXI_CONTROL_INVALIDATE, NULL);

    ui32BufferPtr = (pState->ui32BufferPtr == pState->ui32BufferPong)? pState->ui32BufferPing: pState->ui32BufferPong;

    return ui32BufferPtr;
}

//*****************************************************************************
//
//! @brief AUDADC device specific control function.
//!
//! @param pHandle   - handle for the module instance.
//! @param eRequest - One of:
//!   AM_HAL_AUDADC_REQ_WINDOW_CONFIG
//!   AM_HAL_AUDADC_REQ_TEMP_CELSIUS_GET (pArgs is required, see enums).
//!   AM_HAL_AUDADC_REQ_TEMP_TRIMS_GET   (pArgs is required, see enums).
//!   AM_HAL_AUDADC_REQ_DMA_DISABLE      (pArgs is not required).
//! @param pArgs - Pointer to arguments for Control Switch Case
//!
//! This function provides for special control functions for the AUDADC operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t am_hal_audadc_control(void *pHandle,
                               am_hal_audadc_request_e eRequest,
                               void *pArgs)
{
    uint32_t    ui32Module = ((am_hal_audadc_state_t *)pHandle)->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    switch ( eRequest )
    {
        case AM_HAL_AUDADC_REQ_WINDOW_CONFIG:
        {
            am_hal_audadc_window_config_t *pWindowConfig = (am_hal_audadc_window_config_t *)pArgs;

#ifndef AM_HAL_DISABLE_API_VALIDATION
            //
            // Check the window limits.
            //
            if ( (pWindowConfig->ui32Upper > AUDADC_WULIM_ULIM_Msk)   ||
                 (pWindowConfig->ui32Lower > AUDADC_WLLIM_LLIM_Msk) )
            {
                return AM_HAL_STATUS_OUT_OF_RANGE;
            }
#endif // AM_HAL_DISABLE_API_VALIDATION
            //
            // Set the window comparison upper and lower limits.
            //
            AUDADCn(ui32Module)->WULIM = _VAL2FLD(AUDADC_WULIM_ULIM, pWindowConfig->ui32Upper);
            AUDADCn(ui32Module)->WLLIM = _VAL2FLD(AUDADC_WLLIM_LLIM, pWindowConfig->ui32Lower);

            //
            // Set the window scale per precision mode if indicated.
            //
            AUDADCn(ui32Module)->SCWLIM = _VAL2FLD(AUDADC_SCWLIM_SCWLIMEN,
                                                pWindowConfig->bScaleLimits);
        }
        break;

        case AM_HAL_AUDADC_REQ_TEMP_CELSIUS_GET:
            //
            // pArgs must point to an array of 3 floats.  To assure that the
            // array is valid, upon calling the 3rd float (pArgs[2]) must be
            // set to the value -123.456F.
            //
            if ( pArgs != NULL )
            {
                float *pfArray = (float*)pArgs;
                float fTemp, fCalibration_temp, fCalibration_voltage, fCalibration_offset, fVoltage;

                if ( pfArray[2] == -123.456F )
                {
                    //
                    // Get the scaled voltage obtained from the AUDADC sample.
                    // The AUDADC sample value is scaled up by the reference voltage
                    // (e.g. 1.5F), then divided by 65536.0F.
                    //
                    fVoltage = pfArray[0];

                    //
                    // Get calibration temperature from trimmed values & convert to degrees K.
                    //
                    fCalibration_temp = priv_temp_trims.flt.fCalibrationTemperature;
                    fCalibration_voltage = priv_temp_trims.flt.fCalibrationVoltage;
                    fCalibration_offset  = priv_temp_trims.flt.fCalibrationOffset;

                    //
                    // Compute the temperature.
                    //
                    fTemp  = fCalibration_temp;
                    fTemp /= (fCalibration_voltage - fCalibration_offset);
                    fTemp *= (fVoltage - fCalibration_offset);

                    //
                    // Give it back to the caller in Celsius.
                    //
                    pfArray[1] = fTemp - 273.15f;
                }
                else
                {
                    return AM_HAL_STATUS_INVALID_OPERATION;
                }
            }
            else
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            break;

        case AM_HAL_AUDADC_REQ_TEMP_TRIMS_GET:
            //
            // pArgs must point to an array of 4 floats.  To assure that the
            // array is valid, upon calling the 4th float (pArgs[3]) must be
            // set to the value -123.456.
            // On return, pArgs[3] is set to 1 if the returned values are
            //  calibrated, or 0 if default calibration values.
            //
            if ( pArgs != NULL )
            {
                float *pfArray = (float*)pArgs;
                if ( pfArray[3] == -123.456F )
                {
                    //
                    // Return trim temperature as a float.
                    //
                    pfArray[0] = priv_temp_trims.flt.fCalibrationTemperature;

                    //
                    // Return trim voltage as a float.
                    //
                    pfArray[1] = priv_temp_trims.flt.fCalibrationVoltage;

                    //
                    // Return trim AUDADC offset voltage as a float.
                    //
                    pfArray[2] = priv_temp_trims.flt.fCalibrationOffset;

                    //
                    // Set the calibrated or uncalibrated flag
                    //
                    ((uint32_t*)pArgs)[3] = priv_temp_trims.ui32.bMeasured;
                }
                else
                {
                    return AM_HAL_STATUS_INVALID_OPERATION;
                }
            }
            else
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            break;

        case AM_HAL_AUDADC_REQ_DMA_DISABLE:
            //
            // Clear DMAEN register
            //
            AUDADCn(ui32Module)->DMACFG_b.DMAEN = AUDADC_DMACFG_DMAEN_DIS;
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief AUDADC enable function
//!
//! @param pHandle   - handle for the module instance.
//!
//! This function enables the AUDADC operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_enable(void *pHandle)
{
    am_hal_audadc_state_t  *pAUDADCState = (am_hal_audadc_state_t *)pHandle;
    uint32_t            ui32Module = pAUDADCState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( pAUDADCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_SUCCESS;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Enable the AUDADC.
    //
    AUDADCn(ui32Module)->CFG_b.ADCEN = 0x1;

    //
    // Set flag to indicate module is enabled.
    //
    pAUDADCState->prefix.s.bEnable = true;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief AUDADC disable function
//!
//! @param pHandle - handle for the module instance.
//!
//! This function disables the AUDADC operation.
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_disable(void *pHandle)
{
    am_hal_audadc_state_t  *pAUDADCState = (am_hal_audadc_state_t *)pHandle;
    uint32_t            ui32Module = pAUDADCState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Disable the AUDADC.
    //
    AUDADCn(ui32Module)->CFG_b.ADCEN = 0x0;

    //
    // Set flag to indicate module is disabled.
    //
    pAUDADCState->prefix.s.bEnable = false;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief AUDADC status function
//!
//! @param pHandle - handle for the interface.
//! @param pStatus - pointer to status
//!
//! This function returns the current status of the DMA operation.
//!
//! @return status - DMA status flags.
//
//*****************************************************************************
uint32_t
am_hal_audadc_status_get(void *pHandle, am_hal_audadc_status_t *pStatus )
{
    uint32_t    ui32Module = ((am_hal_audadc_state_t *)pHandle)->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Get the power status.
    //
    pStatus->bPoweredOn = (AUDADCn(ui32Module)->STAT & AUDADC_STAT_PWDSTAT_Msk) ==
                          _VAL2FLD(AUDADC_STAT_PWDSTAT, AUDADC_STAT_PWDSTAT_ON);

    //
    // Get the low power mode 1 status.
    //
    pStatus->bLPMode1 = (AUDADCn(ui32Module)->STAT & AUDADC_STAT_PWDSTAT_Msk) ==
                        _VAL2FLD(AUDADC_STAT_PWDSTAT, AUDADC_STAT_PWDSTAT_POWERED_DOWN);

    //
    //  Get the DMA status.
    //
    pStatus->bErr = ((AUDADCn(ui32Module)->DMASTAT & AUDADC_DMASTAT_DMAERR_Msk) > 0);
    pStatus->bCmp = ((AUDADCn(ui32Module)->DMASTAT & AUDADC_DMASTAT_DMACPL_Msk) > 0);
    pStatus->bTIP = ((AUDADCn(ui32Module)->DMASTAT & AUDADC_DMASTAT_DMATIP_Msk) > 0);

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief AUDADC enable interrupts function
//!
//! @param pHandle       - handle for the interface.
//! @param ui32IntMask  - AUDADC interrupt mask.
//!
//! This function enables the specific indicated interrupts.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_interrupt_enable(void *pHandle, uint32_t ui32IntMask)
{
    uint32_t    ui32Module = ((am_hal_audadc_state_t*)pHandle)->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Enable the interrupts.
    //
    AUDADCn(ui32Module)->INTEN |= ui32IntMask;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief AUDADC disable interrupts function
//!
//! @param pHandle     - handle for the interface.
//! @param ui32IntMask - AUDADC interrupt mask.
//!
//! This function disable the specific indicated interrupts.
//!
//! @return status     - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_interrupt_disable(void *pHandle, uint32_t ui32IntMask)
{
    uint32_t    ui32Module = ((am_hal_audadc_state_t*)pHandle)->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Disable the interrupts.
    //
    AUDADCn(ui32Module)->INTEN &= ~ui32IntMask;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief AUDADC interrupt status function
//!
//! @param pHandle      - handle for the interface.
//! @param pui32Status  - pointer to status
//! @param bEnabledOnly - if AUDADC enabled
//!
//! This function returns the specific indicated interrupt status.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_interrupt_status(void *pHandle,
                               uint32_t  *pui32Status,
                               bool bEnabledOnly)
{
    uint32_t    ui32Module = ((am_hal_audadc_state_t*)pHandle)->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // if requested, only return the interrupts that are enabled.
    //
    if ( bEnabledOnly )
    {
        uint32_t ui32RetVal = AUDADCn(ui32Module)->INTSTAT;
        *pui32Status = AUDADCn(ui32Module)->INTEN & ui32RetVal;
    }
    else
    {
        *pui32Status = AUDADCn(ui32Module)->INTSTAT;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief AUDADC interrupt clear
//!
//! @param pHandle         - handle for the interface.
//! @param ui32IntMask    - uint32_t for interrupts to clear
//!
//! This function clears the interrupts for the given peripheral.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_interrupt_clear(void *pHandle, uint32_t ui32IntMask)
{
    uint32_t    ui32Module = ((am_hal_audadc_state_t*)pHandle)->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Clear the interrupts.
    //
    AUDADCn(ui32Module)->INTCLR = ui32IntMask;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief AUDADC interrupt service
//!
//! @param pHandle       - handle for the interface.
//! @param pDMAConfig   - pointer to DMA configuration
//!
//! Interrupt service routine.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_interrupt_service(void *pHandle, am_hal_audadc_dma_config_t *pDMAConfig)
{
    am_hal_audadc_state_t* pAUDADCState = (am_hal_audadc_state_t *)pHandle;
    uint32_t ui32Module = pAUDADCState->ui32Module;

    pAUDADCState->ui32BufferPtr = (pAUDADCState->ui32BufferPtr == pAUDADCState->ui32BufferPong)? pAUDADCState->ui32BufferPing: pAUDADCState->ui32BufferPong;

    //
    // Set the DMA transfer count.
    //
    AUDADCn(ui32Module)->DMATOTCOUNT_b.TOTCOUNT = pDMAConfig->ui32SampleCount;

    //
    // Set the DMA target address.
    //
    AUDADCn(ui32Module)->DMATARGADDR = pAUDADCState->ui32BufferPtr;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// AUDADC sample read function
//
// This function reads samples from the AUDADC FIFO or an SRAM sample buffer
// returned by a DMA operation.
//
//*****************************************************************************
uint32_t am_hal_audadc_samples_read(void *pHandle,
                                    uint32_t *pui32InSampleBuffer,
                                    uint32_t *pui32InOutNumberSamples,
                                    bool bLowSample, am_hal_audadc_sample_t *pui32LGOutBuffer,
                                    bool bHighSample, am_hal_audadc_sample_t *pui32HGOutBuffer)
{
    uint32_t      ui32Sample;
    uint32_t      ui32RequestedSamples = *pui32InOutNumberSamples;

    uint32_t ui32Module = ((am_hal_audadc_state_t*)pHandle)->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    //
    // Check the output sample buffer pointer.
    //
    if ( (NULL == pui32LGOutBuffer) && (NULL == pui32HGOutBuffer))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    *pui32InOutNumberSamples = 0;

    //
    // Check if we are reading directly from FIFO or DMA SRAM buffer.
    //
    if ( NULL == pui32InSampleBuffer )
    {
        //
        // Grab a value from the AUDADC FIFO
        //
        do
        {
            ui32Sample = AUDADCn(ui32Module)->FIFOPR;
            if ( bLowSample )
            {
              pui32LGOutBuffer->ui16AudChannel = AM_HAL_AUDADC_FIFO_SLOT(ui32Sample);
              pui32LGOutBuffer->int16Sample    = AM_HAL_AUDADC_FIFO_LGDATA(ui32Sample) << 4;
              pui32LGOutBuffer++;
            }

            if ( bHighSample )
            {
              pui32HGOutBuffer->ui16AudChannel = AM_HAL_AUDADC_FIFO_SLOT(ui32Sample);
              pui32HGOutBuffer->int16Sample    = AM_HAL_AUDADC_FIFO_HGDATA(ui32Sample) << 4;
              pui32HGOutBuffer++;
            }
            (*pui32InOutNumberSamples)++;
        } while (!AM_HAL_AUDADC_FIFO_EMPTY(AUDADCn(ui32Module)) &&
                 (*pui32InOutNumberSamples < ui32RequestedSamples));
    }
    else
    {
        //
        // Process the samples from the provided sample buffer
        //
        do
        {
            if ( bLowSample )
            {
              pui32LGOutBuffer->ui16AudChannel = AM_HAL_AUDADC_FIFO_SLOT(*pui32InSampleBuffer);
              pui32LGOutBuffer->int16Sample    = AM_HAL_AUDADC_FIFO_LGDATA(*pui32InSampleBuffer) << 4; //extend to 16 bits with sign bit
              pui32LGOutBuffer++;
            }

            if ( bHighSample )
            {
              pui32HGOutBuffer->ui16AudChannel  = AM_HAL_AUDADC_FIFO_SLOT(*pui32InSampleBuffer);
              pui32HGOutBuffer->int16Sample     = AM_HAL_AUDADC_FIFO_HGDATA(*pui32InSampleBuffer) << 4;
              pui32HGOutBuffer++;
            }
            pui32InSampleBuffer++;
            (*pui32InOutNumberSamples)++;
        } while (*pui32InOutNumberSamples < ui32RequestedSamples);
    }

    //
    // Return FIFO valid bits.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Issue Software Trigger to the AUDADC.
//!
//! @param pHandle   - handle for the module instance.
//!
//! This function triggers the AUDADC operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_sw_trigger(void *pHandle)
{
    uint32_t    ui32Module = ((am_hal_audadc_state_t*)pHandle)->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Write to the Software trigger register in the AUDADC.
    //
    AUDADCn(ui32Module)->SWT = 0x37;

    //am_hal_delay_us(100);

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief AUDADC power control function
//!
//! @param pHandle      - handle for the interface.
//! @param ePowerState  - the desired power state to move the peripheral to.
//! @param bRetainState - flag (if true) to save/restore peripheral state upon
//!                       power state ui32ChanNumge.
//!
//! This function updates the peripheral to a given power state.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_power_control(void *pHandle,
                            am_hal_sysctrl_power_state_e ePowerState,
                            bool bRetainState)
{
    am_hal_audadc_state_t  *pAUDADCState = (am_hal_audadc_state_t *)pHandle;
    uint32_t            ui32Module = pAUDADCState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Decode the requested power state and update ADC operation accordingly.
    //
    switch (ePowerState)
    {
        case AM_HAL_SYSCTRL_WAKE:
            if ( bRetainState  &&  !pAUDADCState->registerState.bValid )
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }

            //
            // Enable the AUDADC power domain.
            //
            am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_AUDADC);

            if ( bRetainState )
            {
                AUDADCn(ui32Module)->SL0CFG = pAUDADCState->registerState.regSL0CFG;
                AUDADCn(ui32Module)->SL1CFG = pAUDADCState->registerState.regSL1CFG;
                AUDADCn(ui32Module)->SL2CFG = pAUDADCState->registerState.regSL2CFG;
                AUDADCn(ui32Module)->SL3CFG = pAUDADCState->registerState.regSL3CFG;
                AUDADCn(ui32Module)->SL4CFG = pAUDADCState->registerState.regSL4CFG;
                AUDADCn(ui32Module)->SL5CFG = pAUDADCState->registerState.regSL5CFG;
                AUDADCn(ui32Module)->SL6CFG = pAUDADCState->registerState.regSL6CFG;
                AUDADCn(ui32Module)->SL7CFG = pAUDADCState->registerState.regSL7CFG;
                AUDADCn(ui32Module)->WULIM  = pAUDADCState->registerState.regWULIM;
                AUDADCn(ui32Module)->WLLIM  = pAUDADCState->registerState.regWLLIM;
                AUDADCn(ui32Module)->INTEN  = pAUDADCState->registerState.regINTEN;
                AUDADCn(ui32Module)->CFG    = pAUDADCState->registerState.regCFG;

                pAUDADCState->registerState.bValid     = false;
            }
            break;

        case AM_HAL_SYSCTRL_NORMALSLEEP:
        case AM_HAL_SYSCTRL_DEEPSLEEP:
            if ( bRetainState )
            {
                pAUDADCState->registerState.regSL0CFG  = AUDADCn(ui32Module)->SL0CFG;
                pAUDADCState->registerState.regSL1CFG  = AUDADCn(ui32Module)->SL1CFG;
                pAUDADCState->registerState.regSL2CFG  = AUDADCn(ui32Module)->SL2CFG;
                pAUDADCState->registerState.regSL3CFG  = AUDADCn(ui32Module)->SL3CFG;
                pAUDADCState->registerState.regSL4CFG  = AUDADCn(ui32Module)->SL4CFG;
                pAUDADCState->registerState.regSL5CFG  = AUDADCn(ui32Module)->SL5CFG;
                pAUDADCState->registerState.regSL6CFG  = AUDADCn(ui32Module)->SL6CFG;
                pAUDADCState->registerState.regSL7CFG  = AUDADCn(ui32Module)->SL7CFG;
                pAUDADCState->registerState.regWULIM   = AUDADCn(ui32Module)->WULIM;
                pAUDADCState->registerState.regWLLIM   = AUDADCn(ui32Module)->WLLIM;
                pAUDADCState->registerState.regINTEN   = AUDADCn(ui32Module)->INTEN;
                pAUDADCState->registerState.regCFG     = AUDADCn(ui32Module)->CFG;

                pAUDADCState->registerState.bValid     = true;
            }

            //
            // Disable the AUDADC power domain.
            //
            am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_AUDADC);
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Set PGA gain
//!
//! @param  ui32ChanNum  - PGA ui32ChanNumnel number: 0:A0, 1:A1, 2:B0, 3:B1,
//! @param  in32GaindBx2 - gain in dB. in32GaindBx2 = 2 * gain (dB)
//!
//! This function sets the gain of PGA.
//!
//! @return status       - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_gain_set(uint32_t ui32ChanNum, int32_t in32GaindBx2)
{
    uint32_t temp;
    uint32_t ui32BypassStage1;
    int32_t in32Gain2Div2 = 0;
    int32_t in32Gain1Val;
    int32_t in32Gain2Val;

    // add support for -6 dB to -0.5 dB by using PGACH*GAIN2DIV2SEL
    if (in32GaindBx2 < 0)
    {
        in32Gain2Div2 = 1;
        in32GaindBx2 += 12; // subsequent calculations with 6 dB (x2) added (should be positive)
    }

    //
    // Calculate first stage gain.
    // Most of the gain comes from the first stage, unless gain < 12 dB.
    // If gain < 12 dB, we bypass first stagea
    //
    if (in32GaindBx2 < 24)
    {
        ui32BypassStage1 = 1;
        in32Gain1Val = 0;
        in32Gain2Val = in32GaindBx2;
    }
    else
    {
        ui32BypassStage1 = 0;
        in32Gain1Val = (in32GaindBx2 - 24) / 6;
        //
        // however, in32Gain1Val saturates at code 7 (33 dB)
        // (0:12, 1:15, 2:18, 3:21, 4:24, 5:27, 6:30, 7:33) dB
        //
        if (in32Gain1Val > 7)
        {
            in32Gain1Val = 7;
        }

        //
        // The remainder of the gain is provided by the second stage
        //
        in32Gain2Val = in32GaindBx2 - 24 - 6*in32Gain1Val;
    }

    switch (ui32ChanNum)
    {
        case 0:
            temp = _FLD2VAL(MCUCTRL_PGACTRL1_PGACHABYPASSEN, MCUCTRL->PGACTRL1) & 0x2; // keep CHA1 bypassen state
            temp |= ui32BypassStage1;
            temp &= 0x3;
            MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHABYPASSEN_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHABYPASSEN, temp));
            MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHAOPAMPINPDNB_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHAOPAMPINPDNB, 0x3 & ~temp)); // input stage opamp can be powered off when bypassed to save power
            //MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHAOPAMPINPDNB_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHAOPAMPINPDNB, 0x3)); // debug - set to 1
            MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHA0GAIN1SEL_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHA0GAIN1SEL, in32Gain1Val));
            MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHA0GAIN2SEL_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHA0GAIN2SEL, in32Gain2Val));
            MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHA0GAIN2DIV2SEL_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHA0GAIN2DIV2SEL, in32Gain2Div2 ? 1 : 0));
            break;
        case 1:
            temp = _FLD2VAL(MCUCTRL_PGACTRL1_PGACHABYPASSEN, MCUCTRL->PGACTRL1) & 0x1; // keep CHA0 bypassen state
            temp |= ui32BypassStage1 << 1;
            temp &= 0x3;
            MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHABYPASSEN_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHABYPASSEN, temp));
            MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHAOPAMPINPDNB_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHAOPAMPINPDNB, 0x3 & ~temp)); // input stage opamp can be powered off when bypassed to save power
            MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHA1GAIN1SEL_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHA1GAIN1SEL, in32Gain1Val));
            MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHA1GAIN2SEL_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHA1GAIN2SEL, in32Gain2Val));
            MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHA1GAIN2DIV2SEL_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHA1GAIN2DIV2SEL, in32Gain2Div2 ? 1 : 0));
            break;
        case 2:
            temp = _FLD2VAL(MCUCTRL_PGACTRL2_PGACHBBYPASSEN, MCUCTRL->PGACTRL2) & 0x2; // keep CHB1 bypassen state
            temp |= ui32BypassStage1;
            temp &= 0x3;
            MCUCTRL->PGACTRL2 = ((MCUCTRL->PGACTRL2 & ~MCUCTRL_PGACTRL2_PGACHBBYPASSEN_Msk) | _VAL2FLD(MCUCTRL_PGACTRL2_PGACHBBYPASSEN, temp));
            MCUCTRL->PGACTRL2 = ((MCUCTRL->PGACTRL2 & ~MCUCTRL_PGACTRL2_PGACHBOPAMPINPDNB_Msk) | _VAL2FLD(MCUCTRL_PGACTRL2_PGACHBOPAMPINPDNB, 0x3 & ~temp)); // input stage opamp can be powered off when bypassed to save power
            MCUCTRL->PGACTRL2 = ((MCUCTRL->PGACTRL2 & ~MCUCTRL_PGACTRL2_PGACHB0GAIN1SEL_Msk) | _VAL2FLD(MCUCTRL_PGACTRL2_PGACHB0GAIN1SEL, in32Gain1Val));
            MCUCTRL->PGACTRL2 = ((MCUCTRL->PGACTRL2 & ~MCUCTRL_PGACTRL2_PGACHB0GAIN2SEL_Msk) | _VAL2FLD(MCUCTRL_PGACTRL2_PGACHB0GAIN2SEL, in32Gain2Val));
            MCUCTRL->PGACTRL2 = ((MCUCTRL->PGACTRL2 & ~MCUCTRL_PGACTRL2_PGACHB0GAIN2DIV2SEL_Msk) | _VAL2FLD(MCUCTRL_PGACTRL2_PGACHB0GAIN2DIV2SEL, in32Gain2Div2 ? 1 : 0));
            break;
        case 3:
            temp = _FLD2VAL(MCUCTRL_PGACTRL2_PGACHBBYPASSEN, MCUCTRL->PGACTRL2) & 0x1; // keep CHB0 bypassen state
            temp |= ui32BypassStage1 << 1;
            temp &= 0x3;
            MCUCTRL->PGACTRL2 = ((MCUCTRL->PGACTRL2 & ~MCUCTRL_PGACTRL2_PGACHBBYPASSEN_Msk) | _VAL2FLD(MCUCTRL_PGACTRL2_PGACHBBYPASSEN, temp));
            MCUCTRL->PGACTRL2 = ((MCUCTRL->PGACTRL2 & ~MCUCTRL_PGACTRL2_PGACHBOPAMPINPDNB_Msk) | _VAL2FLD(MCUCTRL_PGACTRL2_PGACHBOPAMPINPDNB, 0x3 & ~temp)); // input stage opamp can be powered off when bypassed to save power
            MCUCTRL->PGACTRL2 = ((MCUCTRL->PGACTRL2 & ~MCUCTRL_PGACTRL2_PGACHB1GAIN1SEL_Msk) | _VAL2FLD(MCUCTRL_PGACTRL2_PGACHB1GAIN1SEL, in32Gain1Val));
            MCUCTRL->PGACTRL2 = ((MCUCTRL->PGACTRL2 & ~MCUCTRL_PGACTRL2_PGACHB1GAIN2SEL_Msk) | _VAL2FLD(MCUCTRL_PGACTRL2_PGACHB1GAIN2SEL, in32Gain2Val));
            MCUCTRL->PGACTRL2 = ((MCUCTRL->PGACTRL2 & ~MCUCTRL_PGACTRL2_PGACHB1GAIN2DIV2SEL_Msk) | _VAL2FLD(MCUCTRL_PGACTRL2_PGACHB1GAIN2DIV2SEL, in32Gain2Div2 ? 1 : 0));
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Turn on reference
//!
//! This function turn on the reference voltage and current.
//!
//! @return status          - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_refgen_powerup(void)
{
    //
    // Turn on PGA VREFGEN
    //
    MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGAVREFGENPDNB_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGAVREFGENPDNB, 1));

    //
    // Turn on PGA IREFGEN
    //
    MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGAIREFGENPDNB_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGAIREFGENPDNB, 1));

    //
    // Turn on PGA VREFGEN Quick Charge
    //
    MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGAVREFGENQUICKSTARTEN_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGAVREFGENQUICKSTARTEN, 1));

    //
    // Delay 4ms
    //
    am_hal_delay_us(4000);

    //
    // Turn off PGA VREFGEN Quick Charge
    //
    MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGAVREFGENQUICKSTARTEN_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGAVREFGENQUICKSTARTEN, 0));

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Turn off reference
//!
//! This function turn off the reference voltage and current.
//!
//! @return status          - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_refgen_powerdown(void)
{
    g_AUDADCPgaChConfigured = 0x0;
    //
    // Turn on PGA VREFGEN
    //
    MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGAVREFGENPDNB_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGAVREFGENPDNB, 0));

    //
    // Turn on PGA IREFGEN
    //
    MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGAIREFGENPDNB_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGAIREFGENPDNB, 0));

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}
//*****************************************************************************
//
//! @brief Turn on PGA
//!
//! @param  ui32ChanNum     - PGA ui32ChanNumnel number: 0:A0, 1:A1, 2:B0, 3:B1,
//!
//! This function turn on PGA.
//!
//! @return status          - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_pga_powerup(uint32_t ui32ChanNum)
{
    //
    // turn on quick charge
    //
    switch (ui32ChanNum)
    {
        case 0:
            g_AUDADCPgaChConfigured |= 0x01;
            MCUCTRL->PGACTRL1 |= (_VAL2FLD(MCUCTRL_PGACTRL1_PGACHAVCMGENPDNB, 1)) | (_VAL2FLD(MCUCTRL_PGACTRL1_PGACHAVCMGENQCHARGEEN, 1));
            break;

        case 1:
            g_AUDADCPgaChConfigured |= 0x02;
            MCUCTRL->PGACTRL1 |= (_VAL2FLD(MCUCTRL_PGACTRL1_PGACHAVCMGENPDNB, 1)) | (_VAL2FLD(MCUCTRL_PGACTRL1_PGACHAVCMGENQCHARGEEN, 1));
            break;

        case 2:
            g_AUDADCPgaChConfigured |= 0x04;
            MCUCTRL->PGACTRL2 |= (_VAL2FLD(MCUCTRL_PGACTRL2_PGACHBVCMGENPDNB, 1)) | (_VAL2FLD(MCUCTRL_PGACTRL2_PGACHBVCMGENQCHARGEEN, 1));
            break;

        case 3:
            g_AUDADCPgaChConfigured |= 0x08;
            MCUCTRL->PGACTRL2 |= (_VAL2FLD(MCUCTRL_PGACTRL2_PGACHBVCMGENPDNB, 1)) | (_VAL2FLD(MCUCTRL_PGACTRL2_PGACHBVCMGENQCHARGEEN, 1));
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Delay 4ms
    //
    am_hal_delay_us(4000 + 400);    // +10% for modeling differences

    //
    // turn off quick charge
    //
    switch (ui32ChanNum)
    {
        case 0:
        //
        // Turn on ChA VCMGEN & VCM Quick Charge
        //
//        MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHAVCMGENPDNB_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHAVCMGENPDNB, 1));
//        MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHAVCMGENQCHARGEEN_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHAVCMGENQCHARGEEN, 1));
            MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHAVCMGENQCHARGEEN_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHAVCMGENQCHARGEEN, 0));
            break;

        case 1:
            MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHAVCMGENQCHARGEEN_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHAVCMGENQCHARGEEN, 0));
            break;

        case 2:
            MCUCTRL->PGACTRL2 = ((MCUCTRL->PGACTRL2 & ~MCUCTRL_PGACTRL2_PGACHBVCMGENQCHARGEEN_Msk) | _VAL2FLD(MCUCTRL_PGACTRL2_PGACHBVCMGENQCHARGEEN, 0));
            break;

        case 3:
            MCUCTRL->PGACTRL2 = ((MCUCTRL->PGACTRL2 & ~MCUCTRL_PGACTRL2_PGACHBVCMGENQCHARGEEN_Msk) | _VAL2FLD(MCUCTRL_PGACTRL2_PGACHBVCMGENQCHARGEEN, 0));
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Turn on ChA0/ChA1 opamps
    //
    switch (ui32ChanNum)
    {
        case 0:
            MCUCTRL->PGACTRL1 |= (_VAL2FLD(MCUCTRL_PGACTRL1_PGACHAOPAMPOUTPDNB, 0x1));
            MCUCTRL->PGAADCIFCTRL |= (_VAL2FLD(MCUCTRL_PGAADCIFCTRL_PGAADCIFCHAPDNB, 0x1));
            break;

        case 1:
            MCUCTRL->PGACTRL1 |= (_VAL2FLD(MCUCTRL_PGACTRL1_PGACHAOPAMPOUTPDNB, 0x2));
            MCUCTRL->PGAADCIFCTRL |= (_VAL2FLD(MCUCTRL_PGAADCIFCTRL_PGAADCIFCHAPDNB, 0x2));
            break;

        case 2:
            MCUCTRL->PGACTRL2 |= (_VAL2FLD(MCUCTRL_PGACTRL2_PGACHBOPAMPOUTPDNB, 0x1));
            MCUCTRL->PGAADCIFCTRL |= (_VAL2FLD(MCUCTRL_PGAADCIFCTRL_PGAADCIFCHBPDNB, 0x1));
            break;

        case 3:
            MCUCTRL->PGACTRL2 |= (_VAL2FLD(MCUCTRL_PGACTRL2_PGACHBOPAMPOUTPDNB, 0x2));
            MCUCTRL->PGAADCIFCTRL |= (_VAL2FLD(MCUCTRL_PGAADCIFCTRL_PGAADCIFCHBPDNB, 0x2));
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Turn off PGA
//!
//! @param  ui32ChanNum     - PGA ui32ChanNumnel number: 0:A0, 1:A1, 2:B0, 3:B1,
//!
//! This function is to turn off PGA.
//!
//! @return status          - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_pga_powerdown(uint32_t ui32ChanNum)
{
    switch (ui32ChanNum)
    {
        case 0:
            g_AUDADCPgaChConfigured &= 0xe;
            MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHABYPASSEN_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHABYPASSEN, g_AUDADCPgaChConfigured & 0x3));
            MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHAOPAMPOUTPDNB_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHAOPAMPOUTPDNB, g_AUDADCPgaChConfigured & 0x3));
            MCUCTRL->PGAADCIFCTRL = ((MCUCTRL->PGAADCIFCTRL & ~MCUCTRL_PGAADCIFCTRL_PGAADCIFCHAPDNB_Msk) | _VAL2FLD(MCUCTRL_PGAADCIFCTRL_PGAADCIFCHAPDNB, g_AUDADCPgaChConfigured & 0x3));
            break;

        case 1:
            g_AUDADCPgaChConfigured &= 0xd;
            MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHABYPASSEN_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHABYPASSEN, g_AUDADCPgaChConfigured & 0x3));
            MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHAOPAMPOUTPDNB_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHAOPAMPOUTPDNB, g_AUDADCPgaChConfigured&0x3));
            MCUCTRL->PGAADCIFCTRL = ((MCUCTRL->PGAADCIFCTRL & ~MCUCTRL_PGAADCIFCTRL_PGAADCIFCHAPDNB_Msk) | _VAL2FLD(MCUCTRL_PGAADCIFCTRL_PGAADCIFCHAPDNB, g_AUDADCPgaChConfigured & 0x3));
            break;

        case 2:
            g_AUDADCPgaChConfigured &= 0xb;
            MCUCTRL->PGACTRL2 = ((MCUCTRL->PGACTRL2 & ~MCUCTRL_PGACTRL2_PGACHBBYPASSEN_Msk) | _VAL2FLD(MCUCTRL_PGACTRL2_PGACHBBYPASSEN, (g_AUDADCPgaChConfigured & 0xC)>>2));
            MCUCTRL->PGACTRL2 = ((MCUCTRL->PGACTRL2 & ~MCUCTRL_PGACTRL2_PGACHBOPAMPOUTPDNB_Msk) | _VAL2FLD(MCUCTRL_PGACTRL2_PGACHBOPAMPOUTPDNB, (g_AUDADCPgaChConfigured & 0xC)>>2));
            MCUCTRL->PGAADCIFCTRL = ((MCUCTRL->PGAADCIFCTRL & ~MCUCTRL_PGAADCIFCTRL_PGAADCIFCHBPDNB_Msk) | _VAL2FLD(MCUCTRL_PGAADCIFCTRL_PGAADCIFCHBPDNB, (ui32ChanNum & 0xC) >> 2));
            break;

        case 3:
            g_AUDADCPgaChConfigured &= 0x7;
            MCUCTRL->PGACTRL2 = ((MCUCTRL->PGACTRL2 & ~MCUCTRL_PGACTRL2_PGACHBBYPASSEN_Msk) | _VAL2FLD(MCUCTRL_PGACTRL2_PGACHBBYPASSEN, (g_AUDADCPgaChConfigured & 0xC)>>2));
            MCUCTRL->PGACTRL2 = ((MCUCTRL->PGACTRL2 & ~MCUCTRL_PGACTRL2_PGACHBOPAMPOUTPDNB_Msk) | _VAL2FLD(MCUCTRL_PGACTRL2_PGACHBOPAMPOUTPDNB, (g_AUDADCPgaChConfigured & 0xC)>>2));
            MCUCTRL->PGAADCIFCTRL = ((MCUCTRL->PGAADCIFCTRL & ~MCUCTRL_PGAADCIFCTRL_PGAADCIFCHBPDNB_Msk) | _VAL2FLD(MCUCTRL_PGAADCIFCTRL_PGAADCIFCHBPDNB, (ui32ChanNum & 0xC) >> 2));
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Turn off ChA VCM
    //
    if ((g_AUDADCPgaChConfigured & 0x3) == 0x0)
    {
        MCUCTRL->PGACTRL1 = ((MCUCTRL->PGACTRL1 & ~MCUCTRL_PGACTRL1_PGACHAVCMGENPDNB_Msk) | _VAL2FLD(MCUCTRL_PGACTRL1_PGACHAVCMGENPDNB, 0));
    }

    //
    // Turn off ChB VCM
    //
    if ((g_AUDADCPgaChConfigured & 0xC) == 0x0)
    {
        MCUCTRL->PGACTRL2 = ((MCUCTRL->PGACTRL2 & ~MCUCTRL_PGACTRL2_PGACHBVCMGENPDNB_Msk) | _VAL2FLD(MCUCTRL_PGACTRL2_PGACHBVCMGENPDNB, 0));
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Turn on mic bias voltage to power up the mic
//!
//! @param  ui32VolTrim         - The value to set the output voltage
//!
//! This function sets output of the mic bias voltage.
//!
//*****************************************************************************
void am_hal_audadc_micbias_powerup(uint32_t ui32VolTrim)
{
    MCUCTRL->AUDIO1 = ((MCUCTRL->AUDIO1 & ~MCUCTRL_AUDIO1_MICBIASVOLTAGETRIM_Msk) | _VAL2FLD(MCUCTRL_AUDIO1_MICBIASVOLTAGETRIM, ui32VolTrim & 0x3f));
    MCUCTRL->AUDIO1 = ((MCUCTRL->AUDIO1 & ~MCUCTRL_AUDIO1_MICBIASPDNB_Msk) | _VAL2FLD(MCUCTRL_AUDIO1_MICBIASPDNB, 1));
    //
    // take 1ms to power up
    //
    am_hal_delay_us(1000);
}

//*****************************************************************************
//
//! @brief Turn off mic bias voltage to power off the mic
//!
//! This function turns off the mic bias voltage.
//!
//*****************************************************************************
void am_hal_audadc_micbias_powerdown(void)
{
    MCUCTRL->AUDIO1 = ((MCUCTRL->AUDIO1 & ~MCUCTRL_AUDIO1_MICBIASPDNB_Msk) | _VAL2FLD(MCUCTRL_AUDIO1_MICBIASPDNB, 0));
}

//*****************************************************************************
//
//! @brief Config internal PGA.
//!
//! @param pHandle      - handle for the interface.
//! @param psGainConfig - pointer to the AUDADC Gain Config
//!
//! This function configures the AUADC Pre-Amplifier Gain.
//!
//! @return status          - generic or interface specific status.
//!
//*****************************************************************************
uint32_t am_hal_audadc_internal_pga_config(void *pHandle, am_hal_audadc_gain_config_t* psGainConfig)
{
    am_hal_audadc_state_t  *pAUDADCState = (am_hal_audadc_state_t *)pHandle;
    uint32_t ui32Module = pAUDADCState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Set the AUDADC gain codes.
    //
    AUDADCn(ui32Module)->GAIN =
         _VAL2FLD(AUDADC_GAIN_LGA, psGainConfig->ui32LGA)            |
         _VAL2FLD(AUDADC_GAIN_HGADELTA, psGainConfig->ui32HGADELTA)  |
         _VAL2FLD(AUDADC_GAIN_LGB, psGainConfig->ui32LGB)            |
         _VAL2FLD(AUDADC_GAIN_HGBDELTA, psGainConfig->ui32HGBDELTA);

    //
    // PGA Gain Configuration
    //
    AUDADCn(ui32Module)->GAINCFG = _VAL2FLD(AUDADC_GAINCFG_UPDATEMODE, psGainConfig->eUpdateMode) |
                                   _VAL2FLD(AUDADC_GAINCFG_PGACTRLEN, 1);

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Config Saturation Comparator.
//!
//! @param pHandle     - handle for the interface.
//! @param psSATConfig - pointer to the AUDADC Saturation Config
//!
//! This function configures the Saturation Comparator.
//!
//! @return status          - generic or interface specific status.
//!
//*****************************************************************************
uint32_t am_hal_audadc_saturation_config(void *pHandle, am_hal_audadc_sat_config_t* psSATConfig)
{
    am_hal_audadc_state_t  *pAUDADCState = (am_hal_audadc_state_t *)pHandle;
    uint32_t ui32Module = pAUDADCState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_AUDADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    AUDADCn(ui32Module)->SATLIM = 0;

    AUDADCn(ui32Module)->SATMAX = 0;

    //
    // Saturation Comparator Configuration
    //
    AUDADCn(ui32Module)->SATCFG = _VAL2FLD(AUDADC_SATCFG_SATCHANSEL, psSATConfig->ui32ChanSel) |
                                  _VAL2FLD(AUDADC_SATCFG_SATEN, 1);

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
