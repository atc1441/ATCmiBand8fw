//*****************************************************************************
//
//! @file am_hal_adc.c
//!
//! @brief Functions for interfacing with the Analog to Digital Converter.
//!
//! @addtogroup adc4_4p ADC - Analog-to-Digital Converter
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

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

//*****************************************************************************
//
// Private Types.
//
//*****************************************************************************
#define AM_HAL_MAGIC_ADC                0xAFAFAF
#define AM_HAL_ADC_CHK_HANDLE(h)        ((h) && ((am_hal_handle_prefix_t *)(h))->s.bInit && (((am_hal_handle_prefix_t *)(h))->s.magic == AM_HAL_MAGIC_ADC))

#define ADC_CRITICAL_BEGIN(ui32Module)                              \
        if ( 1 )                                                    \
        {                                                           \
            uint32_t ui32adcintensave, ui32adcintstatsave;          \
            ui32adcintensave = ADCn(ui32Module)->INTEN;             \
            ADCn(ui32Module)->INTEN = 0x0;                          \
            ui32adcintstatsave = ADCn(ui32Module)->INTSTAT;         \

#define ADC_CRITICAL_END(ui32Module)                                \
            if ( !(ui32adcintstatsave & (ADC_INTCLR_SCNCMP_Msk |    \
                                         ADC_INTCLR_CNVCMP_Msk)))   \
            {                                                       \
                ADCn(ui32Module)->INTCLR = ADC_INTCLR_SCNCMP_Msk |  \
                                           ADC_INTCLR_CNVCMP_Msk;   \
            }                                                       \
            ADCn(ui32Module)->INTEN = ui32adcintensave;             \
        }

//*****************************************************************************
//
//! @name Calibration Coefficients
//! @{
//! Default coefficients (used when trims not provided):
//!  TEMP_DEFAULT    = Temperature in deg K (e.g. 299.5 - 273.15 = 26.35)
//!  AMBIENT_DEFAULT = Voltage measurement at default temperature.
//!  OFFSET_DEFAULT  = Default ADC offset at 1v.
//
//*****************************************************************************
#define AM_HAL_ADC_CALIB_TEMP_DEFAULT               (299.5F)
#define AM_HAL_ADC_CALIB_AMBIENT_DEFAULT            (1.02809F)
#define AM_HAL_ADC_CALIB_ADC_OFFSET_DEFAULT         (-0.004281F)
//! @}

// ****************************************************************************
//
// State
//
// ****************************************************************************
// ****************************************************************************
//
// Storage for pre-computed constant terms in the temperature sensor equation.
//
// ****************************************************************************
static float g_fTempEqnTerms = 0.0F;

// ****************************************************************************
//
//! @brief ADC Power save register state.
//
// ****************************************************************************
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
    uint32_t      regIntTrigTmr;
    uint32_t      regWULIM;
    uint32_t      regWLLIM;
    uint32_t      regINTEN;
} am_hal_adc_register_state_t;

// ****************************************************************************
//
//! @brief ADC State structure.
//
// ****************************************************************************
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
    //! ADC Capabilities.
    //
    am_hal_adc_capabilities_t   capabilities;

    //
    //! Power Save-Restore register state
    //
    am_hal_adc_register_state_t registerState;

} am_hal_adc_state_t;

//*****************************************************************************
//
//! @brief Private SRAM view of temperature trims.
//!
//! This static SRAM union is private to the ADC HAL functions.
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

        //! ADC offset voltage measured on the package test head.
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

        //! ADC offset voltage measured on the package test head.
        //! This value is recorded in volts, although the value
        //! itself is likely to be on the order of millivolts.
        float    fCalibrationOffset;

        //! Flag if default (guess) or measured.
        float fMeasuredFlag;
    } flt;
} priv_temp_trims;

//*****************************************************************************
//
//! @brief Private SRAM view of correction trims.
//!
//! This static SRAM union is private to the ADC HAL functions.
//
//*****************************************************************************
static union
{
    //
    // Define access to the general ADC correction values.
    //
    // These correction values are loaded as uint32_t values.
    struct
    {
        uint32_t ui32ADCoffset;
        uint32_t ui32ADCgain;
        uint32_t ui32Sample;
    } ui32;
    // These correction values are accessed as floats
    struct
    {
        float    fADCoffset;
        float    fADCgain;
        float    fSample;
    } flt;
} priv_correction_trims;

//*****************************************************************************
//
// Global Variables.
//
//*****************************************************************************
am_hal_adc_state_t             g_ADCState[AM_REG_ADC_NUM_MODULES];

uint32_t                       g_ADCSlotsConfigured;

bool     g_bDoADCadjust   = false;

//*****************************************************************************
//
//! @brief ForceFIFOpop()
//
// ERR090: ADC "No CNVCMP interrupt for first single scan"
// Please refer to the Apollo4 errata for further information.
//
//*****************************************************************************
static void
ForceFIFOpop(void *pHandle)
{
    uint32_t ui32Module = ((am_hal_adc_state_t *)pHandle)->ui32Module;

    //
    // Make sure the ADC is properly configured and enabled.
    //
    if ( !((ADCn(ui32Module)->CFG_b.RPTEN == ADC_CFG_RPTEN_SINGLE_SCAN)     &&
           (ADCn(ui32Module)->CFG_b.ADCEN == ADC_CFG_ADCEN_EN))                 &&
         !((ADCn(ui32Module)->CFG_b.ADCEN == ADC_CFG_ADCEN_EN)              &&
           (ADCn(ui32Module)->CFG_b.RPTEN == ADC_CFG_RPTEN_REPEATING_SCAN)  &&
           (ADCn(ui32Module)->INTTRIGTIMER_b.TIMEREN == ADC_INTTRIGTIMER_TIMEREN_EN)) )
    {
        return;
    }

    //
    // ERR090: After enable, a forced FIFO pop is required to make
    //         sure that the first sample actually emitted is valid.
    //
    while ( _FLD2VAL(ADC_FIFO_COUNT, ADCn(ui32Module)->FIFO) == 0 )
    {
        am_hal_adc_sw_trigger(pHandle);
        am_hal_delay_us(30);
    }

    while ( _FLD2VAL(ADC_FIFO_COUNT, ADCn(ui32Module)->FIFO) )
    {
        ADCn(ui32Module)->FIFO = 0;     // Pop the FIFO
    }

} // ForceFIFOpop()

//*****************************************************************************
//
//! @brief ADC initialization function
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
am_hal_adc_initialize(uint32_t ui32Module, void **ppHandle)
{
    uint32_t ui32Ret;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Validate the module number
    //
    if ( ui32Module >= AM_REG_ADC_NUM_MODULES )
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
    if ( g_ADCState[ui32Module].prefix.s.bInit )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Initialize the handle.
    //
    g_ADCState[ui32Module].prefix.s.bInit = true;
    g_ADCState[ui32Module].prefix.s.magic = AM_HAL_MAGIC_ADC;
    g_ADCState[ui32Module].ui32Module = ui32Module;

    //
    // Initialize the number of slots configured.
    //
    g_ADCSlotsConfigured = 0;

    //
    // Return the handle.
    //
    *ppHandle = (void *)&g_ADCState[ui32Module];

    //
    // Before returning, read the temperature trims from INFO1,
    // which are safely read using am_hal_mram_info_read().
    //
    ui32Ret  = am_hal_mram_info_read(1, AM_REG_INFO1_TEMP_CAL_ATE_O / 4, 1,     // INFO1, word offset, read 1 word
                                     &priv_temp_trims.ui32.ui32CalibrationTemperature);
    ui32Ret |= am_hal_mram_info_read(1, AM_REG_INFO1_TEMP_CAL_MEASURED_O / 4, 1,
                                     &priv_temp_trims.ui32.ui32CalibrationVoltage);
    ui32Ret |= am_hal_mram_info_read(1, AM_REG_INFO1_TEMP_CAL_ADC_OFFSET_O / 4, 1,
                                     &priv_temp_trims.ui32.ui32CalibrationOffset);

    if ( (priv_temp_trims.ui32.ui32CalibrationTemperature == 0xFFFFFFFF)    ||
         (priv_temp_trims.ui32.ui32CalibrationVoltage     == 0xFFFFFFFF)    ||
         (priv_temp_trims.ui32.ui32CalibrationOffset      == 0xFFFFFFFF)    ||
         (ui32Ret != 0) )
    {
        //
        // Since the device has not been calibrated on the tester, we'll load
        // default calibration values.  These default values should result
        // in worst-case temperature measurements of +-6 degress C.
        //
        priv_temp_trims.flt.fCalibrationTemperature = AM_HAL_ADC_CALIB_TEMP_DEFAULT;
        priv_temp_trims.flt.fCalibrationVoltage     = AM_HAL_ADC_CALIB_AMBIENT_DEFAULT;
        priv_temp_trims.flt.fCalibrationOffset      = AM_HAL_ADC_CALIB_ADC_OFFSET_DEFAULT;
        priv_temp_trims.ui32.bMeasured = false;
    }
    else
    {
        priv_temp_trims.ui32.bMeasured = true;
    }

    //
    // Get and save the ADC gain/offset correction values. These are really only
    // needed for rev B0 and B1, but they will also be useful on later revs.
    // Note: Only applies to the general ADC, not for AUDADC.
    //
    // If the correction values are 0xFFFFFFFF then we must disable CALONPWRUP.
    //
    ui32Ret  = am_hal_mram_info_read(1, AM_REG_INFO1_ADC_GAIN_ERR_O / 4, 1,
                                     &priv_correction_trims.ui32.ui32ADCgain);
    ui32Ret |= am_hal_mram_info_read(1, AM_REG_INFO1_ADC_OFFSET_ERR_O / 4, 1,
                                     &priv_correction_trims.ui32.ui32ADCoffset);

    //
    // Disable CALONPWRUP so that corrections can be applied to samples.
    //
    MCUCTRL->ADCCAL_b.CALONPWRUP = MCUCTRL_ADCCAL_CALONPWRUP_DIS;

    if ( (ui32Ret != 0)                                             ||
         (priv_correction_trims.ui32.ui32ADCgain   == 0xFFFFFFFF)   ||
         (priv_correction_trims.ui32.ui32ADCoffset == 0xFFFFFFFF) )
    {
        g_bDoADCadjust = false;
    }
    else
    {
        g_bDoADCadjust = true;
    }

    //
    // Return the status.
    // Note - This does not infer that temperature calibrations are valid.
    // The caller must check bMeasured to determine that.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_adc_initialize()

//*****************************************************************************
//
//! @brief ADC deinitialization function
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
am_hal_adc_deinitialize(void *pHandle)
{
    uint32_t            status = AM_HAL_STATUS_SUCCESS;
    am_hal_adc_state_t  *pADCState = (am_hal_adc_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_ADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    if ( pADCState->prefix.s.bEnable )
    {
        status = am_hal_adc_disable(pHandle);
    }

    pADCState->prefix.s.bInit = false;

    //
    // Return the status.
    //
    return status;

} // am_hal_adc_deinitialize()

//*****************************************************************************
//
//! @brief ADC configuration function
//!
//! @param pHandle   - handle for the module instance.
//! @param psConfig  - pointer to the configuration structure.
//!
//! This function configures the ADC for operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_adc_configure(void *pHandle,
                     am_hal_adc_config_t *psConfig)
{
    uint32_t            ui32Config;
    am_hal_adc_state_t  *pADCState = (am_hal_adc_state_t *)pHandle;
    uint32_t            ui32Module = pADCState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_ADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Config = 0;

    //
    // Set the ADC clock source.
    //
    if ( psConfig->eClock != AM_HAL_ADC_CLKSEL_HFRC_24MHZ )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ui32Config |= _VAL2FLD(ADC_CFG_CLKSEL, psConfig->eClock);

    //
    // Set the ADC periodic trigger source.
    //
    ui32Config |= _VAL2FLD(ADC_CFG_RPTTRIGSEL, psConfig->eRepeatTrigger);

    //
    // Set the ADC trigger polarity.
    //
    ui32Config |= _VAL2FLD(ADC_CFG_TRIGPOL, psConfig->ePolarity);

    //
    // Set the ADC trigger.
    //
    ui32Config |= _VAL2FLD(ADC_CFG_TRIGSEL, psConfig->eTrigger);

    //
    // Set the Destructive FIFO read.
    //
    ui32Config |= _VAL2FLD(ADC_CFG_DFIFORDEN, 1);

    //
    // Set the ADC clock mode.
    //
    ui32Config |= _VAL2FLD(ADC_CFG_CKMODE, psConfig->eClockMode);

    //
    // Set the ADC low power mode.
    //
    ui32Config |= _VAL2FLD(ADC_CFG_LPMODE, psConfig->ePowerMode);

    //
    // Set the ADC repetition mode.
    //
    ui32Config |= _VAL2FLD(ADC_CFG_RPTEN, psConfig->eRepeat);

    //
    // Set the configuration in the ADC peripheral.
    //
    ADCn(ui32Module)->CFG = ui32Config;

    //
    // Return status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_adc_configure()

//*****************************************************************************
//
//! @brief ADC slot configuration function
//!
//! @param pHandle - handle for the module instance.
//! @param ui32SlotNumber - ADC Slot Number
//! @param pSlotConfig    - pointer to the configuration structure.
//!
//! This function configures the ADC slot for operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_adc_configure_slot(void *pHandle,
                          uint32_t ui32SlotNumber,
                          am_hal_adc_slot_config_t *pSlotConfig)
{
    uint32_t            ui32Config;
    uint32_t            ui32RegOffset;
    am_hal_adc_state_t  *pADCState = (am_hal_adc_state_t *)pHandle;
    uint32_t            ui32Module = pADCState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_ADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    //
    // Check the slot number.
    //
    if ( ui32SlotNumber >= AM_HAL_ADC_MAX_SLOTS )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    if ( (pSlotConfig->ui32TrkCyc < AM_HAL_ADC_MIN_TRKCYC) ||
         (pSlotConfig->ui32TrkCyc > _FLD2VAL(ADC_SL0CFG_TRKCYC0, 0xFFFFFFFF)) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Config = 0;

    //
    // Set the measurements to average
    //
    ui32Config |= _VAL2FLD(ADC_SL0CFG_ADSEL0, pSlotConfig->eMeasToAvg);

    //
    // Set additional sampling ADC clock cycles
    //
    ui32Config |= _VAL2FLD(ADC_SL0CFG_TRKCYC0, pSlotConfig->ui32TrkCyc);

    //
    // Set the precision mode.
    //
    ui32Config |= _VAL2FLD(ADC_SL0CFG_PRMODE0, pSlotConfig->ePrecisionMode);

    //
    // Set the channel.
    //
    ui32Config |= _VAL2FLD(ADC_SL0CFG_CHSEL0, pSlotConfig->eChannel);

    //
    // Enable window comparison if configured.
    //
    ui32Config |= _VAL2FLD(ADC_SL0CFG_WCEN0, pSlotConfig->bWindowCompare);

    //
    // Enable the slot if configured.
    //
    ui32Config |= _VAL2FLD(ADC_SL0CFG_SLEN0, pSlotConfig->bEnabled);

    //
    // Locate the correct register for this ADC slot.
    //
    ui32RegOffset = ((uint32_t)&ADCn(ui32Module)->SL0CFG) + (4 * ui32SlotNumber);

    //
    // Write the register with the caller's configuration value.
    //
    AM_REGVAL(ui32RegOffset) = ui32Config;

    //
    // Update the nubmer of slots configured.
    //
    g_ADCSlotsConfigured++;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_adc_configure_slot()

//*****************************************************************************
//
//! @brief ADC internal repeat trigger timer configuration function
//!
//! @param pHandle - handle for the module instance.
//! @param pConfig - pointer to the configuration structure.
//!
//! This function configures the ADC internal repeat trigger timer for operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_adc_configure_irtt(void *pHandle,
                          am_hal_adc_irtt_config_t *pConfig)
{
    uint32_t    ui32Config = 0;
    uint32_t    ui32Module = ((am_hal_adc_state_t *)pHandle)->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_ADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Disable ADC internal repeating trigger timer
    //
    ui32Config |= _VAL2FLD(ADC_INTTRIGTIMER_TIMEREN, ADC_INTTRIGTIMER_TIMEREN_DIS);

    //
    // Set ADC internal repeating trigger timer clock division
    //
    ui32Config |= _VAL2FLD(ADC_INTTRIGTIMER_CLKDIV, pConfig->eClkDiv);

    //
    // Set ADC internal repeating trigger timer count
    //
    ui32Config |= _VAL2FLD(ADC_INTTRIGTIMER_TIMERMAX, pConfig->ui32IrttCountMax);

    //
    // Set ADC internal repeating trigger timer configuration.
    //
    ADCn(ui32Module)->INTTRIGTIMER = ui32Config;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_adc_configure_irtt()

//*****************************************************************************
//
//! @brief ADC internal repeating trigger timer enable function
//!
//! @param pHandle   - handle for the module instance.
//!
//! This function enables internal repeating trigger timer.
//!
//! @note - am_hal_adc_enable() must be called before this function.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_adc_irtt_enable(void *pHandle)
{
    am_hal_adc_state_t  *pADCState = (am_hal_adc_state_t *)pHandle;
    uint32_t            ui32Module = pADCState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_ADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    //
    // Make sure the ADC has been enabled.
    //
    if ( ADCn(ui32Module)->CFG_b.ADCEN != ADC_CFG_ADCEN_EN )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Enable the ADC.
    //
    ADC_CRITICAL_BEGIN(ui32Module)
    ADCn(ui32Module)->INTTRIGTIMER_b.TIMEREN = ADC_INTTRIGTIMER_TIMEREN_EN;
    ForceFIFOpop(pHandle);  // See errata ERR090
    ADC_CRITICAL_END(ui32Module)

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_adc_irtt_enable()

//*****************************************************************************
//
//! @brief ADC internal repeating trigger timer disable function
//!
//! @param pHandle   - handle for the module instance.
//!
//! This function disables internal repeating trigger timer.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_adc_irtt_disable(void *pHandle)
{
    am_hal_adc_state_t  *pADCState = (am_hal_adc_state_t *)pHandle;
    uint32_t            ui32Module = pADCState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_ADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Enable the ADC.
    //
    ADCn(ui32Module)->INTTRIGTIMER_b.TIMEREN = ADC_INTTRIGTIMER_TIMEREN_DIS;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_adc_irtt_disable()

//*****************************************************************************
//
//! @brief ADC DMA configuration function
//!
//! @param pHandle   - handle for the module instance.
//! @param pDMAConfig  - pointer to the configuration structure.
//!
//! This function configures the ADC DMA for operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_adc_configure_dma(void *pHandle,
                         am_hal_adc_dma_config_t *pDMAConfig)
{
    uint32_t    ui32Config;
    uint32_t    ui32Module = ((am_hal_adc_state_t *)pHandle)->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_ADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Config = 0;

    //
    // Configure the DMA complete power-off.
    //
    ui32Config |= _VAL2FLD(ADC_DMACFG_DPWROFF, 0);      // DPWROFF not supported!

    //
    // Configure the data to be transferred.
    //
    if ( g_ADCSlotsConfigured > 1 )
    {
        // Need slot number to distinguish between slot results.
        ui32Config |= _VAL2FLD(ADC_DMACFG_DMAMSK, ADC_DMACFG_DMAMSK_DIS);
    }
    else
    {
        ui32Config |= _VAL2FLD(ADC_DMACFG_DMAMSK, ADC_DMACFG_DMAMSK_EN);
    }

    //
    // Enable DMA Halt on Status (DMAERR or DMACPL) by default. This bit is reserved in apollo4
    //
//    ui32Config |= _VAL2FLD(ADC_DMACFG_DMAHONSTAT, ADC_DMACFG_DMAHONSTAT_EN);

    //
    // Configure the DMA dynamic priority handling.
    //
    ui32Config |= _VAL2FLD(ADC_DMACFG_DMADYNPRI, pDMAConfig->bDynamicPriority);

    //
    // Configure the DMA static priority.
    //
    ui32Config |= _VAL2FLD(ADC_DMACFG_DMAPRI, pDMAConfig->ePriority);

    //
    // Enable the DMA (does not start until ADC is enabled and triggered).
    //
    ui32Config |= _VAL2FLD(ADC_DMACFG_DMAEN, ADC_DMACFG_DMAEN_EN);

    //
    // Set the DMA configuration.
    //
    ADCn(ui32Module)->DMACFG = ui32Config;

    //
    // Set the DMA transfer count.
    //
    ADCn(ui32Module)->DMATOTCOUNT_b.TOTCOUNT = pDMAConfig->ui32SampleCount;

    //
    // Set the DMA target address.
    //
    ADCn(ui32Module)->DMATARGADDR = pDMAConfig->ui32TargetAddress;

    //
    // Set the DMA trigger on FIFO 75% full.
    //
    ADCn(ui32Module)->DMATRIGEN = ADC_DMATRIGEN_DFIFO75_Msk;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_adc_configure_dma()

//*****************************************************************************
//
// ADC device specific control function.
//
//*****************************************************************************
uint32_t
am_hal_adc_control(void *pHandle,
                   am_hal_adc_request_e eRequest,
                   void *pArgs)
{
    uint32_t    ui32Module = ((am_hal_adc_state_t *)pHandle)->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_ADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    switch ( eRequest )
    {
        case AM_HAL_ADC_REQ_WINDOW_CONFIG:
        {
            am_hal_adc_window_config_t *pWindowConfig = (am_hal_adc_window_config_t *)pArgs;

#ifndef AM_HAL_DISABLE_API_VALIDATION
            //
            // Check the window limits.
            //
            if ( (pWindowConfig->ui32Upper > ADC_WULIM_ULIM_Msk)   ||
                 (pWindowConfig->ui32Lower > ADC_WLLIM_LLIM_Msk) )
            {
                return AM_HAL_STATUS_OUT_OF_RANGE;
            }
#endif // AM_HAL_DISABLE_API_VALIDATION
            //
            // Set the window comparison upper and lower limits.
            //
            ADCn(ui32Module)->WULIM = _VAL2FLD(ADC_WULIM_ULIM, pWindowConfig->ui32Upper);
            ADCn(ui32Module)->WLLIM = _VAL2FLD(ADC_WLLIM_LLIM, pWindowConfig->ui32Lower);

            //
            // Set the window scale per precision mode if indicated.
            //
            ADCn(ui32Module)->SCWLIM = _VAL2FLD(ADC_SCWLIM_SCWLIMEN,
                                                pWindowConfig->bScaleLimits);
        }
        break;

        case AM_HAL_ADC_REQ_TEMP_CELSIUS_GET:
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
                    // The caller has provided a voltage as determined from the
                    // sample. The sample is scaled up by the reference voltage
                    // used on the device and then scaled down by the number of
                    // sample bits.
                    // i.e. scaled by AM_HAL_ADC_VREF / (1 << number_bits).
                    //
                    // Get the scaled voltage obtained from the ADC sample.
                    //
                    fVoltage = pfArray[0];

                    //
                    // Compute the calibrated temperature via the equation:
                    //      T = m * Vmeas + T1 - m(V1 + Voff)
                    // m     = AM_HAL_ADC_TEMPSENSOR_SLOPE (degK / V)
                    // Vmeas = The measured voltage (as based on the ADC code)
                    // T1    = Calibration temperature
                    // V1    = Calibration voltage (stored in volts)
                    // Voff  = Calibration offset (stored in volts)
                    //
                    fCalibration_temp    = priv_temp_trims.flt.fCalibrationTemperature;
                    fCalibration_voltage = priv_temp_trims.flt.fCalibrationVoltage;
                    fCalibration_offset  = priv_temp_trims.flt.fCalibrationOffset;

                    //
                    // Compute the temperature in K
                    //
                    if ( g_fTempEqnTerms == 0.0F )
                    {
                        //
                        // The 2nd and 3rd terms of the temperature equation are
                        // consistent for a given device, so compute them only
                        // the first time and save.
                        //
                        g_fTempEqnTerms  = -1.0F * AM_HAL_ADC_TEMPSENSOR_SLOPE;
                        g_fTempEqnTerms *= (fCalibration_voltage + fCalibration_offset);
                        g_fTempEqnTerms += fCalibration_temp;
                    }

                    //
                    // Determine the temperature in K by factoring
                    // in the supplied sample voltage
                    //
                    fTemp  = AM_HAL_ADC_TEMPSENSOR_SLOPE * fVoltage;
                    fTemp += g_fTempEqnTerms;

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

        case AM_HAL_ADC_REQ_TEMP_TRIMS_GET:
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
                    // Return trim ADC offset voltage (in volts) as a float.
                    //
                    pfArray[2] = priv_temp_trims.flt.fCalibrationOffset;

                    //
                    // Set the calibrated or uncalibrated flag
                    //
                    *(uint32_t*)&pfArray[3] = priv_temp_trims.ui32.bMeasured;
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

        case AM_HAL_ADC_REQ_CORRECTION_TRIMS_GET:
            //
            // pArgs must point to an array of 4 floats.  To assure that the
            // array is valid, upon calling the 4th float (pArgs[3]) must be
            // set to the value -123.456.
            //
            // On return,
            //  pArgs[0] contains the offset that is applied to each sample.
            //  pArgs[1] contains the gain   that is applied to each sample.
            //
            //  Note that the am_hal_adc_samples_read() automatically applies
            //  the correction to samples. Therefore this function is primarily
            //  supplied to the user for informational purposes only.
            //
            if ( pArgs != NULL )
            {
                float *pfArray = (float*)pArgs;
                if ( pfArray[3] == -123.456F )
                {
                    //
                    // Return ADC correction offset as a float.
                    //
                    pfArray[0] = priv_correction_trims.flt.fADCoffset;

                    //
                    // Return ADC correction gain as a float.
                    //
                    pfArray[1] = priv_correction_trims.flt.fADCgain;

                    //
                    // Set the return values.
                    //
                    ((uint32_t*)pArgs)[2] = 0;
                    ((uint32_t*)pArgs)[3] = 0;
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

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_adc_control()

//*****************************************************************************
//
//! @brief ADC enable function
//!
//! @param pHandle   - handle for the module instance.
//!
//! This function enables the ADC operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_adc_enable(void *pHandle)
{
    am_hal_adc_state_t  *pADCState = (am_hal_adc_state_t *)pHandle;
    uint32_t            ui32Module = pADCState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_ADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( pADCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_SUCCESS;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Enable the ADC.
    //
    ADC_CRITICAL_BEGIN(ui32Module)
    ADCn(ui32Module)->CFG_b.ADCEN = ADC_CFG_ADCEN_EN;

    //
    // Set flag to indicate module is enabled.
    //
    pADCState->prefix.s.bEnable = true;
    ForceFIFOpop(pHandle);  // See errata ERR090
    ADC_CRITICAL_END(ui32Module)

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_adc_enable()

//*****************************************************************************
//
//! @brief ADC disable function
//!
//! @param pHandle   - handle for the module instance.
//!
//! This function disables the ADC operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_adc_disable(void *pHandle)
{
    am_hal_adc_state_t  *pADCState = (am_hal_adc_state_t *)pHandle;
    uint32_t            ui32Module = pADCState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_ADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Before disabling the ADC, clear RPTEN per the register description, "When
    // disabling the ADC (setting ADCEN to '0') the RPTEN bit should be cleared."
    //
    ADCn(ui32Module)->CFG_b.RPTEN = ADC_CFG_RPTEN_SINGLE_SCAN;

    //
    // Disable the ADC.
    //
    ADCn(ui32Module)->CFG_b.ADCEN = ADC_CFG_ADCEN_DIS;

    //
    // Set flag to indicate module is disabled.
    //
    pADCState->prefix.s.bEnable = false;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_adc_disable()

//*****************************************************************************
//
//! @brief ADC status function
//!
//! @param pHandle - handle for the interface.
//! @param pStatus - return status for the interface
//!
//! This function returns the current status of the DMA operation.
//!
//! @return status - DMA status flags.
//
//*****************************************************************************
uint32_t
am_hal_adc_status_get(void *pHandle, am_hal_adc_status_t *pStatus )
{
    uint32_t    ui32Module = ((am_hal_adc_state_t *)pHandle)->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_ADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Get the power status.
    //
    pStatus->bPoweredOn = (ADCn(ui32Module)->STAT & ADC_STAT_PWDSTAT_Msk) ==
                          _VAL2FLD(ADC_STAT_PWDSTAT, ADC_STAT_PWDSTAT_ON);

    //
    // Get the low power mode 1 status.
    //
    pStatus->bLPMode1 = (ADCn(ui32Module)->STAT & ADC_STAT_PWDSTAT_Msk) ==
                        _VAL2FLD(ADC_STAT_PWDSTAT, ADC_STAT_PWDSTAT_POWERED_DOWN);

    //
    //  Get the DMA status.
    //
    pStatus->bErr = ((ADCn(ui32Module)->DMASTAT & ADC_DMASTAT_DMAERR_Msk) > 0);
    pStatus->bCmp = ((ADCn(ui32Module)->DMASTAT & ADC_DMASTAT_DMACPL_Msk) > 0);
    pStatus->bTIP = ((ADCn(ui32Module)->DMASTAT & ADC_DMASTAT_DMATIP_Msk) > 0);

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_adc_status_get()

//*****************************************************************************
//
//! @brief ADC enable interrupts function
//!
//! @param pHandle       - handle for the interface.
//! @param ui32IntMask  - ADC interrupt mask.
//!
//! This function enables the specific indicated interrupts.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_adc_interrupt_enable(void *pHandle, uint32_t ui32IntMask)
{
    uint32_t    ui32Module = ((am_hal_adc_state_t*)pHandle)->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_ADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Enable the interrupts.
    //
    ADCn(ui32Module)->INTEN |= ui32IntMask;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_adc_interrupt_enable()

//*****************************************************************************
//
//! @brief ADC disable interrupts function
//!
//! @param pHandle       - handle for the interface.
//! @param ui32IntMask  - ADC interrupt mask.
//!
//! This function disable the specific indicated interrupts.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_adc_interrupt_disable(void *pHandle, uint32_t ui32IntMask)
{
    uint32_t    ui32Module = ((am_hal_adc_state_t*)pHandle)->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_ADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Disable the interrupts.
    //
    ADCn(ui32Module)->INTEN &= ~ui32IntMask;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_adc_interrupt_disable()

//*****************************************************************************
//
//! @brief ADC interrupt status function
//!
//! @param pHandle      - handle for the interface.
//! @param pui32Status  - pointer to status
//! @param bEnabledOnly - if ADC enabled
//!
//! This function returns the specific indicated interrupt status.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_adc_interrupt_status(void *pHandle,
                            uint32_t  *pui32Status,
                            bool bEnabledOnly)
{
    uint32_t    ui32Module = ((am_hal_adc_state_t*)pHandle)->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_ADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // if requested, only return the interrupts that are enabled.
    //
    if ( bEnabledOnly )
    {
        uint32_t ui32RetVal = ADCn(ui32Module)->INTSTAT;
        *pui32Status = ADCn(ui32Module)->INTEN & ui32RetVal;
    }
    else
    {
        *pui32Status = ADCn(ui32Module)->INTSTAT;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_adc_interrupt_status()

//*****************************************************************************
//
//! @brief ADC interrupt clear
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
am_hal_adc_interrupt_clear(void *pHandle, uint32_t ui32IntMask)
{
    uint32_t    ui32Module = ((am_hal_adc_state_t*)pHandle)->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_ADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Clear the interrupts.
    //
    ADCn(ui32Module)->INTCLR = ui32IntMask;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_adc_interrupt_clear()

//*****************************************************************************
//
// sample_correction_apply()
//
//*****************************************************************************
static uint32_t
sample_correction_apply(uint32_t ui32Sample, bool bApplyCorrection)
{
    float fSampleAdj;

    if ( g_bDoADCadjust )
    {
        //
        // Apply the gain/offset correction to the full sample for all
        // channels except temperature.
        //
        if ( bApplyCorrection )
        {
            //
            // General correction equation:
            //  sample_corrected = (sample / (1.0F - gain)) - offset
            //
            fSampleAdj = (float)(AM_HAL_ADC_FIFO_SAMPLE(ui32Sample) * AM_HAL_ADC_VREFMV / AM_HAL_ADC_SAMPLE_DIVISOR);
            fSampleAdj /= (1.0F - priv_correction_trims.flt.fADCgain);
            
            //
            // Convert the offset from volts to mv.
            //
            fSampleAdj -= (priv_correction_trims.flt.fADCoffset * 1000.0F);
            fSampleAdj  = fSampleAdj * AM_HAL_ADC_SAMPLE_DIVISORF / AM_HAL_ADC_VREFMVF;
            
            //
            // Check for overflow
            //
            if ( fSampleAdj > 4095.0F )
            {
                fSampleAdj = 4095.0F;
            }
            
            ui32Sample &= 0xFFF00000;
            ui32Sample |= ((((uint32_t)fSampleAdj) << 6) & AM_HAL_ADC_SAMPLE_MASK_FULL);
        }
    }
    return ui32Sample;
} // sample_correction_apply()

//*****************************************************************************
//
// ADC sample read function
//
// This function reads samples from the ADC FIFO or an SRAM sample buffer
// returned by a DMA operation.
//
//*****************************************************************************
uint32_t
am_hal_adc_samples_read(void *pHandle,
                        bool bFullSample,
                        uint32_t *pui32InSampleBuffer,
                        uint32_t *pui32InOutNumberSamples,
                        am_hal_adc_sample_t *pui32OutBuffer)
{
    uint32_t      ui32Sample;
    uint32_t      ui32RequestedSamples = *pui32InOutNumberSamples;
    uint32_t ui32Module = ((am_hal_adc_state_t*)pHandle)->ui32Module;
    uint32_t ui32slot, ui32slotcfgaddr, ui32chsel, ui32TempMask;
    bool bTempChnl;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_ADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    //
    // Check the output sample buffer pointer.
    //
    if ( NULL == pui32OutBuffer )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    *pui32InOutNumberSamples = 0;

    //
    // Check if we are reading directly from FIFO or DMA SRAM buffer.
    //
    if ( pui32InSampleBuffer == NULL )
    {
        //
        // Grab a value from the ADC FIFO
        //
        do
        {
            ui32Sample = ADCn(ui32Module)->FIFOPR;

            ui32slot = AM_HAL_ADC_FIFO_SLOT(ui32Sample);
            ui32slotcfgaddr = ((uint32_t)(&ADCn(ui32Module)->SL0CFG)) + (4 * ui32slot);
            ui32chsel = _FLD2VAL(ADC_SL0CFG_CHSEL0, AM_REGVAL(ui32slotcfgaddr));
            bTempChnl = (ui32chsel == ADC_SL0CFG_CHSEL0_TEMP) ? true : false;

            //
            // Apply the sample correction.
            //
            ui32Sample = sample_correction_apply(ui32Sample, !bTempChnl);
            pui32OutBuffer->ui32Slot   = AM_HAL_ADC_FIFO_SLOT(ui32Sample);
            pui32OutBuffer->ui32Sample = bFullSample                             ?
                                         AM_HAL_ADC_FIFO_FULL_SAMPLE(ui32Sample) :
                                         AM_HAL_ADC_FIFO_SAMPLE(ui32Sample);
            pui32OutBuffer++;
            (*pui32InOutNumberSamples)++;
        } while ((AM_HAL_ADC_FIFO_COUNT(ui32Sample) > 0) &&
                 (*pui32InOutNumberSamples < ui32RequestedSamples));
    }
    else
    {
        //
        // Before processing the buffer of samples, pre-process the slot
        // configuration registers to build a mask of slots that might be
        // configured as temperature. The mask will be used later for
        // determining whether to apply correction.
        // This, of course, depends on and assumes that the slots are still
        // configured as they were while the samples were taken.
        //
        ui32TempMask = 0;
        ui32TempMask |= (_FLD2VAL(ADC_SL0CFG_CHSEL0, ADC->SL0CFG) == ADC_SL0CFG_CHSEL0_TEMP) ? 1 << 0 : 0;
        ui32TempMask |= (_FLD2VAL(ADC_SL1CFG_CHSEL1, ADC->SL1CFG) == ADC_SL1CFG_CHSEL1_TEMP) ? 1 << 1 : 0;
        ui32TempMask |= (_FLD2VAL(ADC_SL2CFG_CHSEL2, ADC->SL2CFG) == ADC_SL2CFG_CHSEL2_TEMP) ? 1 << 2 : 0;
        ui32TempMask |= (_FLD2VAL(ADC_SL3CFG_CHSEL3, ADC->SL3CFG) == ADC_SL3CFG_CHSEL3_TEMP) ? 1 << 3 : 0;
        ui32TempMask |= (_FLD2VAL(ADC_SL4CFG_CHSEL4, ADC->SL4CFG) == ADC_SL4CFG_CHSEL4_TEMP) ? 1 << 4 : 0;
        ui32TempMask |= (_FLD2VAL(ADC_SL5CFG_CHSEL5, ADC->SL5CFG) == ADC_SL5CFG_CHSEL5_TEMP) ? 1 << 5 : 0;
        ui32TempMask |= (_FLD2VAL(ADC_SL6CFG_CHSEL6, ADC->SL6CFG) == ADC_SL6CFG_CHSEL6_TEMP) ? 1 << 6 : 0;
        ui32TempMask |= (_FLD2VAL(ADC_SL7CFG_CHSEL7, ADC->SL7CFG) == ADC_SL7CFG_CHSEL7_TEMP) ? 1 << 7 : 0;

        //
        // Process the samples from the provided sample buffer
        //
        do
        {
            //
            // Apply the sample correction.
            // Note that the correction is not used for temperature.
            //
            ui32slot   = AM_HAL_ADC_FIFO_SLOT(*pui32InSampleBuffer);
            ui32Sample = AM_HAL_ADC_FIFO_FULL_SAMPLE(*pui32InSampleBuffer);
            bTempChnl  = ((ui32TempMask >> ui32slot) & 1) ? true : false;
            pui32OutBuffer->ui32Sample = AM_HAL_ADC_FIFO_SAMPLE(sample_correction_apply(ui32Sample,
                                                                !bTempChnl));
            pui32OutBuffer->ui32Slot   = ui32slot;
            pui32InSampleBuffer++;
            pui32OutBuffer++;
            (*pui32InOutNumberSamples)++;
        } while (*pui32InOutNumberSamples < ui32RequestedSamples);
    }

    //
    // Return FIFO valid bits.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_adc_samples_read()

//*****************************************************************************
//
//! @brief Issue Software Trigger to the ADC.
//!
//! @param pHandle   - handle for the module instance.
//!
//! This function triggers the ADC operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_adc_sw_trigger(void *pHandle)
{
    uint32_t    ui32Module = ((am_hal_adc_state_t*)pHandle)->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_ADC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Write to the Software trigger register in the ADC.
    //
    ADCn(ui32Module)->SWT = 0x37;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_adc_sw_trigger()

//*****************************************************************************
//
//! @brief ADC power control function
//!
//! @param pHandle       - handle for the interface.
//! @param ePowerState  - the desired power state to move the peripheral to.
//! @param bRetainState - flag (if true) to save/restore peripheral state upon
//!                       power state change.
//!
//! This function updates the peripheral to a given power state.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_adc_power_control(void *pHandle,
                         am_hal_sysctrl_power_state_e ePowerState,
                         bool bRetainState)
{
    am_hal_adc_state_t  *pADCState = (am_hal_adc_state_t *)pHandle;
    uint32_t            ui32Module = pADCState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_ADC_CHK_HANDLE(pHandle) )
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
            if ( bRetainState  &&  !pADCState->registerState.bValid )
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }

            //
            // Enable the ADC power domain.
            //
            am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_ADC);

            if ( bRetainState )
            {
                ADCn(ui32Module)->SL0CFG        = pADCState->registerState.regSL0CFG;
                ADCn(ui32Module)->SL1CFG        = pADCState->registerState.regSL1CFG;
                ADCn(ui32Module)->SL2CFG        = pADCState->registerState.regSL2CFG;
                ADCn(ui32Module)->SL3CFG        = pADCState->registerState.regSL3CFG;
                ADCn(ui32Module)->SL4CFG        = pADCState->registerState.regSL4CFG;
                ADCn(ui32Module)->SL5CFG        = pADCState->registerState.regSL5CFG;
                ADCn(ui32Module)->SL6CFG        = pADCState->registerState.regSL6CFG;
                ADCn(ui32Module)->SL7CFG        = pADCState->registerState.regSL7CFG;
                ADCn(ui32Module)->INTTRIGTIMER  = pADCState->registerState.regIntTrigTmr;
                ADCn(ui32Module)->WULIM         = pADCState->registerState.regWULIM;
                ADCn(ui32Module)->WLLIM         = pADCState->registerState.regWLLIM;

                ADCn(ui32Module)->INTEN         = 0x0;
                ADC_CRITICAL_BEGIN(ui32Module)
                ADCn(ui32Module)->CFG           = pADCState->registerState.regCFG;
                ForceFIFOpop(pHandle);  // See errata ERR090
                ADC_CRITICAL_END(ui32Module)
                ADCn(ui32Module)->INTEN         = pADCState->registerState.regINTEN;
                pADCState->registerState.bValid = false;
            }

            break;

        case AM_HAL_SYSCTRL_NORMALSLEEP:
        case AM_HAL_SYSCTRL_DEEPSLEEP:
            if ( bRetainState )
            {
                pADCState->registerState.regSL0CFG      = ADCn(ui32Module)->SL0CFG;
                pADCState->registerState.regSL1CFG      = ADCn(ui32Module)->SL1CFG;
                pADCState->registerState.regSL2CFG      = ADCn(ui32Module)->SL2CFG;
                pADCState->registerState.regSL3CFG      = ADCn(ui32Module)->SL3CFG;
                pADCState->registerState.regSL4CFG      = ADCn(ui32Module)->SL4CFG;
                pADCState->registerState.regSL5CFG      = ADCn(ui32Module)->SL5CFG;
                pADCState->registerState.regSL6CFG      = ADCn(ui32Module)->SL6CFG;
                pADCState->registerState.regSL7CFG      = ADCn(ui32Module)->SL7CFG;
                pADCState->registerState.regIntTrigTmr  = ADCn(ui32Module)->INTTRIGTIMER;
                pADCState->registerState.regWULIM       = ADCn(ui32Module)->WULIM;
                pADCState->registerState.regWLLIM       = ADCn(ui32Module)->WLLIM;
                pADCState->registerState.regINTEN       = ADCn(ui32Module)->INTEN;
                pADCState->registerState.regCFG         = ADCn(ui32Module)->CFG;

                pADCState->registerState.bValid     = true;
            }

            //
            // Disable the ADC power domain.
            //
            am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_ADC);
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_adc_power_control()

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
