//*****************************************************************************
//
//! @file am_hal_adc.h
//!
//! @brief Functions for interfacing with the Analog to Digital Converter
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
#ifndef AM_HAL_ADC_H
#define AM_HAL_ADC_H

//*****************************************************************************
//
//! @brief CMSIS-style macro for handling a variable IOM module number.
//
//*****************************************************************************
#define ADCn(n) ((ADC_Type*)(ADC_BASE + (n * (ADC_BASE - ADC_BASE))))

// ****************************************************************************
//
//! @name Default VREF
//! @{
//!
//! VREF is needed by applications in order to convert samples to voltages,
//! as well as to do the sample corrections in the HAL functions.
//!
//! Important: It is important to use 1.19v, as opposed to 1.20v, as the
//! reference voltage.
//! Ambiq recommends using AM_HAL_ADC_VREF to designate Vref in applications.
//
// ****************************************************************************
#define AM_HAL_ADC_VREF         1.19F
#define AM_HAL_ADC_VREFMV       1190        // Vref specified in millivolts
#define AM_HAL_ADC_VREFMVF      (AM_HAL_ADC_VREF * 1000.0F)
//! @}

//*****************************************************************************
//
//! @name ADC Samples
//! @{
//! ADC Sample Macros
//!
//! These macros may be used to set the ADc Sample bit, divisor and mask
//
//*****************************************************************************
#define AM_HAL_ADC_SAMPLE_BITS          (12)                                        // Number of bits in the sample
#define AM_HAL_ADC_SAMPLE_2N            (1 << AM_HAL_ADC_SAMPLE_BITS)               // Exponentiation of 2^^n
#define AM_HAL_ADC_SAMPLE_DIVISOR       AM_HAL_ADC_SAMPLE_2N                        // Divisor for 12 bits
#define AM_HAL_ADC_SAMPLE_DIVISORF      ((float)AM_HAL_ADC_SAMPLE_2N)               // Divisor for 12 bits
#define AM_HAL_ADC_SAMPLE_MASK          ((AM_HAL_ADC_SAMPLE_2N - 1) << 6)           // Mask for integer portion of the sample
#define AM_HAL_ADC_SAMPLE_MASK_FULL     (((1 << (AM_HAL_ADC_SAMPLE_BITS + 6)) - 1)) // Mask for the full sample
//! @}

// ****************************************************************************
//
//! @brief Maximum number of slots.
//
// ****************************************************************************
#define AM_HAL_ADC_MAX_SLOTS            8

// ****************************************************************************
//
//! @brief Minimum value for TRKCYC for the general purpose ADC.
// Errata ERR091: TRKCYC must be minimum of 32
//
// ****************************************************************************
#define AM_HAL_ADC_MIN_TRKCYC           32

// ****************************************************************************
//
//! @brief Default slope (in degK / V) of the Apollo4 temperature sensor.
//
// ****************************************************************************
#define AM_HAL_ADC_TEMPSENSOR_SLOPE     290.0F

// ****************************************************************************
//
//! @enum am_hal_adc_clksel_e
//! @brief ADC clock selection.
//
// ****************************************************************************
typedef enum
{
    //AM_HAL_ADC_CLKSEL_HFRC        = 0,
    AM_HAL_ADC_CLKSEL_HFRC_48MHZ  = ADC_CFG_CLKSEL_HFRC_48MHZ,
    AM_HAL_ADC_CLKSEL_HFRC_48MHZ1 = ADC_CFG_CLKSEL_HFRC_48MHZ1,
    AM_HAL_ADC_CLKSEL_HFRC_24MHZ  = ADC_CFG_CLKSEL_HFRC_24MHZ,
    AM_HAL_ADC_CLKSEL_HFRC2_48MHZ = ADC_CFG_CLKSEL_HFRC2_48MHZ
} am_hal_adc_clksel_e;

// ****************************************************************************
//
//! @enum am_hal_adc_rpttrigsel_e
//! @brief ADC periodic trigger source selection
//
// ****************************************************************************
typedef enum
{
    AM_HAL_ADC_RPTTRIGSEL_TMR,
    AM_HAL_ADC_RPTTRIGSEL_INT
} am_hal_adc_rpttrigsel_e;

// ****************************************************************************
//
//! @enum am_hal_adc_trigpol_e
//! @brief ADC trigger polarity
//
// ****************************************************************************
typedef enum
{
    AM_HAL_ADC_TRIGPOL_RISING,
    AM_HAL_ADC_TRIGPOL_FALLING
} am_hal_adc_trigpol_e;

// ****************************************************************************
//
//! @enum am_hal_adc_trigsel_e
//! @brief ADC trigger selection
//
// ****************************************************************************
typedef enum
{
    AM_HAL_ADC_TRIGSEL_EXT0,
    AM_HAL_ADC_TRIGSEL_EXT1,
    AM_HAL_ADC_TRIGSEL_EXT2,
    AM_HAL_ADC_TRIGSEL_EXT3,
    AM_HAL_ADC_TRIGSEL_VCOMP,
    AM_HAL_ADC_TRIGSEL_SOFTWARE = 7
} am_hal_adc_trigsel_e;

//
// ADC reference selection.
//
//typedef enum
//{
//    AM_HAL_ADC_REFSEL_INT_2P0,
//    AM_HAL_ADC_REFSEL_INT_1P5,
//    AM_HAL_ADC_REFSEL_EXT_2P0,
//    AM_HAL_ADC_REFSEL_EXT_1P5
//} am_hal_adc_refsel_e;

// ****************************************************************************
//
//! @enum am_hal_adc_clkmode_e
//! @brief ADC clock mode selection.
//
// ****************************************************************************
typedef enum
{
    AM_HAL_ADC_CLKMODE_LOW_POWER,   // Disable the clock between scans for LPMODE0.
                                    // Set LPCKMODE to 0x1 while configuring the ADC.
    AM_HAL_ADC_CLKMODE_LOW_LATENCY  // Low Latency Clock Mode. When set, HFRC and the
                                    // adc_clk will remain on while in functioning in LPMODE0.
} am_hal_adc_clkmode_e;

// ****************************************************************************
//
//! @enum am_hal_adc_lpmode_e
//! @brief ADC low-power mode selection.
//
// ****************************************************************************
typedef enum
{
    AM_HAL_ADC_LPMODE0,  // Low Latency Clock Mode. When set, HFRC and the adc_clk
                         // will remain on while in functioning in LPMODE0.
    AM_HAL_ADC_LPMODE1   // Powers down all circuity and clocks associated with the
                         // ADC until the next trigger event. Between scans, the reference
                         // buffer requires up to 50us of delay from a scan trigger event
                         // before the conversion will commence while operating in this mode.
} am_hal_adc_lpmode_e;

// ****************************************************************************
//
//! @enum am_hal_adc_repeat_e
//! @brief ADC repetition selection.
//
// ****************************************************************************
typedef enum
{
    AM_HAL_ADC_SINGLE_SCAN,
    AM_HAL_ADC_REPEATING_SCAN
} am_hal_adc_repeat_e;

// ****************************************************************************
//
//! @enum am_hal_adc_meas_avg_e
//! @brief ADC measurement averaging configuration.
//
// ****************************************************************************
typedef enum
{
    AM_HAL_ADC_SLOT_AVG_1,
    AM_HAL_ADC_SLOT_AVG_2,
    AM_HAL_ADC_SLOT_AVG_4,
    AM_HAL_ADC_SLOT_AVG_8,
    AM_HAL_ADC_SLOT_AVG_16,
    AM_HAL_ADC_SLOT_AVG_32,
    AM_HAL_ADC_SLOT_AVG_64,
    AM_HAL_ADC_SLOT_AVG_128
} am_hal_adc_meas_avg_e;

// ****************************************************************************
//
//! @enum am_hal_adc_slot_prec_e
//! @brief ADC slot precision mode.
//
// ****************************************************************************
typedef enum
{
    AM_HAL_ADC_SLOT_12BIT,
    AM_HAL_ADC_SLOT_12BIT_1,
    AM_HAL_ADC_SLOT_10BIT,
    AM_HAL_ADC_SLOT_8BIT
} am_hal_adc_slot_prec_e;

// ****************************************************************************
//
//! @enum am_hal_adc_slot_chan_e
//! @brief ADC slot channel selection.
//
// ****************************************************************************
typedef enum
{
    // Single-ended channels
    AM_HAL_ADC_SLOT_CHSEL_SE0,
    AM_HAL_ADC_SLOT_CHSEL_SE1,
    AM_HAL_ADC_SLOT_CHSEL_SE2,
    AM_HAL_ADC_SLOT_CHSEL_SE3,
    AM_HAL_ADC_SLOT_CHSEL_SE4,
    AM_HAL_ADC_SLOT_CHSEL_SE5,
    AM_HAL_ADC_SLOT_CHSEL_SE6,
    AM_HAL_ADC_SLOT_CHSEL_SE7,
    // Miscellaneous other signals.
    AM_HAL_ADC_SLOT_CHSEL_TEMP,
    AM_HAL_ADC_SLOT_CHSEL_BATT,
    AM_HAL_ADC_SLOT_TEST_MUX,
    AM_HAL_ADC_SLOT_CHSEL_VSS
} am_hal_adc_slot_chan_e;

// ****************************************************************************
//
//! @enum am_hal_adc_scale_wincomp_e
//! @brief cale window comparator limits
//
// ****************************************************************************
typedef enum
{
    AM_HAL_ADC_SCALE_WINCOMP_DIS,
    AM_HAL_ADC_SCALE_WINCOMP_EN
} am_hal_adc_scale_wincomp_e;

// ****************************************************************************
//
//! @enum am_hal_adc_irtt_clkdiv_e
//! @brief Internal repeating trigger (irtt) timer clock division
//
// ****************************************************************************
typedef enum
{
    AM_HAL_ADC_RPTT_CLK_DIV1,
    AM_HAL_ADC_RPTT_CLK_DIV2,
    AM_HAL_ADC_RPTT_CLK_DIV4,
    AM_HAL_ADC_RPTT_CLK_DIV16 = 4
} am_hal_adc_irtt_clkdiv_e;

// ****************************************************************************
//
//! @enum am_hal_adc_dma_prior_e
//! @brief DMA priority.
//
// ****************************************************************************
typedef enum
{
    AM_HAL_ADC_PRIOR_BEST_EFFORT,
    AM_HAL_ADC_PRIOR_SERVICE_IMMED
} am_hal_adc_dma_prior_e;

// ****************************************************************************
//!
//! @enum am_hal_adc_request_e
//! @brief ADC control function request types for am_hal_adc_control().
//!
//! AM_HAL_ADC_REQ_TEMP_CELSIUS_GET:
//!     pArgs must point to an array of 3 floats.  To assure that the
//!     array is valid, upon calling the 3rd float (pArgs[2]) must be
//!     set to the value -123.456F.
//! AM_HAL_ADC_REQ_TEMP_TRIMS_GET:
//!     pArgs must point to an array of 4 floats.  To assure that the
//!     array is valid, upon calling the 4th float (pArgs[3]) must be
//!     set to the to the value -123.456F.
//!     On return, pArgs[3] is set to 1 if the returned values are
//!     calibrated, or 0 if default calibration values.
//! AM_HAL_ADC_REQ_CORRECTION_TRIMS_GET:
//!     Returns the offset and gain correction values used by the HAL.
//!     On entry pArgs must point to an array of 4 floats. To assure that
//!     the array is valid, upon calling the 4th float (pArgs[3]) must be
//!     set to the to the value -123.456F.
//!     On return,
//!     pArgs[0] contains the offset that is applied to each sample.
//!     pArgs[1] contains the gain   that is applied to each sample.
//!
//!     Note that am_hal_adc_samples_read() automatically applies the sample
//!     correction to samples. Therefore this function is primarily provided
//!     to the user for informational purposes only.
//!
// ****************************************************************************
typedef enum
{
    AM_HAL_ADC_REQ_WINDOW_CONFIG,
    AM_HAL_ADC_REQ_TEMP_CELSIUS_GET,
    AM_HAL_ADC_REQ_TEMP_TRIMS_GET,
    AM_HAL_ADC_REQ_CORRECTION_TRIMS_GET,
} am_hal_adc_request_e;

// ****************************************************************************
//
//! @struct am_hal_adc_sample_t
//! @brief ADC Sample structure.
//
// ****************************************************************************
typedef struct
{
  uint32_t      ui32Sample;
  uint32_t      ui32Slot;
} am_hal_adc_sample_t;

//*****************************************************************************
//
//! @struct am_hal_adc_config_t
//! @brief Configuration structure for the ADC.
//!
//!     eClock = One of the following:
//!         AM_HAL_ADC_CLKSEL_HFRC
//!         AM_HAL_ADC_CLKSEL_HFRC_48MHZ
//!         AM_HAL_ADC_CLKSEL_HFRC_48MHZ1
//!         AM_HAL_ADC_CLKSEL_HFRC2_24MHZ
//!         AM_HAL_ADC_CLKSEL_HFRC2_48MHZ
//!     Notes:
//!     - AM_HAL_ADC_CLKSEL_HFRC, AM_HAL_ADC_CLKSEL_HFRC_48MHZ, and
//!       AM_HAL_ADC_CLKSEL_HFRC_48MHZ1 all result in effectively
//!       the same 48MHz clock source.
//!     - See the Programmer's Guide for any implications of using
//!       HFRC2 clock sources.
//
//*****************************************************************************
typedef struct
{
    //! Select the ADC clock source.
    am_hal_adc_clksel_e           eClock;

    //! select the periodic trigger source
    am_hal_adc_rpttrigsel_e       eRepeatTrigger;

    //! Select the ADC trigger polarity.
    am_hal_adc_trigpol_e          ePolarity;

    //! Select the ADC trigger source.
    am_hal_adc_trigsel_e          eTrigger;

    //! Whether to disable clocks between samples.
    am_hal_adc_clkmode_e          eClockMode;

    //! Select the ADC power mode.
    am_hal_adc_lpmode_e           ePowerMode;

    //! Select whether the ADC will re-trigger based on a signal from timer.
    am_hal_adc_repeat_e           eRepeat;

} am_hal_adc_config_t;

//*****************************************************************************
//
//! @struct am_hal_adc_slot_config_t
//! @brief Configuration structure for the ADC slot.
//
//*****************************************************************************
typedef struct
{
    //! Select the number of measurements to average
    am_hal_adc_meas_avg_e         eMeasToAvg;

    //! Set additional input sampling ADC clock cycles
    uint32_t                      ui32TrkCyc;

    //! Select the precision mode
    am_hal_adc_slot_prec_e        ePrecisionMode;

    //! Select the channel
    am_hal_adc_slot_chan_e        eChannel;

    //! Select window comparison mode
    bool                          bWindowCompare;

    //! Enable the slot
    bool                          bEnabled;

} am_hal_adc_slot_config_t;

//*****************************************************************************
//
//! @struct am_hal_adc_irtt_config_t
//! @brief Configuration structure for the ADC internal repeat trigger timer.
//
//*****************************************************************************
typedef struct
{
    //! ADC-internal repeat trigger timer enable
    bool                          bIrttEnable;
    am_hal_adc_irtt_clkdiv_e      eClkDiv;
    uint32_t                      ui32IrttCountMax;
} am_hal_adc_irtt_config_t;

//*****************************************************************************
//
//! @struct am_hal_adc_dma_config_t
//! @brief Configuration structure for the ADC DMA
//
//*****************************************************************************
typedef struct
{
    //! ADC DMA dynamic priority enabled.
    bool                          bDynamicPriority;

    //! ADC DMA static priority.
    am_hal_adc_dma_prior_e        ePriority;

    //! Enable DMA for ADC
    bool                          bDMAEnable;

    //! Transfer count in samples
    uint32_t                      ui32SampleCount;

    //! Target address
    uint32_t                      ui32TargetAddress;

} am_hal_adc_dma_config_t;

//*****************************************************************************
//
//! @struct am_hal_adc_window_config_t
//! @brief Window configuration structure for the ADC.
//
//*****************************************************************************
typedef struct
{
    //! Scale window comparison
    bool                          bScaleLimits;

    //! Window limits
    uint32_t                      ui32Upper;
    uint32_t                      ui32Lower;

} am_hal_adc_window_config_t;

//*****************************************************************************
//
//! @struct am_hal_adc_capabilities_t
//! @brief Capabilities structure for the ADC.
//
//*****************************************************************************
typedef struct
{
    uint32_t      dummy;

} am_hal_adc_capabilities_t;

//*****************************************************************************
//
//! @struct am_hal_adc_status_t
//! @brief Status structure for the ADC.
//
//*****************************************************************************
typedef struct
{
    //
    // ADC power status.
    //
    bool                          bPoweredOn;
    bool                          bLPMode1;

    //
    // DMA status.
    //
    bool                          bErr;
    bool                          bCmp;
    bool                          bTIP;

} am_hal_adc_status_t;

// ****************************************************************************
//
//! Transfer callback function prototype
//
// ****************************************************************************
typedef void (*am_hal_adc_callback_t)(void *pCallbackCtxt, uint32_t status);

//*****************************************************************************
//
//! @ingroup adc4
//! @name ADC Interrupts
//! @{
//! Interrupt Status Bits for enable/disble use
//!
//! These macros may be used to enable an individual ADC interrupt cause.
//
//*****************************************************************************
#define AM_HAL_ADC_INT_DERR               (_VAL2FLD(ADC_INTEN_DERR, 1))
#define AM_HAL_ADC_INT_DCMP               (_VAL2FLD(ADC_INTEN_DCMP, 1))
#define AM_HAL_ADC_INT_WCINC              (_VAL2FLD(ADC_INTEN_WCINC, 1))
#define AM_HAL_ADC_INT_WCEXC              (_VAL2FLD(ADC_INTEN_WCEXC, 1))
#define AM_HAL_ADC_INT_FIFOOVR2           (_VAL2FLD(ADC_INTEN_FIFOOVR2, 1))
#define AM_HAL_ADC_INT_FIFOOVR1           (_VAL2FLD(ADC_INTEN_FIFOOVR1, 1))
#define AM_HAL_ADC_INT_SCNCMP             (_VAL2FLD(ADC_INTEN_SCNCMP, 1))
#define AM_HAL_ADC_INT_CNVCMP             (_VAL2FLD(ADC_INTEN_CNVCMP, 1))
//! @}

//*****************************************************************************
//
//! @ingroup adc4
//! @name ADC Fifo Read macros
//! @{
//!
//! These are helper macros for interpreting FIFO data. Each ADC FIFO entry
//! contains information about the slot number and the FIFO depth alongside the
//! current sample. These macros perform the correct masking and shifting to
//! read those values.
//!
//! The SAMPLE and FULL_SAMPLE options refer to the fractional part of averaged
//! samples. If you are not using hardware averaging or don't need the
//! fractional part of the ADC sample, you should just use
//! AM_HAL_ADC_FIFO_SAMPLE.
//!
//! If you do need the fractional part, use AM_HAL_ADC_FIFO_FULL_SAMPLE. This
//! macro will keep six bits of precision past the decimal point. Depending on
//! the number of averaged samples, anywhere between 1 and 6 of these bits will
//! be valid. Please consult the datasheet to find out how many bits of data
//! are valid for your chosen averaging settings.
//!
//
//*****************************************************************************
#define AM_HAL_ADC_FIFO_SAMPLE(value)       (_FLD2VAL(ADC_FIFO_DATA, value) >> 6)
#define AM_HAL_ADC_FIFO_FULL_SAMPLE(value)  (_FLD2VAL(ADC_FIFO_DATA, value))
#define AM_HAL_ADC_FIFO_SLOT(value)         (_FLD2VAL(ADC_FIFO_SLOTNUM, value))
#define AM_HAL_ADC_FIFO_COUNT(value)        (_FLD2VAL(ADC_FIFO_COUNT, value))
//! @}

#ifdef __cplusplus
extern "C"
{
#endif

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
extern uint32_t am_hal_adc_initialize(uint32_t ui32Module, void **ppHandle);

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
extern uint32_t am_hal_adc_deinitialize(void *pHandle);

//*****************************************************************************
//
//! @brief ADC configuration function
//!
//! @param pHandle - handle for the module instance.
//! @param psConfig - pointer to the configuration structure.
//!
//! This function configures the ADC for operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_adc_configure(void *pHandle,
                                     am_hal_adc_config_t *psConfig);

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
extern uint32_t am_hal_adc_configure_slot(void *pHandle,
                                          uint32_t ui32SlotNumber,
                                          am_hal_adc_slot_config_t *pSlotConfig);

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
extern uint32_t am_hal_adc_configure_irtt(void *pHandle,
                                          am_hal_adc_irtt_config_t *pConfig);

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
extern uint32_t am_hal_adc_irtt_enable(void *pHandle);

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
extern uint32_t am_hal_adc_irtt_disable(void *pHandle);

//*****************************************************************************
//
//! @brief ADC DMA configuration function
//!
//! @param pHandle - handle for the module instance.
//! @param pDMAConfig    - pointer to the configuration structure.
//!
//! This function configures the ADC DMA for operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_adc_configure_dma(void *pHandle,
                                         am_hal_adc_dma_config_t *pDMAConfig);

//*****************************************************************************
//
//! @brief ADC device specific control function.
//!
//! @param pHandle   - handle for the module instance.
//! @param eRequest - One of:
//!      AM_HAL_ADC_REQ_WINDOW_CONFIG
//!   @n AM_HAL_ADC_REQ_TEMP_CELSIUS_GET (pArgs is required, see enums).
//!   @n AM_HAL_ADC_REQ_TEMP_TRIMS_GET   (pArgs is required, see enums).
//! @param pArgs - Pointer to arguments for Control Switch Case
//!
//! This function provides for special control functions for the ADC operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_adc_control(void *pHandle,
                                   am_hal_adc_request_e eRequest,
                                   void *pArgs);

//*****************************************************************************
//
//! @brief ADC enable function
//!
//! @param pHandle - handle for the module instance.
//!
//! This function enables the ADC operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_adc_enable(void *pHandle);

//*****************************************************************************
//
//! @brief ADC disable function
//!
//! @param pHandle - handle for the module instance.
//!
//! This function disables the ADC operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_adc_disable(void *pHandle);

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
extern uint32_t am_hal_adc_status_get(void *pHandle,
                                      am_hal_adc_status_t *pStatus );

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
extern uint32_t am_hal_adc_interrupt_enable(void *pHandle, uint32_t ui32IntMask);

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
extern uint32_t am_hal_adc_interrupt_disable(void *pHandle, uint32_t ui32IntMask);

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
extern uint32_t am_hal_adc_interrupt_status(void *pHandle,
                                            uint32_t  *pui32Status,
                                            bool bEnabledOnly);

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
extern uint32_t am_hal_adc_interrupt_clear(void *pHandle, uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief ADC sample read function
//!
//! @param pHandle              - handle for the module instance.
//! @param bFullSample          - true to get a full sample including
//!                               the fractional part.
//! @param pui32InSampleBuffer  - Ptr to the input sample buffer.
//!                               If NULL then samples will be read directly
//!                               from the FIFO.
//! @param pui32InOutNumberSamples - Ptr to variable containing the number of
//!                                  samples.
//! @param pui32OutBuffer - Ptr to the required output sample buffer.
//!
//! This function reads samples from the ADC FIFO or an SRAM sample buffer
//! returned by a DMA operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_adc_samples_read(void *pHandle, bool bFullSample,
                                        uint32_t *pui32InSampleBuffer,
                                        uint32_t *pui32InOutNumberSamples,
                                        am_hal_adc_sample_t *pui32OutBuffer);

//*****************************************************************************
//
//! @brief ADC software trigger function
//!
//! @param pHandle - handle for the module instance.
//!
//! This function triggers the ADC operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_adc_sw_trigger(void *pHandle);

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
extern uint32_t am_hal_adc_power_control(void *pHandle,
                                         am_hal_sysctrl_power_state_e ePowerState,
                                         bool bRetainState);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_ADC_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

