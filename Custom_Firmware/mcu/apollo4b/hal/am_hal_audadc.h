//*****************************************************************************
//
//! @file am_hal_audadc.h
//!
//! @brief Functions for interfacing with the Audio Analog to Digital Converter
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
#ifndef AM_HAL_AUDADC_H
#define AM_HAL_AUDADC_H

//*****************************************************************************
//
//! CMSIS-style macro for handling a variable IOM module number.
//
//*****************************************************************************
#define AUDADCn(n) ((AUDADC_Type*)(AUDADC_BASE + (n * (AUDADC_BASE - AUDADC_BASE))))

//*****************************************************************************
//
//! Maximum number of slots.
//
//*****************************************************************************
#define AM_HAL_AUDADC_MAX_SLOTS            8

//*****************************************************************************
//
//! Minimum value for TRKCYC for the Audio ADC.
//
//*****************************************************************************
#define AM_HAL_AUDADC_MIN_TRKCYC            30

//*****************************************************************************
//
//! AUDADC clock selection.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_AUDADC_CLKSEL_OFF,
    AM_HAL_AUDADC_CLKSEL_HFRC_48MHz,
    AM_HAL_AUDADC_CLKSEL_XTHS_24MHz,
    AM_HAL_AUDADC_CLKSEL_HFRC2_48MHz
} am_hal_audadc_clksel_e;

//*****************************************************************************
//
//! AUDADC periodic trigger source selection
//
//*****************************************************************************
typedef enum
{
    AM_HAL_AUDADC_RPTTRIGSEL_TMR,
    AM_HAL_AUDADC_RPTTRIGSEL_INT
} am_hal_audadc_rpttrigsel_e;

//*****************************************************************************
//
//! AUDADC trigger polarity
//
//*****************************************************************************
typedef enum
{
    AM_HAL_AUDADC_TRIGPOL_RISING,
    AM_HAL_AUDADC_TRIGPOL_FALLING
} am_hal_audadc_trigpol_e;

//*****************************************************************************
//
//! AUDADC trigger selection
//
//*****************************************************************************
typedef enum
{
    AM_HAL_AUDADC_TRIGSEL_EXT0,
    AM_HAL_AUDADC_TRIGSEL_EXT1,
    AM_HAL_AUDADC_TRIGSEL_EXT2,
    AM_HAL_AUDADC_TRIGSEL_EXT3,
    AM_HAL_AUDADC_TRIGSEL_VCOMP,
    AM_HAL_AUDADC_TRIGSEL_SOFTWARE = 7
} am_hal_audadc_trigsel_e;

//
// AUDADC reference selection.
//
//typedef enum
//{
//    AM_HAL_AUDADC_REFSEL_INT_2P0,
//    AM_HAL_AUDADC_REFSEL_INT_1P5,
//    AM_HAL_AUDADC_REFSEL_EXT_2P0,
//    AM_HAL_AUDADC_REFSEL_EXT_1P5
//} am_hal_audadc_refsel_e;

//*****************************************************************************
//
//! AUDADC sample mode selection.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_AUDADC_SAMPMODE_LP = AUDADC_CFG_SAMPMODE_LP,      // LP : Max of 2 low-gain PGA channels configured on slots 0 and 2.
                                                          // In this mode, slots 1 and 3, if enabled, will still
                                                          // consume time but not perform conversions.
    AM_HAL_AUDADC_SAMPMODE_MED = AUDADC_CFG_SAMPMODE_MED,    // MED : Max of 2 low-gain and 2 high-gain PGA channels.
                                                          // In this mode, conversions will be performed on all enabled slots 0 through 3.
} am_hal_audadc_sampmode_e;

//*****************************************************************************
//
//! AUDADC clock mode selection.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_AUDADC_CLKMODE_LOW_POWER,   // Disable the clock between scans for LPMODE0.
                                    // Set LPCKMODE to 0x1 while configuring the AUDADC.
    AM_HAL_AUDADC_CLKMODE_LOW_LATENCY  // Low Latency Clock Mode. When set, HFRC and the
                                    // audadc_clk will remain on while in functioning in LPMODE0.
} am_hal_audadc_clkmode_e;

//*****************************************************************************
//
//! AUDADC low-power mode selection.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_AUDADC_LPMODE0,  // Low Latency Clock Mode. When set, HFRC and the audadc_clk
                            // will remain on while in functioning in LPMODE0.
    AM_HAL_AUDADC_LPMODE1   // Powers down all circuity and clocks associated with the
                            // AUDADC until the next trigger event. Between scans, the reference
                            // buffer requires up to 50us of delay from a scan trigger event
                            // before the conversion will commence while operating in this mode.
} am_hal_audadc_lpmode_e;

//*****************************************************************************
//
//! AUDADC repetition selection.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_AUDADC_SINGLE_SCAN,
    AM_HAL_AUDADC_REPEATING_SCAN
} am_hal_audadc_repeat_e;

//*****************************************************************************
//
//! AUDADC measurement averaging configuration.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_AUDADC_SLOT_AVG_1,
    AM_HAL_AUDADC_SLOT_AVG_2,
    AM_HAL_AUDADC_SLOT_AVG_4,
    AM_HAL_AUDADC_SLOT_AVG_8,
    AM_HAL_AUDADC_SLOT_AVG_16,
    AM_HAL_AUDADC_SLOT_AVG_32,
    AM_HAL_AUDADC_SLOT_AVG_64,
    AM_HAL_AUDADC_SLOT_AVG_128
} am_hal_audadc_meas_avg_e;

//*****************************************************************************
//
//! AUDADC slot precision mode.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_AUDADC_SLOT_12BIT,
    AM_HAL_AUDADC_SLOT_12BIT_1,
    AM_HAL_AUDADC_SLOT_10BIT,
    AM_HAL_AUDADC_SLOT_8BIT
} am_hal_audadc_slot_prec_e;

//*****************************************************************************
//
//! AUDADC slot channel selection.
//
//*****************************************************************************
typedef enum
{
    // Single-ended channels
    AM_HAL_AUDADC_SLOT_CHSEL_SE0,
    AM_HAL_AUDADC_SLOT_CHSEL_SE1,
    AM_HAL_AUDADC_SLOT_CHSEL_SE2,
    AM_HAL_AUDADC_SLOT_CHSEL_SE3,
    AM_HAL_AUDADC_SLOT_CHSEL_SE4,
    AM_HAL_AUDADC_SLOT_CHSEL_SE5,
    AM_HAL_AUDADC_SLOT_CHSEL_SE6,
    AM_HAL_AUDADC_SLOT_CHSEL_SE7,
    // Miscellaneous other signals.
    AM_HAL_AUDADC_SLOT_CHSEL_TEMP,
    AM_HAL_AUDADC_SLOT_CHSEL_BATT,
    AM_HAL_AUDADC_SLOT_TEST_MUX,
    AM_HAL_AUDADC_SLOT_CHSEL_VSS
} am_hal_audadc_slot_chan_e;

//*****************************************************************************
//
//! Scale window comparator limits
//
//*****************************************************************************
typedef enum
{
    AM_HAL_AUDADC_SCALE_WINCOMP_DIS,
    AM_HAL_AUDADC_SCALE_WINCOMP_EN
} am_hal_audadc_scale_wincomp_e;

//*****************************************************************************
//
//! Internal repeating trigger (irtt) timer clock division
//
//*****************************************************************************
typedef enum
{
    AM_HAL_AUDADC_RPTT_CLK_DIV1,
    AM_HAL_AUDADC_RPTT_CLK_DIV2,
    AM_HAL_AUDADC_RPTT_CLK_DIV4,
    AM_HAL_AUDADC_RPTT_CLK_DIV8,
    AM_HAL_AUDADC_RPTT_CLK_DIV16,
    AM_HAL_AUDADC_RPTT_CLK_DIV32,
} am_hal_audadc_irtt_clkdiv_e;

//*****************************************************************************
//
//! DMA priority.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_AUDADC_PRIOR_BEST_EFFORT,
    AM_HAL_AUDADC_PRIOR_SERVICE_IMMED
} am_hal_audadc_dma_prior_e;

//*****************************************************************************
//!
//! AUDADC control function request types for am_hal_audadc_control().
//!
//! AM_HAL_AUDADC_REQ_TEMP_CELSIUS_GET:
//!     pArgs must point to an array of 3 floats.  To assure that the
//!     array is valid, upon calling the 3rd float (pArgs[2]) must be
//!     set to the value -123.456F.
//! AM_HAL_AUDADC_REQ_TEMP_TRIMS_GET:
//!     pArgs must point to an array of 4 floats.  To assure that the
//!     array is valid, upon calling the 4th float (pArgs[3]) must be
//!     set to the to the value -123.456F.
//!     On return, pArgs[3] is set to 1 if the returned values are
//!     calibrated, or 0 if default calibration values.
//!
//*****************************************************************************
typedef enum
{
    AM_HAL_AUDADC_REQ_WINDOW_CONFIG,
    AM_HAL_AUDADC_REQ_TEMP_CELSIUS_GET,
    AM_HAL_AUDADC_REQ_TEMP_TRIMS_GET,
    AM_HAL_AUDADC_REQ_DMA_DISABLE,
} am_hal_audadc_request_e;

//*****************************************************************************
//
//! PGA update mode
//
//*****************************************************************************
typedef enum
{
    AM_HAL_AUDADC_GAIN_UPDATE_IMME, //Immediate update mode. Once gain is written,
                                    // it is immediately encoded and provided to the PGA
    AM_HAL_AUDADC_GAIN_UPDATE_ZX,   //Update gain only at detected zero crossing as configured by ZX registers
}am_hal_audadc_gain_update_e;

//*****************************************************************************
//
//! AUDADC Sample structure.
//
//*****************************************************************************
typedef struct
{
    int16_t      int16Sample;

    //! Which audio channel this data is from encoded as int(slot number/2).
    //! In other words, this is 1 if data is from slots 2 or 3, or 0 if from slots 0 or 1.
    uint16_t      ui16AudChannel;
} am_hal_audadc_sample_t;

//*****************************************************************************
//
//! @brief Configuration structure for the AUDADC.
//
//*****************************************************************************
typedef struct
{
    //! Select the AUDADC clock source.
    am_hal_audadc_clksel_e           eClock;

    //! select the periodic trigger source
    am_hal_audadc_rpttrigsel_e       eRepeatTrigger;

    //! Select the AUDADC trigger polarity.
    am_hal_audadc_trigpol_e          ePolarity;

    //! Select the AUDADC trigger source.
    am_hal_audadc_trigsel_e          eTrigger;

    // Select the AUDADC reference voltage.
//    am_hal_audadc_refsel_e           eReference;

    //! sample mode selection
    am_hal_audadc_sampmode_e         eSampMode;
    //! Whether to disable clocks between samples.
    am_hal_audadc_clkmode_e          eClockMode;

    //! Select the AUDADC power mode.
    am_hal_audadc_lpmode_e           ePowerMode;

    //! Select whether the AUDADC will re-trigger based on a signal from timer.
    am_hal_audadc_repeat_e           eRepeat;

} am_hal_audadc_config_t;

//*****************************************************************************
//
//! @brief Configuration structure for the AUDADC slot.
//
//*****************************************************************************
typedef struct
{
    //! Select the number of measurements to average
    am_hal_audadc_meas_avg_e         eMeasToAvg;

    //! Set additional input sampling AUDADC clcok cycles
    uint32_t                      ui32TrkCyc;

    //! Select the precision mode
    am_hal_audadc_slot_prec_e        ePrecisionMode;

    //! Select the channel
    am_hal_audadc_slot_chan_e        eChannel;

    //! Select window comparison mode
    bool                          bWindowCompare;

    //! Enable the slot
    bool                          bEnabled;

} am_hal_audadc_slot_config_t;

//*****************************************************************************
//
//! @brief Configuration structure for the AUDADC GAIN CODES.
//
//*****************************************************************************
typedef struct
{
    //! Specifies the low gain code
    //! (0 to 102 decimal specifies -6.0 dB to 45.0 dB in half-dB increments) for channel A (slot 0).
    uint32_t ui32LGA;

    //! Specifies the high gain code as an delta from the LGA field for channel A (slot 1).
    uint32_t ui32HGADELTA;

    //! Specifies the low gain code
    //! (0 to 102 decimal specifies -6.0 dB to 45.0 dB in half-dB increments) for channel B (slot 2).
    uint32_t ui32LGB;

    //! Specifies the high gain code as an delta from the LGB field for channel B (slot 3).
    uint32_t ui32HGBDELTA;

    //! PGA update mode
    am_hal_audadc_gain_update_e eUpdateMode;

    //!
} am_hal_audadc_gain_config_t;

//*****************************************************************************
//
//! @brief Configuration structure for the AUDADC Saturation Comparator.
//
//*****************************************************************************
typedef struct
{
    //! Select which slots to use for saturation measurement.
    uint32_t ui32ChanSel;

    /*! Sets the lower integer sample limit for the saturation
                                                     comparator. Note that these values are raw ADC values whose
                                                     bounds are specified by PRMODE but not manipulated by accumulate/divide
                                                     logic. Therefore, there is no oversampling and no binary
                                                     point in this value.                                                 */

    uint32_t ui32LowerSATCLimt;
    //! Sets the upper integer sample limit for the saturation
    uint32_t ui32UpperSATCLimt;

    /*!< Sets the number of saturation events that may occur
                                                     before a SATCA interrupt occurs. Once this interrupt occurs,
                                                     the saturation event counter must be cleared by writing
                                                     the SATCLR register. A value of 0 is invalid and will cause
                                                     the saturation interrupt to assert immediately.           */
    uint32_t ui32SATCAMax;
    //! Sets the number of saturation events that may occur before a SATCB interrupt occurs.
    uint32_t ui32SATCBMax;

} am_hal_audadc_sat_config_t;

//*****************************************************************************
//
//! @brief Configuration structure for the AUDADC internal repeat trigger timer.
//
//*****************************************************************************
typedef struct
{
    //! AUDADC-internal repeat trigger timer enable
    bool                          bIrttEnable;
    am_hal_audadc_irtt_clkdiv_e      eClkDiv;
    uint32_t                      ui32IrttCountMax;
} am_hal_audadc_irtt_config_t;

//*****************************************************************************
//
//! @brief Configuration structure for the AUDADC DMA
//
//*****************************************************************************
typedef struct
{
    //! AUDADC DMA dynamic priority enabled.
    bool                          bDynamicPriority;

    //! AUDADC DMA static priority.
    am_hal_audadc_dma_prior_e        ePriority;

    //! Enable DMA for AUDADC
    bool                          bDMAEnable;

    //! Transfer count in samples
    uint32_t                      ui32SampleCount;

    //! Target address
    uint32_t                      ui32TargetAddress;

    //! Pingpong buffer
    uint32_t                      ui32TargetAddressReverse;

} am_hal_audadc_dma_config_t;

//*****************************************************************************
//
//! @brief Window configuration structure for the AUDADC.
//
//*****************************************************************************
typedef struct
{
    //
    //! Scale window comparison
    //
    bool                          bScaleLimits;

    //
    //! Window limits
    //
    uint32_t                      ui32Upper;
    uint32_t                      ui32Lower;

} am_hal_audadc_window_config_t;

//*****************************************************************************
//
//! @brief Capabilities structure for the AUDADC.
//
//*****************************************************************************
typedef struct
{
    uint32_t      dummy;

} am_hal_audadc_capabilities_t;

//*****************************************************************************
//
//! @brief Status structure for the AUDADC.
//
//*****************************************************************************
typedef struct
{
    //
    //! AUDADC power status.
    //
    bool                          bPoweredOn;
    bool                          bLPMode1;

    //
    //! DMA status.
    //
    bool                          bErr;
    bool                          bCmp;
    bool                          bTIP;

} am_hal_audadc_status_t;

//*****************************************************************************
//
//! Transfer callback function prototype
//
//*****************************************************************************
typedef void (*am_hal_audadc_callback_t)(void *pCallbackCtxt, uint32_t status);

//*****************************************************************************
//
//! @name AUDADC Interrupts
//! @{
//! Interrupt Status Bits for enable/disble use
//!
//! These macros may be used to enable an individual AUDADC interrupt cause.
//
//*****************************************************************************
#define AM_HAL_AUDADC_INT_DERR               (_VAL2FLD(AUDADC_INTEN_DERR, 1))
#define AM_HAL_AUDADC_INT_DCMP               (_VAL2FLD(AUDADC_INTEN_DCMP, 1))
#define AM_HAL_AUDADC_INT_WCINC              (_VAL2FLD(AUDADC_INTEN_WCINC, 1))
#define AM_HAL_AUDADC_INT_WCEXC              (_VAL2FLD(AUDADC_INTEN_WCEXC, 1))
#define AM_HAL_AUDADC_INT_FIFOOVR2           (_VAL2FLD(AUDADC_INTEN_FIFOOVR2, 1))
#define AM_HAL_AUDADC_INT_FIFOOVR1           (_VAL2FLD(AUDADC_INTEN_FIFOOVR1, 1))
#define AM_HAL_AUDADC_INT_SCNCMP             (_VAL2FLD(AUDADC_INTEN_SCNCMP, 1))
#define AM_HAL_AUDADC_INT_CNVCMP             (_VAL2FLD(AUDADC_INTEN_CNVCMP, 1))
#define AM_HAL_AUDADC_INT_ZXCA               (_VAL2FLD(AUDADC_INTEN_ZXCA, 1))
#define AM_HAL_AUDADC_INT_ZXCB               (_VAL2FLD(AUDADC_INTEN_ZXCB, 1))
#define AM_HAL_AUDADC_INT_SATCA              (_VAL2FLD(AUDADC_INTEN_SATCA, 1))
#define AM_HAL_AUDADC_INT_SATCB              (_VAL2FLD(AUDADC_INTEN_SATCB, 1))
//! @}

//*****************************************************************************
//
//! @name AUDADC Fifo Read macros
//! @{
//! These are helper macros for interpreting FIFO data. Each AUDADC FIFO entry
//! contains information about the slot number and the FIFO depth alongside the
//! current sample. These macros perform the correct masking and shifting to
//! read those values.
//!
//! The SAMPLE and FULL_SAMPLE options refer to the fractional part of averaged
//! samples. If you are not using hardware averaging or don't need the
//! fractional part of the AUDADC sample, you should just use
//! AM_HAL_AUDADC_FIFO_SAMPLE.
//!
//! If you do need the fractional part, use AM_HAL_AUDADC_FIFO_FULL_SAMPLE. This
//! macro will keep six bits of precision past the decimal point. Depending on
//! the number of averaged samples, anywhere between 1 and 6 of these bits will
//! be valid. Please consult the datasheet to find out how many bits of data
//! are valid for your chosen averaging settings.
//!
//
//*****************************************************************************
//#define AM_HAL_AUDADC_FIFO_SAMPLE(value)       (_FLD2VAL(AUDADC_FIFOPR_LGDATAPR, value) >> 6)
#define AM_HAL_AUDADC_FIFO_LGDATA(value)       (_FLD2VAL(AUDADC_FIFOPR_LGDATAPR, value))
#define AM_HAL_AUDADC_FIFO_HGDATA(value)       (_FLD2VAL(AUDADC_FIFOPR_HGDATAPR, value))
//#define AM_HAL_AUDADC_FIFO_FULL_SAMPLE(value)  (_FLD2VAL(AUDADC_FIFOPR_METALOPR, value))
#define AM_HAL_AUDADC_FIFO_SLOT(value)         (_FLD2VAL(AUDADC_FIFO_MIC, value))
#define AM_HAL_AUDADC_FIFO_COUNT(value)        (_FLD2VAL(AUDADC_FIFO_COUNT, value))
#define AM_HAL_AUDADC_FIFO_EMPTY(h)            (((AUDADC_Type*)h)->FIFO == 0xFFFFFFFF)? 1:0
//! @}

#ifdef __cplusplus
extern "C"
{
#endif

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
extern uint32_t am_hal_audadc_initialize(uint32_t ui32Module, void **ppHandle);

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
extern uint32_t am_hal_audadc_deinitialize(void *pHandle);

//*****************************************************************************
//
//! @brief AUDADC configuration function
//!
//! @param pHandle - handle for the module instance.
//! @param psConfig    - pointer to the configuration structure.
//!
//! This function configures the AUDADC for operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_audadc_configure(void *pHandle,
                                        am_hal_audadc_config_t *psConfig);

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
extern uint32_t am_hal_audadc_configure_slot(void *pHandle,
                                             uint32_t ui32SlotNumber,
                                             am_hal_audadc_slot_config_t *pSlotConfig);

//*****************************************************************************
//
//! @brief AUDADC internal repeat trigger timer configuration function
//!
//! @param pHandle - handle for the module instance.
//! @param pConfig - pointer to the configuration structure.
//!
//! This function configures the AUDADC internal trigger timer for operation.
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_audadc_configure_irtt(void *pHandle,
                                             am_hal_audadc_irtt_config_t *pConfig);

//*****************************************************************************
//
//! @brief AUDADC internal repeating trigger timer enable function
//!
//! @param pHandle   - handle for the module instance.
//!
//! This function enables internal repeating trigger timer.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_irtt_enable(void *pHandle);

//*****************************************************************************
//
//! @brief AUDADC internal repeating trigger timer disable function
//!
//! @param pHandle   - handle for the module instance.
//!
//! This function disables internal repeating trigger timer.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
uint32_t
am_hal_audadc_irtt_disable(void *pHandle);

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
extern uint32_t am_hal_audadc_configure_dma(void *pHandle,
                                            am_hal_audadc_dma_config_t *pDMAConfig);

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
extern uint32_t am_hal_audadc_dma_get_buffer(void *pHandle);

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
extern uint32_t am_hal_audadc_control(void *pHandle,
                                      am_hal_audadc_request_e eRequest,
                                      void *pArgs);

//*****************************************************************************
//
//! @brief AUDADC enable function
//!
//! @param pHandle - handle for the module instance.
//!
//! This function enables the AUDADC operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_audadc_enable(void *pHandle);

//*****************************************************************************
//
//! @brief AUDADC disable function
//!
//! @param pHandle - handle for the module instance.
//!
//! This function disables the AUDADC operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_audadc_disable(void *pHandle);

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
extern uint32_t am_hal_audadc_status_get(void *pHandle,
                                         am_hal_audadc_status_t *pStatus );

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
extern uint32_t am_hal_audadc_interrupt_enable(void *pHandle, uint32_t ui32IntMask);

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
extern uint32_t am_hal_audadc_interrupt_disable(void *pHandle, uint32_t ui32IntMask);

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
extern uint32_t am_hal_audadc_interrupt_status(void *pHandle,
                                               uint32_t  *pui32Status,
                                               bool bEnabledOnly);

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
extern uint32_t am_hal_audadc_interrupt_clear(void *pHandle, uint32_t ui32IntMask);

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
extern uint32_t am_hal_audadc_interrupt_service(void *pHandle,
                                                am_hal_audadc_dma_config_t *pDMAConfig);

//*****************************************************************************
//
//! @brief AUDADC sample read function
//!
//! @param pHandle              - handle for the module instance.
//! @param pui32InSampleBuffer  - Ptr to the input sample buffer.
//!                               If NULL then samples will be read directly
//!                               from the FIFO.
//! @param pui32InOutNumberSamples - Ptr to variable containing the number of
//!                                  samples.
//! @param bLowSample              - Boolean for Low Sample
//! @param pui32LGOutBuffer        - Ptr to the required output Low-gain PGA sample buffer.
//! @param bHighSample             - Boolean for High Sample
//! @param pui32HGOutBuffer        - Ptr to the required output High-gain PGA sample buffer.
//!
//! This function reads samples from the AUDADC FIFO or an SRAM sample buffer
//! returned by a DMA operation, and calibrates the samples.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_audadc_samples_read(void *pHandle,
                                           uint32_t *pui32InSampleBuffer,
                                           uint32_t *pui32InOutNumberSamples,
                                           bool bLowSample,  am_hal_audadc_sample_t *pui32LGOutBuffer,
                                           bool bHighSample, am_hal_audadc_sample_t *pui32HGOutBuffer);

//*****************************************************************************
//
//! @brief AUDADC software trigger function
//!
//! @param pHandle - handle for the module instance.
//!
//! This function triggers the AUDADC operation.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_audadc_sw_trigger(void *pHandle);


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
extern uint32_t am_hal_audadc_power_control(void *pHandle,
                                            am_hal_sysctrl_power_state_e ePowerState,
                                            bool bRetainState);

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
extern uint32_t am_hal_audadc_gain_set(uint32_t ui32ChanNum, int32_t in32GaindBx2);

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
extern uint32_t am_hal_audadc_pga_powerup(uint32_t ui32ChanNum);

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
extern uint32_t am_hal_audadc_pga_powerdown(uint32_t ui32ChanNum);

//*****************************************************************************
//
//! @brief Turn on reference
//!
//! This function turn on the reference voltage and current.
//!
//! @return status          - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_audadc_refgen_powerup(void);

//*****************************************************************************
//
//! @brief Turn off reference
//!
//! This function turn off the reference voltage and current.
//!
//! @return status          - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_audadc_refgen_powerdown(void);

//*****************************************************************************
//
//! @brief Turn on mic bias voltage to power up the mic
//!
//! @param  ui32VolTrim         - The value to set the output voltage
//!
//! This function sets output of the mic bias voltage.
//
//*****************************************************************************
extern void am_hal_audadc_micbias_powerup(uint32_t ui32VolTrim);

//*****************************************************************************
//
//! @brief Turn off mic bias voltage to power off the mic
//!
//! This function turns off the mic bias voltage.
//
//*****************************************************************************
extern void am_hal_audadc_micbias_powerdown(void);

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
//
//*****************************************************************************
extern uint32_t am_hal_audadc_internal_pga_config(void *pHandle, am_hal_audadc_gain_config_t* psGainConfig);

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
//
//*****************************************************************************
extern uint32_t am_hal_audadc_saturation_config(void *pHandle, am_hal_audadc_sat_config_t* psSATConfig);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_AUDADC_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

