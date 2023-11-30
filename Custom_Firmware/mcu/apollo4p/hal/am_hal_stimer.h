//*****************************************************************************
//
//! @file am_hal_stimer.h
//!
//! @brief Functions for interfacing with the system timer (STIMER).
//!
//! @addtogroup stimer4_4p STIMER - System Timer
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
#ifndef AM_HAL_STIMER_H
#define AM_HAL_STIMER_H

#ifdef __cplusplus
extern "C"
{
#endif

//
//! Compute address of a given COMPARE register.
//! @note - The parameter n should be 0 (as only 1 stimer module exists).
//!         For Apollo3, the parameter r should be 0-7 (compare) or 0-3 (capture).
//
#define AM_HAL_STIMER_COMPARE_OFFSET (&STIMER->SCMPR1 - &STIMER->SCMPR0)
#define AM_REG_STIMER_COMPARE(n, r)     ((&STIMER->SCMPR0) +   \
                                         (r * AM_HAL_STIMER_COMPARE_OFFSET))

//! Compute address of a given CAPTURE register. r should be 0-3.
#define AM_HAL_STIMER_CAPTURE_OFFSET (&STIMER->SCAPT1 - &STIMER->SCAPT0)
#define AM_REG_STIMER_CAPTURE(n, r)     ((&STIMER->SCAPT0) +   \
                                         (r * AM_HAL_STIMER_CAPTURE_OFFSET))

//! Compute address of a given NVRAM register. r should be 0-3.
#define AM_HAL_STIMER_NVRAM_OFFSET (&STIMER->SNVR1 - &STIMER->SNVR0)
#define AM_REG_STIMER_NVRAM(n, r)       ((&STIMER->SNVR0) +    \
                                         (r * AM_HAL_STIMER_NVRAM_OFFSET))

//*****************************************************************************
//
//! @name Interrupt Status Bits
//! @{
//! Interrupt Status Bits for enable/disble use
//!
//! These macros may be used to set and clear interrupt bits
//
//*****************************************************************************
#define AM_HAL_STIMER_INT_COMPAREA         STIMER_STMINTSTAT_COMPAREA_Msk
#define AM_HAL_STIMER_INT_COMPAREB         STIMER_STMINTSTAT_COMPAREB_Msk
#define AM_HAL_STIMER_INT_COMPAREC         STIMER_STMINTSTAT_COMPAREC_Msk
#define AM_HAL_STIMER_INT_COMPARED         STIMER_STMINTSTAT_COMPARED_Msk
#define AM_HAL_STIMER_INT_COMPAREE         STIMER_STMINTSTAT_COMPAREE_Msk
#define AM_HAL_STIMER_INT_COMPAREF         STIMER_STMINTSTAT_COMPAREF_Msk
#define AM_HAL_STIMER_INT_COMPAREG         STIMER_STMINTSTAT_COMPAREG_Msk
#define AM_HAL_STIMER_INT_COMPAREH         STIMER_STMINTSTAT_COMPAREH_Msk

#define AM_HAL_STIMER_INT_OVERFLOW         STIMER_STMINTSTAT_OVERFLOW_Msk

#define AM_HAL_STIMER_INT_CAPTUREA         STIMER_STMINTSTAT_CAPTUREA_Msk
#define AM_HAL_STIMER_INT_CAPTUREB         STIMER_STMINTSTAT_CAPTUREB_Msk
#define AM_HAL_STIMER_INT_CAPTUREC         STIMER_STMINTSTAT_CAPTUREC_Msk
#define AM_HAL_STIMER_INT_CAPTURED         STIMER_STMINTSTAT_CAPTURED_Msk
//! @}

//*****************************************************************************
//
//! @name STimer Configuration Bits
//! @{
//! Interrupt Status Bits for enable/disble use
//!
//! These macros may be used to set and clear interrupt bits
//
//*****************************************************************************
#define AM_HAL_STIMER_CFG_THAW              _VAL2FLD(STIMER_STCFG_FREEZE,       STIMER_STCFG_FREEZE_THAW)
#define AM_HAL_STIMER_CFG_FREEZE            _VAL2FLD(STIMER_STCFG_FREEZE,       STIMER_STCFG_FREEZE_FREEZE)
#define AM_HAL_STIMER_CFG_RUN               _VAL2FLD(STIMER_STCFG_CLEAR,        STIMER_STCFG_CLEAR_RUN)
#define AM_HAL_STIMER_CFG_CLEAR             _VAL2FLD(STIMER_STCFG_CLEAR,        STIMER_STCFG_CLEAR_CLEAR)
#define AM_HAL_STIMER_CFG_COMPARE_A_ENABLE  _VAL2FLD(STIMER_STCFG_COMPAREAEN, STIMER_STCFG_COMPAREAEN_ENABLE)
#define AM_HAL_STIMER_CFG_COMPARE_B_ENABLE  _VAL2FLD(STIMER_STCFG_COMPAREBEN, STIMER_STCFG_COMPAREBEN_ENABLE)
#define AM_HAL_STIMER_CFG_COMPARE_C_ENABLE  _VAL2FLD(STIMER_STCFG_COMPARECEN, STIMER_STCFG_COMPARECEN_ENABLE)
#define AM_HAL_STIMER_CFG_COMPARE_D_ENABLE  _VAL2FLD(STIMER_STCFG_COMPAREDEN, STIMER_STCFG_COMPAREDEN_ENABLE)
#define AM_HAL_STIMER_CFG_COMPARE_E_ENABLE  _VAL2FLD(STIMER_STCFG_COMPAREEEN, STIMER_STCFG_COMPAREEEN_ENABLE)
#define AM_HAL_STIMER_CFG_COMPARE_F_ENABLE  _VAL2FLD(STIMER_STCFG_COMPAREFEN, STIMER_STCFG_COMPAREFEN_ENABLE)
#define AM_HAL_STIMER_CFG_COMPARE_G_ENABLE  _VAL2FLD(STIMER_STCFG_COMPAREGEN, STIMER_STCFG_COMPAREGEN_ENABLE)
#define AM_HAL_STIMER_CFG_COMPARE_H_ENABLE  _VAL2FLD(STIMER_STCFG_COMPAREHEN, STIMER_STCFG_COMPAREHEN_ENABLE)
//! @}

//*****************************************************************************
//
//! @name Clock Configuration options
//! @{
//! STimer Configuration register options.
//!
//! These options are to be used with the am_hal_stimer_config() function.
//
//*****************************************************************************
#define AM_HAL_STIMER_NO_CLK            _VAL2FLD(STIMER_STCFG_CLKSEL, STIMER_STCFG_CLKSEL_NOCLK)
#if defined(AM_PART_APOLLO4)
#define AM_HAL_STIMER_HFRC_3MHZ         _VAL2FLD(STIMER_STCFG_CLKSEL, STIMER_STCFG_CLKSEL_HFRC_3MHZ)
#define AM_HAL_STIMER_HFRC_187_5KHZ     _VAL2FLD(STIMER_STCFG_CLKSEL, STIMER_STCFG_CLKSEL_HFRC_187KHZ)
#elif defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P)
#define AM_HAL_STIMER_HFRC_6MHZ         _VAL2FLD(STIMER_STCFG_CLKSEL, STIMER_STCFG_CLKSEL_HFRC_6MHZ)
#define AM_HAL_STIMER_HFRC_375KHZ       _VAL2FLD(STIMER_STCFG_CLKSEL, STIMER_STCFG_CLKSEL_HFRC_375KHZ)
#endif
#define AM_HAL_STIMER_XTAL_32KHZ        _VAL2FLD(STIMER_STCFG_CLKSEL, STIMER_STCFG_CLKSEL_XTAL_32KHZ)
#define AM_HAL_STIMER_XTAL_16KHZ        _VAL2FLD(STIMER_STCFG_CLKSEL, STIMER_STCFG_CLKSEL_XTAL_16KHZ)
#define AM_HAL_STIMER_XTAL_1KHZ         _VAL2FLD(STIMER_STCFG_CLKSEL, STIMER_STCFG_CLKSEL_XTAL_1KHZ)
#define AM_HAL_STIMER_LFRC_1KHZ         _VAL2FLD(STIMER_STCFG_CLKSEL, STIMER_STCFG_CLKSEL_LFRC_1KHZ)
#define AM_HAL_STIMER_HFRC_CTIMER0A     _VAL2FLD(STIMER_STCFG_CLKSEL, STIMER_STCFG_CLKSEL_CTIMER0)
#define AM_HAL_STIMER_HFRC_CTIMER0B     _VAL2FLD(STIMER_STCFG_CLKSEL, STIMER_STCFG_CLKSEL_CTIMER1)
//! @}

//*****************************************************************************
//
//! @name Minimum delta value that can be set for Stimer
//! @{
//! STimer Limit options.
//!
//! This constant is relevant to the am_hal_stimer_compare_delta_set() function.
//
//*****************************************************************************
#define AM_HAL_STIMER_MIN_DELTA         4
//! @}

//*****************************************************************************
//
//! @name Capture Control Register options.
//! @{
//! Configuration options for capture control register.
//!
//! These options are to be used with the am_hal_stimer_capture_control_set
//! function.
//
//*****************************************************************************
#define AM_HAL_STIMER_CAPTURE0_ENABLE   _VAL2FLD(STIMER_CAPTURECONTROL_CAPTURE0, STIMER_CAPTURECONTROL_CAPTURE0_ENABLE)
#define AM_HAL_STIMER_CAPTURE1_ENABLE   _VAL2FLD(STIMER_CAPTURECONTROL_CAPTURE1, STIMER_CAPTURECONTROL_CAPTURE1_ENABLE)
#define AM_HAL_STIMER_CAPTURE2_ENABLE   _VAL2FLD(STIMER_CAPTURECONTROL_CAPTURE2, STIMER_CAPTURECONTROL_CAPTURE2_ENABLE)
#define AM_HAL_STIMER_CAPTURE3_ENABLE   _VAL2FLD(STIMER_CAPTURECONTROL_CAPTURE3, STIMER_CAPTURECONTROL_CAPTURE3_ENABLE)
//! @}

//*****************************************************************************
//
//! STIMER Specific status codes
//
//*****************************************************************************
typedef enum
{
    //
    //! Delta too small for setting Compare
    //
    AM_HAL_STIMER_DELTA_TOO_SMALL = AM_HAL_STATUS_MODULE_SPECIFIC_START
} am_hal_stimer_err_e;

//*****************************************************************************
//
//! Stimer configuration structure
//
//*****************************************************************************
typedef struct
{
    //
    //! Configuration options for the STIMER
    //
    uint32_t ui32STimerConfig;
}
am_hal_stimer_config_t;

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief Set up the stimer.
//!
//! @param ui32STimerConfig is the value to load into the configuration reg.
//!
//! This function should be used to perform the initial set-up of the
//! stimer.
//!
//! @return The 32-bit current config of the STimer Config register
//
//*****************************************************************************
extern uint32_t am_hal_stimer_config(uint32_t ui32STimerConfig);

//*****************************************************************************
//
//! @brief Check if the STIMER is running.
//!
//! This function should be used to perform the initial set-up of the
//! stimer.
//!
//! @return true = STIMER running;  false = STIMER needs to be configured.
//
//*****************************************************************************
extern bool am_hal_stimer_is_running(void);

//*****************************************************************************
//
//! @brief Reset the current stimer block to power-up state.
//
//*****************************************************************************
extern void am_hal_stimer_reset_config(void);

//*****************************************************************************
//
//! @brief Get the current stimer value.
//!
//! This function can be used to read, uninvasively, the value in the stimer.
//!
//! @return The 32-bit value from the STimer counter register.
//
//*****************************************************************************
extern uint32_t am_hal_stimer_counter_get(void);

//*****************************************************************************
//
//! @brief Clear the stimer counter.
//!
//! This function clears the STimer Counter and leaves the stimer running.
//
//*****************************************************************************
extern void     am_hal_stimer_counter_clear(void);

//*****************************************************************************
//
//! @brief Set the compare value.
//!
//! @param ui32CmprInstance is the compare register instance number (0-7).
//! @param ui32Delta is the value to add to the STimer counter and load into
//!        the comparator register.
//!
//! @note There is no way to set an absolute value into a comparator register.
//!       Only deltas added to the STimer counter can be written to the compare
//!       registers.
//!       Back to back writes to COMPARE are not reliable. This function takes
//!       care of adding necessary wait till the value can be safely written.
//!       If blocking in this function is not desirable, application should
//!       check for it beforehand using am_hal_stimer_check_compare_delta_set()
//!
//! @return AM_HAL_STATUS_SUCCESS on success.
//!       The minimum value that can be set is AM_HAL_STIMER_MIN_DELTA. If the
//!       value supplied is less - the function bumps it up to safe value and
//!       returns AM_HAL_STIMER_DELTA_TOO_SMALL
//
//*****************************************************************************
extern uint32_t am_hal_stimer_compare_delta_set(uint32_t ui32CmprInstance,
                                                uint32_t ui32Delta);

//*****************************************************************************
//
//! @brief Get the current stimer compare register value.
//!
//! @param ui32CmprInstance is the compare register instance number (0-7).
//!
//! This function can be used to read the value in an stimer compare register.
//!
//! @note stimer compare register cannot be reliably read immediately after it
//!       has been written to. This function will block in a wait loop
//!       till the value can be safely read.
//!
//! @return Return the compare register value.
//
//*****************************************************************************
extern uint32_t am_hal_stimer_compare_get(uint32_t ui32CmprInstance);

//*****************************************************************************
//
//! @brief Start capturing data with the specified capture register.
//!
//! @param ui32CaptureNum is the Capture Register Number to read (0-3).
//! @param ui32GPIONumber is the pin number.
//! @param bPolarity: false (0) = Capture on low to high transition.
//!                   true  (1) = Capture on high to low transition.
//!
//! Use this function to start capturing.
//
//*****************************************************************************
extern void     am_hal_stimer_capture_start(uint32_t ui32CaptureNum,
                                            uint32_t ui32GPIONumber,
                                            bool bPolarity);

//*****************************************************************************
//
//! @brief Stop capturing data with the specified capture register.
//!
//! @param ui32CaptureNum is the Capture Register Number to read.
//!
//! Use this function to stop capturing.
//
//*****************************************************************************
extern void     am_hal_stimer_capture_stop(uint32_t ui32CaptureNum);

//*****************************************************************************
//
//! @brief Get the current stimer capture register value.
//!
//! @param ui32CaptureNum is the Capture Register Number to read.
//!
//! This function can be used to read the value in an stimer capture register.
//!
//! @return Return the capture register value.
//
//*****************************************************************************
extern uint32_t am_hal_stimer_capture_get(uint32_t ui32CaptureNum);

//*****************************************************************************
//
//! @brief Set the current stimer nvram register value.
//!
//! @param ui32NvramNum is the NVRAM Register Number to read.
//! @param ui32NvramVal is the value to write to NVRAM.
//!
//! This function can be used to read the value in an stimer NVRAM register.
//
//*****************************************************************************
extern void am_hal_stimer_nvram_set(uint32_t ui32NvramNum, uint32_t ui32NvramVal);

//*****************************************************************************
//
//! @brief Get the current stimer nvram register value.
//!
//! @param ui32NvramNum is the NVRAM Register Number to read.
//!
//! This function can be used to read the value in an stimer NVRAM register.
//!
//! @return Return the nvram register value.
//
//*****************************************************************************
extern uint32_t am_hal_stimer_nvram_get(uint32_t ui32NvramNum);

//*****************************************************************************
//
//! @brief Enables the selected system timer interrupt.
//!
//! @param ui32Interrupt is the interrupt to be used.
//!
//! This function will enable the selected interrupts in the STIMER interrupt
//! enable register. In order to receive an interrupt from an stimer component,
//! you will need to enable the interrupt for that component in this main
//! register, as well as in the stimer configuration register (accessible though
//! am_hal_stimer_config()), and in the NVIC.
//!
//! ui32Interrupt should be the logical OR of one or more of the following
//! values:
//!
//!     AM_HAL_STIMER_INT_COMPAREA
//!     AM_HAL_STIMER_INT_COMPAREB
//!     AM_HAL_STIMER_INT_COMPAREC
//!     AM_HAL_STIMER_INT_COMPARED
//!     AM_HAL_STIMER_INT_COMPAREE
//!     AM_HAL_STIMER_INT_COMPAREF
//!     AM_HAL_STIMER_INT_COMPAREG
//!     AM_HAL_STIMER_INT_COMPAREH
//!
//!     AM_HAL_STIMER_INT_OVERFLOW
//!
//!     AM_HAL_STIMER_INT_CAPTUREA
//!     AM_HAL_STIMER_INT_CAPTUREB
//!     AM_HAL_STIMER_INT_CAPTUREC
//!     AM_HAL_STIMER_INT_CAPTURED
//
//*****************************************************************************
extern void     am_hal_stimer_int_enable(uint32_t ui32Interrupt);

//*****************************************************************************
//
//! @brief Return the enabled stimer interrupts.
//!
//! This function will return all enabled interrupts in the STIMER
//! interrupt enable register.
//!
//! @return return enabled interrupts. This will be a logical or of:
//!
//!     AM_HAL_STIMER_INT_COMPAREA
//!     AM_HAL_STIMER_INT_COMPAREB
//!     AM_HAL_STIMER_INT_COMPAREC
//!     AM_HAL_STIMER_INT_COMPARED
//!     AM_HAL_STIMER_INT_COMPAREE
//!     AM_HAL_STIMER_INT_COMPAREF
//!     AM_HAL_STIMER_INT_COMPAREG
//!     AM_HAL_STIMER_INT_COMPAREH
//!
//!     AM_HAL_STIMER_INT_OVERFLOW
//!
//!     AM_HAL_STIMER_INT_CAPTUREA
//!     AM_HAL_STIMER_INT_CAPTUREB
//!     AM_HAL_STIMER_INT_CAPTUREC
//!     AM_HAL_STIMER_INT_CAPTURED
//!
//! @return Return the enabled timer interrupts.
//
//*****************************************************************************
extern uint32_t am_hal_stimer_int_enable_get(void);

//*****************************************************************************
//
//! @brief Disables the selected stimer interrupt.
//!
//! @param ui32Interrupt is the interrupt to be used.
//!
//! This function will disable the selected interrupts in the STIMER
//! interrupt register.
//!
//! ui32Interrupt should be the logical OR of one or more of the following
//! values:
//!
//!     AM_HAL_STIMER_INT_COMPAREA
//!     AM_HAL_STIMER_INT_COMPAREB
//!     AM_HAL_STIMER_INT_COMPAREC
//!     AM_HAL_STIMER_INT_COMPARED
//!     AM_HAL_STIMER_INT_COMPAREE
//!     AM_HAL_STIMER_INT_COMPAREF
//!     AM_HAL_STIMER_INT_COMPAREG
//!     AM_HAL_STIMER_INT_COMPAREH
//!
//!     AM_HAL_STIMER_INT_OVERFLOW
//!
//!     AM_HAL_STIMER_INT_CAPTUREA
//!     AM_HAL_STIMER_INT_CAPTUREB
//!     AM_HAL_STIMER_INT_CAPTUREC
//!     AM_HAL_STIMER_INT_CAPTURED
//
//*****************************************************************************
extern void     am_hal_stimer_int_disable(uint32_t ui32Interrupt);

//*****************************************************************************
//
//! @brief Sets the selected stimer interrupt.
//!
//! @param ui32Interrupt is the interrupt to be used.
//!
//! This function will set the selected interrupts in the STIMER
//! interrupt register.
//!
//! ui32Interrupt should be the logical OR of one or more of the following
//! values:
//!
//!     AM_HAL_STIMER_INT_COMPAREA
//!     AM_HAL_STIMER_INT_COMPAREB
//!     AM_HAL_STIMER_INT_COMPAREC
//!     AM_HAL_STIMER_INT_COMPARED
//!     AM_HAL_STIMER_INT_COMPAREE
//!     AM_HAL_STIMER_INT_COMPAREF
//!     AM_HAL_STIMER_INT_COMPAREG
//!     AM_HAL_STIMER_INT_COMPAREH
//!
//!     AM_HAL_STIMER_INT_OVERFLOW
//!
//!     AM_HAL_STIMER_INT_CAPTUREA
//!     AM_HAL_STIMER_INT_CAPTUREB
//!     AM_HAL_STIMER_INT_CAPTUREC
//!     AM_HAL_STIMER_INT_CAPTURED
//
//*****************************************************************************
extern void     am_hal_stimer_int_set(uint32_t ui32Interrupt);

//*****************************************************************************
//
//! @brief Clears the selected stimer interrupt.
//!
//! @param ui32Interrupt is the interrupt to be used.
//!
//! This function will clear the selected interrupts in the STIMER
//! interrupt register.
//!
//! ui32Interrupt should be the logical OR of one or more of the following
//! values:
//!
//!     AM_HAL_STIMER_INT_COMPAREA
//!     AM_HAL_STIMER_INT_COMPAREB
//!     AM_HAL_STIMER_INT_COMPAREC
//!     AM_HAL_STIMER_INT_COMPARED
//!     AM_HAL_STIMER_INT_COMPAREE
//!     AM_HAL_STIMER_INT_COMPAREF
//!     AM_HAL_STIMER_INT_COMPAREG
//!     AM_HAL_STIMER_INT_COMPAREH
//!
//!     AM_HAL_STIMER_INT_OVERFLOW
//!
//!     AM_HAL_STIMER_INT_CAPTUREA
//!     AM_HAL_STIMER_INT_CAPTUREB
//!     AM_HAL_STIMER_INT_CAPTUREC
//!     AM_HAL_STIMER_INT_CAPTURED
//
//*****************************************************************************
extern void     am_hal_stimer_int_clear(uint32_t ui32Interrupt);

//*****************************************************************************
//
//! @brief Returns either the enabled or raw stimer interrupt status.
//!
//! This function will return the stimer interrupt status.
//!
//! @param bEnabledOnly if true returns the status of the enabled interrupts
//! only.
//!
//! The return value will be the logical OR of one or more of the following
//! values:
//!
//! @return Returns the stimer interrupt status.
//
//*****************************************************************************
extern uint32_t am_hal_stimer_int_status_get(bool bEnabledOnly);

//*****************************************************************************
//
//! @brief Check if the compare value can be set without blocking.
//!
//! @param ui32CmprInstance is the compare register instance number (0-7).
//!
//! @return true if delta can be set safely without blocking.
//
//*****************************************************************************
extern bool     am_hal_stimer_check_compare_delta_set(uint32_t ui32CmprInstance);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_STIMER_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

