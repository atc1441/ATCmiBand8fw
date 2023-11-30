//*****************************************************************************
//
//! @file am_hal_sdhc.h
//!
//! @brief Functions for interfacing with the SDHC.
//!
//! @addtogroup sdhc_4b SDHC host controller
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
#ifndef AM_HAL_SDHC_H
#define AM_HAL_SDHC_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! CMSIS-Style macro for handling a variable SDHC module number.
//
//*****************************************************************************
#define SDHCn(n)                     ((SDIO_Type*)(SDIO_BASE + n))

//*****************************************************************************
//
//! Internal macros to support CMSIS-Style macros
//! These should never be used from callers
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Configuration structure for the SDHC.
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief SDHC initialization function
//!
//! @param ui32Module   - module instance.
//! @param ppHandle     - returns the handle for the module instance.
//!
//! This function accepts a module instance, allocates the interface and then
//! returns a handle to be used by the remaining interface functions.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_sdhc_initialize(uint32_t ui32Module,
                                       void **ppHandle);

//*****************************************************************************
//
//! @brief SDHC deinitialization function
//!
//! @param pHandle      - the handle for the module instance.
//!
//! This function accepts a handle to an instance and de-initializes the
//! interface.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_sdhc_deinitialize(void *pHandle);

//*****************************************************************************
//
//! @brief SDHC power control function
//!
//! @param pHandle      - handle for the interface.
//! @param ePowerState  - the desired power state to move the peripheral to.
//! @param bRetainState - flag (if true) to save/restore peripheral state upon
//!                       power state change.
//!
//! This function updates the peripheral to a given power state.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_sdhc_power_control(void *pHandle,
                                          am_hal_sysctrl_power_state_e ePowerState,
                                          bool bRetainState);

//*****************************************************************************
//
//! @brief SDHC setup card host function
//!
//! @param pHandle      - handle for the interface.
//! @param pHost        - pointer to the card host data structure.
//!
//! This function updates pHost related settings by checking the capabilites of underlying
//! SDHC host controller. These settings are important for validate the arguments to the card
//! block read, write, erase, speed, bus width.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_sdhc_setup_host(void *pHandle, am_hal_card_host_t *pHost);

//*****************************************************************************
//
//! @brief SDHC detects the card function
//!
//! @param pHandle      - handle for the interface.
//!
//! This function detects the present of card in the slot.
//!
//! @return status      - boolean.
//
//*****************************************************************************
extern bool am_hal_sdhc_get_cd(void *pHandle);

//*****************************************************************************
//
//! @brief SDHC sets the SDIO bus IO volage
//!
//! @param pHandle      - handle for the interface.
//! @param eBusVoltage    - SDIO Bus Voltage
//!     AM_HAL_HOST_BUS_VOLTAGE_1_8,
//!     AM_HAL_HOST_BUS_VOLTAGE_3_0,
//!     AM_HAL_HOST_BUS_VOLTAGE_3_3,
//!
//! This function sets the bus voltage needed to communiate with the card.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_sdhc_set_bus_voltage(void *pHandle, am_hal_host_bus_voltage_e eBusVoltage);

//*****************************************************************************
//
//! @brief SDHC sets the SDIO bus width
//!
//! @param pHandle      - handle for the interface.
//! @param eBusWidth    - SDIO Bus Width
//!     AM_HAL_HOST_BUS_WIDTH_1 = 1,
//!     AM_HAL_HOST_BUS_WIDTH_4 = 4,
//!     AM_HAL_HOST_BUS_WIDTH_8 = 8,
//!
//! This function sets the bus width needed to communiate with the card.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_sdhc_set_bus_width(void *pHandle, am_hal_host_bus_width_e eBusWidth);

//*****************************************************************************
//
//! @brief SDHC sets the SDIO bus clock speed
//!
//! @param pHandle      - handle for the interface.
//! @param ui32Clock    - Clock Speed
//!
//! This function sets the bus clock speed needed to communiate with the card.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_sdhc_set_bus_clock(void *pHandle, uint32_t ui32Clock);

//*****************************************************************************
//
//! @brief SDHC sets the SDIO UHS Mode
//!
//! @param pHandle      - handle for the interface.
//! @param eUHSMode    - UHS Mode
//!    AM_HAL_HOST_UHS_NONE = 0,
//!    AM_HAL_HOST_UHS_SDR12 = 0,
//!    AM_HAL_HOST_UHS_SDR25 = 1,
//!    AM_HAL_HOST_UHS_SDR50 = 2,
//!    AM_HAL_HOST_UHS_SDR104 = 3,
//!    AM_HAL_HOST_UHS_DDR50 = 4,
//!
//! This function sets the bus clock speed needed to communiate with the card.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_sdhc_set_uhs_mode(void *pHandle, am_hal_host_uhs_mode_e eUHSMode);

//*****************************************************************************
//
//! @brief SDHC checks the SDIO bus busy DAT0 line
//!
//! @param pHandle        - handle for the interface.
//! @param ui32TimeoutMS  - the timeout of checking if this busy line is released (
//!                         changed from low voltage to high voltage).
//!
//! This function checks if DAT0 line is busy or not.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_sdhc_card_busy(void *pHandle, uint32_t ui32TimeoutMS);

extern void am_hal_sdhc_set_txrx_delay(void *pHandle, uint8_t ui8TxRxDelays[2]);

//*****************************************************************************
//
//! @brief SDHC enable function
//!
//! @param pHandle      - the handle for the module instance.
//!
//! This function accepts a handle to an instance and enables the interface.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_sdhc_enable(void *pHandle);

//*****************************************************************************
//
//! @brief SDHC disable function
//!
//! @param pHandle      - the handle for the module instance.
//!
//! This function accepts a handle to an instance and disables the interface.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_sdhc_disable(void *pHandle);

//*****************************************************************************
//
//! @brief SDHC send command function
//!
//! @param pHandle      - handle for the interface.
//! @param pCmd         - pointer to command data structure which including the command index,
//!                       argument, response type, command response and command error code.
//! @param pCmdData     - pointer to command related data structure, like command direction,
//!                       command transfer mode, block size, block count, data buffer, etc.
//!
//! This function sends a command to SD/MMC/eMMC/SDIO card and gets the response. if this command
//! is using synchronous transfer mode, it will be blocked until the data in the buffer has been
//! transmited or received. if this command is using asynchronous transfer mode, it will return immediately
//! after sending the command, data transfer completion done event will be notified by registered callback
//! function in the ISR.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_sdhc_execute_cmd(void *pHandle, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData);

//*****************************************************************************
//
//! @brief SDHC enable interrupts function
//!
//! @param pHandle      - handle for the interface.
//! @param ui32IntMask  - SDHC interrupt mask.
//!
//! This function enables the normal or error specific indicated interrupts.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_sdhc_intr_status_enable(void *pHandle,
                                             uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief SDHC disable interrupts function
//!
//! @param pHandle      - handle for the interface.
//! @param ui32IntMask  - SDHC interrupt mask.
//!
//! This function disables the normal or error specific indicated interrupts.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_sdhc_intr_status_disable(void *pHandle,
                                              uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief SDHC interrupt status function
//!
//! @param pHandle      - handle for the interface.
//! @param pui32Status  - returns the interrupt status value.
//! @param bEnabledOnly - TRUE: only report interrupt status for enalbed ints.
//!                       FALSE: report all interrupt status values.
//!
//! This function returns the normal or error specific indicated interrupt status.
//!
//! @return status      - interrupt status.
//
//*****************************************************************************
extern uint32_t am_hal_sdhc_intr_status_get(void *pHandle,
                                                 uint32_t  *pui32Status,
                                                 bool bEnabledOnly);

//*****************************************************************************
//
//! @brief SDHC interrupt clear
//!
//! @param pHandle        - handle for the interface.
//! @param ui32IntMask    - uint32_t for interrupts to clear
//!
//! This function clears the normal or error interrupts for the given peripheral.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_sdhc_intr_status_clear(void *pHandle,
                                            uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief SDHC interrupt clear
//!
//! @param pHandle        - handle for the interface.
//! @param ui32IntMask    - uint32_t for interrupts to enable
//!
//! This function enables the normal or error interrupts to generate the SDIO IRQ.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_sdhc_intr_signal_enable(void *pHandle, uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief SDHC interrupt clear
//!
//! @param pHandle        - handle for the interface.
//! @param ui32IntMask    - uint32_t for interrupts to clear
//!
//! This function disables the normal or error interrupts to generate the SDIO IRQ.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************

extern uint32_t am_hal_sdhc_intr_signal_disable(void *pHandle, uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief SDHC interrupt service routine
//!
//! @param pHandle       - handle for the interface.
//! @param ui32IntStatus - interrupt status.
//!
//! This function is designed to be called from within the user defined ISR
//! in order to service the non-blocking, PIO, SDMA or ADMA processing for a given
//! module instance.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_sdhc_interrupt_service(void *pHandle,
                                              uint32_t ui32IntStatus);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_MSPI_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

