//*****************************************************************************
//
//! @file am_hal_usb.h
//!
//! @brief Functions for USB module
//!
//! @addtogroup usb_4p USB Functionality
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

#ifndef AM_HAL_USB_H_
#define AM_HAL_USB_H_

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
//
//! @brief This is Used to select the HFRC2 configuration for HighSpeed USB
//! used in function am_hal_usb_setHFRC2 arguments
//
//*****************************************************************************
typedef enum
{
    AM_HAL_USB_HS_CLK_DISABLE,            ///< hfrc2 will be disabled, default off method
    AM_HAL_USB_HS_CLK_DISABLE_HFRC2_ADJ,  ///< hfrc2 will be disabled, not default
    AM_HAL_USB_HS_CLK_HFRC2,              ///< default HFRC2 will be enabled
    AM_HAL_USB_HS_CLK_HFRC2_ADJ,          ///< HFRC2 adjust based on the internal 32Mhz
    AM_HAL_USB_HS_CLK_HFRC2_ADJ_EXTERN_CLK, ///< HFRC2 adjust based on external clock
    AM_HAL_USB_HS_CLK_X32  = 0x7FFFFFF,     ///< force vars derived from this enum to 4 bytes
}
am_hal_usb_hs_clock_type ;

//*****************************************************************************
//
//! @brief Enum for the USB speed type.
//!
//! These macros correspond to the defintions in the USB specification.
//! They may be used with the \e am_hal_usb_set_dev_speed() function.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_USB_SPEED_LOW,
    AM_HAL_USB_SPEED_FULL,
    AM_HAL_USB_SPEED_HIGH,
    AM_HAL_USB_SPEED_UNKNOWN
}
am_hal_usb_dev_speed_e;

//*****************************************************************************
//
//! @brief Enum for the USB device state.
//!
//! These macros partially correspond to the defintions in the USB specification.
//! They may be used with the \e am_hal_usb_set_dev_state() function.
//!
//
//*****************************************************************************
typedef enum
{
    AM_HAL_USB_DEV_STATE_INIT,
    AM_HAL_USB_DEV_STATE_ADDRESSED,
    AM_HAL_USB_DEV_STATE_CONFIGED,
    AM_HAL_USB_DEV_STATE_RESUMING,
    AM_HAL_USB_DEV_STATE_ACTIVE,
    AM_HAL_USB_DEV_STATE_SUSPENDING,
    AM_HAL_USB_DEV_STATE_SUSPENDED
}
am_hal_usb_dev_state_e;

//*****************************************************************************
//
//! @brief Enum for the USB device bus event.
//!
//! These macros correspond to the defintions in the USB specification.
//! They may be used with the \e am_hal_usb_dev_evt_callback callback function
//! provided by upper layer USB stack.
//!
//
//*****************************************************************************
typedef enum
{
    AM_HAL_USB_DEV_EVT_BUS_RESET,
    AM_HAL_USB_DEV_EVT_SOF,
    AM_HAL_USB_DEV_EVT_RESUME,
    AM_HAL_USB_DEV_EVT_SUSPEND,
    AM_HAL_USB_DEV_EVT_NUM
}
am_hal_usb_dev_event_e;

//*****************************************************************************
//
//! @brief Enum for the USB transfer status.
//!
//! These macros is used to indicate how the USB transfer is completed.
//! They may be used with the \e am_hal_usb_ep_xfer_complete_callback callback
//! function provided by upper layer USB stack.
//!
//
//*****************************************************************************
typedef enum
{
    // Transfer is done without error, for ctrl it means status packet done
    USB_XFER_DONE,
    // For control transfer only, data stage is done without error
    USB_XFER_DATA,
    // Endpoint stall is set
    USB_XFER_STALL,
    // Endpoint stall is cleared
    USB_XFER_UNSTALL,
    // Transfer is aborted
    USB_XFER_ABORT,
    // Transfer is aborted because endpoint reset/disable
    USB_XFER_RESET,
    // There was an error
    USB_XFER_ERROR
}
am_hal_usb_xfer_code_e;


//*****************************************************************************
//
//! @brief Enum for the USB control call
//!
//! The macros defined here are used to specify how the void * parameter(s)
//! passed into the control function are interpreted.
//!
//
//*****************************************************************************
typedef enum
{
    //
    //! this is used when the XTAL freq is not the default 32mhz and
    //! high speed USB mode is needed. This will allow the caller to set a
    //! non 32Mhz HF_XTAL freq. (The default value is 32Mhz)
    //
    AM_HAL_CLKGEN_CONTROL_SET_XTAL_FREQ,
    //
    //! this is controller by an argument of type am_hal_usb_hs_clock_type
    //! this is how modules configure the HFRC2
    AM_HAL_CLKGEN_CONTROL_SET_HFRC2_TYPE,


} am_hal_usb_control_e;

//*****************************************************************************
//
//! @brief device event callback function pointer type
//!
//! Upper layer USB stack uses it to define a callback function to receive the
//! the USB bus events and registers it by 'am_hal_usb_register_dev_evt_callback'
//! function.
//
//*****************************************************************************
typedef void (*am_hal_usb_dev_evt_callback)(am_hal_usb_dev_event_e eDevState);

//*****************************************************************************
//
//! @brief ep0 setup request callback function pointer type
//!
//! Upper layer USB stack uses it to define a callback function to receive
//! setup requst from the USB host and registers it by
//! 'am_hal_usb_register_ep0_setup_received_callback' function.
//
//*****************************************************************************
typedef void (*am_hal_usb_ep0_setup_received_callback)(uint8_t *pui8Setup);

//*****************************************************************************
//
//! @brief ctrl/bulk/intr/iso transfer callback function pointer type
//!
//! Upper layer USB stack uses it to define a callback function to receive a
//! confirm for the USB HAL driver about how many bytes has been tranfered and
//! the transfer status.
//
//*****************************************************************************
typedef void (*am_hal_usb_ep_xfer_complete_callback)(uint8_t ui8EpAddr, uint16_t ui16XferLen, am_hal_usb_xfer_code_e eXferCode, void *param);

//*****************************************************************************
//
//! @brief register a USB bus event callback function
//!
//! @param pHandle - handle for the module instance.
//! @param cb      - a USB bus event callback function.
//!
//! This function registers a USB bus event callback function defined in the upper layer
//! USB stack.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_register_dev_evt_callback(void *pHandle, am_hal_usb_dev_evt_callback cb);

//*****************************************************************************
//
//! @brief register a setup requst callback function
//!
//! @param pHandle - handle for the module instance.
//! @param cb      - setup request callback function.
//!
//! This function registers a setup request callback function defined in the upper layer
//! USB stack.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_register_ep0_setup_received_callback(void *pHandle, am_hal_usb_ep0_setup_received_callback cb);

//*****************************************************************************
//
//! @brief register a transfer completion callback function
//!
//! @param pHandle - handle for the module instance.
//! @param cb      - transfer completion callback function.
//!
//! This function registers a bulk/intr/iso transfer completion callback function.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_register_ep_xfer_complete_callback(void *pHandle, am_hal_usb_ep_xfer_complete_callback cb);

//*****************************************************************************
//
//! @brief set the USB device state
//!
//! @param pHandle   - handle for the module instance.
//! @param eDevState - device states like suspended, resuming, etc.
//!
//! This function is used by upper layer USB stack to set the device state.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_set_dev_state(void *pHandle, am_hal_usb_dev_state_e eDevState);

//*****************************************************************************
//
//! @brief start the remote wakeup
//!
//! @param pHandle - handle for the module instance.
//!
//! This function is used by upper layer USB stack to start a remote wakeup action.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_start_remote_wakeup(void *pHandle);

//*****************************************************************************
//
//! @brief stop the remote wakeup
//!
//! @param pHandle - handle for the module instance.
//!
//! This function is used by upper layer USB stack to end a remote wakeup action.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_end_remote_wakeup(void *pHandle);

//*****************************************************************************
//
//! @brief set the USB device address
//!
//! @param pHandle   - handle for the module instance.
//! @param ui8EpAddr - set the USB device address
//!
//! This function is used by upper layer USB stack to set the device address allocated
//! by the USB host during the enumeration.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_set_addr(void *pHandle, uint8_t ui8EpAddr);

//*****************************************************************************
//
//! @brief soft connect to the USB host
//!
//! @param pHandle - handle for the module instance.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_attach(void *pHandle);

//*****************************************************************************
//
//! @brief soft disconnect to the USB host
//!
//! @param pHandle - handle for the module instance.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_detach(void *pHandle);

//*****************************************************************************
//
//! @brief get the USB frame number
//!
//! @param pHandle - handle for the module instance.
//!
//! This function is used by upper layer USB stack to get the USB frame number.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_get_frame_number(void *pHandle);

//*****************************************************************************
//
//! @brief enable the SOF interrupt
//!
//! @param pHandle - handle for the module instance.
//!
//! This function is used by upper layer USB stack to enable the SOF interrupt.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_enable_sof_intr(void *pHandle);

//*****************************************************************************
//
//! @brief disable the SOF interrupt
//!
//! @param pHandle   - handle for the module instance.
//!
//! This function is used by upper layer USB stack to disable the SOF interrupt.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_disable_sof_intr(void *pHandle);

#define AM_HAL_USB_GET_HW_INFO_ENABLED
#ifdef AM_HAL_USB_GET_HW_INFO_ENABLED

//*****************************************************************************
//
//! @brief Macro definitions for the USB hardware information.
//!
//! These macros correspond to the definitions in the datasheet of
//! apollo4 USB peripheral.
//! They may be used with the \e am_hal_usb_get_hw_infor function.
//!
//
//*****************************************************************************
typedef struct
{
    // the major version number
    uint8_t  ui8Major;
    // the minor version number
    uint16_t ui16Minor;
    // the number of OUT endpoints
    uint8_t  ui8OutEpNum;
    // the number of IN endpoints
    uint8_t  ui8InEpNum;
    // the width of RAM bus address
    uint8_t  ui8RamBits;
} am_hal_usb_hw_info;

//*****************************************************************************
//
//! @brief get the hardware information
//!
//! @param pHandle - handle for the module instance.
//! @param sHWInfo - the information about the USB device hardware configuration.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_get_hw_infor(void *pHandle, am_hal_usb_hw_info *sHWInfo);

#endif

#define AM_HAL_USB_TEST_MODE_ENABLED
#ifdef AM_HAL_USB_TEST_MODE_ENABLED

//*****************************************************************************
//
//! @brief Macro definitions for the USB devie test mode.
//!
//! These macros correspond to the defintions in the USB specification.
//! They may be used with the \e am_hal_usb_test_mode function.
//!
//
//*****************************************************************************
typedef enum
{
    AM_HAL_USB_TEST_SE0_NAK,
    AM_HAL_USB_TEST_J,
    AM_HAL_USB_TEST_K,
    AM_HAL_USB_TEST_PACKET
}
am_hal_usb_test_mode_e;

//
// Upper layer USB stack should call below functions
// when receive TEST_MODE request
//
//*****************************************************************************
//
//! @brief set the USB test mode flag
//!
//! @param pHandle - handle for the module instance.
//!
//! set a flag in the handle to indicate the current USB device in the test mode.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_enter_test_mode(const void *pHandle);

//*****************************************************************************
//
//! @brief do the USB test
//!
//! @param pHandle   - handle for the module instance.
//! @param eTestMode - one of the test mode like TEST_SE0_NAK, TEST_J, TEST_K and TEST_PACKET.
//!
//! This function is used by upper layer USB stack to order the USB device controller
//! to send the specific USB packet or signals to verify the timing and signal quaility.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_test_mode(const void *pHandle, const am_hal_usb_test_mode_e eTestMode);
#endif

//*****************************************************************************
//
//! @brief initialize the endpoint
//!
//! @param pHandle   - handle for the module instance.
//! @param ui8EpAddr - endpoint address including the endpoint direction
//! @param ui8EpAttr - the type of endpoint (bulk/intr/iso/control)
//! @param ui16MaxPacket - the max packet length of the endpoint
//!
//! This function is used by upper layer USB stack to initialize the endpoints
//! when USB host sets the configuration of the USB device.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_ep_init(void *pHandle, uint8_t ui8EpAddr, uint8_t ui8EpAttr, uint16_t ui16MaxPacket);

//*****************************************************************************
//
//! @brief stall the endpoint
//!
//! @param pHandle   - handle for the module instance.
//! @param ui8EpAddr - endpoint address including the endpoint direction
//!
//! This function is used by upper layer USB stack to stall the endpoint when
//! some unknown requests can't be handled or transfer can't continue.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_ep_stall(void *pHandle, uint8_t ui8EpAddr);

//*****************************************************************************
//
//! @brief clear the endpoint stall
//!
//! @param pHandle   - handle for the module instance.
//! @param ui8EpAddr - endpoint address including the endpoint direction
//!
//! This function is used by upper layer USB stack to clear the endpoint stall
//! when USB device stack can recovry from some error status or USB host sends
//! the clear feature request to clear the stall of the endpoint.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_ep_clear_stall(void *pHandle, uint8_t ui8EpAddr);

//*****************************************************************************
//
//! @brief submit a USB transfer
//!
//! @param pHandle   - handle for the module instance.
//! @param ui8EpAddr - endpoint address including the endpoint direction
//! @param pui8Buf   - the buffer for receiving the data from USB host or
//!                    sending data to the USB host.
//! @param ui16Len   - the length of the buffer.
//!
//! This function is used by the upper layer USB stack to submit a transfer requst
//! to the USB device controller.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_ep_xfer(void *pHandle, uint8_t ui8EpAddr, uint8_t *pui8Buf, uint16_t ui16Len);

//*****************************************************************************
//
//! @brief get the current USB speed
//!
//! @param pHandle - handle for the module instance.
//!
//! This function is used by the upper layer USB stack to get the current USB
//! speed type
//! like full-speed or high-speed.
//!
//! @return one of am_hal_usb_dev_speed_e like
//!    AM_HAL_USB_SPEED_FULL
//!    AM_HAL_USB_SPEED_HIGH, etc.
//
//*****************************************************************************
extern am_hal_usb_dev_speed_e am_hal_get_usb_dev_speed(void *pHandle);

//*****************************************************************************
//
//! @brief set the USB speed
//!
//! @param pHandle - handle for the module instance.
//! @param eSpeed  - speed type of the USB device
//!
//! This function is used by the upper layer USB stack to force USB device controller
//! to run under a certain speed, for example full-speed by ignoring the controller
//! high-speed capability.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_set_dev_speed(void *pHandle, am_hal_usb_dev_speed_e eSpeed);

//*****************************************************************************
//
//! @brief initialize the USB device controller module
//!
//! @param ui32Module - the index to the USB module
//! @param ppHandle   - the handle of initialized USB instance
//!
//! This function should be called firstly before we use any other USB HAL driver
//! functions.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_initialize(uint32_t ui32Module, void **ppHandle);

//*****************************************************************************
//
//! @brief uninitialize the USB device controller module
//!
//! @param pHandle - the handle of initialized USB instance
//!
//! This function should be called finally after we want to shutdown the USB
//! device controller.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_deinitialize(void *pHandle);

//*****************************************************************************
//
//! @brief power on/off the USB device controller
//!
//! @param pHandle     - the handle of initialized USB instance
//! @param ePowerState  - the power state of USB device controller
//! @param bRetainState - wether retain the USB registers or not
//!
//! This function should be called after USB device controller has been initalized
//! successfully by 'am_hal_usb_initialize'.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_power_control(void *pHandle, am_hal_sysctrl_power_state_e ePowerState, bool bRetainState);

//*****************************************************************************
//
//! @brief enable the IN endpoints' interrupt
//!
//! @param pHandle     - the handle of initialized USB instance
//! @param ui32IntMask  - the bit mask of IN endpoints' interrupt
//!
//! This function is used to enable the IN endpoints' interrupt.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_intr_ep_in_enable(void *pHandle, uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief disable the IN endpoints' interrupt
//!
//! @param pHandle     - the handle of initialized USB instance
//! @param ui32IntMask  - the bit mask of IN endpoints' interrupt
//!
//! This function is used to disable the IN endpoints' interrupt.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_intr_ep_in_disable(void *pHandle, uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief clear the IN endpoints' interrupt status
//!
//! @param pHandle - the handle of initialized USB instance
//!
//! This function is used to clear the IN endpoints' interrupt status.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_intr_ep_in_clear(void *pHandle);

//*****************************************************************************
//
//! @brief get the IN endpoints' interrupt status
//!
//! @param pHandle      - the handle of initialized USB instance
//! @param ui32IntStatus - the IN endpoints' interrupt status
//! @param bEnabledOnly   - only check enabled IN endpoint interrupt or not
//!
//! This function is used to get IN endpoints' interrupt status.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_intr_ep_in_status_get(void *pHandle, uint32_t *ui32IntStatus, bool bEnabledOnly);

//*****************************************************************************
//
//! @brief enable the OUT endpoints' interrupt
//!
//! @param pHandle    - the handle of initialized USB instance
//! @param ui32IntMask - the bit mask of OUT endpoints' interrupt
//!
//! This function is used to enable the OUT endpoints' interrupt.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_intr_ep_out_enable(void *pHandle, uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief disable the endpoints' interrupt
//!
//! @param pHandle    - the handle of initialized USB instance
//! @param ui32IntMask - the bit mask of OUT endpoints' interrupt
//!
//! This function is used to disable the OUT endpoints' interrupt.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_intr_ep_out_disable(void *pHandle, uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief clear the OUT endpoints' interrupt status
//!
//! @param pHandle - the handle of initialized USB instance
//!
//! This function is used to clear the OUT endpoints' interrupt.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_intr_ep_out_clear(void *pHandle);

//*****************************************************************************
//
//! @brief get the OUT endpoints' interrupt status
//!
//! @param pHandle      - the handle of initialized USB instance
//! @param pui32IntStatus - pointer to the OUT endpoints' interrupt status
//! @param bEnabledOnly   - only check enabled OUT endpoint interrupt or not
//!
//! This function is used to get the OUT endpoints' interrupt status.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_intr_ep_out_status_get(void *pHandle, uint32_t *pui32IntStatus, bool bEnabledOnly);

//*****************************************************************************
//
//! @brief enable the USB bus's interrupts
//!
//! @param pHandle    - the handle of initialized USB instance
//! @param ui32IntMask - the bit mask of enabled USB bus interrupts
//!
//! This function is used to enable the USB bus interrupts.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_intr_usb_enable(void *pHandle, uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief disable the USB bus's interrupts
//!
//! @param pHandle    - the handle of initialized USB instance
//! @param ui32IntMask - the bit mask of USB bus interrupts.
//!
//! This function is used to disable the USB bus interrupts.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_intr_usb_disable(void *pHandle, uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief clear the USB bus interrupts
//!
//! @param pHandle - the handle of initialized USB instance
//!
//! This function is used to clear the USB bus interrupt status.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_intr_usb_clear(void *pHandle);

//*****************************************************************************
//
//! @brief get the USB bus interrupt status
//!
//! This function is used to get the USB bus interrupt status.
//!
//! @param pHandle      - the handle of initialized USB instance
//! @param ui32IntStatus - the USB bus interrupt status
//! @param bEnabledOnly   - only check enabled USB bus interrupt or not
//!
//! This function is used to get the USB bus interrupt status.
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_intr_usb_status_get(void *pHandle, uint32_t *ui32IntStatus, bool bEnabledOnly);

//*****************************************************************************
//
//! @brief Enum for the USB device bus event mask bit.
//!
//! This Enum are just some convenient utility for easing the programming.
//
//*****************************************************************************
enum
{
    USB_INTRUSB_Suspend_Msk = 0x1,
    USB_INTRUSB_Resume_Msk  = 0x2,
    USB_INTRUSB_Reset_Msk   = 0x4,
    USB_INTRUSB_SOF_Msk     = 0x8,
    USB_INTRIN_EP0_Msk      = 0x1,
};

//*****************************************************************************
//
//! @brief get all USB related interrupt status
//!
//! This function is used to get all USB related interrupt status.
//!
//! @param pHandle          - the handle of initialized USB instance
//! @param ui32IntrUsbStatus - the USB bus interrupt status
//! @param ui32IntrInStatus  - the USB IN endpoint interrupt status
//! @param ui32IntrOutStatus - the USB OUT endpoint interrupt status
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_intr_status_get(void *pHandle, uint32_t *ui32IntrUsbStatus, uint32_t *ui32IntrInStatus, uint32_t *ui32IntrOutStatus);

//*****************************************************************************
//
//! @brief USB interrupt service routine
//!
//! This function is USB interrupt service routine function.
//!
//! @param pHandle - the handle of initialized USB instance
//! @param ui32IntrUsbStatus - the USB bus interrupt status
//! @param ui32IntrInStatus  - the USB IN endpoint interrupt status
//! @param ui32IntrOutStatus - the USB OUT endpoint interrupt status
//!
//
//*****************************************************************************
extern void am_hal_usb_interrupt_service(void *pHandle,
                                         uint32_t ui32IntrUsbStatus,
                                         uint32_t ui32IntrInStatus,
                                         uint32_t ui32IntrOutStatus);



//*****************************************************************************
//
//! @brief Apply various specific commands / controls to the USB module
//!
//! @param eControl control enum
//! @param pArgs    data used. This differs for each enum
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_control(am_hal_usb_control_e eControl, void *pArgs);

//*****************************************************************************
//
//! @brief set and enable HFRC2 FLL for 24Mhz speed usb
//!
//! @note this is used for HIGH-SPEED usb
//!
//! @param tUsbHsClockType - enable or disable the HFRC2
//!
//! @return one of am_hal_status_e like AM_HAL_STATUS_SUCCESS
//
//*****************************************************************************
extern uint32_t am_hal_usb_setHFRC2(am_hal_usb_hs_clock_type tUsbHsClockType);


#ifdef __cplusplus
}
#endif

#endif /* AM_HAL_USB_H_ */

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

