//*****************************************************************************
//
//! @file am_bsp.c
//!
//! @brief Top level functions for performing board initialization.
//!
//! @addtogroup apollo4l_blue_evb_bsp BSP for the Apollo4l blue EVB
//! @ingroup BSP
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

#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
//
// Configuration.
//
//*****************************************************************************
#define AM_BSP_ITM_FREQUENCY     AM_HAL_TPIU_BAUD_1M

//*****************************************************************************
//
// LEDs
//
//*****************************************************************************
#if AM_BSP_NUM_LEDS
am_devices_led_t am_bsp_psLEDs[AM_BSP_NUM_LEDS] =
{
    {AM_BSP_GPIO_LED0, AM_DEVICES_LED_ON_LOW | AM_DEVICES_LED_POL_OPEN_DRAIN},
    {AM_BSP_GPIO_LED1, AM_DEVICES_LED_ON_LOW | AM_DEVICES_LED_POL_OPEN_DRAIN},
    {AM_BSP_GPIO_LED2, AM_DEVICES_LED_ON_LOW | AM_DEVICES_LED_POL_OPEN_DRAIN}
};
#endif

//*****************************************************************************
//
// Buttons.
//
//*****************************************************************************
#if AM_BSP_NUM_BUTTONS
am_devices_button_t am_bsp_psButtons[AM_BSP_NUM_BUTTONS] =
{
    AM_DEVICES_BUTTON(AM_BSP_GPIO_BUTTON0, AM_DEVICES_BUTTON_NORMAL_HIGH),
    AM_DEVICES_BUTTON(AM_BSP_GPIO_BUTTON1, AM_DEVICES_BUTTON_NORMAL_HIGH),
};


am_devices_button_pin_cfg_t am_bsp_psButtonsInfo[AM_BSP_NUM_BUTTONS] =
{
    {
        .sDevButton.ui32GPIONumber = AM_BSP_GPIO_BUTTON0,
        .sDevButton.ui32Polarity = AM_DEVICES_BUTTON_NORMAL_HIGH,
        .tPinCfg = &g_AM_BSP_GPIO_BUTTON0
    },
    {
       .sDevButton.ui32GPIONumber = AM_BSP_GPIO_BUTTON1,
       .sDevButton.ui32Polarity = AM_DEVICES_BUTTON_NORMAL_HIGH,
       .tPinCfg = &g_AM_BSP_GPIO_BUTTON1
    },

};

#endif // AM_BSP_NUM_BUTTONS

//*****************************************************************************
//
// Print interface tracking variable.
//
//*****************************************************************************
typedef enum
{
    AM_BSP_PRINT_IF_NONE,
    AM_BSP_PRINT_IF_SWO,
    AM_BSP_PRINT_IF_UART,
    AM_BSP_PRINT_IF_BUFFERED_UART,
    AM_BSP_PRINT_IF_MEMORY,
}
am_bsp_print_interface_e;

static am_bsp_print_interface_e g_ePrintInterface = AM_BSP_PRINT_IF_NONE;

//*****************************************************************************
//
// Default UART configuration settings.
//
//*****************************************************************************
static void *g_sCOMUART;

static const am_hal_uart_config_t g_sBspUartConfig =
{
    //
    // Standard UART settings: 115200-8-N-1
    //
    .ui32BaudRate = 115200,
    .eDataBits = AM_HAL_UART_DATA_BITS_8,
    .eParity = AM_HAL_UART_PARITY_NONE,
    .eStopBits = AM_HAL_UART_ONE_STOP_BIT,
    .eFlowControl = AM_HAL_UART_FLOW_CTRL_NONE,

    //
    // Set TX and RX FIFOs to interrupt at half-full.
    //
    .eTXFifoLevel = AM_HAL_UART_FIFO_LEVEL_16,
    .eRXFifoLevel = AM_HAL_UART_FIFO_LEVEL_16,
};
#ifndef AM_BSP_DISABLE_BUFFERED_UART
//*****************************************************************************
//
// Default UART configuration settings if using buffers.
//
//*****************************************************************************
#define AM_BSP_UART_BUFFER_SIZE     1024
static uint8_t pui8UartTxBuffer[AM_BSP_UART_BUFFER_SIZE];
static uint8_t pui8UartRxBuffer[AM_BSP_UART_BUFFER_SIZE];
#endif // AM_BSP_DISABLE_BUFFERED_UART


//*****************************************************************************
//
// External Power Switch.
//
//*****************************************************************************
void am_bsp_external_pwr_on(void) {}

//*****************************************************************************
//
// This function enables several power-saving features of the MCU, and
// disables some of the less-frequently used peripherals. It also sets the
// system clock to 24 MHz.
//
//*****************************************************************************
void
am_bsp_low_power_init(void)
{
    //
    // Disable the ITM interface.
    //
    //am_bsp_itm_printf_disable();

    //
    // Initialize for low power in the power control block
    //
    am_hal_pwrctrl_low_power_init();

#if AM_BSP_ENABLE_SIMOBUCK
    //
    // Enable SIMOBUCK for this board
    //
    am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_SIMOBUCK_INIT, 0);
#endif

    //
    // Disable the RTC.
    //
    am_hal_rtc_osc_disable();

} // am_bsp_low_power_init()

//*****************************************************************************
//
// This function enables TPIU registers for debug printf messages and enables
// ITM GPIO pin to SWO mode. This function should be called after reset and
// after waking up from deep sleep.
//
//*****************************************************************************
int32_t
am_bsp_debug_printf_enable(void)
{
    switch (g_ePrintInterface)
    {
        case AM_BSP_PRINT_IF_NONE:
            // Fall on through to default to SWO
        case AM_BSP_PRINT_IF_SWO:
            return am_bsp_itm_printf_enable();

        case AM_BSP_PRINT_IF_UART:
            return am_bsp_uart_printf_enable();

        case AM_BSP_PRINT_IF_BUFFERED_UART:
            return am_bsp_buffered_uart_printf_enable();

        default:
            break;
    }

    return -1;

} // am_bsp_debug_printf_enable()

//*****************************************************************************
//
// This function disables TPIU registers for debug printf messages and disables
// ITM GPIO pin to SWO mode. This function should be called after reset and
// after waking up from deep sleep.
//
//*****************************************************************************
void
am_bsp_debug_printf_disable(void)
{
    switch (g_ePrintInterface)
    {
        case AM_BSP_PRINT_IF_SWO:
            am_bsp_itm_printf_disable();
            break;

        case AM_BSP_PRINT_IF_UART:
            am_bsp_uart_printf_disable();
            break;

        case AM_BSP_PRINT_IF_BUFFERED_UART:
            am_bsp_buffered_uart_printf_disable();
            break;


        default:
            break;
    }
} // am_bsp_debug_printf_disable()

//*****************************************************************************
//
// Enable printing over ITM.
//
//*****************************************************************************
int32_t
am_bsp_itm_printf_enable(void)
{
    uint32_t ui32dcuVal;
    int32_t i32RetValue = 0;
    bool bOffCryptoOnExit = false;

    //
    // Need to make sure that SWO is enabled
    //
    {
        if (PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO == 0)
        {
            bOffCryptoOnExit = true;
            am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);
        }

        if ((PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO == 1) && (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 1))
        {
            am_hal_dcu_get(&ui32dcuVal);
            // Enable SWO
            if ( !(ui32dcuVal & AM_HAL_DCU_CPUTRC_TPIU_SWO) &&
                 (am_hal_dcu_update(true, AM_HAL_DCU_CPUTRC_TPIU_SWO) != AM_HAL_STATUS_SUCCESS) )
            {
                // Cannot enable SWO
                i32RetValue = -1;
            }
        }
        else
        {
            // If DCU is not accessible, we cannot determine if ITM can be safely enabled.
            i32RetValue = -1;
        }
    }

    if (i32RetValue == 0)
    {
        //
        // Set the global print interface.
        //
        g_ePrintInterface = AM_BSP_PRINT_IF_SWO;

        //
        // Enable the ITM interface and the SWO pin.
        //
        am_hal_itm_enable();

        am_hal_tpiu_enable(AM_BSP_ITM_FREQUENCY);

        am_hal_gpio_pinconfig(AM_BSP_GPIO_ITM_SWO, g_AM_BSP_GPIO_ITM_SWO);

        //
        // Attach the ITM to the STDIO driver.
        //
        am_util_stdio_printf_init(am_hal_itm_print);
    }

    if (bOffCryptoOnExit == true)
    {
        am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);
    }

    return i32RetValue;
} // am_bsp_itm_printf_enable()

//*****************************************************************************
//
// Disable printing over ITM.
//
//*****************************************************************************
void
am_bsp_itm_printf_disable(void)
{
    if (g_ePrintInterface != AM_BSP_PRINT_IF_SWO)
    {
        return;
    }

    //
    // Disable the ITM/TPIU
    //
    am_hal_itm_disable();

    //
    // Detach the ITM interface from the STDIO driver.
    //
    am_util_stdio_printf_init(0);

    //
    // Disconnect the SWO pin
    //
    am_hal_gpio_pinconfig(AM_BSP_GPIO_ITM_SWO, am_hal_gpio_pincfg_disabled);
    g_ePrintInterface = AM_BSP_PRINT_IF_NONE;
} // am_bsp_itm_printf_disable()

//*****************************************************************************
//
// This function configures SDIO's CMD, CLK, DAT0-7 pins
//
//*****************************************************************************
void
am_bsp_sdio_pins_enable(uint8_t ui8BusWidth)
{
    am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_CMD,  g_AM_BSP_GPIO_SDIF_CMD);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_CLKOUT,  g_AM_BSP_GPIO_SDIF_CLKOUT);

    switch (ui8BusWidth)
    {
        case AM_HAL_HOST_BUS_WIDTH_8:
            am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_DAT4, g_AM_BSP_GPIO_SDIF_DAT4);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_DAT5, g_AM_BSP_GPIO_SDIF_DAT5);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_DAT6, g_AM_BSP_GPIO_SDIF_DAT6);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_DAT7, g_AM_BSP_GPIO_SDIF_DAT7);
        case AM_HAL_HOST_BUS_WIDTH_4:
            am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_DAT1, g_AM_BSP_GPIO_SDIF_DAT1);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_DAT2, g_AM_BSP_GPIO_SDIF_DAT2);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_DAT3, g_AM_BSP_GPIO_SDIF_DAT3);
        case AM_HAL_HOST_BUS_WIDTH_1:
            am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_DAT0, g_AM_BSP_GPIO_SDIF_DAT0);
            break;
    }

} // am_bsp_sdio_pins_enable()

//*****************************************************************************
//
// This function disables SDIO's CMD, CLK, DAT0-7 pins
//
//*****************************************************************************
void
am_bsp_sdio_pins_disable(uint8_t ui8BusWidth)
{
    am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_CMD,  am_hal_gpio_pincfg_default);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_CLKOUT,  am_hal_gpio_pincfg_default);

    switch (ui8BusWidth)
    {
        case AM_HAL_HOST_BUS_WIDTH_8:
            am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_DAT4, am_hal_gpio_pincfg_default);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_DAT5, am_hal_gpio_pincfg_default);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_DAT6, am_hal_gpio_pincfg_default);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_DAT7, am_hal_gpio_pincfg_default);
        case AM_HAL_HOST_BUS_WIDTH_4:
            am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_DAT1, am_hal_gpio_pincfg_default);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_DAT2, am_hal_gpio_pincfg_default);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_DAT3, am_hal_gpio_pincfg_default);
        case AM_HAL_HOST_BUS_WIDTH_1:
            am_hal_gpio_pinconfig(AM_BSP_GPIO_SDIF_DAT0, am_hal_gpio_pincfg_default);
            break;
    }
} // am_bsp_sdio_pins_disable()

//*****************************************************************************
//
// Initialize and configure the UART
//
//*****************************************************************************
int32_t
am_bsp_uart_printf_enable(void)
{
    //
    // Initialize, power up, and configure the communication UART. Use the
    // custom configuration if it was provided. Otherwise, just use the default
    // configuration.
    //
    if (am_hal_uart_initialize(AM_BSP_UART_PRINT_INST, &g_sCOMUART) ||
        am_hal_uart_power_control(g_sCOMUART, AM_HAL_SYSCTRL_WAKE, false) ||
        am_hal_uart_configure(g_sCOMUART, &g_sBspUartConfig) ||

        //
        // Enable the UART pins.
        //
        am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_TX, g_AM_BSP_GPIO_COM_UART_TX) ||
        am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_RX, g_AM_BSP_GPIO_COM_UART_RX) )
    {
        return -1;
    }

    //
    // Save the information that we're using the UART for printing.
    //
    g_ePrintInterface = AM_BSP_PRINT_IF_UART;

    //
    // Register the BSP print function to the STDIO driver.
    //
    am_util_stdio_printf_init(am_bsp_uart_string_print);

    return 0;

} // am_bsp_uart_printf_enable()

//*****************************************************************************
//
// Disable the UART
//
//*****************************************************************************
void
am_bsp_uart_printf_disable(void)
{
    if (g_ePrintInterface != AM_BSP_PRINT_IF_UART)
    {
        return;
    }

    //
    // Make sure the UART has finished sending everything it's going to send.
    //
    am_hal_uart_tx_flush(g_sCOMUART);

    //
    // Detach the UART from the stdio driver.
    //
    am_util_stdio_printf_init(0);

    //
    // Power down the UART, and surrender the handle.
    //
    am_hal_uart_power_control(g_sCOMUART, AM_HAL_SYSCTRL_DEEPSLEEP, false);
    am_hal_uart_deinitialize(g_sCOMUART);

    //
    // Disable the UART pins.
    //
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_TX, am_hal_gpio_pincfg_disabled);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_RX, am_hal_gpio_pincfg_disabled);
    g_ePrintInterface = AM_BSP_PRINT_IF_NONE;
} // am_bsp_uart_printf_disable()

//*****************************************************************************
//
// This function is used for printing a string via the UART, which for some
// MCU devices may be multi-module.
//
//*****************************************************************************
void
am_bsp_uart_string_print(char *pcString)
{
    uint32_t ui32StrLen = 0;
    uint32_t ui32BytesWritten = 0;

    //
    // Measure the length of the string.
    //
    while (pcString[ui32StrLen] != 0)
    {
        ui32StrLen++;
    }

    //
    // Print the string via the UART.
    //
    const am_hal_uart_transfer_t sUartWrite =
    {
        .eType = AM_HAL_UART_BLOCKING_WRITE,
        .pui8Data = (uint8_t *) pcString,
        .ui32NumBytes = ui32StrLen,
        .ui32TimeoutMs = AM_HAL_UART_WAIT_FOREVER,
        .pui32BytesTransferred = &ui32BytesWritten,
    };

    am_hal_uart_transfer(g_sCOMUART, &sUartWrite);

    if (ui32BytesWritten != ui32StrLen)
    {
        //
        // Couldn't send the whole string!!
        //
        while (1);
    }
} // am_bsp_uart_string_print()

#ifndef AM_BSP_DISABLE_BUFFERED_UART
//*****************************************************************************
//
// Initialize and configure the UART
//
//*****************************************************************************
int32_t
am_bsp_buffered_uart_printf_enable(void)
{
    //
    // Initialize, power up, and configure the communication UART. Use the
    // custom configuration if it was provided. Otherwise, just use the default
    // configuration.
    //
    if (am_hal_uart_initialize(AM_BSP_UART_PRINT_INST, &g_sCOMUART) ||
        am_hal_uart_power_control(g_sCOMUART, AM_HAL_SYSCTRL_WAKE, false) ||
        am_hal_uart_configure(g_sCOMUART, &g_sBspUartConfig) ||
        am_hal_uart_buffer_configure(g_sCOMUART, pui8UartTxBuffer, sizeof(pui8UartTxBuffer), pui8UartRxBuffer, sizeof(pui8UartRxBuffer)) ||

        //
        // Enable the UART pins.
        //
        am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_TX, g_AM_BSP_GPIO_COM_UART_TX) ||
        am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_RX, g_AM_BSP_GPIO_COM_UART_RX) )
    {
        return -1;
    }

    //
    // Register the BSP print function to the STDIO driver.
    //
    am_util_stdio_printf_init(am_bsp_uart_string_print);

    //
    // Enable the interrupts for the UART.
    //
    NVIC_SetPriority((IRQn_Type)(UART0_IRQn + AM_BSP_UART_PRINT_INST), AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + AM_BSP_UART_PRINT_INST));

    //
    // Set the global print interface.
    //
    g_ePrintInterface = AM_BSP_PRINT_IF_BUFFERED_UART;

    return 0;

} // am_bsp_buffered_uart_printf_enable()

//*****************************************************************************
//
// Disable the UART
//
//*****************************************************************************
void
am_bsp_buffered_uart_printf_disable(void)
{
    if (g_ePrintInterface != AM_BSP_PRINT_IF_BUFFERED_UART)
    {
        return;
    }

    //
    // Make sure the UART has finished sending everything it's going to send.
    //
    am_hal_uart_tx_flush(g_sCOMUART);

    //
    // Detach the UART from the stdio driver.
    //
    am_util_stdio_printf_init(0);

    //
    // Power down the UART, and surrender the handle.
    //
    am_hal_uart_power_control(g_sCOMUART, AM_HAL_SYSCTRL_DEEPSLEEP, false);
    am_hal_uart_deinitialize(g_sCOMUART);

    //
    // Disable the UART pins.
    //
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_TX, am_hal_gpio_pincfg_disabled);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_RX, am_hal_gpio_pincfg_disabled);

    //
    // Disable the interrupts for the UART.
    //
    NVIC_DisableIRQ((IRQn_Type)(UART0_IRQn + AM_BSP_UART_PRINT_INST));
    g_ePrintInterface = AM_BSP_PRINT_IF_NONE;
} // am_bsp_buffered_uart_printf_disable()

//*****************************************************************************
//
// Interrupt routine for the buffered UART interface.
//
//*****************************************************************************
void
am_bsp_buffered_uart_service(void)
{
    uint32_t ui32Status;
    am_hal_uart_interrupt_status_get(g_sCOMUART, &ui32Status, true);
    am_hal_uart_interrupt_clear(g_sCOMUART, ui32Status);
    am_hal_uart_interrupt_service(g_sCOMUART, ui32Status);
} // am_bsp_buffered_uart_service()
#endif // AM_BSP_DISABLE_BUFFERED_UART

//*****************************************************************************
//
// This function configures up to 10-pins for MSPI serial, dual, quad,
// dual-quad, and octal operation.
//
//*****************************************************************************
void
am_bsp_iom_pins_enable(uint32_t ui32Module, am_hal_iom_mode_e eIOMMode)
{
    uint32_t ui32Combined;

    //
    // Validate parameters
    //
    if ( ui32Module >= AM_REG_IOM_NUM_MODULES )
    {
        return;
    }

    ui32Combined = ((ui32Module << 2) | eIOMMode);

    switch ( ui32Combined )
    {
#if defined(AM_BSP_GPIO_IOM0_SCK)
        case ((0 << 2) | AM_HAL_IOM_SPI_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_SCK,  g_AM_BSP_GPIO_IOM0_SCK);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_MISO, g_AM_BSP_GPIO_IOM0_MISO);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_MOSI, g_AM_BSP_GPIO_IOM0_MOSI);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_CS,   g_AM_BSP_GPIO_IOM0_CS);
            break;
#endif /* AM_BSP_GPIO_IOM0_SCK */

#if defined(AM_BSP_GPIO_IOM1_SCK)
        case ((1 << 2) | AM_HAL_IOM_SPI_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SCK,  g_AM_BSP_GPIO_IOM1_SCK);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_MISO, g_AM_BSP_GPIO_IOM1_MISO);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_MOSI, g_AM_BSP_GPIO_IOM1_MOSI);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_CS,   g_AM_BSP_GPIO_IOM1_CS);
            break;
#endif /* AM_BSP_GPIO_IOM1_SCK */

#if defined(AM_BSP_GPIO_IOM2_SCK)
        case ((2 << 2) | AM_HAL_IOM_SPI_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_SCK,  g_AM_BSP_GPIO_IOM2_SCK);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_MISO, g_AM_BSP_GPIO_IOM2_MISO);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_MOSI, g_AM_BSP_GPIO_IOM2_MOSI);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_CS,   g_AM_BSP_GPIO_IOM2_CS);
            break;
#endif /* AM_BSP_GPIO_IOM2_SCK */

#if defined(AM_BSP_GPIO_IOM3_SCK)
        case ((3 << 2) | AM_HAL_IOM_SPI_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_SCK,  g_AM_BSP_GPIO_IOM3_SCK);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_MISO, g_AM_BSP_GPIO_IOM3_MISO);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_MOSI, g_AM_BSP_GPIO_IOM3_MOSI);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_CS,   g_AM_BSP_GPIO_IOM3_CS);
            break;
#endif /* AM_BSP_GPIO_IOM3_SCK */

#if defined(AM_BSP_GPIO_IOM4_SCK)
        case ((4 << 2) | AM_HAL_IOM_SPI_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_SCK,  g_AM_BSP_GPIO_IOM4_SCK);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_MISO, g_AM_BSP_GPIO_IOM4_MISO);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_MOSI, g_AM_BSP_GPIO_IOM4_MOSI);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_CS,   g_AM_BSP_GPIO_IOM4_CS);
            break;
#endif /* AM_BSP_GPIO_IOM4_SCK */

#if defined(AM_BSP_GPIO_IOM5_SCK)
        case ((5 << 2) | AM_HAL_IOM_SPI_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM5_SCK,  g_AM_BSP_GPIO_IOM5_SCK);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM5_MISO, g_AM_BSP_GPIO_IOM5_MISO);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM5_MOSI, g_AM_BSP_GPIO_IOM5_MOSI);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM5_CS,   g_AM_BSP_GPIO_IOM5_CS);
            break;
#endif /* AM_BSP_GPIO_IOM5_SCK */

#if defined(AM_BSP_GPIO_IOM6_SCK)
        case ((6 << 2) | AM_HAL_IOM_SPI_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM6_SCK,  g_AM_BSP_GPIO_IOM6_SCK);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM6_MISO, g_AM_BSP_GPIO_IOM6_MISO);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM6_MOSI, g_AM_BSP_GPIO_IOM6_MOSI);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM6_CS,   g_AM_BSP_GPIO_IOM6_CS);
            break;
#endif /* AM_BSP_GPIO_IOM6_SCK */

#if defined(AM_BSP_GPIO_IOM7_SCK)
        case ((7 << 2) | AM_HAL_IOM_SPI_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM7_SCK,  g_AM_BSP_GPIO_IOM7_SCK);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM7_MISO, g_AM_BSP_GPIO_IOM7_MISO);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM7_MOSI, g_AM_BSP_GPIO_IOM7_MOSI);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM7_CS,   g_AM_BSP_GPIO_IOM7_CS);
            break;
#endif /* AM_BSP_GPIO_IOM7_SCK */

#if defined(AM_BSP_GPIO_IOM0_SCL)
        case ((0 << 2) | AM_HAL_IOM_I2C_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_SCL,  g_AM_BSP_GPIO_IOM0_SCL);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_SDA,  g_AM_BSP_GPIO_IOM0_SDA);
            break;
#endif /* AM_BSP_GPIO_IOM0_SCL */

#if defined(AM_BSP_GPIO_IOM1_SCL)
        case ((1 << 2) | AM_HAL_IOM_I2C_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SCL,  g_AM_BSP_GPIO_IOM1_SCL);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SDA,  g_AM_BSP_GPIO_IOM1_SDA);
            break;
#endif /* AM_BSP_GPIO_IOM1_SCL */

#if defined(AM_BSP_GPIO_IOM2_SCL)
        case ((2 << 2) | AM_HAL_IOM_I2C_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_SCL,  g_AM_BSP_GPIO_IOM2_SCL);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_SDA,  g_AM_BSP_GPIO_IOM2_SDA);
            break;
#endif /* AM_BSP_GPIO_IOM2_SCL */

#if defined(AM_BSP_GPIO_IOM3_SCL)
        case ((3 << 2) | AM_HAL_IOM_I2C_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_SCL,  g_AM_BSP_GPIO_IOM3_SCL);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_SDA,  g_AM_BSP_GPIO_IOM3_SDA);
            break;
#endif /* AM_BSP_GPIO_IOM3_SCL */

#if defined(AM_BSP_GPIO_IOM4_SCL)
        case ((4 << 2) | AM_HAL_IOM_I2C_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_SCL,  g_AM_BSP_GPIO_IOM4_SCL);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_SDA,  g_AM_BSP_GPIO_IOM4_SDA);
            break;
#endif /* AM_BSP_GPIO_IOM4_SCL */

#if defined(AM_BSP_GPIO_IOM5_SCL)
        case ((5 << 2) | AM_HAL_IOM_I2C_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM5_SCL,  g_AM_BSP_GPIO_IOM5_SCL);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM5_SDA,  g_AM_BSP_GPIO_IOM5_SDA);
            break;
#endif /* AM_BSP_GPIO_IOM5_SCL */

#if defined(AM_BSP_GPIO_IOM6_SCL)
        case ((6 << 2) | AM_HAL_IOM_I2C_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM6_SCL,  g_AM_BSP_GPIO_IOM6_SCL);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM6_SDA,  g_AM_BSP_GPIO_IOM6_SDA);
            break;
#endif /* AM_BSP_GPIO_IOM6_SCL */

#if defined(AM_BSP_GPIO_IOM7_SCL)
        case ((7 << 2) | AM_HAL_IOM_I2C_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM7_SCL,  g_AM_BSP_GPIO_IOM7_SCL);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM7_SDA,  g_AM_BSP_GPIO_IOM7_SDA);
            break;
#endif /* AM_BSP_GPIO_IOM7_SCL */

        default:
            break;
    }
} // am_bsp_iom_pins_enable()

//*****************************************************************************
//
// Disable the IOM pins based on mode and module.
//
//*****************************************************************************
void
am_bsp_iom_pins_disable(uint32_t ui32Module, am_hal_iom_mode_e eIOMMode)
{
    uint32_t ui32Combined;

    //
    // Validate parameters
    //
    if ( ui32Module >= AM_REG_IOM_NUM_MODULES )
    {
        return;
    }

    ui32Combined = ((ui32Module << 2) | eIOMMode);

    switch ( ui32Combined )
    {
#if defined(AM_BSP_GPIO_IOM0_SCK)
        case ((0 << 2) | AM_HAL_IOM_SPI_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_SCK,  am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_MISO, am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_MOSI, am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_CS,   am_hal_gpio_pincfg_disabled);
            break;
#endif /* AM_BSP_GPIO_IOM0_SCK */

#if defined(AM_BSP_GPIO_IOM1_SCK)
        case ((1 << 2) | AM_HAL_IOM_SPI_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SCK,  am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_MISO, am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_MOSI, am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_CS,   am_hal_gpio_pincfg_disabled);
            break;
#endif /* AM_BSP_GPIO_IOM1_SCK */

#if defined(AM_BSP_GPIO_IOM2_SCK)
        case ((2 << 2) | AM_HAL_IOM_SPI_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_SCK,  am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_MISO, am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_MOSI, am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_CS,   am_hal_gpio_pincfg_disabled);
            break;
#endif /* AM_BSP_GPIO_IOM2_SCK */

#if defined(AM_BSP_GPIO_IOM3_SCK)
        case ((3 << 2) | AM_HAL_IOM_SPI_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_SCK,  am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_MISO, am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_MOSI, am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_CS,   am_hal_gpio_pincfg_disabled);
            break;
#endif /* AM_BSP_GPIO_IOM3_SCK */

#if defined(AM_BSP_GPIO_IOM4_SCK)
        case ((4 << 2) | AM_HAL_IOM_SPI_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_SCK,  am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_MISO, am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_MOSI, am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_CS,   am_hal_gpio_pincfg_disabled);
            break;
#endif /* AM_BSP_GPIO_IOM4_SCK */

#if defined(AM_BSP_GPIO_IOM5_SCK)
        case ((5 << 2) | AM_HAL_IOM_SPI_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM5_SCK,  am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM5_MISO, am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM5_MOSI, am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM5_CS,   am_hal_gpio_pincfg_disabled);
            break;
#endif /* AM_BSP_GPIO_IOM5_SCK */

#if defined(AM_BSP_GPIO_IOM6_SCK)
        case ((6 << 2) | AM_HAL_IOM_SPI_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM6_SCK,  am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM6_MISO, am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM6_MOSI, am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM6_CS,   am_hal_gpio_pincfg_disabled);
            break;
#endif /* AM_BSP_GPIO_IOM6_SCK */

#if defined(AM_BSP_GPIO_IOM7_SCK)
        case ((7 << 2) | AM_HAL_IOM_SPI_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM7_SCK,  am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM7_MISO, am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM7_MOSI, am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM7_CS,   am_hal_gpio_pincfg_disabled);
            break;
#endif /* AM_BSP_GPIO_IOM7_SCK */

#if defined(AM_BSP_GPIO_IOM0_SCL)
        case ((0 << 2) | AM_HAL_IOM_I2C_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_SCL,  am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_SDA,  am_hal_gpio_pincfg_disabled);
            break;
#endif /* AM_BSP_GPIO_IOM0_SCL */

#if defined(AM_BSP_GPIO_IOM1_SCL)
        case ((1 << 2) | AM_HAL_IOM_I2C_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SCL,  am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SDA,  am_hal_gpio_pincfg_disabled);
            break;
#endif /* AM_BSP_GPIO_IOM1_SCL */

#if defined(AM_BSP_GPIO_IOM2_SCL)
        case ((2 << 2) | AM_HAL_IOM_I2C_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_SCL,  am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM2_SDA,  am_hal_gpio_pincfg_disabled);
            break;
#endif /* AM_BSP_GPIO_IOM2_SCL */

#if defined(AM_BSP_GPIO_IOM3_SCL)
        case ((3 << 2) | AM_HAL_IOM_I2C_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_SCL,  am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM3_SDA,  am_hal_gpio_pincfg_disabled);
            break;
#endif /* AM_BSP_GPIO_IOM3_SCL */

#if defined(AM_BSP_GPIO_IOM4_SCL)
        case ((4 << 2) | AM_HAL_IOM_I2C_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_SCL,  am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM4_SDA,  am_hal_gpio_pincfg_disabled);
            break;
#endif /* AM_BSP_GPIO_IOM4_SCL */

#if defined(AM_BSP_GPIO_IOM5_SCL)
        case ((5 << 2) | AM_HAL_IOM_I2C_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM5_SCL,  am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM5_SDA,  am_hal_gpio_pincfg_disabled);
            break;
#endif /* AM_BSP_GPIO_IOM5_SCL */

#if defined(AM_BSP_GPIO_IOM6_SCL)
        case ((6 << 2) | AM_HAL_IOM_I2C_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM6_SCL,  am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM6_SDA,  am_hal_gpio_pincfg_disabled);
            break;
#endif /* AM_BSP_GPIO_IOM6_SCL */

#if defined(AM_BSP_GPIO_IOM7_SCL)
        case ((7 << 2) | AM_HAL_IOM_I2C_MODE):
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM7_SCL,  am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM7_SDA,  am_hal_gpio_pincfg_disabled);
            break;
#endif /* AM_BSP_GPIO_IOM7_SCL */

        default:
            break;
    }
} // am_bsp_iom_pins_disable()

//*****************************************************************************
//
// Set up the IOS pins based on mode and module.
//
//*****************************************************************************
void
am_bsp_ios_pins_enable(uint32_t ui32Module, uint32_t ui32IOSMode)
{
    uint32_t ui32Combined;

    //
    // Validate parameters
    //
    if ( ui32Module >= AM_REG_IOSLAVE_NUM_MODULES )
    {
        return;
    }

    ui32Combined = ((ui32Module << 2) | ui32IOSMode);

    switch ( ui32Combined )
    {
        case ((0 << 2) | AM_HAL_IOS_USE_SPI):
#if defined(AM_BSP_GPIO_IOS_SCK)
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOS_SCK,  g_AM_BSP_GPIO_IOS_SCK);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOS_MISO, g_AM_BSP_GPIO_IOS_MISO);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOS_MOSI, g_AM_BSP_GPIO_IOS_MOSI);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOS_CE,   g_AM_BSP_GPIO_IOS_CE);
#endif /* AM_BSP_GPIO_IOS_SCK */
            break;

        case ((0 << 2) | AM_HAL_IOS_USE_I2C):
#if defined(AM_BSP_GPIO_IOS_SCL)
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOS_SCL,  g_AM_BSP_GPIO_IOS_SCL);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOS_SDA,  g_AM_BSP_GPIO_IOS_SDA);
#endif /* AM_BSP_GPIO_IOS_SCL */
            break;

        default:
            break;
    }
} // am_bsp_ios_pins_enable()

//*****************************************************************************
//
// Disable the IOS pins based on mode and module.
//
//*****************************************************************************
void
am_bsp_ios_pins_disable(uint32_t ui32Module, uint32_t ui32IOSMode)
{
    uint32_t ui32Combined;

    //
    // Validate parameters
    //
    if ( ui32Module >= AM_REG_IOSLAVE_NUM_MODULES )
    {
        return;
    }

    ui32Combined = ((ui32Module << 2) | ui32IOSMode);

    switch ( ui32Combined )
    {
        case ((0 << 2) | AM_HAL_IOS_USE_SPI):
#if defined(AM_BSP_GPIO_IOS_SCK)
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOS_SCK,  am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOS_MISO, am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOS_MOSI, am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOS_CE,   am_hal_gpio_pincfg_disabled);
#endif /* AM_BSP_GPIO_IOS_SCK */
            break;

        case ((0 << 2) | AM_HAL_IOS_USE_I2C):
#if defined(AM_BSP_GPIO_IOS_SCL)
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOS_SCL,  am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_IOS_SDA,  am_hal_gpio_pincfg_disabled);
#endif /* AM_BSP_GPIO_IOS_SCL */
            break;

        default:
            break;
    }
} // am_bsp_ios_pins_disable()

//*****************************************************************************
//
// Set up I2S pins based on module.
//
//*****************************************************************************
void
am_bsp_i2s_pins_enable(uint32_t ui32Module, bool bBidirectionalData)
{
    //
    // Validate parameters
    //
    if ( ui32Module >= AM_REG_I2S_NUM_MODULES )
    {
        return;
    }
    
    switch ( ui32Module )
    {
        case 0:
            if ( bBidirectionalData )
            {
                am_hal_gpio_pinconfig(AM_BSP_GPIO_I2S0_DATA,    g_AM_BSP_GPIO_I2S0_DATA);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_I2S0_CLK,     g_AM_BSP_GPIO_I2S0_CLK);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_I2S0_WS,      g_AM_BSP_GPIO_I2S0_WS);
            }
            else
            {
                am_hal_gpio_pinconfig(AM_BSP_GPIO_I2S0_SDOUT,   g_AM_BSP_GPIO_I2S0_SDOUT);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_I2S0_CLK,     g_AM_BSP_GPIO_I2S0_CLK);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_I2S0_WS,      g_AM_BSP_GPIO_I2S0_WS);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_I2S0_SDIN,    g_AM_BSP_GPIO_I2S0_SDIN);
            }
            break;
        default:
            break;
    }
}

//*****************************************************************************
//
// Disable I2S pins based on module.
//
//*****************************************************************************
void
am_bsp_i2s_pins_disable(uint32_t ui32Module, bool bBidirectionalData)
{
    //
    // Validate parameters
    //
    if ( ui32Module >= AM_REG_I2S_NUM_MODULES )
    {
        return;
    }
    switch ( ui32Module )
    {
        case 0:
            if ( bBidirectionalData )
            {
                am_hal_gpio_pinconfig(AM_BSP_GPIO_I2S0_DATA,    am_hal_gpio_pincfg_disabled);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_I2S0_CLK,     am_hal_gpio_pincfg_disabled);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_I2S0_WS,      am_hal_gpio_pincfg_disabled);
            }
            else
            {
                am_hal_gpio_pinconfig(AM_BSP_GPIO_I2S0_SDOUT,   am_hal_gpio_pincfg_disabled);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_I2S0_CLK,     am_hal_gpio_pincfg_disabled);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_I2S0_WS,      am_hal_gpio_pincfg_disabled);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_I2S0_SDIN,    am_hal_gpio_pincfg_disabled);
            }
            break;
        default:
            break;
    }
}

//*****************************************************************************
//
// Set up PDM pins based on module.
//
//*****************************************************************************
void am_bsp_pdm_pins_enable(uint32_t ui32Module)
{
    //
    // Validate parameters
    //
    if ( ui32Module >= AM_REG_PDM_NUM_MODULES )
    {
        return;
    }
    
    switch ( ui32Module )
    {
        case 0:
            am_hal_gpio_pinconfig(AM_BSP_GPIO_PDM0_CLK,     g_AM_BSP_GPIO_PDM0_CLK);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_PDM0_DATA,    g_AM_BSP_GPIO_PDM0_DATA);
            break;
        default:
            break;
    }
}

//*****************************************************************************
//
// Disable PDM pins based on module.
//
//*****************************************************************************
void am_bsp_pdm_pins_disable(uint32_t ui32Module)
{
    //
    // Validate parameters
    //
    if ( ui32Module >= AM_REG_PDM_NUM_MODULES )
    {
        return;
    }
    
    switch ( ui32Module )
    {
        case 0:
            am_hal_gpio_pinconfig(AM_BSP_GPIO_PDM0_CLK,     am_hal_gpio_pincfg_disabled);
            am_hal_gpio_pinconfig(AM_BSP_GPIO_PDM0_DATA,    am_hal_gpio_pincfg_disabled);
            break;
        default:
            break;
    }
}

//*****************************************************************************
//
// This function configures up to 10-pins for MSPI serial, dual, quad,
// dual-quad, and octal operation.
//
//*****************************************************************************
void
am_bsp_mspi_pins_enable(uint32_t ui32Module, am_hal_mspi_device_e eMSPIDevice)
{
    switch (ui32Module)
    {
        case 0:
#if (AM_BSP_GPIO_MSPI0_SCK)
            switch ( eMSPIDevice )
            {
                case AM_HAL_MSPI_FLASH_SERIAL_CE0:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_CE0, g_AM_BSP_GPIO_MSPI0_CE0);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D0,  g_AM_BSP_GPIO_MSPI0_D0);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D1,  g_AM_BSP_GPIO_MSPI0_D1);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_SCK, g_AM_BSP_GPIO_MSPI0_SCK);
                    break;
                case AM_HAL_MSPI_FLASH_SERIAL_CE1:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_CE1, g_AM_BSP_GPIO_MSPI0_CE1);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D0,  g_AM_BSP_GPIO_MSPI0_D0);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D1,  g_AM_BSP_GPIO_MSPI0_D1);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_SCK, g_AM_BSP_GPIO_MSPI0_SCK);
                    break;
                case AM_HAL_MSPI_FLASH_QUAD_CE0:
                case AM_HAL_MSPI_FLASH_QUAD_CE0_1_1_4:
                case AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_CE0, g_AM_BSP_GPIO_MSPI0_CE0);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D0,  g_AM_BSP_GPIO_MSPI0_D0);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D1,  g_AM_BSP_GPIO_MSPI0_D1);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D2,  g_AM_BSP_GPIO_MSPI0_D2);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D3,  g_AM_BSP_GPIO_MSPI0_D3);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_SCK, g_AM_BSP_GPIO_MSPI0_SCK);
                    break;
                case AM_HAL_MSPI_FLASH_QUAD_CE1:
                case AM_HAL_MSPI_FLASH_QUAD_CE1_1_1_4:
                case AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_CE1, g_AM_BSP_GPIO_MSPI0_CE1);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D0,  g_AM_BSP_GPIO_MSPI0_D0);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D1,  g_AM_BSP_GPIO_MSPI0_D1);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D2,  g_AM_BSP_GPIO_MSPI0_D2);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D3,  g_AM_BSP_GPIO_MSPI0_D3);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_SCK, g_AM_BSP_GPIO_MSPI0_SCK);
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_CE0:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_CE0, g_AM_BSP_GPIO_MSPI0_CE0);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D0,  g_AM_BSP_GPIO_MSPI0_D0);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D1,  g_AM_BSP_GPIO_MSPI0_D1);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D2,  g_AM_BSP_GPIO_MSPI0_D2);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D3,  g_AM_BSP_GPIO_MSPI0_D3);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D4,  g_AM_BSP_GPIO_MSPI0_D4);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D5,  g_AM_BSP_GPIO_MSPI0_D5);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D6,  g_AM_BSP_GPIO_MSPI0_D6);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D7,  g_AM_BSP_GPIO_MSPI0_D7);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_SCK, g_AM_BSP_GPIO_MSPI0_SCK);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_DQSDM, g_AM_BSP_GPIO_MSPI0_DQSDM);
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_CE1:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_CE1, g_AM_BSP_GPIO_MSPI0_CE1);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D0,  g_AM_BSP_GPIO_MSPI0_D0);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D1,  g_AM_BSP_GPIO_MSPI0_D1);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D2,  g_AM_BSP_GPIO_MSPI0_D2);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D3,  g_AM_BSP_GPIO_MSPI0_D3);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D4,  g_AM_BSP_GPIO_MSPI0_D4);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D5,  g_AM_BSP_GPIO_MSPI0_D5);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D6,  g_AM_BSP_GPIO_MSPI0_D6);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D7,  g_AM_BSP_GPIO_MSPI0_D7);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_SCK, g_AM_BSP_GPIO_MSPI0_SCK);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_DQSDM, g_AM_BSP_GPIO_MSPI0_DQSDM);
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_CE0, g_AM_BSP_GPIO_MSPI0_CE0);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D0,  g_AM_BSP_GPIO_MSPI0_D0);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D1,  g_AM_BSP_GPIO_MSPI0_D1);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D2,  g_AM_BSP_GPIO_MSPI0_D2);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D3,  g_AM_BSP_GPIO_MSPI0_D3);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D4,  g_AM_BSP_GPIO_MSPI0_D4);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D5,  g_AM_BSP_GPIO_MSPI0_D5);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D6,  g_AM_BSP_GPIO_MSPI0_D6);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D7,  g_AM_BSP_GPIO_MSPI0_D7);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_SCK, g_AM_BSP_GPIO_MSPI0_SCK);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_DQSDM, g_AM_BSP_GPIO_MSPI0_DQSDM);
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_CE1, g_AM_BSP_GPIO_MSPI0_CE1);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D0,  g_AM_BSP_GPIO_MSPI0_D0);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D1,  g_AM_BSP_GPIO_MSPI0_D1);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D2,  g_AM_BSP_GPIO_MSPI0_D2);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D3,  g_AM_BSP_GPIO_MSPI0_D3);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D4,  g_AM_BSP_GPIO_MSPI0_D4);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D5,  g_AM_BSP_GPIO_MSPI0_D5);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D6,  g_AM_BSP_GPIO_MSPI0_D6);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D7,  g_AM_BSP_GPIO_MSPI0_D7);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_SCK, g_AM_BSP_GPIO_MSPI0_SCK);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_DQSDM, g_AM_BSP_GPIO_MSPI0_DQSDM);
                    break;
                case AM_HAL_MSPI_FLASH_HEX_DDR_CE0:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_CE0, g_AM_BSP_GPIO_X16SPI_CE0);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D0, g_AM_BSP_GPIO_X16SPI_D0);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D1, g_AM_BSP_GPIO_X16SPI_D1);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D2, g_AM_BSP_GPIO_X16SPI_D2);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D3, g_AM_BSP_GPIO_X16SPI_D3);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D4, g_AM_BSP_GPIO_X16SPI_D4);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D5, g_AM_BSP_GPIO_X16SPI_D5);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D6, g_AM_BSP_GPIO_X16SPI_D6);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D7, g_AM_BSP_GPIO_X16SPI_D7);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D8, g_AM_BSP_GPIO_X16SPI_D8);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D9, g_AM_BSP_GPIO_X16SPI_D9);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D10, g_AM_BSP_GPIO_X16SPI_D10);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D11, g_AM_BSP_GPIO_X16SPI_D11);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D12, g_AM_BSP_GPIO_X16SPI_D12);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D13, g_AM_BSP_GPIO_X16SPI_D13);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D14, g_AM_BSP_GPIO_X16SPI_D14);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D15, g_AM_BSP_GPIO_X16SPI_D15);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_SCK, g_AM_BSP_GPIO_X16SPI_SCK);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_DQSDM, g_AM_BSP_GPIO_X16SPI_DQSDM);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_DQS1DM1, g_AM_BSP_GPIO_X16SPI_DQS1DM1);
                    break;
                case AM_HAL_MSPI_FLASH_HEX_DDR_CE1:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_CE1, g_AM_BSP_GPIO_X16SPI_CE1);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D0, g_AM_BSP_GPIO_X16SPI_D0);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D1, g_AM_BSP_GPIO_X16SPI_D1);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D2, g_AM_BSP_GPIO_X16SPI_D2);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D3, g_AM_BSP_GPIO_X16SPI_D3);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D4, g_AM_BSP_GPIO_X16SPI_D4);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D5, g_AM_BSP_GPIO_X16SPI_D5);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D6, g_AM_BSP_GPIO_X16SPI_D6);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D7, g_AM_BSP_GPIO_X16SPI_D7);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D8, g_AM_BSP_GPIO_X16SPI_D8);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D9, g_AM_BSP_GPIO_X16SPI_D9);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D10, g_AM_BSP_GPIO_X16SPI_D10);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D11, g_AM_BSP_GPIO_X16SPI_D11);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D12, g_AM_BSP_GPIO_X16SPI_D12);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D13, g_AM_BSP_GPIO_X16SPI_D13);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D14, g_AM_BSP_GPIO_X16SPI_D14);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D15, g_AM_BSP_GPIO_X16SPI_D15);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_SCK, g_AM_BSP_GPIO_X16SPI_SCK);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_DQSDM, g_AM_BSP_GPIO_X16SPI_DQSDM);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_DQS1DM1, g_AM_BSP_GPIO_X16SPI_DQS1DM1);
                    break;

                default:
                    break;
            }
        break;
#endif /* AM_BSP_GPIO_MSPI0_SCK */
    case 1:
#if (AM_BSP_GPIO_MSPI1_SCK)
        switch ( eMSPIDevice )
        {
            case AM_HAL_MSPI_FLASH_SERIAL_CE0:
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_CE0, g_AM_BSP_GPIO_MSPI1_CE0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D0,  g_AM_BSP_GPIO_MSPI1_D0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D1,  g_AM_BSP_GPIO_MSPI1_D1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_SCK, g_AM_BSP_GPIO_MSPI1_SCK);
                break;
            case AM_HAL_MSPI_FLASH_QUAD_CE0:
            case AM_HAL_MSPI_FLASH_QUAD_CE0_1_1_4:
            case AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4:
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_CE0, g_AM_BSP_GPIO_MSPI1_CE0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D0,  g_AM_BSP_GPIO_MSPI1_D0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D1,  g_AM_BSP_GPIO_MSPI1_D1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D2,  g_AM_BSP_GPIO_MSPI1_D2);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D3,  g_AM_BSP_GPIO_MSPI1_D3);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_SCK, g_AM_BSP_GPIO_MSPI1_SCK);
                break;
            case AM_HAL_MSPI_FLASH_SERIAL_CE1:
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_CE1, g_AM_BSP_GPIO_MSPI1_CE1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D0,  g_AM_BSP_GPIO_MSPI1_D0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D1,  g_AM_BSP_GPIO_MSPI1_D1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_SCK, g_AM_BSP_GPIO_MSPI1_SCK);
                break;
            case AM_HAL_MSPI_FLASH_QUAD_CE1:
            case AM_HAL_MSPI_FLASH_QUAD_CE1_1_1_4:
            case AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4:
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_CE1, g_AM_BSP_GPIO_MSPI1_CE1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D0,  g_AM_BSP_GPIO_MSPI1_D0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D1,  g_AM_BSP_GPIO_MSPI1_D1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D2,  g_AM_BSP_GPIO_MSPI1_D2);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D3,  g_AM_BSP_GPIO_MSPI1_D3);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_SCK, g_AM_BSP_GPIO_MSPI1_SCK);
                break;
            case AM_HAL_MSPI_FLASH_OCTAL_CE0:
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_CE0, g_AM_BSP_GPIO_MSPI1_CE0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D0,  g_AM_BSP_GPIO_MSPI1_D0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D1,  g_AM_BSP_GPIO_MSPI1_D1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D2,  g_AM_BSP_GPIO_MSPI1_D2);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D3,  g_AM_BSP_GPIO_MSPI1_D3);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D4,  g_AM_BSP_GPIO_MSPI1_D4);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D5,  g_AM_BSP_GPIO_MSPI1_D5);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D6,  g_AM_BSP_GPIO_MSPI1_D6);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D7,  g_AM_BSP_GPIO_MSPI1_D7);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_SCK, g_AM_BSP_GPIO_MSPI1_SCK);
                break;
            case AM_HAL_MSPI_FLASH_OCTAL_CE1:
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_CE1, g_AM_BSP_GPIO_MSPI1_CE1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D0,  g_AM_BSP_GPIO_MSPI1_D0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D1,  g_AM_BSP_GPIO_MSPI1_D1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D2,  g_AM_BSP_GPIO_MSPI1_D2);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D3,  g_AM_BSP_GPIO_MSPI1_D3);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D4,  g_AM_BSP_GPIO_MSPI1_D4);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D5,  g_AM_BSP_GPIO_MSPI1_D5);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D6,  g_AM_BSP_GPIO_MSPI1_D6);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D7,  g_AM_BSP_GPIO_MSPI1_D7);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_SCK, g_AM_BSP_GPIO_MSPI1_SCK);
                break;
             case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0:
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_CE0, g_AM_BSP_GPIO_MSPI1_CE0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D0,  g_AM_BSP_GPIO_MSPI1_D0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D1,  g_AM_BSP_GPIO_MSPI1_D1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D2,  g_AM_BSP_GPIO_MSPI1_D2);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D3,  g_AM_BSP_GPIO_MSPI1_D3);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D4,  g_AM_BSP_GPIO_MSPI1_D4);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D5,  g_AM_BSP_GPIO_MSPI1_D5);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D6,  g_AM_BSP_GPIO_MSPI1_D6);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D7,  g_AM_BSP_GPIO_MSPI1_D7);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_SCK, g_AM_BSP_GPIO_MSPI1_SCK);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_DQSDM, g_AM_BSP_GPIO_MSPI1_DQSDM);
                break;
             case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1:
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_CE1, g_AM_BSP_GPIO_MSPI1_CE1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D0,  g_AM_BSP_GPIO_MSPI1_D0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D1,  g_AM_BSP_GPIO_MSPI1_D1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D2,  g_AM_BSP_GPIO_MSPI1_D2);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D3,  g_AM_BSP_GPIO_MSPI1_D3);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D4,  g_AM_BSP_GPIO_MSPI1_D4);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D5,  g_AM_BSP_GPIO_MSPI1_D5);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D6,  g_AM_BSP_GPIO_MSPI1_D6);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D7,  g_AM_BSP_GPIO_MSPI1_D7);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_SCK, g_AM_BSP_GPIO_MSPI1_SCK);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_DQSDM, g_AM_BSP_GPIO_MSPI1_DQSDM);
                break;
            default:
                break;
        }
        break;
#endif /* AM_BSP_GPIO_MSPI1_SCK */
    case 2:
#if (AM_BSP_GPIO_MSPI2_SCK)
        switch ( eMSPIDevice )
        {
            case AM_HAL_MSPI_FLASH_SERIAL_CE0:
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_CE0, g_AM_BSP_GPIO_MSPI2_CE0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D0,  g_AM_BSP_GPIO_MSPI2_D0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D1,  g_AM_BSP_GPIO_MSPI2_D1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_SCK, g_AM_BSP_GPIO_MSPI2_SCK);
                break;
            case AM_HAL_MSPI_FLASH_SERIAL_CE1:
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_CE1, g_AM_BSP_GPIO_MSPI2_CE1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D0,  g_AM_BSP_GPIO_MSPI2_D0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D1,  g_AM_BSP_GPIO_MSPI2_D1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_SCK, g_AM_BSP_GPIO_MSPI2_SCK);
                break;
            case AM_HAL_MSPI_FLASH_QUAD_CE0:
            case AM_HAL_MSPI_FLASH_QUAD_CE0_1_1_4:
            case AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4:
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_CE0, g_AM_BSP_GPIO_MSPI2_CE0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D0,  g_AM_BSP_GPIO_MSPI2_D0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D1,  g_AM_BSP_GPIO_MSPI2_D1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D2,  g_AM_BSP_GPIO_MSPI2_D2);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D3,  g_AM_BSP_GPIO_MSPI2_D3);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_SCK, g_AM_BSP_GPIO_MSPI2_SCK);
                break;
            case AM_HAL_MSPI_FLASH_QUAD_CE1:
            case AM_HAL_MSPI_FLASH_QUAD_CE1_1_1_4:
            case AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4:
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_CE1, g_AM_BSP_GPIO_MSPI2_CE1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D0,  g_AM_BSP_GPIO_MSPI2_D0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D1,  g_AM_BSP_GPIO_MSPI2_D1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D2,  g_AM_BSP_GPIO_MSPI2_D2);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D3,  g_AM_BSP_GPIO_MSPI2_D3);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_SCK, g_AM_BSP_GPIO_MSPI2_SCK);
                break;
            case AM_HAL_MSPI_FLASH_OCTAL_CE0:
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_CE0, g_AM_BSP_GPIO_MSPI2_CE0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D0,  g_AM_BSP_GPIO_MSPI2_D0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D1,  g_AM_BSP_GPIO_MSPI2_D1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D2,  g_AM_BSP_GPIO_MSPI2_D2);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D3,  g_AM_BSP_GPIO_MSPI2_D3);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D4,  g_AM_BSP_GPIO_MSPI2_D4);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D5,  g_AM_BSP_GPIO_MSPI2_D5);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D6,  g_AM_BSP_GPIO_MSPI2_D6);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D7,  g_AM_BSP_GPIO_MSPI2_D7);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_SCK, g_AM_BSP_GPIO_MSPI2_SCK);
                break;
            case AM_HAL_MSPI_FLASH_OCTAL_CE1:
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_CE1, g_AM_BSP_GPIO_MSPI2_CE1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D0,  g_AM_BSP_GPIO_MSPI2_D0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D1,  g_AM_BSP_GPIO_MSPI2_D1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D2,  g_AM_BSP_GPIO_MSPI2_D2);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D3,  g_AM_BSP_GPIO_MSPI2_D3);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D4,  g_AM_BSP_GPIO_MSPI2_D4);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D5,  g_AM_BSP_GPIO_MSPI2_D5);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D6,  g_AM_BSP_GPIO_MSPI2_D6);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D7,  g_AM_BSP_GPIO_MSPI2_D7);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_SCK, g_AM_BSP_GPIO_MSPI2_SCK);
                break;
            case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0:
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_CE0, g_AM_BSP_GPIO_MSPI2_CE0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D0,  g_AM_BSP_GPIO_MSPI2_D0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D1,  g_AM_BSP_GPIO_MSPI2_D1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D2,  g_AM_BSP_GPIO_MSPI2_D2);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D3,  g_AM_BSP_GPIO_MSPI2_D3);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D4,  g_AM_BSP_GPIO_MSPI2_D4);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D5,  g_AM_BSP_GPIO_MSPI2_D5);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D6,  g_AM_BSP_GPIO_MSPI2_D6);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D7,  g_AM_BSP_GPIO_MSPI2_D7);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_SCK, g_AM_BSP_GPIO_MSPI2_SCK);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_DQSDM, g_AM_BSP_GPIO_MSPI2_DQSDM);
                break;
            case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1:
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_CE1, g_AM_BSP_GPIO_MSPI2_CE1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D0,  g_AM_BSP_GPIO_MSPI2_D0);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D1,  g_AM_BSP_GPIO_MSPI2_D1);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D2,  g_AM_BSP_GPIO_MSPI2_D2);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D3,  g_AM_BSP_GPIO_MSPI2_D3);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D4,  g_AM_BSP_GPIO_MSPI2_D4);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D5,  g_AM_BSP_GPIO_MSPI2_D5);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D6,  g_AM_BSP_GPIO_MSPI2_D6);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D7,  g_AM_BSP_GPIO_MSPI2_D7);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_SCK, g_AM_BSP_GPIO_MSPI2_SCK);
                am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_DQSDM, g_AM_BSP_GPIO_MSPI2_DQSDM);
                break;
            default:
                break;
        }
        break;
#endif /* AM_BSP_GPIO_MSPI2_SCK */
    default:
        break;
    }
} // am_bsp_mspi_pins_enable()

//*****************************************************************************
//
// This function configures up to 10-pins for MSPI serial, dual, quad,
// dual-quad, and octal operation.
//
//*****************************************************************************
void
am_bsp_mspi_pins_disable(uint32_t ui32Module, am_hal_mspi_device_e eMSPIDevice)
{
    switch (ui32Module)
    {
        case 0:
#if (AM_BSP_GPIO_MSPI0_SCK)
            switch ( eMSPIDevice )
            {
                case AM_HAL_MSPI_FLASH_SERIAL_CE0:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_CE0, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_SCK, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_SERIAL_CE1:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_CE1, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_SCK, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_QUAD_CE0:
                case AM_HAL_MSPI_FLASH_QUAD_CE0_1_1_4:
                case AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_CE0, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D2,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D3,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_SCK, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_QUAD_CE1:
                case AM_HAL_MSPI_FLASH_QUAD_CE1_1_1_4:
                case AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_CE1, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D2,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D3,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_SCK, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_CE0:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_CE0, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D2,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D3,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D4,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D5,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D6,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D7,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_SCK, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_CE1:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_CE1, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D2,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D3,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D4,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D5,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D6,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D7,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_SCK, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_CE0, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D2,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D3,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D4,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D5,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D6,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D7,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_SCK, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_DQSDM, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_CE1, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D2,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D3,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D4,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D5,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D6,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_D7,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_SCK, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI0_DQSDM, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_HEX_DDR_CE0:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_CE0, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D0, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D1, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D2, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D3, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D4, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D5, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D6, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D7, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D8, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D9, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D10, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D11, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D12, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D13, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D14, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D15, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_SCK, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_DQSDM, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_DQS1DM1, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_HEX_DDR_CE1:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_CE1, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D0, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D1, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D2, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D3, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D4, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D5, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D6, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D7, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D8, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D9, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D10, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D11, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D12, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D13, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D14, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_D15, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_SCK, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_DQSDM, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_X16SPI_DQS1DM1, am_hal_gpio_pincfg_disabled);
                    break;

                default:
                    break;
            }
#endif /* AM_BSP_GPIO_MSPI0_SCK */
            break;
        case 1:
#if (AM_BSP_GPIO_MSPI1_SCK)
            switch ( eMSPIDevice )
            {
                case AM_HAL_MSPI_FLASH_SERIAL_CE0:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_CE0, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_SCK, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_QUAD_CE0:
                case AM_HAL_MSPI_FLASH_QUAD_CE0_1_1_4:
                case AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_CE0, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D2,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D3,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_SCK, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_CE0:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_CE0, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D2,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D3,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D4,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D5,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D6,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D7,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_SCK, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_SERIAL_CE1:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_CE1, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_SCK, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_QUAD_CE1:
                case AM_HAL_MSPI_FLASH_QUAD_CE1_1_1_4:
                case AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_CE1, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D2,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D3,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_SCK, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_CE1:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_CE1, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D2,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D3,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D4,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D5,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D6,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D7,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_SCK, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_CE0, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D2,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D3,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D4,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D5,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D6,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D7,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_SCK, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_DQSDM, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_CE1, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D2,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D3,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D4,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D5,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D6,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_D7,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_SCK, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI1_DQSDM, am_hal_gpio_pincfg_disabled);
                    break;
                default:
                    break;
            }
            break;
#endif /* AM_BSP_GPIO_MSPI1_SCK*/
        case 2:
#if (AM_BSP_GPIO_MSPI2_SCK)
            switch ( eMSPIDevice )
            {
                case AM_HAL_MSPI_FLASH_SERIAL_CE0:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_CE0, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_SCK, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_QUAD_CE0:
                case AM_HAL_MSPI_FLASH_QUAD_CE0_1_1_4:
                case AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_CE0, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D2,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D3,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_SCK, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_CE0:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_CE0, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D2,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D3,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D4,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D5,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D6,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D7,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_SCK, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_SERIAL_CE1:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_CE1, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_SCK, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_QUAD_CE1:
                case AM_HAL_MSPI_FLASH_QUAD_CE1_1_1_4:
                case AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_CE1, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D2,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D3,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_SCK, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_CE1:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_CE1, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D2,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D3,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D4,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D5,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D6,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D7,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_SCK, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_CE0, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D2,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D3,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D4,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D5,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D6,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D7,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_SCK, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_DQSDM, am_hal_gpio_pincfg_disabled);
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1:
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_CE1, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D0,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D1,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D2,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D3,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D4,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D5,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D6,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_D7,  am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_SCK, am_hal_gpio_pincfg_disabled);
                    am_hal_gpio_pinconfig(AM_BSP_GPIO_MSPI2_DQSDM, am_hal_gpio_pincfg_disabled);
                    break;
                default:
                    break;
            }
            break;
#endif /* AM_BSP_GPIO_MSPI2_SCK*/
        default:
            break;
    }
} // am_bsp_mspi_pins_disable()

//*****************************************************************************
//
// This function returns the pinnum and pincfg for the CE of MSPI requested.
//
//*****************************************************************************
void
am_bsp_mspi_ce_pincfg_get(uint32_t ui32Module,
                          am_hal_mspi_device_e eMSPIDevice,
                          uint32_t * pPinnum,
                          am_hal_gpio_pincfg_t * pPincfg)
{
    switch (ui32Module)
    {
        case PINCFG_GET_DEPRECATED:
            switch (eMSPIDevice)
            {
                case AM_HAL_MSPI_FLASH_HEX_DDR_CE0:
                    *pPinnum = AM_BSP_GPIO_X16SPI_CE0;
                    *pPincfg = g_AM_BSP_GPIO_X16SPI_CE0;
                    break;
                case AM_HAL_MSPI_FLASH_HEX_DDR_CE1:
                    *pPinnum = AM_BSP_GPIO_X16SPI_CE1;
                    *pPincfg = g_AM_BSP_GPIO_X16SPI_CE1;
                    break;
                default:
                    break;
            }
            break;
        case 0:
#if (AM_BSP_GPIO_MSPI0_SCK)
            switch ( eMSPIDevice )
            {
                case AM_HAL_MSPI_FLASH_SERIAL_CE0:
                case AM_HAL_MSPI_FLASH_QUAD_CE0:
                case AM_HAL_MSPI_FLASH_QUAD_CE0_1_1_4:
                case AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4:
                case AM_HAL_MSPI_FLASH_OCTAL_CE0:
                case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0:
                    *pPinnum = AM_BSP_GPIO_MSPI0_CE0;
                    *pPincfg = g_AM_BSP_GPIO_MSPI0_CE0;
                    break;
                case AM_HAL_MSPI_FLASH_SERIAL_CE1:
                case AM_HAL_MSPI_FLASH_QUAD_CE1:
                case AM_HAL_MSPI_FLASH_QUAD_CE1_1_1_4:
                case AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4:
                case AM_HAL_MSPI_FLASH_OCTAL_CE1:
                case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1:
                    *pPinnum = AM_BSP_GPIO_MSPI0_CE1;
                    *pPincfg = g_AM_BSP_GPIO_MSPI0_CE1;
                    break;
                case AM_HAL_MSPI_FLASH_HEX_DDR_CE0:
                    *pPinnum = AM_BSP_GPIO_X16SPI_CE0;
                    *pPincfg = g_AM_BSP_GPIO_X16SPI_CE0;
                    break;
                case AM_HAL_MSPI_FLASH_HEX_DDR_CE1:
                    *pPinnum = AM_BSP_GPIO_X16SPI_CE1;
                    *pPincfg = g_AM_BSP_GPIO_X16SPI_CE1;
                    break;
                default:
                    break;
            }
#endif /* AM_BSP_GPIO_MSPI0_SCK */
            break;
        case 1:
#if (AM_BSP_GPIO_MSPI1_SCK)
            switch ( eMSPIDevice )
            {
                case AM_HAL_MSPI_FLASH_SERIAL_CE0:
                case AM_HAL_MSPI_FLASH_QUAD_CE0:
                case AM_HAL_MSPI_FLASH_QUAD_CE0_1_1_4:
                case AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4:
                case AM_HAL_MSPI_FLASH_OCTAL_CE0:
                case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0:
                    *pPinnum = AM_BSP_GPIO_MSPI1_CE0;
                    *pPincfg = g_AM_BSP_GPIO_MSPI1_CE0;
                    break;
                case AM_HAL_MSPI_FLASH_SERIAL_CE1:
                case AM_HAL_MSPI_FLASH_QUAD_CE1:
                case AM_HAL_MSPI_FLASH_QUAD_CE1_1_1_4:
                case AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4:
                case AM_HAL_MSPI_FLASH_OCTAL_CE1:
                case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1:
                    *pPinnum = AM_BSP_GPIO_MSPI1_CE1;
                    *pPincfg = g_AM_BSP_GPIO_MSPI1_CE1;
                    break;
                default:
                    break;
            }
            break;
#endif /* AM_BSP_GPIO_MSPI1_SCK*/
        case 2:
#if (AM_BSP_GPIO_MSPI2_SCK)
            switch ( eMSPIDevice )
            {
                case AM_HAL_MSPI_FLASH_SERIAL_CE0:
                case AM_HAL_MSPI_FLASH_QUAD_CE0:
                case AM_HAL_MSPI_FLASH_QUAD_CE0_1_1_4:
                case AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4:
                case AM_HAL_MSPI_FLASH_OCTAL_CE0:
                case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0:
                    *pPinnum = AM_BSP_GPIO_MSPI2_CE0;
                    *pPincfg = g_AM_BSP_GPIO_MSPI2_CE0;
                    break;
                case AM_HAL_MSPI_FLASH_SERIAL_CE1:
                case AM_HAL_MSPI_FLASH_QUAD_CE1:
                case AM_HAL_MSPI_FLASH_QUAD_CE1_1_1_4:
                case AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4:
                case AM_HAL_MSPI_FLASH_OCTAL_CE1:
                case AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1:
                    *pPinnum = AM_BSP_GPIO_MSPI2_CE1;
                    *pPincfg = g_AM_BSP_GPIO_MSPI2_CE1;
                    break;
                default:
                    break;
            }
            break;
#endif /* AM_BSP_GPIO_MSPI2_SCK*/
        default:
            break;
    }
} // am_bsp_mspi_ce_pincfg_get()

