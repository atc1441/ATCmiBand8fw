#include "uart_output.h"

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

void *phUART;
uint8_t g_pui8TxBuffer[256];
uint8_t g_pui8RxBuffer[2];

const am_hal_uart_config_t g_sUartConfig =
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

void am_uart_isr(void)
{
    //
    // Service the FIFOs as necessary, and clear the interrupts.
    //
    uint32_t ui32Status;
    am_hal_uart_interrupt_status_get(phUART, &ui32Status, true);
    am_hal_uart_interrupt_clear(phUART, ui32Status);
    am_hal_uart_interrupt_service(phUART, ui32Status);
}

void uart_print(char *pcStr)
{
    uint32_t ui32StrLen = 0;
    uint32_t ui32BytesWritten = 0;

    //
    // Measure the length of the string.
    //
    while (pcStr[ui32StrLen] != 0)
    {
        ui32StrLen++;
    }

    //
    // Print the string via the UART.
    //
    const am_hal_uart_transfer_t sUartWrite =
        {
            .eType = AM_HAL_UART_BLOCKING_WRITE,
            .pui8Data = (uint8_t *)pcStr,
            .ui32NumBytes = ui32StrLen,
            .pui32BytesTransferred = &ui32BytesWritten,
            .ui32TimeoutMs = 100,
            .pfnCallback = NULL,
            .pvContext = NULL,
            .ui32ErrorStatus = 0};

    am_hal_uart_transfer(phUART, &sUartWrite);

    if (ui32BytesWritten != ui32StrLen)
    {
        //
        // Couldn't send the whole string!!
        //
        while (1)
            ;
    }
}

am_hal_gpio_pincfg_t g_AM_GPIO_COM_UART_TX =
    {
        .GP.cfg_b.uFuncSel = AM_HAL_PIN_0_UART0TX,
        .GP.cfg_b.eGPInput = AM_HAL_GPIO_PIN_INPUT_NONE,
        .GP.cfg_b.eGPRdZero = AM_HAL_GPIO_PIN_RDZERO_READPIN,
        .GP.cfg_b.eIntDir = AM_HAL_GPIO_PIN_INTDIR_NONE,
        .GP.cfg_b.eGPOutCfg = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
        .GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P5X,
        .GP.cfg_b.uSlewRate = 0,
        .GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE,
        .GP.cfg_b.uNCE = 0,
        .GP.cfg_b.eCEpol = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
        .GP.cfg_b.uRsvd_0 = 0,
        .GP.cfg_b.ePowerSw = AM_HAL_GPIO_PIN_POWERSW_NONE,
        .GP.cfg_b.eForceInputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.eForceOutputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.uRsvd_1 = 0,
};

am_hal_gpio_pincfg_t g_AM_GPIO_COM_UART_RX =
    {
        .GP.cfg_b.uFuncSel = AM_HAL_PIN_2_UART0RX,
        .GP.cfg_b.eGPInput = AM_HAL_GPIO_PIN_INPUT_NONE,
        .GP.cfg_b.eGPRdZero = AM_HAL_GPIO_PIN_RDZERO_READPIN,
        .GP.cfg_b.eIntDir = AM_HAL_GPIO_PIN_INTDIR_NONE,
        .GP.cfg_b.eGPOutCfg = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
        .GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,
        .GP.cfg_b.uSlewRate = 0,
        .GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE,
        .GP.cfg_b.uNCE = 0,
        .GP.cfg_b.eCEpol = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
        .GP.cfg_b.uRsvd_0 = 0,
        .GP.cfg_b.ePowerSw = AM_HAL_GPIO_PIN_POWERSW_NONE,
        .GP.cfg_b.eForceInputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.eForceOutputEn = AM_HAL_GPIO_PIN_FORCEEN_NONE,
        .GP.cfg_b.uRsvd_1 = 0,
};

void uart_init()
{
    //
    // Initialize the printf interface for UART output.
    //
    am_hal_uart_initialize(0, &phUART);
    am_hal_uart_power_control(phUART, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_uart_configure(phUART, &g_sUartConfig);

    //
    // Enable the UART pins.
    //
    am_hal_gpio_pinconfig(0, g_AM_GPIO_COM_UART_TX);
    am_hal_gpio_pinconfig(2, g_AM_GPIO_COM_UART_RX);

    //
    // Enable interrupts.
    //
    NVIC_SetPriority((IRQn_Type)(UART0_IRQn), AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn));
    am_hal_interrupt_master_enable();

    //
    // Set the main print interface to use the UART print function we defined.
    //
    am_util_stdio_printf_init(uart_print);
}

void uart_flush()
{
    am_hal_uart_tx_flush(phUART);
}