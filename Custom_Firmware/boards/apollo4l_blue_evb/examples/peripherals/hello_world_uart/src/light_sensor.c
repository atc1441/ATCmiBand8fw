#include "light_sensor.h"

#include "uart_output.h"
#include "gpios.h"

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

uint32_t ui32ModuleLightSens = 0;
void *LightSens_pIomHandle;

am_hal_iom_config_t LightSens_sti2cSettings =
    {
        .eInterfaceMode = AM_HAL_IOM_I2C_MODE,
        .ui32ClockFreq = AM_HAL_IOM_400KHZ,
        .eSpiMode = AM_HAL_IOM_SPI_MODE_0,
        .ui32NBTxnBufLength = 0,
        .pNBTxnBuf = NULL,
};

void lightSens_transmit(uint8_t cmd, uint8_t data)
{
    uint8_t outBuff[2];
    outBuff[0] = cmd;
    outBuff[1] = data;
    am_hal_iom_transfer_t Transaction;

    Transaction.ui32InstrLen = 0;
    Transaction.ui64Instr = 0;
    Transaction.eDirection = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes = 2;
    Transaction.pui32TxBuffer = (uint32_t *)&outBuff;
    Transaction.uPeerInfo.ui32I2CDevAddr = 0x45;
    Transaction.bContinue = 0;
    Transaction.ui8RepeatCount = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    int error_ret = am_hal_iom_blocking_transfer(LightSens_pIomHandle, &Transaction);
    if (error_ret)
    {
        am_util_stdio_printf("Light Sensor I2C_STATUS_ERROR 1: %i\r\n", error_ret);
        return;
    }
}

void lightSens_transceive(uint8_t cmd, uint32_t *buffer, int readLen)
{
    am_hal_iom_transfer_t Transaction;

    Transaction.ui32InstrLen = 0;
    Transaction.ui64Instr = 0;
    Transaction.eDirection = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes = 1;
    Transaction.pui32TxBuffer = (uint32_t *)&cmd;
    Transaction.uPeerInfo.ui32I2CDevAddr = 0x45;
    Transaction.bContinue = 1;
    Transaction.ui8RepeatCount = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    if (am_hal_iom_blocking_transfer(LightSens_pIomHandle, &Transaction))
    {
        am_util_stdio_printf("Light Sensor I2C_STATUS_ERROR 1\r\n");
        return;
    }

    Transaction.ui32InstrLen = 0;
    Transaction.ui64Instr = 0;
    Transaction.eDirection = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes = readLen;
    Transaction.pui32RxBuffer = buffer;
    Transaction.uPeerInfo.ui32I2CDevAddr = 0x45;
    Transaction.bContinue = 0;
    Transaction.ui8RepeatCount = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    if (am_hal_iom_blocking_transfer(LightSens_pIomHandle, &Transaction))
    {
        am_util_stdio_printf("Light Sensor I2C_STATUS_ERROR 2\r\n");
        return;
    }
}

void lightSens_init()
{
    pinMode(80, 0);
    // digitalWrite(80, 0);
    am_util_delay_ms(10);
    am_util_stdio_printf("Light Sensor init started\r\n");
    if (am_hal_iom_initialize(ui32ModuleLightSens, &LightSens_pIomHandle) ||
        am_hal_iom_power_ctrl(LightSens_pIomHandle, AM_HAL_SYSCTRL_WAKE, false) ||
        am_hal_iom_configure(LightSens_pIomHandle, &LightSens_sti2cSettings) ||
        am_hal_iom_enable(LightSens_pIomHandle))
    {
        am_util_stdio_printf("Light Sensor AM_DEVICES_I2C_STATUS_ERROR\r\n");
    }
    else
    {
        am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_SCL, g_AM_BSP_GPIO_IOM0_SCL);
        am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_SDA, g_AM_BSP_GPIO_IOM0_SDA);
        am_util_stdio_printf("Light Sensor AM_DEVICES_I2C_STATUS_SUCCESS\r\n");
        uint8_t dataOut[50];
        lightSens_transceive(0x3E, (uint32_t *)&dataOut, 1);
        am_util_stdio_printf("Light answer: %02X %02X %02X\r\n", dataOut[0], dataOut[1], dataOut[2]);
        lightSens_transceive(0x3E, (uint32_t *)&dataOut, 1);
        am_util_stdio_printf("Light answer: %02X %02X %02X\r\n", dataOut[0], dataOut[1], dataOut[2]);
        lightSens_transmit(0x80, 0xFF);
        am_util_delay_ms(50);

        /////////////////////////////
        lightSens_transmit(0x02, 0x20);
        lightSens_transmit(0x04, 0x00);
        lightSens_transmit(0x05, 0x40);
        lightSens_transmit(0x10, 0x00);
        lightSens_transmit(0x4e, 0x26);
        lightSens_transmit(0x6f, 0x14);
        lightSens_transmit(0xa1, 0x03);
        lightSens_transmit(0xa5, 0x00);
        lightSens_transmit(0xdb, 0x00);
        lightSens_transmit(0x60, 0xa2);
        lightSens_transmit(0x61, 0x00);
        lightSens_transmit(0x62, 0x00);
        lightSens_transmit(0x63, 0x00);
        lightSens_transmit(0xf6, 0x09);
        lightSens_transmit(0xf1, 0x00);

        /////////////////////////////

        lightSens_transmit(0x60, 0x22); // Clear FIFO
        lightSens_transmit(0x00, 0x02); // Mode Enable
        // lightSens_transmit(0x00, 0x00); // Mode Disable

        lightSens_transceive(0x64, (uint32_t *)&dataOut, 1);// Read if fifo is full = 1
        am_util_stdio_printf("Light answer: %02X\r\n", dataOut[0]);
        lightSens_transceive(0x65, (uint32_t *)&dataOut, 1);// Read amount of data in fifo
        am_util_stdio_printf("Light answer: %02X\r\n", dataOut[0]);

        /*while (1)
        {
            lightSens_transceive(0x66, (uint32_t *)&dataOut, 50);
            // am_util_stdio_printf("Light answer: %02X %02X %02X\r\n", dataOut[0], dataOut[1], dataOut[2]);
            uint16_t light_value = (dataOut[0] << 8 | dataOut[1]);
            uint8_t light_state = dataOut[2];
            am_util_stdio_printf("Light:    %i  %i\r\n", light_value, light_state);
            am_util_delay_ms(10);
        }
        dataOut[0] = 0x22;
        lightSens_transmit(0x60, 0x22);*/
    }
}