#include "light_sensor.h"

#include "uart_output.h"
#include "gpios.h"

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#define ACCL_CLK 47
#define ACCL_MOSI 48
#define ACCL_MISO 49
#define ACCL_CS 12
#define ACCL_INT1 61
#define ACCL_INT2 62

uint8_t spiSend(uint8_t data)
{
    uint8_t tempRet = 0;

    for (int i = 0; i < 8; i++)
    {
        if ((data << i) & 0x80)
            digitalWrite(ACCL_MOSI, 1);
        else
            digitalWrite(ACCL_MOSI, 0);
        digitalWrite(ACCL_CLK, 0);
        am_util_delay_us(100);
        digitalWrite(ACCL_CLK, 1);
        tempRet <<= 1;
        if (digitalRead(ACCL_MISO))
        {

            tempRet |= 1;
        }
        else
        {
        }
        am_util_delay_us(100);
    }

    return tempRet;
}
void accl_transmit(uint8_t cmd, uint8_t *data, int len)
{
    digitalWrite(ACCL_CS, 0);
    am_util_delay_us(100);
    spiSend(cmd);
    am_util_delay_us(100);
    for (int i = 0; i < len; i++)
    {
        spiSend(data[i]);
    }
    am_util_delay_us(100);
    digitalWrite(ACCL_CS, 1);
}

void accl_transceive(uint8_t cmd, uint8_t *buffer, int readLen)
{
    digitalWrite(ACCL_CS, 0);
    am_util_delay_us(100);
    cmd |= 0x80;
    spiSend(cmd);
    spiSend(0x00);
    //spiSend(cmd);
    am_util_delay_us(100);
    for (int i = 0; i < readLen; i++)
    {
        buffer[i] = spiSend(0xff);
    }
    am_util_delay_us(100);
    digitalWrite(ACCL_CS, 1);
}

uint8_t spi_buffer[100];

uint8_t dd = 0;
void accl_init()
{
    pinMode(ACCL_CS, 1);
    digitalWrite(ACCL_CS, 1);

    pinMode(ACCL_MOSI, 1);
    digitalWrite(ACCL_MOSI, 1);
    pinMode(ACCL_CLK, 1);
    digitalWrite(ACCL_CLK, 1);

    pinMode(ACCL_MISO, 0);

    pinMode(ACCL_INT1, 0);
    pinMode(ACCL_INT2, 0);
    am_util_delay_ms(50);
    am_util_stdio_printf("Accl init started\r\n"); 
    //while (1)
    {
        accl_transceive(0x00, spi_buffer, 10);
        am_util_stdio_printf("Accl answer: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n", spi_buffer[0], spi_buffer[1], spi_buffer[2], spi_buffer[3], spi_buffer[4], spi_buffer[5], spi_buffer[6], spi_buffer[7], spi_buffer[8], spi_buffer[9], spi_buffer[10], spi_buffer[11], spi_buffer[12], spi_buffer[13], spi_buffer[14], spi_buffer[15], spi_buffer[16], spi_buffer[17], spi_buffer[18], spi_buffer[19]);
    }
}