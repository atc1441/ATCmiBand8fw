#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "gpios.h"
#include "display.h"
#include "uart_output.h"
#include "touch.h"
#include "light_sensor.h"
#include "accl.h"

uint16_t color565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
int main(void)
{
    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();

    //
    // Configure the board for low power operation.
    //
    am_bsp_external_pwr_on();
    am_util_delay_ms(100);
    am_bsp_low_power_init();

    pinMode(88, 1);
    digitalWrite(88, 0);
    pinMode(91, 1);
    digitalWrite(91, 1);
    pinMode(23, 1);
    digitalWrite(23, 0);

    pinMode(7, 2);
    digitalWrite(7, 1);

    digitalWrite(55, 0);
    pinMode(55, 1);
    digitalWrite(5, 1);
    digitalWrite(5, 0);

    pinMode(18, 1);
    digitalWrite(18, 1);
    am_util_delay_ms(100);
    pinMode(88, 1);
    digitalWrite(88, 0);
    am_util_delay_ms(10);

    uart_init();

    am_util_stdio_printf("Hello World!\r\n");

    init_display();
    display_buffset(color565(123, 123, 0));
    /*am_util_delay_ms(2000);
    am_util_stdio_printf("End display:    %i\r\n", a);
    end_display();*/

    touch_init();
    lightSens_init();
    accl_init();

    am_hal_interrupt_master_enable();
    uint8_t a = 0;
    while (1)
    {
        uart_flush();
        // am_util_delay_ms(1000);
        if (!digitalRead(24))
        {
            // am_util_stdio_printf("App:    %i\r\n", pin_state);
            uint8_t data[16] = {0};
            touch_transceive(0x0010, (uint32_t *)&data, 16);
            // am_util_stdio_printf("Touch answer: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);
            uint16_t touch_x = (data[4] << 8 | data[3]);
            uint16_t touch_y = (data[6] << 8 | data[5]);
            uint8_t touch_state = data[8];
            am_util_stdio_printf("Touch:    %i  %i  %i\r\n", touch_x, touch_y, touch_state);
            set_xy_here(touch_x, touch_y);
            /*if (touch_state == 3 || touch_state == 4)
                display_buffset(color565(0, 0, 255));
            else
                display_buffset(color565(touch_x, touch_y / 2, 0));*/
        }
        else
        {
            uint8_t dataOut[50];
            lightSens_transceive(0x66, (uint32_t *)&dataOut, 4);
            // am_util_stdio_printf("Light answer: %02X %02X %02X\r\n", dataOut[0], dataOut[1], dataOut[2]);
            uint16_t light_value = (dataOut[0] << 8 | dataOut[1]);
            uint16_t light_state = (dataOut[2] << 8 | dataOut[3]);
            am_util_stdio_printf("Light:    %i  %i\r\n", light_value, light_state);
            set_light_here(light_value, light_state);
            lightSens_transmit(0x60, 0x22); // Clear FIFO
            am_util_delay_ms(20);
        }
    }
}
