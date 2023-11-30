#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "gpios.h"
#include "display.h"
#include "uart_output.h"

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

    am_hal_interrupt_master_enable();
    init_display();
    uint8_t a = 0;
    while (1)
    {
        uart_flush();
        display_buffset(color565(a, a, a++));
        am_util_delay_ms(20);
        // end_display();
        am_util_stdio_printf("App:    %i\r\n", a);
    }
}
