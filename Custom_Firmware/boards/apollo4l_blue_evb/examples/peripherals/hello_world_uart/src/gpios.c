#include "gpios.h"

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#define AM_HAL_GPIO_PINCFG_OUTPUT22                                    \
    {                                                                  \
        .GP.cfg_b.uFuncSel = 3,                                        \
        .GP.cfg_b.eGPOutCfg = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL,         \
        .GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_1P0X, \
        .GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE,               \
        .GP.cfg_b.eGPInput = AM_HAL_GPIO_PIN_INPUT_NONE,               \
        .GP.cfg_b.eGPRdZero = AM_HAL_GPIO_PIN_RDZERO_READPIN,          \
        .GP.cfg_b.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI,              \
        .GP.cfg_b.uSlewRate = 0,                                       \
        .GP.cfg_b.uNCE = 0,                                            \
        .GP.cfg_b.eCEpol = 0,                                          \
        .GP.cfg_b.ePowerSw = 0,                                        \
        .GP.cfg_b.eForceInputEn = 0,                                   \
        .GP.cfg_b.eForceOutputEn = 0,                                  \
        .GP.cfg_b.uRsvd_0 = 0,                                         \
        .GP.cfg_b.uRsvd_1 = 0,                                         \
    }
const am_hal_gpio_pincfg_t am_hal_gpio_pincfg_output22 = AM_HAL_GPIO_PINCFG_OUTPUT22;

const am_hal_gpio_pincfg_t am_hal_gpio_pincfg_output33 = {.GP.cfg = 0x103};

void pinMode(uint32_t pin, uint32_t mode)
{
    switch (mode)
    {
    case 0:
        am_hal_gpio_pinconfig(pin, am_hal_gpio_pincfg_input);
        break;
    case 1:
        am_hal_gpio_pinconfig(pin, am_hal_gpio_pincfg_output);
        break;
    case 2:
        am_hal_gpio_pinconfig(pin, am_hal_gpio_pincfg_output33);
        break;
    }
}

void digitalWrite(uint32_t pin, bool state)
{
    if (state)
        am_hal_gpio_state_write(pin, AM_HAL_GPIO_OUTPUT_SET);
    else
        am_hal_gpio_state_write(pin, AM_HAL_GPIO_OUTPUT_CLEAR);
}

bool digitalRead(uint32_t pin)
{
    uint32_t pin_state = 0;
    am_hal_gpio_state_read(pin, AM_HAL_GPIO_INPUT_READ, &pin_state);
    return pin_state & 1;
}