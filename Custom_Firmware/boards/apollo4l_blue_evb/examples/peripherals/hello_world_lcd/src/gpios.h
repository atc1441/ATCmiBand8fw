#pragma once
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

void pinMode(uint32_t pin, uint32_t mode);
void digitalWrite(uint32_t pin, bool state);