#pragma once
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

void touch_transmit(uint16_t cmd, uint32_t *buffer, int sendLen);
void touch_transceive(uint16_t cmd, uint32_t *buffer, int readLen);
void touch_init();