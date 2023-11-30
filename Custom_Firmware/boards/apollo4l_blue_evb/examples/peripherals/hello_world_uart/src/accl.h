#pragma once
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

void accl_transmit(uint8_t cmd, uint8_t data);
void accl_transceive(uint8_t cmd, uint8_t *buffer, int readLen);
void accl_init();