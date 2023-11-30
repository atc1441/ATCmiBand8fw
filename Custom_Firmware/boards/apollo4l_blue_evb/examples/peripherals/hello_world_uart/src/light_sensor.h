#pragma once
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

void lightSens_transmit(uint8_t cmd, uint8_t data);
void lightSens_transceive(uint8_t cmd, uint32_t *buffer, int readLen);
void lightSens_init();