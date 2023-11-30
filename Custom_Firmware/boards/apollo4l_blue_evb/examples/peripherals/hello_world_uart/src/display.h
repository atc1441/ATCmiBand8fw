#pragma once
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

void init_display();
void end_display();
void set_xy_here(uint16_t x_now, uint16_t y_now);
void set_light_here(uint16_t light_value, uint16_t light_small);
void display_buffset(uint16_t data);