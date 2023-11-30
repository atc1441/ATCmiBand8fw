/*******************************************************************************
 * Copyright (c) 2022 Think Silicon S.A.
 *
   Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this header file and/or associated documentation files to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies of the
 * Materials, and to permit persons to whom the Materials are furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Materials.
 *
 * MODIFICATIONS TO THIS FILE MAY MEAN IT NO LONGER ACCURATELY REFLECTS
 * NEMAGFX API. THE UNMODIFIED, NORMATIVE VERSIONS OF THINK-SILICON NEMAGFX
 * SPECIFICATIONS AND HEADER INFORMATION ARE LOCATED AT:
 *   https://think-silicon.com/products/software/nemagfx-api
 *
 *  The software is provided 'as is', without warranty of any kind, express or
 *  implied, including but not limited to the warranties of merchantability,
 *  fitness for a particular purpose and noninfringement. In no event shall
 *  Think Silicon S.A. be liable for any claim, damages or other liability, whether
 *  in an action of contract, tort or otherwise, arising from, out of or in
 *  connection with the software or the use or other dealings in the software.
 ******************************************************************************/


#ifndef NEMA_EVENT_H__
#define NEMA_EVENT_H__

#include "nema_sys_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NEMA_EVENT_HIDE_CURSOR  (1 << 1)

typedef enum {
    MOUSE_EVENT_NONE = 0,
    MOUSE_EVENT_LEFT_CLICK,
    MOUSE_EVENT_LEFT_RELEASE,
    MOUSE_EVENT_MIDDLE_CLICK,
    MOUSE_EVENT_MIDDLE_RELEASE,
    MOUSE_EVENT_RIGHT_CLICK,
    MOUSE_EVENT_RIGHT_RELEASE,
    MOUSE_EVENT_SCROLL_UP,
    MOUSE_EVENT_SCROLL_DOWN,
    MOUSE_EVENT_MAX
} nema_mouse_event_t;

typedef enum {
    KB_EVENT_NONE  = 0,
    KB_EVENT_PRESS,
    KB_EVENT_HOLD,
    KB_EVENT_RELEASE,
    KB_EVENT_MAX
} nema_kb_event_t;

typedef enum {
    MOUSE_STATE_NONE           = 0,
    MOUSE_STATE_LEFT_CLICKED   = 1,
    MOUSE_STATE_MIDDLE_CLICKED = 1U<<1,
    MOUSE_STATE_RIGHT_CLICKED  = 1U<<2
} nema_mouse_state_t;

typedef struct {
    int mouse_x;
    int mouse_y;
    int mouse_dx;
    int mouse_dy;
    int mouse_event;
    int mouse_state;
    int kb_event;
    char kb_key;
    int timer_id;
    uint32_t timer_expirations;
} nema_event_t;

int  nema_event_init(int flags, int mouse_init_x, int mouse_init_y, int mouse_max_x, int mouse_max_y);
int  nema_event_wait(nema_event_t *event, int block_until_event);

void nema_event_force_cursor_xy(int x, int y);

//Triple Framebuffer
uintptr_t nema_init_triple_fb(int layer, uintptr_t fb0_phys, uintptr_t fb1_phys, uintptr_t fb2_phys);
uintptr_t nema_swap_fb(int layer);

int nema_timer_create(void);
void nema_timer_destroy(int timer_id);
int  nema_timer_set_periodic(int timer_id, uint32_t timeout_milisecs);
int  nema_timer_set_oneshot(int timer_id, uint32_t timeout_milisecs);
void nema_timer_stop(int timer_id);

//Nema wait interrupt
//void nema_wait_irq(void);
//void nema_wait_cl(nema_cmdlist_t *cl);

#ifdef __cplusplus
}
#endif

#endif //NEMA_EVENT_H__
