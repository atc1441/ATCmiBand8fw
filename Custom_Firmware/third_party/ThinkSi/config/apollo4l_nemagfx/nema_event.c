// -----------------------------------------------------------------------------
// Copyright (c) 2019 Think Silicon S.A.
// Think Silicon S.A. Confidential Proprietary
// -----------------------------------------------------------------------------
//     All Rights reserved - Unpublished -rights reserved under
//         the Copyright laws of the European Union
//
//  This file includes the Confidential information of Think Silicon S.A.
//  The receiver of this Confidential Information shall not disclose
//  it to any third party and shall protect its confidentiality by
//  using the same degree of care, but not less than a reasonable
//  degree of care, as the receiver uses to protect receiver's own
//  Confidential Information. The entire notice must be reproduced on all
//  authorised copies and copies may only be made to the extent permitted
//  by a licensing agreement from Think Silicon S.A..
//
//  The software is provided 'as is', without warranty of any kind, express or
//  implied, including but not limited to the warranties of merchantability,
//  fitness for a particular purpose and noninfringement. In no event shall
//  Think Silicon S.A. be liable for any claim, damages or other liability, whether
//  in an action of contract, tort or otherwise, arising from, out of or in
//  connection with the software or the use or other dealings in the software.
//
//
//  This file can be modified by OEMs as specified in the license agreement
//
//                    Think Silicon S.A.
//                    http://www.think-silicon.com
//                    Patras Science Park
//                    Rion Achaias 26504
//                    Greece
// -----------------------------------------------------------------------------

#ifndef BAREMETAL
#include "FreeRTOS.h"
#include "portable.h"
#include "timers.h"
#include "semphr.h"

#include "nema_event.h"

#include "am_bsp.h"
#include "am_util_delay.h"
#include "am_util_stdio.h"
#include "string.h"
#include "am_util_debug.h"
#include "am_devices_tma525.h"

#ifdef USE_TOUCH
static int touch_init();
#endif

#define CLAMPX( x )  ( (x) < 0 ? 0 : (x) >= maxx ? maxx : (x) );
#define CLAMPY( y )  ( (y) < 0 ? 0 : (y) >= maxy ? maxy : (y) );

#define MAX_EVENTS 10

#define FB_GPU  0
#define FB_DC   1
#define FB_FREE 2

#define NUM_LAYERS 1U

static uintptr_t triple_fbs[NUM_LAYERS][3];

uintptr_t
nema_init_triple_fb(int layer, uintptr_t fb0_phys, uintptr_t fb1_phys, uintptr_t fb2_phys)
{
    //actually doing always 2 framebuffers
    //fb2_phys is ignored

    triple_fbs[layer][FB_GPU]  = fb0_phys;
    triple_fbs[layer][FB_DC]   = fb1_phys;
    // triple_fbs[layer][FB_FREE] = fb2_phys;

    return triple_fbs[layer][FB_GPU];
}

uintptr_t
nema_swap_fb(int layer)
{
    if (layer < 0) {
        layer = 0;
    }

    {
        uintptr_t tmp = triple_fbs[layer][FB_DC];
        triple_fbs[layer][FB_DC] = triple_fbs[layer][FB_GPU];
        triple_fbs[layer][FB_GPU]  = tmp;
    }
    return triple_fbs[layer][FB_GPU];
}

static int timer1_is_initialized = 0;

static SemaphoreHandle_t xSemaphore = NULL;

#ifdef USE_TOUCH
void *g_pIOMHandle_touch;

bool touch_release = false;
bool i2c_read_flag = false;
nema_event_t    g_s_last_event;
nema_event_t    g_s_buf_touch;

am_hal_iom_config_t     g_sIomCfg_touch =
{
    // Set up IOM
    // Initialize the Device
    .eInterfaceMode       = AM_HAL_IOM_I2C_MODE,
    .ui32ClockFreq        = AM_HAL_IOM_400KHZ,
    .ui32NBTxnBufLength = 0,
    .pNBTxnBuf = NULL
};

#define TOUCH_GPIO_IDX  GPIO_NUM2IDX(AM_BSP_GPIO_TOUCH_INT)

#if TOUCH_GPIO_IDX == 0
#define disp_touch_isr        am_gpio0_001f_isr
#elif TOUCH_GPIO_IDX == 1
#define disp_touch_isr        am_gpio0_203f_isr
#elif TOUCH_GPIO_IDX == 2
#define disp_touch_isr        am_gpio0_405f_isr
#elif TOUCH_GPIO_IDX == 3
#define disp_touch_isr        am_gpio0_607f_isr
#endif

//
// GPIO TE interrupts.
//
static const IRQn_Type touch_interrupts[] =
{
    GPIO0_001F_IRQn,
    GPIO0_203F_IRQn,
    GPIO0_405F_IRQn,
    GPIO0_607F_IRQn
};
//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void
disp_touch_isr(void)
{
    uint32_t ui32Status;
    am_hal_gpio_interrupt_irq_status_get(touch_interrupts[TOUCH_GPIO_IDX], false, &ui32Status);
    am_hal_gpio_interrupt_irq_clear(touch_interrupts[TOUCH_GPIO_IDX], ui32Status);
    am_hal_gpio_interrupt_service(touch_interrupts[TOUCH_GPIO_IDX], ui32Status);
}
void
disp_touch_cb(void)
{
    if (am_devices_tma525_get_point((uint16_t*)(&g_s_buf_touch.mouse_x),
                               (uint16_t*)(&g_s_buf_touch.mouse_y),
                               &touch_release) == AM_DEVICES_TMA525_STATUS_SUCCESS)
    {
        if (touch_release == true)
        {
            i2c_read_flag = false;
        }
        else
        {
            i2c_read_flag = true;
        }
    }
}
static int
touch_init()
{
    uint32_t IntNum = AM_BSP_GPIO_TOUCH_INT;

    am_devices_tma525_init(AM_BSP_TP_IOM_MODULE, &g_sIomCfg_touch, &g_pIOMHandle_touch);

    am_hal_gpio_mask_t gpio_mask = AM_HAL_GPIO_MASK_DECLARE_ZERO;
    gpio_mask.U.Msk[GPIO_NUM2IDX(IntNum)] = GPIO_NUM2MSK(IntNum);
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_INT_CHANNEL_0, &gpio_mask);
    am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, IntNum,(am_hal_gpio_handler_t)disp_touch_cb, NULL);
    am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0,AM_HAL_GPIO_INT_CTRL_INDV_ENABLE,(void *)&IntNum);

    NVIC_SetPriority(touch_interrupts[TOUCH_GPIO_IDX], 0x4);
    NVIC_EnableIRQ(touch_interrupts[TOUCH_GPIO_IDX]);

    return 0;
}
#endif

int
nema_event_init(int flags, int mouse_init_x, int mouse_init_y, int mouse_max_x, int mouse_max_y)
{
    timer1_is_initialized = 0;

    xSemaphore = xSemaphoreCreateMutex();

    if ( xSemaphore == NULL ) {
        return -1;
    }

#ifdef USE_TOUCH
    touch_init();
#endif

    return 0;
}

static void semaphore_take(void) {
    (void)xSemaphoreTake(xSemaphore, portMAX_DELAY);
}

static void semaphore_give(void) {
    (void)xSemaphoreGive( xSemaphore );
}

#define TIMER_1 1
static TimerHandle_t timer1_handle;
static UBaseType_t timer1_reload = pdTRUE;

static volatile uint32_t timer1_passed = 0;

static TaskHandle_t xHandlingTask = (TaskHandle_t) 0;

static void
wake_task(void) {
    if ( xHandlingTask != (TaskHandle_t) 0 )
    {
        (void)xTaskNotify( xHandlingTask,
                            0,
                            eNoAction);
    }
}

static void
timer1_handler(TimerHandle_t pxTimer)
{
    //Not allowed, DBG_print() is also working with interrupt, not possible to have
    //uart higher priority compared to timer (no nesting)
    //DBG_print("@@ Timer1\n\r");

    semaphore_take();
    timer1_passed++;
    semaphore_give();

    wake_task();
}

int
nema_timer_create(void)
{
    //we have only one timer
    if ( timer1_is_initialized != 0) {
        return -1;
    }

    timer1_handle = xTimerCreate(
            "timer1", /* name */
            pdMS_TO_TICKS(100), /* period/time */
            timer1_reload, /* auto reload */
            (void*)1, /* timer ID */
            timer1_handler); /* callback */

    if (timer1_handle==NULL) {
        // am_util_stdio_printf("failed to create timer 1Sec\r\n");
        return -1;
    }

    timer1_is_initialized = 1;

/*     xTimerStart(timer1_handle, 0); */

    return TIMER_1;
}

void
nema_timer_destroy(int timer_id)
{
    //If timer isn't initialized, we've nothing to destroy
    if ( timer1_is_initialized == 0) {
        return;
    }

    //Only TIMER_1 is available
    if (timer_id != TIMER_1) {
        return;
    }

    //stop current timer for safety
    nema_timer_stop(timer_id);
    (void)xTimerDelete(timer1_handle, 0);

    timer1_is_initialized = 0;
}

static int
timer_set(int timer1_id, uint32_t timeout_milisecs, UBaseType_t reload)
{
    if (timer1_is_initialized == 0) {
        return -1;
    }

    //Only TIMER_1 is available
    if (timer1_id != TIMER_1) {
        return -1;
    }

    if ( timer1_reload == reload) {
        //just change the timer period
        if (xTimerChangePeriod(timer1_handle, pdMS_TO_TICKS(timeout_milisecs), 0) != pdPASS) {
            return -1;
        }

        // No need to call xTimerStart()
        // xTimerChangePeriod() will cause the timer to start.
        //
        // if (xTimerStart(timer1_handle, 0) != pdPASS) {
        //     return -1;
        // }
    } else {
        if ( xTimerIsTimerActive(timer1_handle) == pdTRUE ) {
            //Timer is running.
            //Can't change reload mode
            return -2;
        }

        //delete previous timer
        if ( xTimerDelete(timer1_handle, 0) == pdFALSE ) {
            return -3;
        }

        timer1_reload = reload;

        timer1_handle = xTimerCreate(
                "timer1", /* name */
                pdMS_TO_TICKS(timeout_milisecs), /* period/time */
                timer1_reload, /* auto reload */
                (void*)1, /* timer ID */
                timer1_handler); /* callback */

        if (xTimerStart(timer1_handle, 0) != pdPASS) {
            return -4;
        }
    }

    return timer1_id;
}

int
nema_timer_set_oneshot(int timer_id, uint32_t timeout_milisecs)
{
    return timer_set(timer_id, timeout_milisecs, pdFALSE);
}

int
nema_timer_set_periodic(int timer_id, uint32_t timeout_milisecs)
{
    return timer_set(timer_id, timeout_milisecs, pdTRUE);
}

void
nema_timer_start(int timer_id)
{
    if (timer1_is_initialized == 0) {
        return;
    }

    if (timer_id != TIMER_1) {
        return;
    }

    xTimerStart(timer1_handle, 10);
}

void
nema_timer_stop(int timer_id)
{
    if (timer1_is_initialized == 0) {
        return;
    }

    //Only TIMER_1 is available
    if (timer_id != TIMER_1) {
        return;
    }

    if ( xTimerIsTimerActive(timer1_handle) == pdTRUE) {
        (void)xTimerStop(timer1_handle, 0);
    }
}

static void
wait_for_timer(void)
{
    BaseType_t xResult;

    xHandlingTask = xTaskGetCurrentTaskHandle();

    /* If a task is in the Blocked state to wait for a notification when the
       notification arrives then the task immediately exits the Blocked state
       and the notification does not remain pending. If a task was not waiting
       for a notification when a notification arrives then the notification
       will remain pending until the receiving task reads its notification
       value. */

    /* Wait to be notified of an interrupt. */
    xResult = xTaskNotifyWait( 0,    /* Don't clear bits on entry. */
                       0,                  /* Don't clear bits on exit. */
                       NULL,               /* No nitification value */
                       portMAX_DELAY );    /* Block indefinitely. */

    (void)xResult;
}

int
nema_event_wait(nema_event_t *event, int block_until_event)
{
    int i32WaitEvent = 0;
#ifdef USE_TOUCH
    event->mouse_event         = MOUSE_EVENT_NONE;
    //g_s_last_event.mouse_event = MOUSE_EVENT_NONE;
    if(i2c_read_flag)
	{
        //
	    // touch display panel.
        //
        i2c_read_flag = false;
        event->mouse_x     = g_s_buf_touch.mouse_x;
        event->mouse_y     = g_s_buf_touch.mouse_y;

        event->mouse_state = MOUSE_STATE_LEFT_CLICKED;
        // if previous state is not CLICKED, CLICK now
        if (g_s_last_event.mouse_state != event->mouse_state)
        {
            event->mouse_event         = MOUSE_EVENT_LEFT_CLICK;

            // it's a click, not a drag/swipe
            event->mouse_dx = 0;
            event->mouse_dy = 0;
        }
        else
        {
            event->mouse_dx = event->mouse_x - g_s_last_event.mouse_x;
            event->mouse_dy = event->mouse_y - g_s_last_event.mouse_y;
        }
        g_s_last_event.mouse_state = event->mouse_state;
        g_s_last_event.mouse_x     = event->mouse_x;
        g_s_last_event.mouse_y     = event->mouse_y;
        i32WaitEvent = 1;
	}
    else if (touch_release)
    {
        //
        // touch release
        //
        if ( g_s_last_event.mouse_state == MOUSE_STATE_LEFT_CLICKED)
        {
            event->mouse_event         = MOUSE_EVENT_LEFT_RELEASE;
            g_s_last_event.mouse_event = event->mouse_event;
            i32WaitEvent = 1;
        }
        event->mouse_state = MOUSE_STATE_NONE;
        g_s_last_event.mouse_state = event->mouse_state;
        touch_release = false;
    }

    if ( timer1_passed > 0 )
    {
        event->timer_id          = TIMER_1;
        event->timer_expirations = timer1_passed;
        timer1_passed = 0;
        i32WaitEvent = 1;
    }

    return i32WaitEvent;
#else

    do {
        if ( timer1_passed != 0U )
        {
            semaphore_take();
            uint32_t tmp_timer_passed = timer1_passed;
            timer1_passed = 0;
            semaphore_give();

            event->timer_id          = TIMER_1;
            event->timer_expirations = tmp_timer_passed;
            ++i32WaitEvent;
        }
        else
        {
            if ( block_until_event != 0 )
            {
                wait_for_timer();
            }
        }
    } while ( (block_until_event != 0) && (i32WaitEvent <= 0) );

    if (i32WaitEvent > 0)
    {
        return 1;
    }
#endif
    return 0;
}

#endif // BAREMETAL
