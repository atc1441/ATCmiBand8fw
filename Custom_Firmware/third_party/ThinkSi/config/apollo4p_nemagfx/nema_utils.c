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
//                    Think Silicon S.A.
//                    http://www.think-silicon.com
//                    Patras Science Park
//                    Rion Achaias 26504
//                    Greece
// -----------------------------------------------------------------------------

#include <string.h>
#ifndef BAREMETAL
#include "FreeRTOS.h"
#include "task.h"
#endif
#include "nema_utils.h"
#include "nema_blender.h"
#include "nema_cmdlist.h"

#include "am_util_stdio.h"
#include "am_mcu_apollo.h"
#include "nema_regs.h"

#define NEMA_MAX_SINGLE_TRANSFER            0x200

#define STIME_FREQUENCY										1500000.0f
//returns time in seconds
float nema_get_time(void)
{
#ifdef BAREMETAL
    static uint32_t count_last = 0;
    static float last_sec = 0;
    float full_scale_sec,timer_offset;
    uint32_t ui32TimerCount;
#if defined(APOLLO4_FPGA) && defined(AM_PART_APOLLO4P)
    //
    // use stimer
    //
    ui32TimerCount = am_hal_stimer_counter_get();
    full_scale_sec = (float)0xFFFFFFFF / STIME_FREQUENCY;
    timer_offset = (float)ui32TimerCount / STIME_FREQUENCY;
    if (count_last > ui32TimerCount)
    {
        // overflow
        last_sec += full_scale_sec;
    }
    count_last = ui32TimerCount;
    return last_sec + timer_offset;
#else
    //
    // use timer
    //
    float fTimerInputFreq;
	ui32TimerCount = am_hal_timer_read(0); //!< This API only supports timer0.

    uint32_t ui32DivideRatio, ui32Timer0Clock;
    ui32Timer0Clock = TIMER->CTRL0_b.TMR0CLK;
    if ((ui32Timer0Clock >= 1) || (ui32Timer0Clock <= 5))
    {
        //
        // Calculate divide ratio
        // TIMER_CTRL0_TMR0CLK_HFRC_DIV16       = 1,
        // TIMER_CTRL0_TMR0CLK_HFRC_DIV64       = 2,
        // TIMER_CTRL0_TMR0CLK_HFRC_DIV256      = 3,
        // TIMER_CTRL0_TMR0CLK_HFRC_DIV1024     = 4,
        // TIMER_CTRL0_TMR0CLK_HFRC_DIV4K       = 5,
        //
        ui32DivideRatio = (1U << ((ui32Timer0Clock + 1) << 1));
    }
    else
    {
        return 0; //!< This API only supports HFRC as timer input clock.
    }
    fTimerInputFreq = (float)AM_HAL_CLKGEN_FREQ_MAX_HZ / (float)ui32DivideRatio;
    full_scale_sec = (float)0xFFFFFFFF / fTimerInputFreq;
    timer_offset = (float)ui32TimerCount / fTimerInputFreq;
    if (count_last > ui32TimerCount)
    {
        // overflow
        last_sec += full_scale_sec;
    }
    count_last = ui32TimerCount;
    return last_sec + timer_offset;

#endif
#else
	float sec;

    TickType_t ticks = xTaskGetTickCount();

    sec = (float)ticks/(float) configTICK_RATE_HZ;

    return sec;
#endif

}

static unsigned long s[] = {123456789, 362436069};
unsigned int nema_rand()
{
    uint64_t x = s[0];
    uint64_t const y = s[1];
    s[0] = y;
    x ^= x << 23; // a
    s[1] = x ^ y ^ (x >> 17) ^ (y >> 26); // b, c

    return s[1] + y;
}

nema_buffer_t nema_load_file(const char *filename, int length, void *buffer)
{
  nema_buffer_t bo = {0};
  return bo;
}


int nema_save_file(const char *filename, int length, void *buffer)
{
    return 0;
}


// returns wall time in useconds
float nema_get_wall_time(void)
{
    return nema_get_time();
}

void nema_calculate_fps(void)
{
    static int   frame      = 0;

    ++frame;
    if ( frame%100 == 0 ) {
        static float start_time = 0.f;
        static float stop_time  = 0.f;

        stop_time = nema_get_time();
        am_util_stdio_printf("\nfps: %.02f\n", 100.f/(stop_time-start_time));
        start_time = stop_time;
    }
}

//
// Maximum num is 0x7FFF, return NULL if larger than 0x7FFF
//
static
void *nema_gpu_memcpy ( void * destination, const void * source, size_t num )
{
    img_obj_t sFB;
    img_obj_t sObjSourceRGB332;
    nema_cmdlist_t *psCLLast;
    static nema_cmdlist_t sCL;
    static bool bFlag = false;

    if (num > NEMA_MAX_SINGLE_TRANSFER)
    {
        return NULL;
    }

    if (nema_reg_read(NEMA_STATUS) != 0U)
    {
        return NULL;
    }

    psCLLast = nema_cl_get_bound();

    if (bFlag == false)
    {
        bFlag = true;
        sCL = nema_cl_create();
    }
    else
    {
        nema_cl_rewind(&sCL);
    }

    nema_cl_bind(&sCL);

    sFB.bo.base_phys = (uintptr_t)destination;
    sFB.bo.base_virt = (void*)sFB.bo.base_phys;
    sFB.bo.fd = 0;
    sFB.bo.size = num;
    sFB.w = sFB.bo.size;
    sFB.h = 1;
    sFB.stride = sFB.w;
    sFB.color = 0;
    sFB.format = NEMA_RGB332;
    sFB.sampling_mode = 0;
    nema_bind_dst_tex(sFB.bo.base_phys, sFB.w, sFB.h, sFB.format, sFB.stride);

    sObjSourceRGB332.bo.base_virt = (void*)source;
    sObjSourceRGB332.bo.base_phys = (uintptr_t)sObjSourceRGB332.bo.base_virt;
    sObjSourceRGB332.bo.fd = 0;
    sObjSourceRGB332.bo.size = sFB.bo.size;
    sObjSourceRGB332.w = sFB.bo.size;
    sObjSourceRGB332.h = sFB.h;
    sObjSourceRGB332.stride = sFB.stride;
    sObjSourceRGB332.format = sFB.format;
    nema_bind_src_tex(sObjSourceRGB332.bo.base_phys,
                      sObjSourceRGB332.w,
                      sObjSourceRGB332.h,
                      sObjSourceRGB332.format,
                      sObjSourceRGB332.stride,
                      NEMA_FILTER_PS);
    nema_set_clip(0, 0, sFB.bo.size, sFB.h);
    nema_set_blend_blit(NEMA_BL_SRC);
    nema_blit(0, 0);

    nema_cl_submit(&sCL);
    nema_cl_wait(&sCL);

    nema_cl_unbind();

    if (psCLLast != NULL)
    {
        nema_cl_bind(psCLLast);
    }

    return destination;
}

void *nema_memcpy ( void * destination, const void * source, size_t num )
{
    uint32_t ui32MaxLoops;
    uint32_t ui32LastNum;
    uint32_t ui32CurrentLoop;
    uint8_t *pui8CurrentDestination = (uint8_t*)destination;
    uint8_t *pui8CurrentSource = (uint8_t*)source;

    ui32MaxLoops = num / NEMA_MAX_SINGLE_TRANSFER;
    ui32LastNum = num % NEMA_MAX_SINGLE_TRANSFER;

    for (ui32CurrentLoop = 0; ui32CurrentLoop < ui32MaxLoops; ui32CurrentLoop++)
    {
        nema_gpu_memcpy(pui8CurrentDestination, pui8CurrentSource, NEMA_MAX_SINGLE_TRANSFER);
        pui8CurrentDestination += NEMA_MAX_SINGLE_TRANSFER;
        pui8CurrentSource += NEMA_MAX_SINGLE_TRANSFER;
    }

    if (ui32LastNum > 0)
    {
        nema_gpu_memcpy(pui8CurrentDestination, pui8CurrentSource, ui32LastNum);
    }

    return destination;
}

