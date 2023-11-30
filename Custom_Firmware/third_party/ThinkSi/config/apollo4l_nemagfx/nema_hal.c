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


#ifdef BAREMETAL

#  ifndef WAIT_IRQ_POLL
#    define WAIT_IRQ_POLL               1
#  endif

#else  // BAREMETAL

#  ifndef WAIT_IRQ_POLL
#    define WAIT_IRQ_POLL               0
#  endif

#  ifndef WAIT_IRQ_BINARY_SEMAPHORE
#    define WAIT_IRQ_BINARY_SEMAPHORE   0
#  endif
#ifdef SYSTEM_VIEW
#  include "SEGGER_SYSVIEW_FreeRTOS.h"
#endif
#  include "FreeRTOS.h"
#  include "portable.h"
#  include "task.h"
#  include "event_groups.h"
#endif // BAREMETAL

//#include "interpose.h"
#include "nema_hal.h"
#include "nema_regs.h"
#include "nema_graphics.h"
#include "nema_ringbuffer.h"
#include "am_mcu_apollo.h"
#include "am_hal_global.h"
#include "am_util_delay.h"

#include <stdlib.h>

#ifdef NEMA_USE_CUSTOM_MALLOC
#include NEMA_CUSTOM_MALLOC_INCLUDE
#else
#include "tsi_malloc.h"

#ifndef VMEM_BASEADDR
#define VMEM_BASEADDR       tsi_buffer
#endif   //VMEM_BASEADDR

#ifndef VMEM_SIZE
#define VMEM_SIZE           (0xF0000)
#endif //VMEM_SIZE

#if VMEM_BASEADDR==tsi_buffer
static AM_SHARED_RW uint64_t tsi_buffer[VMEM_SIZE/8];
#endif   //VMEM_BASEADDR==tsi_buffer
#endif

#ifndef NEMA_BASEADDR
#include "apollo4l.h"
#define NEMA_BASEADDR       GPU_BASE
#endif

// IRQ number
#ifndef NEMA_IRQ
#define NEMA_IRQ            ((IRQn_Type)28U)
#endif

// MAX pending CL in the core ring buffer
#ifndef MAX_PENDING_CL
#define MAX_PENDING_CL (200UL)
#endif

#if (MAX_PENDING_CL < 10)
#error "max pending CL must be bigger than 10"
#endif

// IRQ handler
//#define prvNemaInterruptHandler     am_gpu_isr

#if (defined(NEMA_MULTI_PROCESS) || defined(NEMA_MULTI_THREAD))
static int enable_mutices = 0;
#endif

static const uintptr_t nema_regs = (uintptr_t) NEMA_BASEADDR;

#if (defined(NEMA_MULTI_PROCESS) || defined(NEMA_MULTI_THREAD))
    #include "semphr.h"
    static SemaphoreHandle_t xMutex[MUTEX_MAX+1] = {NULL};
#endif

#ifndef BAREMETAL
#if WAIT_IRQ_BINARY_SEMAPHORE == 1

    #include "semphr.h"
    static SemaphoreHandle_t xSemaphore = NULL;

#else
    #include "task.h"
    static TaskHandle_t xHandlingTask = 0;

#endif
#endif /* BAREMETAL */

static volatile int last_cl_id = -1;
static volatile bool building_cl = false;
static nema_ringbuffer_t ring_buffer_str = {{0}};

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
#define GPU_ON_WORKAROUND                       0

//
// GFX Power workarounf user a TIMER GPU_PWR_WA_TIMER_NUM and interrupt after (GPU_PWR_WA_TIMER_PERIOD)us
//
#define MCUL_RESET_TIMER_NUM                    10
#define MCUL_RESET_TIMER_PERIOD                 50   

#define MCUL_RESET_DEBUG                        1

#if MCUL_RESET_DEBUG
#  define DEBUG_REG(v)                          (*(volatile uint32_t *)0x4ffff000) = (v)
#else
#  define DEBUG_REG(v)
#endif

#if MCUL_RESET_DEBUG
#define NEMA_ASSERT(x)                           if (( x ) == 0) while(1);
#else
#define NEMA_ASSERT(...)
#endif

#define FORCE_AXI_CLK                           *((uint32_t *)0x40040284)
#define APB_SYNC                                *(volatile uint32_t*)(0x47FF0000)
//*****************************************************************************
//
//! gpu power ctrl flags to indicate ongoing DMA xfers and user request status
//
//*****************************************************************************
typedef struct
{
    __IOM uint32_t CTRL;                        //GPU Power Control Flags.
}GPU_POWER_CTRL_Type;

static GPU_POWER_CTRL_Type g_gpu_power_ctrl;

//
//! GPU power ctrl enum.
//
typedef enum
{
    NEMA_GPU_POWER_DOWN  = 0 ,
    NEMA_GPU_POWER_UP    = 1 ,
} nema_gpu_pwrctrl_control_e;


#if MCUL_RESET_DEBUG
uint32_t g_startCNT     = 1;
uint32_t g_endCNT       = 1;
#endif

//*****************************************************************************
//
//! NVIC backup/restore status 
//
//*****************************************************************************
static uint32_t nvic_en[8];
static uint32_t nvic_pending[8];

//*****************************************************************************
//
// Function to initialize the timer for the GPU power workaround.
//
//*****************************************************************************
RAMFUNC static uint32_t
mcul_reset_timer_init(uint32_t ui32Delayus)
{
    am_hal_timer_config_t  TimerConfig;
    uint32_t ui32ConfigCtrl, ui32ConfigMode;
    //
    // Configure the timer
    //
    TimerConfig.eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV16;
    TimerConfig.eFunction = AM_HAL_TIMER_FN_EDGE;
    TimerConfig.ui32Compare0 = 0xFFFFFFFF;
    TimerConfig.ui32Compare1 = 0xFFFFFFFF;
    TimerConfig.bInvertOutput0 = false;
    TimerConfig.bInvertOutput1 = false;
    TimerConfig.eTriggerType = AM_HAL_TIMER_TRIGGER_DIS;
    TimerConfig.eTriggerSource = AM_HAL_TIMER_TRIGGER_TMR0_OUT1;
    TimerConfig.ui32PatternLimit = 0;

    TimerConfig.ui32Compare0 = ui32Delayus * (192 / 16) / 2;

    //
    //
    // Build up a value in SRAM before we start writing to the timer control
    // registers.
    //
    ui32ConfigCtrl  = _VAL2FLD(TIMER_CTRL0_TMR0CLK,     TimerConfig.eInputClock);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0FN,      TimerConfig.eFunction);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0POL1,    TimerConfig.bInvertOutput1);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0POL0,    TimerConfig.bInvertOutput0);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0TMODE,   TimerConfig.eTriggerType);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0LMT,     TimerConfig.ui32PatternLimit);
    ui32ConfigCtrl |= _VAL2FLD(TIMER_CTRL0_TMR0EN, 0);

    ui32ConfigMode  = _VAL2FLD(TIMER_MODE0_TMR0TRIGSEL, TimerConfig.eTriggerSource);

    //
    // Disable the timer.
    //
    TIMERn(MCUL_RESET_TIMER_NUM)->CTRL0_b.TMR0EN = 0;

    //
    // Apply the settings from the configuration structure.
    //
    TIMERn(MCUL_RESET_TIMER_NUM)->CTRL0 = ui32ConfigCtrl;
    TIMERn(MCUL_RESET_TIMER_NUM)->MODE0 = ui32ConfigMode;
    TIMERn(MCUL_RESET_TIMER_NUM)->TMR0CMP0 = TimerConfig.ui32Compare0;
    TIMERn(MCUL_RESET_TIMER_NUM)->TMR0CMP1 = TimerConfig.ui32Compare1;

    //
    // Clear the timer to make sure it has the appropriate starting value.
    //
    TIMERn(MCUL_RESET_TIMER_NUM)->CTRL0_b.TMR0CLR = 1;
        
    //am_hal_timer_clear(MCUL_RESET_TIMER_NUM);
    //
    // Disable the timer.
    //
    TIMERn(MCUL_RESET_TIMER_NUM)->CTRL0_b.TMR0EN = 0;

    //
    // Clear the timer.
    //
    TIMERn(MCUL_RESET_TIMER_NUM)->CTRL0_b.TMR0CLR = 1;
    TIMERn(MCUL_RESET_TIMER_NUM)->CTRL0_b.TMR0CLR = 0;

    //
    // Clear the timer Interrupt
    //
    TIMER->INTCLR = AM_HAL_TIMER_MASK(MCUL_RESET_TIMER_NUM, AM_HAL_TIMER_COMPARE0);
    //
    // Enable the timer Interrupt.
    //
    TIMER->INTEN |= AM_HAL_TIMER_MASK(MCUL_RESET_TIMER_NUM, AM_HAL_TIMER_COMPARE0);
    //
    // Enable the timer interrupt in the NVIC.
    //
    // This interrupt needs to be set as the highest priority (0)
    //
    NVIC->IP[TIMER0_IRQn + MCUL_RESET_TIMER_NUM ] = (uint8_t)((0 << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
    
    return 0;
}

//*****************************************************************************
//
//! shutdown gpu power workaround timer
//
//*****************************************************************************
RAMFUNC static void 
mcul_reset_timer_deinit(void)
{
     uint32_t ui32IntStat;
    //
    // Clear the interrupt that got us here.
    //
    DEBUG_REG(0xfefe0001);    
    //
    // Clear the timer Interrupt
    //
    TIMER->INTCLR = AM_HAL_TIMER_MASK(MCUL_RESET_TIMER_NUM, AM_HAL_TIMER_COMPARE0);
    //
    // Before clearing the NVIC pending, avoid a race condition by
    // making sure the interrupt clear has propogated by reading
    // the INTSTAT register.
    //
    ui32IntStat = TIMER->INTSTAT ;
    ui32IntStat = ui32IntStat & TIMER->INTEN;
    //
    // Clear pending NVIC interrupt for the timer-specific IRQ.
    //
    NVIC->ICPR[(((uint32_t)TIMER0_IRQn + MCUL_RESET_TIMER_NUM) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)TIMER0_IRQn + MCUL_RESET_TIMER_NUM) & 0x1FUL));
    //
    // There is also a pending on the timer common IRQ. But it should
    // only be cleared if the workaround timer is the only interrupt.
    //
    if ( !(ui32IntStat &
                ~AM_HAL_TIMER_MASK(MCUL_RESET_TIMER_NUM, AM_HAL_TIMER_COMPARE0)) )
    {
        NVIC->ICPR[(((uint32_t)TIMER_IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)TIMER_IRQn) & 0x1FUL));
        //
        // One more race to consider.
        // If a different timer interrupt occurred while clearing the
        // common IRQ, set the timer common IRQ back to pending.
        //
        ui32IntStat = TIMER->INTSTAT ;
        ui32IntStat = ui32IntStat & TIMER->INTEN;
        if ( ui32IntStat &
                ~AM_HAL_TIMER_MASK(MCUL_RESET_TIMER_NUM, AM_HAL_TIMER_COMPARE0) )
        {
            NVIC->ISPR[(((uint32_t)TIMER_IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)TIMER_IRQn) & 0x1FUL));
        }
    }

} // am_timer00_isr()

//*****************************************************************************
//
//! @brief Function for the GPU power workaround
//!
//! @note  It resets MCUL power domain after turning on or off GPU
//!
//! @param on - 1 for GPU power on, 0 for off 
//!
//! @return void
//
//*****************************************************************************
RAMFUNC uint32_t 
gpu_power_timer_workaround(nema_gpu_pwrctrl_control_e eControl)
{
    uint32_t origBasePri;
    uint32_t basePrioGrouping;
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    basePrioGrouping = ((uint32_t)((SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) >> SCB_AIRCR_PRIGROUP_Pos));
    if (basePrioGrouping == 7)
    {
        //
        // We cannot implement this workaround
        //
        ui32Status = AM_HAL_STATUS_FAIL;
    }
    else
    {
        //
        // Mask off all other interrupts
        //
        origBasePri = __get_BASEPRI();
        if (basePrioGrouping >= (8 - __NVIC_PRIO_BITS))
        {
            __set_BASEPRI(1 << (basePrioGrouping + 1));
        }
        else
        {
            __set_BASEPRI(1 << (8 - __NVIC_PRIO_BITS));
        }
        
        MCUCTRL->APBDMACTRL_b.DMAENABLE = 0; // disable DMA

#if MCUL_RESET_DEBUG
        CPU->CACHECFG_b.ENABLEMONITOR = 1;
#endif            
        FORCE_AXI_CLK = 1; // force AXI clock on   
        
        //
        //  Disable MMS override for MCUL on by PD_GFX setting. 
        //  
        PWRCTRL->MMSOVERRIDE_b.MMSOVRMCULGFX = 1;        
        
        ui32Status = mcul_reset_timer_init(MCUL_RESET_TIMER_PERIOD);
        if ( ui32Status != AM_HAL_STATUS_SUCCESS )
        {
        return ui32Status;
        }
    
        TIMER->GLOBEN |= 1 <<  MCUL_RESET_TIMER_NUM;
        //
        // Toggle the clear bit (required by the hardware), and then enable the timer.
        //
        TIMERn(MCUL_RESET_TIMER_NUM)->CTRL0_b.TMR0CLR = 1;
        TIMERn(MCUL_RESET_TIMER_NUM)->CTRL0_b.TMR0CLR = 0;
        
        DEBUG_REG(0xEEE01251); 
        
        SCB->SCR |= _VAL2FLD(SCB_SCR_SLEEPDEEP, 1);

        // daxi flush
        //
        // Call DSB
        //
        __DSB();
        //
        // Call DAXI Flush
        //
        CPU->DAXICTRL_b.DAXIFLUSHWRITE = 1;
        //
        // APB_SYNC
        //
        APB_SYNC;
        //
        // Call DAXI Invalidate
        //
        CPU->DAXICTRL_b.DAXIINVALIDATE = 1;

#if MCUL_RESET_DEBUG
        //
        // Set for detection of MCUL reset
        //
        CPU->CACHECFG_b.ENABLEMONITOR = 0;
        SECURITY->SRCADDR = 0x1234ABCD;
#endif

        //
        // APB_SYNC
        //
        APB_SYNC;

        //
        // Start timer
        //
        TIMERn(MCUL_RESET_TIMER_NUM)->CTRL0_b.TMR0EN = 1;

#if MCUL_RESET_DEBUG
        g_startCNT = TIMERn(MCUL_RESET_TIMER_NUM)->TIMER0_b.TIMER0;
#endif    

        //
        // Power on or off GPU
        //
        if (eControl == NEMA_GPU_POWER_DOWN)
        {     
            PWRCTRL->DEVPWREN_b.PWRENGFX = 0; 
        }
        else if (eControl == NEMA_GPU_POWER_UP)
        {
            PWRCTRL->DEVPWREN_b.PWRENGFX = 1; 
        }
    
        //
        // NVIC backup
        //
        for (uint32_t i = 0; i < 4; i++ ) 
        {
            nvic_en[i] = NVIC->ISER[i];
            NVIC->ICER[i] = nvic_en[i];

            nvic_pending[i] = NVIC->ISPR[i];
            NVIC->ICPR[i] = nvic_pending[i];
        }

        //
        // Set the timer interrupt
        //
        NVIC->ISER[(TIMER0_IRQn + MCUL_RESET_TIMER_NUM) / 32] = (1 << ((TIMER0_IRQn + MCUL_RESET_TIMER_NUM) % 32));
        
        __WFI();
        __ISB();

        //
        // NVIC restore
        //
        for (uint32_t i = 0; i < 4; i++ ) 
        {
            NVIC->ISER[i] = nvic_en[i]; 
            NVIC->ISPR[i] = nvic_pending[i];
        }
        DEBUG_REG(0xEEE01255);

#if MCUL_RESET_DEBUG
        //
        // Check the timer time
        //
        g_endCNT = TIMERn(MCUL_RESET_TIMER_NUM)->TIMER0_b.TIMER0;
        //
        // check if MCUL reset working
        //
        while (SECURITY->SRCADDR != 0);
        DEBUG_REG(0xEEE01256);
#endif

        //
        // shut down timer 
        //
        TIMERn(MCUL_RESET_TIMER_NUM)->CTRL0_b.TMR0EN = 0; 

        mcul_reset_timer_deinit();
        //
        // APB Sync
        //
        APB_SYNC; 
        DEBUG_REG(0xEEE01257); 
         //
        //  restore default value of MMS override for MCUL on by PD_GFX setting. 
        // 
        PWRCTRL->MMSOVERRIDE_b.MMSOVRMCULGFX = 0;           

        MCUCTRL->APBDMACTRL_b.DMAENABLE = 1; // enable DMA
        FORCE_AXI_CLK = 0; // clear force AXI clock 

        //
        // Restore interrupts
        //        
        __set_BASEPRI(origBasePri);    
    } 

    return ui32Status;
}


//*****************************************************************************
//
//! @brief this function used to disable GPU power
//!
//! @note after called this function,please don't operation any GPU registers
//! until enable and initial the GPU.
//!
//! @return 1- GPU power had disabled before
//!         0- GPU power have disabled now.
//
//*****************************************************************************
static int32_t
_gpu_power_down(void)
{
    bool status;
    bool bBackToLpMode = false;
    uint32_t rpt_count;
    
    am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_GFX, &status);
    if ( status )
    {
        // wait nema busy bits all cleared. 
        while((nema_reg_read(NEMA_STATUS) != 0) || (nema_reg_read(NEMA_CMDSTATUS) != 0))
        {
            am_hal_delay_us(1);     
            DEBUG_REG(0xEEE01240); 
        }

        //
        // Deep-sleep Jlink issue workaround
        //
        if ( PWRCTRL->MCUPERFREQ_b.MCUPERFSTATUS != AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE )
        {
            am_hal_pwrctrl_mcu_mode_select(AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE);
            bBackToLpMode = true;
        }
        else
        {
            bBackToLpMode = false;
        }

        rpt_count = 0;
        DEBUG_REG(0xEEE01234); 

        //
        // Begin critical section
        //
        AM_CRITICAL_BEGIN

        if ( gpu_power_timer_workaround(NEMA_GPU_POWER_DOWN) != AM_HAL_STATUS_SUCCESS)
        {
            DEBUG_REG(0xEEE01236);
#if MCUL_RESET_DEBUG        
            while(1);
#endif       
        }
        //
        // End critical section
        //
        AM_CRITICAL_END

        while(PWRCTRL->DEVPWRSTATUS_b.PWRSTGFX == 1)
        {
            am_hal_delay_us(1);
            rpt_count++;
            if (rpt_count >= 1000000)
            {
                DEBUG_REG(0xfff01234); 
#if MCUL_RESET_DEBUG                     
                while(1);
#else
                break;
#endif                                        
            }
        }
        DEBUG_REG(0xEEE01237); 

        //
        // Deep-sleep jlink issue workaround
        //
        if ( bBackToLpMode == true )
        {
            am_hal_pwrctrl_mcu_mode_select(AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER);
        }
    }
    else
    {
        return 1;
    }
    //am_hal_delay_us(300);
    DEBUG_REG(0x4567fff0);

    return 0;
}
//*****************************************************************************
//
//! @brief this function used to enable GPU power and initialize nemaGFX
//!
//! @return 0- GPU power have initialize completely.
//!         other- GPU power initialize error.
//
//*****************************************************************************
static int32_t
_gpu_power_up(void)
{
    bool status;
    int32_t i32Ret;
#if GPU_ON_WORKAROUND    
    uint32_t rpt_count = 0;
    bool bBackToLpMode = false;
        
    am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_GFX, &status);
    if ( !status )
    {
        //
        // Deep-sleep jlink issue workaround
        //
        if ( PWRCTRL->MCUPERFREQ_b.MCUPERFSTATUS != AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE )
        {
            am_hal_pwrctrl_mcu_mode_select(AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE);
            bBackToLpMode = true;
        }
        else
        {
            bBackToLpMode = false;
        }

        rpt_count = 0;
        DEBUG_REG(0xEEE01234); 
        //
        // Begin critical section
        //
         AM_CRITICAL_BEGIN
        if ( gpu_power_timer_workaround(NEMA_GPU_POWER_UP) != AM_HAL_STATUS_SUCCESS)
        {
            DEBUG_REG(0xEEE01237); 
#if MCUL_RESET_DEBUG                     
            while(1);
#endif           
        }
        //
        // End critical section
        //
        AM_CRITICAL_END

        while(PWRCTRL->DEVPWRSTATUS_b.PWRSTGFX == 0)
        {
            am_hal_delay_us(1);
            rpt_count++;
            if (rpt_count >= 1000000)
            {
                DEBUG_REG(0xfff01234); 
#if MCUL_RESET_DEBUG                     
                while(1);
#else
                break;
#endif                                        
            }
        }
        DEBUG_REG(0xEEE01237); 

        //
        // Deep-sleep jlink issue workaround
        //
        if ( bBackToLpMode == true )
        {
            am_hal_pwrctrl_mcu_mode_select(AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER);
        }

        DEBUG_REG(0x789a0002);
        //
        // Initialize NemaGFX
        //
        if (rpt_count < 1000000)
          i32Ret = nema_init();
        else
          i32Ret = 1;
    }
#else  // GPU_ON_WORKAROUND
    am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_GFX, &status);
    if ( !status )
    {
        am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_GFX);
        //
        // Initialize NemaGFX
        //
        i32Ret = nema_init();
    }
#endif  // GPU_ON_WORKAROUND    
    else
    {
        i32Ret = 0;
    }
    DEBUG_REG(0x789affff);
    return i32Ret;
}
//*****************************************************************************
//
//! @brief this function used to enable GPU power
//!
//! @param gfxPowerCtrl  GPU power ctrl enums.
//!
//! @note Turn on GPU when any DMA transaction starts running, or when the user 
//!       requests to turn on GPU
//!
//
//*****************************************************************************
void am_gpu_power_enable(uint32_t gfxPowerCtrl)
{
    AM_CRITICAL_BEGIN
    NEMA_ASSERT(gfxPowerCtrl < AM_GPU_PWRCTRL_MAX);
    g_gpu_power_ctrl.CTRL |= 1 << gfxPowerCtrl;  
  
    if (PWRCTRL->DEVPWRSTATUS_b.PWRSTGFX == 0)
    {
        _gpu_power_up();
    }    
    AM_CRITICAL_END
}

//*****************************************************************************
//
//! @brief this function is for the user to disable GPU power
//!
//! @note the actual power off is not done here, this function sets a flag
//!       to indicate the user wants to do turn off GPU
//!
//! @param gfxPowerCtrl - the bit to set for GFX power control 
//!
//! @return none
//
//*****************************************************************************
void am_gpu_power_disable(uint32_t gfxPowerCtrl)
{
    AM_CRITICAL_BEGIN
    NEMA_ASSERT(gfxPowerCtrl < AM_GPU_PWRCTRL_MAX);
    g_gpu_power_ctrl.CTRL &= ~(1 << gfxPowerCtrl);
    AM_CRITICAL_END
}

//*****************************************************************************
//
//! @brief check and turn off GPU
//!
//! @note This function should be called peridically
//!
//! @return void
//
//*****************************************************************************
void am_gpu_power_check_and_disable(void)
{
    if (PWRCTRL->DEVPWRSTATUS_b.PWRSTGFX == 1)
    {
        AM_CRITICAL_BEGIN   
       if ((g_gpu_power_ctrl.CTRL == 0) && 
           (!(PWRCTRL->DEVPWRSTATUS_b.PWRSTUART0) || (PWRCTRL->DEVPWRSTATUS_b.PWRSTUART0 && UARTn(0)->FR_b.TXFE)) &&
           (!(PWRCTRL->DEVPWRSTATUS_b.PWRSTUART1) || (PWRCTRL->DEVPWRSTATUS_b.PWRSTUART1 && UARTn(1)->FR_b.TXFE)) &&
           (!(PWRCTRL->DEVPWRSTATUS_b.PWRSTUART2) || (PWRCTRL->DEVPWRSTATUS_b.PWRSTUART2 && UARTn(2)->FR_b.TXFE)) &&
           (!(PWRCTRL->DEVPWRSTATUS_b.PWRSTUART3) || (PWRCTRL->DEVPWRSTATUS_b.PWRSTUART3 && UARTn(3)->FR_b.TXFE)) &&
           (!(PWRCTRL->DEVPWRSTATUS_b.PWRSTADC) || (PWRCTRL->DEVPWRSTATUS_b.PWRSTADC && !(ADC->CFG_b.ADCEN))) &&
           (!(PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO)) &&
           (!(PWRCTRL->DEVPWRSTATUS_b.PWRSTDBG)) )
        {
            _gpu_power_down();
        }
        AM_CRITICAL_END
    }
}

//*****************************************************************************
//
// example timer in FreeRTOS for peridical checking the GPU power-off status
//
//*****************************************************************************
#ifdef BAREMETAL
#define PERIODICALL_TASK_DELAY      10000      // How often to run the periodicall task(in us). default is 10ms
#define PERIODICALl_TIMER_NUM       11
//******************************************************************************
//
// TIMER ISR
//
//******************************************************************************
#define periodicall_task_isr                                            \
    am_timer_isr1(PERIODICALl_TIMER_NUM)
#define am_timer_isr1(n)                                                \
    am_timer_isr(n)
#define am_timer_isr(n)                                                 \
    am_timer ## n ## _isr

void periodicall_task_isr(void)
{
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(PERIODICALl_TIMER_NUM, AM_HAL_TIMER_COMPARE1));
    am_hal_timer_clear(PERIODICALl_TIMER_NUM);    
    am_gpu_power_check_and_disable();

} // am_ctimer_isr()

uint32_t gpu_power_periodicall_task_init(void)
{
    am_hal_timer_config_t  TimerConfig;
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    //
    // Configure the timer
    //
    am_hal_timer_default_config_set( &TimerConfig ) ;
    TimerConfig.ui32Compare1 = PERIODICALL_TASK_DELAY * 6 ;     // Default clock is 6MHz
    am_hal_timer_config(PERIODICALl_TIMER_NUM, &TimerConfig);
    am_hal_timer_clear(PERIODICALl_TIMER_NUM);
    am_hal_timer_stop(PERIODICALl_TIMER_NUM);

    //
    // Clear the timer Interrupt
    //
    ui32Status = am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(PERIODICALl_TIMER_NUM, AM_HAL_TIMER_COMPARE1));
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Status;
    }

    //
    // Enable the timer Interrupt.
    //
    ui32Status = am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(PERIODICALl_TIMER_NUM, AM_HAL_TIMER_COMPARE1));
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
       return ui32Status;
    }

    //
    // Enable the timer interrupt in the NVIC.
    //
    // This interrupt cannnot be set to the highest priority (0)
    //
    NVIC_SetPriority((IRQn_Type)((uint32_t)TIMER0_IRQn + PERIODICALl_TIMER_NUM), 1);
    NVIC_EnableIRQ((IRQn_Type)((uint32_t)TIMER0_IRQn + PERIODICALl_TIMER_NUM));

    am_hal_timer_enable(PERIODICALl_TIMER_NUM);
    return ui32Status;
}
#else

static TimerHandle_t gpuPowerCheckSwtmrHandle = NULL;

//*****************************************************************************
//
// timer callback for periodicall checking the GPU power-off status
//
//*****************************************************************************
static void gpuPowerCheckSwtmr_Callback(void* parameter)
{
    am_gpu_power_check_and_disable();  
}

//*****************************************************************************
//
// timer creation for periodicall checking the GPU power-off status
//
//*****************************************************************************
void gpu_power_check_and_set_task(void)
{  
  gpuPowerCheckSwtmrHandle = xTimerCreate( (const char*)"GpuPowerCheckTimer",
                                            (TickType_t)10,                     // The timer period is 10ms
                                            (UBaseType_t)pdTRUE,                // uxAutoReload       
                                            (void*)1,
                                            (TimerCallbackFunction_t)gpuPowerCheckSwtmr_Callback);
  if (gpuPowerCheckSwtmrHandle != NULL) 
  {
       xTimerStart(gpuPowerCheckSwtmrHandle, 0);     
  }   
}
#endif

//*****************************************************************************
//
//! @brief this function used to disable GPU power
//!
//! @note after called this function,please don't operation any GPU registers
//! until enable and initial the GPU.
//!
//! @return 0- GPU power have disabled now.
//!         
//
//*****************************************************************************
int32_t
nema_gfx_power_down(void)
{
    am_gpu_power_disable(AM_GPU_PWRCTRL_USER);
    return 0;
}

//*****************************************************************************
//
//! @brief this function used to enable GPU power and initialize nemaGFX
//!
//! @return 0- GPU power have initialize completely.
//!         other- GPU power initialize error.
//
//*****************************************************************************
int32_t
nema_gfx_power_up(void)
{  
    am_gpu_power_enable(AM_GPU_PWRCTRL_USER);
    return 0;
}

//*****************************************************************************
//
//! @brief Check GPU status and power down if it is idle
//!
//! @return
//
//*****************************************************************************
void
nema_gfx_check_busy_and_suspend(void)
{
    if((last_cl_id == ring_buffer_str.last_submission_id) && (building_cl == false) )
    {
        nema_gfx_power_down();
    }
}

//*****************************************************************************
//
//! @brief Check wether the core ring buffer is full or not
//!
//! @return True, the core ring buffer is full, we need wait for GPU before 
//!         submit the next CL.
//!         False, the core ring buffer is not full, we can submit the next CL.
//*****************************************************************************
bool nema_rb_check_full(void)
{
    uint32_t total_pending_cl = 0;
    if(ring_buffer_str.last_submission_id > last_cl_id)
    {
        total_pending_cl = ring_buffer_str.last_submission_id - last_cl_id;
    }
    else if(ring_buffer_str.last_submission_id < last_cl_id)
    {
        if(last_cl_id == 0xFFFFFF)
        {
            total_pending_cl = ring_buffer_str.last_submission_id + 1;
        }
        else
        {
            //should never got here.
        }
    }

    return (total_pending_cl >= MAX_PENDING_CL);
}

//*****************************************************************************
//
// GPU interrupt callback
//
//*****************************************************************************
static void prvNemaInterruptHandler( void *pvUnused )
{
    /* Clear the interrupt */
    nema_reg_write(NEMA_INTERRUPT, 0);
#ifndef BAREMETAL
#ifdef SYSTEM_VIEW
    traceISR_ENTER();
#endif

    BaseType_t xHigherPriorityTaskWoken;

#if WAIT_IRQ_BINARY_SEMAPHORE == 1
    xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
#else
    if ( xHandlingTask )
    {
        xTaskNotifyFromISR( xHandlingTask,
                            0,
                            eNoAction,
                            &xHigherPriorityTaskWoken );
    }

    /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
    should be performed to ensure the interrupt returns directly to the highest
    priority task.  The macro used for this purpose is dependent on the port in
    use and may be called portEND_SWITCHING_ISR(). */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#endif // WAIT_SEMA
#ifdef SYSTEM_VIEW
    traceISR_EXIT();
#endif
#endif // BAREMETAL

    last_cl_id = (int)nema_reg_read(NEMA_CLID);
#if defined(NEMA_GFX_POWERSAVE) && defined(NEMA_GFX_POWEROFF_END_CL)
    nema_gfx_check_busy_and_suspend();
#endif
}

void am_gpu_isr()
{
    // Invalidate DAXI to make sure CPU sees the new data when loaded
    am_hal_daxi_control(AM_HAL_DAXI_CONTROL_INVALIDATE, 0);

    // Nema GPU interrupt handler
    prvNemaInterruptHandler(NULL);
}

//*****************************************************************************
//
// User portion of NemaGFX SDK initialization 
//
//*****************************************************************************
int32_t nema_sys_init (void)
{
    static uint32_t mempool_created = false;
    // disable clockgating of GPU
    nema_reg_write(NEMA_CGCTRL, 0xffffffff);

#if (defined(NEMA_MULTI_PROCESS) || defined(NEMA_MULTI_THREAD))
    enable_mutices = 0;
#endif
    nema_reg_write(NEMA_INTERRUPT, 0);

    /* Install Interrupt Handler */
    NVIC_SetPriority(NEMA_IRQ, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(NEMA_IRQ);
#if (defined(BAREMETAL))
    static uint32_t gpu_power_periodicall_task_started = false;
    if (!gpu_power_periodicall_task_started)
    {
        gpu_power_periodicall_task_init();
        gpu_power_periodicall_task_started = true;
    }
#endif
    if(!mempool_created)
    {
#if WAIT_IRQ_BINARY_SEMAPHORE
        if (xSemaphore == NULL)
        {
            xSemaphore = xSemaphoreCreateBinary();
        }
#endif


#if (defined(NEMA_MULTI_PROCESS) || defined(NEMA_MULTI_THREAD))
        for (int i = 0; i <= MUTEX_MAX; ++i) {
            xMutex[i] = xSemaphoreCreateMutex();
        }
#endif

#ifndef NEMA_USE_CUSTOM_MALLOC
        // Map and initialize Graphics Memory
        tsi_malloc_init((void *)VMEM_BASEADDR, (uintptr_t)VMEM_BASEADDR, VMEM_SIZE, 1);
#endif

        mempool_created = true;
    }
    //ring_buffer_str.bo may be already allocated
    if ( ring_buffer_str.bo.base_phys == 0U )
    {
        //allocate ring_buffer memory
        ring_buffer_str.bo = nema_buffer_create((MAX_PENDING_CL*12 + 16)*4);
        (void)nema_buffer_map(&ring_buffer_str.bo);
    }

    //Initialize Ring BUffer
    int ret = nema_rb_init(&ring_buffer_str, 1);

#if (defined(NEMA_MULTI_PROCESS) || defined(NEMA_MULTI_THREAD))
    enable_mutices = 1;
#endif

    if (ret) {
        // am_util_stdio_printf("nema_rb_init FAILED\n");
        return ret;
    }

    last_cl_id = -1;

    return 0;
}

//*****************************************************************************
//
// User implementation for waiting GPU interrupt
//
//*****************************************************************************
int nema_wait_irq (void)
{
#ifdef BAREMETAL
    uint32_t ui32usMaxDelay = 100000;
    uint32_t ui32Status;
    /* Wait for the interrupt */
#if WAIT_IRQ_POLL == 1
    //irq_handler sets NEMADC_REG_INTERRUPT to 0. Poll until this happens
    ui32Status = am_hal_delay_us_status_change(ui32usMaxDelay, (uint32_t)&GPU->INTERRUPTCTRL, 0xFFFFFFFF, 0);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }
#else // WAIT_IRQ_POLL
    while (nema_reg_read(NEMA_INTERRUPT) != 0U )
    {
        AM_CRITICAL_BEGIN
        if (nema_reg_read(NEMA_INTERRUPT) != 0U)
        {
            am_hal_sysctrl_sleep(1);
        }
        AM_CRITICAL_END
    }
#endif // WAIT_IRQ_POLL
#else /* BAREMETAL */
    /* Wait for the interrupt */
#if WAIT_IRQ_BINARY_SEMAPHORE == 1
    TickType_t block_ms = pdMS_TO_TICKS(1000);
    BaseType_t xResult;
    xResult = xSemaphoreTake( xSemaphore, block_ms );
    return (int)xResult;
#else
    BaseType_t xResult;

    xHandlingTask = xTaskGetCurrentTaskHandle();

    /* If a task is in the Blocked state to wait for a notification when the
       notification arrives then the task immediately exits the Blocked state
       and the notification does not remain pending. If a task was not waiting
       for a notification when a notification arrives then the notification
       will remain pending until the receiving task reads its notification
       value. */

    TickType_t block_ms = pdMS_TO_TICKS(1000);

    /* Wait to be notified of an interrupt. */
    xResult = xTaskNotifyWait( pdFALSE,    /* Don't clear bits on entry. */
                       0,                  /* Don't clear bits on exit. */
                       NULL,               /* No nitification value */
                       block_ms );

    return (int)xResult;
#endif
#endif /* BAREMETAL */

#ifdef BAREMETAL
    return 0;
#endif
}

//*****************************************************************************
//
// User implementation if wait GPU interrupt is expired
//
//*****************************************************************************
int nema_wait_irq_cl (int cl_id)
{
  int loops = 0;

  while ( last_cl_id < cl_id) 
  {
        loops++;
        int ret = nema_wait_irq();
        (void)ret;

#if 0 // remove GPU soft reset
        // try to soft reset GPU if wait too long
        if (loops > 3) {
          nema_reg_write(NEMA_CMDSTATUS, 0);
          nema_reg_write(NEMA_STATUS, 0);
          nema_reg_write(NEMA_CLID, cl_id);
          nema_reg_write(NEMA_INTERRUPT, 1);
        }
#endif        
    }

    return 0;
}

uint32_t nema_reg_read (uint32_t reg)
{
    volatile uint32_t *ptr = (volatile uint32_t *)(nema_regs + reg);
    return *ptr;
}

void nema_reg_write (uint32_t reg,uint32_t value)
{
    volatile uint32_t *ptr = (volatile uint32_t *)(nema_regs + reg);
    *ptr = value;
}


nema_buffer_t nema_buffer_create (int size)
{
    nema_mutex_lock(MUTEX_MALLOC);

    nema_buffer_t bo;
#ifdef NEMA_USE_CUSTOM_MALLOC
    bo.base_virt = NEMA_CUSTOM_MALLOC(size);

    // Check the address alignment, ThinkSi request all GPU buffer
    // should align to 8 bytes.
    if(((uint32_t)bo.base_virt) &0x00000007 )
    {
        //If not aligned, free and set to NULL.
        NEMA_CUSTOM_FREE(bo.base_virt);
        bo.base_virt = NULL;
    }
#else
    bo.base_virt = tsi_malloc(size);
#endif

    bo.base_phys = (uintptr_t) (bo.base_virt);
    bo.size      = size;
    bo.fd        = 0;

    nema_mutex_unlock(MUTEX_MALLOC);
    return bo;
}

nema_buffer_t nema_buffer_create_pool (int pool, int size)
{
    return nema_buffer_create(size);
}

void *nema_buffer_map (nema_buffer_t * bo)
{
    return bo->base_virt;
}

void nema_buffer_unmap (nema_buffer_t * bo)
{

}

void nema_buffer_destroy (nema_buffer_t * bo)
{
    nema_mutex_lock(MUTEX_MALLOC);

#ifdef NEMA_USE_CUSTOM_MALLOC
    NEMA_CUSTOM_FREE(bo->base_virt);
#else
    tsi_free(bo->base_virt);
#endif

    bo->base_virt = (void *)NULL;
    bo->base_phys = 0;
    bo->size      = 0;
    bo->fd        = -1;

    nema_mutex_unlock(MUTEX_MALLOC);
}

uintptr_t nema_buffer_phys (nema_buffer_t * bo)
{
    return bo->base_phys;
}

void nema_buffer_flush(nema_buffer_t * bo)
{
    nema_mutex_lock(MUTEX_FLUSH);

#ifdef XPAR_CPU_ID
    // Only for Zynq platforms!!!
    // Use XPAR_CPU_ID to decide if Xil_DCacheFlushRange is available
    // For Zynq platforms, XPAR_CPU_ID is defined in Xilinx BSP
    // ps7_cortexa9_0/include/xparameters.h
    #include "xil_cache.h"
    Xil_DCacheFlushRange(bo->base_virt, bo->size);
#endif

    am_hal_sysctrl_bus_write_flush();

    nema_mutex_unlock(MUTEX_FLUSH);
}

void * nema_host_malloc (size_t size)
{
    nema_mutex_lock(MUTEX_MALLOC);

#ifdef NEMA_USE_CUSTOM_MALLOC
    void *ptr = NEMA_CUSTOM_MALLOC(size);
#else
    void *ptr = tsi_malloc(size);
#endif


    nema_mutex_unlock(MUTEX_MALLOC);
    return ptr;
}

void nema_host_free (void * ptr)
{
    nema_mutex_lock(MUTEX_MALLOC);

#ifdef NEMA_USE_CUSTOM_MALLOC
    NEMA_CUSTOM_FREE(ptr);
#else
    tsi_free(ptr);
#endif

    nema_mutex_unlock(MUTEX_MALLOC);
}

int nema_mutex_lock (int mutex_id)
{
#if (defined(NEMA_MULTI_PROCESS) || defined(NEMA_MULTI_THREAD))
    if ((enable_mutices == 1) && (mutex_id >= 0) && (mutex_id <= MUTEX_MAX)) {
        xSemaphoreTake( xMutex[mutex_id], portMAX_DELAY );
    }
#endif
    return 0;
}

int nema_mutex_unlock (int mutex_id)
{
#if (defined(NEMA_MULTI_PROCESS) || defined(NEMA_MULTI_THREAD))
    if ((enable_mutices == 1) && (mutex_id >= 0) && (mutex_id <= MUTEX_MAX)) {
        xSemaphoreGive( xMutex[mutex_id] );
    }
#endif
    return 0;
}



void nema_build_cl_start(void)
{
    AM_CRITICAL_BEGIN
    building_cl = true;
    AM_CRITICAL_END
}

void nema_build_cl_end(void)
{
    AM_CRITICAL_BEGIN
    building_cl = false;
    AM_CRITICAL_END
}
