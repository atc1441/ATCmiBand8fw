/*
 * FreeRTOS Kernel V10.5.1
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM CM4F port.
 *----------------------------------------------------------*/
/* hardware includes */
#include "am_mcu_apollo.h"
#include "am_bsp.h"
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

// A Possible clock glitch could rarely cause the Stimer interrupt to be lost.
// Set up a backup comparator to handle this case
#define AM_FREERTOS_STIMER_BACKUP

//#define FREERTOS_STIMER_DIAGS
#ifdef AM_FREERTOS_STIMER_DIAGS
uint32_t gF_stimerHistory[256][4];
uint8_t gF_stimerHistoryCount = 0;
uint32_t gF_stimerGetHistory[256][4];
uint8_t gF_stimerGetHistoryCount = 0;
#endif

// Check to make sure the FreeRTOSConfig.h options are consistent per the implementation
#if configOVERRIDE_DEFAULT_TICK_CONFIGURATION == 1

#if configUSE_TICKLESS_IDLE != 2
#error "configOVERRIDE_DEFAULT_TICK_CONFIGURATION == 1 supported only for configUSE_TICKLESS_IDLE = 2"
#endif

#ifndef AM_FREERTOS_USE_STIMER_FOR_TICK

// Determine which Timer to use if configured to use Timer for FreeRTOS Tick
#ifndef configTIMER_NUM
// Default
#define configTIMER_NUM 	0
#endif


#ifdef APOLLO4_FPGA
#ifndef configTIMER_CLOCK_HZ
#define configTIMER_CLOCK_HZ   1500000
#define configTIMER_CLOCK      AM_HAL_TIMER_CLOCK_HFRC_DIV16
#else
#error configTIMER_CLOCK_HZ not correctly specified.
#endif

#else // APOLLO4_FPGA

#ifndef configTIMER_CLOCK_HZ
// Default
#define configTIMER_CLOCK_HZ   32768
#define configTIMER_CLOCK      AM_HAL_TIMER_XT_32_768KHZ
#else
#ifndef configTIMER_CLOCK
#if configTIMER_CLOCK_HZ == 32768
// Default - for backward compatibility
#define configTIMER_CLOCK      AM_HAL_TIMER_XT_32_768KHZ
#else
#error "configTIMER_CLOCK not specified"
#endif
#endif
#endif

#endif



#else

#ifdef AM_PART_APOLLO
#error "Apollo can not use STimer for FreeRTOS"
#endif

#ifndef configSTIMER_CLOCK_HZ
// Default
#define configSTIMER_CLOCK_HZ   32768
#define configSTIMER_CLOCK      AM_HAL_STIMER_XTAL_32KHZ
#else
#ifndef configSTIMER_CLOCK
#if configSTIMER_CLOCK_HZ == 32768
// Default - for backward compatibility
#define configSTIMER_CLOCK      AM_HAL_STIMER_XTAL_32KHZ
#else
#error "configSTIMER_CLOCK not specified"
#endif
#endif
#endif

// Keeps the snapshot of the STimer corresponding to last tick update
static uint32_t g_lastSTimerVal = 0;
#endif

#define portMAX_16_BIT_NUMBER		( 0x0000ffffUL )
#define portMAX_32_BIT_NUMBER		( 0xffffffffUL )


#endif
#if configOVERRIDE_DEFAULT_TICK_CONFIGURATION == 0
#if configUSE_TICKLESS_IDLE == 2
#error "configOVERRIDE_DEFAULT_TICK_CONFIGURATION == 0 not supported for configUSE_TICKLESS_IDLE = 2"
#endif
#endif

#ifndef __VFP_FP__
	#error This port can only be used when the project options are configured to enable hardware floating point support.
#endif

#if( configMAX_SYSCALL_INTERRUPT_PRIORITY == 0 )
	#error configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to 0.  See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html
#endif

#ifndef configSYSTICK_CLOCK_HZ
	#define configSYSTICK_CLOCK_HZ configCPU_CLOCK_HZ
	/* Ensure the SysTick is clocked at the same frequency as the core. */
	#define portNVIC_SYSTICK_CLK_BIT	( 1UL << 2UL )
#else
	/* The way the SysTick is clocked is not modified in case it is not the same
	as the core. */
	#define portNVIC_SYSTICK_CLK_BIT	( 0 )
#endif

/* Constants required to manipulate the core.  Registers first... */
#define portNVIC_SYSTICK_CTRL_REG			( * ( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_LOAD_REG			( * ( ( volatile uint32_t * ) 0xe000e014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG	( * ( ( volatile uint32_t * ) 0xe000e018 ) )
#define portNVIC_SYSPRI2_REG				( * ( ( volatile uint32_t * ) 0xe000ed20 ) )
/* ...then bits in the registers. */
#define portNVIC_SYSTICK_INT_BIT			( 1UL << 1UL )
#define portNVIC_SYSTICK_ENABLE_BIT			( 1UL << 0UL )
#define portNVIC_SYSTICK_COUNT_FLAG_BIT		( 1UL << 16UL )
#define portNVIC_PENDSVCLEAR_BIT 			( 1UL << 27UL )
#define portNVIC_PEND_SYSTICK_CLEAR_BIT		( 1UL << 25UL )

/* Constants used to detect a Cortex-M7 r0p1 core, which should use the ARM_CM7
r0p1 port. */
#define portCPUID							( * ( ( volatile uint32_t * ) 0xE000ed00 ) )
#define portCORTEX_M7_r0p1_ID				( 0x410FC271UL )
#define portCORTEX_M7_r0p0_ID				( 0x410FC270UL )

#define portNVIC_PENDSV_PRI					( ( ( uint32_t ) configKERNEL_INTERRUPT_PRIORITY ) << 16UL )
#define portNVIC_SYSTICK_PRI				( ( ( uint32_t ) configKERNEL_INTERRUPT_PRIORITY ) << 24UL )

/* Constants required to check the validity of an interrupt priority. */
#define portFIRST_USER_INTERRUPT_NUMBER		( 16 )
#define portNVIC_IP_REGISTERS_OFFSET_16 	( 0xE000E3F0 )
#define portAIRCR_REG						( * ( ( volatile uint32_t * ) 0xE000ED0C ) )
#define portMAX_8_BIT_VALUE					( ( uint8_t ) 0xff )
#define portTOP_BIT_OF_BYTE					( ( uint8_t ) 0x80 )
#define portMAX_PRIGROUP_BITS				( ( uint8_t ) 7 )
#define portPRIORITY_GROUP_MASK				( 0x07UL << 8UL )
#define portPRIGROUP_SHIFT					( 8UL )

/* Masks off all bits but the VECTACTIVE bits in the ICSR register. */
#define portVECTACTIVE_MASK					( 0xFFUL )

/* Constants required to manipulate the VFP. */
#define portFPCCR							( ( volatile uint32_t * ) 0xe000ef34 ) /* Floating point context control register. */
#define portASPEN_AND_LSPEN_BITS			( 0x3UL << 30UL )

/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR					( 0x01000000 )
#define portINITIAL_EXC_RETURN				( 0xfffffffd )

/* The systick is a 24-bit counter. */
#define portMAX_24_BIT_NUMBER				( 0xffffffUL )

/* For strict compliance with the Cortex-M spec the task start address should
have bit-0 clear, as it is loaded into the PC on exit from an ISR. */
#define portSTART_ADDRESS_MASK		( ( StackType_t ) 0xfffffffeUL )

/* A fiddle factor to estimate the number of SysTick counts that would have
occurred while the SysTick counter is stopped during tickless idle
calculations. */
#define portMISSED_COUNTS_FACTOR			( 45UL )

/* Let the user override the pre-loading of the initial LR with the address of
prvTaskExitError() in case it messes up unwinding of the stack in the
debugger. */
#ifdef configTASK_RETURN_ADDRESS
	#define portTASK_RETURN_ADDRESS	configTASK_RETURN_ADDRESS
#else
	#define portTASK_RETURN_ADDRESS	prvTaskExitError
#endif

/*
 * Setup the timer to generate the tick interrupts.  The implementation in this
 * file is weak to allow application writers to change the timer used to
 * generate the tick interrupt.
 */
void vPortSetupTimerInterrupt( void );

/*
 * Exception handlers.
 */
void xPortPendSVHandler( void ) __attribute__ (( naked ));
void xPortSysTickHandler( void );
void vPortSVCHandler( void ) __attribute__ (( naked ));

/*
 * Start first task is a separate function so it can be tested in isolation.
 */
static void prvPortStartFirstTask( void ) __attribute__ (( naked ));

/*
 * Function to enable the VFP.
 */
static void vPortEnableVFP( void ) __attribute__ (( naked ));

/*
 * Used to catch tasks that attempt to return from their implementing function.
 */
static void prvTaskExitError( void );

/*-----------------------------------------------------------*/

/* Each task maintains its own interrupt status in the critical nesting
variable. */
static UBaseType_t uxCriticalNesting = 0xaaaaaaaa;

/*
 * The number of SysTick increments that make up one tick period.
 */
#if( configUSE_TICKLESS_IDLE == 1 )
	static uint32_t ulTimerCountsForOneTick = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * The maximum number of tick periods that can be suppressed is limited by the
 * 24 bit resolution of the SysTick timer.
 */
#if( configUSE_TICKLESS_IDLE == 1 )
	static uint32_t xMaximumPossibleSuppressedTicks = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * Compensate for the CPU cycles that pass while the SysTick is stopped (low
 * power functionality only.
 */
#if( configUSE_TICKLESS_IDLE == 1 )
	static uint32_t ulStoppedTimerCompensation = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * Used by the portASSERT_IF_INTERRUPT_PRIORITY_INVALID() macro to ensure
 * FreeRTOS API functions are not called from interrupts that have been assigned
 * a priority above configMAX_SYSCALL_INTERRUPT_PRIORITY.
 */
#if( configASSERT_DEFINED == 1 )
	 static uint8_t ucMaxSysCallPriority = 0;
	 static uint32_t ulMaxPRIGROUPValue = 0;
	 static const volatile uint8_t * const pcInterruptPriorityRegisters = ( const volatile uint8_t * const ) portNVIC_IP_REGISTERS_OFFSET_16;
#endif /* configASSERT_DEFINED */

/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
	/* Simulate the stack frame as it would be created by a context switch
	interrupt. */

	/* Offset added to account for the way the MCU uses the stack on entry/exit
	of interrupts, and to ensure alignment. */
	pxTopOfStack--;

	*pxTopOfStack = portINITIAL_XPSR;	/* xPSR */
	pxTopOfStack--;
	*pxTopOfStack = ( ( StackType_t ) pxCode ) & portSTART_ADDRESS_MASK;	/* PC */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) portTASK_RETURN_ADDRESS;	/* LR */

	/* Save code space by skipping register initialisation. */
	pxTopOfStack -= 5;	/* R12, R3, R2 and R1. */
	*pxTopOfStack = ( StackType_t ) pvParameters;	/* R0 */

	/* A save method is being used that requires each task to maintain its
	own exec return value. */
	pxTopOfStack--;
	*pxTopOfStack = portINITIAL_EXC_RETURN;

	pxTopOfStack -= 8;	/* R11, R10, R9, R8, R7, R6, R5 and R4. */

	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

static void prvTaskExitError( void )
{
volatile uint32_t ulDummy = 0;

	/* A function that implements a task must not exit or attempt to return to
	its caller as there is nothing to return to.  If a task wants to exit it
	should instead call vTaskDelete( NULL ).

	Artificially force an assert() to be triggered if configASSERT() is
	defined, then stop here so application writers can catch the error. */
	configASSERT( uxCriticalNesting == ~0UL );
	portDISABLE_INTERRUPTS();
	while( ulDummy == 0 )
	{
		/* This file calls prvTaskExitError() after the scheduler has been
		started to remove a compiler warning about the function being defined
		but never called.  ulDummy is used purely to quieten other warnings
		about code appearing after this function is called - making ulDummy
		volatile makes the compiler think the function could return and
		therefore not output an 'unreachable code' warning for code that appears
		after it. */
	}
}
/*-----------------------------------------------------------*/

void vPortSVCHandler( void )
{
	__asm volatile (
					"	ldr	r3, pxCurrentTCBConst2		\n" /* Restore the context. */
					"	ldr r1, [r3]					\n" /* Use pxCurrentTCBConst to get the pxCurrentTCB address. */
					"	ldr r0, [r1]					\n" /* The first item in pxCurrentTCB is the task top of stack. */
					"	ldmia r0!, {r4-r11, r14}		\n" /* Pop the registers that are not automatically saved on exception entry and the critical nesting count. */
					"	msr psp, r0						\n" /* Restore the task stack pointer. */
					"	isb								\n"
					"	mov r0, #0 						\n"
					"	msr	basepri, r0					\n"
					"	bx r14							\n"
					"									\n"
					"	.align 4						\n"
					"pxCurrentTCBConst2: .word pxCurrentTCB				\n"
				);
}
/*-----------------------------------------------------------*/

static void prvPortStartFirstTask( void )
{
	/* Start the first task.  This also clears the bit that indicates the FPU is
	in use in case the FPU was used before the scheduler was started - which
	would otherwise result in the unnecessary leaving of space in the SVC stack
	for lazy saving of FPU registers. */
	__asm volatile(
					" ldr r0, =0xE000ED08 	\n" /* Use the NVIC offset register to locate the stack. */
					" ldr r0, [r0] 			\n"
					" ldr r0, [r0] 			\n"
					" msr msp, r0			\n" /* Set the msp back to the start of the stack. */
					" mov r0, #0			\n" /* Clear the bit that indicates the FPU is in use, see comment above. */
					" msr control, r0		\n"
					" cpsie i				\n" /* Globally enable interrupts. */
					" cpsie f				\n"
					" dsb					\n"
					" isb					\n"
					" svc 0					\n" /* System call to start first task. */
					" nop					\n"
				);
}
/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
BaseType_t xPortStartScheduler( void )
{
	/* configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to 0.
	See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html */
	configASSERT( configMAX_SYSCALL_INTERRUPT_PRIORITY );

	/* This port can be used on all revisions of the Cortex-M7 core other than
	the r0p1 parts.  r0p1 parts should use the port from the
	/source/portable/GCC/ARM_CM7/r0p1 directory. */
	configASSERT( portCPUID != portCORTEX_M7_r0p1_ID );
	configASSERT( portCPUID != portCORTEX_M7_r0p0_ID );

	#if( configASSERT_DEFINED == 1 )
	{
		volatile uint32_t ulOriginalPriority;
		volatile uint8_t * const pucFirstUserPriorityRegister = ( volatile uint8_t * const ) ( portNVIC_IP_REGISTERS_OFFSET_16 + portFIRST_USER_INTERRUPT_NUMBER );
		volatile uint8_t ucMaxPriorityValue;

		/* Determine the maximum priority from which ISR safe FreeRTOS API
		functions can be called.  ISR safe functions are those that end in
		"FromISR".  FreeRTOS maintains separate thread and ISR API functions to
		ensure interrupt entry is as fast and simple as possible.

		Save the interrupt priority value that is about to be clobbered. */
		ulOriginalPriority = *pucFirstUserPriorityRegister;

		/* Determine the number of priority bits available.  First write to all
		possible bits. */
		*pucFirstUserPriorityRegister = portMAX_8_BIT_VALUE;

		/* Read the value back to see how many bits stuck. */
		ucMaxPriorityValue = *pucFirstUserPriorityRegister;

		/* Use the same mask on the maximum system call priority. */
		ucMaxSysCallPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY & ucMaxPriorityValue;

		/* Calculate the maximum acceptable priority group value for the number
		of bits read back. */
		ulMaxPRIGROUPValue = portMAX_PRIGROUP_BITS;
		while( ( ucMaxPriorityValue & portTOP_BIT_OF_BYTE ) == portTOP_BIT_OF_BYTE )
		{
			ulMaxPRIGROUPValue--;
			ucMaxPriorityValue <<= ( uint8_t ) 0x01;
		}

		#ifdef __NVIC_PRIO_BITS
		{
			/* Check the CMSIS configuration that defines the number of
			priority bits matches the number of priority bits actually queried
			from the hardware. */
			configASSERT( ( portMAX_PRIGROUP_BITS - ulMaxPRIGROUPValue ) == __NVIC_PRIO_BITS );
		}
		#endif

		#ifdef configPRIO_BITS
		{
			/* Check the FreeRTOS configuration that defines the number of
			priority bits matches the number of priority bits actually queried
			from the hardware. */
			configASSERT( ( portMAX_PRIGROUP_BITS - ulMaxPRIGROUPValue ) == configPRIO_BITS );
		}
		#endif

		/* Shift the priority group value back to its position within the AIRCR
		register. */
		ulMaxPRIGROUPValue <<= portPRIGROUP_SHIFT;
		ulMaxPRIGROUPValue &= portPRIORITY_GROUP_MASK;

		/* Restore the clobbered interrupt priority register to its original
		value. */
		*pucFirstUserPriorityRegister = ulOriginalPriority;
	}
	#endif /* conifgASSERT_DEFINED */

	/* Make PendSV and SysTick the lowest priority interrupts. */
	portNVIC_SYSPRI2_REG |= portNVIC_PENDSV_PRI;
	portNVIC_SYSPRI2_REG |= portNVIC_SYSTICK_PRI;

	/* Start the timer that generates the tick ISR.  Interrupts are disabled
	here already. */
	vPortSetupTimerInterrupt();

	/* Initialise the critical nesting count ready for the first task. */
	uxCriticalNesting = 0;

	/* Ensure the VFP is enabled - it should be anyway. */
	vPortEnableVFP();

	/* Lazy save always. */
	*( portFPCCR ) |= portASPEN_AND_LSPEN_BITS;

	/* Start the first task. */
	prvPortStartFirstTask();

	/* Should never get here as the tasks will now be executing!  Call the task
	exit error function to prevent compiler warnings about a static function
	not being called in the case that the application writer overrides this
	functionality by defining configTASK_RETURN_ADDRESS.  Call
	vTaskSwitchContext() so link time optimisation does not remove the
	symbol. */
	vTaskSwitchContext();
	prvTaskExitError();

	/* Should not get here! */
	return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* Not implemented in ports where there is nothing to return to.
	Artificially force an assert. */
	configASSERT( uxCriticalNesting == 1000UL );
}
/*-----------------------------------------------------------*/

void vPortEnterCritical( void )
{
	portDISABLE_INTERRUPTS();
	uxCriticalNesting++;

	/* This is not the interrupt safe version of the enter critical function so
	assert() if it is being called from an interrupt context.  Only API
	functions that end in "FromISR" can be used in an interrupt.  Only assert if
	the critical nesting count is 1 to protect against recursive calls if the
	assert function also uses a critical section. */
	if( uxCriticalNesting == 1 )
	{
		configASSERT( ( portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK ) == 0 );
	}
}
/*-----------------------------------------------------------*/

void vPortExitCritical( void )
{
	configASSERT( uxCriticalNesting );
	uxCriticalNesting--;
	if( uxCriticalNesting == 0 )
	{
		portENABLE_INTERRUPTS();
	}
}
/*-----------------------------------------------------------*/

void xPortPendSVHandler( void )
{
	/* This is a naked function. */

	__asm volatile
	(
	"	mrs r0, psp							\n"
	"	isb									\n"
	"										\n"
	"	ldr	r3, pxCurrentTCBConst			\n" /* Get the location of the current TCB. */
	"	ldr	r2, [r3]						\n"
	"										\n"
	"	tst r14, #0x10						\n" /* Is the task using the FPU context?  If so, push high vfp registers. */
	"	it eq								\n"
	"	vstmdbeq r0!, {s16-s31}				\n"
	"										\n"
	"	stmdb r0!, {r4-r11, r14}			\n" /* Save the core registers. */
	"	str r0, [r2]						\n" /* Save the new top of stack into the first member of the TCB. */
	"										\n"
	"	stmdb sp!, {r0, r3}					\n"
	"	mov r0, %0 							\n"
	"	msr basepri, r0						\n"
	"	dsb									\n"
	"	isb									\n"
	"	bl vTaskSwitchContext				\n"
	"	mov r0, #0							\n"
	"	msr basepri, r0						\n"
	"	ldmia sp!, {r0, r3}					\n"
	"										\n"
	"	ldr r1, [r3]						\n" /* The first item in pxCurrentTCB is the task top of stack. */
	"	ldr r0, [r1]						\n"
	"										\n"
	"	ldmia r0!, {r4-r11, r14}			\n" /* Pop the core registers. */
	"										\n"
	"	tst r14, #0x10						\n" /* Is the task using the FPU context?  If so, pop the high vfp registers too. */
	"	it eq								\n"
	"	vldmiaeq r0!, {s16-s31}				\n"
	"										\n"
	"	msr psp, r0							\n"
	"	isb									\n"
	"										\n"
	#ifdef WORKAROUND_PMU_CM001 /* XMC4000 specific errata workaround. */
		#if WORKAROUND_PMU_CM001 == 1
	"			push { r14 }				\n"
	"			pop { pc }					\n"
		#endif
	#endif
	"										\n"
	"	bx r14								\n"
	"										\n"
	"	.align 4							\n"
	"pxCurrentTCBConst: .word pxCurrentTCB	\n"
	::"i"(configMAX_SYSCALL_INTERRUPT_PRIORITY)
	);
}
/*-----------------------------------------------------------*/

void xPortSysTickHandler( void )
{
	/* The SysTick runs at the lowest interrupt priority, so when this interrupt
	executes all interrupts must be unmasked.  There is therefore no need to
	save and then restore the interrupt mask value as its value is already
	known. */
	portDISABLE_INTERRUPTS();
	{
		/* Increment the RTOS tick. */
		if( xTaskIncrementTick() != pdFALSE )
		{
			/* A context switch is required.  Context switching is performed in
			the PendSV interrupt.  Pend the PendSV interrupt. */
			portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
		}
	}
	portENABLE_INTERRUPTS();
}
/*-----------------------------------------------------------*/

#if( configUSE_TICKLESS_IDLE == 1 )

	__attribute__((weak)) void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
	{
	uint32_t ulReloadValue, ulCompleteTickPeriods, ulCompletedSysTickDecrements;
	TickType_t xModifiableIdleTime;

		/* Make sure the SysTick reload value does not overflow the counter. */
		if( xExpectedIdleTime > xMaximumPossibleSuppressedTicks )
		{
			xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
		}

		/* Stop the SysTick momentarily.  The time the SysTick is stopped for
		is accounted for as best it can be, but using the tickless mode will
		inevitably result in some tiny drift of the time maintained by the
		kernel with respect to calendar time. */
		portNVIC_SYSTICK_CTRL_REG &= ~portNVIC_SYSTICK_ENABLE_BIT;

		/* Calculate the reload value required to wait xExpectedIdleTime
		tick periods.  -1 is used because this code will execute part way
		through one of the tick periods. */
		ulReloadValue = portNVIC_SYSTICK_CURRENT_VALUE_REG + ( ulTimerCountsForOneTick * ( xExpectedIdleTime - 1UL ) );
		if( ulReloadValue > ulStoppedTimerCompensation )
		{
			ulReloadValue -= ulStoppedTimerCompensation;
		}

		/* Enter a critical section but don't use the taskENTER_CRITICAL()
		method as that will mask interrupts that should exit sleep mode. */
		__asm volatile( "cpsid i" ::: "memory" );
		__asm volatile( "dsb" );
		__asm volatile( "isb" );

		/* If a context switch is pending or a task is waiting for the scheduler
		to be unsuspended then abandon the low power entry. */
		if( eTaskConfirmSleepModeStatus() == eAbortSleep )
		{
			/* Restart from whatever is left in the count register to complete
			this tick period. */
			portNVIC_SYSTICK_LOAD_REG = portNVIC_SYSTICK_CURRENT_VALUE_REG;

			/* Restart SysTick. */
			portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

			/* Reset the reload register to the value required for normal tick
			periods. */
			portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;

			/* Re-enable interrupts - see comments above the cpsid instruction()
			above. */
			__asm volatile( "cpsie i" ::: "memory" );
		}
		else
		{
			/* Set the new reload value. */
			portNVIC_SYSTICK_LOAD_REG = ulReloadValue;

			/* Clear the SysTick count flag and set the count value back to
			zero. */
			portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

			/* Restart SysTick. */
			portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

			/* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
			set its parameter to 0 to indicate that its implementation contains
			its own wait for interrupt or wait for event instruction, and so wfi
			should not be executed again.  However, the original expected idle
			time variable must remain unmodified, so a copy is taken. */
			xModifiableIdleTime = xExpectedIdleTime;
			configPRE_SLEEP_PROCESSING( xModifiableIdleTime );
			if( xModifiableIdleTime > 0 )
			{
				__asm volatile( "dsb" ::: "memory" );
				__asm volatile( "wfi" );
				__asm volatile( "isb" );
			}
			configPOST_SLEEP_PROCESSING( xExpectedIdleTime );

			/* Re-enable interrupts to allow the interrupt that brought the MCU
			out of sleep mode to execute immediately.  see comments above
			__disable_interrupt() call above. */
			__asm volatile( "cpsie i" ::: "memory" );
			__asm volatile( "dsb" );
			__asm volatile( "isb" );

			/* Disable interrupts again because the clock is about to be stopped
			and interrupts that execute while the clock is stopped will increase
			any slippage between the time maintained by the RTOS and calendar
			time. */
			__asm volatile( "cpsid i" ::: "memory" );
			__asm volatile( "dsb" );
			__asm volatile( "isb" );

			/* Disable the SysTick clock without reading the
			portNVIC_SYSTICK_CTRL_REG register to ensure the
			portNVIC_SYSTICK_COUNT_FLAG_BIT is not cleared if it is set.  Again,
			the time the SysTick is stopped for is accounted for as best it can
			be, but using the tickless mode will inevitably result in some tiny
			drift of the time maintained by the kernel with respect to calendar
			time*/
			portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT );

			/* Determine if the SysTick clock has already counted to zero and
			been set back to the current reload value (the reload back being
			correct for the entire expected idle time) or if the SysTick is yet
			to count to zero (in which case an interrupt other than the SysTick
			must have brought the system out of sleep mode). */
			if( ( portNVIC_SYSTICK_CTRL_REG & portNVIC_SYSTICK_COUNT_FLAG_BIT ) != 0 )
			{
				uint32_t ulCalculatedLoadValue;

				/* The tick interrupt is already pending, and the SysTick count
				reloaded with ulReloadValue.  Reset the
				portNVIC_SYSTICK_LOAD_REG with whatever remains of this tick
				period. */
				ulCalculatedLoadValue = ( ulTimerCountsForOneTick - 1UL ) - ( ulReloadValue - portNVIC_SYSTICK_CURRENT_VALUE_REG );

				/* Don't allow a tiny value, or values that have somehow
				underflowed because the post sleep hook did something
				that took too long. */
				if( ( ulCalculatedLoadValue < ulStoppedTimerCompensation ) || ( ulCalculatedLoadValue > ulTimerCountsForOneTick ) )
				{
					ulCalculatedLoadValue = ( ulTimerCountsForOneTick - 1UL );
				}

				portNVIC_SYSTICK_LOAD_REG = ulCalculatedLoadValue;

				/* As the pending tick will be processed as soon as this
				function exits, the tick value maintained by the tick is stepped
				forward by one less than the time spent waiting. */
				ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
			}
			else
			{
				/* Something other than the tick interrupt ended the sleep.
				Work out how long the sleep lasted rounded to complete tick
				periods (not the ulReload value which accounted for part
				ticks). */
				ulCompletedSysTickDecrements = ( xExpectedIdleTime * ulTimerCountsForOneTick ) - portNVIC_SYSTICK_CURRENT_VALUE_REG;

				/* How many complete tick periods passed while the processor
				was waiting? */
				ulCompleteTickPeriods = ulCompletedSysTickDecrements / ulTimerCountsForOneTick;

				/* The reload value is set to whatever fraction of a single tick
				period remains. */
				portNVIC_SYSTICK_LOAD_REG = ( ( ulCompleteTickPeriods + 1UL ) * ulTimerCountsForOneTick ) - ulCompletedSysTickDecrements;
			}

			/* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG
			again, then set portNVIC_SYSTICK_LOAD_REG back to its standard
			value. */
			portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
			portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
			vTaskStepTick( ulCompleteTickPeriods );
			portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;

			/* Exit with interrpts enabled. */
			__asm volatile( "cpsie i" ::: "memory" );
		}
	}

#endif /* #if configUSE_TICKLESS_IDLE */
/*-----------------------------------------------------------*/

/*
 * Setup the systick timer to generate the tick interrupts at the required
 * frequency.
 */
#if configOVERRIDE_DEFAULT_TICK_CONFIGURATION == 0
__attribute__(( weak )) void vPortSetupTimerInterrupt( void )
{
	/* Calculate the constants required to configure the tick interrupt. */
	#if( configUSE_TICKLESS_IDLE == 1 )
	{
		ulTimerCountsForOneTick = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ );
		xMaximumPossibleSuppressedTicks = portMAX_24_BIT_NUMBER / ulTimerCountsForOneTick;
		ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / ( configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ );
	}
	#endif /* configUSE_TICKLESS_IDLE */

	/* Stop and clear the SysTick. */
	portNVIC_SYSTICK_CTRL_REG = 0UL;
	portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

	/* Configure SysTick to interrupt at the requested rate. */
	portNVIC_SYSTICK_LOAD_REG = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
	portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT );
}
#endif /* configOVERRIDE_DEFAULT_TICK_CONFIGURATION */
/*-----------------------------------------------------------*/

/* This is a naked function. */
static void vPortEnableVFP( void )
{
	__asm volatile
	(
		"	ldr.w r0, =0xE000ED88		\n" /* The FPU enable bits are in the CPACR. */
		"	ldr r1, [r0]				\n"
		"								\n"
		"	orr r1, r1, #( 0xf << 20 )	\n" /* Enable CP10 and CP11 coprocessors, then save back. */
		"	str r1, [r0]				\n"
		"	bx r14						"
	);
}
/*-----------------------------------------------------------*/

#if( configASSERT_DEFINED == 1 )

	void vPortValidateInterruptPriority( void )
	{
	uint32_t ulCurrentInterrupt;
	uint8_t ucCurrentPriority;

		/* Obtain the number of the currently executing interrupt. */
		__asm volatile( "mrs %0, ipsr" : "=r"( ulCurrentInterrupt ) :: "memory" );

		/* Is the interrupt number a user defined interrupt? */
		if( ulCurrentInterrupt >= portFIRST_USER_INTERRUPT_NUMBER )
		{
			/* Look up the interrupt's priority. */
			ucCurrentPriority = pcInterruptPriorityRegisters[ ulCurrentInterrupt ];

			/* The following assertion will fail if a service routine (ISR) for
			an interrupt that has been assigned a priority above
			configMAX_SYSCALL_INTERRUPT_PRIORITY calls an ISR safe FreeRTOS API
			function.  ISR safe FreeRTOS API functions must *only* be called
			from interrupts that have been assigned a priority at or below
			configMAX_SYSCALL_INTERRUPT_PRIORITY.

			Numerically low interrupt priority numbers represent logically high
			interrupt priorities, therefore the priority of the interrupt must
			be set to a value equal to or numerically *higher* than
			configMAX_SYSCALL_INTERRUPT_PRIORITY.

			Interrupts that	use the FreeRTOS API must not be left at their
			default priority of	zero as that is the highest possible priority,
			which is guaranteed to be above configMAX_SYSCALL_INTERRUPT_PRIORITY,
			and	therefore also guaranteed to be invalid.

			FreeRTOS maintains separate thread and ISR API functions to ensure
			interrupt entry is as fast and simple as possible.

			The following links provide detailed information:
			http://www.freertos.org/RTOS-Cortex-M3-M4.html
			http://www.freertos.org/FAQHelp.html */
			configASSERT( ucCurrentPriority >= ucMaxSysCallPriority );
		}

		/* Priority grouping:  The interrupt controller (NVIC) allows the bits
		that define each interrupt's priority to be split between bits that
		define the interrupt's pre-emption priority bits and bits that define
		the interrupt's sub-priority.  For simplicity all bits must be defined
		to be pre-emption priority bits.  The following assertion will fail if
		this is not the case (if some bits represent a sub-priority).

		If the application only uses CMSIS libraries for interrupt
		configuration then the correct setting can be achieved on all Cortex-M
		devices by calling NVIC_SetPriorityGrouping( 0 ); before starting the
		scheduler.  Note however that some vendor specific peripheral libraries
		assume a non-zero priority group setting, in which cases using a value
		of zero will result in unpredictable behaviour. */
		configASSERT( ( portAIRCR_REG & portPRIORITY_GROUP_MASK ) <= ulMaxPRIGROUPValue );
	}

#endif /* configASSERT_DEFINED */

#if configOVERRIDE_DEFAULT_TICK_CONFIGURATION != 0
/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the Ambiq Apollo_2 port.
 *----------------------------------------------------------*/
/* This port requires using the Stimer for Tickless_Idle in the Apollo_2 device  */      // dv**** 102616


#if configUSE_TICKLESS_IDLE == 2
	uint32_t ulTimerCountsForOneTick = 0;
/*
 * The maximum number of tick periods that can be suppressed is limited by the
 * resolution of the Tick timer.
 */
	static uint32_t xMaximumPossibleSuppressedTicks = 0;

void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
	uint32_t ulReloadValue;
    uint32_t New_Timer, Delta_Sleep;
	TickType_t xModifiableIdleTime;
    uint32_t elapsed_time;

	/* Make sure the SysTick reload value does not overflow the counter. */
	if( xExpectedIdleTime > xMaximumPossibleSuppressedTicks )
	{
		xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
	}


	/* Calculate the reload value required to wait xExpectedIdleTime
	tick periods.  -1 is used because this code will execute part way
	through one of the tick periods. */
	ulReloadValue =  ulTimerCountsForOneTick * ( xExpectedIdleTime - 1 );

	/* Enter a critical section but don't use the taskENTER_CRITICAL()
	method as that will mask interrupts that should exit sleep mode. */
	__asm volatile( "cpsid i" );
	__asm volatile( "dsb" );
	__asm volatile( "isb" );
#ifdef AM_FREERTOS_USE_STIMER_FOR_TICK
    // Adjust for the time already elapsed
    uint32_t curTime = am_hal_stimer_counter_get();
#ifdef AM_FREERTOS_STIMER_DIAGS
    gF_stimerGetHistory[gF_stimerGetHistoryCount][0] = gF_stimerGetHistoryCount;
    gF_stimerGetHistory[gF_stimerGetHistoryCount][1] = curTime;
    gF_stimerGetHistory[gF_stimerGetHistoryCount][2] = AM_REGVAL(AM_REG_STIMER_COMPARE(0, 0));
    gF_stimerGetHistory[gF_stimerGetHistoryCount][3] = gF_stimerHistoryCount;
    gF_stimerGetHistoryCount++;
#endif
    elapsed_time = curTime - g_lastSTimerVal;
#else
    am_hal_timer_stop(configTIMER_NUM);
    // Adjust for the time already elapsed
    elapsed_time = am_hal_timer_read(configTIMER_NUM);
#endif


	/* If a context switch is pending or a task is waiting for the scheduler
	to be unsuspended then abandon the low power entry. */
    /* Abandon low power entry if the sleep time is too short */
	if( (eTaskConfirmSleepModeStatus() == eAbortSleep) || ((elapsed_time + ulTimerCountsForOneTick) > ulReloadValue) )
	{
#ifndef AM_FREERTOS_USE_STIMER_FOR_TICK
        am_hal_timer_start(configTIMER_NUM);
#endif
		/* Re-enable interrupts - see comments above the cpsid instruction()
		above. */
		__asm volatile( "cpsie i" );
	}
	else
	{
        // Adjust for the time already elapsed
        ulReloadValue -= elapsed_time;
        // Initialize new timeout value
#ifdef AM_FREERTOS_USE_STIMER_FOR_TICK
        am_hal_stimer_compare_delta_set(0, ulReloadValue);
#ifdef AM_FREERTOS_STIMER_BACKUP
        am_hal_stimer_compare_delta_set(1, ulReloadValue+1);
#endif
#else
        am_hal_timer_clear(configTIMER_NUM);
        am_hal_timer_compare1_set(configTIMER_NUM, ulReloadValue);
        am_hal_timer_start(configTIMER_NUM);
#endif

		/* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
		set its parameter to 0 to indicate that its implementation contains
		its own wait for interrupt or wait for event instruction, and so wfi
		should not be executed again.  However, the original expected idle
		time variable must remain unmodified, so a copy is taken. */
		xModifiableIdleTime = xExpectedIdleTime;

		configPRE_SLEEP_PROCESSING( xModifiableIdleTime );       // Turn OFF all Periphials in this function

		if( xModifiableIdleTime > 0 )
		{
			__asm volatile( "dsb" );
			__asm volatile( "wfi" );
			__asm volatile( "isb" );
		}

		configPOST_SLEEP_PROCESSING( xExpectedIdleTime );       // Turn ON all Periphials in this function

        // Any interrupt may have woken us up

        // Before renable interrupts, check how many ticks the processor has been in SLEEP
        // Adjust xTickCount via vTaskStepTick( Delta_Sleep )
        // to keep xTickCount up to date, as if ticks have been running all along

#ifdef AM_FREERTOS_USE_STIMER_FOR_TICK
        New_Timer = am_hal_stimer_counter_get();
        Delta_Sleep = (signed long) New_Timer - (signed long) g_lastSTimerVal;
        g_lastSTimerVal = New_Timer - Delta_Sleep%ulTimerCountsForOneTick;
#else
        am_hal_timer_stop(configTIMER_NUM);
        New_Timer = am_hal_timer_read(configTIMER_NUM);

        // INTSTAT check is needed to handle a possible case where the we came here without timer
        // incrementing at all....the value will still say 0, but it does not mean it expired
        uint32_t ui32IntStatus = 0;
        am_hal_timer_interrupt_status_get(false, &ui32IntStatus);

        if ((New_Timer == 0) && (ui32IntStatus & (1 << configTIMER_NUM)))
        {
            // The timer ran to completion and reset itself
            Delta_Sleep = ulReloadValue;
            // Clear the INTSTAT to prevent interrupt handler from counting an extra tick
            am_hal_timer_interrupt_clear(ui32IntStatus);
        }
        else
        {
            Delta_Sleep = New_Timer; // Indicates the time elapsed since we slept
        }
#endif

        Delta_Sleep /= ulTimerCountsForOneTick;

        // Correct System Tick after Sleep
        vTaskStepTick( Delta_Sleep );

		/* Restart System Tick */
#ifdef AM_FREERTOS_USE_STIMER_FOR_TICK

        // Clear the interrupt - to avoid extra tick counting in ISR
#ifdef AM_FREERTOS_STIMER_BACKUP
        am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREA | AM_HAL_STIMER_INT_COMPAREB);
#else
        am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREA);
#endif
        am_hal_stimer_compare_delta_set(0, ulTimerCountsForOneTick);
#ifdef AM_FREERTOS_STIMER_BACKUP
        am_hal_stimer_compare_delta_set(1, ulTimerCountsForOneTick+1);
#endif
#else
        am_hal_timer_clear(configTIMER_NUM);
        am_hal_timer_compare1_set(configTIMER_NUM, ulTimerCountsForOneTick);


        am_hal_timer_start(configTIMER_NUM);
#endif
		/* Re-enable interrupts - see comments above the cpsid instruction()
		above. */
		__asm volatile( "cpsie i" );

	}
}

#endif /* #if configUSE_TICKLESS_IDLE = 2 */

#ifdef AM_FREERTOS_USE_STIMER_FOR_TICK

//*****************************************************************************
//
// Events associated with STimer CMP0 Interrupt
//
//  This is the FreeRTOS System Timer
//
//  Real Time events must be controlled by FreeRTOS as the Stimer is also used for sleep functions.
//  At any time the Stimer->Cmp0 interrupt can be a regular Tick interrupt or
//  an interrupt from a long Deep_Sleep().
//  The Deep Sleep in entered in Port.c->vPortSuppressTicksAndSleep() which is
//  entered from the IDLE task.
//  If no tasks are READY to run, vPortSuppressTicksAndSleep() is called.
//  See tasks.c-> portTASK_FUNCTION(...)
//
//
//
//
//*****************************************************************************
void
xPortStimerTickHandler(uint32_t delta)
{
    uint32_t remainder = 0;
    uint32_t curSTimer;
    uint32_t timerCounts;
    uint32_t numTicksElapsed;
    BaseType_t ctxtSwitchReqd = pdFALSE;

    curSTimer = am_hal_stimer_counter_get();
    //
    // Configure the STIMER->COMPARE_0
    //
    am_hal_stimer_compare_delta_set(0, (ulTimerCountsForOneTick-delta));
#ifdef AM_FREERTOS_STIMER_BACKUP
    am_hal_stimer_compare_delta_set(1, (ulTimerCountsForOneTick-delta+1));
#endif

    timerCounts = curSTimer - g_lastSTimerVal;
    numTicksElapsed = timerCounts/ulTimerCountsForOneTick;
    remainder = timerCounts % ulTimerCountsForOneTick;
    g_lastSTimerVal = curSTimer - remainder;

    //
    // This is a timer a0 interrupt, perform the necessary functions
    // for the tick ISR.
    //
    (void) portSET_INTERRUPT_MASK_FROM_ISR();
    {
        //
        // Increment RTOS tick
        // Allowing for need to increment the tick more than one... to avoid accumulation of
        // error in case of interrupt latencies
        //
        while (numTicksElapsed--)
        {
            ctxtSwitchReqd = (( xTaskIncrementTick() != pdFALSE ) ? pdTRUE : ctxtSwitchReqd);
        }
        if ( ctxtSwitchReqd != pdFALSE )
        {
            //
            // A context switch is required.  Context switching is
            // performed in the PendSV interrupt. Pend the PendSV
            // interrupt.
            //
            portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
        }
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}


//*****************************************************************************
//
// Interrupt handler for the STIMER module Compare 0.
//
//*****************************************************************************
void
am_stimer_cmpr0_isr(void)
{

    //
    // Check the timer interrupt status.
    //
    uint32_t ui32Status = am_hal_stimer_int_status_get(false);
    if (ui32Status & AM_HAL_STIMER_INT_COMPAREA)
    {
        am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREA);

        //
        // Run handlers for the various possible timer events.
        //
        xPortStimerTickHandler(0);
    }
}

#ifdef AM_FREERTOS_STIMER_BACKUP
uint32_t gNumCmpB = 0;
//*****************************************************************************
//
// Interrupt handler for the STIMER module Compare 0.
//
//*****************************************************************************
void
am_stimer_cmpr1_isr(void)
{

    //
    // Check the timer interrupt status.
    //
    uint32_t ui32Status = am_hal_stimer_int_status_get(false);
    if (ui32Status & AM_HAL_STIMER_INT_COMPAREB)
    {
        am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREB);
        gNumCmpB++;
        //
        // Run handlers for the various possible timer events.
        //
        xPortStimerTickHandler(1);
    }
}
#endif

#else // Use Timer


//
// Macro concatenation magic to get the correct am_timerXX_isr function name
// from the configTIMER_NUM value.
//
#define PORT_GET_TIMER_IRQN(x) PORT_GET_TIMER_IRQN2(x)
#define PORT_GET_TIMER_IRQN2(x) TIMER ## x ## _IRQn

#define PORT_GET_TIMER_ISR(x) PORT_GET_TIMER_ISR2(x)

#if configTIMER_NUM < 10
#define PORT_GET_TIMER_ISR2(x) am_timer ## 0 ## x ## _isr
#else
#define PORT_GET_TIMER_ISR2(x) am_timer ## 0 ## x ## _isr
#endif

#define vPortTimerISR PORT_GET_TIMER_ISR(configTIMER_NUM)
#define PORT_TIMER_IRQN PORT_GET_TIMER_IRQN(configTIMER_NUM)

//*****************************************************************************
//
// Events associated with the timer.
//
// Macros will recast this to something like am_timer00_isr()
//
//*****************************************************************************
void
vPortTimerISR(void)
{
    uint32_t ui32Status = 0;

    am_hal_timer_interrupt_status_get(false, &ui32Status);
    am_hal_timer_interrupt_clear(ui32Status);

    if (ui32Status & AM_HAL_TIMER_MASK(configTIMER_NUM, AM_HAL_TIMER_COMPARE1))
    {
        // Restart the one-shot timer for next 'tick'
        am_hal_timer_clear(configTIMER_NUM);
        am_hal_timer_compare1_set(configTIMER_NUM, ulTimerCountsForOneTick);
        am_hal_timer_start(configTIMER_NUM);
        //
        // This is a timer a0 interrupt, perform the necessary functions
        // for the tick ISR.
        //
        (void) portSET_INTERRUPT_MASK_FROM_ISR();

        {
            //
            // Increment RTOS tick
            //
            if ( xTaskIncrementTick() != pdFALSE )
            {

                //
                // A context switch is required.  Context switching is
                // performed in the PendSV interrupt. Pend the PendSV
                // interrupt.
                //
                portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
            }
        }
        portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
    }
}

#endif // AM_FREERTOS_USE_STIMER_FOR_TICK


void vPortSetupTimerInterrupt( void )
{
#ifdef AM_FREERTOS_USE_STIMER_FOR_TICK
    uint32_t oldCfg, stimer_src;
    /* Calculate the constants required to configure the tick interrupt. */
    #if configUSE_TICKLESS_IDLE == 2
    {
#ifdef AM_PART_APOLLO4B
		if (APOLLO4_B0)
		{
			// STIMER with XTAL is not functional in Apollo4 B0, use STIMER_HFRC instead
        	ulTimerCountsForOneTick = (375000 /configTICK_RATE_HZ) ; //( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ );
		}
		else
#endif
		{
        	ulTimerCountsForOneTick = (configSTIMER_CLOCK_HZ /configTICK_RATE_HZ) ; //( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ );
		}
#ifdef AM_FREERTOS_STIMER_BACKUP
        xMaximumPossibleSuppressedTicks = portMAX_32_BIT_NUMBER / ulTimerCountsForOneTick - 1;
#else
        xMaximumPossibleSuppressedTicks = portMAX_32_BIT_NUMBER / ulTimerCountsForOneTick;
#endif
    }
    #endif /* configUSE_TICKLESS_IDLE */
    //
    //
    //
#ifdef AM_FREERTOS_STIMER_BACKUP
    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREA | AM_HAL_STIMER_INT_COMPAREB);
#else
    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREA);
#endif

    //
    // Enable the timer interrupt in the NVIC, making sure to use the
    // appropriate priority level.
    //
#if AM_CMSIS_REGS
    NVIC_SetPriority(STIMER_CMPR0_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(STIMER_CMPR0_IRQn);
#else // AM_CMSIS_REGS
    am_hal_interrupt_priority_set(AM_HAL_INTERRUPT_STIMER_CMPR0, configKERNEL_INTERRUPT_PRIORITY);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_STIMER_CMPR0);
#endif // AM_CMSIS_REGS
#ifdef AM_FREERTOS_STIMER_BACKUP
#if AM_CMSIS_REGS
    NVIC_SetPriority(STIMER_CMPR1_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(STIMER_CMPR1_IRQn);
#else // AM_CMSIS_REGS
    am_hal_interrupt_priority_set(AM_HAL_INTERRUPT_STIMER_CMPR1, configKERNEL_INTERRUPT_PRIORITY);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_STIMER_CMPR1);
#endif // AM_CMSIS_REGS
#endif
    //
    // Configure the STIMER
    //
    oldCfg = am_hal_stimer_config(AM_HAL_STIMER_CFG_FREEZE);
    g_lastSTimerVal = am_hal_stimer_counter_get();
    am_hal_stimer_compare_delta_set(0, ulTimerCountsForOneTick);
	// STIMER Source Configuration
#ifdef AM_PART_APOLLO4B
	if (APOLLO4_B0)
	{
		// STIMER with XTAL is not functional in Apollo4 B0, use STIMER_HFRC instead
		stimer_src = AM_HAL_STIMER_HFRC_375KHZ;
	}
	else
#endif
	{
		stimer_src = configSTIMER_CLOCK;
	}
#ifdef AM_FREERTOS_STIMER_BACKUP
    am_hal_stimer_compare_delta_set(1, ulTimerCountsForOneTick+1);
#if AM_CMSIS_REGS
    am_hal_stimer_config((oldCfg & ~(AM_HAL_STIMER_CFG_FREEZE | STIMER_STCFG_CLKSEL_Msk)) | stimer_src | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE | AM_HAL_STIMER_CFG_COMPARE_B_ENABLE);
#else // AM_CMSIS_REGS
    am_hal_stimer_config((oldCfg & ~(AM_HAL_STIMER_CFG_FREEZE|AM_REG_CTIMER_STCFG_CLKSEL_M)) | stimer_src | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE | AM_HAL_STIMER_CFG_COMPARE_B_ENABLE);
#endif // AM_CMSIS_REGS
#else
#if AM_CMSIS_REGS
    am_hal_stimer_config((oldCfg & ~(AM_HAL_STIMER_CFG_FREEZE | STIMER_STCFG_CLKSEL_Msk)) | stimer_src | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE);
#else // AM_CMSIS_REGS
    am_hal_stimer_config((oldCfg & ~(AM_HAL_STIMER_CFG_FREEZE|AM_REG_CTIMER_STCFG_CLKSEL_M)) | stimer_src | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE);
#endif // AM_CMSIS_REGS

#endif
#else // AM_FREERTOS_USE_STIMER_FOR_TICK

    /* Calculate the constants required to configure the tick interrupt. */
    #if configUSE_TICKLESS_IDLE == 2
    {
        ulTimerCountsForOneTick = ( configTIMER_CLOCK_HZ/configTICK_RATE_HZ) ;
        xMaximumPossibleSuppressedTicks = portMAX_32_BIT_NUMBER / ulTimerCountsForOneTick;
    }
    #endif /* configUSE_TICKLESS_IDLE */

    am_hal_timer_config_t xTimerConfig;

    am_hal_timer_default_config_set(&xTimerConfig);
    xTimerConfig.eInputClock = configTIMER_CLOCK;
    xTimerConfig.eFunction = AM_HAL_TIMER_FN_UPCOUNT;
    xTimerConfig.ui32Compare1 = ulTimerCountsForOneTick;

    //
    // Configure the timer frequency and mode.
    //
    if ( am_hal_timer_config(configTIMER_NUM, &xTimerConfig) )
    {
        // Error: timer configuration failed. Check the exact return code for
        // more detailed info.
        while (1);
    }

    am_hal_timer_clear(configTIMER_NUM);

    //
    // Enable the interrupt for timer A0
    //
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(configTIMER_NUM, AM_HAL_TIMER_COMPARE1));
    am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(configTIMER_NUM, AM_HAL_TIMER_COMPARE1));

    //
    // Enable the timer interrupt in the NVIC, making sure to use the
    // appropriate priority level.
    //
#if AM_CMSIS_REGS
    NVIC_SetPriority(PORT_TIMER_IRQN, NVIC_configKERNEL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(PORT_TIMER_IRQN);
#else // AM_CMSIS_REGS
    am_hal_interrupt_priority_set(AM_HAL_INTERRUPT_TIMER, configKERNEL_INTERRUPT_PRIORITY);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_TIMER);
#endif // AM_CMSIS_REGS

    //
    // Enable the timer.
    //
    am_hal_timer_start(configTIMER_NUM);


#endif // AM_FREERTOS_USE_STIMER_FOR_TICK
}


#endif /* configOVERRIDE_DEFAULT_TICK_CONFIGURATION */


