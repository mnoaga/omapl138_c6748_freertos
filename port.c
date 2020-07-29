/*
 * FreeRTOS Kernel V10.3.1
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/*-----------------------------------------------------------------------*
 * Implementation of functions defined in portable.h for the C6748 port. *
 *-----------------------------------------------------------------------*/

/* Standard includes */
#include <stdio.h>
#include <stdint.h>

/* Hardware specifics */
#include <c6x.h>
#include "hw_types.h"
#include "hw_dspintc.h"
#include "interrupt.h"
#include "soc_OMAPL138.h"
#include "hw_syscfg0_OMAPL138.h"
#include "timer.h"

/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"

/* Constants. */
#define portTMR_PERIOD_LSB32        ( 0x00024706 )                      /* TMR2 period msb for ~1ms     */
#define portTMR_PERIOD_MSB32        ( 0x0 )                             /* TMR2 period lsb for ~1ms     */
#define portINITIAL_ITSR            ( ( StackType_t ) 0x00000003 )      /* Init value for ITSR register.*/

/* We require the address of the pxCurrentTCB variable, but don't want to know
 any details of its type. */
typedef void TCB_t;
extern volatile TCB_t * volatile pxCurrentTCB;
/*-----------------------------------------------------------*/

/*
 * Sets up the periodic ISR used for the RTOS tick.
 */
void vPortSetupTimerInterrupt(void);
/*-----------------------------------------------------------*/

/*
 * Function to start the first task executing - written in assembly.
 */
void vPortSartFirstTask(void);
/*-----------------------------------------------------------*/

/*
 * Used to catch tasks that attempt to return from their implementing function.
 */
static void prvTaskExitError(void);
/*-----------------------------------------------------------*/

/*
 * A version of IntEnable() that does not disable interrupts.
 * This is to avoid a DINT-DINT-RINT-RINT sequence.
 */
static void portIntEnable(unsigned int cpuINT)
{
    /* Check the CPU maskable interrupt number */
    configASSERT(((cpuINT >= 4) && (cpuINT <= 15)));

    /* Enable CPU maskable interrupt */
    IER |= (1 << cpuINT);
}
/*-----------------------------------------------------------*/

/*
 * A version of IntEventMap() that does not disable interrupt.
 * This is to avoid a DINT-DINT-RINT-RINT sequence.
 */
static void vPortIntEventMap(unsigned int cpuINT, unsigned int sysINT)
{
    unsigned int dspintcREG;

    /* Check the CPU maskable interrupt number */
    configASSERT(((cpuINT >= 4) && (cpuINT <= 15)));

    /* Check the system event number */
    configASSERT((sysINT <= 127));

    /* Get the address of the correct interrupt mux register */
    dspintcREG = SOC_INTC_0_REGS + DSPINTC_INTMUX((cpuINT >> 2));

    /* Clear and set INTSELx with system event number */
    HWREG(dspintcREG) = (HWREG(dspintcREG) & ~DSPINTC_INTMUX_INTSEL(cpuINT))
            | (sysINT << DSPINTC_INTMUX_INTSEL_SHIFT(cpuINT));

    /* Clear any residual interrupt */
    ICR = (1 << cpuINT);
}

/*
 * Clear all pending events.
 */
static void portEventClearAll(void)
{
    unsigned int dspintcREG;

    /* 
     * Get the address of the correct event register.
     * Clear the corresponding bit within the event clear register.
     * Regs are: 0x01800040 0x01800044 0x01800048 0x0180004C 
     */

    dspintcREG = SOC_INTC_0_REGS + DSPINTC_EVTCLR((0 >> 5));
    HWREG(dspintcREG) = 0xFFFFFFFF;
    dspintcREG = SOC_INTC_0_REGS + DSPINTC_EVTCLR((32 >> 5));
    HWREG(dspintcREG) = 0xFFFFFFFF;
    dspintcREG = SOC_INTC_0_REGS + DSPINTC_EVTCLR((64 >> 5));
    HWREG(dspintcREG) = 0xFFFFFFFF;
    dspintcREG = SOC_INTC_0_REGS + DSPINTC_EVTCLR((96 >> 5));
    HWREG(dspintcREG) = 0xFFFFFFFF;
}
/*-----------------------------------------------------------*/

/*
 * Clear the tick timer interrupt.
 */
void vPortTickInterruptClear(void)
{
    /* Clear interrupt status in DSPINTC. */
    IntEventClear(SYS_INT_T64P2_TINTALL);

    /* Clear interrupt status in the timer device. */
    TimerIntStatusClear(SOC_TMR_2_REGS, TMR_INTSTAT12_TIMER_NON_CAPT);
}
/*-------------------------------------------------------------*/

/*
 *  Clear the yield interrupt.
 */
void vPortYieldInterruptClear(void)
{
    ICR = (1UL << portYIELD_INT_MASK);
}
/*-------------------------------------------------------------*/

/*
 * Hardware initialization to generate the RTOS tick.
 */
void vPortSetupTimerInterrupt(void)
{
    /* Set the timer configuration. */
    TimerConfigure(SOC_TMR_2_REGS, TMR_CFG_32BIT_UNCH_CLK_BOTH_INT);

    /* Set the 64 bit timer period. */
    TimerPeriodSet(SOC_TMR_2_REGS, TMR_TIMER12, portTMR_PERIOD_LSB32);

    /* Map Timer interrupts to DSP maskable interrupt. */
    vPortIntEventMap(portTICK_INT_MASK, SYS_INT_T64P2_TINTALL);

    /* Enable DSP interrupt that the timer event is mapped to. */
    portIntEnable(portTICK_INT_MASK);

    /* Enable the timer interrupt. */
    TimerIntEnable(SOC_TMR_2_REGS, TMR_INT_TMR12_NON_CAPT_MODE);

    /* Start the timer. */
    TimerEnable(SOC_TMR_2_REGS, TMR_TIMER12, TMR_ENABLE_CONT);
}
/*-------------------------------------------------------------*/

/*
 * Initialize the stack of a task to look exactly as if a call to
 * portSAVE_CONTEXT has occurred.
 *
 * See the header file portable.h.
 */
StackType_t *pxPortInitialiseStack(StackType_t *pxTopOfStack,
                                   TaskFunction_t pxCode, void *pvParameters)
{
    /* General purpose registers.*/

    *--pxTopOfStack = 0x31B31B31;                     /* B31 */
    *--pxTopOfStack = 0x30B30B30;                     /* B30 */
    *--pxTopOfStack = 0x29B29B29;                     /* B29 */
    *--pxTopOfStack = 0x28B28B28;                     /* B28 */
    *--pxTopOfStack = 0x27B27B27;                     /* B27 */
    *--pxTopOfStack = 0x26B26B26;                     /* B26 */
    *--pxTopOfStack = 0x25B25B25;                     /* B25 */
    *--pxTopOfStack = 0x24B24B24;                     /* B24 */
    *--pxTopOfStack = 0x23B23B23;                     /* B23 */
    *--pxTopOfStack = 0x22B22B22;                     /* B22 */
    *--pxTopOfStack = 0x21B21B21;                     /* B21 */
    *--pxTopOfStack = 0x20B20B20;                     /* B20 */
    *--pxTopOfStack = 0x19B19B19;                     /* B19 */
    *--pxTopOfStack = 0x18B18B18;                     /* B18 */
    *--pxTopOfStack = 0x17B17B17;                     /* B17 */
    *--pxTopOfStack = 0x16B16B16;                     /* B16 */
                                                      /* B15: Stack pointer (SP): saved in the TCB structure.*/
                                                      /* B14: Data page pointer (DP): points to the beginning of the .bss section. */
    *--pxTopOfStack = 0x13B13B13;                     /* B13 */
    *--pxTopOfStack = 0x12B12B12;                     /* B12 */
    *--pxTopOfStack = 0x11B11B11;                     /* B11 */
    *--pxTopOfStack = 0x10B10B10;                     /* B10 */
    *--pxTopOfStack = 0xB9B9B9B9;                     /* B9 */
    *--pxTopOfStack = 0xB8B8B8B8;                     /* B8 */
    *--pxTopOfStack = 0xB7B7B7B7;                     /* B7 */
    *--pxTopOfStack = 0xB6B6B6B6;                     /* B6 */
    *--pxTopOfStack = 0xB5B5B5B5;                     /* B5 */
    *--pxTopOfStack = 0xB4B4B4B4;                     /* B4 */
    *--pxTopOfStack = (StackType_t) prvTaskExitError; /* B3: Return register (address to return to). */
    *--pxTopOfStack = 0xB2B2B2B2;                     /* B2 */
    *--pxTopOfStack = 0xB1B1B1B1;                     /* B1 */
    *--pxTopOfStack = 0xB0B0B0B0;                     /* B0 */

    *--pxTopOfStack = 0x31A31A31;                     /* A31 */
    *--pxTopOfStack = 0x30A30A30;                     /* A30 */
    *--pxTopOfStack = 0x29A29A29;                     /* A29 */
    *--pxTopOfStack = 0x28A28A28;                     /* A28 */
    *--pxTopOfStack = 0x27A27A27;                     /* A27 */
    *--pxTopOfStack = 0x26A26A26;                     /* A26 */
    *--pxTopOfStack = 0x25A25A25;                     /* A25 */
    *--pxTopOfStack = 0x24A24A24;                     /* A24 */
    *--pxTopOfStack = 0x23A23A23;                     /* A23 */
    *--pxTopOfStack = 0x22A22A22;                     /* A22 */
    *--pxTopOfStack = 0x21A21A21;                     /* A21 */
    *--pxTopOfStack = 0x20A20A20;                     /* A20 */
    *--pxTopOfStack = 0x19A19A19;                     /* A19 */
    *--pxTopOfStack = 0x18A18A18;                     /* A18 */
    *--pxTopOfStack = 0x17A17A17;                     /* A17 */
    *--pxTopOfStack = 0x16A16A16;                     /* A16 */
    *--pxTopOfStack = 0x00000000;                     /* A15: Frame pointer (FP).*/
    *--pxTopOfStack = 0x14A14A14;                     /* A14 */
    *--pxTopOfStack = 0x13A13A13;                     /* A13 */
    *--pxTopOfStack = 0x12A12A12;                     /* A12 */
    *--pxTopOfStack = 0x11A11A11;                     /* A11 */
    *--pxTopOfStack = 0x10A10A10;                     /* A10 */
    *--pxTopOfStack = 0xA9A9A9A9;                     /* A9 */
    *--pxTopOfStack = 0xA8A8A8A8;                     /* A8 */
    *--pxTopOfStack = 0xA7A7A7A7;                     /* A7 */
    *--pxTopOfStack = 0xA6A6A6A6;                     /* A6 */
    *--pxTopOfStack = 0xA5A5A5A5;                     /* A5 */
    *--pxTopOfStack = (StackType_t) pvParameters;     /* A4: Argument 1 or return value, the task parameter.*/
    *--pxTopOfStack = 0xA3A3A3A3;                     /* A3 */
    *--pxTopOfStack = 0xA2A2A2A2;                     /* A2 */
    *--pxTopOfStack = 0xA1A1A1A1;                     /* A1 */
    *--pxTopOfStack = 0xA0A0A0A0;                     /* A0 */

    /* Control registers.*/

    *--pxTopOfStack = (StackType_t) pxCode;           /* IRP. */
    *--pxTopOfStack = 0;                              /* ILC. */
    *--pxTopOfStack = 0;                              /* RILC. */
    *--pxTopOfStack = (StackType_t) pxCode;           /* NRP. */
    *--pxTopOfStack = portINITIAL_ITSR;               /* ITSR: GIE=1 & SGIE=1. */

    /*
     *  If you are reading bits out of the floating pointer control registers,
     * and if the interrupt service routine (or any called function) performs
     * floating point operations, then the relevant floating point control
     * registers should be saved and restored.	(spru187u-c6000-compiler-ug)
     */

    *--pxTopOfStack = 0; /* FADCR.*/
    *--pxTopOfStack = 0; /* FAUCR.*/
    *--pxTopOfStack = 0; /* FMCR. */

    /*
     * The AMR must be set to 0 when interrupts are enabled, or the SAVE_AMR and STORE_AMR macros
     * should be used in all interrupts.
     * If an interrupt service routine modifies the SAT bit, then the routine should be written
     * to save and restore the CSR.	(spru187u-c6000-compiler-ug)
     */

    *--pxTopOfStack = 0; /* AMR.*/
    *--pxTopOfStack = 0; /* CSR.*/

    return pxTopOfStack;
}
/* ----------------------------------------------------------- */

/*
 * Function to start the first task executing - written in assembly as direct
 * access to registers is required.
 */
portBASE_TYPE xPortStartScheduler(void)
{
    extern void vPortSartFirstTask(void);

#ifndef APP_INIT_INTC
    /*
     *  Setup the DSP Interrupt Controller (INTC).
     * This code shall be disabled if the application initializes the interrupt controller.
     */
    IntDSPINTCInit();

    /* Clear all pending event. */
    portEventClearAll();
#endif

    /*
     * By default, an event (n°14) is mapped to the interrupt number (n°14) we want to use as
     * Yield interrupt. Therefore we add this event to the exception combiner to track the occurrence
     * of this unused event. If required application may route this event on another vector or
     * combine it with the Yield interrupt by configuring the event combiner accordingly. In that case
     * the interrupt handler will require to check which event occurred before processing.
     */
    ExcCombineAdd(SYS_INT_IDMAINT1);

    /* Enable DSP interrupt used as Yield interrupt. */
    portIntEnable(portYIELD_INT_MASK);

    /* Configure and enable tick timer interrupt. */
    vPortSetupTimerInterrupt();

    /* Start the first task. */
    vPortSartFirstTask();

    return 0;
}
/*-------------------------------------------------------------*/

/*
 * RTOS tick handler called by the tick ISR written in assembly.
 */
void vPortTickHandler(void)
{

#if ( configUSE_PREEMPTION == 1 )
    /* Get the scheduler to update the task states following the tick. */
    if (xTaskIncrementTick() != pdFALSE)
    {
        /* Switch in the context of the next task to be run. */
        vTaskSwitchContext();
    }
#else
    xTaskIncrementTick();
#endif

}
/*-------------------------------------------------------------*/

/*
 * Used to catch tasks that attempt to return from their implementing function.
 */
static void prvTaskExitError(void)
{
    /* 
     * A function that implements a task must not exit or attempt to return to
     * its caller as there is nothing to return to.  If a task wants to exit it
     * should instead call vTaskDelete( NULL ).
     */

    portDISABLE_INTERRUPTS();

    for (;;)
        ;
}
/*-------------------------------------------------------------*/

/*
 * scheduler end routine.
 */
void vPortEndScheduler(void)
{
    /* Interrupt are already disabled when this function is called. */
}
/* ----------------------------------------------------------- */
