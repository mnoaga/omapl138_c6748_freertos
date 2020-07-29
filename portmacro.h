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

#ifndef PORTMACRO_H
#define PORTMACRO_H

/* Hardware specifics. */
#include <c6x.h>
#include "interrupt.h"

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */

#define portCHAR        char
#define portFLOAT       float
#define portDOUBLE      double
#define portLONG        long
#define portSHORT       short
#define portSTACK_TYPE  unsigned portLONG
#define portBASE_TYPE   portLONG

typedef long            BaseType_t;
typedef portSTACK_TYPE  StackType_t;
typedef unsigned long   UBaseType_t;

typedef uint32_t        TickType_t;

#define portMAX_DELAY       ( TickType_t ) 0xffffffffUL
#define portSTACK_GROWTH    ( -1 )
#define portTICK_PERIOD_MS  ( ( portTickType ) 1000 / configTICK_RATE_HZ )
#define portBYTE_ALIGNMENT  ( 8 )

/*-----------------------------------------------------------*/

/* Interrupt control macros. */

/*
 * By default the event number 14 (IDMAINT1 C674x-EMC) is routed to the vector number
 * C674X_MASK_INT14. So, if you have to manage this event please route it elsewhere
 * and map another unused and unpowered peripheral's interrupt.
 */

#define portYIELD_INT_MASK    ( C674X_MASK_INT14 )    /* Manual context switch interrupt number. */

#define portTICK_INT_MASK     ( C674X_MASK_INT15 )    /* Tick Timer interrupt number. */

/*
 * New instructions that allow for simpler and safer manipulation of the GIE bit
 * (c.f sprufe8b c6748 Instruction Set Ref. Manual )
 * NB: Calling the macro in a nesting fashion {DINT DINT RINT RINT} will leave interrupts disabled.
 */

#define portDISABLE_INTERRUPTS()    asm("	DINT")

#define portENABLE_INTERRUPTS()    asm("	RINT")
/*-----------------------------------------------------------*/

/*
 * Critical nesting counts are stored in the TCB.
 * Posted by Richard B. on March 15, 2010:
 * Most ports (all other ports?) store a critical nesting count per task.
 * Whether this is stored in the TCB or task stack does not make a difference
 * to the amount of RAM required. Storing it in the TCB is much simpler
 * (less context switch code) but can also be less flexible and less
 * efficient - so it really depends on the architecture.
 */

#define portCRITICAL_NESTING_IN_TCB (1)

/* The critical nesting functions defined within tasks.c. */
extern void vTaskEnterCritical(void);
extern void vTaskExitCritical(void);

#define portENTER_CRITICAL()    vTaskEnterCritical()
#define portEXIT_CRITICAL()     vTaskExitCritical()
/*-----------------------------------------------------------*/

/*
 * Manual context switch called by portYIELD or taskYIELD.
 * NB: Writing 0 to any bit of ISR has no effect c.f. SPUFE8B-C674x-Instruction-set.
 * So no need to do read-modify-write.
 */
#define portYIELD() ISR = (1UL << portYIELD_INT_MASK); asm("    NOP 5")

#define portYIELD_FROM_ISR( x ) if( x != pdFALSE ) portYIELD()
/*-----------------------------------------------------------*/

/* Hardware specifics: 5 nop to eventually let multi-cycles instructions to end up. */
#define portNOP()		asm("	NOP	5")
/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )

#endif /* PORTMACRO_H */
