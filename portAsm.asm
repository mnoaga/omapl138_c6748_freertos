; /*
; * FreeRTOS Kernel V10.3.1
; * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
; *
; * Permission is hereby granted, free of charge, to any person obtaining a copy of
; * this software and associated documentation files (the "Software"), to deal in
; * the Software without restriction, including without limitation the rights to
; * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
; * the Software, and to permit persons to whom the Software is furnished to do so,
; * subject to the following conditions:
; *
; * The above copyright notice and this permission notice shall be included in all
; * copies or substantial portions of the Software.
; *
; * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
; * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
; * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
; * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
; * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
; *
; * http://www.FreeRTOS.org
; * http://aws.amazon.com/freertos
; *
; * 1 tab == 4 spaces!
; */


;***************************************************************************************
; This file implements the core of the C674x port.
; Notice that unless manually implemented or special event occurence (NMI), the cpu will
; not nest interrupt. (sprufe8b-c674x-Instruction-Set-Ref-Guide)
;***************************************************************************************

;--------------------------------------------------------------------------------------
; Imported Symbols
;--------------------------------------------------------------------------------------
	.global vPortYieldInterruptClear
	.global vPortTickInterruptClear
	.global xTaskIncrementTick
	.global vTaskSwitchContext
	.global vPortTickHandler
	.global	pxCurrentTCB

;--------------------------------------------------------------------------------------
; Exported Symbols
;--------------------------------------------------------------------------------------
	.def	vPortSartFirstTask
	.def	_vPortYieldISR			; Shall be registered in the Interrupt Vector Table maunally.
	.def	_vPortTickISR			; Shall be registered in the Interrupt Vector Table maunally.

;--------------------------------------------------------------------------------------
; Substitution Symbols
;--------------------------------------------------------------------------------------
	.asg "B15", SP

;--------------------------------------------------------------------------------------
; Substitution Symbols (*)
;--------------------------------------------------------------------------------------
	.define "0x01800040", REG_DSPINTC_EVTCLR	; DSP INTC event clear register address .
	.define "0x01F0C044", REG_TMR2_INTCTLSTAT	; Timer interrupt control and status rigister address.

;--------------------------------------------------------------------------------------

	.sect ".text"

;--------------------------------------------------------------------------------------


;--------------------------------------------------------------------------------------
; Clear the tick interrupt (Optimized).
;---
portCLEAR_TICK_INT .macro

	MVKL    REG_DSPINTC_EVTCLR,    B4       ; Load DSP INTC event clear register address.
	MVKH    REG_DSPINTC_EVTCLR,    B4
	MVKL    0x02000000,            B0       ; Load the event flag.
	MVKH    0x02000000,            B0
	STW     B0,                   *B4[0]    ; Clear the event. (writing 0 on bits has no effect... see Ref. manual.)
	;----------------------------------------------------------------------
	MVKL    REG_TMR2_INTCTLSTAT,   B4       ; Load Timer interrupt control and status rigister address.
	MVKH    REG_TMR2_INTCTLSTAT,   B4
	MVKL    0x00000003,            B0       ; Load the flag to be cleared.
	MVKH    0x00000003,            B0
	STW     B0,                   *B4[0]    ; Clear the interrupt. (writing 0 on bits has no effect... see Ref. manual.)

	.endm
;---
; End of portCLEAR_TICK_INT
;-------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------
; Clear the yield interrupt (optimized).
;---

portCLEAR_YIELD_INT .macro

	MVKL    0x00004000, B0      ; Load the interrupt flag.
	MVKH    0x00004000, B0
	MVC     B0,         ICR     ;Clear the interrupt. (writing 0 on bits has no effect... see Ref. manual.)

	.endm

;---
; End of portCLEAR_YIELD_INT
;-------------------------------------------------------------------------------------


;--------------------------------------------------------------------------------------
; Save the context of the current task.
;---
portSAVE_CONTEXT .macro

	;---
	; Working registers.
	;---

	;-----------------------------------------------------------------------
	; As "STDW" support limited offset range, we manage working and control regs seperatelly.
	;-----------------------------------------------------------------------
	ADDK    -248,       SP       ; Set up SP to save working regs with positive offset (248=31*8).
	;-----------------------------------------------------------------------
	STDW    B31:B30,    *+SP[30]
	STDW    B29:B28,    *+SP[29]
	STDW    B27:B26,    *+SP[28]
	STDW    B25:B24,    *+SP[27]
	STDW    B23:B22,    *+SP[26]
	STDW    B21:B20,    *+SP[25]
	STDW    B19:B18,    *+SP[24]
	STDW    B17:B16,    *+SP[23]
	;---------------------------
	STDW    B13:B12,    *+SP[22]
	STDW    B11:B10,    *+SP[21]
	STDW    B9:B8,      *+SP[20]
	STDW    B7:B6,      *+SP[19]
	STDW    B5:B4,      *+SP[18]
	STDW    B3:B2,      *+SP[17]
	STDW    B1:B0,      *+SP[16]
	;---------------------------
	STDW    A31:A30,    *+SP[15]
	STDW    A29:A28,    *+SP[14]
	STDW    A27:A26,    *+SP[13]
	STDW    A25:A24,    *+SP[12]
	STDW    A23:A22,    *+SP[11]
	STDW    A21:A20,    *+SP[10]
	STDW    A19:A18,    *+SP[9]
	STDW    A17:A16,    *+SP[8]
	STDW    A15:A14,    *+SP[7]
	STDW    A13:A12,    *+SP[6]
	STDW    A11:A10,    *+SP[5]
	STDW    A9:A8,      *+SP[4]
	STDW    A7:A6,      *+SP[3]
	STDW    A5:A4,      *+SP[2]
	STDW    A3:A2,      *+SP[1]
	STDW    A1:A0,      *+SP[0]
	;-----------------------------------------------------------------------
	ADDK    -40,        SP         ; Set up SP to save control regs with positive offset (40 = 5*8)
	;-----------------------------------------------------------------------
	MVC    IRP,         B1
	MVC    ILC,         B0
	MVC    RILC,        B3
	MVC    NRP,         B2
	MVC    ITSR,        B5
	MVC    FADCR,       B4
	MVC    FAUCR,       B7
	MVC    FMCR,        B6
	MVC    AMR,         B9
	MVC    CSR,         B8
	;---
	STDW    B1:B0,      *+SP[4]
	STDW    B3:B2,      *+SP[3]
	STDW    B5:B4,      *+SP[2]
	STDW    B7:B6,      *+SP[1]
	STDW    B9:B8,      *+SP[0]
	;-----------------------------------------------------------------------
	; Save SP on the current task TCB
	;-----------------------------------------------------------------------
	MVKL    pxCurrentTCB,    A0    ; lsb(A0) = lsb(&pxCurrentTCB)
	MVKH    pxCurrentTCB,    A0    ; msb(A0) = msb(&pxCurrentTCB)

	LDW     *A0,             A0    ; A0 = pxCurrentTCB
	NOP     5
	STW     SP,             *A0    ; *pxCurrentTCB = SP, i.e. pxCurrentTCB->pxTopOfStack = SP
	;-----------------------------------------------------------------------
	ADDK    -8,              SP    ; Move SP down as it is still pointing the last saved paire.
	;-----------------------------------------------------------------------

	.endm
;---
; End of portSAVE_CONTEXT
;-----------------------------------------------------------------------


;-----------------------------------------------------------------------
; Restore the context of the current task.
;---
portRESTORE_CONTEXT .macro

	;-----------------------------------------------------------------------
	; Read SP from current task TCB
	;-----------------------------------------------------------------------
	MVKL    pxCurrentTCB,    A0    ; lsb(A0) = lsb(&pxCurrentTCB)
	MVKH    pxCurrentTCB,    A0    ; msb(A0) = msb(&pxCurrentTCB)
	;-----------------------------------------------------------------------
	LDW     *A0,             A0    ; A0 = pxCurrentTCB
	NOP     5
	;-----------------------------------------------------------------------
	LDW     *A0,             SP    ; B15 = *pxCurrentTCB, i.e SP = pxCurrentTCB->pxTopOfStack
	NOP     5
	;-----------------------------------------------------------------------

	;---
	; Control registers.
	;---

	;-----------------------------------------------------------------------
	LDDW    *+SP[0],    B9:B8
	LDDW    *+SP[1],    B7:B6
	LDDW    *+SP[2],    B5:B4
	LDDW    *+SP[3],    B3:B2
	LDDW    *+SP[4],    B1:B0
	NOP     5
	;-----------------------------------------------------------------------
	MVC    B8,    CSR
	MVC    B9,    AMR
	MVC    B6,    FMCR
	MVC    B7,    FAUCR
	MVC    B4,    FADCR
	MVC    B5,    ITSR
	MVC    B2,    NRP
	MVC    B3,    RILC
	MVC    B0,    ILC
	MVC    B1,    IRP
	;-----------------------------------------------------------------------
	ADDK   40,    SP    ; Update SP so that we may use full offset range.
	;-----------------------------------------------------------------------

	;---
	; Working registers.
	;---

	LDDW    *+SP[0],    A1:A0
	LDDW    *+SP[1],    A3:A2
	LDDW    *+SP[2],    A5:A4
	LDDW    *+SP[3],    A7:A6
	LDDW    *+SP[4],    A9:A8
	LDDW    *+SP[5],    A11:A10
	LDDW    *+SP[6],    A13:A12
	LDDW    *+SP[7],    A15:A14
	LDDW    *+SP[8],    A17:A16
	LDDW    *+SP[9],    A19:A18
	LDDW    *+SP[10],   A21:A20
	LDDW    *+SP[11],   A23:A22
	LDDW    *+SP[12],   A25:A24
	LDDW    *+SP[13],   A27:A26
	LDDW    *+SP[14],   A29:A28
	LDDW    *+SP[15],   A31:A30
	;--------------------------
	LDDW    *+SP[16],   B1:B0
	LDDW    *+SP[17],   B3:B2
	LDDW    *+SP[18],   B5:B4
	LDDW    *+SP[19],   B7:B6
	LDDW    *+SP[20],   B9:B8
	LDDW    *+SP[21],   B11:B10
	LDDW    *+SP[22],   B13:B12
	;--------------------------
	LDDW    *+SP[23],   B17:B16
	LDDW    *+SP[24],   B19:B18
	LDDW    *+SP[25],   B21:B20
	LDDW    *+SP[26],   B23:B22
	LDDW    *+SP[27],   B25:B24
	LDDW    *+SP[28],   B27:B26
	LDDW    *+SP[29],   B29:B28
	LDDW    *+SP[30],   B31:B30
	;-----------------------------------------------------------------------
	ADDK    248,        SP        ; Update SP.
	;-----------------------------------------------------------------------
	NOP     4

	.endm
;---
; End of portRESTORE_CONTEXT
;-----------------------------------------------------------------------

;-----------------------------------------------------------------------
; Yield ISR
;---
_vPortYieldISR: .asmfunc

	portSAVE_CONTEXT                              ; Save the context of the current task.

	CALLP    .S2     vTaskSwitchContext,    B3    ; Call the scheduler to select the next task.
	NOP      5

	;----------------------------------------------------------------------
	; CALLP .S2 vPortYieldInterruptClear, B3      ; Clear the yield (manual) interrupt.
	; NOP	5
	;----------------------------------------------------------------------

	portCLEAR_YIELD_INT                           ; Optimized version of yield interrupt clear function call (just above.)

	portRESTORE_CONTEXT                           ; Restore the context of the next task to run.

	B    IRP                                      ; Return from interrupt.
	NOP  5

	.endasmfunc
;---
; End of _vPortYieldISR
;-----------------------------------------------------------------------

;-----------------------------------------------------------------------
; Preemptive tick ISR
;---
_vPortTickISR: .asmfunc

	portSAVE_CONTEXT                           ; Save the context of the current task.

	CALLP    .S2    vPortTickHandler,    B3    ; Increment the RTOS tick and eventually perform scheduling.
	NOP      5

	;----------------------------------------------------------------------
	;CALLP .S2 vPortTickInterruptClear, B3     ; Clear the timer interrupt.
	;NOP 5
	;----------------------------------------------------------------------

	portCLEAR_TICK_INT                         ; Optimized version of timer interrupt clear function call (just above.)

	portRESTORE_CONTEXT                        ; Restore the context of the next task to run.

	B    IRP                                   ; Return from interrupt.
	NOP  5

    .endasmfunc
;---
; End of _vPortTickISR
;-----------------------------------------------------------------------

;-----------------------------------------------------------------------
; Start the first selected task.
;-----------------------------------------------------------------------
; According to the current implementation we must retun using IRP.
; "B IRP" restore TSR with ITSR(SGIE & GIE bits) so that interrupt
; are enabled since GIE bit of CSR and TSR are physically the same.
;;-----------------------------------------------------------------------
vPortSartFirstTask:	.asmfunc

	portRESTORE_CONTEXT    ; Restore the context of the first task to run.

	B    IRP               ; Return on IRP as if returning from interrupt.
	NOP  5

	.endasmfunc
;---
; End of vPortSartFirstTask
;-----------------------------------------------------------------------

