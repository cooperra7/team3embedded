/*******************************************************************************
 System Interrupt Source File

  File Name:
    sys_interrupt_a.S

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the 
    interrupt sub-system.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END
/*
*********************************************************************************************************
*                                           INCLUDES
*********************************************************************************************************
*/
#include <xc.h>



#include "ISR_Support.h"


/* TMR Instance 0 Interrupt */
   .extern  IntHandlerDrvTmrInstance0

   .section	.vector_8,code, keep
   .equ     __vector_dispatch_8, IntVectorDrvTmrInstance0
   .global  __vector_dispatch_8
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvTmrInstance0

IntVectorDrvTmrInstance0:
    la    $26,  _IntHandlerDrvTmrInstance0         
    jr    $26
    nop
	.end	IntVectorDrvTmrInstance0

   .section	.DrvTmrInstance0_vector_text, code, keep
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  _IntHandlerDrvTmrInstance0

_IntHandlerDrvTmrInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvTmrInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	_IntHandlerDrvTmrInstance0




/* USART Instance 0 Interrupt */

   .extern  IntHandlerDrvUsartInstance0

   .section	.vector_24,code, keep
   .equ     __vector_dispatch_24, IntVectorDrvUsartInstance0
   .global  __vector_dispatch_24
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvUsartInstance0

IntVectorDrvUsartInstance0:
    la    $26,  _IntHandlerDrvUsartInstance0         
    jr    $26
    nop
	.end	IntVectorDrvUsartInstance0

   .section	.DrvUsartInstance0_vector_text, code, keep
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  _IntHandlerDrvUsartInstance0

_IntHandlerDrvUsartInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvUsartInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	_IntHandlerDrvUsartInstance0



/* USART Instance 1 Interrupt */

   .extern  IntHandlerDrvUsartInstance1

   .section	.vector_32,code, keep
   .equ     __vector_dispatch_32, IntVectorDrvUsartInstance1
   .global  __vector_dispatch_32
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvUsartInstance1

IntVectorDrvUsartInstance1:
    la    $26,  _IntHandlerDrvUsartInstance1         
    jr    $26
    nop
	.end	IntVectorDrvUsartInstance1

   .section	.DrvUsartInstance1_vector_text, code, keep
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  _IntHandlerDrvUsartInstance1

_IntHandlerDrvUsartInstance1:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvUsartInstance1
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	_IntHandlerDrvUsartInstance1


/* IC Instance 0 Interrupt */

   .extern  IntHandlerDrvICInstance0

   .section	.vector_13,code, keep
   .equ     __vector_dispatch_13, IntVectorDrvICInstance0
   .global  __vector_dispatch_13
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvICInstance0

IntVectorDrvICInstance0:
    la    $26,  _IntHandlerDrvICInstance0         
    jr    $26
    nop
	.end	IntVectorDrvICInstance0

   .section	.DrvICInstance0_vector_text, code, keep
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  _IntHandlerDrvICInstance0

_IntHandlerDrvICInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvICInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	_IntHandlerDrvICInstance0


/* IC Instance 1 Interrupt */

   .extern  IntHandlerDrvICInstance1

   .section	.vector_5,code, keep
   .equ     __vector_dispatch_5, IntVectorDrvICInstance1
   .global  __vector_dispatch_5
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvICInstance1

IntVectorDrvICInstance1:
    la    $26,  _IntHandlerDrvICInstance1         
    jr    $26
    nop
	.end	IntVectorDrvICInstance1

   .section	.DrvICInstance1_vector_text, code, keep
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  _IntHandlerDrvICInstance1

_IntHandlerDrvICInstance1:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvICInstance1
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	_IntHandlerDrvICInstance1


/* IC Instance 2 Interrupt */

   .extern  IntHandlerDrvICInstance2

   .section	.vector_17,code, keep
   .equ     __vector_dispatch_17, IntVectorDrvICInstance2
   .global  __vector_dispatch_17
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvICInstance2

IntVectorDrvICInstance2:
    la    $26,  _IntHandlerDrvICInstance2         
    jr    $26
    nop
	.end	IntVectorDrvICInstance2

   .section	.DrvICInstance2_vector_text, code, keep
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  _IntHandlerDrvICInstance2

_IntHandlerDrvICInstance2:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvICInstance2
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	_IntHandlerDrvICInstance2


/*******************************************************************************
 End of File
 */

