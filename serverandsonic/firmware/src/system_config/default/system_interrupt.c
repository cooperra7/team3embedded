/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
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

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <xc.h>
#include <sys/attribs.h>
#include "grabber.h"
#include "grabber_p.h"
#include "uart_tx.h"
#include "uart_rx.h"
#include "transmit_handler.h"
#include "jsonencoder.h"
#include "jsondecoder.h"
#include "data.h"
#include "pic_interface.h"
#include "system_definitions.h"
#include "debug.h"
#include "peripheral/oc/plib_oc.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
    
void IntHandlerDrvTmrInstance0(void)
{
    int check = whichOC();
    if (check == 0) {
        DRV_OC1_Stop ();
        DRV_OC0_Start ();
    }
    else if (check == 1) {
        DRV_OC0_Stop ();
        DRV_OC2_Start ();
    }
    else if (check == 2) {
        DRV_OC2_Stop ();
        DRV_OC1_Start ();
    }
/*    char buffer[10];
    memset (buffer, 0, 10);
    strcpy (buffer, "\nhello\n\n");
    messageItem_t message;
    message.msgSize = strlen (buffer);
    int i = 0;
    for (i = 0; i < message.msgSize; i++) {
        message.payload[i] = buffer[i];
    }
    SendMessageForTransmitQ (message);*/
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);
}
 void IntHandlerDrvUsartInstance0(void)
{
    DRV_USART_TasksTransmit(sysObj.drvUsart0);
    DRV_USART_TasksReceive(sysObj.drvUsart0);
    DRV_USART_TasksError(sysObj.drvUsart0);
}
 
 
 

void IntHandlerDrvUsartInstance1(void)
{
    DRV_USART_TasksTransmit(sysObj.drvUsart1);
    DRV_USART_TasksReceive(sysObj.drvUsart1);
    DRV_USART_TasksError(sysObj.drvUsart1);
}
 
 
 

 

 

 

 
 
void IntHandlerDrvICInstance0(void)
{
    VALUES_t vals;
    vals.sensor = 0x22;
    vals.val1 = DRV_IC0_Capture16BitDataRead();
    vals.val2 = DRV_IC0_Capture16BitDataRead();
    QSendFromISR (vals);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_INPUT_CAPTURE_3);
}
void IntHandlerDrvICInstance1(void)
{
    VALUES_t vals;
    vals.sensor = 0x00;
    vals.val1 = DRV_IC1_Capture16BitDataRead();
    vals.val2 = DRV_IC1_Capture16BitDataRead();
    QSendFromISR (vals);
    
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_INPUT_CAPTURE_1);
}
void IntHandlerDrvICInstance2(void)
{
    VALUES_t vals;
    vals.sensor = 0x11;
    vals.val1 = DRV_IC2_Capture16BitDataRead();
    vals.val2 = DRV_IC2_Capture16BitDataRead();
    QSendFromISR (vals);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_INPUT_CAPTURE_4);
}

 
/*******************************************************************************
 End of File
*/

