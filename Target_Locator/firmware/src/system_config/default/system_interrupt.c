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
#include "uart_tx.h"
#include "uart_rx.h"
#include "transmit_handler.h"
#include "pic_interface.h"
#include "ping_sensors.h"
#include "grid_handler.h"
#include "json_parser.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

    
void IntHandlerDrvTmrInstance0(void)
{
    dbgOutputLoc (0xB0);
    int check = whichOC();
    if (check == 0) {
        dbgOutputLoc (0xBB);
        DRV_OC1_Stop ();
        DRV_OC0_Start ();
    }
    else if (check == 1) {
        dbgOutputLoc (0xBA);
        DRV_OC0_Stop ();
        DRV_OC2_Start ();
    }
    else if (check == 2) {
        DRV_OC2_Stop ();
        DRV_OC3_Start ();
    }
    else if (check == 3) {
        DRV_OC3_Stop ();
        DRV_OC1_Start ();
    }
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);
}
 void IntHandlerDrvUsartInstance0(void)
{
//     if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT)){
//         uint8_t byte;
//         dbgOutputLoc(0xF0);
//         while(!PLIB_USART_TransmitterBufferIsFull(USART_ID_1)){
////             if(xQueueReceiveFromISR(UART_TX_Byte_Q, &byte, 0)){
//             if(UARTReceiveByteFromTXQFromISR(&byte)){
//                 dbgOutputLoc(0xF1);
//                 PLIB_USART_TransmitterByteSend(USART_ID_1, byte);
//             }
//             else{
//                 dbgOutputLoc(0xF2);
//                 PLIB_INT_SourceDisable(USART_ID_1, INT_SOURCE_USART_1_TRANSMIT);
//                 break;
//             }
//         }
//         dbgOutputLoc(0xF3);
//     }
    DRV_USART_TasksTransmit(sysObj.drvUsart0);
//     PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
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
    dbgOutputLoc (0x40);
//    PLIB_PORTS_Write (PORTS_ID_0, PORT_CHANNEL_E, 0x00FF & 0x91);
    VALUES_t vals;
    vals.sensor = 0x00;
    vals.val1 = DRV_IC0_Capture16BitDataRead();
    vals.val2 = DRV_IC0_Capture16BitDataRead();
    QSendFromISR (vals);
    dbgOutputLoc (0x4C);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_INPUT_CAPTURE_3);
}
void IntHandlerDrvICInstance1(void)
{
    dbgOutputLoc(0xC0);
    VALUES_t vals;
    vals.sensor = 0x11;
    vals.val1 = DRV_IC1_Capture16BitDataRead();
    vals.val2 = DRV_IC1_Capture16BitDataRead();
    QSendFromISR (vals);
    dbgOutputLoc (0xC5);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_INPUT_CAPTURE_1);
}
void IntHandlerDrvICInstance2(void)
{
    VALUES_t vals;
    vals.sensor = 0x22;
    vals.val1 = DRV_IC2_Capture16BitDataRead();
    vals.val2 = DRV_IC2_Capture16BitDataRead();
    QSendFromISR (vals);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_INPUT_CAPTURE_4);
}
void IntHandlerDrvICInstance3(void)

{
    VALUES_t vals;
    vals.sensor = 0x33;
    vals.val1 = DRV_IC3_Capture16BitDataRead();
    vals.val2 = DRV_IC3_Capture16BitDataRead();
    QSendFromISR (vals);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_INPUT_CAPTURE_5);
}

 
/*******************************************************************************
 End of File
*/

