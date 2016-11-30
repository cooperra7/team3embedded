/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    uart_tx.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "uart_tx.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

UART_TX_DATA uart_txData;
QueueHandle_t UART_TX_Byte_Q;
static DRV_HANDLE UART_TX_Comms;
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/
static uint8_t msgCounter;
void APP_USARTTransmitEventHandler(const SYS_MODULE_INDEX index)
{
    //dbgOutputLoc(0xFF);
    msgCounter++;
    if(msgCounter > 6) {
        msgCounter = 0;
    }
    else{
        //DRV_USART_WriteByte(UART_TX_Comms, Name[msgCounter]);
        //USART_TransmitterByteSend_Default(UART_TX_Comms, Name[msgCounter]);
    }
    //DRV_USART_TasksTransmit(index);
    
}
// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/
int UARTSendByteToTXQ(uint8_t byte)
{
    if (xQueueSend(UART_TX_Byte_Q, &byte, portMAX_DELAY) == pdTRUE)
        return 1;
    return 0;
}
void UARTSendByteToTXQFromISR(uint8_t byte)
{
    BaseType_t xHigherPriorityTaskWoken, xResult;
    xHigherPriorityTaskWoken = pdFALSE;
    xResult = xQueueSendFromISR(UART_TX_Byte_Q, &byte, &xHigherPriorityTaskWoken);
    /* Actual macro used here is port specific. */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
int UARTReceiveByteFromTXQ(uint8_t *byte)
{
    if (xQueueReceive(UART_TX_Byte_Q, byte, portMAX_DELAY) == pdTRUE)
       return 1;
   return 0;
}

void UARTReceiveByteFromTXQFromISR(uint8_t *byte)
{
    BaseType_t xHigherPriorityTaskWoken, xResult;
    xHigherPriorityTaskWoken = pdFALSE;
    xResult = xQueueReceiveFromISR(UART_TX_Byte_Q, byte, &xHigherPriorityTaskWoken);
    /* Actual macro used here is port specific. */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void UART_TX_Initialize ( void )

  Remarks:
    See prototype in uart_tx.h.
 */

void UART_TX_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    uart_txData.state = UART_TX_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
    UART_TX_Comms = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_WRITE);
    if (UART_TX_Comms == DRV_HANDLE_INVALID){
    }
    DRV_USART_ByteTransmitCallbackSet(DRV_USART_INDEX_0, APP_USARTTransmitEventHandler);
    UART_TX_Byte_Q = xQueueCreate(MAX_WIFLY_SIZE, sizeof(uint8_t));
}


/******************************************************************************
  Function:
    void UART_TX_Tasks ( void )

  Remarks:
    See prototype in uart_tx.h.
 */

void UART_TX_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( uart_txData.state )
    {
        /* Application's initial state. */
        case UART_TX_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                uart_txData.state = UART_TX_STATE_SERVICE_TASKS;
            }
            break;
        }

        case UART_TX_STATE_SERVICE_TASKS:
        {
            uint8_t byte;
            //dbgOutputLoc(DLOC_UART_TX_WAIT_FOR_QUEUE);
            while(!UARTReceiveByteFromTXQ(&byte));
            //dbgOutputLoc(DLOC_UART_TX_GOT_FROM_QUEUE);
            while(PLIB_USART_TransmitterBufferIsFull(USART_ID_1));
//            dbgOutputLoc (byte);
            DRV_USART_WriteByte(UART_TX_Comms, byte);
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
