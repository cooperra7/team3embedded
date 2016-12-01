/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    uart_rx.c

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

#include "uart_rx.h"

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

UART_RX_DATA uart_rxData;
static DRV_HANDLE UART_RX_Comms;
QueueHandle_t UART_RX_Byte_Q;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/
void APP_USARTReceiveEventHandler(const SYS_MODULE_INDEX index)
{
    dbgOutputLoc(DLOC_UART_RX_ISR_ENTER);
    while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)){
        uint8_t temp = PLIB_USART_ReceiverByteReceive(USART_ID_1);
        UARTSendByteTeRXQFromISR(temp);
        //processByte(temp);
    }
    
    dbgOutputLoc(DLOC_UART_RX_ISR_EXIT);
}
// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/
int UARTSendByteToRXQ(uint8_t byte)
{
    if (xQueueSend(UART_RX_Byte_Q, &byte, portMAX_DELAY) == pdTRUE)
        return 1;
    return 0;
}
void UARTSendByteTeRXQFromISR(uint8_t byte)
{
    BaseType_t xHigherPriorityTaskWoken, xResult;
    xHigherPriorityTaskWoken = pdFALSE;
    xResult = xQueueSendFromISR(UART_RX_Byte_Q, &byte, &xHigherPriorityTaskWoken);
    /* Actual macro used here is port specific. */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
int UARTReceiveByteFromRXQ(uint8_t *byte)
{
    if (xQueueReceive(UART_RX_Byte_Q, byte, portMAX_DELAY) == pdTRUE)
       return 1;
   return 0;
}

static uint16_t byteCount = 0;
static uint16_t checkSum;
static uint16_t calcCheckSum;
static uint16_t messageNumber;
static uint16_t messageSize;
static uint8_t dest;
static uint8_t source;
static uint8_t message[MAX_WIFLY_SIZE];

int checkStart(uint8_t byte)
{
    if (byteCount == 0 && byte != ((MSG_START & 0xFF000000) >> 24))
    {
        return 0;
    }
    if (byteCount == 1 && byte != ((MSG_START & 0x00FF0000) >> 16)){
        byteCount = 0;
        return 0;
    }
    if (byteCount == 2 && byte != ((MSG_START & 0x0000FF00) >> 8)){
        byteCount = 0;
        return 0;
    }
    if (byteCount == 3 && byte != ((MSG_START & 0x000000FF) >> 0)){
        byteCount = 0;
        return 0;
    }
    return 1;
}

int parseHeader()
{
    messageNumber = ((uint16_t)message[5] << 8) + (uint16_t)message[4];
    dest = message[6];
    source = message[7];
    messageSize = ((uint16_t)message[9] << 8) + (uint16_t)message[8];
    checkSum = ((uint16_t)message[11] << 8) + (uint16_t)message[10];
    calcCheckSum = 0;
    if (dest != DEVICEID){
        return 0;//TODO
    }
    if (source == DEVICEID){
        return 0;//TODO
    }
    return 1;
}

void parseMessage()
{
    if(calcCheckSum != checkSum){
        //Potentially send an error message to request retry
        byteCount = 0;
        return;
    }
    messageItem_t receivedMsg;
    int i;
    for(i = 0; i < messageSize; i++)
    {
        receivedMsg.payload[i] = message[i + MSG_HEADER_SIZE];
    }
    receivedMsg.msgSize = messageSize;
    //SendMessageForTransmitQ(receivedMsg);
    //send to queue
    JSONParserNewPacket(receivedMsg);
    byteCount = 0;
}

void processByte(uint8_t byte)
{
    if (!checkStart(byte)){
        return;
    }
    message[byteCount] = byte;
    byteCount++;
    calcCheckSum += byte;
    if(byteCount == MSG_HEADER_SIZE){
        if (!parseHeader()){
            byteCount = 0;
        }
        return;
    }
    if(byteCount == MSG_HEADER_SIZE + messageSize){
        parseMessage();
    }
    if (byteCount == MAX_WIFLY_SIZE){
        byteCount = 0;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void UART_RX_Initialize ( void )

  Remarks:
    See prototype in uart_rx.h.
 */

void UART_RX_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    uart_rxData.state = UART_RX_STATE_INIT;
    enable_dbgOutputLoc();

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    UART_RX_Comms = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_READ);
    if (UART_RX_Comms == DRV_HANDLE_INVALID){
        errorHandler(0xFF);
    }
    //DRV_USART_ByteTransmitCallbackSet(DRV_USART_INDEX_0, APP_USARTTransmitEventHandler);
    DRV_USART_ByteReceiveCallbackSet(DRV_USART_INDEX_0, APP_USARTReceiveEventHandler);
    
    UART_RX_Byte_Q = xQueueCreate(32, sizeof(uint8_t));
}


/******************************************************************************
  Function:
    void UART_RX_Tasks ( void )

  Remarks:
    See prototype in uart_rx.h.
 */

void UART_RX_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( uart_rxData.state )
    {
        /* Application's initial state. */
        case UART_RX_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                uart_rxData.state = UART_RX_STATE_SERVICE_TASKS;
            }
            break;
        }

        case UART_RX_STATE_SERVICE_TASKS:
        {
            uint8_t byte;
            dbgOutputLoc(DLOC_UART_RX_WAIT_FOR_QUEUE);
            while(!UARTReceiveByteFromRXQ(&byte));
            dbgOutputLoc(DLOC_UART_RX_GOT_FROM_QUEUE);
            //UARTSendByteToTXQ(byte);
            processByte(byte);
            dbgOutputLoc(DLOC_UART_RX_SENT_TO_QUEUE);
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
