/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    transmit_handler.c

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

#include "transmit_handler.h"

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

TRANSMIT_HANDLER_DATA transmit_handlerData;
static QueueHandle_t TransmitMessage_Q;
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/
int SendMessageForTransmitQ(messageItem_t message)
{
    if (xQueueSend(TransmitMessage_Q, &message, portMAX_DELAY) == pdTRUE)
        return 1;
    return 0;
}
void SendMessageForTransmitFromISR(messageItem_t message)
{
    BaseType_t xHigherPriorityTaskWoken, xResult;
    xHigherPriorityTaskWoken = pdFALSE;
    xResult = xQueueSendFromISR(TransmitMessage_Q, &message, &xHigherPriorityTaskWoken);
    /* Actual macro used here is port specific. */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
int ReceiveMessageForTransmitQ(messageItem_t *message)
{
    if (xQueueReceive(TransmitMessage_Q, message, portMAX_DELAY) == pdTRUE)
       return 1;
   return 0;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void TRANSMIT_HANDLER_Initialize ( void )

  Remarks:
    See prototype in transmit_handler.h.
 */

void TRANSMIT_HANDLER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    transmit_handlerData.state = TRANSMIT_HANDLER_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    TransmitMessage_Q = xQueueCreate(4, sizeof(messageItem_t));//sizeof(uint8_t)*MAX_PAYLOAD_SIZE);
}


/******************************************************************************
  Function:
    void TRANSMIT_HANDLER_Tasks ( void )

  Remarks:
    See prototype in transmit_handler.h.
 */
static uint16_t messageCount = 0;
void TRANSMIT_HANDLER_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( transmit_handlerData.state )
    {
        /* Application's initial state. */
        case TRANSMIT_HANDLER_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                transmit_handlerData.state = TRANSMIT_HANDLER_STATE_SERVICE_TASKS;
            }
            break;
        }

        case TRANSMIT_HANDLER_STATE_SERVICE_TASKS:
        {
            uint8_t message[MAX_WIFLY_SIZE];
            messageItem_t msgToSend;
            //uint8_t payload[MAX_PAYLOAD_SIZE];
            ReceiveMessageForTransmitQ(&msgToSend);
            //create message and put header information in with payload
            //4 byte start, 2 byte count, 1 byte size, 2 byte checksum(sum of all payload bytes))
            message[0] = ((MSG_START & 0xFF000000) >> 24);
            message[1] = ((MSG_START & 0x00FF0000) >> 16);
            message[2] = ((MSG_START & 0x0000FF00) >> 8);
            message[3] = ((MSG_START & 0x000000FF) >> 0);
            message[4] = messageCount & 0x00FF;
            message[5] = (messageCount & 0xFF00) >> 8;
            message[6] = 1;//dest
            message[7] = 3;//source//DEVICEID
            message[8] = (msgToSend.msgSize & 0xFF);
            message[9] = (msgToSend.msgSize & 0xFF00) >> 8;
            uint16_t sum = 0;
            messageCount++;
            int i;
            for(i = MSG_HEADER_SIZE; i < MAX_WIFLY_SIZE; i++)
            {
                message[i] = msgToSend.payload[i-MSG_HEADER_SIZE];
                sum+= message[i];
                if(i >= (msgToSend.msgSize + MSG_HEADER_SIZE - 1)){
                    break;
                }
            }
            message[10] = (sum & 0x00FF);
            message[11] = (sum & 0xFF00) >> 8;
            for(i = 0; i < msgToSend.msgSize + MSG_HEADER_SIZE; i++){
//            for(i = 0; i < MAX_WIFLY_SIZE; i++){
                UARTSendByteToTXQ(message[i]);
//                PLIB_INT_SourceEnable(USART_ID_1, INT_SOURCE_USART_1_TRANSMIT);
            }
//            PLIB_INT_SourceEnable(USART_ID_1, INT_SOURCE_USART_1_TRANSMIT);
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
