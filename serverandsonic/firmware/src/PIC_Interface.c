/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    pic_interface.c

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

#include "pic_interface.h"
#include "uart_public.h"
#include "grabber_p.h"

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

PIC_INTERFACE_DATA pic_interfaceData;

DRV_HANDLE Pixy;
QueueHandle_t PixyByteHandle;//queue handle
static PIXY_DATA_t currentFrame[10];//current frame of data
static PROCESSED_PIXY_ITEM_t currentProcessedFrame[10];
static uint8_t currentProcessedCount;
static uint8_t currentBlockCount;//count of blocks in current frame
static uint8_t byteCount = 0;//byte count for current pixy message
static uint8_t dataArray[16];//current array of bytes for pixy data

#include <math.h>
//robot status globals TODO MAKE NOT GLOBALS
uint8_t robot_x = 0;
uint8_t robot_y = 0;
int16_t robot_theta = 0;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/
void PixySendByteToQFromISR(PIXY_DATA_t byte);
void processPixyByte(uint8_t newByte);
/* TODO:  Add any necessary local functions.
*/

void Pixy_ByteReceive(const SYS_MODULE_INDEX index)
{
//    dbgOutputLoc (0x67);
    while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_2)){
        uint8_t temp = PLIB_USART_ReceiverByteReceive(USART_ID_2);
        processPixyByte(temp);
        //PixySendByteToQFromISR(temp);
    }
}
// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


int PixySendByteToQ(PIXY_DATA_t byte)
{
    if (xQueueSend(PixyByteHandle, &byte, portMAX_DELAY) == pdTRUE)
        return 1;
    return 0;
}
void PixySendByteToQFromISR(PIXY_DATA_t byte)
{
    BaseType_t xHigherPriorityTaskWoken, xResult;
    xHigherPriorityTaskWoken = pdFALSE;
    xResult = xQueueSendFromISR(PixyByteHandle, &byte, &xHigherPriorityTaskWoken);
    /* Actual macro used here is port specific. */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
int PixyReceiveByteFromRXQ(PIXY_DATA_t *byte)
{
    if (xQueueReceive(PixyByteHandle, byte, portMAX_DELAY) == pdTRUE)
       return 1;
   return 0;
}

void processPixyMessage(uint8_t array[], uint8_t byteCount)
{
    PIXY_DATA_t ReceivedBlock;
    ReceivedBlock.ID = *((uint16_t*)&array[4]);
    ReceivedBlock.x_center = *((uint16_t*)&array[6]);
    ReceivedBlock.y_center = *((uint16_t*)&array[8]);
    ReceivedBlock.width = *((uint16_t*)&array[10]);
    ReceivedBlock.height = *((uint16_t*)&array[12]);
    PixySendByteToQFromISR(ReceivedBlock);
    //send to queue
}


void processPixyByte(uint8_t newByte)
{
    dbgOutputLoc (newByte);
    if(byteCount == 0 && !(newByte == 0x55 || newByte == 0x56)){// || newByte == 0x00)){
        return;
    }
//    if(byteCount == 1 && newByte == 0x00){
//        PIXY_DATA_t ReceivedBlock;
//        ReceivedBlock.ID = 0;
//        PixySendByteToQFromISR(ReceivedBlock);
//        byteCount = 0;
//        return;
//    }
    if(byteCount == 1 && newByte != 0xAA){
        byteCount = 0;
        return;
    }
    dataArray[byteCount] = newByte;
    byteCount++;
    if(byteCount == 4 && (dataArray[2] == 0x55 || dataArray[2] == 0x56) && dataArray[3] == 0xAA){
        PIXY_DATA_t ReceivedBlock;
        ReceivedBlock.ID = 0;
//        dbgOutputLoc (0x65);
        PixySendByteToQFromISR(ReceivedBlock);
        byteCount = 2;
    }
    if(byteCount > 13){
//        dbgOutputLoc (0x64);
        processPixyMessage(dataArray, byteCount);
        byteCount = 0;
    }
}

#define X1 ((float)blockArray[i].x_center)
#define X2 ((float)blockArray[j].x_center)
#define Y1 ((float)blockArray[i].y_center)
#define Y2 ((float)blockArray[j].y_center)
#define THETA1 ((float)blockArray[i].theta)
#define THETA2 ((float)blockArray[j].theta)
#define RAD_TO_DEG(x) ((180.0/M_PI)*x)
#define DEG_TO_RAD(x) ((M_PI/180.0)*x)

void processBlocks(PROCESSED_PIXY_ITEM_t blockArray[], uint8_t numBlocks)
{
  int i, j;
  // if there's a target
  float tan_1, tan_2, m, new_x, new_y;
  float update_theta, update_x, update_y;
  uint8_t theta_count, coord_count;
  theta_count = 0;
  coord_count = 0;
  update_theta = 0;
  update_x = 0;
  update_y = 0;
  for(i = 0; i < numBlocks; i++){
    float delt_y = (Y1 - robot_y);
    float delt_x = (X1 - robot_x);
    float theta_robot = RAD_TO_DEG(atan(delt_y/delt_x)) - THETA1;
    update_theta += theta_robot;
    theta_count++;
    messageItem_t debug;
    debug.msgSize = sprintf(debug.payload, "\r\n\r\n\r\nDelta_y: %f\r\nDelta_x: %f\r\n Theta: %f\r\nTheta_Robot: %f\r\n", delt_y, delt_x, THETA1, (float)theta_robot);
//    SendMessageForTransmitQ(debug);
    for(j = i+1; j < numBlocks; j++){
      // uint8_t *X1 = &blockArray[i].x;
      // uint8_t *X2 = &blockArray[j].x;
      // uint8_t *Y1 = &blockArray[i].y;
      // uint8_t *Y2 = &blockArray[j].y;
      // int8_t *THETA1 = &blockArray[i].theta;
      // int8_t *THETA2 = &blockArray[j].theta;
      //nMinm = (blockArray[i].x - blockArray[j].x);
      debug.msgSize = sprintf(debug.payload, "Y1: %f\r\nY2: %f\r\nX1: %f\r\nX2: %f\r\nTHETA1: %f\r\nTHETA2: %f\r\n", Y1, Y2, X1, X2, THETA1, THETA2);
    //SendMessageForTransmitQ(debug);
      tan_1 = tan(DEG_TO_RAD(THETA1 + theta_robot));
      tan_2 = tan(DEG_TO_RAD(THETA2 + theta_robot));
      //m = ((Y1-Y2)/tan_1 - (X1-X2))/(1-tan_2/tan_1);
      m = ((Y1-Y2)-(X1-X2)*tan_1)/(tan_1-tan_2);
      debug.msgSize = sprintf(debug.payload, "tan_1: %f\r\ntan_2: %f\r\nm: %f\r\n", tan_1, tan_2, m);
    //SendMessageForTransmitQ(debug);
      new_x = X2 - m;
      new_y = Y2 - m*tan_2;
      debug.msgSize = sprintf(debug.payload, "Using new angle\r\nTHETA: %f\r\nX, Y: %f, %f\r\n", theta_robot, new_x, new_y);
    //SendMessageForTransmitQ(debug);
      tan_1 = tan(DEG_TO_RAD(THETA1 + robot_theta));
      tan_2 = tan(DEG_TO_RAD(THETA2 + robot_theta));
      //m = ((Y1-Y2)/tan_1 - (X1-X2))/(1-tan_2/tan_1);
      m = ((Y1-Y2)-(X1-X2)*tan_1)/(tan_1-tan_2);
      //m = ((*Y1-*Y2)/tan_1 - (*X1-*X2))/(1-tan_2/tan_1);
      debug.msgSize = sprintf(debug.payload, "tan_1: %f\ntan_2: %f\r\nm: %f\r\n", tan_1, tan_2, m);
    //SendMessageForTransmitQ(debug);
      new_x = X2 - m;
      new_y = Y2 - m*tan_2;
      debug.msgSize = sprintf(debug.payload, "Using old angle\nTHETA: %f\nX, Y: %f, %f\r\n", robot_theta, new_x, new_y);
    //SendMessageForTransmitQ(debug);
      
      float x_r = ((Y2 - Y1) - (X2*tan_2 - X1*tan_1))/(tan_1 - tan_2);
      float y_r1 = Y1 - (X1 - x_r)*tan_1;
      float y_r2 = Y2 - (X2 - x_r)*tan_2;
      update_x += x_r;
      update_y += (y_r1 + y_r2)/2;
      coord_count++;
      debug.msgSize = sprintf(debug.payload, "X_ROBOT: %f\r\nY_1: %f\r\nY_2: %f\r\n", x_r, y_r1, y_r2);
//    SendMessageForTransmitQ(debug);
      
      //TODO update rover location
    }
  }
  update_theta /= theta_count;
  update_x /= coord_count;
  update_y /= coord_count;
  robot_x = ROUND(update_x);
  robot_y = ROUND(update_y);
  robot_theta = ROUND(update_theta);
}

void processFrame()
{
    processBlocks(currentProcessedFrame, currentProcessedCount);
    messageItem_t debug;
    debug.msgSize = sprintf(debug.payload,
            "END OF PIXY FRAME\r\n\r\n\r\n\r\n"
            );
//    SendMessageForTransmitQ(debug);
}

void processPixyItem(PIXY_DATA_t *data, PROCESSED_PIXY_ITEM_t *processed)
{
    if(data->width < MIN_WIDTH){
        data->ID = PIXY_INVALID;
        processed->ID = PIXY_INVALID;
        return;
    }
    if(data->height < MIN_HEIGHT){
        data->ID = PIXY_INVALID;
        processed->ID = PIXY_INVALID;
        return;
    }
    if(data->ID == TARGET_CODE){
        data->ID = PIXY_TARGET;
        processed->ID = PIXY_TARGET;
    }
    else if (data->ID == VERTEX_1_CODE){
        data->ID = PIXY_VERTEX_1;
        processed->ID = PIXY_VERTEX_1;
        processed->x_center = 70;
        processed->y_center = 24;
    }
    else if (data->ID == VERTEX_2_CODE){
        data->ID = PIXY_VERTEX_2;
        processed->ID = PIXY_VERTEX_2;
        processed->x_center = 118;
        processed->y_center = 0;
    }
/*    else{
        data->ID = PIXY_INVALID;
        processed->ID = PIXY_INVALID;
    }*/
    processed->theta = ((int16_t)160 - data->x_center - 2)/4 + 1;
    
}





void debugPIXY_MESSAGE(PIXY_DATA_t data)
{
    dbgOutputLoc (0x6A);
    dbgOutputLoc (0xA6);
    dbgOutputLoc (0x6A);
    messageItem_t pixy_debug;
    pixy_debug.msgSize = sprintf(pixy_debug.payload, 
            "{\"DEBUG\" : { \"PIXY\" : { \"%s\" : %u  , \"%s\" : %u  , \"%s\" : %u  , \"%s\" : %u  , \"%s\" : %u } } }\n", 
            "ID", data.ID, 
            "xCoord", data.x_center, 
            "yCoord", data.y_center, 
            "Width", data.width,
            "Height", data.height);
    SendMessageForTransmitQ(pixy_debug);
}

void debugPIXY_PROCESSED(PROCESSED_PIXY_ITEM_t data)
{
    messageItem_t debug;
    debug.msgSize = sprintf(debug.payload,
            "{\"DEBUG\" : { \"PIXYANG\" : { \"%s\" : %u  , \"%s\" : %u  , \"%s\" : %u  , \"%s\" : %u  } } }\n",
            "ID", data.ID,
            "ANGLE", data.theta,
            "XCoord", data.x_center,
            "YCoord", data.y_center
            );
    SendMessageForTransmitQ(debug);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void PIC_INTERFACE_Initialize ( void )

  Remarks:
    See prototype in pic_interface.h.
 */

void PIC_INTERFACE_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    pic_interfaceData.state = PIC_INTERFACE_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    Pixy = DRV_USART_Open(DRV_USART_INDEX_1, DRV_IO_INTENT_READ);
    if (Pixy == DRV_HANDLE_INVALID){
        error(0xFF);
    }
    //DRV_USART_ByteTransmitCallbackSet(DRV_USART_INDEX_0, APP_USARTTransmitEventHandler);
    DRV_USART_ByteReceiveCallbackSet(DRV_USART_INDEX_1, Pixy_ByteReceive);
    
    PixyByteHandle = xQueueCreate(16, sizeof(PIXY_DATA_t));
}


/******************************************************************************
  Function:
    void PIC_INTERFACE_Tasks ( void )

  Remarks:
    See prototype in pic_interface.h.
 */

void PIC_INTERFACE_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( pic_interfaceData.state )
    {
        /* Application's initial state. */
        case PIC_INTERFACE_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                pic_interfaceData.state = PIC_INTERFACE_STATE_SERVICE_TASKS;
            }
            break;
        }

        case PIC_INTERFACE_STATE_SERVICE_TASKS:
        {
            while (1) {
            PIXY_DATA_t data;
//            dbgOutputLoc (0x6D);
            PixyReceiveByteFromRXQ(&data);
            PROCESSED_PIXY_ITEM_t processed_item;
            processPixyItem(&data, &processed_item);
            if (data.ID == 0){
                //new Frame
                processFrame();
                currentBlockCount = 0;
                currentProcessedCount = 0;
                break;
            }
            else{
                dbgOutputLoc (0xFA);
                debugPIXY_MESSAGE(data);
                QSendPixy (data);
               // debugPIXY_PROCESSED(processed_item);
                if(data.ID != PIXY_TARGET){
                    currentProcessedFrame[currentProcessedCount].ID = processed_item.ID;
                    currentProcessedFrame[currentProcessedCount].x_center = processed_item.x_center;
                    currentProcessedFrame[currentProcessedCount].y_center = processed_item.y_center;
                    currentProcessedFrame[currentProcessedCount].theta = processed_item.theta;
                    currentProcessedCount++;
                }
            }
            currentFrame[currentBlockCount].ID = data.ID;
            currentFrame[currentBlockCount].x_center = data.x_center;
            currentFrame[currentBlockCount].y_center = data.y_center;
            currentFrame[currentBlockCount].width = data.width;
            currentFrame[currentBlockCount].height = data.height;
            currentBlockCount++;
            }
            break;
        }

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
