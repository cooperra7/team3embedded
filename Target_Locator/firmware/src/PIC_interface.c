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
static VERTEX_t vertexArray[20];

#include <math.h>
//robot status globals TODO MAKE NOT GLOBALS
uint8_t robot_x = 22;
uint8_t robot_y = 24;
int16_t robot_theta = 0;
bool robot_valid;

void getLocation(LOCATION_t *retval)
{
//    LOCATION_t retval;
    retval->theta = robot_theta;
    retval->x = robot_x;
    retval->y = robot_y;
    retval->valid = robot_valid;
    robot_valid = false;
}
void setLocation(LOCATION_t location)
{
    robot_theta = location.theta;
    robot_x = location.x;
    robot_y = location.y;
    robot_valid = true;
}

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
    dbgOutputLoc(DLOC_PIXY_ENTER_ISR);
    while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_2)){
        uint8_t temp = PLIB_USART_ReceiverByteReceive(USART_ID_2);
        dbgOutputVal(temp);
        processPixyByte(temp);
        //PixySendByteToQFromISR(temp);
    }
//    
    dbgOutputLoc(DLOC_PIXY_EXIT_ISR);
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
    if(byteCount == 0 && !(newByte == 0x55 || newByte == 0x56)){// || newByte == 0x00)){
        return;
    }
    if(byteCount == 1 && newByte == 0x00){
        PIXY_DATA_t ReceivedBlock;
        ReceivedBlock.ID = 0;
        PixySendByteToQFromISR(ReceivedBlock);
        byteCount = 0;
        return;
    }
    if(byteCount == 1 && newByte != 0xAA){
        byteCount = 0;
        return;
    }
    dataArray[byteCount] = newByte;
    byteCount++;
    if(byteCount == 4 && (dataArray[2] == 0x55 || dataArray[2] == 0x56) && dataArray[3] == 0xAA){
        PIXY_DATA_t ReceivedBlock;
        ReceivedBlock.ID = 0;
        PixySendByteToQFromISR(ReceivedBlock);
        byteCount = 2;
    }
    if(byteCount > 13){
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
bool stop = false;
void processBlocks(PROCESSED_PIXY_ITEM_t blockArray[], uint8_t numBlocks)
{
  int i, j;
  // if there's a target
  float tan_1, tan_2, m, new_x, new_y;
  float update_theta, update_x, update_y;
  float theta_1, theta_2, theta_1_rad, theta_2_rad;
  uint8_t theta_count, coord_count;
  theta_count = 0;
  coord_count = 0;
  update_theta = 0;
  update_x = 0;
  update_y = 0;
  uint8_t debug_x1, debug_y1, debug_x2, debug_y2;
  for(i = 0; i < numBlocks; i++){
    float delt_y = (Y1 - robot_y);
    float delt_x = (X1 - robot_x);
    float theta_robot = RAD_TO_DEG(atan(delt_y/delt_x)) - THETA1;
    update_theta += theta_robot;
    theta_count++;
    for(j = i+1; j < numBlocks; j++){
        if(blockArray[i].ID == blockArray[j].ID){
            continue;
        }
        theta_1 = THETA1 + robot_theta;
        theta_1_rad = DEG_TO_RAD(theta_1);
        theta_2 = THETA2 + robot_theta;
        theta_2_rad = DEG_TO_RAD(theta_2);
        debug_x1 = X1;
        debug_y1 = Y1;
        debug_x2 = X2;
        debug_y2 = Y2;
      tan_1 = tan(theta_1_rad);
      tan_2 = tan(theta_2_rad);
      float x_r = ((Y1 - Y2) - (X1*tan_1 - X2*tan_2))/(tan_2 - tan_1);
      float y_r1 = Y1 - (X1 - x_r)*tan_1;
      float y_r2 = Y2 - (X2 - x_r)*tan_2;
//      update_x += x_r;
//      update_y += (y_r1 + y_r2)/2;
      update_x = x_r;
      update_y = (y_r1 + y_r2)/2;
      coord_count++;
    }
  }
  uint8_t temp_robot_x, temp_robot_y;
  int16_t temp_robot_theta;
  if(theta_count > 0){
    update_theta /= theta_count;
    temp_robot_theta = ROUND(update_theta);
    robot_theta = temp_robot_theta;
    //TODO SEND LOCATION UPDATE
  }
  if(coord_count > 0){
    messageItem_t debug;
    //update_x /= coord_count;
    //update_y /= coord_count;
    temp_robot_x = ROUND(update_x);
    temp_robot_y = ROUND(update_y);
    robot_x = temp_robot_x;
    robot_y = temp_robot_y;
    debug.msgSize = sprintf(debug.payload, 
            "{\"DEBUG\" : 1 , \"%s\" : %u  , \"%s\" : %u  , \"%s\" : %i , \"%s\" : %f , \"%s\" : %f , \"%s\" : %f  , \"%s\" : %u  , \"%s\" : %u  , \"%s\" : %f , \"%s\" : %f , \"%s\" : %f  , \"%s\" : %u, \"%s\" : %u  }",
            "X_LOCATION", temp_robot_x,
            "Y_LOCATION", temp_robot_y,
            "ANGLE", temp_robot_theta,
            "THETA_1", theta_1,
            "THETA_1_RAD", theta_1_rad,
            "TAN_1", tan_1,
            "X_1", debug_x1,
            "Y_1", debug_y1,
            "THETA_2", theta_2,
            "THETA_2_RAD", theta_2_rad,
            "TAN_2", tan_2,
            "X_2", debug_x2,
            "Y_2", debug_y2
            );
    SendMessageForTransmitQ(debug);
    debug.msgSize = sprintf(debug.payload, "{\"RESPONSE\" : { \"SOURCE\" : \"%s \", \"DEST\" : \"%s\", \"ID\" : %i, \"TYPE\" : \"%s\", \"DATA\" : {\"X\" : %u, \"Y\" : %u, \"THETA\" : %i", TARGET_LOCATOR, SEARCHER_MOVEMENT, LOCATION_TYPE, 0, robot_x, robot_y, robot_theta);
    SendMessageForTransmitQ(debug);
    if(temp_robot_y == 0){
        stop = true;
    }
  }
  
}

void processFrame()
{
    processBlocks(currentProcessedFrame, currentProcessedCount);
//    messageItem_t debug;
//    debug.msgSize = sprintf(debug.payload,
//            "END OF PIXY FRAME\r\n\r\n\r\n\r\n"
//            );
//    SendMessageForTransmitQ(debug);
}

void processPixyItem(PIXY_DATA_t *data, PROCESSED_PIXY_ITEM_t *processed)
{
    processed->ID = PIXY_INVALID;
    processed->theta = 0;
    processed->x_center = 0;
    processed->y_center = 0;
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
    bool found = false;
    int i;
    for (i = 0; i < 20; i++){
        if(vertexArray[i].valid == false){
            continue;
        }
        if(data->ID == vertexArray[i].colorBar){
            processed->ID = vertexArray[i].ID;
            processed->x_center = vertexArray[i].x_center;
            processed->y_center = vertexArray[i].y_center;
            found = true;
            break;
        }
    }
//    else{
    if(!found){
        data->ID = PIXY_INVALID;
        processed->ID = PIXY_INVALID;
    }
    processed->theta = ((int16_t)160 - data->x_center - 2)/4 + 1;
    
}

void updateVertex(VERTEX_t newVertex)
{
    int i;
    for(i = 0; i < 20; i++){
        if(newVertex.ID == vertexArray[i].ID){
            vertexArray[i].valid = true;
            vertexArray[i].x_center = newVertex.x_center;
            vertexArray[i].y_center = newVertex.y_center;
            break;
        }
    }
}



void debugPIXY_MESSAGE(PIXY_DATA_t data)
{
    messageItem_t pixy_debug;
    pixy_debug.msgSize = sprintf(pixy_debug.payload, 
            //"{\"DEBUG\" : 1 , \"%s\" : %u  , \"%s\" : %u  , \"%s\" : %u  , \"%s\" : %u  , \"%s\" : %u }\r\n", 
            "{\"DEBUG\" : 1 , \"%s\" : %u  , \"%s\" : %u  , \"%s\" : %u  , \"%s\" : %u  , \"%s\" : %u }", 
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
            "{\"DEBUG\" : 1 , \"%s\" : %u  , \"%s\" : %i  , \"%s\" : %u  , \"%s\" : %u  }",
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
void vertexInit()
{
    vertexArray[0].ID = PIXY_VERTEX_1;
    vertexArray[0].colorBar = VERTEX_1_CODE;
    vertexArray[0].valid = false;
    
    vertexArray[2].ID = PIXY_VERTEX_2;
    vertexArray[2].colorBar = VERTEX_2_CODE;
    vertexArray[2].valid = false;
    
    vertexArray[3].ID = PIXY_VERTEX_3;
    vertexArray[3].colorBar = VERTEX_3_CODE;
    vertexArray[3].valid = false;
    
    vertexArray[4].ID = PIXY_VERTEX_4;
    vertexArray[4].colorBar = VERTEX_4_CODE;
    vertexArray[4].valid = false;
    
    vertexArray[5].ID = PIXY_VERTEX_5;
    vertexArray[5].colorBar = VERTEX_5_CODE;
    vertexArray[5].valid = false;
    
    vertexArray[6].ID = PIXY_VERTEX_6;
    vertexArray[6].colorBar = VERTEX_6_CODE;
    vertexArray[6].valid = false;
    
    vertexArray[7].ID = PIXY_VERTEX_7;
    vertexArray[7].colorBar = VERTEX_7_CODE;
    vertexArray[7].valid = false;
    
    vertexArray[8].ID = PIXY_VERTEX_8;
    vertexArray[8].colorBar = VERTEX_8_CODE;
    vertexArray[8].valid = false;
    
    vertexArray[9].ID = PIXY_VERTEX_9;
    vertexArray[9].colorBar = VERTEX_9_CODE;
    vertexArray[9].valid = false;
    
    vertexArray[10].ID = PIXY_VERTEX_10;
    vertexArray[10].colorBar = VERTEX_10_CODE;
    vertexArray[10].valid = false;
    
    vertexArray[11].ID = PIXY_VERTEX_11;
    vertexArray[11].colorBar = VERTEX_11_CODE;
    vertexArray[11].valid = false;
    
    vertexArray[12].ID = PIXY_VERTEX_12;
    vertexArray[12].colorBar = VERTEX_12_CODE;
    vertexArray[12].valid = false;
    
    vertexArray[13].ID = PIXY_VERTEX_13;
    vertexArray[13].colorBar = VERTEX_13_CODE;
    vertexArray[13].valid = false;
    
    vertexArray[14].ID = PIXY_VERTEX_14;
    vertexArray[14].colorBar = VERTEX_14_CODE;
    vertexArray[14].valid = false;
    
    vertexArray[15].ID = PIXY_VERTEX_15;
    vertexArray[15].colorBar = VERTEX_15_CODE;
    vertexArray[15].valid = false;
    
    vertexArray[16].ID = PIXY_VERTEX_16;
    vertexArray[16].colorBar = VERTEX_16_CODE;
    vertexArray[16].valid = false;
    
    vertexArray[17].ID = PIXY_VERTEX_17;
    vertexArray[17].colorBar = VERTEX_17_CODE;
    vertexArray[17].valid = false;
    
    vertexArray[18].ID = PIXY_VERTEX_18;
    vertexArray[18].colorBar = VERTEX_18_CODE;
    vertexArray[18].valid = false;
    
    vertexArray[19].ID = PIXY_VERTEX_19;
    vertexArray[19].colorBar = VERTEX_19_CODE;
    vertexArray[19].valid = false;
    
    vertexArray[20].ID = PIXY_VERTEX_20;
    vertexArray[20].colorBar = VERTEX_20_CODE;
    vertexArray[20].valid = false;
}
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
        errorHandler(0xFF);
    }
    
    vertexInit();
    
    //DRV_USART_ByteTransmitCallbackSet(DRV_USART_INDEX_0, APP_USARTTransmitEventHandler);
    DRV_USART_ByteReceiveCallbackSet(DRV_USART_INDEX_1, Pixy_ByteReceive);
    
    PixyByteHandle = xQueueCreate(16, sizeof(PIXY_DATA_t));
    //enableDbgVal();
    dbgOutputVal(0xAA);
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
            PIXY_DATA_t data;
            dbgOutputLoc(DLOC_PIXY_WAIT_FOR_QUEUE);
            PixyReceiveByteFromRXQ(&data);
            PROCESSED_PIXY_ITEM_t processed_item;
            processPixyItem(&data, &processed_item);
            dbgOutputLoc(DLOC_PIXY_GOT_FROM_QUEUE);
            
            if (data.ID == 0){
                //new Frame
                processFrame();
                currentBlockCount = 0;
                currentProcessedCount = 0;
//                debugPIXY_PROCESSED(processed_item);
                break;
            }
            else if (processed_item.ID != PIXY_INVALID){
//                debugPIXY_MESSAGE(data);
                debugPIXY_PROCESSED(processed_item);
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
