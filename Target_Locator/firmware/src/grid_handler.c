/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    grid_handler.c

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

#include "grid_handler.h"
#include "world_public.h"

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

GRID_HANDLER_DATA grid_handlerData;
static uint8_t world[MAPDIM][MAPDIM];

void classRequest()
{
    classifyObjects(world, 0);
}

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
QueueHandle_t GRID_Q;


void GRID_Qinit ()
{
    GRID_Q = xQueueCreate (5, sizeof (PING_DATA_t));
}

void GRID_QSend(PING_DATA_t byte)
{
    xQueueSend(GRID_Q, &byte, portMAX_DELAY);
}

void GRID_QSendFromISR (PING_DATA_t tmrvals)
{
    /* Check for whether the task needs to do a context switch */
    BaseType_t conSwitch;
    conSwitch = pdFALSE;
    
    /* Send to the Queue from an ISR */
    if ((xQueueOverwriteFromISR (GRID_Q, &tmrvals, &conSwitch)) == errQUEUE_FULL) {
        PLIB_PORTS_Write (PORTS_ID_0, PORT_CHANNEL_E, 0x00FA);
//        xQueueReset(Q);
    }
    
    /* Switch if needed */
    portEND_SWITCHING_ISR(conSwitch);
}

void GRID_QReceive (PING_DATA_t *ret)
{
    
    xQueueReceive (GRID_Q, ret, portMAX_DELAY);
    
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void GRID_HANDLER_Initialize ( void )

  Remarks:
    See prototype in grid_handler.h.
 */

void GRID_HANDLER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    grid_handlerData.state = GRID_HANDLER_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    GRID_Qinit();
    int i, j;
    for(i = 0; i < MAPDIM; i++){
        for(j = 0; j < MAPDIM; j++){
            world[i][j] = INITIAL_VAL;
        }
    }
}


/******************************************************************************
  Function:
    void GRID_HANDLER_Tasks ( void )

  Remarks:
    See prototype in grid_handler.h.
 */
uint8_t msgCount;

void GRID_HANDLER_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( grid_handlerData.state )
    {
        /* Application's initial state. */
        case GRID_HANDLER_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                grid_handlerData.state = GRID_HANDLER_STATE_SERVICE_TASKS;
            }
            break;
        }

        case GRID_HANDLER_STATE_SERVICE_TASKS:
        {
            PING_DATA_t data;
            GRID_QReceive(&data);
            uint8_t grid[18][18];
            int x, y;
            for(x = 0; x < 18; x++){
                for(y = 0; y < 18; y++){
                    grid[x][y] = 0;
                    if(y > 2*(abs(6-x))){//left sensor
                        if(y > data.left){
                            grid[x][y] += THING;
                        }
                        else{
                            grid[x][y] += EMPTY;
                        }
                    }
                    if(y > 2*(abs(12-x))){//right sensor
                        if(y > data.right){
                            grid[x][y] += THING;
                        }
                        else{
                            grid[x][y] += EMPTY;
                        }
                    }
                }
            }
            uint8_t relGrid[GRID_SIZE][GRID_SIZE];
            for (x = 0; x < GRID_SIZE; x++){
                for (y = 0; y < GRID_SIZE; y++){
                    relGrid[x][y] = 0;
                }
            }
            uint8_t half_offset = (GRID_SIZE - 18)/2;
            for(x = half_offset; x < (GRID_SIZE - half_offset); x++){
                for(y = GRID_SIZE - 18; y < GRID_SIZE; y++){
                    relGrid[x][y] = grid[x - half_offset][y - (GRID_SIZE - 18)];
                }
            }
            GRID_ITEM_t absGridData[GRID_SIZE][GRID_SIZE];
            LOCATION_t currentLoc;
            getLocation(&currentLoc);
//            if(!currentLoc.valid){
//                break;
//            }
//            transposeRelativeToAbsolute(relGrid, GRID_SIZE, robot_x, robot_y, robot_theta, absGridData);
            transposeRelativeToAbsolute(relGrid, GRID_SIZE, currentLoc.x, currentLoc.y, currentLoc.theta, absGridData);
            updateMyWorld(absGridData, world);
//            classifyObjects(world, msgCount);
            msgCount++;
//            messageItem_t debug;
//            for(y = 17; y > 0; y--){
//                debug.msgSize = sprintf(debug.payload, "%3i %3i %3i %3i %3i %3i %3i %3i %3i %3i %3i %3i %3i %3i %3i %3i %3i %3i \r\n", 
//                        grid[0][y], grid[1][y], grid[2][y],
//                        grid[3][y], grid[4][y], grid[5][y],
//                        grid[6][y], grid[7][y], grid[8][y],
//                        grid[9][y], grid[10][y], grid[11][y],
//                        grid[12][y], grid[13][y], grid[14][y],
//                        grid[15][y], grid[16][y], grid[17][y]
//                        );
                //SendMessageForTransmitQ(debug);
//            }
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
