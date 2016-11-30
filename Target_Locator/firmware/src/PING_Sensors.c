/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    ping_sensors.c

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

#include "ping_sensors.h"

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

PING_SENSORS_DATA ping_sensorsData;

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
QueueHandle_t Q;
int OCenabled;


void Qinit ()
{
    Q = xQueueCreate (5, sizeof (VALUES_t));
}

void QSendFromISR (VALUES_t tmrvals)
{
    /* Check for whether the task needs to do a context switch */
    BaseType_t conSwitch;
    conSwitch = pdFALSE;
    
    /* Send to the Queue from an ISR */
    if ((xQueueOverwriteFromISR (Q, &tmrvals, &conSwitch)) == errQUEUE_FULL) {
        PLIB_PORTS_Write (PORTS_ID_0, PORT_CHANNEL_E, 0x00FA);
//        xQueueReset(Q);
    }
    
    /* Switch if needed */
    portEND_SWITCHING_ISR(conSwitch);
}

VALUES_t QReceive ()
{
    VALUES_t ret;
    xQueueReceive (Q, &ret, portMAX_DELAY);
    return ret;
}

int whichOC ()
{
    if (OCenabled == 0) {
        OCenabled = 1;
        dbgOutputLoc (0xEA);
        return 0;
    }
    else if (OCenabled == 1) {
        dbgOutputLoc (0xEB);
        OCenabled = 2;
        return 1;
    }
    else if (OCenabled == 2) {
        OCenabled = 3;
        return 2;
    }
    else if (OCenabled == 3) {
        OCenabled = 0;
        return 3;
    }
    OCenabled = 0;
    return 2;
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void PING_SENSORS_Initialize ( void )

  Remarks:
    See prototype in ping_sensors.h.
 */

void PING_SENSORS_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    ping_sensorsData.state = PING_SENSORS_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    DRV_TMR0_Initialize (); /* Initialize the driver timer */
    DRV_OC0_Initialize ();
    DRV_OC1_Initialize ();
    DRV_OC2_Initialize ();
    DRV_OC3_Initialize ();
    Qinit(); /* Initialize the Queue */
    OCenabled = 0;
    DRV_TMR0_Start ();
    DRV_IC0_Start ();
    DRV_IC1_Start ();
    DRV_IC2_Start ();
    DRV_IC3_Start ();
    
    
    
    bool center = false;
    bool left = false;
    bool right = false;
}


/******************************************************************************
  Function:
    void PING_SENSORS_Tasks ( void )

  Remarks:
    See prototype in ping_sensors.h.
 */
int count = 0;
int leftAvg = 0;
int centerAvg = 0;
int rightAvg = 0;
int backAvg = 0;
int count;
void PING_SENSORS_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( ping_sensorsData.state )
    {
        /* Application's initial state. */
        case PING_SENSORS_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                ping_sensorsData.state = PING_SENSORS_STATE_SERVICE_TASKS;
            }
            break;
        }

        case PING_SENSORS_STATE_SERVICE_TASKS:
        {
            VALUES_t vals;
            dbgOutputLoc(0x61);
            vals = QReceive();
            int diff = vals.val1 - vals.val2;
            diff = abs(diff);
            int step = diff * 8;
            int cm = step / 58;
            if (cm < 1000){
                cm = (cm*3 - 17)/40;
                if (vals.sensor == 0x00) {
                    leftAvg = cm;
                }
                else if (vals.sensor == 0x11) {
                    rightAvg = cm;
                }
                else if (vals.sensor == 0x22) {
                    centerAvg = cm;
                }
                else if (vals.sensor == 0x33) {
                    backAvg = cm;
                }
            }
            
            if (count % 3 == 0){
                PING_DATA_t new_data;
                new_data.left = leftAvg;
                new_data.right = rightAvg;
                new_data.center = centerAvg;
                new_data.back = backAvg;
                GRID_QSend(new_data);
                if(count > 15){
                    messageItem_t debug;
                    debug.msgSize = sprintf(debug.payload, 
                            "{\"DEBUG\" : 1 , \"%s\" : %u  , \"%s\" : %u  , \"%s\" : %u  , \"%s\" : %u }", 
                            //"{\"DEBUG\" : 1 , \"%s\" : %u  , \"%s\" : %u  , \"%s\" : %u  , \"%s\" : %u  , \"%s\" : %u }", 
                            "LEFT", leftAvg, 
                            "RIGHT", rightAvg, 
                            "CENTER", centerAvg, 
                            "BACK", backAvg);
                    SendMessageForTransmitQ(debug);
                
                
                    count = 0;
                }
            }
            count++;
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
