/*******************************************************************************
  MPLAB Harmony Application Source File
 comment
  Company:
    Microchip Technology Inc.
  
  File Name:
    grabber.c

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

#include "grabber.h"
#include "grabber_p.h"
#include "datastructures.h"
#include "data_pub.h"
#include "debug.h"
#include <math.h>

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

GRABBER_DATA grabberData;

#define LX -40
#define RX 40
#define CX 0
#define LY -10
#define RY -10
#define CY -2
#define LX2 pow(LX, 2)
#define RX2 pow(RX, 2)
#define CX2 pow(CX, 2)
#define LY2 pow(LY, 2)
#define RY2 pow(RY, 2)
#define CY2 pow(CY, 2)
#define R 38

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


bool inSight (int val)
{
    if (val < 200) {
        return true;
    }
}
/*Assumes arc centers at (3,0) and (-3,0)*/
POS_t getPos (int left, int right) {
    
    POS_t ret;
    double rr = right + R;
    double rl = left + R;
    double sqs = 0;
    double yd = 0;
    double xd = 0;
    double c = 0;
    double two = 0;
    double one = 0;
    double zed = 0;
    int y = 0;
    int x = 0;
    
    if (left > right) {
        sqs = (LX * LX) + (LY * LY) - (RX * RX) - (RY * RY);
        yd = RY - LY;
        xd = RX - LX;
        c = (rl * rl) - (rr * rr) - sqs;
        two = ((yd * yd) / (xd * xd)) + 1;
        one = ((c * yd)/(xd * xd)) + ((2 * LX * yd)/xd) - (2*LY);
        zed = ((c/(2 * xd)) * (c/(2 * xd))) + (LX * LX) + (LY * LY) - ((LX * c)/xd) - (rl * rl);
    
        y = ((one * -1) + sqrt ((one * one) - (4 * two * zed))) / (2 * two);
        if (y < 0) {
            y = ((one * -1) - sqrt ((one * one) - (4 * two * zed))) / (2 * two);
        }
        x = ((rl * rl) - (rr * rr) - sqs - (2 * yd * y))/ (2 * xd);
    }
    else if (right > left) {
        sqs = (RX * RX) + (RY * RY) - (LX * LX) - (LY * LY);
        yd = LY - RY;
        xd = LX - RX;
        c = (rr * rr) - (rl * rl) - sqs;
        two = ((yd * yd) / (xd * xd)) + 1;
        one = ((c * yd)/(xd * xd)) + ((2 * RX * yd)/xd) - (2*RY);
        zed = ((c/(2 * xd)) * (c/(2 * xd))) + (RX * RX) + (RY * RY) - ((RX * c)/xd) - (rl * rl);
    
        y = ((one * -1) + sqrt ((one * one) - (4 * two * zed))) / (2 * two);
        if (y < 0) {
            y = ((one * -1) - sqrt ((one * one) - (4 * two * zed))) / (2 * two);
        }
        x = ((rr * rr) - (rl * rl) - sqs - (2 * yd * y))/ (2 * xd);
        x = -x;
    }
    else {
    }
    ret.x = x;
    ret.y = y;
    return ret;
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void GRABBER_Initialize ( void )

  Remarks:
    See prototype in grabber.h.
 */

 int OCenabled;

void GRABBER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    grabberData.state = GRABBER_IDLE;

    
    PLIB_PORTS_DirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_E, 0x01FF);
//    PLIB_PORTS_DirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_A, 0x00EC);
//    PLIB_PORTS_DirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_G, 0x7000);
    PLIB_PORTS_DirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_D, 0x2293);
//    PLIB_PORTS_DirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_C, 0x6010);
    
    DRV_TMR0_Initialize (); /* Initialize the driver timer */
    DRV_OC0_Initialize ();
    DRV_OC1_Initialize ();
    DRV_OC2_Initialize ();
    
    Qinit(); /* Initialize the Queue */
    OCenabled = 0;

    PLIB_PORTS_PinDirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_D, 3);
    PLIB_PORTS_PinDirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_D, 4);
    PLIB_PORTS_PinDirectionInputSet (PORTS_ID_0, PORT_CHANNEL_D, 10);
    PLIB_PORTS_PinDirectionInputSet (PORTS_ID_0, PORT_CHANNEL_D, 8);

    PLIB_PORTS_PinDirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_C, 1);
    PLIB_PORTS_PinDirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_A, 3);
    PLIB_PORTS_PinDirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_D, 5);
    
}


/******************************************************************************
  Function:
    void GRABBER_Tasks ( void )

  Remarks:
    See prototype in grabber.h.
 */

void GRABBER_Tasks ( void )
{

    DRV_TMR0_Start ();
    DRV_IC0_Start ();
    DRV_IC1_Start ();
    DRV_IC2_Start ();
    PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_C, 1, 0);
    PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_A, 3, 0);
//    DRV_TMR1_Start();
//    DRV_OC0_Start ();
    
    RESPONSE_t resp;
    strncpy(resp.sensorval.type, SENSORVAL, 3);
    resp.sensorval.ID = 12;
    strncpy(resp.sensorval.source, GRABBER, 3);
    strncpy(resp.sensorval.dest, GRABBER, 3);
    resp.sensorval.left = 0;
    resp.sensorval.right = 0;
    resp.sensorval.center = 0;
    resp.sensorval.distance = 0;
    resp.sensorval.theta = 0;
    resp.sensorval.direction = 0;
    
    dataQSendResponse (resp);
    
    
    int count = 0;
    int lcount = 0;
    int rcount = 0;
    int ccount = 0;
    int leftAvg = 0;
    int centerAvg = 0;
    int rightAvg = 0;
    PIXY_DATA_t curPixy;
    curPixy.ID = 0;
    curPixy.height = 0;
    curPixy.width = 0;
    curPixy.x_center = 0;
    curPixy.y_center = 0;
    
    bool center = true;
    bool left = true;
    bool right = true;
    
    while (1)
    {
        GRABBERMSG_t msg;
        VALUES_t vals;
        PIXY_DATA_t newPixy;
        msg = QReceive();
        vals = msg.info.sensorvals;
        newPixy = msg.info.pixy;

        switch (grabberData.state) {
            case GRABBER_IDLE : {
                count++;
                if (count == 60) {
                    count = 0;
                    grabberData.state = GRABBER_SEARCHING;
                    PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_C, 1, 0);
                    PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_D, 5, 1);
                }
                break;
            }
            case GRABBER_SEARCHING : {
                PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_C, 1, 1);
                if (msg.type == 1) {
                    int diff = vals.val1 - vals.val2;
                    diff = abs(diff);
                    int step = diff * 32;
                    int cm = step / 58;
                    if (vals.sensor == 0x00) {
                        if (left) {
                            leftAvg = cm;
                            lcount++;
                        }
                    }
                    else if (vals.sensor == 0x11) {
                        if (right) {
                            rightAvg = cm;
                            rcount++;
                        }
                    }
                    else if (vals.sensor == 0x22) {
                        if (center) {
                            centerAvg = cm;
                            ccount++;
                        }
                    }
                }
                else if (msg.type == 2) {
                    curPixy.ID = newPixy.ID;
                    curPixy.height = newPixy.height;
                    curPixy.width = newPixy.width;
                    curPixy.x_center = newPixy.x_center;
                    curPixy.y_center = newPixy.y_center;
                }
                resp.sensorval.left = leftAvg;
                resp.sensorval.right = rightAvg;
                resp.sensorval.center = centerAvg;
                
                if (leftAvg < 200 && rightAvg < 200 && curPixy.x_center < 200 && curPixy.x_center > 120) {
                    POS_t pos = getPos (leftAvg, rightAvg);
                    resp.sensorval.direction = 0;
                    resp.sensorval.distance = pos.x;
                    resp.sensorval.theta = pos.y;
                }
                else {
                    if (curPixy.x_center > 160) {
                        resp.sensorval.direction = 1;
                    }
                    else if (curPixy.x_center < 160) {
                        resp.sensorval.direction = -1;
                    }
                    resp.sensorval.distance = -1;
                    resp.sensorval.theta = -1;
                }

                dataQSendResponse(resp);
                break;
            }
            case GRABBER_HOLDING : {
                count ++;
                resp.sensorval.distance = -1;
                resp.sensorval.theta = 0;
                dataQSendResponse (resp);
                PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_A, 3, 1);
                if (count == 60) {
                    count = 0;
                    PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_A, 3, 0);
                    PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_D, 5, 0);
                    grabberData.state = GRABBER_IDLE;
                    //send request
                }
                break;
            }
            default : 
                grabberData.state = GRABBER_IDLE;
        }
    }
}

QueueHandle_t Q;


void Qinit ()
{
    Q = xQueueCreate (1, sizeof (GRABBERMSG_t));
}

void QSendFromISR (VALUES_t tmrvals)
{
    GRABBERMSG_t msg;
    msg.type = 1;
    msg.info.sensorvals.sensor = tmrvals.sensor;
    msg.info.sensorvals.val1 = tmrvals.val1;
    msg.info.sensorvals.val2 = tmrvals.val2;
    /* Check for whether the task needs to do a context switch */
    BaseType_t conSwitch;
    conSwitch = pdFALSE;
    
    /* Send to the Queue from an ISR */
    if ((xQueueOverwriteFromISR (Q, &msg, &conSwitch)) == errQUEUE_FULL) {
//        xQueueReset(Q);
    }
    
    /* Switch if needed */
    portEND_SWITCHING_ISR(conSwitch);
}

void QSendVals (VALUES_t vals)
{
    GRABBERMSG_t msg;
    msg.type = 1;
    msg.info.sensorvals.sensor = vals.sensor;
    msg.info.sensorvals.val1 = vals.val1;
    msg.info.sensorvals.val2 = vals.val2;
    xQueueSend (Q, &msg, portMAX_DELAY);
}

void QSendPixy (PIXY_DATA_t pixy)
{
    GRABBERMSG_t msg;
    msg.type = 2;
    msg.info.pixy.ID = pixy.ID;
    msg.info.pixy.height = pixy.height;
    msg.info.pixy.width = pixy.width;
    msg.info.pixy.x_center = pixy.x_center;
    msg.info.pixy.y_center = pixy.y_center;
    xQueueSend (Q, &msg, portMAX_DELAY);
}

GRABBERMSG_t QReceive ()
{
    GRABBERMSG_t ret;
    xQueueReceive (Q, &ret, portMAX_DELAY);
    return ret;
}

int whichOC ()
{
    if (OCenabled == 0) {
        OCenabled = 1;
        return 0;
    }
    else if (OCenabled == 1) {
        OCenabled = 2;
        return 1;
    }
    else if (OCenabled == 2) {
        OCenabled = 0;
        return 2;
    }
    else if (OCenabled == 3) {
        OCenabled = 0;
        return 3;
    }
    OCenabled = 0;
    return 2;
}