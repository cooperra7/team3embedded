/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    data.c

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

#include "data.h"
#include "data_pub.h"
#include "datastructures.h"
#include "debug.h"
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

DATA_DATA dataData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


CREDIT_t creditInit ()
{
    CREDIT_t ret;
    strcpy (ret.authorFirstName, "RILEY");
    strcpy (ret.authorLastName, "COOPER");
    strcpy (ret.myName, "GRABBER");
    return ret;
}

COMMSTATS_t commStatsInit ()
{
    COMMSTATS_t ret;
    ret.msLocalTimeTracker = 0;
    strcpy (ret.myName, "GRABBER");
    ret.numCommErrors = 0;
    ret.numGoodMessagesRecved = 0;
    ret.numJSONRequestsRecved = 0;
    ret.numJSONRequestsSent = 0;
    ret.numJSONResponsesSent = 0;
    ret.numJSONResponsesRecved = 0;
    return ret;
}

CONFIGLOC_t configLocInit (int newX, int newY, int newTheta)
{
    CONFIGLOC_t ret;
    ret.theta = newTheta;
    ret.x = newX;
    ret.y = newY;
    return ret;
}

NODE_t nodeInit (unsigned char newID, int newX, int newY, bool newIsTarget, bool newInArena)
{
    NODE_t ret;
    ret.ID = newID;
    ret.x = newX;
    ret.y = newY;
    ret.isTarget = newIsTarget;
    ret.inArena = newInArena;
    return ret;
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void DATA_Initialize ( void )

  Remarks:
    See prototype in data.h.
 */

void DATA_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    dataData.state = DATA_STATE_INIT;
    dataQInit ();
}


/******************************************************************************
  Function:
    void DATA_Tasks ( void )

  Remarks:
    See prototype in data.h.
 */

void DATA_Tasks ( void )
{
    CONFIGLOC_t currentLoc;
    NODE_t nextLoc;
    CREDIT_t credit;
    COMMSTATS_t commstats;
    
    uint16_t leftval = 0;
    uint16_t rightval = 0;
    
    currentLoc = configLocInit(0, 0, -1);
    credit = creditInit();
    commstats = commStatsInit();
    nextLoc = nodeInit (-1, 0, 0, false, false);
    
    while (1) {
        DATAMSG_t msg;
        msg = dataQReceive();
//        dbgOutputLoc (0x70);
        if (msg.request) {
            if (strncmp(msg.msg.request.source, "GRB", 3) == 0) {
                commstats.numJSONRequestsSent += 1;
                JSONencQSendRequest (msg.msg.request);
            }
            else {
                commstats.numJSONRequestsRecved += 1;
                commstats.numJSONResponsesSent += 1;
                RESPONSE_t response;
                if (strncmp (msg.msg.request.type, COMMSTATS, 3) == 0) {
                    strncpy (response.commstats.type, COMMSTATS, 3);
                    strncpy (response.commstats.dest, msg.msg.request.source, 3);
                    strncpy (response.commstats.source, GRABBER, 3);
                    response.commstats.ID = msg.msg.request.ID;
                    response.commstats.commstats = commstats;
                    JSONencQSendResponse (response);
                }
                else if (strncmp (msg.msg.request.type, CREDIT, 3) == 0) {
                    strncpy(response.credit.type, CREDIT, 3);
                    strncpy(response.credit.dest, msg.msg.request.source, 3);
                    strncpy(response.credit.source, GRABBER, 3);
                    response.credit.ID = msg.msg.request.ID;
                    response.credit.credit = credit;
                    JSONencQSendResponse (response);
                }
                else if (strncmp(msg.msg.request.type, SENSORVAL, 3) == 0) {
                    strncpy(response.sensorval.type, SENSORVAL, 3);
                    strncpy(response.sensorval.dest, msg.msg.request.source,3);
                    strncpy(response.sensorval.source, GRABBER, 3);
                    response.sensorval.ID = msg.msg.request.ID;
                    response.sensorval.left = leftval;
                    response.sensorval.right = rightval;
                    JSONencQSendResponse (response);
                }
                else if (strncmp(msg.msg.request.type, TARGETERR, 3) == 0) {
                    strncpy(response.targeterr.type, TARGETERR, 3);
                    strncpy(response.targeterr.dest, msg.msg.request.source,3);
                    strncpy(response.targeterr.source, GRABBER, 3);
                    JSONencQSendResponse (response);
                }
            }
        }
        else {
            if (strncmp(msg.msg.response.commstats.source, GRABBER, 3) == 0) {
                if (strncmp(msg.msg.response.commstats.type, COMMSTATS, 3) == 0) {
                    commstats.numGoodMessagesRecved += msg.msg.response.commstats.commstats.numGoodMessagesRecved;
                    commstats.numCommErrors += msg.msg.response.commstats.commstats.numCommErrors;
                }
                else if (msg.msg.response.sensorval.type == SENSORVAL) {
                    leftval = msg.msg.response.sensorval.left;
                    rightval = msg.msg.response.sensorval.right;
                    REQUEST_t testReq;
                    testReq.ID = 1;
                    strcpy(testReq.dest, "TGL");
                    strcpy(testReq.source, "GRB");
                    strcpy(testReq.type, "OBS");
//                    JSONencQSendRequest (testReq);
                }
            }
            else {
                commstats.numJSONResponsesRecved += 1;
                if (strncmp(msg.msg.response.targeterr.type, TARGETERR, 3) == 0) {
                    VALUES_t vals;
                    vals.val1 = msg.msg.response.targeterr.targeterr.distance;
                    vals.val2 = msg.msg.response.targeterr.targeterr.theta;
                    QSendVals (vals);
                }
            }
        }
    }
}

QueueHandle_t dataQ;

void dataQInit ()
{
    dataQ = xQueueCreate (DATAQSIZE, DATAQITEMSIZE);
}

DATAMSG_t dataQReceive ()
{
    DATAMSG_t newmsg;
    xQueueReceive (dataQ, &newmsg, portMAX_DELAY);
    return newmsg;
}

void dataQSendRequest (REQUEST_t request)
{
    DATAMSG_t msg;
    msg.request = true;
    msg.msg.request = request;
    xQueueSend (dataQ, &msg, portMAX_DELAY);
}

void dataQSendRequestFromISR (REQUEST_t request)
{
    /* Check for whether the task needs to do a context switch */
    BaseType_t conSwitch;
    conSwitch = pdFALSE;
    
    DATAMSG_t msg;
    msg.request = true;
    msg.msg.request = request;
    
    /* Send to the Queue from an ISR */
    xQueueSendFromISR (dataQ, &msg, &conSwitch);
    
    /* Switch if needed */
    portEND_SWITCHING_ISR(conSwitch);
}

void dataQSendResponse (RESPONSE_t response)
{
    DATAMSG_t msg;
    msg.request = false;
    msg.msg.response = response;
    xQueueSend (dataQ, &msg, portMAX_DELAY);
}

/*******************************************************************************
 End of File
 */
