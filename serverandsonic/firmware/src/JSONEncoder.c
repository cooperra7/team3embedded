/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    jsonencoder.c

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

#include "jsonencoder.h"
#include "JSONlayer_pub.h"
#include "UART_public.h"

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

JSONENCODER_DATA jsonencoderData;

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
    
void creditResp (char * object)
{
    sprintf (object, "{ \"CRD\" : { \"myName\" : \"PATHMOVEMENT\", \"authorFirstName\" : \"Riley\", \"authorLastName\" : \"Cooper\" } }");
}
    
void commStatsResp (char * object, COMMSTATS_t commstats)
{
    sprintf (object, "{ \"commstats\" : { \"msLocalTimeTracker\" : %d, \"myName\" : \"PATHMOVEMENT\", \"numGoodMessagesRecved\" : %d, \"numCommErrors\" : %d, \"numJSONRequestsRecved\" : %d, \"numJSONResponsesRecved\" : %d, \"numJSONRequestsSent\" : %d, \"numJSONResponsesSent\" : %d } }", commstats.msLocalTimeTracker, commstats.numGoodMessagesRecved, commstats.numCommErrors, commstats.numJSONRequestsRecved, commstats.numJSONResponsesRecved, commstats.numJSONRequestsSent, commstats.numJSONResponsesSent);
}

void targetErrResp (char * object, TARGETERR_t target)
{
    sprintf (object, "{ \"targeterr\" : { \"X\" : %d, \"Y\" : %d, \"LEFT\" : %d, \"RIGHT\" : %d, \"UNKNOWN\" : %d} }", target.x, target.y, target.left, target.right, target.unknown);
}

void sendRequest (char * out, REQUEST_t request)
{
    sprintf (out, "{ \"REQUEST\" : { \"SOURCE\" : \"%s\", \"DEST\" : \"%s\", \"ID\" : %d, \"TYPE\" : \"%s\" } }", request.source, request.dest, request.ID, request.type);
}

void sendResponse (char * out, RESPONSE_t response)
{
    char object[128];
    memset (object, 0, 128);
   /* if (response.credit.type == CREDIT) {
        creditResp (object);
        sprintf (out, "{ \"RESPONSE\" : { \"SOURCE\" : \"%s\", \"DEST\" : \"%s\", \"ID\" : %d, \"TYPE\" : \"%s\", \"DATA\" : %s } }", response.credit.source, response.credit.dest, response.credit.ID, response.credit.type, object);
    }
    else if (response.commstats.type == COMMSTATS) {
        commStatsResp (object, response.commstats.commstats);
        sprintf (out, "{ \"RESPONSE\" : { \"SOURCE\" : \"%s\", \"DEST\" : \"%s\", \"ID\" : %d, \"TYPE\" : \"%s\", \"DATA\" : %s } }", response.commstats.type, response.commstats.ID, response.commstats.source, response.commstats.dest, object);
    }

    else if (response.sensorval.type == SENSORVAL) {
        sprintf (out, "{ \"DEBUG\" : { \"left\" : %d, \"right\" : %d, \"center\" : %d, \"x\" : %d, \"y\" : %d, \"direction\" : %d } }", response.sensorval.left, response.sensorval.right, response.sensorval.center, response.sensorval.distance, response.sensorval.theta, response.sensorval.direction);
    }*/
    if (strncmp(response.targeterr.type, TARGETERR, 3) == 0) {
        targetErrResp (object, response.targeterr.targeterr);
        sprintf (out, "{ \"RESPONSE\" : { \"SOURCE\" : \"%s\", \"DEST\" : \"%s\", \"ID\" : %d, \"TYPE\" : \"%s\", \"DATA\" : %s } }", response.targeterr.source, response.targeterr.dest, response.targeterr.ID, response.targeterr.type, object);
    }
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void JSONENCODER_Initialize ( void )

  Remarks:
    See prototype in jsonencoder.h.
 */

void JSONENCODER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    jsonencoderData.state = JSONENCODER_STATE_INIT;

    JSONencQInit();
}


/******************************************************************************
  Function:
    void JSONENCODER_Tasks ( void )

  Remarks:
    See prototype in jsonencoder.h.
 */

void JSONENCODER_Tasks ( void )
{
    char buffer[512];
    while (1) {
        memset (buffer, 0, 512);
        JSONRESPMSG_t msg;
        msg = JSONencQReceive();
        dbgOutputLoc(0x43);
        int dest;
        if (msg.request) {
            dbgOutputLoc(0x44);
            sendRequest (buffer, msg.msg.request);
            if (strncmp(msg.msg.request.dest, "TGL", 3) == 0) {
                dest = 1;
            }
            else if (strncmp(msg.msg.request.dest, "SMM", 3) == 0) {
                dest = 0;
            }
            else if (strncmp(msg.msg.request.dest, "SVR", 3) == 0) {
                dest = 4;
            }
        }
        else {
            sendResponse (buffer, msg.msg.response);
            dbgOutputLoc (0x46);
            if (strncmp(msg.msg.response.credit.dest, "TGL", 3) == 0) {
                dbgOutputLoc(0x47);
                dest = 1;
            }
            else if (strncmp(msg.msg.response.credit.dest, "SMM", 3) == 0) {
                dest = 0;
            }
            else if (strncmp(msg.msg.response.credit.dest, "SVR", 3) == 0) {
                dest = 4;
                dbgOutputLoc(0x42);
            }
        }
        messageItem_t message;
        message.msgSize = strlen (buffer);
        int i = 0;
        for (i = 0; i < message.msgSize; i++) {
            message.payload[i] = buffer[i];
        }
        SendMessageForTransmitQ (message);
    }
}

QueueHandle_t JSONencQ;

void JSONencQInit ()
{
    JSONencQ = xQueueCreate (JSONENCQSIZE, JSONENCQITEMSIZE);
}

void JSONencQSendRequest (REQUEST_t request)
{
    JSONRESPMSG_t msg;
    msg.request = true;
    msg.msg.request = request;
    xQueueSend (JSONencQ, &msg, portMAX_DELAY);
}

void JSONencQSendResponse (RESPONSE_t response)
{
    JSONRESPMSG_t msg;
    msg.request = false;
    msg.msg.response = response;
    xQueueSend (JSONencQ, &msg, portMAX_DELAY);
}

JSONRESPMSG_t JSONencQReceive ()
{
    JSONRESPMSG_t msg;
    xQueueReceive (JSONencQ, &msg, portMAX_DELAY);
    return msg;
}
/*******************************************************************************
 End of File
 */


                    
                    