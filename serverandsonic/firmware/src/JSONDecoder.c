/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    jsondecoder.c

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

#include "jsondecoder.h"
#include "JSONlayer_pub.h"

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

JSONDECODER_DATA jsondecoderData;

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


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void JSONDECODER_Initialize ( void )

  Remarks:
    See prototype in jsondecoder.h.
 */

void JSONDECODER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    jsondecoderData.state = JSONDECODER_STATE_INIT;

    JSONdecQInit();
}


/******************************************************************************
  Function:
    void JSONDECODER_Tasks ( void )

  Remarks:
    See prototype in jsondecoder.h.
 */

void JSONDECODER_Tasks ( void )
{
    char buffer [512];
    char test [512];
    while (1) {
        memset (buffer, 0, 512);
        memset (test, 0, 512);
        JSONdecQReceive (buffer);
        
        jsmn_parser parser;
        jsmn_init (&parser);

        jsmntok_t tokens[35];
        jsmn_parse (&parser, buffer, strlen(buffer), tokens, 35);

//        if (strncmp (buffer + tokens[1].start, "request", 7) == 0) {
//            REQUEST_t myrequest = parseRequest ( buffer, tokens);
//            JSONencQSendRequest (myrequest);
            //            dataQSendRequest (parseRequest(buffer, tokens));
//        }
//        else if (strncmp (buffer + tokens[1].start, "response", 8) == 0) {
//            dataQSendResponse (parseResponse(buffer, tokens));
//        }
    }
}

QueueHandle_t JSONdecQ;

void JSONdecQInit ()
{
    JSONdecQ = xQueueCreate (JSONDECQSIZE, JSONDECQITEMSIZE);
}

void JSONdecQSend (char * buffer)
{
    xQueueSend (JSONdecQ, buffer, portMAX_DELAY);
}

void JSONdecQReceive (char * buffer)
{
    xQueueReceive (JSONdecQ, buffer, portMAX_DELAY);
}

NODE_t parseNode (char * buffer, jsmntok_t tokens[])
{
    NODE_t node;
    /*char toInt[16];
    memset (toInt, 0 ,16);
    strncpy (toInt, buffer + tokens[4].start, tokens[4].end - tokens[4].start);
    node.ID = atoi (toInt);
    memset (toInt, 0 ,16);
    strncpy (toInt, buffer + tokens[6].start, tokens[6].end - tokens[6].start);
    node.x = atoi (toInt);
    memset (toInt, 0 ,16);
    strncpy (toInt, buffer + tokens[8].start, tokens[8].end - tokens[8].start);
    node.y = atoi (toInt);
    if (*(buffer + tokens[10].start) == 't') {
        node.isTarget = true;
    }
    else {
        node.isTarget = false;
    }
    if (*(buffer + tokens[12].start) == 't') {
        node.inArena = true;
    }
    else {
        node.inArena = false;
    }*/
    return node;
}

CONFIGLOC_t parseConfigLoc (char * buffer, jsmntok_t tokens[])
{
    CONFIGLOC_t configloc;
    /*
    char toInt[16];
    memset (toInt, 0 ,16);
    strncpy (toInt, buffer + tokens[4].start, tokens[4].end - tokens[4].start);
    configloc.x = atoi (toInt);
    memset (toInt, 0 ,16);
    strncpy (toInt, buffer + tokens[6].start, tokens[6].end - tokens[6].start);
    configloc.y = atoi (toInt);
    memset (toInt, 0 ,16);
    strncpy (toInt, buffer + tokens[8].start, tokens[8].end - tokens[8].start);
    configloc.theta = atoi (toInt);*/
    return configloc;
}

COMMSTATS_t parseCommStats (char * buffer, jsmntok_t tokens[])
{
    COMMSTATS_t commstats;
    char toInt[16];
    memset (toInt, 0 ,16);
    strncpy (toInt, buffer + tokens[4].start, tokens[4].end - tokens[4].start);
    commstats.msLocalTimeTracker = atoi (toInt);
    memset (commstats.myName, 0, 13);
    strncpy (commstats.myName, buffer + tokens[6].start, tokens[6].end - tokens[6].start);
    memset (toInt, 0 ,16);
    strncpy (toInt, buffer + tokens[8].start, tokens[8].end - tokens[8].start);
    commstats.numGoodMessagesRecved = atoi (toInt);
    memset (toInt, 0 ,16);
    strncpy (toInt, buffer + tokens[10].start, tokens[10].end - tokens[10].start);
    commstats.numCommErrors = atoi (toInt);
    memset (toInt, 0 ,16);
    strncpy (toInt, buffer + tokens[12].start, tokens[12].end - tokens[12].start);
    commstats.numJSONRequestsRecved = atoi (toInt);
    memset (toInt, 0 ,16);
    strncpy (toInt, buffer + tokens[14].start, tokens[14].end - tokens[14].start);
    commstats.numJSONResponsesRecved = atoi (toInt);
    memset (toInt, 0 ,16);
    strncpy (toInt, buffer + tokens[16].start, tokens[16].end - tokens[16].start);
    commstats.numJSONRequestsSent = atoi (toInt);
    memset (toInt, 0 ,16);
    strncpy (toInt, buffer + tokens[18].start, tokens[18].end - tokens[18].start);
    commstats.numJSONResponsesSent = atoi (toInt);
    return commstats;
}

CREDIT_t parseCredit (char * buffer, jsmntok_t tokens[])
{
    CREDIT_t credit;
    memset (credit.myName, 0, 13);
    memset (credit.authorFirstName, 0, 9);
    memset (credit.authorLastName, 0, 7);
    strncpy (credit.myName, buffer + tokens[4].start, tokens[4].end - tokens[4].start);
    strncpy (credit.authorFirstName, buffer + tokens[6].start, tokens[6].end - tokens[6].start);
    strncpy (credit.authorLastName, buffer + tokens[8].start, tokens[8].end - tokens[8].start);
    return credit;
}

TARGETERR_t parseTargeterr (char * buffer, jsmntok_t tokens[])
{
    TARGETERR_t targeterr;
    char toInt[16];
    memset(toInt, 0, 16);
    strncpy (toInt, buffer + tokens[4].start, tokens[4].end - tokens[4].start);
    targeterr.distance = atoi (toInt);
    memset(toInt, 0, 16);
    strncpy (toInt, buffer + tokens[6].start, tokens[6].end - tokens[6].start);
    targeterr.theta = atoi (toInt);
    return targeterr;
}

REQUEST_t parseRequest (char * buffer, jsmntok_t tokens[])
{
    REQUEST_t request;
    char toInt[16];
    memset (toInt, 0 ,16);
    strncpy (request.source, buffer + tokens[4].start, tokens[4].end - tokens[4].start);
    strncpy (request.dest, buffer + tokens[6].start, tokens[6].end - tokens[6].start);
    strncpy (toInt, buffer + tokens[8].start, tokens[8].end - tokens[8].start);
    request.ID = atoi (toInt);
    memset (toInt, 0 ,16);
    strncpy (request.type, buffer + tokens[10].start, tokens[10].end - tokens[10].start);
    return request;
}

RESPONSE_t parseResponse (char * buffer, jsmntok_t tokens[])
{
    RESPONSE_t response;
    char toInt[16];
    memset (toInt, 0 ,16);
    strncpy (toInt, buffer + tokens[4].start, tokens[4].end - tokens[4].start);
    response.credit.type = atoi (toInt);
    memset (toInt, 0 ,16);
    strncpy (toInt, buffer + tokens[6].start, tokens[6].end - tokens[6].start);
    response.credit.ID = atoi (toInt);
    memset (toInt, 0 ,16);
    strncpy (toInt, buffer + tokens[8].start, tokens[8].end - tokens[8].start);
    response.credit.source = atoi (toInt);
    memset (toInt, 0 ,16);
    strncpy (toInt, buffer + tokens[10].start, tokens[10].end - tokens[10].start);
    response.credit.dest = atoi (toInt);
    if (response.credit.type == CREDIT) {
        response.credit.credit = parseCredit (buffer, tokens + 10);
    }
    else if (response.node.type == NODE) {
        response.node.node = parseNode (buffer, tokens + 10);
    }
    else if (response.configloc.type == CONFIGLOC) {
        response.configloc.configloc = parseConfigLoc (buffer, tokens + 10);
    }
    else if (response.commstats.type == COMMSTATS) {
        response.commstats.commstats = parseCommStats (buffer, tokens + 10);
    }
    return response;
}