/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    json_parser.c

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

#include "json_parser.h"
#include "PIC_interface.h"

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

JSON_PARSER_DATA json_parserData;
static QueueHandle_t JSON_PARSER_QUEUE;
static jsmn_parser Parser;

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
int JSONParserNewPacket(messageItem_t data)
{
    if (xQueueSend(JSON_PARSER_QUEUE, &data, portMAX_DELAY) == pdTRUE)
        return 1;
    return 0;
}
void JSONParserNewPacketFromISR(messageItem_t data)
{
    BaseType_t xHigherPriorityTaskWoken, xResult;
    xHigherPriorityTaskWoken = pdFALSE;
    xResult = xQueueSendFromISR(JSON_PARSER_QUEUE, &data, &xHigherPriorityTaskWoken);
    /* Actual macro used here is port specific. */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
int JSONParserReceivePacket(messageItem_t *data)
{
    if (xQueueReceive(JSON_PARSER_QUEUE, data, portMAX_DELAY) == pdTRUE)
       return 1;
   return 0;
}

//compare string to the token string
bool json_token_streq(char *js, jsmntok_t *t, char *s) 
{
    return (strncmp(js + t->start, s, t->end - t->start) == 0 
            && strlen(s) == (size_t) (t->end - t->start)); 
} 

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void JSON_PARSER_Initialize ( void )

  Remarks:
    See prototype in json_parser.h.
 */

void JSON_PARSER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    json_parserData.state = JSON_PARSER_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    JSON_PARSER_QUEUE = xQueueCreate(5, sizeof(messageItem_t));
    jsmn_init(&Parser);
}
//void handleJSON(JSON_OBJECT object)
//{
//    
//}

void parseTokens(char data[], jsmntok_t tokenList[], int tokenCount);
void parseJSONMessage(char data[], int dataSize, jsmntok_t tokenList[], int maxTokenCount)
{
    jsmn_init(&Parser);
    int tokenCount =  jsmn_parse(&Parser, data, dataSize, tokenList, maxTokenCount);
    if (tokenCount < 0){
        errorHandler(tokenCount);
    }
    if(tokenCount > maxTokenCount){
        tokenCount = maxTokenCount;
    }
    parseTokens(data, tokenList, tokenCount);
}

//Parse the list of tokens generated by jsmn
void parseTokens(char data[], jsmntok_t tokenList[], int tokenCount)
{
    messageItem_t debug;
//    debug.msgSize = sprintf(debug.payload, 
//                "{\"DEBUG\" : 1 , \"VERTEX\" : 1 }"
//                );
//    SendMessageForTransmitQ(debug);
    if(json_token_streq(data, &tokenList[1], "REQUEST")){
        if(json_token_streq(data, &tokenList[10], "OBSTACLES")){
            //classifyObjects
            debug.msgSize = sprintf(debug.payload,"{\"RESPONSE\" :  { \"TYPE\" : \"OBSTACLE\", \"DATA\" : {\"X\": 0 , \"Y\": 0}}}");
            SendMessageForTransmitQ(debug);
            classRequest();
        }
        else if(json_token_streq(data, &tokenList[10], "LOCATION")){
            
        }
    }
    else if(json_token_streq(data, &tokenList[1], "RESPONSE")){
        if(json_token_streq(data, &tokenList[10], "VERTEX")){
            //messageItem_t debug;
            debug.msgSize = sprintf(debug.payload, 
                        "{\"DEBUG\" : 1 , \"VERTEX\" : %i }", atoi(data + tokenList[13].start)
                        );
            SendMessageForTransmitQ(debug);
            VERTEX_t newVertex;
            newVertex.valid = true;
            newVertex.ID = atoi(data + tokenList[13].start);
            newVertex.x_center = atoi(data + tokenList[15].start);
            newVertex.y_center = atoi(data + tokenList[17].start);
            updateVertex(newVertex);
            
        }
        else if(json_token_streq(data, &tokenList[10], "LOCATION")){
            LOCATION_t newLocation;
            newLocation.x = atoi(data + tokenList[13].start);
            newLocation.y = atoi(data + tokenList[15].start);
            newLocation.theta = atoi(data + tokenList[17].start);
            setLocation(newLocation);
//            debug.msgSize = sprintf(debug.payload,"{\"RESPONSE\" :  { \"SOURCE\" : \"TargetLocator \", \"DEST\" : \"PATH\", \"ID\" : 0, \"TYPE\" : \"OBSTACLE\", \"DATA\" : {\"X\": 0, \"Y\": 0, \"LENGTH\": 0, \"WIDTH\": 0, \"THETA\": 0}}}");
//            debug.msgSize = sprintf(debug.payload,"{\"RESPONSE\" :  { \"TYPE\" : \"OBSTACLE\", \"DATA\" : {\"X\": 0 }}}");
//            SendMessageForTransmitQ(debug);
            debug.msgSize = sprintf(debug.payload, 
                        "{\"DEBUG\" : 1, \"VERTEX\" : 0}"
                        );
            SendMessageForTransmitQ(debug);
        }
        
    }
    
    
    //Erase Tokens
    int i;
    for(i = 0; i < tokenCount; i++){
        tokenList[i].type = JSMN_UNDEFINED;
        tokenList[i].start = 0;
        tokenList[i].end = 0;
        tokenList[i].size = 0;
    }
}
/******************************************************************************
  Function:
    void JSON_PARSER_Tasks ( void )

  Remarks:
    See prototype in json_parser.h.
 */

//array of tokens to store information in
static jsmntok_t tokens[MAX_TOKENS];


int numTokens;
static char data2[MAX_PAYLOAD_SIZE];
static int numValidMsg = 0;
void JSON_PARSER_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( json_parserData.state )
    {
        /* Application's initial state. */
        case JSON_PARSER_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                json_parserData.state = JSON_PARSER_STATE_SERVICE_TASKS;
            }
            break;
        }

        case JSON_PARSER_STATE_SERVICE_TASKS:
        {
            messageItem_t receivedMessage;
            dbgOutputLoc(DLOC_JSON_PARSE_WAIT_FOR_QUEUE);
            JSONParserReceivePacket(&receivedMessage);
            dbgOutputLoc(DLOC_JSON_PARSE_GOT_FROM_QUEUE);
            int count = receivedMessage.msgSize;
            char data[count+1];
            int i;
            for (i=0; i < count; i++){
                data[i] = receivedMessage.payload[i];
            }
            data[count] = '\0';
            //JSON_OBJECT parsedMessage;
            //parseJSON(data, count, &parsedMessage);
            //handleJSON(parsedMessage);
            
            //initialize parser and then parse json
            parseJSONMessage(data, count, tokens, MAX_TOKENS);
            numValidMsg++;
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
