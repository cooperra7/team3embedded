/* 
 * File:   JSONlayer_pub.h
 * Author: theheavyhorse
 *
 * Created on September 21, 2016, 11:40 PM
 */

#ifndef JSONLAYER_PUB_H
#define	JSONLAYER_PUB_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <FreeRTOS.h>
#include <queue.h>
#include <stdio.h>
#include "jsmn.h"
#include "datastructures.h"
#include "debug.h"

void JSONencQInit ();

void JSONencQSendRequest (REQUEST_t request);

void JSONencSendResponse (RESPONSE_t response);

JSONRESPMSG_t JSONencQReceive ();

void JSONdecQInit ();

void JSONdecQReceive (char * buffer);

void JSONdecQSend (char * buffer);

NODE_t parseNode (char * buffer, jsmntok_t tokens[]);

CONFIGLOC_t parseConfigLoc (char * buffer, jsmntok_t tokens[]);

COMMSTATS_t parseCommStats (char * buffer, jsmntok_t tokens[]);

CREDIT_t parseCredit (char * buffer, jsmntok_t tokens[]);

REQUEST_t parseRequest (char * buffer, jsmntok_t tokens[]);

RESPONSE_t parseResponse (char * buffer, jsmntok_t tokens[]);


#ifdef	__cplusplus
}
#endif

#endif	/* JSONLAYER_PUB_H */

