/* 
 * File:   debug.h
 * Author: theheavyhorse
 *
 * Created on September 21, 2016, 1:38 PM
 */

#ifndef DEBUG_H
#define	DEBUG_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include "system_config.h"
#include "system_definitions.h"

/* This function outputs a character to 8 GPIO pins to use for debugging.
    This function is used to output specified characters */
void dbgOutputVal (unsigned char outVal);

/* THis function outputs a character to 8 GPIO pins to use for debugging.
    This function is used to tell if we reach a location in the code */
void dbgOutputLoc (unsigned char outVal);

/* This function indicates an error value and brings the task to a halt */
void error ();


#ifdef	__cplusplus
}
#endif

#endif	/* DEBUG_H */

