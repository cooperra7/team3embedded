/* 
 * File:   debug.h
 * Author: Daniel
 * contains all debug commands
 * 
 * 
 * Created on September 5, 2016, 11:57 AM
 */

#ifndef DEBUG_H
#define	DEBUG_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "system_definitions.h"
#include "pinTranslate.h"
//#include "sys_debug.h"

//helper function for pin interactions
void pinSetOrClearPin(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos, bool dir);

void enableDbgVal();
void enable_dbgOutputLoc();
void dbgOutputVal(unsigned char outVal);//debug chipKIT 46-53?
void dbgOutputLoc(unsigned char outVal);//debug chipKIT 30-37
void errorHandler(uint8_t val);//block forever and print the error value

//list of all task states and values to debug
#define DLOC_APP1_ENTERED_TASK 0x01
#define DLOC_APP1_STARTING_WHILE 0x02
#define DLOC_APP1_WAIT_FOR_QUEUE 0x03
#define DLOC_APP1_GOT_FROM_QUEUE 0x04
#define DLOC_TMR_ISR_ENTER 0x05
#define DLOC_TMR_ISR_EXIT 0x06
#define DLOC_TMR_ISR_QUEUE_SEND 0x07
#define DLOC_TMR_ISR_QUEUE_SENT 0x08
#define DLOC_UART_TX_ISR_ENTER 0x09
#define DLOC_UART_TX_ISR_EXIT 0x0A
#define DLOC_UART_TX_ISR_QUEUE_SEND 0x0B
#define DLOC_UART_TX_ISR_QUEUE_SENT 0x0C
#define DLOC_UART_TX_ENTERED_TASK 0x0D
#define DLOC_UART_TX_STARTING_WHILE 0x0E
#define DLOC_UART_TX_WAIT_FOR_QUEUE 0x0F
#define DLOC_UART_TX_GOT_FROM_QUEUE 0x10
#define DLOC_UART_RX_ISR_ENTER 0x11
#define DLOC_UART_RX_ISR_EXIT 0x12
#define DLOC_UART_RX_ISR_QUEUE_SEND 0x13
#define DLOC_UART_RX_ISR_QUEUE_SENT 0x14
#define DLOC_UART_RX_ENTERED_TASK 0x15
#define DLOC_UART_RX_STARTING_WHILE 0x16
#define DLOC_UART_RX_WAIT_FOR_QUEUE 0x17
#define DLOC_UART_RX_GOT_FROM_QUEUE 0x18
#define DLOC_UART_RX_SENT_TO_QUEUE 0x019
#define DLOC_JSON_GEN_WAIT_FOR_QUEUE 0x1A
#define DLOC_JSON_GEN_GOT_FROM_QUEUE 0x1B
#define DLOC_JSON_PARSE_WAIT_FOR_QUEUE 0x1C
#define DLOC_JSON_PARSE_GOT_FROM_QUEUE 0x1D
#define DLOC_ADC_ENTER_ISR 0x20
#define DLOC_ADC_EXIT_ISR 0x21
#define DLOC_ADC_WAIT_FOR_QUEUE 0x22
#define DLOC_ADC_GOT_FROM_QUEUE 0x23
#define DLOC_PIXY_ENTER_ISR 0x24
#define DLOC_PIXY_EXIT_ISR 0x25
#define DLOC_PIXY_WAIT_FOR_QUEUE 0x26
#define DLOC_PIXY_GOT_FROM_QUEUE 0x27



#ifdef	__cplusplus
}
#endif

#endif	/* DEBUG_H */

