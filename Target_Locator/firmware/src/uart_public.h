/* 
 * File:   uart_public.h
 * Author: Daniel
 *
 * Created on November 3, 2016, 8:33 PM
 */

#ifndef UART_PUBLIC_H
#define	UART_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif
#define SEARCHER_MOVEMENT   "SMM"
#define TARGET_LOCATOR      "TGL"
#define PATH_FINDER         "PTH"
#define GRABBER_MODULE      "GRB"
    
#define TARGET_TYPE         "TGT"
#define OBSTACLE_TYPE       "OBS"
#define GRID_TYPE           "GRD"
#define LOCATION_TYPE       "LOC"
#define TARGET_LOC_TYPE     "TLC"
#define VERTEX_TYPE         "VTX"
    
    
#define MAX_WIFLY_SIZE 512//Arbitrary maximum value, could increase
#define MSG_HEADER_SIZE 12//how much data is contained in the header
#define MAX_PAYLOAD_SIZE (MAX_WIFLY_SIZE - MSG_HEADER_SIZE)//calculates available space
#define MSG_START 0x4C32A5BD//first 4 bytes of message
#define DEVICEID 1
    
    //struct of the message payload information
typedef struct
{
    uint8_t payload[MAX_PAYLOAD_SIZE];//serialized JSON data
    int msgSize;//byte count
    uint8_t dest;//destination
    uint8_t source;//source
} messageItem_t;

//transmit byte to USART
int UARTSendByteToTXQ(uint8_t byte);
void UARTSendByteToTXQFromISR(uint8_t byte);
int UARTReceiveByteFromTXQ(uint8_t *byte);
int UARTReceiveByteFromTXQFromISR(uint8_t *byte);

//receive byte from USART
int UARTSendByteToRXQ(uint8_t byte);
void UARTSendByteTeRXQFromISR(uint8_t byte);
int UARTReceiveByteFromRXQ(uint8_t *byte);

//send payloads for package and transmission
int SendMessageForTransmitQ(messageItem_t message);
void SendMessageForTransmitFromISR(messageItem_t message);
int ReceiveMessageForTransmitQ(messageItem_t *message);

//void TX_InterruptHandler(const SYS_MODULE_INDEX index);

#ifdef	__cplusplus
}
#endif

#endif	/* UART_PUBLIC_H */

