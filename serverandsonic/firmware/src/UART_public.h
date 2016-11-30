/* 
 * File:   UART_public.h
 * Author: Daniel
 *
 * SUMMARY:
 * this file contains all information needed to interface with wifly over UART
 * Created on September 14, 2016, 11:45 AM
 */

#ifndef UART_PUBLIC_H
#define	UART_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    
#define MAX_WIFLY_SIZE 256//Arbitrary maximum value, could increase
#define MSG_HEADER_SIZE 9//how much data is contained in the header
#define MAX_PAYLOAD_SIZE (MAX_WIFLY_SIZE - MSG_HEADER_SIZE)//calculates available space
#define MSG_START 0x4C32A5BD//first 4 bytes of message
    
    //struct of the message payload information
typedef struct
{
    uint8_t payload[MAX_PAYLOAD_SIZE];//serialized JSON data
    int msgSize;//byte count
} messageItem_t;

//transmit byte to USART
int UARTSendByteToTXQ(uint8_t byte);
void UARTSendByteToTXQFromISR(uint8_t byte);
int UARTReceiveByteFromTXQ(uint8_t *byte);

//receive byte from USART
int UARTSendByteToRXQ(uint8_t byte);
void UARTSendByteTeRXQFromISR(uint8_t byte);
int UARTReceiveByteFromRXQ(uint8_t *byte);

//send payloads for package and transmission
int SendMessageForTransmitQ(messageItem_t message);
void SendMessageForTransmitFromISR(messageItem_t message);
int ReceiveMessageForTransmitQ(messageItem_t *message);

#ifdef	__cplusplus
}
#endif

#endif	/* UART_PUBLIC_H */

