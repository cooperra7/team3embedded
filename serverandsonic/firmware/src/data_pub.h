/* 
 * File:   data_pub.h
 * Author: theheavyhorse
 *
 * Created on September 27, 2016, 6:27 AM
 */

#ifndef DATA_PUB_H
#define	DATA_PUB_H

#ifdef	__cplusplus
extern "C" {
#endif

void dataQInit ();
DATAMSG_t dataQReceive ();
void dataQSendRequest (REQUEST_t request);
void dataQSendRequestFromISR (REQUEST_t request);
void dataQSendResponse (RESPONSE_t response);


#ifdef	__cplusplus
}
#endif

#endif	/* DATA_PUB_H */

