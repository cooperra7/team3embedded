/* 
 * File:   PING_public.h
 * Author: Daniel
 *
 * Created on November 4, 2016, 1:38 AM
 */

#ifndef PING_PUBLIC_H
#define	PING_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct values
    {
        unsigned char sensor;
        uint16_t val1;
        uint16_t val2;
    }VALUES_t;
    
    void Qinit ();
    
    void QSendFromISR (VALUES_t tmrvals);
    
    VALUES_t QReceive ();
    
    int whichOC ();


#ifdef	__cplusplus
}
#endif

#endif	/* PING_PUBLIC_H */

