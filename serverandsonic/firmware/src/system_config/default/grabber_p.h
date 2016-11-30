/* 
 * File:   grabber_p.h
 * Author: theheavyhorse
 *
 * Created on October 12, 2016, 11:19 PM
 */

#ifndef GRABBER_P_H
#define	GRABBER_P_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include "PIC_Interface.h"

    typedef struct values
    {
        unsigned char sensor;
        uint16_t val1;
        uint16_t val2;
    }VALUES_t;
    
    typedef union
    {
        VALUES_t sensorvals;
        PIXY_DATA_t pixy;
    }GRABBER_u;
    
    typedef struct
    {
        int type;
        GRABBER_u info;
    }GRABBERMSG_t;
    
    void Qinit ();
    
    void QSendFromISR (VALUES_t tmrvals);
    
    void QSendVals (VALUES_t vals);
    
    void QSendPixy (PIXY_DATA_t pixy);
    
    GRABBERMSG_t QReceive ();
    
    int whichOC ();


#ifdef	__cplusplus
}
#endif

#endif	/* GRABBER_P_H */

