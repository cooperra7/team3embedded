#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#ifdef  __cplusplus
extern "C" {
#endif

#define PATHMOVEMENT 0x50
#define SEARCHINGMOVEMENT 0x60
#define TARGETLOCATOR 0x70
#define GRABBER 0x80
#define CREDIT 0x10
#define CONFIGLOC 0x20
#define COMMSTATS 0x30
#define NODE 0x40
#define SENSORVAL 0x05
#define TARGETERR 0x06
    
    typedef struct
    {
        char myName[13];
        char authorFirstName[6];
        char authorLastName[7];
    }CREDIT_t;
    
    typedef struct
    {
        int x;
        int y;
    }POS_t;
    
    CREDIT_t creditInit();
    
    typedef struct
    {
        int distance;
        int theta;
    }TARGETERR_t;
    
    typedef struct
    {
        int msLocalTimeTracker;
        char myName[13];
        int numGoodMessagesRecved;
        int numCommErrors;
        int numJSONRequestsRecved;
        int numJSONResponsesRecved;
        int numJSONRequestsSent;
        int numJSONResponsesSent;
    }COMMSTATS_t;
    
    COMMSTATS_t commStatsInit ();
    
    typedef struct
    {
        int x;
        int y;
        int theta;
    }CONFIGLOC_t;
    
    CONFIGLOC_t configLocInit (int newX, int newY, int newTheta);

    typedef struct
    {
        int ID;
        int x;
        int y;
        bool isTarget;
        bool inArena;
    }NODE_t;
    
    NODE_t nodeInit (unsigned char newID, int newX, int newY, bool newIsTarget, bool newInArena);

    typedef struct
    {
        unsigned char ID;
        int x;
        int y;
        bool isTarget;
        bool inArena;
        unsigned char numLinks;
        unsigned char links[8];
    }INTERSECTION_t;
    
    typedef struct
    {
        uint32_t val;
    }SENSORVAL_t;

    typedef struct
    {
        char type[4];
        int ID;
        char source[4];
        char dest[4];
    }REQUEST_t;
    
    REQUEST_t requestInit (int newType, int newID, int newSource, int newDest);

    typedef struct
    {
        int type;
        int ID;
        int source;
        int dest;
        CREDIT_t credit;
    }CREDITRESP_t;

    typedef struct
    {
        int type;
        int ID;
        int source;
        int dest;
        NODE_t node;
    }NODERESP_t;

    typedef struct
    {
        int type;
        int ID;
        int source;
        int dest;
        CONFIGLOC_t configloc;
    }CONFIGLOCRESP_t;

    typedef struct
    {
        int type;
        int ID;
        int source;
        int dest;
        COMMSTATS_t commstats;
    }COMMSTATSRESP_t;
    
    typedef struct
    {
        int type;
        int ID;
        int source;
        int dest;
        uint16_t left;
        uint16_t right;
        uint16_t center;
        int distance;
        int theta;
        int direction;
    }SENSORVALRESP_t;
    
    typedef struct
    {
        int type;
        int ID;
        int source;
        int dest;
        TARGETERR_t targeterr;
    }TARGETERRRESP_t;

    typedef union
    {
        NODERESP_t node;
        CONFIGLOCRESP_t configloc;
        COMMSTATSRESP_t commstats;
        CREDITRESP_t credit;
        SENSORVALRESP_t sensorval;
        TARGETERRRESP_t targeterr;
    }RESPONSE_t;

#ifdef  __cplusplus
}
#endif

#endif  /* DATASTRUCTURES_H */

