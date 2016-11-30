
#include "debug.h"

void dbgOutputVal (unsigned char outVal)
{
    /* Output data over adjacent ports on the board */
    if (outVal & 0x01) {
        PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_13);
    }
    else {
        PLIB_PORTS_PinClear (PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_13);
    }
    if (outVal & 0x02) {
        PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_12);
    }
    else {
        PLIB_PORTS_PinClear (PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_12);
    }
    if (outVal & 0x04) {
        PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_14);
    }
    else {
        PLIB_PORTS_PinClear (PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_14);
    }
    if (outVal & 0x08) {
        PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_7);
    }
    else {
        PLIB_PORTS_PinClear (PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_7);
    }
    if (outVal & 0x10) {
        PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_6);
    }
    else {
        PLIB_PORTS_PinClear (PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_6);
    }
    if (outVal & 0x20) {
        PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_4);
    }
    else {
        PLIB_PORTS_PinClear (PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_4);
    }
    if (outVal & 0x40) {
        PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_4);
    }
    else {
        PLIB_PORTS_PinClear (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_4);
    }
    if (outVal & 0x80) {
        PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_2);
    }
    else {
        PLIB_PORTS_PinClear (PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_2);
    }
}

void dbgOutputLoc (unsigned char outVal)
{
    /* Write data to the E channel, which all appear adjacent to each other on the board */
    PLIB_PORTS_Write (PORTS_ID_0, PORT_CHANNEL_E, 0x00FF & outVal);
}

void error ()
{
    /* Output a debugging value and then spinlock */
    dbgOutputLoc (0xFF);
    while (1);
}