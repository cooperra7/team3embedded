#include "debug.h"

//Controls debug pins
//also contains error handler function


//Takes the provided pin information and direction and sets or clears the pin accordingly
void pinSetOrClearPin(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos, bool dir)
{
    if (dir){
        PLIB_PORTS_PinSet(index, channel, bitPos);
    }
    else{
        PLIB_PORTS_PinClear(index, channel, bitPos);
    }
}

void enableDbgVal()
{
    PLIB_PORTS_PinOpenDrainEnable(CHIPKIT_46);
    PLIB_PORTS_PinWrite(CHIPKIT_46, 0);
    PLIB_PORTS_PinDirectionOutputSet(CHIPKIT_46);
    PLIB_PORTS_PinOpenDrainEnable(CHIPKIT_47);
    PLIB_PORTS_PinWrite(CHIPKIT_47, 0);
    PLIB_PORTS_PinDirectionOutputSet(CHIPKIT_47);
    PLIB_PORTS_PinOpenDrainEnable(CHIPKIT_48);
    PLIB_PORTS_PinWrite(CHIPKIT_48, 0);
    PLIB_PORTS_PinDirectionOutputSet(CHIPKIT_48);
    PLIB_PORTS_PinOpenDrainEnable(CHIPKIT_49);
    PLIB_PORTS_PinWrite(CHIPKIT_49, 0);
    PLIB_PORTS_PinDirectionOutputSet(CHIPKIT_49);
    PLIB_PORTS_PinOpenDrainEnable(CHIPKIT_50);
    PLIB_PORTS_PinWrite(CHIPKIT_50, 0);
    PLIB_PORTS_PinDirectionOutputSet(CHIPKIT_50);
    PLIB_PORTS_PinOpenDrainEnable(CHIPKIT_51);
    PLIB_PORTS_PinWrite(CHIPKIT_51, 0);
    PLIB_PORTS_PinDirectionOutputSet(CHIPKIT_51);
    PLIB_PORTS_PinOpenDrainEnable(CHIPKIT_52);
    PLIB_PORTS_PinWrite(CHIPKIT_52, 0);
    PLIB_PORTS_PinDirectionOutputSet(CHIPKIT_52);
    PLIB_PORTS_PinOpenDrainEnable(CHIPKIT_53);
    PLIB_PORTS_PinWrite(CHIPKIT_53, 0);
    PLIB_PORTS_PinDirectionOutputSet(CHIPKIT_53);
}
#define SYS_PORT_E_TRIS         0x300
#define SYS_PORT_E_LAT          0x0
#define SYS_PORT_E_ODC          0x0

void enable_dbgOutputLoc()
{
    PLIB_PORTS_OpenDrainEnable(PORTS_ID_0, PORT_CHANNEL_E, SYS_PORT_E_ODC);
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_E,  SYS_PORT_E_LAT);
	PLIB_PORTS_DirectionOutputSet( PORTS_ID_0, PORT_CHANNEL_E,  SYS_PORT_E_TRIS ^ 0xFFFF);
}

//sets chipkit pins 46-53 to the provided value
void dbgOutputVal(unsigned char outVal)
{
    pinSetOrClearPin(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_1, ((outVal >> 0) & 0x01));
    pinSetOrClearPin(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_6, ((outVal >> 1) & 0x01));
    pinSetOrClearPin(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_8, ((outVal >> 2) & 0x01));
    pinSetOrClearPin(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11, ((outVal >> 3) & 0x01));
    pinSetOrClearPin(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_7, ((outVal >> 4) & 0x01));
    pinSetOrClearPin(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8, ((outVal >> 5) & 0x01));
    pinSetOrClearPin(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_6, ((outVal >> 6) & 0x01));
    pinSetOrClearPin(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_9, ((outVal >> 7) & 0x01));
}

//sets chipkit pins 30-37 to the provided value
void dbgOutputLoc(unsigned char outVal)
{
    pinSetOrClearPin(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_7, ((outVal >> 0) & 0x01));
    pinSetOrClearPin(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_6, ((outVal >> 1) & 0x01));
    pinSetOrClearPin(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_5, ((outVal >> 2) & 0x01));
    pinSetOrClearPin(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_4, ((outVal >> 3) & 0x01));
    pinSetOrClearPin(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_3, ((outVal >> 4) & 0x01));
    pinSetOrClearPin(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_2, ((outVal >> 5) & 0x01));
    pinSetOrClearPin(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_1, ((outVal >> 6) & 0x01));
    pinSetOrClearPin(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_0, ((outVal >> 7) & 0x01));
}

void errorHandler(uint8_t val)
{
    //dbgOutputLoc(0xFF);
    //SYS_DEBUG_BreakPoint();
    while(1)
        dbgOutputLoc(val);
}
