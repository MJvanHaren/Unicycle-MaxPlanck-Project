#ifndef UNIWHEEL_MAEVARM_M2_UDRIVER
#define UNIWHEEL_MAEVARM_M2_UDRIVER
/**************************************************************************
 *
 *  Header Files to include
 *
 **************************************************************************/
/* subystem header files */
#include "m_crc32.h"
#include "m_spi.h"

/* global program files */
#include "m_general.h"
#include "m_usb.h"
#include "config.h"

/* External header files */
#include <math.h>

/**************************************************************************
 *
 *  typedef and enumerates
 *
 **************************************************************************/
enum
{
    SYSTEM_ENABLE = 0x8000,
    MOTOR_1_ENABLE = 0x4000,
    MOTOR_2_ENABLE = 0x2000,
    ROLLOVER_ENABLE = 0x1000,
    MOTOR_1_INDEX_COMPENSATION_ENABLE = 0x0800,
    MOTOR_2_INDEX_COMPENSATION_ENABLE = 0x0400,
    SYSTEM_DISABLE = ~0x8000,
    MOTOR_1_DISABLE = ~0x4000,
    MOTOR_2_DISABLE = ~0x2000,
    ROLLOVER_DISABLE = ~0x1000,
    MOTOR_1_INDEX_COMPENSATION_DISABLE = ~0x0800,
    MOTOR_2_INDEX_COMPENSATION_DISABLE = ~0x0400,
};

typedef struct Commstruct
{
    float currentLimit1;
    float currentTargetMotor1;
    float velocityMotor1;
    bool motor1Ready;
    bool systemEnabled;
    uint8_t verbose;
} Commstruct;

/**************************************************************************
 *
 *  Function declarations
 *
 **************************************************************************/
void setMode(uint16_t setting);
bool send_spi_udriver(Commstruct incoming_comm_struct);
void printState();
Commstruct sendCommStruct();

uint16_t flipu16(uint16_t val);
int16_t flip16(int16_t val);
uint32_t flipu32(uint32_t val);
int32_t flip32(int32_t val);

#endif