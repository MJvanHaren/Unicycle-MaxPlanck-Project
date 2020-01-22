#ifndef UNIWHEEL_MAEVARM_M2_CONTROL
#define UNIWHEEL_MAEVARM_M2_CONTROL
/**************************************************************************
 *
 *  Header Files to include
 *
 **************************************************************************/
/* subystem header files */
#include "matrix_calculations.h"

/* global program files */
#include "m_general.h"
#include "config.h"
#include "m_usb.h"

/* External header files */
#include <math.h>

/**************************************************************************
 *
 *  Function declarations
 *
 **************************************************************************/
float saturate_torque(float torque, float omega, float dCurrent);
float return_current(float theta, float theta_dot, float omega, uint32_t counter);

#endif //UNIWHEEL_MAEVARM_M2_CONTROL