#ifndef UNIWHEEL_MAEVARM_M2_STATE_ESTIMATION
#define UNIWHEEL_MAEVARM_M2_STATE_ESTIMATION
/**************************************************************************
 *
 *  Header Files to include
 *
 **************************************************************************/
/* subystem header files */
#include "mpu9250.h"
#include "state_estimation.h"
#include "matrix_calculations.h"

/* global program files */
#include "config.h"
#include "m_general.h"
#include "m_usb.h"

/* External header files */
#include <math.h>

/**************************************************************************
 *
 *  Function declarations
 *
 **************************************************************************/
float estimate_theta(int16_t *imu_data, uint16_t accel_scaling, uint16_t gyro_scaling, float prev_theta);
float theta_estimate_accel(int16_t *imu_data, uint16_t scaling);
float theta_estimate_gyro(int16_t *imu_data, uint16_t scaling, float theta);
float ApproxAtan(float z);
float ApproxAtan2(float y, float x);

#endif
