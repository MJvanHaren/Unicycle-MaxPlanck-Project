#ifndef IMUTEST_CONFIG
#define IMUTEST_CONFIG
/**************************************************************************
 *
 *  SETTINGS
 *
 **************************************************************************/
#define INIT_WAIT 100           // ms to wait after init. each IMU
#define LOOP_FREQ 200           // desired polling freq [Hz] of IMUs
#define CALIB_NUM_SAMPLES 20000 // amount of samples used for calibrating gyro + accelerometer.
#define NUM_SENSORS 2           // number of sensors
#define IMU_PIN1 PIN_D1         // pin of IMU 1 (closest to hinge)
#define IMU_PIN2 PIN_D2         // pin of IMU 2 (furthest to hinge)

#define IMU1_Y -0.0775 // length in m to hinge for IMU 1
#define IMU2_Y -0.168  // length in m to hinge for IMU 2
#define IMU1_X 0.0     // length in m to hinge for IMU 1
#define IMU2_X 0.020   // length in m to hinge for IMU 2

#define MOTOR_KV (190.0)                      // kv of motor
#define MOTOR_V 22.2                       // nominal volt of motor
#define MOTOR_TORQUE_CONSTANT (0.050259456) // kt of motor
#define FUSION_TUNING_PARAMETER 0.05      // tuning para
#define MD PIN_D5

// ranges for gyro and accel
#define accel_r ACCEL_2G
#define gyro_r GYRO_250DPS

// filters for gyro and accel
#define accel_f ACC_LPF_5HZ
#define gyro_f GY_LPF_41HZ

// protractor angle for bias removal estimator [mdeg]
#define PROTRACTOR_ANGLE 20000

#if PROTRACTOR_ANGLE == 20000
#define MAX_CURRENT_MOTOR (0.75 / MOTOR_TORQUE_CONSTANT)               // max current sent to motor
#elif PROTRACTOR_ANGLE == 30000
#define MAX_CURRENT_MOTOR (1.25 / MOTOR_TORQUE_CONSTANT)               // max current sent to motor
#else
#define MAX_CURRENT_MOTOR 4
#endif
/**************************************************************************
 *
 *  Constants
 *
 **************************************************************************/
#define F_CPU 16000000 // freq of CPU
#define GRAV 9.81
#define PI 3.14159265358979
#define PI_2 1.57079632679 // pi/2
#define STATIC_INIT_VALUE 70000
#define MD_PACKET_SIZE 34

#endif // IMUTEST_CONFIG_H
