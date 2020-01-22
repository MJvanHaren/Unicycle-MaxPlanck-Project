#ifndef UNIWHEEL_MAEVARM_M2_MPU9250
#define UNIWHEEL_MAEVARM_M2_MPU9250
/**************************************************************************
 *
 *  Header Files to include
 *
 **************************************************************************/
/* subystem header files */
#include "m_spi.h"
#include "mpu9250.h"

/* global program files */
#include "m_general.h"
#include "config.h"
#include "m_usb.h"

/* External header files */
#include <stdlib.h>
#include <math.h>

/**************************************************************************
 *
 *  typedef and enumerates
 *
 **************************************************************************/
typedef enum {
  ACC_LPF_1046HZ = 8,
  ACC_LPF_420HZ = 7,
  ACC_LPF_218HZ_B = 0,
  ACC_LPF_218HZ = 1,
  ACC_LPF_99HZ = 2,
  ACC_LPF_45HZ = 3,
  ACC_LPF_21HZ = 4,
  ACC_LPF_10HZ = 5,
  ACC_LPF_5HZ = 6
} lpf_accel_bw_t;

typedef enum {
  ACCEL_2G = (0b00 << 3),
  ACCEL_4G = (0b01 << 3),
  ACCEL_8G = (0b10 << 3),
  ACCEL_16G = (0b11 << 3)
} a_range_t;

// Typedef for adjusting the gyroscope ranges. Editing values in register
typedef enum {
  GYRO_250DPS = (0b00 << 3),
  GYRO_500DPS = (0b01 << 3),
  GYRO_1000DPS = (0b10 << 3),
  GYRO_2000DPS = (0b11 << 3)
} g_range_t;

typedef enum {
  GY_LPF_8800HZ = 9,
  GY_LPF_3600HZ_HISPD = 8,
  GY_LPF_250HZ = 0,
  GY_LPF_184HZ = 1,
  GY_LPF_92HZ = 2,
  GY_LPF_41HZ = 3,
  GY_LPF_20HZ = 4,
  GY_LPF_10HZ = 5,
  GY_LPF_5HZ = 6,
  GY_LPF_3600HZ = 7
} lpf_gyro_bw_t;

/**************************************************************************
 *
 *  Function declarations
 *
 **************************************************************************/
bool mpu9250_init(m2_gpio_t cs_pin);
void mpu9250_get_raw_6dof(m2_gpio_t cs_pin, int16_t *buffer);
void mpu9250_set_accel_filter(m2_gpio_t cs_pin);
void mpu9250_set_gyro_filter(m2_gpio_t cs_pin);
int mpu9250_calib_gyro(m2_gpio_t cs_pin);
void mpu9250_calib_accel(m2_gpio_t *cs_pin, int *accel_adjust, uint16_t accel_scaling);
void mpu9250_get_raw_3dof(m2_gpio_t cs_pin, int16_t *buffer);

/**************************************************************************
 *
 *  Register defintions
 *
 **************************************************************************/
// MPU9250
#define WHO_AM_I 0x75
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D

// VALUES
#define WHOAMI_VALUE 0x71
#define INIT_WAIT_REG 15

#endif
