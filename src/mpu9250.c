#include "mpu9250.h"
/**************************************************************************
 *
 *  Functions
 *
 **************************************************************************/

//! \brief Initializes one MPU9250 with parameters from config
//! \param[in] cs_pin, this is the pin (i.e. PIN_D1) where the desired IMU CS line is hooked up to
//! \return bool if the connection is successful
bool mpu9250_init(m2_gpio_t cs_pin) {

  spi_freq_t current_freq = _spi_freq; // Slow down SPI as needed
  if ((uint8_t)current_freq > (uint8_t)SPI_250KHZ) {
    m_spi_speed(SPI_250KHZ);
  }

  m_spi_cs_setup(cs_pin); // Deselect CS pin and make as output

  m_spi_write_register(cs_pin, PWR_MGMT_1, 0b10000000); // Reset chip
  m_wait(INIT_WAIT_REG);
  m_spi_write_register(cs_pin, PWR_MGMT_1, 0x01); // Automatically select best clock
  m_wait(INIT_WAIT_REG);
  m_spi_write_register(cs_pin, PWR_MGMT_2, 0x00); // Enable accel. and gyro.
  m_wait(INIT_WAIT_REG);

  // write acceleration range to sensor (set in config)
  m_spi_write_register(cs_pin, ACCEL_CONFIG, accel_r);

  // write gyroscope range to sensor (set in config)
  m_spi_write_register(cs_pin, GYRO_CONFIG, (m_spi_read_register(cs_pin, GYRO_CONFIG) | gyro_r));

  mpu9250_set_accel_filter(cs_pin); // set filter for accel.
  mpu9250_set_gyro_filter(cs_pin);  // set filter for gyro.

  m_spi_write_register(cs_pin, USER_CTRL, 0b00010000); // disable I2C
  m_wait(INIT_WAIT_REG);

  // check if connection is succesfull, if not return false
  if (m_spi_read_register(cs_pin, WHO_AM_I) != WHOAMI_VALUE) {
    return false;
  }

  if ((uint8_t)current_freq > (uint8_t)SPI_250KHZ) { // reset SPI speed again
    m_spi_speed(current_freq);
  }

  return true;
} //! \end of mpu9250_init function

//! \brief Reads 6 (3 x acc, 3 x gyro) 16-bit register from SPI device and set in buffer. use for 3D estimation
//! \param[in] cs_pin, Chip select pin of IMU, i.e. PIN_D1
//! \param[in] buffer, 16 bit signed pointer to array. There needs to be space for 6 16 bit values. Order: xdd, ydd,
//! zdd, gx, gy, gz
void mpu9250_get_raw_6dof(m2_gpio_t cs_pin, int16_t *buffer) {
  for (int i = 0; i < 3; ++i) {
    buffer[i] = (m_spi_read_register(cs_pin, ACCEL_XOUT_H + (i * 2)) << 8) |    // high byte
                (m_spi_read_register(cs_pin, ACCEL_XOUT_L + (i * 2)));          // low byte
  }
  for (int i = 3; i < 6; ++i) {
    buffer[i] = (m_spi_read_register(cs_pin, ACCEL_XOUT_H + ((i + 1) * 2)) << 8) |  // high byte
                (m_spi_read_register(cs_pin, ACCEL_XOUT_L + ((i + 1) * 2)));        // low byte
  }
} //! \end of mpu9250_get_raw_6dof function

//! \brief Reads 3 (2 x acc, 1 x gyro) 16-bit register from SPI device and set in buffer. use for 2D estimation
//! \param[in] cs_pin, Chip select pin of IMU, i.e. PIN_D1
//! \param[in] buffer, 16 bit signed pointer to array. There needs to be space for 3 16 bit values. Order: xdd, ydd, gz
void mpu9250_get_raw_3dof(m2_gpio_t cs_pin, int16_t *buffer) {
  int i;
  for (i = 0; i < 2; ++i) {
    buffer[i] = (m_spi_read_register(cs_pin, ACCEL_XOUT_H + (i * 2)) << 8) |    // high byte
                (m_spi_read_register(cs_pin, ACCEL_XOUT_L + (i * 2)));          // low byte
  }
  buffer[i] = ((m_spi_read_register(cs_pin, GYRO_ZOUT_H) << 8) | (m_spi_read_register(cs_pin, GYRO_ZOUT_L)));
} //! \end of mpu9250_get_raw_3dof function

//! \brief Set internal low pass filter (acceleration) of MPU9250. LPF is defined (preferable in config)
//! \param[in] cs_pin, Chip select pin of IMU, i.e. PIN_D1
void mpu9250_set_accel_filter(m2_gpio_t cs_pin) {
  if (accel_f == ACC_LPF_1046HZ) {
    m_spi_write_register(cs_pin, ACCEL_CONFIG2, (accel_f & 0b00001111));
  } else {
    m_spi_write_register(cs_pin, ACCEL_CONFIG2, ((accel_f)&0b00000111));
  }
} //! \end of mpu9250_set_accel_filter function

//! \brief Set internal low pass filter (gyroscope) of MPU9250. LPF is defined (preferable in config)
//! \param[in] cs_pin, Chip select pin of IMU, i.e. PIN_D1
void mpu9250_set_gyro_filter(m2_gpio_t cs_pin) {
  m_spi_write_register(cs_pin, CONFIG, (gyro_f & 0b00000111));
  m_wait(INIT_WAIT_REG);
  if (gyro_f == GY_LPF_3600HZ_HISPD) {
    m_spi_write_register(cs_pin, GYRO_CONFIG, (m_spi_read_register(cs_pin, GYRO_CONFIG) | 0b00000010));
  } else if (gyro_f == GY_LPF_8800HZ) {
    m_spi_write_register(cs_pin, GYRO_CONFIG, (m_spi_read_register(cs_pin, GYRO_CONFIG) | 0b00000011));
  }
} //! End of mpu9250_init function

//! \brief remove bias/calibrate gyroscope by sampling. gyroscope need to be still for this.
//! \param[in] cs_pin, Chip select pin of IMU, i.e. PIN_D1
//! \return bias of sensor as signed 16 bit integer
int mpu9250_calib_gyro(m2_gpio_t cs_pin) {
  float gyro_calib = 0;
  for (int i = 0; i < CALIB_NUM_SAMPLES; i++) {
    gyro_calib +=
        ((float)((m_spi_read_register(cs_pin, GYRO_ZOUT_H) << 8) | (m_spi_read_register(cs_pin, GYRO_ZOUT_L))) /
         (float)CALIB_NUM_SAMPLES);
  }
  return (int)(gyro_calib);
} //! \end of mpu9250_calib_gyro function

//! \brief remove bias/calibrate accelerometer by sampling. Accelerometer needs to be still, in a known position for
//! this. Uses the known position to calculate the accelerations the sensors should read, then calculate bias from
//! average reading.
//! \param[in] cs_pin, Chip select pin of IMU, i.e. PIN_D1
//! \param[in] accel_adjust 16 bit signed pointer to store the bias of the gyro
//! \param[in] accel_scaling scaling of the accelerometer (i.e. +/-2G = 4*9.81)
void mpu9250_calib_accel(m2_gpio_t *cs_pin, int *accel_adjust, uint16_t accel_scaling) {
  float accel_calib = 0;
  int ddot_ref[2] = {0}; // reference value for accelerations based on protractor angle.
  ddot_ref[0] =
      (int)((sin((float)PROTRACTOR_ANGLE / 1000.0 / 360.0 * 2.0 * (float)PI)) * 65536.0 / (float)accel_scaling); // x
  ddot_ref[1] =
      (int)((-cos((float)PROTRACTOR_ANGLE / 1000.0 / 360.0 * 2.0 * (float)PI)) * 65536.0 / (float)accel_scaling); // y
  for (int k = 0; k < NUM_SENSORS; k++) { // loop over all sensors
    for (int j = 0; j < 2; j++) {         // loop over all directions (x,y)
      accel_calib = 0;
      for (int i = 0; i < CALIB_NUM_SAMPLES; i++) {
        accel_calib += ((float)((m_spi_read_register(*(cs_pin + k), ACCEL_XOUT_H + j * 2) << 8) |
                                (m_spi_read_register(*(cs_pin + k), ACCEL_XOUT_L + j * 2))) /
                        (float)CALIB_NUM_SAMPLES);
      }
      (*(accel_adjust + k * 2 + j)) = (int)accel_calib - ddot_ref[j];
      m_usb_tx_int(accel_calib);
      m_usb_tx_string("\t");
      m_usb_tx_int(ddot_ref[j]);
      m_usb_tx_string("\t");
      m_usb_tx_int((*(accel_adjust + k * 2 + j)));
      m_usb_tx_string("\n");
      m_usb_tx_push();
    }
  }
  m_usb_tx_string("\n");
} //! \end of mpu9250_calib_accel function