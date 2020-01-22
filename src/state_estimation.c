#include "state_estimation.h"
/**************************************************************************
 *
 *  Variables
 *
 **************************************************************************/

float sensor_pos_matrix[3][NUM_SENSORS] = {
    {1, 1}, {IMU1_X, IMU2_X}, {IMU1_Y, IMU2_Y}}; // Sensor placement (2D case, [m])
//! \attention the calculation of x1_star is done offline (in matlab). If the sensor position changes adjust this!
//float x1_star[NUM_SENSORS] = {1.906096, -0.906446}; // PP' + eye(3)*0.000000001 old?
float x1_star[NUM_SENSORS] =   {1.815917, -0.816162}; // PP' + eye(3)*0.000000000000001 CHECK!

/**************************************************************************
 *
 *  Functions
 *
 **************************************************************************/

//! \brief Estimates state of pendulum based on gyroscope and accelerometer data (2D!)
//! \param[in] imu_data, 16 bit pointer to start of array of size [NUM_SENSORS][3], containing IMU data (2D!)
//! \param[in] accel_scaling 16 bit signed value of acceleration scaling (i.e. +/-2G = 4*9.81)
//! \param[in] gyro_scaling 16 bit signed value of gyroscope scaling (i.e. +/-250DPS = 500)
//! \param[in] prev_theta, float value of previous angle used in trapezoid integration of gyroscope
//! \return float value which is the estimate of the current angle of the pendulum
float estimate_theta(int16_t *imu_data, uint16_t accel_scaling, uint16_t gyro_scaling, float prev_theta) {
  return (FUSION_TUNING_PARAMETER * (theta_estimate_accel(imu_data, accel_scaling)) +
          (1 - FUSION_TUNING_PARAMETER) * (theta_estimate_gyro(imu_data, gyro_scaling, prev_theta)));
} //! \end of estimate_theta function

//! \brief Estiamtes states of pendulum only based on accelerometers
//! \param[in] imu_data, 16 bit pointer to start of array of size [NUM_SENSORS][3], containing IMU data (2D!)
//! \param[in] scaling 16 bit signed value of acceleration scaling (i.e. +/-2G = 4*9.81)
float theta_estimate_accel(int16_t *imu_data, uint16_t scaling) {
  float M[2][NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) {
    for (int j = 0; j < 2; j++) {
      M[j][i] = (float)scaling / 65535 * (*(imu_data + i * 3 + j));
    }
  }
  float ghat_B[2];
  multiply_matrix_vector(2, NUM_SENSORS, M, x1_star, ghat_B);
  return ((ApproxAtan2(ghat_B[0], -ghat_B[1])));
} //! \end of theta_estimate_accel function

//! \brief Estimates angle of pendulum only based on gyroscope
//! \param[in] scaling 16 bit signed value of gyroscope scaling (i.e. +/-250DPS = 500)
//! \param[in] imu_data 16 bit pointer to start of array of size [NUM_SENSORS][3], containing IMU data (2D!)
//! \param[in] theta, float value of previous angle used in trapezoid integration of gyroscope
float theta_estimate_gyro(int16_t *imu_data, uint16_t scaling, float theta) {
  // static is initialized out of bounds, so integration is implemented correctly.
  static float prev_avg_gyro = STATIC_INIT_VALUE;
  float returnval;

  // average gyroscope measurements
  float avg_gyro = 0.0;
  for (int i = 0; i < NUM_SENSORS; ++i) {
    avg_gyro -= (float)scaling / 65535.0 / 180.0 * PI * (*(imu_data + i * 3 + 2));
  }
  avg_gyro /= NUM_SENSORS;

  // do not add area when time step = 0. For all other time steps, this will
  // never happen since the static init value is higher than a 16 bit value.
  // if it is the first time step, return initialized angle.
  if (prev_avg_gyro != STATIC_INIT_VALUE) {
    returnval = (0.5 * (prev_avg_gyro + avg_gyro) / LOOP_FREQ + theta);
    prev_avg_gyro = avg_gyro;
    return returnval;
  } else {
    prev_avg_gyro = avg_gyro;
    return theta;
  }
} //! \end of theta_estimate_gyro function

// Polynomial approximating arctangenet on the range -1,1.
// Max error < 0.005 (or 0.29 degrees)
float ApproxAtan(float z) {
  const float n1 = 0.97239411f;
  const float n2 = -0.19194795f;
  return (n1 + n2 * z * z) * z;
}

float ApproxAtan2(float y, float x) {
  if (x != 0.0f) {
    if (fabsf(x) > fabsf(y)) {
      const float z = y / x;
      if (x > 0.0) {
        // atan2(y,x) = atan(y/x) if x > 0
        return ApproxAtan(z);
      } else if (y >= 0.0) {
        // atan2(y,x) = atan(y/x) + PI if x < 0, y >= 0
        return ApproxAtan(z) + PI;
      } else {
        // atan2(y,x) = atan(y/x) - PI if x < 0, y < 0
        return ApproxAtan(z) - PI;
      }
    } else // Use property atan(y/x) = PI/2 - atan(x/y) if |y/x| > 1.
    {
      const float z = x / y;
      if (y > 0.0) {
        // atan2(y,x) = PI/2 - atan(x/y) if |y/x| > 1, y > 0
        return -ApproxAtan(z) + PI_2;
      } else {
        // atan2(y,x) = -PI/2 - atan(x/y) if |y/x| > 1, y < 0
        return -ApproxAtan(z) - PI_2;
      }
    }
  } else {
    if (y > 0.0f) // x = 0, y > 0
    {
      return PI_2;
    } else if (y < 0.0f) // x = 0, y < 0
    {
      return -PI_2;
    }
  }
  return 0.0f; // x,y = 0. Could return NaN instead.
}
