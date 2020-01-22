/**************************************************************************
 *
 *  Header Files to include
 *
 **************************************************************************/
/* subystem header files */
#include "control.h"
#include "m_spi.h"
#include "matrix_calculations.h"
#include "mpu9250.h"
#include "state_estimation.h"
#include "udriver.h"

/* global program files */
#include "config.h"
#include "m_general.h"
#include "m_usb.h"

/* External header files */
#include <avr/wdt.h> // Watchdog

/**************************************************************************
 *
 *  Function declarations
 *
 **************************************************************************/
// setup timer for main loop execution
void setup_timer();
// sets data in correct format for post-processing
uint16_t select_timer1_prescaler();
// returns and prints microseconds since this function is last called
uint16_t micros(bool stream);
// function to wait for screen connection or imu installation
void wait_for_screen();
void wait_for_confirmation();
// function to calc initial angle of pendulum used in gyroscope integration
float calc_initial_angle();
// function to initialize and calibrate BLDC motor
void init_calibrate_udriver();

/**************************************************************************
 *
 *  Variables - these are the only non-stack RAM usage
 *
 **************************************************************************/
volatile bool tick = FALSE; // ISR tick when loop is executed
uint8_t cycle_overflow_counter = 0;
uint32_t counter = 0; // counter used to count amount of loops done

bool imu_init_status;                   // bool used for checking IMU initialization
int16_t imu_data[NUM_SENSORS][3] = {0}; // mulidimensional array for data
int gyro_adjust[NUM_SENSORS] = {0};     // bias adjust for gyros
//int accel_adjust[NUM_SENSORS][2] = {0}; // bias adjust for accel
int accel_adjust[NUM_SENSORS][2] = {{112, 1162}, {303, 196}}; // bias adjust for accel (12-11-2019)
uint16_t accel_scaling, gyro_scaling;                         // scaling factors for bytes->dps or m/s^2
m2_gpio_t imu_list[NUM_SENSORS] = {IMU_PIN1, IMU_PIN2};       // Chip select pins for each sensor
bool accel_calibrate = FALSE;                                 // bool used for accel calibration.

float theta, omega, theta_dot; // states of the 2D pendulum

char incoming;      // character used for keyboard input
bool stream = TRUE; // Start streaming input

Commstruct outgoing_comm = {0}; // struct used for communicating with udriver
uint8_t udriver_state = 0;
bool transmitOK;

/**************************************************************************
 *
 *  Main function
 *
 **************************************************************************/

int main() {

  m_clockdivide(0); // Do not lower CPU clock speed
  m_usb_init();     // Init. USB streaming library

  m_spi_init();            // initialize SPI library
  m_spi_speed(SPI_125KHZ); // slower SPI speed for initialization

  //   wait for screen to connect. can be commented if not needed
  wait_for_screen();

start:
  for (int i = 0; i < NUM_SENSORS; ++i) {
    // initialize each IMU with same settings
    imu_init_status = mpu9250_init(imu_list[i]);

    // visually show init. state with LEDs
    if (imu_init_status) {
      m_green(ON);
      m_red(ON);
    } else {
      m_red(ON);
      m_usb_tx_string("cannot connect to IMU(s), going back to init IMU \n");
      m_wait(10 * INIT_WAIT);
      goto start;
    }
    m_wait(INIT_WAIT);
    m_red(OFF);
    m_green(OFF);
  }

  switch (accel_r) {
  case ACCEL_2G:
    accel_scaling = 4;
    break;
  case ACCEL_4G:
    accel_scaling = 8;
    break;
  case ACCEL_8G:
    accel_scaling = 16;
    break;
  case ACCEL_16G:
    accel_scaling = 32;
    break;
  default:
    m_usb_tx_string("ERROR: no correct accel. range set \n");
    break;
  }
  switch (gyro_r) {
  case GYRO_250DPS:
    gyro_scaling = 500;
    break;
  case GYRO_500DPS:
    gyro_scaling = 1000;
    break;
  case GYRO_1000DPS:
    gyro_scaling = 2000;
    break;
  case GYRO_2000DPS:
    gyro_scaling = 4000;
    break;
  default:
    m_usb_tx_string("ERROR: no correct gyro. range set \n");
    break;
  }

  // SPI setup for udriver
  m_spi_cs_setup(MD);              // setup udriver pin for SPI
  setMode(SYSTEM_DISABLE);         // first disable system.
  outgoing_comm.currentLimit1 = MAX_CURRENT_MOTOR; // set current limits
  send_spi_udriver(outgoing_comm); // execute set commands.

  setup_timer(); // setup timer for desired frequency to poll IMUs

  m_usb_tx_string("Calibrating motor\n");
  m_usb_tx_push();
  init_calibrate_udriver(); // initialize and calibrate BLDC motor

  m_spi_speed(SPI_4MHZ); // SPI speed reset

  // measure accelerometer bias. can be stored in hardware for the future
  if (accel_calibrate) {
    mpu9250_calib_accel(imu_list, &accel_adjust[0][0], accel_scaling);
    wait_for_confirmation();
  }

  // measure gyroscope bias
  for (int i = 0; i < NUM_SENSORS; ++i) {
    gyro_adjust[i] = mpu9250_calib_gyro(imu_list[i]);
  }

  theta = calc_initial_angle(); // calculate initial angle of pendulum



  while (1) {

    if (tick) {
      counter++;
      micros(FALSE);
      tick = FALSE;

      // get IMU data, remove gyro/accel bias
      for (int i = 0; i < NUM_SENSORS; ++i) {
        mpu9250_get_raw_3dof(imu_list[i], imu_data[i]);
        imu_data[i][0] -= accel_adjust[i][0];
        imu_data[i][1] -= accel_adjust[i][1];
        imu_data[i][2] -= gyro_adjust[i];
      }

      // estimate angle using accel. and gyro data + ullllse scalings
      theta = estimate_theta(&imu_data[0][0], accel_scaling * GRAV, gyro_scaling, theta);
      // get pendulum speed
      theta_dot = 0.0;
      for (int i = 0; i < NUM_SENSORS; i++) {
        theta_dot -= ((float)gyro_scaling) / 65535.0 / 180.0 * PI * (imu_data[i][2]);
      }
      theta_dot /= (float)NUM_SENSORS;

      // get current wheel velocity of udriversudo
      omega = sendCommStruct().velocityMotor1;

      // get current from states
      outgoing_comm.currentTargetMotor1 = return_current(theta, theta_dot, omega, counter);
      if (stream) {
        m_usb_tx_long((theta / 2.0 / PI * 360.0*1000.0));
        m_usb_tx_string("\t");
        m_usb_tx_long(theta_dot / 2.0 / PI * 360.0*1000.0);
        m_usb_tx_string("\t");
        m_usb_tx_long(omega / 2.0 / PI * 360.0*1000.0);
        m_usb_tx_string("\t");
        m_usb_tx_long(outgoing_comm.currentTargetMotor1*1000.0);
      }

      // send package to udriver
      if (accel_calibrate ==  TRUE) {
          outgoing_comm.currentTargetMotor1 = 0.0;
      } else {
          //outgoing_comm.currentTargetMotor1 = 0.0;
          transmitOK = send_spi_udriver(outgoing_comm); // Line that sends current to motor
      }
      micros(TRUE);
      m_usb_tx_string("\n");
      m_usb_tx_push();
    }
  }
}

/**************************************************************************
 *
 *  Interrupt Service routines
 *
 **************************************************************************/
ISR(TIMER1_COMPA_vect) // fires approximately every 1/LOOP_FREQ [s]
{
  tick = TRUE;
}

ISR(TIMER3_OVF_vect) { cycle_overflow_counter++; }

/**************************************************************************
 *
 *  Functions
 *
 **************************************************************************/

//! \brief setup timers for compare match interrupt and cycle clock timer
void setup_timer() {
  cli(); // disable interrupts

  TCCR1A = 0; // Reset timer registers and timer itself
  TCCR1B = 0; // ^
  TCNT1 = 0;  // ^

  // Compare match register, 16MHz / PRESCALER / DES. IMU FREQ.
  uint16_t prescaler = select_timer1_prescaler(); // select correct prescaler for timer
  OCR1A = F_CPU / prescaler / LOOP_FREQ;

  // PWM (Fast PWM, top = OCR1A, update OCR1 at TOP, flag set on TOP)
  set(TCCR1B, WGM13); // ^
  set(TCCR1B, WGM12); // ^
  set(TCCR1A, WGM11); // ^
  set(TCCR1A, WGM10); // ^

  set(TIMSK1, OCIE1A); // Enable interrupts on timers

  // clock used to calculate cycle time, for micros function
  set(TCCR3B, CS30);  // prescale 1
  set(TIMSK3, TOIE3); // interrupt on overflow.

  sei(); // enable interrupts
} //! \end of setup_timer function

//! \brief Selects correct CPU prescaler for timer needed. also sets registers on 32u4
//! \return 16 bit unsigned value of prescaler. has to be used in OCRxx for correct freq.
uint16_t select_timer1_prescaler() {
  long int fcpu = F_CPU;
  long int loop_freq = LOOP_FREQ;
  if ((65534) > (fcpu / loop_freq)) {
    set(TCCR1B, CS10);
    m_usb_tx_string("Set prescaler:\t");
    m_usb_tx_int(1);
    m_usb_tx_string("\n");
    return 1;
  } else if (((65534 * 8) > (fcpu / loop_freq)) && ((65534) <= (fcpu / loop_freq))) {
    set(TCCR1B, CS11);
    m_usb_tx_string("Set prescaler:\t");
    m_usb_tx_int(8);
    m_usb_tx_string("\n");
    return 8;
  } else if (((65534 * 64) > (fcpu / loop_freq)) && ((65334 * 8) <= (fcpu / loop_freq))) {
    set(TCCR1B, CS11);
    set(TCCR1B, CS10);
    m_usb_tx_string("Set prescaler:\t");
    m_usb_tx_int(64);
    m_usb_tx_string("\n");
    return 64;
  } else if (((65334 * 256) > (fcpu / loop_freq)) && ((65534 * 64) <= (fcpu / loop_freq))) {
    set(TCCR1B, CS12);
    m_usb_tx_string("Set prescaler:\t");
    m_usb_tx_int(256);
    m_usb_tx_string("\n");
    return 256;
  } else if (((65534 * 1024) > (fcpu / loop_freq)) && ((65534 * 256) <= (fcpu / loop_freq))) {
    set(TCCR1B, CS12);
    set(TCCR1B, CS11);
    set(TCCR1B, CS10);
    m_usb_tx_string("Set prescaler:\t");
    m_usb_tx_int(1024);
    m_usb_tx_string("\n");
    return 1024;
  } else {
    m_usb_tx_string("cannot find correct presaler");
    return 0;
  }
} //! \end of select_timer1_prescaler function

//! \brief function to check loop duration in us
//! \param[in] stream, bool, if true, this will print the cycle time, otherwise not
//! \return amount of us the last time the function is called has lapsed
uint16_t micros(bool stream) {
  uint16_t val = 0;
  val = (float)(((long)cycle_overflow_counter * 65535) + (((long)TCNT3H) << 8) + ((long)TCNT3L)) /
        16; // this must be done twice somehow
  val = (float)(((long)cycle_overflow_counter * 65535) + (((long)TCNT3H) << 8) + ((long)TCNT3L)) / 16;
  cycle_overflow_counter = 0;
  TCNT3H = 0x00;
  TCNT3L = 0x00;
  if (stream) {
    m_usb_tx_string("\t");
    m_usb_tx_uint(val);
  }
  return val;
} //! \end of micros function

//! \brief script does not execute until screen ("sudo screen /dev/ttyACMx") is connected and "c" is pressed
void wait_for_screen() {
  while (1) {
    if (m_usb_rx_available()) {
      incoming = m_usb_rx_char();
      switch (incoming) {
      case 'c':
        m_usb_tx_string("pressed.\n");
        m_usb_tx_string("fusion tuning/loop freq:");
        m_usb_tx_string("\t");
        m_usb_tx_int(FUSION_TUNING_PARAMETER * 1000);
        m_usb_tx_string("\t");
        m_usb_tx_long(LOOP_FREQ);
        m_usb_tx_string("\n");
        return;
      default:
        m_usb_tx_string("waiting for conn (c).\n");
        m_usb_tx_push();
        m_wait(10);
        break;
      }
    }
  }
} //! \end of wait_for_screen function

//! \brief waits for confirmation of user. Need to press "s"  to continue
void wait_for_confirmation() {
  m_usb_tx_string("\n waiting for confirmation (s)\n");
  m_usb_tx_push();
  while (1) {
    if (m_usb_rx_available()) {
      incoming = m_usb_rx_char();
      switch (incoming) {
      case 's':
        m_usb_tx_string("continue \n");
        return;
      default:
        m_wait(10);
        break;
      }
    }
  }
} //! \end of wait_for confirmation function

//! \brief function to enable, calibrate and wait for motor 1 to be ready.
void init_calibrate_udriver() {
  while (1) {
    if (tick) {
      tick = FALSE;
      switch (udriver_state) {
      case 0: // enable system
        setMode(SYSTEM_ENABLE);
        if (sendCommStruct().systemEnabled) {
          m_usb_tx_string("System enabled\n");
          udriver_state++;
        }
        break;
      case 1: // enable motor
        setMode(MOTOR_1_ENABLE);
        if (sendCommStruct().motor1Ready) {
          m_usb_tx_string("Motor 1 ready \n");
          outgoing_comm.currentLimit1 = MAX_CURRENT_MOTOR;
          udriver_state++;
        }
        break;
      case 2: // send current
        outgoing_comm.currentTargetMotor1 = 0;
        transmitOK = send_spi_udriver(outgoing_comm); // send stuff
        return;
      }

      transmitOK = send_spi_udriver(outgoing_comm); // send stuff
    }
  }
} //! \end of init_calibrate_udriver function

//! \brief calculate initial angle of pendulum only based on accelerometer data. useful for gyro integration
//! \return float value of angle of pendulum, averaged over CALIB_NUM_SAMPLES
float calc_initial_angle() {
  float avg_theta = 0;
  m_usb_tx_string("Initial pendulum angle correct in config?\n");
  m_usb_tx_string("pendulum value [mdeg]:\t");
  m_usb_tx_long(PROTRACTOR_ANGLE);
  wait_for_confirmation();

  for (int i = 0; i < CALIB_NUM_SAMPLES; i++) {
    for (int i = 0; i < NUM_SENSORS; ++i) {
      mpu9250_get_raw_3dof(imu_list[i], imu_data[i]);
      imu_data[i][0] -= accel_adjust[i][0];
      imu_data[i][1] -= accel_adjust[i][1];
    }
    avg_theta += theta_estimate_accel(&imu_data[0][0], accel_scaling * GRAV) / ((float)(CALIB_NUM_SAMPLES));
  }

  m_usb_tx_string("\n est. avg. theta [mdeg]:\t");
  m_usb_tx_long((long)(1000 * avg_theta / 2.0 / PI * 360));
  wait_for_confirmation();
  return avg_theta;
} //! \end of calc_initial angle function
