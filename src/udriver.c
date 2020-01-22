#include "udriver.h"
/**************************************************************************
 *
 *  Variables
 *
 **************************************************************************/

uint8_t mosi[MD_PACKET_SIZE] = {0};
uint8_t miso[MD_PACKET_SIZE] = {0};

uint16_t desiredMode = 0;
uint8_t crc_check[4];
uint32_t *crc_check_ptr = (uint32_t *)&crc_check[0];

float positionMotor1;
float velocityMotor1;
float currentMotor1;
float currentTargetMotor1 = 0;
float ADCMotor1;
float currentLimit1;
bool systemEnabled;
bool motor1Enabled;
bool motor1Ready;
bool motor1IndexDetected;
bool motor1IndexToggle;

uint8_t errorCode;
bool crcOK = 0;
uint16_t boardStatus;
uint32_t boardCrc;
uint32_t checkCrc;
uint16_t timestamp;

uint8_t verbose = 0;
uint16_t communicationInputIndex;

// MOSI POINTERS
uint16_t *mode = (uint16_t *)&mosi[0];
int32_t *refPosition1 = (int32_t *)&mosi[2];
// int32_t* refPosition2 = (int32_t*)&mosi[6];
int16_t *refVelocity1 = (int16_t *)&mosi[10];
// int16_t* refVelocity2 = (int16_t*)&mosi[12];
int16_t *refCurrent1 = (int16_t *)&mosi[14];
// int16_t* refCurrent2 = (int16_t*)&mosi[16];
// uint16_t* kp1 = (uint16_t*)&mosi[18];
// uint16_t* kp2 = (uint16_t*)&mosi[20];
// uint16_t* kd1 = (uint16_t*)&mosi[22];
// uint16_t* kd2 = (uint16_t*)&mosi[24];
uint8_t *currentSaturation1 = (uint8_t *)&mosi[26];
// uint8_t* currentSaturation2 = (uint8_t*)&mosi[27];
uint16_t *sendIdx = (uint16_t *)&mosi[28];
uint32_t *sendCrc = (uint32_t *)&mosi[30];

// MISO POINTERS
uint16_t *status = (uint16_t *)&miso[0];
uint16_t *ts = (uint16_t *)&miso[2];
int32_t *mPositionMotor1 = (int32_t *)&miso[4];
// int32_t* mPositionMotor2 = (int32_t*)&miso[8];
int16_t *mVelocityMotor1 = (int16_t *)&miso[12];
// int16_t* mVelocityMotor2 = (int16_t*)&miso[14];
int16_t *mcurrentMotor1 = (int16_t *)&miso[16];
// int16_t* mcurrentMotor2 = (int16_t*)&miso[18];
uint16_t *r_a = (uint16_t *)&miso[20];
uint16_t *r_b = (uint16_t *)&miso[22];
uint16_t *mADCMotor1 = (uint16_t *)&miso[24];
// uint16_t* mADCMotor2 = (uint16_t*)&miso[26];
uint16_t *receiveIndex = (uint16_t *)&miso[28];
uint32_t *receiveCrc = (uint32_t *)&miso[30];

/**************************************************************************
 *
 *  Functions
 *
 **************************************************************************/

//! \brief Set mode of uDriver board, this is cumulative, except for system/motor disable. check
//! https://atlas.is.localnet/confluence/pages/viewpage.action?pageId=64260519
//! \param[in] setting, setting you want to send to the uDriver board
void setMode(uint16_t setting) {
  // set mode bits in human readable commands
  switch (setting) {
  case SYSTEM_ENABLE:
    desiredMode |= (uint16_t)SYSTEM_ENABLE;
    break;
  case SYSTEM_DISABLE:
    desiredMode &= (uint16_t)SYSTEM_DISABLE;
    break;
  case MOTOR_1_ENABLE:
    desiredMode |= (uint16_t)MOTOR_1_ENABLE;
    break;
  case MOTOR_1_DISABLE:
    desiredMode &= (uint16_t)MOTOR_1_DISABLE;
    break;
  case MOTOR_2_ENABLE:
    desiredMode |= (uint16_t)MOTOR_2_ENABLE;
    break;
  case MOTOR_2_DISABLE:
    desiredMode &= (uint16_t)MOTOR_2_DISABLE;
    break;
  case ROLLOVER_ENABLE:
    desiredMode |= (uint16_t)ROLLOVER_ENABLE;
    break;
  case ROLLOVER_DISABLE:
    desiredMode &= (uint16_t)ROLLOVER_DISABLE;
    break;
  case MOTOR_1_INDEX_COMPENSATION_ENABLE:
    desiredMode |= (uint16_t)MOTOR_1_INDEX_COMPENSATION_ENABLE;
    break;
  case MOTOR_1_INDEX_COMPENSATION_DISABLE:
    desiredMode &= (uint16_t)MOTOR_1_INDEX_COMPENSATION_DISABLE;
    break;
  case MOTOR_2_INDEX_COMPENSATION_ENABLE:
    desiredMode |= (uint16_t)MOTOR_2_INDEX_COMPENSATION_ENABLE;
    break;
  case MOTOR_2_INDEX_COMPENSATION_DISABLE:
    desiredMode &= (uint16_t)MOTOR_2_INDEX_COMPENSATION_DISABLE;
    break;
  default:                               // set communication timeout in ms
    if ((setting > 0) & (setting < 255)) // delay in ms
      desiredMode |= setting;
  }
} //! \end of setMode function

//! \brief exchange data/commands with uDriver board
//! \param[in] incoming_comm_struct, a struct of type "Commstruct". see udriver.h
//! \return bool if the transfer/CRC is successful
bool send_spi_udriver(Commstruct incoming_comm_struct) {
  // reset the outbound packet
  for (int i = 0; i < MD_PACKET_SIZE; i++) {
    mosi[i] = 0;
  }

  // set received commands
  currentLimit1 = incoming_comm_struct.currentLimit1;
  currentTargetMotor1 = incoming_comm_struct.currentTargetMotor1;
  verbose = incoming_comm_struct.verbose;

  // load mosi buffer
  *mode = flipu16(desiredMode);

  // set current in desired format, with correct resolution etc.
  *refCurrent1 = flip16((int16_t)((currentTargetMotor1) * (1 << 10)));
  *currentSaturation1 = (uint8_t)((currentLimit1) * (1 << 3));

  if (verbose) {
    m_usb_tx_string("Current pointer: ");
    m_usb_tx_int((int)(*refCurrent1));
    m_usb_tx_string("Current target casted ");
    m_usb_tx_int((int16_t)(currentTargetMotor1));
    m_usb_tx_string("\n");
  }

  // send SPI stuff and check incoming CRC
  *sendCrc = flipu32(m_crc32((uint8_t *)mosi, 30));
  m_spi_shift_MD(MD, (uint8_t *)mosi, (uint8_t *)miso, MD_PACKET_SIZE);
  *crc_check_ptr = flipu32(m_crc32((uint8_t *)miso, 30));
  crcOK = ((crc_check[0] == miso[32]) && (crc_check[1] == miso[33]) && (crc_check[2] == miso[30]) &&
           (crc_check[3] == miso[31]));

  if (crcOK) // write values to public variables
  {
    positionMotor1 = (2 * PI * ((float)(flip32(*mPositionMotor1)) / (float)((int32_t)1 << 24)));
    velocityMotor1 =
        ((float)(2.0 * PI * 1000.0 / 60.0 * ((double)(flip16(*mVelocityMotor1)) / (double)((int32_t)1 << 11))));
    currentMotor1 = (((float)(flip16(*mcurrentMotor1)) / (float)((int32_t)1 << 10)));
    ADCMotor1 = (int16_t)(3.3 * ((float)(flip16(*mADCMotor1)) / (float)((int32_t)1 << 16)));

    communicationInputIndex = flipu16(*receiveIndex);

    boardStatus = flipu16(*status);
    timestamp = flipu16(*ts);
    systemEnabled = (boardStatus >> 15) & 1;
    motor1Enabled = (boardStatus >> 14) & 1;
    motor1Ready = (boardStatus >> 13) & 1;
    motor1IndexDetected = (boardStatus >> 10) & 1;
    motor1IndexDetected = (boardStatus >> 8) & 1;

    errorCode = boardStatus & 0x07;
  }
  return crcOK;
} //! \end of send_spi_udriver function

//! \brief Prints state of uDriver as 17 hexchars. see
//! https://atlas.is.localnet/confluence/pages/viewpage.action?pageId=64260519
void printState() {
  m_usb_tx_string("\n Status: ");
  for (int i = 0; i < 34; i++) {
    m_usb_tx_hexchar(miso[i]);
    m_usb_tx_string(" ");
  }

  m_usb_tx_string("\n Command: ");
  for (int i = 0; i < 34; i++) {
    m_usb_tx_hexchar(mosi[i]);
    m_usb_tx_string(" ");
  }
  m_usb_tx_string("\n");
} //! \end of printState function

//! \brief return global variables of the udriver board
//! \return a struct of type "Commstruct" containing sent currents/liits/velocity etc.
Commstruct sendCommStruct() {
  Commstruct output_comm;
  output_comm.currentLimit1 = currentLimit1;
  output_comm.currentTargetMotor1 = currentTargetMotor1;
  output_comm.motor1Ready = motor1Ready;
  output_comm.systemEnabled = systemEnabled;
  output_comm.verbose = verbose;
  output_comm.velocityMotor1 = velocityMotor1;
  return output_comm;
} //! \end of sendCommStruct function

// switch bytes to low byte first
uint16_t flipu16(uint16_t val) { return (val << 8) | (val >> 8); }
// switch bytes to low byte first
int16_t flip16(int16_t val) { return (val << 8) | ((val >> 8) & 0xFF); }
// switch bytes to low byte first
uint32_t flipu32(uint32_t val) {
  val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
  return (val << 16) | (val >> 16);
}
// switch bytes to low byte first
int32_t flip32(int32_t val) {
  val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
  return (val << 16) | ((val >> 16) & 0xFFFF);
}
