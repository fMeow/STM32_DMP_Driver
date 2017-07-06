// I2Cdev library collection - MPU6050 I2C device class, 6-axis MotionApps 2.0 implementation
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 5/20/2013 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     ... - ongoing debug release

/* ============================================
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2012 Jeff Rowberg

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ===============================================
 */

#include "MPU6050_6Axis_MotionApps20.h"

//// Tom Carpenter's conditional PROGMEM code
//// http://forum.arduino.cc/index.php?topic=129407.0
//#ifndef __arm__
//#include <avr/pgmspace.h>
//#else
//// Teensy 3.0 library conditional PROGMEM code from Paul Stoffregen
//#ifndef __PGMSPACE_H_
//#define __PGMSPACE_H_ 1
//#include <inttypes.h>
//
//#define PROGMEM
//#define PGM_P  const char *
//#define PSTR(str) (str)
//#define F(x) x
//
//typedef void prog_void;
//typedef char prog_char;
//typedef unsigned char prog_uchar;
//typedef int8_t prog_int8_t;
//typedef uint8_t prog_uint8_t;
//typedef int16_t prog_int16_t;
//typedef uint16_t prog_uint16_t;
//typedef int32_t prog_int32_t;
//typedef uint32_t prog_uint32_t;
//
//#define strcpy_P(dest, src) strcpy((dest), (src))
//#define strcat_P(dest, src) strcat((dest), (src))
//#define strcmp_P(a, b) strcmp((a), (b))
//
//#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
//#define pgm_read_word(addr) (*(const unsigned short *)(addr))
//#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
//#define pgm_read_float(addr) (*(const float *)(addr))
//
//#define pgm_read_byte_near(addr) pgm_read_byte(addr)
//#define pgm_read_word_near(addr) pgm_read_word(addr)
//#define pgm_read_dword_near(addr) pgm_read_dword(addr)
//#define pgm_read_float_near(addr) pgm_read_float(addr)
//#define pgm_read_byte_far(addr) pgm_read_byte(addr)
//#define pgm_read_word_far(addr) pgm_read_word(addr)
//#define pgm_read_dword_far(addr) pgm_read_dword(addr)
//#define pgm_read_float_far(addr) pgm_read_float(addr)
//#endif
//#endif

/* Source is from the InvenSense MotionApps v2 demo code. Original source is
 * unavailable, unless you happen to be amazing as decompiling binary by
 * hand (in which case, please contact me, and I'm totally serious).
 *
 * Also, I'd like to offer many, many thanks to Noah Zerkin for all of the
 * DMP reverse-engineering he did to help make this bit of wizardry
 * possible.
 */

// NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.
// Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)
// after moving string constants to flash memory storage using the F()
// compiler macro (Arduino IDE 1.0+ required).
//#define DEBUG
#ifdef DEBUG

#ifdef STM32_HAL
#ifdef TRACE
#define PRINTF trace_printf
#else
#define PRINTF printf
#endif

#include"diag/Trace.h"

#define HEX 1

void debug_printf(int x, bool newline) {
  PRINTF("%d", x);
  if (newline == true) {
    PRINTF("\n");
  }
}
void debug_printf(const char * s, bool newline) {
  PRINTF("%s", s);
  if (newline == true) {
    PRINTF("\n");
  }
}
void debug_printf(int s, int mode, bool newline) {
  if (mode == HEX)
    PRINTF("0x%x", s);
  if (newline == true) {
    PRINTF("\n");
  }
}

#define DEBUG_PRINT(x) debug_printf(x,false)
#define DEBUG_PRINTF(x, y) debug_printf(x,y,false)
#define DEBUG_PRINTLN(x) debug_printf(x,true)
#define DEBUG_PRINTLNF(x, y) debug_printf(x,y,true)
#else
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTF(x, y) Serial.print(x, y)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#endif /* STM32_HAL*/

#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTF(x, y)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTLNF(x, y)
#endif /* DEBUG */

#define MPU6050_DMP_CODE_SIZE       1929    // dmpMemory[]
#define MPU6050_DMP_CONFIG_SIZE     192     // dmpConfig[]
#define MPU6050_DMP_UPDATES_SIZE    47      // dmpUpdates[]

/* ================================================================================================ *
 | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
 * ================================================================================================ */

// this block of memory gets written to the MPU on start-up, and it seems
// to be volatile memory, so it has to be done each time (it only takes ~1
// second though)
uint8_t MPU6050::dmpInitialize() {
  // reset device
  DEBUG_PRINTLN(F("\n\nResetting MPU6050..."));
  reset();
  delay(30); // wait after reset

  // enable sleep mode and wake cycle
  /*Serial.println(F("Enabling sleep mode..."));
   setSleepEnabled(true);
   Serial.println(F("Enabling wake cycle..."));
   setWakeCycleEnabled(true);*/

  // disable sleep mode
  DEBUG_PRINTLN(F("Disabling sleep mode..."));
  setSleepEnabled(false);

  // get MPU hardware revision
  DEBUG_PRINTLN(F("Selecting user bank 16..."));
  setMemoryBank(0x10, true, true);
  DEBUG_PRINTLN(F("Selecting memory byte 6..."));
  setMemoryStartAddress(0x06);
  DEBUG_PRINTLN(F("Checking hardware revision..."));
  uint8_t hwRevision = readMemoryByte();
  DEBUG_PRINT(F("Revision @ user[16][6] = "));
  DEBUG_PRINTLNF(hwRevision, HEX);
  DEBUG_PRINTLN(F("Resetting memory bank selection to 0..."));
  setMemoryBank(0, false, false);

  // check OTP bank valid
  DEBUG_PRINTLN(F("Reading OTP bank valid flag..."));
  uint8_t otpValid = getOTPBankValid();
  DEBUG_PRINT(F("OTP bank is "));
  DEBUG_PRINTLN(otpValid ? F("valid!") : F("invalid!"));

  // get X/Y/Z gyro offsets
  DEBUG_PRINTLN(F("Reading gyro offset TC values..."));
  int8_t xgOffsetTC = getXGyroOffsetTC();
  int8_t ygOffsetTC = getYGyroOffsetTC();
  int8_t zgOffsetTC = getZGyroOffsetTC();
  DEBUG_PRINT(F("X gyro offset = "));
  DEBUG_PRINTLN(xgOffsetTC);
  DEBUG_PRINT(F("Y gyro offset = "));
  DEBUG_PRINTLN(ygOffsetTC);
  DEBUG_PRINT(F("Z gyro offset = "));
  DEBUG_PRINTLN(zgOffsetTC);

  // setup weird slave stuff (?)
  DEBUG_PRINTLN(F("Setting slave 0 address to 0x7F..."));
  setSlaveAddress(0, 0x7F);
  DEBUG_PRINTLN(F("Disabling I2C Master mode..."));
  setI2CMasterModeEnabled(false);
  DEBUG_PRINTLN(F("Setting slave 0 address to 0x68 (self)..."));
  setSlaveAddress(0, 0x68);
  DEBUG_PRINTLN(F("Resetting I2C Master control..."));
  resetI2CMaster();
  delay(20);

  // load DMP code into memory banks
  DEBUG_PRINT(F("Writing DMP code to MPU memory banks ("));
  DEBUG_PRINT(MPU6050_DMP_CODE_SIZE);
  DEBUG_PRINTLN(F(" bytes)"));
  if (writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE)) {
    DEBUG_PRINTLN(F("Success! DMP code written and verified."));

    // write DMP configuration
    DEBUG_PRINT(F("Writing DMP configuration to MPU memory banks ("));
    DEBUG_PRINT(MPU6050_DMP_CONFIG_SIZE);
    DEBUG_PRINTLN(F(" bytes in config def)"));
    if (writeProgDMPConfigurationSet(dmpConfig, MPU6050_DMP_CONFIG_SIZE)) {
      DEBUG_PRINTLN(F("Success! DMP configuration written and verified."));

      DEBUG_PRINTLN(F("Setting clock source to Z Gyro..."));
      setClockSource(MPU6050_CLOCK_PLL_ZGYRO);

      DEBUG_PRINTLN(F("Setting DMP and FIFO_OFLOW interrupts enabled..."));
      setIntEnabled(0x12);

      DEBUG_PRINTLN(F("Setting sample rate to 200Hz..."));
      setRate(4); // 1khz / (1 + 4) = 200 Hz

      DEBUG_PRINTLN(F("Setting external frame sync to TEMP_OUT_L[0]..."));
      setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);

      DEBUG_PRINTLN(F("Setting DLPF bandwidth to 42Hz..."));
      setDLPFMode(MPU6050_DLPF_BW_42);

      DEBUG_PRINTLN(F("Setting gyro sensitivity to +/- 2000 deg/sec..."));
      setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

      DEBUG_PRINTLN(F("Setting DMP configuration bytes (function unknown)..."));
      setDMPConfig1(0x03);
      setDMPConfig2(0x00);

      DEBUG_PRINTLN(F("Clearing OTP Bank flag..."));
      setOTPBankValid(false);

      DEBUG_PRINTLN(F("Setting X/Y/Z gyro offset TCs to previous values..."));
      setXGyroOffsetTC(xgOffsetTC);
      setYGyroOffsetTC(ygOffsetTC);
      setZGyroOffsetTC(zgOffsetTC);

      //DEBUG_PRINTLN(F("Setting X/Y/Z gyro user offsets to zero..."));
      //setXGyroOffset(0);
      //setYGyroOffset(0);
      //setZGyroOffset(0);

      DEBUG_PRINTLN(F("Writing final memory update 1/7 (function unknown)..."));
      uint8_t dmpUpdate[16], j;
      uint16_t pos = 0;
      for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
        dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
      writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

      DEBUG_PRINTLN(F("Writing final memory update 2/7 (function unknown)..."));
      for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
        dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
      writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

      DEBUG_PRINTLN(F("Resetting FIFO..."));
      resetFIFO();

      DEBUG_PRINTLN(F("Reading FIFO count..."));
      uint16_t fifoCount = getFIFOCount();
      uint8_t fifoBuffer[128];

      DEBUG_PRINT(F("Current FIFO count="));
      DEBUG_PRINTLN(fifoCount);
      getFIFOBytes(fifoBuffer, fifoCount);

      DEBUG_PRINTLN(F("Setting motion detection threshold to 2..."));
      setMotionDetectionThreshold(2);

      DEBUG_PRINTLN(F("Setting zero-motion detection threshold to 156..."));
      setZeroMotionDetectionThreshold(156);

      DEBUG_PRINTLN(F("Setting motion detection duration to 80..."));
      setMotionDetectionDuration(80);

      DEBUG_PRINTLN(F("Setting zero-motion detection duration to 0..."));
      setZeroMotionDetectionDuration(0);

      DEBUG_PRINTLN(F("Resetting FIFO..."));
      resetFIFO();

      DEBUG_PRINTLN(F("Enabling FIFO..."));
      setFIFOEnabled(true);

      DEBUG_PRINTLN(F("Enabling DMP..."));
      setDMPEnabled(true);

      DEBUG_PRINTLN(F("Resetting DMP..."));
      resetDMP();

      DEBUG_PRINTLN(F("Writing final memory update 3/7 (function unknown)..."));
      for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
        dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
      writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

      DEBUG_PRINTLN(F("Writing final memory update 4/7 (function unknown)..."));
      for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
        dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
      writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

      DEBUG_PRINTLN(F("Writing final memory update 5/7 (function unknown)..."));
      for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
        dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
      writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

      DEBUG_PRINTLN(F("Waiting for FIFO count > 2..."));
      while ((fifoCount = getFIFOCount()) < 3)
        ;

      DEBUG_PRINT(F("Current FIFO count="));
      DEBUG_PRINTLN(fifoCount);
      DEBUG_PRINTLN(F("Reading FIFO data..."));
      getFIFOBytes(fifoBuffer, fifoCount);

      DEBUG_PRINTLN(F("Reading interrupt status..."));
      uint8_t mpuIntStatus = getIntStatus();

      DEBUG_PRINT(F("Current interrupt status="));
      DEBUG_PRINTLNF(mpuIntStatus, HEX);

      DEBUG_PRINTLN(F("Reading final memory update 6/7 (function unknown)..."));
      for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
        dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
      readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

      DEBUG_PRINTLN(F("Waiting for FIFO count > 2..."));
      while ((fifoCount = getFIFOCount()) < 3)
        ;

      DEBUG_PRINT(F("Current FIFO count="));
      DEBUG_PRINTLN(fifoCount);

      DEBUG_PRINTLN(F("Reading FIFO data..."));
      getFIFOBytes(fifoBuffer, fifoCount);

      DEBUG_PRINTLN(F("Reading interrupt status..."));
      mpuIntStatus = getIntStatus();

      DEBUG_PRINT(F("Current interrupt status="));
      DEBUG_PRINTLNF(mpuIntStatus, HEX);

      DEBUG_PRINTLN(F("Writing final memory update 7/7 (function unknown)..."));
      for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++)
        dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
      writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

      DEBUG_PRINTLN(F("DMP is good to go! Finally."));

      DEBUG_PRINTLN(F("Disabling DMP (you turn it on later)..."));
      setDMPEnabled(false);

      DEBUG_PRINTLN(
          F("Setting up internal 42-byte (default) DMP packet buffer..."));
      dmpPacketSize = 42;
      /*if ((dmpPacketBuffer = (uint8_t *)malloc(42)) == 0) {
       return 3; // TODO: proper error code for no memory
       }*/

      DEBUG_PRINTLN(
          F("Resetting FIFO and clearing INT status one last time..."));
      resetFIFO();
      getIntStatus();
    } else {
      DEBUG_PRINTLN(F("ERROR! DMP configuration verification failed."));
      return 2; // configuration block loading failed
    }
  } else {
    DEBUG_PRINTLN(F("ERROR! DMP code verification failed."));
    return 1; // main binary block loading failed
  }
  return 0; // success
}

bool MPU6050::dmpPacketAvailable() {
  return getFIFOCount() >= dmpGetFIFOPacketSize();
}


uint8_t MPU6050::dmpGetAccel(int32_t *data, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default supported now)
  if (packet == 0)
    packet = dmpPacketBuffer;
  data[0] = ((packet[28] << 24) + (packet[29] << 16) + (packet[30] << 8)
      + packet[31]);
  data[1] = ((packet[32] << 24) + (packet[33] << 16) + (packet[34] << 8)
      + packet[35]);
  data[2] = ((packet[36] << 24) + (packet[37] << 16) + (packet[38] << 8)
      + packet[39]);
  return 0;
}
uint8_t MPU6050::dmpGetAccel(int16_t *data, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default supported now)
  if (packet == 0)
    packet = dmpPacketBuffer;
  data[0] = (packet[28] << 8) + packet[29];
  data[1] = (packet[32] << 8) + packet[33];
  data[2] = (packet[36] << 8) + packet[37];
  return 0;
}
uint8_t MPU6050::dmpGetAccel(VectorInt16 *v, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default supported now)
  if (packet == 0)
    packet = dmpPacketBuffer;
  v->x = (packet[28] << 8) + packet[29];
  v->y = (packet[32] << 8) + packet[33];
  v->z = (packet[36] << 8) + packet[37];
  return 0;
}
uint8_t MPU6050::dmpGetQuaternion(int32_t *data, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default supported now)
  if (packet == 0)
    packet = dmpPacketBuffer;
  data[0] = ((packet[0] << 24) + (packet[1] << 16) + (packet[2] << 8)
      + packet[3]);
  data[1] = ((packet[4] << 24) + (packet[5] << 16) + (packet[6] << 8)
      + packet[7]);
  data[2] = ((packet[8] << 24) + (packet[9] << 16) + (packet[10] << 8)
      + packet[11]);
  data[3] = ((packet[12] << 24) + (packet[13] << 16) + (packet[14] << 8)
      + packet[15]);
  return 0;
}
uint8_t MPU6050::dmpGetQuaternion(int16_t *data, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default supported now)
  if (packet == 0)
    packet = dmpPacketBuffer;
  data[0] = ((packet[0] << 8) + packet[1]);
  data[1] = ((packet[4] << 8) + packet[5]);
  data[2] = ((packet[8] << 8) + packet[9]);
  data[3] = ((packet[12] << 8) + packet[13]);
  return 0;
}
uint8_t MPU6050::dmpGetQuaternion(Quaternion *q, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default supported now)
  int16_t qI[4];
  uint8_t status = dmpGetQuaternion(qI, packet);
  if (status == 0) {
    q->w = (float) qI[0] / 16384.0f;
    q->x = (float) qI[1] / 16384.0f;
    q->y = (float) qI[2] / 16384.0f;
    q->z = (float) qI[3] / 16384.0f;
    return 0;
  }
  return status; // int16 return value, indicates error if this line is reached
}
// uint8_t MPU6050::dmpGet6AxisQuaternion(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetRelativeQuaternion(long *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetGyro(int32_t *data, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default supported now)
  if (packet == 0)
    packet = dmpPacketBuffer;
  data[0] = ((packet[16] << 24) + (packet[17] << 16) + (packet[18] << 8)
      + packet[19]);
  data[1] = ((packet[20] << 24) + (packet[21] << 16) + (packet[22] << 8)
      + packet[23]);
  data[2] = ((packet[24] << 24) + (packet[25] << 16) + (packet[26] << 8)
      + packet[27]);
  return 0;
}
uint8_t MPU6050::dmpGetGyro(int16_t *data, const uint8_t* packet) {
  // TODO: accommodate different arrangements of sent data (ONLY default supported now)
  if (packet == 0)
    packet = dmpPacketBuffer;
  data[0] = (packet[16] << 8) + packet[17];
  data[1] = (packet[20] << 8) + packet[21];
  data[2] = (packet[24] << 8) + packet[25];
  return 0;
}
// uint8_t MPU6050::dmpSetLinearAccelFilterCoefficient(float coef);
// uint8_t MPU6050::dmpGetLinearAccel(long *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw,
    VectorFloat *gravity) {
  // get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet, sensitivity is 2g)
  v->x = vRaw->x - gravity->x * 8192;
  v->y = vRaw->y - gravity->y * 8192;
  v->z = vRaw->z - gravity->z * 8192;
  return 0;
}
// uint8_t MPU6050::dmpGetLinearAccelInWorld(long *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal,
    Quaternion *q) {
  // rotate measured 3D acceleration vector into original state
  // frame of reference based on orientation quaternion
  memcpy(v, vReal, sizeof(VectorInt16));
  v->rotate(q);
  return 0;
}
// uint8_t MPU6050::dmpGetGyroAndAccelSensor(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetGyroSensor(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetControlData(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetTemperature(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetGravity(long *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetGravity(VectorFloat *v, Quaternion *q) {
  v->x = 2 * (q->x * q->z - q->w * q->y);
  v->y = 2 * (q->w * q->x + q->y * q->z);
  v->z = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
  return 0;
}
// uint8_t MPU6050::dmpGetUnquantizedAccel(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetQuantizedAccel(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetExternalSensorData(long *data, int size, const uint8_t* packet);
// uint8_t MPU6050::dmpGetEIS(long *data, const uint8_t* packet);

uint8_t MPU6050::dmpGetEuler(float *data, Quaternion *q) {
  data[0] = atan2(2 * q->x * q->y - 2 * q->w * q->z,
      2 * q->w * q->w + 2 * q->x * q->x - 1);   // psi
  data[1] = -asin(2 * q->x * q->z + 2 * q->w * q->y);                   // theta
  data[2] = atan2(2 * q->y * q->z - 2 * q->w * q->x,
      2 * q->w * q->w + 2 * q->z * q->z - 1);   // phi
  return 0;
}
uint8_t MPU6050::dmpGetYawPitchRoll(float *data, Quaternion *q,
    VectorFloat *gravity) {
  // yaw: (about Z axis)
  data[0] = atan2(2 * q->x * q->y - 2 * q->w * q->z,
      2 * q->w * q->w + 2 * q->x * q->x - 1);
  // pitch: (nose up/down, about Y axis)
  data[1] = atan(
      gravity->x / sqrt(gravity->y * gravity->y + gravity->z * gravity->z));
  // roll: (tilt left/right, about X axis)
  data[2] = atan(
      gravity->y / sqrt(gravity->x * gravity->x + gravity->z * gravity->z));
  return 0;
}

// uint8_t MPU6050::dmpGetAccelFloat(float *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetQuaternionFloat(float *data, const uint8_t* packet);

uint8_t MPU6050::dmpProcessFIFOPacket(const unsigned char *dmpData) {
  /*for (uint8_t k = 0; k < dmpPacketSize; k++) {
   if (dmpData[k] < 0x10) Serial.print("0");
   Serial.print(dmpData[k], HEX);
   Serial.print(" ");
   }
   Serial.print("\n");*/
  //Serial.println((uint16_t)dmpPacketBuffer);
  return 0;
}
uint8_t MPU6050::dmpReadAndProcessFIFOPacket(uint8_t numPackets,
    uint8_t *processed) {
  uint8_t status;
  uint8_t buf[dmpPacketSize];
  for (uint8_t i = 0; i < numPackets; i++) {
    // read packet from FIFO
    getFIFOBytes(buf, dmpPacketSize);

    // process packet
    if ((status = dmpProcessFIFOPacket(buf)) > 0)
      return status;

    // increment external process count variable, if supplied
    if (processed != 0)
      *processed++;
  }
  return 0;
}

// uint8_t MPU6050::dmpSetFIFOProcessedCallback(void (*func) (void));

// uint8_t MPU6050::dmpInitFIFOParam();
// uint8_t MPU6050::dmpCloseFIFO();
// uint8_t MPU6050::dmpSetGyroDataSource(uint_fast8_t source);
// uint8_t MPU6050::dmpDecodeQuantizedAccel();
// uint32_t MPU6050::dmpGetGyroSumOfSquare();
// uint32_t MPU6050::dmpGetAccelSumOfSquare();
// void MPU6050::dmpOverrideQuaternion(long *q);
uint16_t MPU6050::dmpGetFIFOPacketSize() {
  return dmpPacketSize;
}

