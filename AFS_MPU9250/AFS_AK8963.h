/*!
 *  @file AFS_AK8963.h
 *
 * 	I2C Driver for AKM's AK8963 Magnetometer
 *
 * 	This is the AK8963 Magnetometer component of the MPU-9250 IMU library.
 *
 * 	Full credit to Adafruit for the Adafruit_MPU6050 library from which
 *  this library was derived.
 *
 *	MIT license (see license.txt)
 */
 
#ifndef AK8963_H
#define AK8963_H

/*****************************************************************************
 * AKM AK8963 Magnetometer Registers and Bit Fields
 *
 * Credit to Kris Winer/Tlera Corporation for poking in the register info.
 * See github.com/kriswiner/MPU9250 for more.
 *
 * Per Kris W.:
 * See also MPU-9250 Register Map and Descriptions, Revision 4.0,
 * RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in above
 * document; the MPU9250 and MPU9150 are virtually identical but the latter
 * has a different register map
******************************************************************************/

#include "IMU_Device.h"
#include <Adafruit_I2CDevice.h>

// default values
#define AK8963_I2C_ADDRESS  0x0C
#define AK8963_WHO_I_AM     0x48

// AKM AK8963 Magnetometer Registers
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // raw data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL1                                                           \
  0x0A                    // Power down (0000), single-measurement (0001),
                               // self-test (1000) and Fuse ROM (1111) modes
                               // on bits 3:0
#define AK8963_CNTL2     0x0B  // Reset
#define AK8963_ASTC      0x0C  // Self test control
//#define AK8963_TS1     0x0D  // Test 1 - DO NOT ACCESS
//#define AK8963_TS2     0x0E  // Test 2 - DO NOT ACCESS
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value
//#define AK8963_RSV     0x13  // Reserved = DO NOT ACCESS

// bit fields

// AK8963_ST1 0x02 (READ_ONLY)
#define ST1_DOR   1,1   //0-normal 1-data overrun
#define ST1_DRDY  1,0   //0-normal 1-data ready

// AK8963_HXL - HZH 0x03-0x08
// array and read sizes for bulk data transfer
#define AK8963_BULK_RAW_OUT_BYTES   6   // continuous read of raw magXYZ data
#define AK63_RAW_BYTES              6   // continuous read of raw magXYZ data
#define AK63_RAW_BYTES_ST2                                                     \
  7 /* continuous read of raw magXYZ data                                      \
                                             + STATUS_2 (resets INTR)       */
#define AK63_ST1_RAW_BYTES_ST2                                                 \
  8 /* continuous read of raw magXYZ data                                      \
         + both status registers                                               \
                                             (resets INTR)                  */

// AK8963_ST2 0x09 (Read-Only)
#define ST2_BITM 1,4    // 0-14-bit 1-16-bit output stream, matches C1_BIT below
#define ST2_HOFL 1,3    // 0-normal, 1-magnetic sensor overflow occurred

// AK8963_CNTL1 0x0A R/W
#define C1_BIT   1,4    // 0-14-bit 1-16-bit resolution output
#define C1_MODE  4,0    // operation_mode

// AK8963_CNTL2 0x0B R/W
#define C2_SRST  1,0    // 1-Soft Reset (returns to 0)

// AK8963_ASTC 0x0C R/W
#define AS_SELF  1,6    // 1-Self Test

typedef enum {
  OP_MODE_POWER_DOWN,
  OP_MODE_SINGLE_MSMT,
  OP_MODE_CONTINUOUS_1, // mode 1=8Hz, mode 2=100Hz sampling
  OP_MODE_CONTINUOUS_2 = 0b0110,
  OP_MODE_EXT_TRIGGER  = 0b0100,
  OP_MODE_SELF_TEST    = 0b1000,
  OP_MODE_FUSE_ROM     = 0b1111
} magn_operation_mode_t;

typedef enum { RESOLUTION_14_BIT, RESOLUTION_16_BIT } magn_resolution_t;

const float RES_14BIT_SCALING = 4912.0/8190;  // scale max reading to max
const float RES_16BIT_SCALING = 4912.0/32760; //   range +/-4792uT

class AK8963_Magnetometer : public Adafruit_Sensor {

public:
  AK8963_Magnetometer(IMU_Device* imu_dev);
  virtual ~AK8963_Magnetometer(void) {}

  bool begin(uint8_t sensor_id, bool in_pass_through_mode,
             magn_operation_mode_t operation_mode=OP_MODE_CONTINUOUS_2,
             magn_resolution_t res_type=RESOLUTION_16_BIT);
  bool getEvent(sensors_event_t* event);
  void getSensor(sensor_t* sensor);
  bool setResolution(magn_resolution_t res_bits,
                     bool in_pass_through_mode = false);
  bool setMode(magn_operation_mode_t op_mode,
               bool in_pass_through_mode = false);
  bool test_who_am_i(bool in_pass_through_mode=false);

protected:
  IMU_Device* imuHost;  // the pass-through host
  float resolution_uT;  // value based on magn_resolution_t

private:
  uint32_t sensor_id;
  Adafruit_I2CDevice* magn_i2c_device;

  // add your specific classes here
  // for access to private elements
  friend class AFS_MPU9250;
};

#endif /* _H */