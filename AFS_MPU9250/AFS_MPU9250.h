/*!
 *  @file AFS_MPU9250.h
 *
 * 	I2C Driver for MPU-9250 9-DoF Gyro, Accelerometer, Magnetometer
 *
 * 	This is a library for various MPU-9250 breakout boards (GY-5221, etc.).
 *
 * 	Full credit to Adafruit for the Adafruit_MPU6050 library from which this
 *  library was derived (i.e., initially ported). Thanks to SparkFun and
 *  Kris W. for some code that provided clarity.
 *
 *	MIT license (see license.txt)
 */

#ifndef AFS_MPU9250_H
#define AFS_MPU9250_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "IMU_Device.h"
#include "AFS_AK8963.h"

#define MPU9250_I2CADDR_DEFAULT   0x68  ///< MPU9250 default i2c address w/ AD0 high
#define MPU9250_DEVICE_ID         0x71  ///< The correct MPU9250_WHO_AM_I value
#define MPU9250_AK8963_ADDRESS    0x0C  ///< Address of magnetometer

// Registers
#define MPU9250_SELF_TEST_X_GYRO  0x00  ///< Self test factory calibrated values register
#define MPU9250_SELF_TEST_Y_GYRO  0x01  ///< Self test factory calibrated values register
#define MPU9250_SELF_TEST_Z_GYRO  0x02  ///< Self test factory calibrated values register
#define MPU9250_SELF_TEST_X_ACCEL 0x0D  ///< Self test factory calibrated values register
#define MPU9250_SELF_TEST_Y_ACCEL 0x0E  ///< Self test factory calibrated values register
#define MPU9250_SELF_TEST_Z_ACCEL 0x0F  ///< Self test factory calibrated values register
#define MPU9250_XG_OFFSET_H       0x13  ///< Self test factory calibrated values register
#define MPU9250_XG_OFFSET_L       0x14  ///< Self test factory calibrated values register
#define MPU9250_YG_OFFSET_H       0x15  ///< Self test factory calibrated values register
#define MPU9250_YG_OFFSET_L       0x16  ///< Self test factory calibrated values register
#define MPU9250_ZG_OFFSET_H       0x17  ///< Self test factory calibrated values register
#define MPU9250_ZG_OFFSET_L       0x18  ///< Self test factory calibrated values register
#define MPU9250_SMPLRT_DIV        0x19  ///< sample rate divisor register
#define MPU9250_CONFIG            0x1A  ///< General configuration register
#define MPU9250_GYRO_CONFIG       0x1B  ///< Gyro specfic configuration register
#define MPU9250_ACCEL_CONFIG      0x1C  ///< Accelerometer specific configration register
#define MPU9250_ACCEL_CONFIG_2    0x1D  ///<
#define MPU9250_LP_ACCEL_ODR      0x1E
#define MPU9250_WOM_THR           0x1F
#define MPU9250_FIFO_EN           0x23
#define MPU9250_I2C_MST_CTRL      0x24
#define MPU9250_I2C_SLV0_ADDR     0x25
#define MPU9250_I2C_SLV0_REG      0x26
#define MPU9250_I2C_SLV0_CTRL     0x27
#define MPU9250_I2C_SLV1_ADDR     0x28
#define MPU9250_I2C_SLV1_REG      0x29
#define MPU9250_I2C_SLV1_CTRL     0x2A
#define MPU9250_I2C_SLV2_ADDR     0x2B
#define MPU9250_I2C_SLV2_REG      0x2C
#define MPU9250_I2C_SLV2_CTRL     0x2D
#define MPU9250_I2C_SLV3_ADDR     0x2E
#define MPU9250_I2C_SLV3_REG      0x2F
#define MPU9250_I2C_SLV3_CTRL     0x30
#define MPU9250_I2C_SLV4_ADDR     0x31
#define MPU9250_I2C_SLV4_REG      0x32
#define MPU9250_I2C_SLV4_DO       0x33
#define MPU9250_I2C_SLV4_CTRL     0x34
#define MPU9250_I2C_SLV4_DI       0x35
#define MPU9250_I2C_MST_STATUS    0x36
#define MPU9250_INT_PIN_CFG       0x37  ///< Interrupt pin configuration register
#define MPU9250_INT_ENABLE        0x38  ///< Interrupt enable configuration register
#define MPU9250_INT_STATUS        0x3A  ///< Interrupt status register
#define MPU9250_ACCEL_XOUT_H      0x3B
#define MPU9250_ACCEL_XOUT_L      0x3C
#define MPU9250_ACCEL_YOUT_H      0x3D
#define MPU9250_ACCEL_YOUT_L      0x3E
#define MPU9250_ACCEL_ZOUT_H      0x3F
#define MPU9250_ACCEL_ZOUT_L      0x40
#define MPU9250_TEMP_OUT_H        0x41
#define MPU9250_TEMP_OUT_L        0x42
#define MPU9250_GYRO_XOUT_H       0x43
#define MPU9250_GYRO_XOUT_L       0x44
#define MPU9250_GYRO_YOUT_H       0x45
#define MPU9250_GYRO_YOUT_L       0x46
#define MPU9250_GYRO_ZOUT_H       0x47
#define MPU9250_GYRO_ZOUT_L       0x48
#define MPU9250_EXT_SENS_DATA_00  0x49
#define MPU9250_EXT_SENS_DATA_01  0x4A
#define MPU9250_EXT_SENS_DATA_02  0x4B
#define MPU9250_EXT_SENS_DATA_03  0x4C
#define MPU9250_EXT_SENS_DATA_04  0x4D
#define MPU9250_EXT_SENS_DATA_05  0x4E
#define MPU9250_EXT_SENS_DATA_06  0x4F
#define MPU9250_EXT_SENS_DATA_07  0x50
#define MPU9250_EXT_SENS_DATA_08  0x51
#define MPU9250_EXT_SENS_DATA_09  0x52
#define MPU9250_EXT_SENS_DATA_10  0x53
#define MPU9250_EXT_SENS_DATA_11  0x54
#define MPU9250_EXT_SENS_DATA_12  0x55
#define MPU9250_EXT_SENS_DATA_13  0x56
#define MPU9250_EXT_SENS_DATA_14  0x57
#define MPU9250_EXT_SENS_DATA_15  0x58
#define MPU9250_EXT_SENS_DATA_16  0x59
#define MPU9250_EXT_SENS_DATA_17  0x5A
#define MPU9250_EXT_SENS_DATA_18  0x5B
#define MPU9250_EXT_SENS_DATA_19  0x5C
#define MPU9250_EXT_SENS_DATA_20  0x5D
#define MPU9250_EXT_SENS_DATA_21  0x5E
#define MPU9250_EXT_SENS_DATA_22  0x5F
#define MPU9250_EXT_SENS_DATA_23  0x60
#define MPU9250_I2C_SLV0_DO       0X63
#define MPU9250_I2C_SLV1_DO       0X64
#define MPU9250_I2C_SLV2_DO       0X65
#define MPU9250_I2C_SLV3_DO       0X66
#define MPU9250_I2C_MST_DELAY_CTRL 0X67
#define MPU9250_SIGNAL_PATH_RESET 0x68  ///< Signal path reset register
#define MPU9250_MOT_DETECT_CTRL   0X69
#define MPU9250_USER_CTRL         0x6A  ///< FIFO and I2C Master control register
#define MPU9250_PWR_MGMT_1        0x6B  ///< Primary power/sleep control register
#define MPU9250_PWR_MGMT_2        0x6C  ///< Secondary power/sleep control register
#define MPU9250_FIFO_COUNTH       0X72
#define MPU9250_FIFO_COUNTL       0X73
#define MPU9250_FIFO_R_W          0X74
#define MPU9250_WHO_AM_I          0x75  ///< Divice ID register
#define MPU9250_XA_OFFSET_H       0X77
#define MPU9250_XA_OFFSET_L       0X78
#define MPU9250_YA_OFFSET_H       0X7A
#define MPU9250_YA_OFFSET_L       0X7B
#define MPU9250_ZA_OFFSET_H       0X7D
#define MPU9250_ZA_OFFSET_L       0X7E

// bit fields

// MPU9250_CONFIG 0x1A R/W
#define CF_FIFO_MODE    1,6
#define CF_EXT_SYNC_SET 3,3
#define CF_DLPF_CFG     3,0

// MPU9250_GYRO_CONFIG 0x1B R/W
#define GC_XGYRO_CTEN   1,7
#define GC_YGYRO_CTEN   1,6
#define GC_ZGYRO_CTEN   1,5
#define GC_GYRO_FS_SEL  2,3
// reserved             1,2
#define GC_FCHOICE_B    2,0

// MPU9250_ACCEL_CONFIG 0x1C R/W
#define AC_AX_ST_EN     1,7
#define AC_AY_ST_EN     1,6
#define AC_AZ_ST_EN     1,5
#define AC_ACCEL_FS_SEL 2,3
#define AC_RESV         3,0

// MPU9250_ACCEL_CONFIG_2 0x1D R/W
#define AC2_RESV              4,4
#define AC2_ACCEL_FCHOICE_B   1,3
#define AC2_A_DLPFCFG         3,0

// MPU9250_LP_ACCEL_ODR 0x1E R/W
#define LP_A_ODR_RESV         4,4
#define LP_A_ODR_LPOSC_CLKSEL 4,0

// MPU9250_FIFO_EN 0x23 R/W
#define FE_TEMP_OUT           1,7
#define FE_GYRO_XOUT          1,6
#define FE_GYRO_YOUT          1,5
#define FE_GYRO_ZOUT          1,4
#define FE_ACCEL              1,3
#define FE_SLV_2              1,2
#define FE_SLV_1              1,1
#define FE_SLV_0              1,0

// MPU9250_I2C_MST_CTRL 0x24 R/W
#define MC_MULT_MST_EN        1,7
#define MC_WAIT_FOR_ES        1,6
#define MC_SLV_3_FIFO_EN      1,5
#define MC_I2C_MST_P_NSR      1,4
#define MC_I2C_MST_CLK        4,0

// I2C_MST_STATUS 0x36 (read only)

// MPU9250_INT_PIN_CFG 0x37 R/W
#define IPC_ACTL              1,7
#define IPC_OPEN              1,6
#define IPC_LATCH_INT_EN      1,5
#define IPC_INT_ANYRD_2CLEAR  1,4
#define IPC_ACTL_FSYNC        1,3
#define IPC_FSYNC_INT_MODE_EN 1,2
#define IPC_BYPASS_EN         1,1
// reserved 1,0

// MPU9250_INT_ENABLE 0x38 R/W
#define IE_WOM_EN             1,6
#define IE_FIFO_OVERFLOW_EN   1,4
#define IE_FSYNC_INT_EN       1,3
#define IE_RAW_RDY_EN         1,0

// MPU9250_INT_STATUS 0x3A (read only)
#define IS_WOM_INT            1,6
#define IS_FIFO_OVERFLOW_INT  1,4
#define IS_FSYNC_INT          1,3
#define IS_RAW_DATA_RDY_INT   1,0

// MPU9250_EXT_SENS_DATA_00 0x49 (read only)

// MPU9250_I2C_MST_DELAY_CTRL 0x67 R/W

// MPU9250_SIGNAL_PATH_RESET 0x68 R/W
// reserved                   5,3
#define SPR_GYRO_RST          1,2
#define SPR_ACCL_RST          1,1
#define SPR_TEMP_RST          1,0
#define SPR_ALL_SENSOR_RST    3,0

// MPU9250_MOT_DETECT_CTRL 0x69 R/W

// MPU9250_USER_CTRL 0x6A R/W
// reserved                   1,7
#define UC_FIFO_EN            1,6
#define UC_I2C_MST_EN         1,5
#define UC_I2C_IF_DIS         1,4
// reserved                   1,3
#define UC_FIFO_RST           1,2
#define UC_I2C_MST_RST        1,1
#define UC_SIG_CON_RST        1,0

// MPU9250_PWR_MGMT_1 0x6B R/W
#define PM1_H_RESET 1,7
#define PM1_SLEEP   1,6
#define PM1_CYCLE   1,5
#define PM1_GYRO_STANDBY 1,4
#define PM1_PD_STAT 1,3
#define PM1_CLK_SEL 3,0

// MPU9250_PWR_MGMT_2 0x6C R/W
#define PM2_DISABLE_XA  1,5
#define PM2_DISABLE_YA  1,4
#define PM2_DISABLE_ZA  1,3
#define PM2_DISABLE_XG  1,2
#define PM2_DISABLE_YG  1,1
#define PM2_DISABLE_ZG  1,0
#define PM2_DISABLE_AC  3,3
#define PM2_DISABLE_GY  3,0

// MPU9250_I2C_SLVx_ADDR
#define SL_I2C_SLV_RNW        1,7 // 0-write, 1-read
#define SL_I2C_ID             7,0

// MPU9250_I2C_SLVx_REG
#define SL_I2C_SLV_REG        8,0 // register

// MPU9250_I2C_SLVx_CTRL
#define SL_I2C_SLV_EN         1,7 // 0-disable, 1-enable
#define SL_I2C_SLV_BYTE_SW    1,6 // 0-no swapping, else various
#define SL_I2C_SLV_REG_DIS    1,5 // 1-do not write a register value
#define SL_I2C_SLV_GRP        1,4 // 0-even grouping, 1-odd grouping
#define SL_I2C_SLV_LENG       4,0 // number of bytes to be read from slave

// MPU9250_I2C_SLVx_DO
#define SL_I2C_SLV_DO         8,0 // slave data out

/**
 * @brief FSYNC output values
 *
 * Allowed values for `setFsyncSampleOutput`.
 */
typedef enum fsync_out {
  MPU9250_FSYNC_OUT_DISABLED,
  MPU9250_FSYNC_OUT_TEMP,
  MPU9250_FSYNC_OUT_GYROX,
  MPU9250_FSYNC_OUT_GYROY,
  MPU9250_FSYNC_OUT_GYROZ,
  MPU9250_FSYNC_OUT_ACCELX,
  MPU9250_FSYNC_OUT_ACCELY,
  MPU9250_FSYNC_OUT_ACCEL_Z,
} MPU9250_fsync_out_t;

/**
 * @brief Clock source options
 *
 * Allowed values for `setClock`.
 */
typedef enum clock_select {
  MPU9250_INTERNAL_20MHZ,
  MPU9250_AUTO_BEST_ELSE_PLL1,
  MPU9250_AUTO_BEST_ELSE_PLL2,
  MPU9250_AUTO_BEST_ELSE_PLL3,
  MPU9250_AUTO_BEST_ELSE_PLL4,
  MPU9250_AUTO_BEST_ELSE_PLL5,
  MPU9250_INTERNAL2_20MHZ,
  MPU9250_STOP = 7,
} MPU9250_clock_select_t;

/**
 * @brief Accelerometer range options
 *
 * Allowed values for `setAccelerometerRange`.
 */
typedef enum {
  MPU9250_RANGE_2_G = 0b00,  ///< +/- 2g (default value)
  MPU9250_RANGE_4_G = 0b01,  ///< +/- 4g
  MPU9250_RANGE_8_G = 0b10,  ///< +/- 8g
  MPU9250_RANGE_16_G = 0b11, ///< +/- 16g
} MPU9250_accel_range_t;

/**
 * @brief Gyroscope range options
 *
 * Allowed values for `setGyroRange`.
 */
typedef enum {
  MPU9250_RANGE_250_DEG,  ///< +/- 250 deg/s (default value)
  MPU9250_RANGE_500_DEG,  ///< +/- 500 deg/s
  MPU9250_RANGE_1000_DEG, ///< +/- 1000 deg/s
  MPU9250_RANGE_2000_DEG, ///< +/- 2000 deg/s
} MPU9250_gyro_range_t;

typedef enum {
  MPU9250_GYRO_FCHOICE_8800   = 0b00, // 0bx0 AFC1 AFC0
  MPU9250_GYRO_FCHOICE_8800_2 = 0b10,
  MPU9250_GYRO_FCHOICE_3600   = 0b01, // 0b01
  MPU9250_GYRO_FCHOICE_DLPF   = 0b11  // ob11
} gyro_fchoice_t; // invert for config

/* SAMPLE_RATE_DIV only effective when
   FChoice == 0b11 (FChoice_b == 0b11) */

typedef enum {
  MPU9250_GYRO_FCHOICE_1130 = 0b0,  // A_FC
  MPU9250_GYRO_FCHOICE_DLPD = 0b1,
} accel_fchoice_t; // invert for config

/**
 * @brief Gyroscope low pass filter options
 *
 * Allowed values for `setLowPassFilter`.
 */
typedef enum {
  MPU9250_LOWPASS_GYRO_BW_250_HZ, // Gyro BW   for FCHOICE == 0b11
  MPU9250_LOWPASS_GYRO_BW_184_HZ, // else 3600 for FCHOICE == 0bx0
  MPU9250_LOWPASS_GYRO_BW_92_HZ,  // else 1800 for FCHOICE == 0b01
  MPU9250_LOWPASS_GYRO_BW_41_HZ,
  MPU9250_LOWPASS_GYRO_BW_20_HZ,
  MPU9250_LOWPASS_GYRO_BW_10_HZ,
  MPU9250_LOWPASS_GYRO_BW_5_HZ,
  MPU9250_LOWPASS_GYRO_BW_3600_HZ,
} MPU9250_gyro_dlpf_t;

/**
 * @brief Accelerometer low pass filter options
 *
 * Allowed values for `setLowPassFilter`.
 */
typedef enum {
  MPU9250_LOWPASS_ACCEL_BW_460_HZ, // Accel BW  for ACCEL_FCHOICE == 0b1
  MPU9250_LOWPASS_ACCEL_BW_184_HZ, // else 1130 for ACCEL_FCHOICE == 0b0
  MPU9250_LOWPASS_ACCEL_BW_92_HZ,
  MPU9250_LOWPASS_ACCEL_BW_41_HZ,
  MPU9250_LOWPASS_ACCEL_BW_20_HZ,
  MPU9250_LOWPASS_ACCEL_BW_10_HZ,
  MPU9250_LOWPASS_ACCEL_BW_5_HZ,
  MPU9250_LOWPASS_ACCEL_BW2_460_HZ, // duplicated value for cfg 0 & 7
} MPU9250_accel_dlpf_t;

/**
 * @brief Periodic measurement options
 *
 * Allowed values for `setCycleRate`.
 */
typedef enum {
  MPU9250_CYCLE_1_25_HZ, ///< 1.25 Hz
  MPU9250_CYCLE_5_HZ,    ///< 5 Hz
  MPU9250_CYCLE_20_HZ,   ///< 20 Hz
  MPU9250_CYCLE_40_HZ,   ///< 40 Hz
} MPU9250_cycle_rate_t;

class AFS_MPU9250;

/** Adafruit Unified Sensor interface for temperature component of MPU9250 */
class AFS_MPU9250_Temp : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the temp sensor
      @param parent A pointer to the MPU9250 class */
  AFS_MPU9250_Temp(AFS_MPU9250 *parent) { _theMPU9250 = parent; }
  bool getEvent(sensors_event_t*);
  void getSensor(sensor_t*);

private:
  int _sensorID = 0x650;
  AFS_MPU9250* _theMPU9250 = NULL;
};

#ifndef FOO
/** Adafruit Unified Sensor interface for accelerometer component of MPU9250 */
class AFS_MPU9250_Magnetometer : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the magnetometer
     sensor
      @param parent A pointer to the MPU9250 class */
  AFS_MPU9250_Magnetometer(AFS_MPU9250* parent) {
    _theMPU9250 = parent;
  }
  bool getEvent(sensors_event_t*);
  void getSensor(sensor_t*);

private:
  int _sensorID = 0x651;
  AFS_MPU9250* _theMPU9250 = NULL;
};
#endif

/** Adafruit Unified Sensor interface for accelerometer component of MPU9250 */
class AFS_MPU9250_Accelerometer : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the accelerometer
     sensor
      @param parent A pointer to the MPU9250 class */
  AFS_MPU9250_Accelerometer(AFS_MPU9250* parent) {
    _theMPU9250 = parent;
  }
  bool getEvent(sensors_event_t*);
  void getSensor(sensor_t*);

private:
  int _sensorID = 0x651;
  AFS_MPU9250* _theMPU9250 = NULL;
};

/** Adafruit Unified Sensor interface for gyro component of MPU9250 */
class AFS_MPU9250_Gyro : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the gyro sensor
      @param parent A pointer to the MPU9250 class */
  AFS_MPU9250_Gyro(AFS_MPU9250* parent) { _theMPU9250 = parent; }
  bool getEvent(sensors_event_t*);
  void getSensor(sensor_t*);

private:
  int _sensorID = 0x652;
  AFS_MPU9250* _theMPU9250 = NULL;
};

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the MPU9250 9-DOF Motion Processing Unit
 */
class AFS_MPU9250 : public IMU_Device {
public:
  AFS_MPU9250();
  ~AFS_MPU9250();

  bool begin(uint8_t i2c_addr = MPU9250_I2CADDR_DEFAULT,
             uint8_t i2c_addr_magn = MPU9250_AK8963_ADDRESS,
             TwoWire* wire = &Wire, int32_t sensorID = 0);

  // Adafruit_Sensor API/Interface
  bool getEvent(sensors_event_t* accel, sensors_event_t* magn,
                sensors_event_t* gyro,  sensors_event_t* temp);

  MPU9250_accel_range_t getAccelerometerRange(void);
  void setAccelerometerRange(MPU9250_accel_range_t);

  MPU9250_gyro_range_t getGyroRange(void);
  void setGyroRange(MPU9250_gyro_range_t);

  void setInterruptPinPolarity(bool active_low);
  void setInterruptPinLatch(bool held);
  void setFsyncSampleOutput(MPU9250_fsync_out_t fsync_output);

  MPU9250_gyro_dlpf_t getGyroFilterBandwidth(void);
  void setGyroFilterBandwidth(MPU9250_gyro_dlpf_t bandwidth,
                              gyro_fchoice_t gfchoice);
  MPU9250_accel_dlpf_t getAccelFilterBandwidth(void);
  void setAccelFilterBandwidth(MPU9250_accel_dlpf_t bandthwidth,
                               accel_fchoice_t afchoice);

  void setDataReadyInterrupt(bool enable_dri);
  void setMotionInterrupt(bool active);
  void setMotionDetectionThreshold(uint8_t thr);
  bool getMotionInterruptStatus(void);
  void setClearIntrOnRead(bool clear_on_any_read);

  void setI2C_Bypass(bool bypass);
  int  getI2C_Bypass(void);

  MPU9250_fsync_out_t getFsyncSampleOutput(void);

  void setClock(MPU9250_clock_select_t);
  MPU9250_clock_select_t getClock(void);

  void setSampleRateDivisor(uint8_t);
  uint8_t getSampleRateDivisor(void);

  bool enableSleep(bool enable);
  bool enableCycle(bool enable);

  bool setGyroStandby(bool xAxisStandby, bool yAxisStandby, bool zAxisStandby);
  bool setAccelerometerStandby(bool xAxisStandby, bool yAxisStandby,
                               bool zAxisStandby);
  bool setTemperatureStandby(bool enable);

  void reset(void);

  Adafruit_Sensor* getTemperatureSensor(void);
  Adafruit_Sensor* getMagnetometerSensor(void);
  Adafruit_Sensor* getAccelerometerSensor(void);
  Adafruit_Sensor* getGyroSensor(void);

private:
  void _getRawSensorData(void);
  void _scaleSensorData(void);

protected:
  float temperature, ///< Last reading's temperature (C)
      accX,          ///< Last reading's accelerometer X axis m/s^2
      accY,          ///< Last reading's accelerometer Y axis m/s^2
      accZ,          ///< Last reading's accelerometer Z axis m/s^2
      gyroX,         ///< Last reading's gyro X axis in rad/s
      gyroY,         ///< Last reading's gyro Y axis in rad/s
      gyroZ;         ///< Last reading's gyro Z axis in rad/s

  Adafruit_I2CDevice* i2c_dev = NULL; ///< Pointer to MPU-9250 I2C device

  AFS_MPU9250_Accelerometer* accel_sensor = NULL; ///< Accelerometer sensor object
  AK8963_Magnetometer* magn_sensor = NULL;  ///< Magnetometer sensor object
  AFS_MPU9250_Gyro* gyro_sensor = NULL; ///< Gyro sensor object
  AFS_MPU9250_Temp* temp_sensor = NULL; ///< Temp sensor sensor object

  bool proxy_write(Adafruit_I2CDevice* i2c_device, uint8_t sregister,
                   uint8_t bitwidth, uint8_t shift, uint8_t value,
                   uint8_t nbytes=1);
  uint32_t proxy_read(Adafruit_I2CDevice* i2c_device, uint8_t sregister,
                      uint8_t bitwidth, uint8_t shift,
                      uint8_t nbytes=1);
  bool proxy_read(Adafruit_I2CDevice* i2c_device, uint8_t start_register,
                  uint8_t* buffer, uint8_t nbytes=1);
  bool bypass_mode(void);

  uint16_t
      _sensorid_accel, ///< ID number for accelerometer
      _sensorid_magn,       ///< ID number for magnetometer
      _sensorid_gyro,       ///< ID number for gyro
      _sensorid_temp;       ///< ID number for temperature

  void _read(void);
  virtual bool _init(int32_t sensor_id);
  bool config_slave_regs(uint8_t sl_i2c_address, uint slave_number,
                         uint8_t sl_base_register, uint8_t num_registers);

private:
  friend class AFS_MPU9250_Accelerometer; ///< Gives Accelerometer access to private members
  friend class AFS_MPU9250_Magnetometer;  ///< Gives Magn access to private members
  friend class AFS_MPU9250_Gyro; ///< Gives Gyro access to private members
  friend class AFS_MPU9250_Temp; ///< Gives Temp access to private members

  int16_t rawAccX, rawAccY, rawAccZ, rawTemp,
          rawGyroX, rawGyroY, rawGyroZ,
          rawMagnX, rawMagnY, rawMagnZ;

  void fillGyroEvent(sensors_event_t* gyro, uint32_t timestamp);
  void fillTempEvent(sensors_event_t* temp, uint32_t timestamp);
  void fillMagnEvent(sensors_event_t* magn, uint32_t timestamp);
  void fillAccelEvent(sensors_event_t* accel, uint32_t timestamp);

  uint magnInstance;
};

#endif /* _H */
