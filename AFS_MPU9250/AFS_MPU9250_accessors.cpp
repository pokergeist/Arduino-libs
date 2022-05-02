/*!
 *  @file AFS_MPU9250_accessors.cpp
 *
 *  @mainpage Accessor Methods for the MPU-9250 9-DoF Gyro, Accelerometer,
 *            Magnetometer sensor library.
 *
 *  @section intro_sec Introduction
 *
 * 	This file contains basic accessor methods for the MPU9250. The were moved
 *  out of the main file simply for ease of navigation. They may be
 *  re-integrated later.
 *
 *  @section dependencies Dependencies
 *
 *  This library depends on the Adafruit BusIO, Wire, and
 *  Adafruit_Unified_Sensor libraries.
 *
 *  @section author Author
 *
 *  John Jordan
 *
 * 	@section license License
 *
 * 	MIT (see license.txt)
 *
 * 	@section  HISTORY
 *
 * 	Full credit to Adafruit for the Adafruit_MPU6050 library from which this
 *  library was derived (i.e., initially ported). Thanks to SparkFun and
 *  Kris W. for some code that provided clarity.
 *
 *  v0.1 - First release. Still lots to do to access the full potential of this
 *         device. Some methods from the initial port may not work as intended.
 */

#include "Arduino.h"
#include <Wire.h>
#include <Streaming.h>

#ifdef DEBUG
  #define DBG_SERIAL(...) Serial.print(__VAR_ARGS__)
#else
  #define DBG_SERIAL(...) do {} while(0)
#endif

#include "AFS_MPU9250.h"

/**************************************************************************/
/*!
    @brief Gets the sample rate divisor.
    @return  The sample rate divisor
*/
/**************************************************************************/
uint8_t AFS_MPU9250::getSampleRateDivisor(void) {
  Adafruit_BusIO_Register sample_rate_div(i2c_dev, MPU9250_SMPLRT_DIV, 1);
  return sample_rate_div.read();
}

/**************************************************************************/
/*!
    @brief  Sets the divisor used to divide the base clock rate into a
            measurement rate
    @param  divisor
            The new clock divisor
*/
/**************************************************************************/
void AFS_MPU9250::setSampleRateDivisor(uint8_t divisor) {
  Adafruit_BusIO_Register sample_rate_div(i2c_dev, MPU9250_SMPLRT_DIV, 1);
  sample_rate_div.write(divisor);
}

/**************************************************************************/
/*!
*     @brief  Connects or disconects the I2C master pins to the main I2C pins
*     @param  bypass
              If `true` the I2C Master pins are connected to the main I2C pins,
              bypassing the I2C Master functions of the sensor
              If `false` the I2C Master pins are controlled by the I2C master
              functions of the sensor
*/
/**************************************************************************/
void AFS_MPU9250::setI2C_Bypass(bool bypass) {
  Adafruit_BusIO_Register int_pin_config(i2c_dev, MPU9250_INT_PIN_CFG);
  Adafruit_BusIO_RegisterBits i2c_bypass(&int_pin_config, IPC_BYPASS_EN);

  Adafruit_BusIO_Register user_ctrl(i2c_dev, MPU9250_USER_CTRL);
  Adafruit_BusIO_RegisterBits i2c_master_enable(&user_ctrl, UC_I2C_MST_EN);

  i2c_bypass.write(bypass);
  DBG_SERIAL("bypass:");
  DBG_SERIAL(bypass);
  DBG_SERIAL(" master now:");
  DBG_SERIAL((i2c_master_enable.read()) ? "true" : "false");
  DBG_SERIAL("\n");
  i2c_master_enable.write(!bypass); // 0-disable master (bypass mode) 1-master
}

/**************************************************************************/
/*!
*     @brief  Connects or disconects the I2C master pins to the main I2C pins
*     @param  bypass FIXME
              If `true` the I2C Master pins are connected to the main I2C pins,
              bypassing the I2C Master functions of the sensor
              If `false` the I2C Master pins are controlled by the I2C master
              functions of the sensor
*/
/**************************************************************************/
int AFS_MPU9250::getI2C_Bypass(void) {
  Adafruit_BusIO_Register int_pin_cfg_reg(i2c_dev, MPU9250_INT_PIN_CFG);
  Adafruit_BusIO_RegisterBits i2c_bypass(&int_pin_cfg_reg, IPC_BYPASS_EN);

/* The master enable should probably be checked first. If disabled, the bypass
   bit will be DON'T CARE. TODO - test.
  Adafruit_BusIO_Register user_ctrl_reg(i2c_dev, MPU9250_USER_CTRL);
  Adafruit_BusIO_RegisterBits i2c_master_enable(&user_ctrl_reg, UC_I2C_MST_EN);
  i2c_master_enable.read(bypass);
 */

  return i2c_bypass.read();
}

/**************************************************************************/
/*!
    @brief Gets the acceleration measurement range.
    @return  The acceleration measurement range
*/
/**************************************************************************/
MPU9250_accel_range_t AFS_MPU9250::getAccelerometerRange(void) {
  Adafruit_BusIO_Register accel_config(i2c_dev, MPU9250_ACCEL_CONFIG, 1);
  Adafruit_BusIO_RegisterBits accel_range(&accel_config, AC_ACCEL_FS_SEL);
  return (MPU9250_accel_range_t)accel_range.read();
}

/**************************************************************************/
/*!
    @brief Sets the accelerometer measurement range
    @param  new_range
            The new range to set. Must be a `MPU9250_accel_range_t`
*/
/**************************************************************************/
void AFS_MPU9250::setAccelerometerRange(MPU9250_accel_range_t new_range) {
  Adafruit_BusIO_Register accel_config(i2c_dev, MPU9250_ACCEL_CONFIG, 1);
  Adafruit_BusIO_RegisterBits accel_range(&accel_config, AC_ACCEL_FS_SEL);
  accel_range.write(new_range);
}
/**************************************************************************/
/*!
    @brief Gets the gyroscope measurement range
    @return  The `MPU9250_gyro_range_t` gyroscope measurement range
*/
/**************************************************************************/
MPU9250_gyro_range_t AFS_MPU9250::getGyroRange(void) {
  Adafruit_BusIO_Register gyro_config(i2c_dev, MPU9250_GYRO_CONFIG, 1);
  Adafruit_BusIO_RegisterBits gyro_range(&gyro_config, GC_GYRO_FS_SEL);
  return (MPU9250_gyro_range_t)gyro_range.read();
}

/**************************************************************************/
/*!
*     @brief  Sets how the interrupt is cleared
*     @param  clear_on_any_read
              If `true`  clears interrupt on any read operation
              If `false` clears interrupt when intr_status register is read
*/
/**************************************************************************/
void AFS_MPU9250::setClearIntrOnRead(bool clear_on_any_read) {
  Adafruit_BusIO_Register int_pin_config(i2c_dev, MPU9250_INT_PIN_CFG, 1);
  Adafruit_BusIO_RegisterBits int_latch(&int_pin_config, IPC_INT_ANYRD_2CLEAR);
  int_latch.write(clear_on_any_read?1:0);
}

/**************************************************************************/
/*!
*     @brief  Sets the motion interrupt
*     @param  active
              If `true` motion interrupt will activate based on thr and dur
              If `false` motion interrupt will be disabled
*/
/**************************************************************************/
void AFS_MPU9250::setMotionInterrupt(bool active) {
  Adafruit_BusIO_Register int_enable(i2c_dev, MPU9250_INT_ENABLE, 1);
  Adafruit_BusIO_RegisterBits int_motion(&int_enable, IE_WOM_EN);
  int_motion.write(active);
}

/**************************************************************************/
/*!
*     @brief  Sets the data ready interrupt
*     @param  enable_dri
              If `true`  enables the data ready interrupt
              If `false` disables the data ready interrupt
*/
/**************************************************************************/
void AFS_MPU9250::setDataReadyInterrupt(bool enable_dri) {
  Adafruit_BusIO_Register int_enable(i2c_dev, MPU9250_INT_ENABLE, 1);
  Adafruit_BusIO_RegisterBits int_data_rdy(&int_enable, IE_RAW_RDY_EN);
  int_data_rdy.write(enable_dri?1:0);
}

/**************************************************************************/
/*!
 *     @brief  Gets motion interrupt status
 *     @return  motion_interrupt
 */
/**************************************************************************/
bool AFS_MPU9250::getMotionInterruptStatus(void) {
  Adafruit_BusIO_Register status(i2c_dev, MPU9250_INT_STATUS, 1);
  Adafruit_BusIO_RegisterBits motion(&status, IS_WOM_INT);
  return (bool)motion.read();
}

/**************************************************************************/
/*!
    @brief Sets clock source.
    @param  new_clock
            The clock source to set. Must be a `MPU9250_clock_select_t`
*/
/**************************************************************************/
void AFS_MPU9250::setClock(MPU9250_clock_select_t new_clock) {
  Adafruit_BusIO_Register pwr_mgmt(i2c_dev, MPU9250_PWR_MGMT_1, 1);
  Adafruit_BusIO_RegisterBits clock_select(&pwr_mgmt, PM1_CLK_SEL);
  clock_select.write(new_clock);
}

/**************************************************************************/
/*!
    @brief Gets clock source.
    @return  The current `MPU9250_clock_select_t` clock source
*/
/**************************************************************************/
MPU9250_clock_select_t AFS_MPU9250::getClock(void) {
  Adafruit_BusIO_Register pwr_mgmt(i2c_dev, MPU9250_PWR_MGMT_1, 1);
  Adafruit_BusIO_RegisterBits clock_select(&pwr_mgmt, PM1_CLK_SEL);
  return (MPU9250_clock_select_t)clock_select.read();
}

/**************************************************************************/
/*!
 *     @brief  Sets the location that the FSYNC pin sample is stored
 *     @return fsync_output
 */
/**************************************************************************/
MPU9250_fsync_out_t AFS_MPU9250::getFsyncSampleOutput(void) {
  Adafruit_BusIO_Register config(i2c_dev, MPU9250_CONFIG);
  Adafruit_BusIO_RegisterBits fsync_out(&config, CF_EXT_SYNC_SET);
  return (MPU9250_fsync_out_t)fsync_out.read();
}

/**************************************************************************/
/*!
*     @brief  Sets the location that the FSYNC pin sample is stored
*     @param  fsync_output
              a `MPU9250_fsync_out_t` to specify the LSB of which data register
              should be used to store the state of the FSYNC pin
*/
/**************************************************************************/
void AFS_MPU9250::setFsyncSampleOutput(MPU9250_fsync_out_t fsync_output) {
  Adafruit_BusIO_Register config(i2c_dev, MPU9250_CONFIG);
  Adafruit_BusIO_RegisterBits fsync_out(&config, CF_EXT_SYNC_SET);
  fsync_out.write(fsync_output);
}

/**************************************************************************/
/*!
    @brief Sets the gyroscope measurement range
    @param  new_range
            The new range to set. Must be a `MPU9250_gyro_range_t`
*/
/**************************************************************************/
void AFS_MPU9250::setGyroRange(MPU9250_gyro_range_t new_range) {
  Adafruit_BusIO_Register gyro_config(i2c_dev, MPU9250_GYRO_CONFIG);
  Adafruit_BusIO_RegisterBits gyro_range(&gyro_config, GC_GYRO_FS_SEL);
  gyro_range.write(new_range);
}

/**************************************************************************/
/*!
 *     @brief  Gets bandwidth of the Digital Low Pass Filter
 *     @return  The current `MPU9250_gyro_dlpf_t` filter bandwidth
 */
/**************************************************************************/
MPU9250_gyro_dlpf_t AFS_MPU9250::getGyroFilterBandwidth(void) {
  Adafruit_BusIO_Register config(i2c_dev, MPU9250_CONFIG);
  Adafruit_BusIO_RegisterBits filter_config(&config, CF_DLPF_CFG);
  return (MPU9250_gyro_dlpf_t)filter_config.read();
}

/**************************************************************************/
/*!
 *    @brief Sets the bandwidth of the Digital Low-Pass Filter
 *    @param bandwidth the new `MPU9250_gyro_dlpf_t` bandwidth
 */
/**************************************************************************/
void AFS_MPU9250::setGyroFilterBandwidth(MPU9250_gyro_dlpf_t bandwidth,
                                         gyro_fchoice_t gfchoice) {
  Adafruit_BusIO_Register config(i2c_dev, MPU9250_CONFIG);
  Adafruit_BusIO_RegisterBits filter_config(&config, CF_DLPF_CFG);
  filter_config.write(bandwidth);
  Adafruit_BusIO_Register gyro_cfg_reg(i2c_dev, MPU9250_GYRO_CONFIG);
  Adafruit_BusIO_RegisterBits gyro_fchoice(&gyro_cfg_reg, GC_FCHOICE_B);
  gyro_fchoice.write(~gfchoice); // convert fchoice to fchoice_b (inverted)
}

/**************************************************************************/
/*!
 *     @brief  Gets bandwidth of the Digital Low Pass Filter
 *     @return  The current `MPU9250_lowpass_t` filter bandwidth
 */
/**************************************************************************/
MPU9250_accel_dlpf_t AFS_MPU9250::getAccelFilterBandwidth(void) {
  Adafruit_BusIO_Register ac2_config(i2c_dev, MPU9250_ACCEL_CONFIG_2);
  Adafruit_BusIO_RegisterBits filter_config(&ac2_config, AC2_A_DLPFCFG);
  return (MPU9250_accel_dlpf_t)filter_config.read();
}

/**************************************************************************/
/*!
 *    @brief Sets the bandwidth of the Digital High-Pass Filter
 *    @param bandwidth the new `MPU9250_highpass_t` bandwidth
 */
/**************************************************************************/
void AFS_MPU9250::setAccelFilterBandwidth(MPU9250_accel_dlpf_t bandwidth,
                                          accel_fchoice_t afchoice) {
  Adafruit_BusIO_Register ac2_config(i2c_dev, MPU9250_ACCEL_CONFIG_2);
  Adafruit_BusIO_RegisterBits filter_config(&ac2_config, AC2_A_DLPFCFG);
  filter_config.write(bandwidth);
  Adafruit_BusIO_RegisterBits accel_fchoice(&ac2_config, AC2_ACCEL_FCHOICE_B);
  accel_fchoice.write(~afchoice); // convert fchoice to fchoice_b (inverted)
}

/**************************************************************************/
/*!
*     @brief  Sets the polarity of the INT pin when active
*     @param  active_low
              If `true` the pin will be low when an interrupt is active
              If `false` the pin will be high when an interrupt is active
*/
/**************************************************************************/
void AFS_MPU9250::setInterruptPinPolarity(bool active_low) {
  Adafruit_BusIO_Register int_pin_config(i2c_dev, MPU9250_INT_PIN_CFG);
  Adafruit_BusIO_RegisterBits int_level(&int_pin_config, IPC_ACTL);
  int_level.write(active_low);
}

/**************************************************************************/
/*!
*     @brief  Sets the latch behavior of the INT pin when active
*     @param  held
              If `true` the pin will remain held until cleared
              If `false` the pin will reset after a 50us pulse
*/
/**************************************************************************/
void AFS_MPU9250::setInterruptPinLatch(bool held) {
  Adafruit_BusIO_Register int_pin_config(i2c_dev, MPU9250_INT_PIN_CFG, 1);
  Adafruit_BusIO_RegisterBits int_latch(&int_pin_config, IPC_LATCH_INT_EN);
  int_latch.write(held);
}

/**************************************************************************/
/*!
 *     @brief  Sets the motion detection threshold
 *     @param  thr
 */
/**************************************************************************/
void AFS_MPU9250::setMotionDetectionThreshold(uint8_t thr) {
  Adafruit_BusIO_Register threshold(i2c_dev, MPU9250_WOM_THR, 1);
  threshold.write(thr);
}

/**************************************************************************/
/*!
*     @brief  Controls the sleep state of the sensor
*     @param  enable
              If `true` the sensor is put into a low power state
              and measurements are halted until sleep mode is deactivated
              Setting `false` wakes up the sensor from sleep mode,
              resuming measurements.
      @returns True or false on successful write
*/
/**************************************************************************/
bool AFS_MPU9250::enableSleep(bool enable) {
  Adafruit_BusIO_Register pwr_mgmt(i2c_dev, MPU9250_PWR_MGMT_1, 1);
  Adafruit_BusIO_RegisterBits sleep(&pwr_mgmt, PM1_SLEEP);
  return sleep.write(enable);
}

/**************************************************************************/
/*!
*     @brief  Controls sensor's 'Cycle' measurement mode
*     @param  enable
              If `true` the sensor will take measurements at the rate
              set by calling `setCycleRate`, sleeping between measurements.
              *Setting the sensor into 'Cycle' mode will have no effect
              if the sensor has been put into a sleep state with `enableSleep`
              Setting `false` returns the sensor to the normal
              measurement mode.
      @returns True or false on successful write
*/
/**************************************************************************/
bool AFS_MPU9250::enableCycle(bool enable) {
  Adafruit_BusIO_Register pwr_mgmt(i2c_dev, MPU9250_PWR_MGMT_1, 1);
  Adafruit_BusIO_RegisterBits cycle(&pwr_mgmt, PM1_CYCLE);
  return cycle.write(enable);
}

/**************************************************************************/
/*!
 *     @brief  Sets standbye mode for each of the gyroscope axes.
 *     @param  xAxisStandby
 *             If `true` the gyroscope stops sensing in the X-axis.
 *             Setting `false` resumes X-axis sensing.
 *     @param  yAxisStandby
 *             If `true` the gyroscope stops sensing in the Y-axis.
 *             Setting `false` resumes Y-axis sensing.
 *     @param  zAxisStandby
 *             If `true` the gyroscope stops sensing in the Z-axis.
 *             Setting `false` resumes Z-axis sensing.
 *     @return True if setting was successful, otherwise false.
 */
/**************************************************************************/
bool AFS_MPU9250::setGyroStandby(bool xAxisStandby, bool yAxisStandby,
                                 bool zAxisStandby) {
  Adafruit_BusIO_Register pwr_mgmt_2(i2c_dev, MPU9250_PWR_MGMT_2, 1);
  Adafruit_BusIO_RegisterBits gyro_stdby(&pwr_mgmt_2, PM2_DISABLE_GY);
  return gyro_stdby.write(xAxisStandby << 2 | yAxisStandby << 1 | zAxisStandby);
}

/**************************************************************************/
/*!
 *     @brief  Sets standbye mode for each of the accelerometer axes.
 *     @param  xAxisStandby
 *             If `true` the accelerometer stops sensing in the X-axis.
 *             Setting `false` resumes X-axis sensing.
 *     @param  yAxisStandby
 *             If `true` the accelerometer stops sensing in the Y-axis.
 *             Setting `false` resumes Y-axis sensing.
 *     @param  zAxisStandby
 *             If `true` the accelerometer stops sensing in the Z-axis.
 *             Setting `false` resumes Z-axis sensing.
 *     @return True if setting was successful, otherwise false.
 */
/**************************************************************************/
bool AFS_MPU9250::setAccelerometerStandby(bool xAxisStandby,
                                               bool yAxisStandby,
                                               bool zAxisStandby) {
  Adafruit_BusIO_Register pwr_mgmt_2(i2c_dev, MPU9250_PWR_MGMT_2);
  Adafruit_BusIO_RegisterBits accel_stdby(&pwr_mgmt_2, PM2_DISABLE_AC);
  return accel_stdby.write(xAxisStandby << 2 | yAxisStandby << 1 |
                           zAxisStandby);
}

/**************************************************************************/
/*!
 *     @brief  Sets disable mode for thermometer sensor.
 *     @param  enable
 *             If `true` the temperature sensor will stop taking measurements.
 *             Setting `false` returns the temperature sensor to the normal
 *             measurement mode.
 *     @return True if setting was successful, otherwise false.
 */
/**************************************************************************/
bool AFS_MPU9250::setTemperatureStandby(bool enable) {
  Adafruit_BusIO_Register pwr_mgmt(i2c_dev, MPU9250_PWR_MGMT_1);
  Adafruit_BusIO_RegisterBits temp_stdby(&pwr_mgmt, PM1_PD_STAT);
  return temp_stdby.write(1);
}

