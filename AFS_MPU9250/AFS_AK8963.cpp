/*!
 *  @file AFS_AK8963.cpp
 *
 *  @mainpage I2C Driver for AKM's AK8963 Magnetometer
 *
 *  @section intro_sec Introduction
 *
 * 	This is the AK8963 Magnetometer component of the MPU-9250 IMU library.
 *
 *  @section dependencies Dependencies
 *
 *  This library as a whole depends on the Adafruit BusIO & Wire libraries.
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
 *     v0.1 - First release
 */

#include <Adafruit_BusIO_Register.h>
#include <Wire.h>
#include "AFS_AK8963.h"

extern char* trFunction;
extern int   trLine;

/**************************************************************************/
/*!
    @brief  Constructor for the magnetometer sensor object.
    @param  host_device
            pointer to the IMU pass-through device
    @returns returns the AK8963_Magnetometer object
*/
/**************************************************************************/
AK8963_Magnetometer::AK8963_Magnetometer(IMU_Device* host_device) {
    imuHost = host_device;
}

/**************************************************************************/
/*!
    @brief  Initializes the magnetometer object using direct access I2C
            pass-through mode previously enabled by the host IMU.
    @param  sensorId
            The unique sensor ID provided by the host IMU.
    @param  in_pass_through_mode
            Verifies that the AK8963 is accessible via the MCU's I2C buss.
    @param  operation_mode
            Select op mode from magn_operation_mode_t.
            Def=CONTINUOUS_2, 100Hz
    @param  res_bits
            Set the resolution from magn_resolution_t.
            Def=16-bit
    @returns False if direct I2C comms fail. True otherwise.
*/
/**************************************************************************/
bool AK8963_Magnetometer::begin(uint8_t sensorId, bool in_pass_through_mode,
                                magn_operation_mode_t operation_mode,
                                magn_resolution_t res_bits) {
  sensor_id = sensorId;
  magn_i2c_device = new Adafruit_I2CDevice(AK8963_I2C_ADDRESS);
  if (not magn_i2c_device->begin()) {
    // Serial.print("mag i2cDev.begin() fail. Pass-through set?\n");
    return false;
  }
  // who am I test
  if (not test_who_am_i(in_pass_through_mode)) {
    // Serial.print("mag who_am_i() failed\n");
    return false;
  }
  // add soft reset if necessary
  if (not setResolution(res_bits, in_pass_through_mode)) {
    // Serial.print("mag setRes fail\n");
    return false;
  }
  if (not setMode(operation_mode, in_pass_through_mode)) {
    // Serial.print("mag setMode fail\n");
    return false;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Gets magnetometer sensor event data from the IMU
    @param  event
            Pointer to the Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
bool AK8963_Magnetometer::getEvent(sensors_event_t* event) {
  // copy the last event from the host IMU
  imuHost->fillMagnEvent(event, imuHost->last_event_timestamp);
  return true;
}

/**************************************************************************/
/*!
 *    @brief  Gets sensor_t data from the AK8963
 *    @param  sensor
 *            Pointer to the sensor_t object
 */
/**************************************************************************/
void AK8963_Magnetometer::getSensor(sensor_t* sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));
  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "AK8963", sizeof(sensor->name) - 1);
  sensor->version = 1;
  sensor->sensor_id = sensor_id;
  sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay = 0;
  sensor->min_value = -4190; // microtTesla (uT)
  sensor->max_value = +4190;
  sensor->resolution = resolution_uT;
}

/**************************************************************************/
/*!
 *    @brief  Sets the sample resolution to 14- or 16-bits and sets
 *            the conversion factor to uT
 *    @param  res_bits
 *            Enumeration for 14- or 16-bit resolution, magn_resolution_t
 *    @param   in_pass_through_mode
 *            indicates that the IMU I2C buss is in pass-through mode
 *            and that the MCU can communicate directly with the AK8963
 *    @returns True on success, False otherwise
 */
/**************************************************************************/
bool AK8963_Magnetometer::setResolution(magn_resolution_t res_bits,
                                        bool in_pass_through_mode) {
  resolution_uT = 4190.0 / ((res_bits == RESOLUTION_16_BIT) ? 8190 : 32760);
  if (in_pass_through_mode) {
    Adafruit_BusIO_Register ct1_reg(magn_i2c_device, AK8963_CNTL1);
    Adafruit_BusIO_RegisterBits c_bit(&ct1_reg, C1_BIT);
    return c_bit.write(res_bits);
  } // else
  return imuHost->proxy_write(magn_i2c_device, AK8963_CNTL1, C1_BIT, res_bits);
}

/**************************************************************************/
/*!
 *    @brief  Sets the opereration mode
 *    @param  op_mode
 *            Enumeration operation modes, magn_operation_mode_t
 *    @param   in_pass_through_mode
 *            indicates that the IMU I2C buss is in pass-through mode
 *            and that the MCU can communicate directly with the AK8963
 *    @returns True on success, False otherwise
 */
/**************************************************************************/
bool AK8963_Magnetometer::setMode(magn_operation_mode_t op_mode,
                                  bool in_pass_through_mode) {
  if (in_pass_through_mode) {
    Adafruit_BusIO_Register ct1_reg(magn_i2c_device, AK8963_CNTL1);
    Adafruit_BusIO_RegisterBits c_mode(&ct1_reg, C1_MODE);
    return c_mode.write(op_mode);
  } // else
  return imuHost->proxy_write(magn_i2c_device, AK8963_CNTL1, C1_MODE, op_mode);
}

// test who am I register contents
/**************************************************************************/
/*!
 *    @brief  test who am I register contents
 *    @param  in_pass_through_mode
 *            indicates that the IMU I2C buss is in pass-through mode
 *            and that the MCU can communicate directly with the AK8963
 *    @returns True if in bypass mode and the chip ID matches,
               False otherwise
 */
/**************************************************************************/
bool AK8963_Magnetometer::test_who_am_i(bool in_pass_through_mode) {
  if (not in_pass_through_mode) return false;
  Adafruit_BusIO_Register wia_reg(magn_i2c_device, AK8963_WHO_AM_I);
  return (wia_reg.read() == AK8963_WHO_I_AM);
}
