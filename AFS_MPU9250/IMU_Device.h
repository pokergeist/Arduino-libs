/*!
 *  @file IMU_Device.h
 *
 *  @mainpage IMU_Device Interface
 *
 *  @section intro_sec Introduction
 *
 * 	This is a virtual class (interface) that attempts to abstract the
 *  host IMU somewhat.
 *
 *  @section dependencies Dependencies
 *
 *  This library depends on the Adafruit BusIO and Unified Sensor libraries.
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
 
 #ifndef IMU_DEVICE_H
#define IMU_DEVICE_H

#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>

class IMU_Device {
 public:
  virtual ~IMU_Device(void) { }

  // pure virtual methods

  /* Proxy methods are provided so downstream device data can be accessed
   * accessed by the host IMU via pass-through comms or a slave I2C device
   * comminication mechanism. */

  virtual bool proxy_write(Adafruit_I2CDevice *i2c_device, uint8_t sregister,
                   uint8_t bitwidth, uint8_t shift, uint8_t value,
                   uint8_t nbytes=1) = 0;
  virtual uint32_t proxy_read(Adafruit_I2CDevice *i2c_device, uint8_t sregister,
                      uint8_t bitwidth, uint8_t shift,
                      uint8_t nbytes=1) = 0;
  virtual bool proxy_read(Adafruit_I2CDevice *i2c_device,
                          uint8_t start_register, uint8_t *buffer,
                          uint8_t nbytes = 1) = 0;

  virtual void fillTempEvent(sensors_event_t *temp, uint32_t timestamp) = 0;

  virtual void fillAccelEvent(sensors_event_t *accel, uint32_t timestamp) = 0;

  virtual void fillGyroEvent(sensors_event_t *gyro, uint32_t timestamp) = 0;

  virtual void fillMagnEvent(sensors_event_t *magn, uint32_t timestamp) = 0;

  uint32_t last_event_timestamp;
};

#endif /* _H */
