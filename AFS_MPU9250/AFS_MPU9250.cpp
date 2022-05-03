/*!
 *  @file AFS_MPU9250.cpp
 *
 *  @mainpage MPU-9250 9-DoF Gyro, Accelerometer, Magnetometer sensor library
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for MPU-9250 9-DoF Gyro, Accelerometer, Magnetometer
 *
 * 	This is a library for varous MPU-9250 breakout boards (GY-5221, etc.).
 *
 *  @section dependencies Dependencies
 *
 *  This library depends on the Adafruit BusIO and Wire libraries
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

#include "AFS_MPU9250.h"

/**************************************************************************/
/*!
 *  @brief  Instantiates a new MPU9250 class
 */
/**************************************************************************/
AFS_MPU9250::AFS_MPU9250(void) {}

/**************************************************************************/
/*!
 *  @brief  Cleans up the MPU9250 class
 */
/**************************************************************************/
AFS_MPU9250::~AFS_MPU9250(void) {
  delete temp_sensor;
  delete magn_sensor;
  delete accel_sensor;
  delete gyro_sensor;
  delete i2c_dev;
}

/**************************************************************************/
/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address of the MPU-9250.
 *    @param  i2c_address_magn
 *            The I2C address of the AL8963 magnetometer.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @param sensor_id
 *            The user-defined ID to differentiate different sensors
 *    @return True if initialization was successful, otherwise false.
 */
/**************************************************************************/
bool AFS_MPU9250::begin(uint8_t i2c_address, uint8_t i2c_address_magn,
                        TwoWire* wire, int32_t sensor_id) {
  delete i2c_dev; // remove old interfaces
  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);
  if (!i2c_dev->begin()) {
    // Serial.print("9250 I2C.begin() fault\n");
    return false;
  }

  Adafruit_BusIO_Register chip_id(i2c_dev, MPU9250_WHO_AM_I, 1);
  // make sure we're talking to the right chip
  int id;
  if ((id = chip_id.read()) != MPU9250_DEVICE_ID) {
    // Serial.print("9250 chip ID fault: got 0x");
    // Serial.print(id, HEX);
    // Serial.print(" instead of 0x");
    // Serial.print(MPU9250_DEVICE_ID, HEX);
    // Serial.print("\n");
    return false;
  }

  // reset & init sensors
  if (not _init(sensor_id)) {
    // Serial.print("MP9250._init() fault\n");
    return false;
  }

  setI2C_Bypass(true); // put I2C in pass-through mode
  if (not magn_sensor->begin(_sensorid_magn, true)) {
    // defaults: 100Hz sampling, 16-bit resolution
    // Serial.print("magn.begin fail\n");
    return false;
  }
  setI2C_Bypass(false);

  // config magnetometer reads via slave 0 starting at MPU9250_EXT_SENS_DATA_00
  config_slave_regs(AK8963_I2C_ADDRESS, 0, AK8963_XOUT_L, AK63_RAW_BYTES_ST2);

  return true;
}

/**************************************************************************/
/*!
 *   @brief Initilizes the sensor
 *   @param sensor_id
 *          Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
 /**************************************************************************/

bool AFS_MPU9250::_init(int32_t sensor_id) {

  _sensorid_accel = sensor_id;
  _sensorid_gyro  = sensor_id + 1;
  _sensorid_temp  = sensor_id + 2;
  _sensorid_magn  = sensor_id + 3;

  reset();

  // set data ready interrupt
  setClearIntrOnRead(true);
  setInterruptPinPolarity(true); // active low
  setDataReadyInterrupt(true);

  // set gyro range & LPFilter
  // was setGyroFilterBandwidth(MPU9250_LOWPASS_GYRO_BW_250_HZ); // that's 0, no sample div
  // LPF option 1-7 to enable sample_rate_div
  setGyroFilterBandwidth(MPU9250_LOWPASS_GYRO_BW_184_HZ, MPU9250_GYRO_FCHOICE_DLPF);
  setGyroRange(MPU9250_RANGE_250_DEG);      // max resolution
  setAccelerometerRange(MPU9250_RANGE_2_G); // max resolution

  // set clock config to PLL with Gyro X reference
  Adafruit_BusIO_Register power_mgmt_1(i2c_dev, MPU9250_PWR_MGMT_1);
  Adafruit_BusIO_RegisterBits clk_src(&power_mgmt_1, PM1_CLK_SEL);
  // power_mgmt_1.write(0x01); // set clock config to PLL with Gyro X reference
  clk_src.write(0b001);
  delay(100);

  // respawn sensor_t objects
  delete temp_sensor;
  delete magn_sensor;
  delete accel_sensor;
  delete gyro_sensor;
  magnInstance = 1;   // may be used to associate slave I2C device instance
  magn_sensor = new AK8963_Magnetometer(this);
  temp_sensor = new AFS_MPU9250_Temp(this);
  gyro_sensor = new AFS_MPU9250_Gyro(this);
  accel_sensor = new AFS_MPU9250_Accelerometer(this);

  return true;
}

/**************************************************************************/
/*!
    @brief Resets registers to their initial value and resets the sensors'
    analog and digital signal paths.
*/
/**************************************************************************/
void AFS_MPU9250::reset(void) {
  Adafruit_BusIO_Register power_mgmt_1(i2c_dev, MPU9250_PWR_MGMT_1);
  Adafruit_BusIO_RegisterBits device_reset(&power_mgmt_1, PM1_H_RESET);
  // see register map page 41
  device_reset.write(1);             // reset
  while (device_reset.read() == 1) { // check for the post reset value
    delay(1);
  }
  delay(100);

  Adafruit_BusIO_Register sig_path_reset(i2c_dev, MPU9250_SIGNAL_PATH_RESET);
  Adafruit_BusIO_RegisterBits sensor_paths(&sig_path_reset, SPR_ALL_SENSOR_RST);
  // sig_path_reset.write(0x7); // reset Accel Gyro Temp paths
  sensor_paths.write(0b111);

  delay(100);
}

/******************* Adafruit_Sensor functions *****************/
/*!
 *  @brief  Updates the measurement data for all sensors simultaneously
 */
/**************************************************************************/
void AFS_MPU9250::_read(void) {
  // get raw sensor data
  uint8_t buffer[14+AK63_RAW_BYTES];
  Adafruit_BusIO_Register data_reg(i2c_dev, MPU9250_ACCEL_XOUT_H, 14+AK63_RAW_BYTES);
  data_reg.read(buffer, 14+AK63_RAW_BYTES);

  rawAccX = buffer[0] << 8 | buffer[1];
  rawAccY = buffer[2] << 8 | buffer[3];
  rawAccZ = buffer[4] << 8 | buffer[5];

  rawTemp = buffer[6] << 8 | buffer[7];

  rawGyroX = buffer[8]  << 8 | buffer[9];
  rawGyroY = buffer[10] << 8 | buffer[11];
  rawGyroZ = buffer[12] << 8 | buffer[13];

  rawMagnX = buffer[14] << 8 | buffer[15];
  rawMagnY = buffer[16] << 8 | buffer[17];
  rawMagnZ = buffer[18] << 8 | buffer[19];

  temperature = (rawTemp / 340.0) + 36.53; // sensor_event.temp in degC
  // datasheet formula:
  // temperature = ((rawTemp - ROOM_TEMP_OFFSET)/TEMP_SENSITIVITY) + 21.0;

  MPU9250_accel_range_t accel_range = getAccelerometerRange();

  float accel_scale = 1;
  if (accel_range == MPU9250_RANGE_16_G)
    accel_scale = 2048;
  if (accel_range == MPU9250_RANGE_8_G)
    accel_scale = 4096;
  if (accel_range == MPU9250_RANGE_4_G)
    accel_scale = 8192;
  if (accel_range == MPU9250_RANGE_2_G)
    accel_scale = 16384;

  // setup range dependant scaling
  accX = ((float)rawAccX) / accel_scale;
  accY = ((float)rawAccY) / accel_scale;
  accZ = ((float)rawAccZ) / accel_scale;

  MPU9250_gyro_range_t gyro_range = getGyroRange();

  float gyro_scale = 1;
  if (gyro_range == MPU9250_RANGE_250_DEG)
    gyro_scale = 131;
  if (gyro_range == MPU9250_RANGE_500_DEG)
    gyro_scale = 65.5;
  if (gyro_range == MPU9250_RANGE_1000_DEG)
    gyro_scale = 32.8;
  if (gyro_range == MPU9250_RANGE_2000_DEG)
    gyro_scale = 16.4;

  gyroX = ((float)rawGyroX) / gyro_scale;
  gyroY = ((float)rawGyroY) / gyro_scale;
  gyroZ = ((float)rawGyroZ) / gyro_scale;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event, Adafruit Unified Sensor format
    @param  accel
            Pointer to an Adafruit Unified sensor_event_t object to be filled
            with acceleration event data.
    @param  magn
            Pointer to an Adafruit Unified sensor_event_t object to be filled
            with magnetometer event data.
    @param  gyro
            Pointer to an Adafruit Unified sensor_event_t object to be filled
            with gyroscope event data.
    @param  temp
            Pointer to an Adafruit Unified sensor_event_t object to be filled
            with temperature event data.
    @return True on successful read
*/
/**************************************************************************/
bool AFS_MPU9250::getEvent(sensors_event_t* accel, sensors_event_t* magn,
                           sensors_event_t* gyro,  sensors_event_t* temp) {
  uint32_t timestamp = millis();
  last_event_timestamp = timestamp;
  _read(); // read raw, stores locally
  // convert to sensor_event units and save there
  fillTempEvent(temp, timestamp);
  fillAccelEvent(accel, timestamp);
  fillGyroEvent(gyro, timestamp);
  fillMagnEvent(magn, timestamp);
  return true;
}

void AFS_MPU9250::fillTempEvent(sensors_event_t* temp, uint32_t timestamp) {
  memset(temp, 0, sizeof(sensors_event_t));
  temp->version = sizeof(sensors_event_t);
  temp->sensor_id = _sensorid_temp;
  temp->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  temp->timestamp = timestamp;
  temp->temperature = temperature;
}

void AFS_MPU9250::fillAccelEvent(sensors_event_t* accel, uint32_t timestamp) {
  memset(accel, 0, sizeof(sensors_event_t));
  accel->version = 1;
  accel->sensor_id = _sensorid_accel;
  accel->type = SENSOR_TYPE_ACCELEROMETER;
  accel->timestamp = timestamp;
  accel->acceleration.x = accX * SENSORS_GRAVITY_STANDARD;
  accel->acceleration.y = accY * SENSORS_GRAVITY_STANDARD;
  accel->acceleration.z = accZ * SENSORS_GRAVITY_STANDARD;
}

void AFS_MPU9250::fillGyroEvent(sensors_event_t* gyro, uint32_t timestamp) {
  memset(gyro, 0, sizeof(sensors_event_t));
  gyro->version = 1;
  gyro->sensor_id = _sensorid_gyro;
  gyro->type = SENSOR_TYPE_GYROSCOPE;
  gyro->timestamp = timestamp;
  gyro->gyro.x = gyroX * SENSORS_DPS_TO_RADS;
  gyro->gyro.y = gyroY * SENSORS_DPS_TO_RADS;
  gyro->gyro.z = gyroZ * SENSORS_DPS_TO_RADS;
}

void AFS_MPU9250::fillMagnEvent(sensors_event_t* magn, uint32_t timestamp) {
  memset(magn, 0, sizeof(sensors_event_t));
  // from external sensor data registers
  magn->version = 1;
  magn->sensor_id = _sensorid_magn;
  magn->type = SENSOR_TYPE_MAGNETIC_FIELD;
  magn->timestamp = timestamp;
  magn->magnetic.x = magn_sensor->resolution_uT * rawMagnX;
  magn->magnetic.y = magn_sensor->resolution_uT * rawMagnY;
  magn->magnetic.z = magn_sensor->resolution_uT * rawMagnZ;
}

/*!
  @brief  Gets an Adafruit Unified Sensor object for the temp sensor component
  @return Adafruit_Sensor pointer to temperature sensor
*/
Adafruit_Sensor* AFS_MPU9250::getTemperatureSensor(void) {
  return temp_sensor;
}

/*!
 *  @brief  Gets an Adafruit Unified Sensor object for the
 *          magnetometer sensor component
 *  @return Adafruit_Sensor pointer to magnetometer sensor
 */
Adafruit_Sensor* AFS_MPU9250::getMagnetometerSensor(void) {
  return magn_sensor;
}


/*!
    @brief  Gets an Adafruit Unified Sensor object for the accelerometer
    sensor component
    @return Adafruit_Sensor pointer to accelerometer sensor
 */
Adafruit_Sensor* AFS_MPU9250::getAccelerometerSensor(void) {
  return accel_sensor;
}

/*!
    @brief  Gets an Adafruit Unified Sensor object for the gyro sensor component
    @return Adafruit_Sensor pointer to gyro sensor
 */
Adafruit_Sensor* AFS_MPU9250::getGyroSensor(void) { return gyro_sensor; }

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the MPU9250's gyroscope sensor
*/
/**************************************************************************/
void AFS_MPU9250_Gyro::getSensor(sensor_t* sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "MPU9250_G", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_GYROSCOPE;
  sensor->min_delay = 0;
  sensor->min_value = -34.91; /* -000 dps -> rad/s (radians per second) */
  sensor->max_value = +34.91;
  sensor->resolution = 1.332e-4; /* 131.5 LSB/DPS */
}

/**************************************************************************/
/*!
    @brief  Gets the gyroscope as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
bool AFS_MPU9250_Gyro::getEvent(sensors_event_t* event) {
  _theMPU9250->_read();
  _theMPU9250->fillGyroEvent(event, millis());
  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the MPU9250's accelerometer
*/
/**************************************************************************/
void AFS_MPU9250_Accelerometer::getSensor(sensor_t* sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "MPU9250_A", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay = 0;
  sensor->min_value = -156.9064F; /*  -16g = 156.9064 m/s^2  */
  sensor->max_value = 156.9064F;  /* 16g = 156.9064 m/s^2  */
  sensor->resolution = 0.061;     /* 0.061 mg/LSB at +-2g */
}

/**************************************************************************/
/*!
    @brief  Gets the accelerometer as a standard sensor event
    @param  event Sensor event object that will be populatedx
    @returns True
*/
/**************************************************************************/
bool AFS_MPU9250_Accelerometer::getEvent(sensors_event_t* event) {
  _theMPU9250->_read();
  _theMPU9250->fillAccelEvent(event, millis());
  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the MPU9250's tenperature
*/
/**************************************************************************/
void AFS_MPU9250_Temp::getSensor(sensor_t* sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "MPU9250_T", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  sensor->min_delay = 0;
  sensor->min_value = -40;
  sensor->max_value = 105;
  sensor->resolution = 0.00294; /* 340 LSB/C => 1/340 C/LSB */
}

/**************************************************************************/
/*!
    @brief  Gets the temperature as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
bool AFS_MPU9250_Temp::getEvent(sensors_event_t* event) {
  _theMPU9250->_read();
  _theMPU9250->fillTempEvent(event, millis());
  return true;
}

/**************************************************************************/
/*!
    @brief  Writes data to the magnetometer via pass-through mode.
    @param  i2c_device
            The Adafruit_I2CDevice object for the slave device.
    @param  sregister
            The starting slave register to write to.
    @param  bitwidth
            The number of bits to be written.
    @param  shift
            The bit offset into the register.
    @param  value
            The value to write to the bitfield.
    @param  nbytes
            The number bytes to write.
    @returns True if the write was successful, False otherwise.
*/
/**************************************************************************/
bool AFS_MPU9250::proxy_write(Adafruit_I2CDevice* i2c_device,
                              uint8_t sregister, uint8_t bitwidth,
                              uint8_t shift, uint8_t value,
                              uint8_t nbytes) {
  // Serial.print("in proxy_write sreg=");
  // Serial.print(sregister, HEX);
  // Serial.print("\n");
  int initialBypassMode = getI2C_Bypass();
  if (not initialBypassMode) setI2C_Bypass(1);
  Adafruit_BusIO_Register slave_register(i2c_dev, sregister, nbytes);
  Adafruit_BusIO_RegisterBits bit_field(&slave_register, bitwidth, shift);
  if (not initialBypassMode) setI2C_Bypass(0);
  return bit_field.write(value);
}

/**************************************************************************/
/*!
    @brief  Reads data from the magnetometer via pass-through mode.
    @param  i2c_device
            The Adafruit_I2CDevice object for the slave device.
    @param  sregister
            The starting slave register to be read from.
    @param  bitwidth
            The number of bits to be read.
    @param  shift
            The bit offset into the register.
    @param  nbytes
            The number bytes to be read.
    @returns True if the read was successful, False otherwise.
*/
/**************************************************************************/
uint32_t AFS_MPU9250::proxy_read(Adafruit_I2CDevice* i2c_device,
                                 uint8_t sregister, uint8_t bitwidth,
                                 uint8_t shift, uint8_t nbytes) {
  int initialBypassMode = getI2C_Bypass();
  if (not initialBypassMode) setI2C_Bypass(1);
  Adafruit_BusIO_Register slave_register(i2c_device, sregister, nbytes);
  Adafruit_BusIO_RegisterBits bit_field(&slave_register, bitwidth, shift);
  if (not initialBypassMode) setI2C_Bypass(0);
  return bit_field.read();
}

/**************************************************************************/
/*!
    @brief  Reads data from the magnetometer via pass-through mode.
    @param  i2c_device
            The Adafruit_I2CDevice object for the slave device.
    @param  sregister
            The starting slave register to be read from.
    @param  buffer
            The byte array to write read data to.
    @param  nbytes
            The number bytes to be read.
    @returns True if the read was successful, False otherwise.
*/
/**************************************************************************/
bool AFS_MPU9250::proxy_read(Adafruit_I2CDevice* i2c_device,
                             uint8_t start_register, uint8_t* buffer,
                             uint8_t nbytes) {
  Adafruit_BusIO_Register slave_register(i2c_device, start_register);
  return slave_register.read(buffer, nbytes);
}

/**************************************************************************/
/*!
    @brief  Configure registers for reading I2C slave devices.
    @param  sl_i2c_address
            I2C address of the slave device.
    @param  slave_number
            Slave number 0-5. Used as an offset for the other config registers.
    @param  sl_base_register
            The first register to be read on the slave device.
    @param  num_registers
            The number of registers to be read on the slave device.
    @returns True
*/
/**************************************************************************/
bool AFS_MPU9250::config_slave_regs(uint8_t sl_i2c_address, uint slave_number,
                  uint8_t sl_base_register, uint8_t num_registers) {
  if (slave_number > 4) return false;
  const int register_set_offset = 3;
  uint8_t reg_base = MPU9250_I2C_SLV0_ADDR + (register_set_offset * slave_number);
  uint8_t ctrl_offset = (slave_number != 4) ? 2 : 3; // stupid DO
  Adafruit_BusIO_Register slave_addr_reg(i2c_dev, reg_base);
  Adafruit_BusIO_Register slave_reg_reg(i2c_dev, reg_base+1);
  Adafruit_BusIO_Register slave_crtl_reg(i2c_dev, reg_base+ctrl_offset);
  /* config bit fields - for reference, but slower iterative read/write
  Adafruit_BusIO_RegisterBits ctrl_enable(&slave_crtl_reg, SL_I2C_SLV_EN);
  Adafruit_BusIO_RegisterBits ctrl_swap(&slave_crtl_reg, SL_I2C_SLV_BYTE_SWAP);
  Adafruit_BusIO_RegisterBits ctrl_dis(&slave_crtl_reg, SL_I2C_SLV_DIS);
  Adafruit_BusIO_RegisterBits ctrl_grp(&slave_crtl_reg, SL_I2C_SLV_GRP);
  Adafruit_BusIO_RegisterBits ctrl_read_len(&slave_crtl_reg, SL_I2C_SLV_LENG); */

  if (not slave_addr_reg.write(0x80 | sl_i2c_address)) { // read bit + slave address
    // Serial.print("Slave I/O config fault address reg\n");
    return false;
  }
  if (not slave_reg_reg.write(sl_base_register)) {
    // Serial.print("Slave I/O config fault register reg\n");
    return false;
  }

/* control fields
  enable  1         enable this slave I/O
  swap     1        swap mag LH byte order to HL
  dis wr    0       enable register writing
  group      ?      even grouping
  rd_len      xxxx  length param
          --------
        0b1100xxxx                              */

  uint8_t ctrl_byte = 0b11000000 | (num_registers & 0xF);
  ctrl_byte |= (sl_base_register % 2) << 4;  // set grouping boundary, 0=even
  if (not slave_crtl_reg.write(ctrl_byte)) {
    // Serial.print("Slave I/O config fault control reg"\n);
    return false;
  }
  return true;
}
