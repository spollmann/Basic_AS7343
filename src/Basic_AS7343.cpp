/**
**  (C) 2024 Steven Pollmann
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
** The above copyright notice and this permission notice shall be
** included in all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
** EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
** OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
** IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
** DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
** OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
**  USE OR OTHER DEALINGS IN THE SOFTWARE.
**  
**  The Following Copyright notices also apply.
**/

/****************************************************************************
** Copyright (C) 2020 MikroElektronika d.o.o.
** Contact: https://www.mikroe.com/contact
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
** The above copyright notice and this permission notice shall be
** included in all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
** EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
** OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
** IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
** DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
** OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
**  USE OR OTHER DEALINGS IN THE SOFTWARE.
****************************************************************************/
/* Software License Agreement (BSD License)

Copyright (c) 2019 Bryan Siepert for Adafruit Industries
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holders nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */


/*!
 *  @file Basic_AS7343.cpp
 * 
 * 
 *
 * 	I2C Driver for the Library for the AS7343 14-Channel Spectral Sensor
 *   
 */

#include "Arduino.h"
#include <Wire.h>

#include "Basic_AS7343.h"

/**
 * @brief Construct a new Basic_AS7343::Basic_AS7343 object
 *
 */
Basic_AS7343::Basic_AS7343(void) {}

/**
 * @brief Destroy the Basic_AS7343::Basic_AS7343 object
 *
 */
Basic_AS7343::~Basic_AS7343(void) {
}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @param  sensor_id
 *            The unique ID to differentiate the sensors from others
 *    @return True if initialization was successful, otherwise false.
 */
bool Basic_AS7343::begin(uint8_t i2c_address, TwoWire *wire,
                            int32_t sensor_id) {
  if (i2c_dev) {
    delete i2c_dev; // remove old interface
  }

  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    return false;
  }
  bool isInit;
  isInit = _init(sensor_id);
  return isInit;
}


/*!  @brief Initializer for post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
bool Basic_AS7343::_init(int32_t sensor_id) {

  // silence compiler warning - variable may be used in the future
  (void)sensor_id;
  setBank(true); // Access 0x58 to 0x66 
  Adafruit_BusIO_Register chip_id =
      Adafruit_BusIO_Register(i2c_dev, AS7343_WHOAMI);
  // make sure we're talking to the right chip
  if (chip_id.read() != AS7343_CHIP_ID) {
    setBank(false);
    return false;
  }
  setBank(false);
  powerEnable(false);
  delay(100);
  powerEnable(true);
  delay(100);
  return setAutoChannelReadout(AS7343_18CHANNEL);
}

/*May modify the following function to include default integration times, wait times etc, interrupts */
/*!  @brief Setup the AS7343 to a default configuration
 *   @returns True if chip has been setup correctly
 */
bool Basic_AS7343::setDefaultConfig() {
  bool errorFlag = 0;
  powerEnable(false);
  delay(100);
  //set CFG_20 to AUTO_SMUX_18CH
  powerEnable(true);
  delay(100);
  return setAutoChannelReadout(AS7343_18CHANNEL);
}



/********************* EXAMPLE EXTRACTS **************/
// maybe return a typedef enum

/**
 * @brief Returns the data for a given channel, read from the register directly
 *
 * @param channel The Color channel to read
 * @return uint16_t The measured data for the currently configured sensor
 */
uint16_t Basic_AS7343::readChannel(as7343_color_channel_t channel) {
  // each channel has two bytes, so offset by two for each next channel
  Adafruit_BusIO_Register channel_data_reg = Adafruit_BusIO_Register(
      i2c_dev, (AS7343_DATA_00_L + 2 * channel), 2, LSBFIRST);

  return channel_data_reg.read();
}

/**
 * @brief Returns the reading data for the specified color channel, stored in this class
 *
 *  call `readAllChannels` before reading to update the stored readings
 *
 * @param channel The color sensor channel to read
 * @return uint16_t The measured data for the selected sensor channel
 */
uint16_t Basic_AS7343::getChannel(as7343_color_channel_t channel) {
  return _channel_readings[channel];
}

/**
 * @brief fills the provided buffer with the current measurements for Spectral
 * channels F1-8, FZ, FXL, FY, FD, 2xVIS and NIR
 *
 * @param readings_buffer Pointer to a buffer of length 18 or more to fill with
 * sensor data
 * @return true: success false: failure
 */
bool Basic_AS7343::readAllChannels(uint16_t *readings_buffer) {
  enableSpectralMeasurement(true); // Start integration
  delayForData(0);                 // I'll wait for you for all time
  Adafruit_BusIO_Register channel_data_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_DATA_00_L, 2);
  bool readStatus;

  readStatus = channel_data_reg.read((uint8_t *)readings_buffer, 36); 
  enableSpectralMeasurement(false); // stop integration, so that next calls are not bufferred with data already.
  return readStatus;
}

/**
 * @brief starts the process of getting readings from all channels without using
 * delays
 *
 * @return true: success false: failure (a bit arbitrary)
 */
bool Basic_AS7343::startReading(void) {
  _readingState = AS7343_WAITING_START; // Start the measurement please
  checkReadingProgress();               // Call the check function to start it
  return true;
}

/**
 * @brief runs the process of getting readings from all channels without using
 * delays.  Should be called regularly (ie. in loop()) Need to call
 * startReading() to initialise the process Need to call getAllChannels() to
 * transfer the data into an external buffer
 *
 * @return true: reading is complete false: reading is incomplete (or failed)
 */
bool Basic_AS7343::checkReadingProgress() {
  if (_readingState == AS7343_WAITING_START) {
    enableSpectralMeasurement(true); // Start integration
    _readingState = AS7343_WAITING_DATA;
    return false;
  }

  if (!getIsDataReady() || _readingState == AS7343_WAITING_DONE)
    return false;

  if (_readingState == AS7343_WAITING_DATA) // Check of getIsDataRead() is already done
  {
    Adafruit_BusIO_Register channel_data_reg =
        Adafruit_BusIO_Register(i2c_dev, AS7343_DATA_00_L, 2);

    channel_data_reg.read((uint8_t *)_channel_readings, 36);
    _readingState = AS7343_WAITING_DONE;
    enableSpectralMeasurement(false);
    return true;
  }

  return false;
}

/**
 * @brief transfer all the values from the private result buffer into one
 * nominated
 *
 * @param readings_buffer Pointer to a buffer of length 18 (THERE IS NO ERROR
 * CHECKING...BE WARNED!)
 *
 * @return true: success false: failure
 */
bool Basic_AS7343::getAllChannels(uint16_t *readings_buffer) {
  for (int i = 0; i < 18; i++)
    readings_buffer[i] = _channel_readings[i];
  return true;
}

/**
 * @brief Delay while waiting for data, with option to time out and recover
 *
 * @param waitTime the maximum amount of time to wait
 * @return none
 */
void Basic_AS7343::delayForData(int waitTime) {
  if (waitTime == 0) // Wait forever
  {
    while (!getIsDataReady()) {
      delay(1);
    }
    return;
  }
  if (waitTime > 0) // Wait for that many milliseconds
  {
    uint32_t elapsedMillis = 0;
    while (!getIsDataReady() && elapsedMillis < (uint32_t)waitTime) {
      delay(1);
      elapsedMillis++;
    }
    return;
  }
  if (waitTime < 0) {
    // For future use?
    return;
  }
}

/**
 * @brief Take readings for F1-8, FY, FZ, FXL, FD, 2xVIS and NIR and store them in a buffer
 *
 * @return true: success false: failure
 */
bool Basic_AS7343::readAllChannels(void) {
  return readAllChannels(_channel_readings);
}


/**
 * @brief Sets the power state of the sensor
 *
 * @param enable_power true: on false: off
 */
void Basic_AS7343::powerEnable(bool enable_power) {
  Adafruit_BusIO_Register enable_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_ENABLE);
  Adafruit_BusIO_RegisterBits pon_en =
      Adafruit_BusIO_RegisterBits(&enable_reg, 1, 0);
  pon_en.write(enable_power);
}


/**
 * @brief Do a software reset to force a power on reset
 */
void Basic_AS7343::softwareReset(){
  Adafruit_BusIO_Register control_reg = Adafruit_BusIO_Register(i2c_dev, AS7343_CONTROL);
  Adafruit_BusIO_RegisterBits sw_reset_en = Adafruit_BusIO_RegisterBits(&control_reg, 1, 3);
  sw_reset_en.write(true);
}


/**
 * @brief Sets the wait time enable status of the sensor.  Must be set before enabling Spectral Measurements (SP_EN in AS7343_ENABLE register)
 *
 * @param enable_waitTime true: on false: off
 */
bool Basic_AS7343::enableWaitTime(bool enable_waitTime){
  Adafruit_BusIO_Register enable_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_ENABLE);
  Adafruit_BusIO_RegisterBits wen =
      Adafruit_BusIO_RegisterBits(&enable_reg, 1, 3);
  return wen.write(enable_waitTime);
}



/**
 * @brief Disable Spectral reading, flicker detection, and power
 *
 * */
void Basic_AS7343::disableAll(void) {
  Adafruit_BusIO_Register enable_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_ENABLE);

  enable_reg.write(0);
}

/**
 * @brief Enables measurement of spectral data
 *
 * @param enable_measurement true: enabled false: disabled
 * @return true: success false: failure
 */
bool Basic_AS7343::enableSpectralMeasurement(bool enable_measurement) {

  Adafruit_BusIO_Register enable_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_ENABLE);

  Adafruit_BusIO_RegisterBits spec_enable_bit =
      Adafruit_BusIO_RegisterBits(&enable_reg, 1, 1);
  return spec_enable_bit.write(enable_measurement);
}

/**
 * @brief Enable control of an attached LED on the LDR pin
 *
 * @param enable_led true: LED enabled false: LED disabled
 * @return true: success false: failure
 */
bool Basic_AS7343::enableLED(bool enable_led) {
    Adafruit_BusIO_Register led_reg = Adafruit_BusIO_Register(i2c_dev, AS7343_LED);
    // turns the LED on or off
    Adafruit_BusIO_RegisterBits led_activate = Adafruit_BusIO_RegisterBits(&led_reg, 1, 7);
    return led_activate.write(enable_led);
}


/**
 * @brief Set the current limit for the LED
 *
 * @param led_current_ma the value to set in milliamps. With a minimum of 4. Any
 * amount under 4 will be rounded up to 4
 *
 * Range is 4mA to 258mA
 * @return true: success false: failure
 */

/**
 * @brief Sets the LED drive strength (current limit in mA)
 * @param current_mA the value to set in milliamps.  Min 4, max 258 in increments of 4mA.
 * @return true: success false: failure
 */
bool Basic_AS7343::setLEDCurrent(uint16_t current_mA) {
  // check within permissible range
  uint8_t driveStrength;
  if (current_mA > 258) {
    return false;
  }
  if (current_mA < 4) {
    current_mA = 4;
  }
  Adafruit_BusIO_Register led_reg = Adafruit_BusIO_Register(i2c_dev, AS7343_LED);
  Adafruit_BusIO_RegisterBits drive_strength_bits = Adafruit_BusIO_RegisterBits(&led_reg, 7, 0);
  driveStrength = (current_mA - 4)/2;  
  return drive_strength_bits.write(driveStrength);
}

/**
 * @brief Get the LED drive strength (current limit in mA for the LED on LDR)
 *
 * Range is 4mA to 258mA
 * @return current limit in mA
 */
uint16_t Basic_AS7343::getLEDCurrent(void) {
  uint16_t led_current_ma;
  uint32_t led_raw;

  Adafruit_BusIO_Register led_reg = Adafruit_BusIO_Register(i2c_dev, AS7343_LED);
  Adafruit_BusIO_RegisterBits drive_strength_bits = Adafruit_BusIO_RegisterBits(&led_reg, 7, 0);
  led_raw = drive_strength_bits.read();
  led_current_ma = (uint16_t)(led_raw * 2) + 4;
  return led_current_ma;
}



/**
 * @brief Sets the LED drive strength register
 * To convert this to mA, mA = (driveStrength * 2) + 4mA
 * @param driveStrength a number from 0 to 127.  The set current is (driveStrength * 2) + 4mA
 * @return true: success false: failure
 */
bool Basic_AS7343::setLEDCurrentByte(uint8_t driveStrength) {
  Adafruit_BusIO_Register led_reg = Adafruit_BusIO_Register(i2c_dev, AS7343_LED);
  Adafruit_BusIO_RegisterBits drive_strength_bits = Adafruit_BusIO_RegisterBits(&led_reg, 7, 0);
  return drive_strength_bits.write(driveStrength);
}

/**
 * @brief Gets the current LED Drive strength (current). Returns the actual register value
 *
 * @return the LED Drive strength register setting
 */
uint8_t Basic_AS7343::getLEDCurrentByte() {
  Adafruit_BusIO_Register led_reg = Adafruit_BusIO_Register(i2c_dev, AS7343_LED);
  Adafruit_BusIO_RegisterBits drive_strength_bits = Adafruit_BusIO_RegisterBits(&led_reg, 7, 0);
  return drive_strength_bits.read();
}




/**
 * @brief Sets the active register bank
 *
 * The AS7343 uses banks to organize the register making it nescessary to set
 * the correct bank to access a register.
 *

 * @param low **true**: allow access of register bank 0x58 to 0x66 
 * **false**: Set the current bank to allow access to registers with addresses
 of `0x80` and above
 * @return true: success false: failure
 */
bool Basic_AS7343::setBank(bool low) {
  Adafruit_BusIO_Register cfg0_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_CFG0);
  Adafruit_BusIO_RegisterBits bank_bit =
      Adafruit_BusIO_RegisterBits(&cfg0_reg, 1, 4);

  return bank_bit.write(low);
}

/**
 * @brief Sets the threshold below which spectral measurements will trigger
 * interrupts when the APERS count is reached
 *
 * @param low_threshold the new threshold
 * @return true: success false: failure
 */
bool Basic_AS7343::setLowThreshold(uint16_t low_threshold) {
  Adafruit_BusIO_Register sp_low_threshold_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_SP_LOW_TH_L, 2, LSBFIRST);
  return sp_low_threshold_reg.write(low_threshold);
}

/**
 * @brief Returns the current low thighreshold for spectral measurements
 *
 * @return int16_t The current low threshold
 */
uint16_t Basic_AS7343::getLowThreshold(void) {
  Adafruit_BusIO_Register sp_low_threshold_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_SP_LOW_TH_L, 2, LSBFIRST);
  return sp_low_threshold_reg.read();
}

/**
 * @brief Sets the threshold above which spectral measurements will trigger
 * interrupts when the APERS count is reached
 *
 * @param high_threshold
 * @return true: success false: failure
 */
bool Basic_AS7343::setHighThreshold(uint16_t high_threshold) {
  Adafruit_BusIO_Register sp_high_threshold_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_SP_HIGH_TH_L, 2, LSBFIRST);
  return sp_high_threshold_reg.write(high_threshold);
}

/**
 * @brief Returns the current high thighreshold for spectral measurements
 *
 * @return int16_t The current high threshold
 */
uint16_t Basic_AS7343::getHighThreshold(void) {
  Adafruit_BusIO_Register sp_high_threshold_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_SP_HIGH_TH_L, 2, LSBFIRST);
  return sp_high_threshold_reg.read();
}

/**
 * @brief Enable Interrupts based on spectral measurements
 *
 * @param enable_int true: enable false: disable
 * @return true: success false: falure
 */
bool Basic_AS7343::enableSpectralInterrupt(bool enable_int) {
  Adafruit_BusIO_Register int_enable_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_INTENAB);
  Adafruit_BusIO_RegisterBits sp_int_bit =
      Adafruit_BusIO_RegisterBits(&int_enable_reg, 1, 3);
  return sp_int_bit.write(enable_int);
}

/**
 * @brief Enabled system interrupts
 *
 * @param enable_int Set to true to enable system interrupts
 * @return true: success false: failure
 */
bool Basic_AS7343::enableSystemInterrupt(bool enable_int) {
  Adafruit_BusIO_Register int_enable_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_INTENAB);
  Adafruit_BusIO_RegisterBits sien_int_bit =
      Adafruit_BusIO_RegisterBits(&int_enable_reg, 1, 0);
  return sien_int_bit.write(enable_int);
}

// Spectral Interrupt Persistence.
// Defines a filter for the number of consecutive
// occurrences that spectral data must remain outside
// the threshold range between SP_TH_L and
// SP_TH_H before an interrupt is generated. The
// spectral data channel used for the persistence filter
// is set by SP_TH_CHANNEL. Any sample that is
// inside the threshold range resets the counter to 0.

/**
 * @brief Sets the number of times an interrupt threshold must be exceeded
 * before an interrupt is triggered
 *
 * @param cycle_count The number of cycles to trigger an interrupt
 * @return true: success false: failure
 */
bool Basic_AS7343::setAPERS(as7343_int_cycle_count_t cycle_count) {
  Adafruit_BusIO_Register pers_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_PERS);
  Adafruit_BusIO_RegisterBits apers_bits =
      Adafruit_BusIO_RegisterBits(&pers_reg, 4, 0);
  return apers_bits.write(cycle_count);
}

/**
 * @brief Set the ADC channel to use for spectral thresholds including
 * interrupts, automatic gain control, and persistance settings
 *
 * @param channel The channel to use for spectral thresholds. Must be a
 * as7343_adc_channel_t
 * @return true: success false: failure
 */
bool Basic_AS7343::setSpectralThresholdChannel(as7343_adc_channel_t channel) {
  bool success = false;
  if (channel > AS7343_ADC_CHANNEL_5) {
    return false;
  }
  setBank(true); // Access 0x58 to 0x66 AS7343_CFG12 is 0x66
  Adafruit_BusIO_Register cfg_12_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_CFG12);
  Adafruit_BusIO_RegisterBits spectral_threshold_ch_bits =
      Adafruit_BusIO_RegisterBits(&cfg_12_reg, 3, 0);
  success = spectral_threshold_ch_bits.write(channel);
  setBank(false); // Access 0x80 and up 
  return success;
}

/**
 * @brief Returns the current value of the Interupt status register
 *
 * @return uint8_t
 */
uint8_t Basic_AS7343::getInterruptStatus(void) {
  Adafruit_BusIO_Register int_status_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_STATUS);
  return (uint8_t)int_status_reg.read();
}

/**
 * @brief Indicates if the latched data is affected by analog or digital saturation.
 * @param gainAtStatusRead (output variable) will indicate the GAIN that was applied for the spectral data during this saturation check.
 * @return true: if data is saturated false: if not
 */
bool Basic_AS7343::spectralLatchedSaturationStatus(as7343_gain_t &gainAtStatusRead){
  Adafruit_BusIO_Register astatus_reg = Adafruit_BusIO_Register(i2c_dev, AS7343_ASTATUS);
  uint8_t astatusData;
  astatusData = astatus_reg.read();
  gainAtStatusRead = (as7343_gain_t)(0b00001111 & astatusData);
  return (astatusData & 0b10000000) > 0;
}

/**
 * @brief Returns the status of the spectral measurement threshold interrupts
 *
 * @return true: interrupt triggered false: interrupt not triggered
 */
bool Basic_AS7343::spectralInterruptTriggered(void) {
  Adafruit_BusIO_Register int_status_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_STATUS);
  Adafruit_BusIO_RegisterBits aint_bit =
      Adafruit_BusIO_RegisterBits(&int_status_reg, 1, 3);

  return aint_bit.read();
}

/**
 * @brief Clear the interrupt status register
 *
 * @return true: success false: failure
 */
bool Basic_AS7343::clearInterruptStatus(void) {
  Adafruit_BusIO_Register int_status_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_STATUS);

  return int_status_reg.write(0xFF);
}

/**
 * @brief The current state of the spectral measurement interrupt status
 * register
 *
 * @return uint8_t The current status register
 */
uint8_t Basic_AS7343::spectralInterruptSource(void) {
  Adafruit_BusIO_Register status3_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_STATUS3);

  uint8_t spectral_int_source = status3_reg.read();
  last_spectral_int_source = spectral_int_source;
  return spectral_int_source;
}

/**
 * @brief The status of the low threshold interrupt
 *
 * @return true: low interrupt triggered false: interrupt not triggered
 */
bool Basic_AS7343::spectralLowTriggered(void) {
  return (last_spectral_int_source & AS7343_SPECTRAL_INT_LOW_MSK) > 0;
}

/**
 * @brief The status of the high threshold interrupt
 *
 * @return true: high interrupt triggered false: interrupt not triggered
 */
bool Basic_AS7343::spectralHighTriggered(void) {
  return (last_spectral_int_source & AS7343_SPECTRAL_INT_HIGH_MSK) > 0;
}

/**
 * @brief
 *
 * @return true: success false: failure
 */
bool Basic_AS7343::getIsDataReady() {
  Adafruit_BusIO_Register status2_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_STATUS2);
  Adafruit_BusIO_RegisterBits avalid_bit =
      Adafruit_BusIO_RegisterBits(&status2_reg, 1, 6);

  return avalid_bit.read();
}

// TODO; check for valid values
/**
 * @brief Sets the integration time step count
 *
 * Total integration time will be `(ATIME + 1) * (ASTEP + 1) * 2.78µS`
 *
 * @param atime_value The integration time step count
 * @return true: success false: failure
 */
bool Basic_AS7343::setATIME(uint8_t atime_value) {
  Adafruit_BusIO_Register atime_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_ATIME);
  return atime_reg.write(atime_value);
}

// TODO; check for valid values
/**
 * @brief Sets the Wait time, if WEN is enabled in AS7343_ENABLE register.
 * This specifies the delay time between two spectral measurements.  The wait time is: 2.78ms * (wtime_value+1) 
 * If wait is enabled, each new measurement is started based on WTIME. 
 * It is necessary for WTIME to be sufficiently long for spectral integration and 
 * any other functions to be completed within the period. The device will warn the user if the timing 
 * is configured incorrectly. If WTIME is too short, then SP_TRIG in register STATUS6 (ADDR: 0xA7) will be set to “1”.
 * Total integration time will be `(ATIME + 1) * (ASTEP + 1) * 2.78µS`
 *
 * @param wtime_value The integration time step count
 * @return true: success false: failure
 */
bool Basic_AS7343::setWTIME(uint8_t wtime_value) {
  Adafruit_BusIO_Register wtime_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_WTIME);
  return wtime_reg.write(wtime_value);
}


/**
 * @brief Returns the integration time step count
 *
 * Total integration time will be `(ATIME + 1) * (ASTEP + 1) * 2.78µS`
 *
 * @return uint8_t The current integration time step count
 */
uint8_t Basic_AS7343::getATIME() {
  Adafruit_BusIO_Register atime_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_ATIME);
  return atime_reg.read();
}

/**
 * @brief Returns the delay time between two spectral measurements.
 * Wait time in ms can be computed as: (return uint8_t + 1) * 2.78ms
 *
 * @return uint8_t The current WTIME setting.
 */
uint8_t Basic_AS7343::getWTIME() {
  Adafruit_BusIO_Register wtime_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_WTIME);
  return wtime_reg.read();
}

/**
 * @brief Sets the integration time step size
 *
 * @param astep_value Integration time step size in 2.78 microsecon increments
 * Step size is `(astep_value+1) * 2.78 uS`
 * @return true: success false: failure
 */
bool Basic_AS7343::setASTEP(uint16_t astep_value) {
  Adafruit_BusIO_Register astep_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_ASTEP_L, 2, LSBFIRST);
  return astep_reg.write(astep_value);
}

/**
 * @brief Returns the integration time step size
 *
 * Step size is `(astep_value+1) * 2.78 uS`
 *
 * @return uint16_t The current integration time step size
 */
uint16_t Basic_AS7343::getASTEP() {
  Adafruit_BusIO_Register astep_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_ASTEP_L, 2, LSBFIRST);
  return astep_reg.read();
}

/**
 * @brief Sets the ADC gain multiplier
 *
 * @param gain_value The gain amount. must be an `as7343_gain_t`
 * @return true: success false: failure
 */
bool Basic_AS7343::setGain(as7343_gain_t gain_value) {
  Adafruit_BusIO_Register cfg1_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_CFG1);
  return cfg1_reg.write(gain_value);
  // AGAIN bitfield is only[0:4] but the rest is empty
}


/**
 * @brief Sets the Auto channel readout to 6,12, or 18 channels (sets the autoSMUX bits in CFG20)
 * Must set before making spectral measurements.
 *
 * @param acr_value The auto channel readout value. must be an `as7343_auto_channel_readout_t`
 * @return true: success false: failure
 */
bool Basic_AS7343::setAutoChannelReadout(as7343_auto_channel_readout_t autoSMUX_value){
  Adafruit_BusIO_Register cfg20_reg = Adafruit_BusIO_Register(i2c_dev, AS7343_CFG20);
  Adafruit_BusIO_RegisterBits autoSMUX_bits = Adafruit_BusIO_RegisterBits(&cfg20_reg, 2, 5);
  return autoSMUX_bits.write(autoSMUX_value);
}

/**
 * @brief Returns the auto channel readout setting (AS7343_6CHANNEL, AS7343_12CHANNEL or AS7343_18CHANNEL)
 *        (gets the autoSMUX bits in CFG20)
 * @return as7343_auto_channel_readout_t The current auto channel readout setting
 */
as7343_auto_channel_readout_t Basic_AS7343::getAutoChannelReadout(){
  Adafruit_BusIO_Register cfg20_reg = Adafruit_BusIO_Register(i2c_dev, AS7343_CFG20);
  Adafruit_BusIO_RegisterBits autoSMUX_bits = Adafruit_BusIO_RegisterBits(&cfg20_reg, 2, 5);
  return (as7343_auto_channel_readout_t)autoSMUX_bits.read();
}

/**
 * @brief Returns the ADC gain multiplier
 *
 * @return as7343_gain_t The current ADC gain multiplier
 */
as7343_gain_t Basic_AS7343::getGain() {
  Adafruit_BusIO_Register cfg1_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7343_CFG1);
  return (as7343_gain_t)cfg1_reg.read();
}

/**
 * @brief Returns the integration time
 *
 * The integration time is `(ATIME + 1) * (ASTEP + 1) * 2.78µS`
 *
 * @return long The current integration time in ms
 */
long Basic_AS7343::getTINT() {
  long astep = getASTEP();
  long atime = getATIME();

  return (atime + 1) * (astep + 1) * 2.78 / 1000;
}

/**
 * @brief Converts raw ADC values to basic counts
 *
 * The basic counts are `RAW/(GAIN * TINT)`
 *
 * @param raw The raw ADC values to convert
 *
 * @return float The basic counts
 */
float Basic_AS7343::toBasicCounts(uint16_t raw) {
  float gain_val = 0;
  as7343_gain_t gain = getGain();
  switch (gain) {
  case AS7343_GAIN_0_5X:
    gain_val = 0.5;
    break;
  case AS7343_GAIN_1X:
    gain_val = 1;
    break;
  case AS7343_GAIN_2X:
    gain_val = 2;
    break;
  case AS7343_GAIN_4X:
    gain_val = 4;
    break;
  case AS7343_GAIN_8X:
    gain_val = 8;
    break;
  case AS7343_GAIN_16X:
    gain_val = 16;
    break;
  case AS7343_GAIN_32X:
    gain_val = 32;
    break;
  case AS7343_GAIN_64X:
    gain_val = 64;
    break;
  case AS7343_GAIN_128X:
    gain_val = 128;
    break;
  case AS7343_GAIN_256X:
    gain_val = 256;
    break;
  case AS7343_GAIN_512X:
    gain_val = 512;
    break;
  case AS7343_GAIN_1024X:
    gain_val = 1024;
    break;
  case AS7343_GAIN_2048X:
    gain_val = 2048;
    break;
  }
  return raw / (gain_val * (getATIME() + 1) * (getASTEP() + 1) * 2.78 / 1000);
}

/**
 * @brief Write a byte to the given register
 *
 * @param addr Register address
 * @param val The value to set the register to
 */
void Basic_AS7343::writeRegister(byte addr, byte val) {
  Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(i2c_dev, addr);
  reg.write(val);
}

