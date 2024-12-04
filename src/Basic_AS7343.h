/*!
 *  @file Basic_AS7343.h

 *  @mainpage AS7343 14-Channel Spectral Sensor
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the Library for the AS7343 14-Channel Spectral Sensor
 *
 *  @section dependencies Dependencies
 *  This library depends on the Adafruit BusIO library
 *
 *  @section author Author
 *
 *  Steven Pollmann for Robarts Research, Western University
 *
 * 	@section license License
 *  
 * 	MIT - see Top of Basic_AS7343.cpp for notices
 *
 * 	@section  HISTORY
 *
 *     v0.3 - Third beta release.  Fix bufferring of data, when device is not powered off but a new MCU program runs
 *            eg.  After programming, where readAllChannels return early, and with old Data readings.  Device is now "powered off" and
 *                 "powered on" when .begin() is called.
 *            Fix issue with readAllChannels in a loop sometimes returning quickly, and showing data from previous reads (buffered data)
 *     v0.2 - Second beta release.  Change LED Current functions, and add LED current functions that work with actual register settings.
 *     v0.1 - First beta release
 */

/*
  This library is adapted and made to feel similar to the adafruit as7341 library, with additions, and elements of the
  Mikroe color16 driver.  See copyright notices in Basic_AS7343.cpp heading.
*/

#ifndef _BASIC_AS7343_H
#define _BASIC_AS7343_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Wire.h>


#define AS7343_I2CADDR_DEFAULT  0x39 ///< AS7343 default i2c address
#define AS7343_CHIP_ID          0x81         ///< AS7343 default device id from WHOAMI


#define AS7343_AUXID            0x58 ///< AS7343_AUXID (unused)
#define AS7343_REVID            0x59 ///< AS7343_REVID (unused)
#define AS7343_WHOAMI           0x5A ///< Chip ID register
#define AS7343_CFG12            0x66 ///< Spectral threshold channel for interrupts, persistence and auto-gain
#define AS7343_ENABLE           0x80 ///< Main enable register. Controls SMUX, Flicker Detection, Spectral Measurements and Power
#define AS7343_ATIME            0x81       ///< Sets ADC integration step count
#define AS7343_WTIME            0x83       ///< AS7343_WTIME (unused)
#define AS7343_SP_LOW_TH_L      0x84 ///< Spectral measurement Low Threshold low byte (LSB)
#define AS7343_SP_LOW_TH_H      0x85 ///< Spectral measurement Low Threshold high byte (MSB)
#define AS7343_SP_HIGH_TH_L     0x86 ///< Spectral measurement High Threshold low byte (LSB)
#define AS7343_SP_HIGH_TH_H     0x87 ///< Spectral measurement High Threshold low byte (MSB)
#define AS7343_STATUS           0x93 ///< Interrupt status registers. Indicates the occourance of an interrupt
#define AS7343_ASTATUS          0x94   ///< AS7343_ASTATUS

#define AS7343_DATA_00_L        0x95 ///< Spectral Data Low (LSB)
#define AS7343_DATA_00_H        0x96 ///< Spectral Data High (MSB)
#define AS7343_DATA_01_L        0x97 ///< Spectral Data Low (LSB)
#define AS7343_DATA_01_H        0x98 ///< Spectral Data High (MSB)
#define AS7343_DATA_02_L        0x99 ///< Spectral Data Low (LSB)
#define AS7343_DATA_02_H        0x9A ///< Spectral Data High (MSB)
#define AS7343_DATA_03_L        0x9B ///< Spectral Data Low (LSB)
#define AS7343_DATA_03_H        0x9C ///< Spectral Data High (MSB)
#define AS7343_DATA_04_L        0x9D ///< Spectral Data Low (LSB)
#define AS7343_DATA_04_H        0x9E ///< Spectral Data High (MSB)
#define AS7343_DATA_05_L        0x9F ///< Spectral Data Low (LSB)
#define AS7343_DATA_05_H        0xA0 ///< Spectral Data Low (MSB)
#define AS7343_DATA_06_L        0xA1 ///< Spectral Data Low (LSB)
#define AS7343_DATA_06_H        0xA2 ///< Spectral Data High (MSB)
#define AS7343_DATA_07_L        0xA3 ///< Spectral Data Low (LSB)
#define AS7343_DATA_07_H        0xA4 ///< Spectral Data High (MSB)
#define AS7343_DATA_08_L        0xA5 ///< Spectral Data Low (LSB)
#define AS7343_DATA_08_H        0xA6 ///< Spectral Data High (MSB)
#define AS7343_DATA_09_L        0xA7 ///< Spectral Data Low (LSB)
#define AS7343_DATA_09_H        0xA8 ///< Spectral Data High (MSB)
#define AS7343_DATA_10_L        0xA9 ///< Spectral Data Low (LSB)
#define AS7343_DATA_10_H        0xAA ///< Spectral Data High (MSB)
#define AS7343_DATA_11_L        0xAB ///< Spectral Data Low (LSB)
#define AS7343_DATA_11_H        0xAC ///< Spectral Data High (MSB)
#define AS7343_DATA_12_L        0xAD ///< Spectral Data Low (LSB)
#define AS7343_DATA_12_H        0xAE ///< Spectral Data High (MSB)
#define AS7343_DATA_13_L        0xAF ///< Spectral Data Low (LSB)
#define AS7343_DATA_13_H        0xB0 ///< Spectral Data High (MSB)
#define AS7343_DATA_14_L        0xB1 ///< Spectral Data Low (LSB)
#define AS7343_DATA_14_H        0xB2 ///< Spectral Data High (MSB)
#define AS7343_DATA_15_L        0xB3 ///< Spectral Data Low (LSB)
#define AS7343_DATA_15_H        0xB4 ///< Spectral Data High (MSB)
#define AS7343_DATA_16_L        0xB5 ///< Spectral Data Low (LSB)
#define AS7343_DATA_16_H        0xB6 ///< Spectral Data High (MSB)
#define AS7343_DATA_17_L        0xB7 ///< Spectral Data Low (LSB)
#define AS7343_DATA_17_H        0xB8 ///< Spectral Data High (MSB)

#define AS7343_STATUS2          0x90 ///< Measurement status flags; Spectral valid, digital saturation, analog saturation, flicker detect analog saturation, flicker detect digital saturation
#define AS7343_STATUS3          0x91 ///< Spectral interrupt source, due to exceding high or below low threshold
#define AS7343_STATUS5          0xBB ///< AS7343_STATUS5 Status of potential interrupt: Flicker detect interrupt, and smux operation interrupt
#define AS7343_STATUS4          0xBC ///< AS7343_STATUS4 Status of potential errors FIFO Buffer overflow, Overtemp, Flicker Detect trigger error


#define AS7343_CFG0             0xBF ///< Sets Low power mode, Register bank access, and Trigger lengthening
#define AS7343_CFG1             0xC6 ///< Controls Spectral engine Gain setting
#define AS7343_CFG3             0xC7 ///< Sleep after interrupt
#define AS7343_CFG6             0xF5 ///< Used to configure Smux
#define AS7343_CFG8             0xC9 ///< FIFO Threshold
#define AS7343_CFG9             0xCA ///< Enables flicker detection and smux command completion system interrupts
#define AS7343_CFG10            0x65 ///< Flicker detect persistence
#define AS7343_PERS             0xCF ///< Number of measurement cycles outside thresholds to trigger an interupt
#define AS7343_GPIO             0x6B ///< GPIO Settings and status: polarity, direction, sets output, reads
#define AS7343_ASTEP_L          0xD4 ///< Integration step size low byte (LSB)
#define AS7343_ASTEP_H          0xD5 ///< Integration step size high byte (MSB)
#define AS7343_CFG20            0xD6 ///< Enable/Disable 8bit FIFO, and Automatic channel readout 6ch,12ch, 18ch readout 
#define AS7343_LED              0xCD ///< LED Register; Enables external LED on LDR, and sets current
#define AS7343_AGC_GAIN_MAX     0xD7 ///< set Flicker detection gain max
#define AS7343_AZ_CONFIG        0xDE ///< How often spectral engine offsets are reset (auto zero) to compensate for device temperature changes
#define AS7343_FD_TIME1         0xE0 ///< Flicker detection integration time low byte (LSB) of 11 bit number
#define AS7343_FD_TIME2         0xE2 ///< Flicker detection gain and high 3 bits of integration time (MSB)
#define AS7343_FD_CFG0          0xDF ///< FIFO Write Flicker Detection. If set, flicker raw data written to FIFO (one byte per sample)
#define AS7343_FD_STATUS        0xE3 ///< Flicker detection status; measurement valid, saturation, flicker type (120Hz, 100Hz)
#define AS7343_INTENAB          0xF9 ///< Enables individual interrupt types (Spectral and Flicker saturation, Spectral, FIFO Buffer and System interrupt enable)
#define AS7343_CONTROL          0xFA ///< Software reset, Auto-zero, fifo clear, clear Sleep-After-Interrupt active
#define AS7343_FIFO_MAP         0xFC ///< FIFO WRITE CH5-0, astatus to FIFO buffer (? CH0 to 5? leftover from other doc?)
#define AS7343_FIFO_LVL         0xFD ///< FIFO buffer level of entries from 1 to 128 (i.e. 2bytes per entry up to 256Bytes)
#define AS7343_FDATA_L          0xFE ///< FIFO buffer data (LSB)
#define AS7343_FDATA_H          0xFF ///< FIFO buffer data (MSB)


#define AS7343_SPECTRAL_INT_HIGH_MSK 0b00100000 ///< bitmask to check for a high threshold interrupt
#define AS7343_SPECTRAL_INT_LOW_MSK  0b00010000 ///< bitmask to check for a low threshold interrupt

/**
 * @brief Allowable gain multipliers for `setGain`
 *
 */
typedef enum {
  AS7343_GAIN_0_5X,
  AS7343_GAIN_1X,
  AS7343_GAIN_2X,
  AS7343_GAIN_4X,
  AS7343_GAIN_8X,
  AS7343_GAIN_16X,
  AS7343_GAIN_32X,
  AS7343_GAIN_64X,
  AS7343_GAIN_128X,
  AS7343_GAIN_256X,
  AS7343_GAIN_512X,
  AS7343_GAIN_1024X,
  AS7343_GAIN_2048X,
} as7343_gain_t;

/**
 * @brief ADC Channel specifiers for configuration
 *
 */
typedef enum {
  AS7343_ADC_CHANNEL_0,
  AS7343_ADC_CHANNEL_1,
  AS7343_ADC_CHANNEL_2,
  AS7343_ADC_CHANNEL_3,
  AS7343_ADC_CHANNEL_4,
  AS7343_ADC_CHANNEL_5,
} as7343_adc_channel_t;


/**
 * @brief The number of measurement cycles with spectral data outside of a
 * threshold required to trigger an interrupt
 *
 */
typedef enum {
  AS7343_INT_COUNT_ALL, ///< 0
  AS7343_INT_COUNT_1,   ///< 1
  AS7343_INT_COUNT_2,   ///< 2
  AS7343_INT_COUNT_3,   ///< 3
  AS7343_INT_COUNT_5,   ///< 4
  AS7343_INT_COUNT_10,  ///< 5
  AS7343_INT_COUNT_15,  ///< 6
  AS7343_INT_COUNT_20,  ///< 7
  AS7343_INT_COUNT_25,  ///< 8
  AS7343_INT_COUNT_30,  ///< 9
  AS7343_INT_COUNT_35,  ///< 10
  AS7343_INT_COUNT_40,  ///< 11
  AS7343_INT_COUNT_45,  ///< 12
  AS7343_INT_COUNT_50,  ///< 13
  AS7343_INT_COUNT_55,  ///< 14
  AS7343_INT_COUNT_60,  ///< 15
} as7343_int_cycle_count_t;

/**
 * @brief Pin directions to set how the GPIO pin is to be used
 *
 */
typedef enum {
  AS7343_GPIO_OUTPUT, ///< THhe GPIO pin is configured as an open drain output
  AS7343_GPIO_INPUT,  ///< The GPIO Pin is set as a high-impedence input
} as7343_gpio_dir_t;

/**
 * @brief Wait states for async reading
 */
typedef enum {
  AS7343_WAITING_START, //
  AS7343_WAITING_DATA,  //
  AS7343_WAITING_DONE,  //
} as7343_waiting_t;

typedef enum {
  AS7343_6CHANNEL = 0,
  AS7343_12CHANNEL = 2,
  AS7343_18CHANNEL = 3,
} as7343_auto_channel_readout_t;

/**
 * @brief Spectral Channel specifiers for configuration and reading
 *
 */
typedef enum {
  AS7343_CHANNEL_450nm_FZ,
  AS7343_CHANNEL_555nm_FY,
  AS7343_CHANNEL_600nm_FXL,
  AS7343_CHANNEL_855nm_NIR,
  AS7343_CHANNEL_2X_VIS_1,
  AS7343_CHANNEL_FD_1,
  AS7343_CHANNEL_425nm_F2,
  AS7343_CHANNEL_475nm_F3,
  AS7343_CHANNEL_515nm_F4,
  AS7343_CHANNEL_640nm_F6,
  AS7343_CHANNEL_2X_VIS_2,
  AS7343_CHANNEL_FD_2,
  AS7343_CHANNEL_405nm_F1,
  AS7343_CHANNEL_550nm_F5,
  AS7343_CHANNEL_690nm_F7,
  AS7343_CHANNEL_745nm_F8,
  AS7343_CHANNEL_2X_VIS_3,
  AS7343_CHANNEL_FD_3,
} as7343_color_channel_t;




class Basic_AS7343;

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the AS7343 14-Channel Spectral Sensor
 */
class Basic_AS7343 {
public:
  Basic_AS7343();
  ~Basic_AS7343();

  bool begin(uint8_t i2c_addr = AS7343_I2CADDR_DEFAULT, TwoWire *wire = &Wire,
             int32_t sensor_id = 0);
  bool setDefaultConfig();
  bool setASTEP(uint16_t astep_value);
  bool setATIME(uint8_t atime_value);
  bool setWTIME(uint8_t wtime_value);
  bool setGain(as7343_gain_t gain_value);
  bool setAutoChannelReadout(as7343_auto_channel_readout_t acr_value);
  bool setLEDCurrentByte(uint8_t driveStrength);
  bool setLEDCurrent(uint16_t current_mA);


  uint16_t getASTEP();
  uint8_t getATIME();
  uint8_t getWTIME();
  as7343_gain_t getGain();
  as7343_auto_channel_readout_t getAutoChannelReadout();
  uint8_t getLEDCurrentByte();
  uint16_t getLEDCurrent();

  long getTINT();
  float toBasicCounts(uint16_t raw);

  bool readAllChannels(void);
  bool readAllChannels(uint16_t *readings_buffer);
  void delayForData(int waitTime = 0);
  uint16_t readChannel(as7343_color_channel_t channel);
  uint16_t getChannel(as7343_color_channel_t channel);

  bool startReading(void);
  bool checkReadingProgress();
  bool getAllChannels(uint16_t *readings_buffer);

  void softwareReset();
  void powerEnable(bool enable_power);
  bool enableWaitTime(bool enable_waitTime);
  bool enableSpectralMeasurement(bool enable_measurement);
  bool enableLED(bool enable_led);

  bool setHighThreshold(uint16_t high_threshold);
  bool setLowThreshold(uint16_t low_threshold);

  uint16_t getHighThreshold(void);
  uint16_t getLowThreshold(void);

  bool enableSpectralInterrupt(bool enable_int);
  bool enableSystemInterrupt(bool enable_int);

  bool setAPERS(as7343_int_cycle_count_t cycle_count);
  bool setSpectralThresholdChannel(as7343_adc_channel_t channel);

  uint8_t getInterruptStatus(void);
  bool clearInterruptStatus(void);

  bool spectralLatchedSaturationStatus(as7343_gain_t &gainAtStatusRead);

  bool spectralInterruptTriggered(void);
  uint8_t spectralInterruptSource(void);
  bool spectralLowTriggered(void);
  bool spectralHighTriggered(void);

  void disableAll(void);

  bool getIsDataReady();
  bool setBank(bool low); // low true gives access to 0x58 to 0x66 

protected:
  virtual bool _init(int32_t sensor_id);
  uint8_t last_spectral_int_source = 0; ///< The value of the last reading of the spectral interrupt source register

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface

private:
  void writeRegister(byte addr, byte val);
  void setSMUXLowChannels(bool f1_f4);
  uint16_t _channel_readings[18];
  as7343_waiting_t _readingState;
};

#endif

