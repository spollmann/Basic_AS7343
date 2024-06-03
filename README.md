# Basic_AS7343
Arduino driver for the AS7343 14-channel spectral sensor chip (like the one found on the Mikroe Color 16 Click)
AS7343 is a 14-Channel Multi-Spectral Sensor by the "ams OSRAM Group" (ams-osram.com)

This project is based on the following repositories:
1. Adafruit_AS7341 (https://github.com/adafruit/Adafruit_AS7341) - This is the Adafruit AS7341 11-Channel Spectral Sensor for Arduino
2. Mikrosdk Click v2-Color16 driver (https://github.com/MikroElektronika/mikrosdk_click_v2/tree/master/clicks/color16) - AS7343 14-Channel Spectral sensor

## Compatible Hardware
Tested and works with the Mikroe Color-16 Click Breakout Board.  This chip uses I2C to communicate.
Other breakout boards may work as well.  Ensure the i2c address is correct for your breakout board (if it changed from default).

## Description
Implements a Driver for the AS7343 14-channel spectral sensor chip (like on the Mikroe Color 16 Click).

This library is adapted and made to function similar to the Adafruit_AS7341 library (https://github.com/adafruit/Adafruit_AS7341), with additions, and also elements of the
Mikroe color16 driver (default config function).  See copyright notices in the LICENSE file

## Installation
Clone this repository under your Arduino/libraries folder

## Dependencies
* [Adafruit_BusIO](https://github.com/adafruit/Adafruit_BusIO)

## TODO:
-Flicker Detection <br />
-CUSTOM Smux <br />
...and more <br />
