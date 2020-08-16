/*!
 *  @file Adafruit_LIS331DLH.h
 *
 *  This is a library for the Adafruit LIS331DLH Accel breakout board
 *
 *  Designed specifically to work with the [Adafruit LIS331DLH Triple-Axis
 *  Accelerometer (+-2g/4g/8g)](https://www.adafruit.com/product/4XXX)
 *
 *  This sensor communicates over I2C or SPI (our library code supports both) so
 *  you can share it with a bunch of other sensors on the same I2C bus.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Bryan Siepert for Adafruit Industries
 *  BSD license, all text above must be included in any redistribution
 *
 */

#ifndef ADAFRUIT_LIS331DLH_H
#define ADAFRUIT_LIS331DLH_H

#include "Adafruit_LIS331.h"

/** I2C ADDRESS/BITS **/
#define LIS331DLH_DEFAULT_ADDRESS (0x18) // if SDO/SA0 is 3V, its 0x19

/** A structure to represent scales **/
typedef enum {
  LIS331DLH_RANGE_2_G = 0x0,   ///< +/- 6G
  LIS331DLH_RANGE_4_G = 0x1,  ///< +/- 12G
  LIS331DLH_RANGE_8_G = 0x03, ///< +/- 24Gvalue)
} lis331dlh_range_t;

/*!
 *  @brief  Class that stores state and functions for interacting with
 *          Adafruit_LIS331DLH
 */
class Adafruit_LIS331DLH : public Adafruit_LIS331 {
public:
  Adafruit_LIS331DLH();

  bool begin_I2C(uint8_t i2c_addr = LIS331_DEFAULT_ADDRESS,
                 TwoWire *wire = &Wire, int32_t sensorID = 0);

  bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI,
                 int32_t sensor_id = 0);
  bool begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                 int8_t mosi_pin, int32_t sensor_id = 0);

  void setRange(lis331dlh_range_t range);
  lis331dlh_range_t getRange(void);

private:
  bool _init(int32_t sensor_id);
  void _scaleValues(void);
};

#endif
