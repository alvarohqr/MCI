/***************************************************************************
  This is a library for the BME680 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME680 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include "Arduino.h"
#include "Adafruit_BME680.h"
#include <Wire.h>

#define BME680_DEBUG

#define INT8_C(c)               c
#define INT16_C(c)              c
#define UINT8_C(c)              c ## U
#define UINT16_C(c)             c ## U

// must be global in order to work with underlying library
int8_t _BME680_SoftwareSPI_MOSI, _BME680_SoftwareSPI_MISO, _BME680_SoftwareSPI_SCK;


/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

Adafruit_BME680::Adafruit_BME680(int8_t cspin)
  : _cs(cspin)
{
  _BME680_SoftwareSPI_MOSI = -1;
  _BME680_SoftwareSPI_MISO = -1;
  _BME680_SoftwareSPI_SCK = -1;
  _filterEnabled = _tempEnabled = _humEnabled = _presEnabled = _gasEnabled = false;
}

Adafruit_BME680::Adafruit_BME680(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin)
  : _cs(cspin)
{
  _BME680_SoftwareSPI_MOSI = mosipin;
  _BME680_SoftwareSPI_MISO = misopin;
  _BME680_SoftwareSPI_SCK = sckpin;
  _filterEnabled = _tempEnabled = _humEnabled = _presEnabled = _gasEnabled = false;
}


bool Adafruit_BME680::begin(uint8_t addr) {
  _i2caddr = addr;

  if (_cs == -1) {
    // i2c
    Wire.begin();

    gas_sensor.dev_id = addr;
    gas_sensor.intf = BME680_I2C_INTF;
    gas_sensor.read = &i2c_read;
    gas_sensor.write = &i2c_write;
  } /*
  else {
    digitalWrite(_cs, HIGH);
    pinMode(_cs, OUTPUT);

    if (_BME680_SoftwareSPI_SCK == -1) {
      // hardware SPI
      SPI.begin();
    } else {
      // software SPI
      pinMode(_BME680_SoftwareSPI_SCK, OUTPUT);
      pinMode(_BME680_SoftwareSPI_MOSI, OUTPUT);
      pinMode(_BME680_SoftwareSPI_MISO, INPUT);
    }

    gas_sensor.dev_id = _cs;
    gas_sensor.intf = BME680_SPI_INTF;
    gas_sensor.read = &spi_read;
    gas_sensor.write = &spi_write;
  }
*/
  gas_sensor.delay_ms = delay;

  int8_t rslt = BME680_OK;
  rslt = bme680_init(&gas_sensor);
  //Serial.print("Result: "); Serial.println(rslt);
  if (rslt != BME680_OK) 
    return false;

  Serial.print("T1 = "); Serial.println(gas_sensor.calib.par_t1);
  Serial.print("T2 = "); Serial.println(gas_sensor.calib.par_t2);
  Serial.print("T3 = "); Serial.println(gas_sensor.calib.par_t3);
  Serial.print("P1 = "); Serial.println(gas_sensor.calib.par_p1);
  Serial.print("P2 = "); Serial.println(gas_sensor.calib.par_p2);
  Serial.print("P3 = "); Serial.println(gas_sensor.calib.par_p3);
  Serial.print("P4 = "); Serial.println(gas_sensor.calib.par_p4);
  Serial.print("P5 = "); Serial.println(gas_sensor.calib.par_p5);
  Serial.print("P6 = "); Serial.println(gas_sensor.calib.par_p6);
  Serial.print("P7 = "); Serial.println(gas_sensor.calib.par_p7);
  Serial.print("P8 = "); Serial.println(gas_sensor.calib.par_p8);
  Serial.print("P9 = "); Serial.println(gas_sensor.calib.par_p9);
  Serial.print("P10 = "); Serial.println(gas_sensor.calib.par_p10);
  Serial.print("H1 = "); Serial.println(gas_sensor.calib.par_h1);
  Serial.print("H2 = "); Serial.println(gas_sensor.calib.par_h2);
  Serial.print("H3 = "); Serial.println(gas_sensor.calib.par_h3);
  Serial.print("H4 = "); Serial.println(gas_sensor.calib.par_h4);
  Serial.print("H5 = "); Serial.println(gas_sensor.calib.par_h5);
  Serial.print("H6 = "); Serial.println(gas_sensor.calib.par_h6);
  Serial.print("H7 = "); Serial.println(gas_sensor.calib.par_h7);
  Serial.print("G1 = "); Serial.println(gas_sensor.calib.par_gh1);
  Serial.print("G2 = "); Serial.println(gas_sensor.calib.par_gh2);
  Serial.print("G3 = "); Serial.println(gas_sensor.calib.par_gh3);
  Serial.print("G1 = "); Serial.println(gas_sensor.calib.par_gh1);
  Serial.print("G2 = "); Serial.println(gas_sensor.calib.par_gh2);
  Serial.print("G3 = "); Serial.println(gas_sensor.calib.par_gh3);
  Serial.print("Heat Range = "); Serial.println(gas_sensor.calib.res_heat_range);
  Serial.print("Heat Val = "); Serial.println(gas_sensor.calib.res_heat_val);
  Serial.print("SW Error = "); Serial.println(gas_sensor.calib.range_sw_err);

  setTemperatureOversampling(BME680_OS_8X);
  setHumidityOversampling(BME680_OS_2X);
  setPressureOversampling(BME680_OS_4X);
  setIIRFilterSize(BME680_FILTER_SIZE_3);
  setGasHeater(320, 150); // 320*C for 150 ms
  
  // don't do anything till we request a reading
  gas_sensor.power_mode = BME680_FORCED_MODE;

  return true;
}

bool Adafruit_BME680::performReading(void) {
  uint8_t set_required_settings = 0;
  struct bme680_field_data data;
  int8_t rslt;

  /* Select the power mode */
  /* Must be set before writing the sensor configuration */
  gas_sensor.power_mode = BME680_FORCED_MODE;

  /* Set the required sensor settings needed */
  if (_tempEnabled)
    set_required_settings |= BME680_OST_SEL;
  if (_humEnabled)
    set_required_settings |= BME680_OSH_SEL;
  if (_presEnabled)
    set_required_settings |= BME680_OSP_SEL;
  if (_filterEnabled)
    set_required_settings |= BME680_FILTER_SEL;
  if (_gasEnabled) 
    set_required_settings |= BME680_GAS_SENSOR_SEL;

  /* Set the desired sensor configuration */
  Serial.println("Setting sensor settings");
  rslt = bme680_set_sensor_settings(set_required_settings, &gas_sensor);
  if (rslt != BME680_OK) 
    return false;
  
  /* Set the power mode */
  Serial.println("Setting power mode");
  rslt = bme680_set_sensor_mode(&gas_sensor);
  if (rslt != BME680_OK) 
    return false;

  /* Get the total measurement duration so as to sleep or wait till the
   * measurement is complete */
  uint16_t meas_period;
  bme680_get_profile_dur(&meas_period, &gas_sensor);
  Serial.print("Waiting (ms) "); Serial.println(meas_period);
  delay(meas_period * 2); /* Delay till the measurement is ready */
  
  Serial.print("t_fine = "); Serial.println(gas_sensor.calib.t_fine);

  Serial.println("Getting sensor data");
  rslt = bme680_get_sensor_data(&data, &gas_sensor);
  if (rslt != BME680_OK) 
    return false;

  if (_tempEnabled) {
    Serial.print("Temp: "); Serial.println(data.temperature / 100.0, 2);
  }
  if (_humEnabled) {
    Serial.print("Hum:  "); Serial.println(data.humidity / 1000.0, 2);
  }
  if (_presEnabled) {
    Serial.print("Pres: "); Serial.println(data.pressure / 100.0, 2);
  }

  /* Avoid using measurements from an unstable heating setup */
  if (_gasEnabled) {
    if (data.status & BME680_HEAT_STAB_MSK) {
      Serial.print("Gas resistance: "); Serial.println(data.gas_resistance);
    } else {
      Serial.println("Gas reading unstable!");
    }
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Enable and configure gas reading + heater
*/
/**************************************************************************/
bool Adafruit_BME680::setGasHeater(uint16_t heaterTemp, uint16_t heaterTime) {
  gas_sensor.gas_sett.heatr_temp = heaterTemp;
  gas_sensor.gas_sett.heatr_dur = heaterTime;

  if ( (heaterTemp == 0) || (heaterTime == 0) ) {
    // disabled!
    gas_sensor.gas_sett.run_gas = BME680_DISABLE_GAS_MEAS;
    _gasEnabled = false;
  } else {
    gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    _gasEnabled = true;
  }
  return true;
}


/**************************************************************************/
/*!
    @brief  Setters for Temp, Humidity, Pressure oversampling
*/
/**************************************************************************/

bool Adafruit_BME680::setTemperatureOversampling(uint8_t os) {
  if (os > BME680_OS_16X) return false;

  gas_sensor.tph_sett.os_temp = os;

  if (os == BME680_OS_NONE)
    _tempEnabled = false;
  else
    _tempEnabled = true;

  return true;
}

bool Adafruit_BME680::setHumidityOversampling(uint8_t os) {
  if (os > BME680_OS_16X) return false;

  gas_sensor.tph_sett.os_hum = os;

  if (os == BME680_OS_NONE)
    _humEnabled = false;
  else
    _humEnabled = true;

  return true;
}

bool Adafruit_BME680::setPressureOversampling(uint8_t os) {
  if (os > BME680_OS_16X) return false;

  gas_sensor.tph_sett.os_pres = os;

  if (os == BME680_OS_NONE)
    _presEnabled = false;
  else
    _presEnabled = true;

  return true;
}

/**************************************************************************/
/*!
    @brief  Setter for IIR filter
*/
/**************************************************************************/
bool Adafruit_BME680::setIIRFilterSize(uint8_t fs) {
  if (fs > BME680_FILTER_SIZE_127) return false;

  gas_sensor.tph_sett.filter = fs;

  if (fs == BME680_FILTER_SIZE_0)
    _filterEnabled = false;
  else
    _filterEnabled = true;

  return true;
}

/**************************************************************************/
/*!
    @brief  Reads 8 bit values over I2C
*/
/**************************************************************************/
int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
#ifdef BME680_DEBUG
  Serial.print("\tI2C $"); Serial.print(reg_addr, HEX); Serial.print(" => ");
#endif

  Wire.beginTransmission((uint8_t)dev_id);
  Wire.write((uint8_t)reg_addr);
  Wire.endTransmission();
  if (len != Wire.requestFrom((uint8_t)dev_id, (byte)len)) {
#ifdef BME680_DEBUG
    Serial.print("Failed to read "); Serial.print(len); Serial.print(" bytes from "); Serial.println(dev_id, HEX);
#endif
    return 1;
  }
  while (len--) {
    *reg_data = (uint8_t)Wire.read();
#ifdef BME680_DEBUG
    Serial.print("0x"); Serial.print(*reg_data, HEX); Serial.print(", ");
#endif
    reg_data++;
  }
#ifdef BME680_DEBUG
  Serial.println("");
#endif
  return 0;
}

/**************************************************************************/
/*!
    @brief  Writes 8 bit values over I2C
*/
/**************************************************************************/
int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
#ifdef BME680_DEBUG
  Serial.print("\tI2C $"); Serial.print(reg_addr, HEX); Serial.print(" <= ");
#endif
  Wire.beginTransmission((uint8_t)dev_id);
  Wire.write((uint8_t)reg_addr);
  while (len--) {
    Wire.write(*reg_data);
#ifdef BME680_DEBUG
    Serial.print("0x"); Serial.print(*reg_data, HEX); Serial.print(", ");
#endif
    reg_data++;
  }
  Wire.endTransmission();
#ifdef BME680_DEBUG
  Serial.println("");
#endif
  return 0;
}



/**************************************************************************/
/*!
    @brief  Reads 8 bit values over SPI
*/
/**************************************************************************/
/*
static int8_t spi_read(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
#ifdef BME680_DEBUG
  Serial.print("\tSPI $"); Serial.print(reg_addr, HEX); Serial.print(" => ");
#endif

  digitalWrite(cspin, LOW);

  // If hardware SPI we should use transactions!
  if (_BME680_SoftwareSPI_SCK == -1) {
    SPI.beginTransaction(SPISettings(BME680_DEFAULT_SPIFREQ, MSBFIRST, SPI_MODE0));
  }

  spi_transfer(reg_addr);

  while (len--) {
    *reg_data = spi_transfer(0x00);
#ifdef BME680_DEBUG
    Serial.print("0x"); Serial.print(*reg_data, HEX); Serial.print(", ");
#endif
    reg_data++;
  }

  if (_BME680_SoftwareSPI_SCK == -1) {
    SPI.endTransaction();
  }

  digitalWrite(cspin, HIGH);

#ifdef BME680_DEBUG
  Serial.println("");
#endif
  return 0;
}
*/
/**************************************************************************/
/*!
    @brief  Writes 8 bit values over SPI
*/
/**************************************************************************/
/*
static int8_t spi_write(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
#ifdef BME680_DEBUG
  Serial.print("\tSPI $"); Serial.print(reg_addr, HEX); Serial.print(" <= ");
#endif

  digitalWrite(cspin, LOW);

  // If hardware SPI we should use transactions!
  if (_BME680_SoftwareSPI_SCK == -1) {
    SPI.beginTransaction(SPISettings(BME680_DEFAULT_SPIFREQ, MSBFIRST, SPI_MODE0));
  }

  spi_transfer(reg_addr);
  while (len--) {
    spi_transfer(*reg_data);
#ifdef BME680_DEBUG
    Serial.print("0x"); Serial.print(*reg_data, HEX); Serial.print(", ");
#endif
    reg_data++;
  }

  if (_BME680_SoftwareSPI_SCK == -1) {
    SPI.endTransaction();
  }

  digitalWrite(cspin, HIGH);

#ifdef BME680_DEBUG
  Serial.println("");
#endif
  return 0;
}
*/
/*
static uint8_t spi_transfer(uint8_t x) {
  if (_BME680_SoftwareSPI_SCK == -1)
    return SPI.transfer(x);

  // software spi
  //Serial.println("Software SPI");
  uint8_t reply = 0;
  for (int i=7; i>=0; i--) {
    reply <<= 1;
    digitalWrite(_BME680_SoftwareSPI_SCK, LOW);
    digitalWrite(_BME680_SoftwareSPI_MOSI, x & (1<<i));
    digitalWrite(_BME680_SoftwareSPI_SCK, HIGH);
    if (digitalRead(_BME680_SoftwareSPI_MISO))
      reply |= 1;
  }
  return reply;
}

*/
