/**
 * Bosch BMP180 pressure and temperature sensor stand-alone library for pico sdk.
 * It simplyfies the use of the sensor and hides the tiresome calculations.  
 * 
 * This library was inspried by the BMP180 Breakout Arduino library by SparkFun.
 * GitHub: https://github.com/sparkfun/BMP180_Breakout_Arduino_Library
 * 
 * Datasheet used for reference: https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
 * 
 * Version 1.0 2021/11/03 - Initial Release
 * Version 1.1 2021/11/10 - Minor changes, changed naming conventions, created new structs for more structure
 * 
 * @version 1.1
 * @author Maik Steiger (maik.steiger@tu-dortmund.de)
*/
#ifndef _BMP180_H_
#define _BMP180_H_

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define BMP180_ADDRESS 0x77

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef I2C_INFORMATION_S_
#define I2C_INFORMATION_S_
    struct i2c_information
    {
        i2c_inst_t *instance;
        uint8_t address;
    };
#endif

    struct bmp180_calibration_data
    {
        int16_t AC1, AC2, AC3, VB1, VB2, MB, MC, MD;
        uint16_t AC4, AC5, AC6;
        double c5, c6, mc, md, x0, x1, x2, y0, y1, y2, p0, p1, p2;
    };

    struct bmp180_configuration
    {
        uint8_t oversampling;
        uint8_t measTemperature;
        uint8_t measPressure;
        int ack;
    };

    /**
     * BMP180 sensor data structure. It contains all the needed calculated constants and values
     * for runtime use. Do not modify this structure directly, instead use an appropriate function.
    */
    typedef struct bmp180
    {
        struct i2c_information i2c;
        struct bmp180_calibration_data calibration_data;
        struct bmp180_configuration config;

        uint16_t rawTemperature, rawPressure;
        double temperature, temperatureF, pressure, seaLevel, altitude;
    } bmp180_t;

    /**
     * Initializes a BMP180 structure and returns it. 
     * 
     * @param i2c_inst_t* i2c_instance: I2C bus instance. Needed for multicore applications.
     * @returns bmp180_t
    */
    struct bmp180 bmp180_init(i2c_inst_t *i2c_instance);

    /**
     * Starts a communication with the BMP180. It reads all the registers and 
     * calculates the different constants. 
     * 
     * @param bmp180_t* self: Reference to itself
     * @returns uint8_t: 1 if success, otherwise 0
    */
    uint8_t bmp180_begin(struct bmp180 *self);

    /**
     * Reads all the needed registers and saves their results into a buffer.
     * 
     * @param bmp180_t* self: Reference to itself
     * @returns uint8_t: 1 if success, otherwise 0
    */
    uint8_t bmp180_event(struct bmp180 *self);

    /**
     * Enables or disables the reading of the temperature register. 
     * If state is set to 1 then the temperature will be read. Otherwise 
     * it will be skipped. 
     * 
     * @param bmp180_t* self: Reference to itself 
     * @param uint8_t state: Sets the state of temperature measuring 
    */
    void bmp180_set_temperature_measuring(struct bmp180 *self, uint8_t state);

    /**
     * Enables or disables the reading of the pressure register. 
     * If state is set to 1 then the pressure will be read. Otherwise 
     * it will be skipped. 
     * 
     * @param bmp180_t* self: Reference to itself
     * @param uint8_t state: Sets the state of pressure measuring
    */
    void bmp180_set_pressure_measuring(struct bmp180 *self, uint8_t state);

    /**
     * Changes the sampling rate of the BMP180. 
     * 0 - Lowest to 3 - Highest 
     * 
     * More sampling means more accurate data, but also a bigger delay. 
     * 
     * @param bmp180_t* self: Reference to itself
     * @param uint8_t oversampling: Oversampling rate
    */
    void bmp180_set_pressure_sampling(struct bmp180 *self, uint8_t oversampling);

    /**
     * Returns the temperature in celsius. 
     * You must have activated temperature readings beforehand. 
     * You must call bmp180_event(self) before. 
     * 
     * @param bmp180_t* self: Reference to itself
     * @returns double: Temperature in celsius
    */
    double bmp180_get_temperature_c(struct bmp180 *self);

    /**
     * Returns the temperature in fahrenheit. 
     * You must have activated temperatures readings beforehand. 
     * You must call bmp180_event(self) before. 
     * 
     * @param bmp180_t* self: Reference to itself
     * @returns double: Temperature in fahrenheit
    */
    double bmp180_get_temperature_f(struct bmp180 *self);

    /**
     * Returns the air pressure in mbar. 
     * You must have activated pressure readings beforehand. 
     * You must call bmp180_event(self) before. 
     * 
     * @param bmp180_t* self: Reference to itself
     * @returns double: Air pressure in mbar
    */
    double bmp180_get_pressure(struct bmp180 *self);

    /**
     * Returns the equivalent pressure at sea level. These pressure readings can be used for weather measurements.
     * 
     * @param bmp180_t* self: Reference to itself
     * @param double current_altitude: Altitude in meters
     * @returns double: Air pressure in mbar
    */
    double bmp180_get_sea_level(struct bmp180 *self, double current_altitude);

    /**
     * Returns the altitude of the current pressure and the passed baseline pressure.
     * 
     * @param bmp180_t* self: Reference to itself
     * @returns double: Altitude in meters above baseline
    */
    double bmp180_get_altitude(struct bmp180 *self, double fixed_pressure);

    /**
     * If any error occures, the most recent one can be fetched with this function.
     * The RPi Pico has usually one error code if the I2C communication does not work.
     *
     * PICO_ERROR_GENERIC = I2C error
     *
     * @returns int: Error code
    */
    int bmp180_get_error(struct bmp180 *self);

#ifdef __cplusplus
}
#endif

#endif