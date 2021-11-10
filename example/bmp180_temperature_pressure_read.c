#include "stdio.h"
#include "pico/stdlib.h"
#include "haw/BMP180.h"

int main(void)
{
    stdio_init_all();

    // Setup I²C properly
    i2c_init(i2c_default, 100 * 1000);
    gpio_init(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_init(PICO_DEFAULT_I2C_SCL_PIN);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    // Don't forget the pull ups!
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Pass in the I²C driver (important for dual-core code)
    bmp180_t bmp = bmp180_init(i2c_default);

    // Check if BMP can initialize
    if (bmp180_begin(&bmp))
    {
        // Enable temperature measurings
        bmp180_set_temperature_measuring(&bmp, 1);

        // Enable pressure measurings
        bmp180_set_pressure_measuring(&bmp, 1);

        //with an oversampling rate of 3
        // You can choose from 0 (lowest) to 3 (highest), which will
        // differ in the delay time
        bmp180_set_pressure_sampling(&bmp, 3);
    }
    else
    {
        printf("Error! The BMP180 could not be initialized.");

        while (1) // Endless loop if BMP180 could not start
            ;
    }

    while (1)
    {
        // BMP fetches all the data | I²C bus is only used here
        bmp180_event(&bmp);

        // Print all the measurements
        printf("Temperature: %f°C - Temperature: %f°F - Pressure: %fhPa (mbar) - Altitude(Compared to 1013.25): %fhPa (mbar) - @SeaLevel(10): %fhPa (mbar)\n", bmp180_get_temperature_c(&bmp), bmp180_get_temperature_f(&bmp), bmp180_get_pressure(&bmp), bmp180_get_altitude(&bmp, 1013.25), bmp180_get_sea_level(&bmp, 10.0));
    }

    return 0;
}