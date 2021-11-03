# Create an executable
add_executable(bmp180_temperature_pressure_read 
    bmp180_temperature_pressure_read.c
)

# Link all required libraries
target_link_libraries(bmp180_temperature_pressure_read 
    pico_stdlib
    hardware_i2c 
    haw_bmp180
)

# Enable printf output for USB communication
pico_enable_stdio_usb(bmp180_temperature_pressure_read 1)
pico_enable_stdio_uart(bmp180_temperature_pressure_read 0)

# Add the .uf2 output to the build directory
pico_add_extra_outputs(bmp180_temperature_pressure_read)