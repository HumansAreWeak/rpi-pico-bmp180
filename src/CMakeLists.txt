# Include source file(s) to project
add_library(haw_bmp180 STATIC
    BMP180.c
)

# Links all required dependencies
target_link_libraries(haw_bmp180 
    pico_stdlib
    hardware_i2c 
)

# Link include directory / Path to header file
target_include_directories(haw_bmp180 PUBLIC ../include)