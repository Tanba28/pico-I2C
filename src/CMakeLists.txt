add_executable(pico-I2C
    pico_i2c.cpp
    )

target_include_directories(pico-I2C
    PUBLIC ${PROJECT_SOURCE_DIR}/src
    )

target_link_libraries(sensor_module
    pico_stdlib
    hardware_i2c
    freertos)