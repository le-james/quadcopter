cmake_minimum_required(VERSION 3.16.3)
include(pico_sdk_import.cmake)

# set(CMAKE_CXX_FLAGS "-std=c++17 -03")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3")

project(Quadcopter_FC C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()


add_executable(${PROJECT_NAME} Quadcopter_FC.cpp)

# libraries linked to the mpu6050_lib too
# links the libraries to main executable
target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_adc)

# libraries that mpu6050_lib.hpp uses
target_link_libraries(${PROJECT_NAME} hardware_i2c hardware_timer)

# library that pwm_lib.hpp uses
target_link_libraries(${PROJECT_NAME} hardware_pwm)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
