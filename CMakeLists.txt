cmake_minimum_required(VERSION 3.16.3)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include(pico_sdk_import.cmake)

pico_sdk_init()

project(drone C CXX ASM)

add_executable(${PROJECT_NAME} flight_controller.c)

add_subdirectory(imu)

target_link_libraries(${PROJECT_NAME} imu pico_stdlib)
# target_link_libraries(mpu6050_i2c pico_stdlib hardware_i2c)


