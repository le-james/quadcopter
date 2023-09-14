cmake_minimum_required(VERSION 3.16.3)

include(pico_sdk_import.cmake)

# project(test_project C)
project(test_project C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()



# add_library(pico_mpu6050 mpu6050lib/mpu6050.c mpu6050lib/mpu6050.h)   # alt method to add library's - other method is in library cmakelist.txt

add_subdirectory(mpu6050lib)


# alt method - other method is in library's cmakelist.txt
# other executables need to link the pico_mpu6050 library
target_include_directories(pico_mpu6050 PUBLIC mpu6050lib)


# main file (test.c) cmakelist.txt stuff

# main file for imu
add_executable(test_imu_main test_imu_main.c)
# links the libraries to main executable
target_link_libraries(test_imu_main PRIVATE hardware_timer pico_mpu6050)
pico_enable_stdio_usb(test_imu_main 1)
pico_enable_stdio_uart(test_imu_main 0)

# file for testing the accel
add_executable(accel_test accel_test.c)
target_link_libraries(accel_test PRIVATE hardware_timer pico_mpu6050)
pico_add_extra_outputs(accel_test)
pico_enable_stdio_usb(accel_test 1)
pico_enable_stdio_uart(accel_test 0)

# file for testing the complementary filter
add_executable(comp_filter_test comp_filter_test.c)
target_link_libraries(comp_filter_test PRIVATE hardware_timer pico_mpu6050)
pico_add_extra_outputs(comp_filter_test)
pico_enable_stdio_usb(comp_filter_test 1)
pico_enable_stdio_uart(comp_filter_test 0)

# file for testing the mahony filter
add_executable(mahony_filter_test mahony_filter.c)
target_link_libraries(mahony_filter_test PRIVATE hardware_timer pico_mpu6050)
pico_add_extra_outputs(mahony_filter_test)
pico_enable_stdio_usb(mahony_filter_test 1)
pico_enable_stdio_uart(mahony_filter_test 0)



# doesnt work - no file system implemented in c
add_executable(out_imu_data_txt out_imu_data_txt.c)
target_link_libraries(out_imu_data_txt PRIVATE hardware_timer pico_mpu6050)
# pico_add_extra_outputs(out_imu_data_txt)
pico_enable_stdio_usb(out_imu_data_txt 1)
pico_enable_stdio_uart(out_imu_data_txt 0)

# simple user input from terminal - interacting with pico
add_executable(user_input_test user_input_test.c)
target_link_libraries(user_input_test PRIVATE pico_mpu6050)
pico_enable_stdio_usb(user_input_test 1)
pico_enable_stdio_uart(user_input_test 0)
