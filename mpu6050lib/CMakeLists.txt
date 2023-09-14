# add_library(pico_mpu6050 mpu6050.c mpu6050.h)
add_library(pico_mpu6050 mpu6050.c)     # alt method

# pull in common dependencies
target_link_libraries(pico_mpu6050 pico_stdlib hardware_i2c)

# alt method from the root cmakelist.txt
# target_include_directories(pico_mpu6050 PUBLIC ../include/)
# target_include_directories(pico_mpu6050 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# can add messages - after saving the terminal will pop up and this message will be in there
message("Full path to module: ${CMAKE_CURRENT_SOURCE_DIR}")