add_library(imu mpu6050.c)

# pull in common dependencies and additional i2c hardware support
target_link_libraries(imu pico_stdlib)
# target_link_libraries(mpu6050_i2c pico_stdlib hardware_i2c)

# create map/bin/hex file etc.
pico_add_extra_outputs(imu)
