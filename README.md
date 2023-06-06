# Quadcopter Flight Software
- Developed for the Raspberry Pi Pico microcontroller
- Uses the mpu6050 inertial measurement unit
- Attitude (quaternion and euler angle) estimation using the Mahony Filter
- PWM motor control


### Header-only Libraries
- mpu6050_lib.hpp - Class with functions to read and output gyroscope and accelerometer 
data from the IMU through I2C.

- mahony_filter.hpp - Inherits the mpu6050_lib.hpp class, implements the Mahony Filter for 
attitude estimation (Can output either quaternion or euler angles)

- pwm_lib.hpp - Class to setup a gpio pin for PWM mode. Useful for motor electronic speed 
controllers (ESC) and servo motors since they operate at 50Hz (Default PWM output frequency 
doesn't go as low at 50Hz)
