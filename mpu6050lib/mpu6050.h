/* mpu6050.h */
#include <stdio.h>
// #include <stdint.h>  
#include "pico/stdlib.h"
#include "hardware/i2c.h"


void hello_world(uint8_t a);


// earth gravity
static const double GRAVITY = 9.80665;


/* gyroscope parameters */

int16_t gx_raw, gy_raw, gz_raw;
double gx, gy, gz;

// static const double gx_corr = 2.694708049;
// static const double gy_corr = 0.7979291351;
// static const double gz_corr = -0.04956675169;

static const double gx_corr = 0;
static const double gy_corr = 0;
static const double gz_corr = 0;

// gyro config register - used to change the gyro range
static const uint8_t GYRO_CONFIG = 0x1B;

// gyro ranges
static const uint8_t GYRO_RANGE_2G = 0x00;
static const uint8_t GYRO_RANGE_4G = 0x08;
static const uint8_t GYRO_RANGE_8G = 0x10;
static const uint8_t GYRO_RANGE_16G = 0x18;

// gyroscope scale modifiers
static const double GYRO_SEN_SCL_FAC_250 = 131.0;
static const double GYRO_SEN_SCL_FAC_500 = 65.5;
static const double GYRO_SEN_SCL_FAC_1000 = 32.8;
static const double GYRO_SEN_SCL_FAC_2000 = 16.4;

/* gyroscope parameters */


/* accelerometer parameters */

int16_t ax_raw, ay_raw, az_raw;
double ax, ay, az;

// static const double ax_corr = -0.4210398024;
// static const double ay_corr = -0.2922786284;
// static const double az_corr = 0.4695140845;
static const double ax_corr = 0;
static const double ay_corr = 0;
static const double az_corr = 0;

// accel config register - used to change the accel range
static const uint8_t ACCEL_CONFIG = 0x1C;

// accel ranges
static const uint8_t ACCEL_RANGE_2G = 0x00;
static const uint8_t ACCEL_RANGE_4G = 0x08;
static const uint8_t ACCEL_RANGE_8G = 0x10;
static const uint8_t ACCEL_RANGE_16G = 0x18;

// accelerometer scale modifiers
static const int ACCEL_SEN_SCL_FAC_2G = 16384;
static const int ACCEL_SEN_SCL_FAC_4G = 8192;
static const int ACCEL_SEN_SCL_FAC_8G = 4096;
static const int ACCEL_SEN_SCL_FAC_16G = 2048;

/* accelerometer parameters */


// initialize imu function
void init_mpu6050(uint8_t MPU6050_ADDR, i2c_inst_t * i2c, uint8_t PIN_SDA, uint8_t PIN_SCL);

// accelerometer functions
void set_accel_range(uint8_t MPU6050_ADDR, i2c_inst_t * i2c, uint8_t accel_range);
uint8_t read_accel_range(uint8_t MPU6050_ADDR, i2c_inst_t * i2c, bool isPrint);
void get_accel_data(uint8_t MPU6050_ADDR, i2c_inst_t *i2c, \
                    int16_t *ax_raw, int16_t *ay_raw, int16_t *az_raw, \
                    double *ax, double *ay, double *az);

// gyroscope functions
void set_gyro_range(uint8_t MPU6050_ADDR, i2c_inst_t * i2c, uint8_t gyro_range);
uint8_t read_gyro_range(uint8_t MPU6050_ADDR, i2c_inst_t * i2c, bool isPrint);
void get_gyro_rates(uint8_t MPU6050_ADDR, i2c_inst_t *i2c, \
                    int16_t *gx_raw, int16_t *gy_raw, int16_t *gz_raw, \
                    double *gx, double *gy, double *gz);
