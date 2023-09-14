#include "mpu6050.h"

#include <math.h>
#include "hardware/timer.h"
// #include <time.h>
// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "hardware/i2c.h"

int main() {

    stdio_init_all();

    // mpu6050 address
    static const uint8_t MPU6050_ADDR = 0X68;

    // Ports
    i2c_inst_t * i2c = i2c0;

    // init ports for mpu
    int sda_pin = 4;
    int scl_pin = 5;
    init_mpu6050(MPU6050_ADDR,i2c0,sda_pin,scl_pin);

    while(!stdio_usb_connected()) {
        sleep_ms(500);
    }
    printf("usb connected to pc\n");


    // set accel range
    set_accel_range(MPU6050_ADDR,i2c,0x10);
    // read_accel_range(MPU6050_ADDR,i2c,1);

    // set gyro range
    set_gyro_range(MPU6050_ADDR,i2c,0x08);
    // read_gyro_range(MPU6050_ADDR,i2c,0);



    absolute_time_t start;
    float dt = 0.01;    // 10ms

    // initialize the gyro pitch and roll with accel pitch and roll
    get_accel_data(MPU6050_ADDR,i2c,&ax_raw,&ay_raw,&az_raw,&ax,&ay,&az);
    float p_ratio = ax/sqrt((ax*ax + ay*ay + az*az));
    float roll_g = atan2f(ay,az)*180/3.1415926;
    float pitch_g = asinf(p_ratio)*180/3.1415926;
    float yaw_g = 0;

    // calibrate gyro
    int num_samp = 2000;
    float gx_cal = 0;
    float gy_cal = 0;
    float gz_cal = 0;

    for (int i = 0; i < num_samp; i++){
        get_gyro_rates(MPU6050_ADDR,i2c,&gx_raw,&gy_raw,&gz_raw,&gx,&gy,&gz);
        gx_cal += gx_raw/65.5;
        gy_cal += gy_raw/65.5;
        gz_cal += gz_raw/65.5;
        sleep_ms(1);
    }

    // deg/s
    float gx_off = gx_cal/num_samp;
    float gy_off = gy_cal/num_samp;
    float gz_off = gz_cal/num_samp;

    while (1) {
        start = get_absolute_time();
        // hello_world(5);


        // get accel data
        get_accel_data(MPU6050_ADDR,i2c,&ax_raw,&ay_raw,&az_raw,&ax,&ay,&az);
        // calibrated accel values
        ax*1.004616 - 0.49043;
        ay*1.00279 + 0.141377;
        az*1.016184 + 0.645471;

        // get gyro data
        get_gyro_rates(MPU6050_ADDR,i2c,&gx_raw,&gy_raw,&gz_raw,&gx,&gy,&gz);
        // printf("%d,%f,%f,%f,%f,%f,%f\n",iter,ax,ay,az,gx,gy,gz);
        // printf("ax:%f, ay:%f, az:%f\n",ax,ay,az);
        // printf("gx:%f, gy:%f, gz:%f\n",gx,gy,gz);


        // calibrated gyro rates
        float gxx = gx_raw/65.5 - gx_off;
        float gyy = gy_raw/65.5 - gy_off;
        float gzz = gz_raw/65.5 - gz_off;
        // printf("gxx:%f, gyy:%f, gzz:%f\n",gxx,gyy,gzz);

        // pitch roll and yaw from gyro
        roll_g += gxx*dt;
        pitch_g -= gyy*dt;
        yaw_g += gzz*dt;
        // printf("dt:%f, gx_ang:%f, gy_ang:%f, gz_ang:%f\n",dt,gx_ang,gy_ang,gz_ang);


        // pitch and roll from accelerometer
        float pitch_ratio = ax/sqrt((ax*ax + ay*ay + az*az));

        float pitch_a = asinf(pitch_ratio)*180/3.1415926;
        float roll_a = atan2f(ay,az)*180/3.1415926;
        // printf("accel pitch: %f, accel roll: %f\n",p_accel,r_accel);

        // roll and pitch using complementary filter
        float p_cf = 0.96*pitch_g + 0.04*pitch_a;
        float r_cf = 0.96*roll_g + 0.04*roll_a;
        printf("pitch: %f, roll: %f\n",p_cf,r_cf);


        while(absolute_time_diff_us(start,get_absolute_time())/1000000.0 < dt);

    }
}