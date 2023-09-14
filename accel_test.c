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

    int iter = 0;

    absolute_time_t start;
    float dt = 0.004;
    // float dt = 1;

    // set accel range
    set_accel_range(MPU6050_ADDR,i2c,0x10);
    // read_accel_range(MPU6050_ADDR,i2c,1);

    while (1) {
        start = get_absolute_time();

        // accel func tests
        get_accel_data(MPU6050_ADDR,i2c,&ax_raw,&ay_raw,&az_raw,&ax,&ay,&az);

        printf("ax:%f, ay:%f, az:%f\n", ax*1.004616-0.49043, ay*1.00279+0.141377, az*1.016184+0.645471);

        // x
        // printf("%f\n",ax);
        // y
        // printf("%f\n",ay);
        // z
        // printf("%f\n",az);

        iter++;

        // pitch and roll from accelerometer
        // float p_ratio = ax/GRAVITY;
        // float p = asin(p_ratio)*180/3.1415926;
        // printf("pitch: %f\n",p);
        
        while(absolute_time_diff_us(start,get_absolute_time())/1000000.0 < dt);

    }
}