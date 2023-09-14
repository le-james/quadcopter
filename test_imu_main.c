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

    // DOES WORK BUT COMMAND HAS TO BE THROUGH TERMINAL - CHECK OUT user_input_test.c
    // not working for some reason - need to import lib? - obivous it doesn't work egghead,
    // there is no keyboard attached to the pico
    // int cont;
    // printf("Press 1 to continue: ");
    // scanf("%d", &cont);  
    // printf("Pressed = %d",cont);

    // exits when run cat tty/ACM# - so the program doesn't start when im not ready
    while(!stdio_usb_connected()) {
        sleep_ms(500);
    }
    printf("usb connected to pc\n");

    int iter = 0;

    absolute_time_t start;
    float timer = 0;
    float dt = 0.004;

    float gx_ang = 0;
    float gy_ang = 0;
    float gz_ang = 0;

    // set accel range
    set_accel_range(MPU6050_ADDR,i2c,0x00);
    // read_accel_range(MPU6050_ADDR,i2c,1);

    // set gyro range
    set_gyro_range(MPU6050_ADDR,i2c,0x08);
    // read_gyro_range(MPU6050_ADDR,i2c,0);


    // calibrate gyro
    int num_samp = 2000;
    float gx_cal = 0;
    float gy_cal = 0;
    float gz_cal = 0;

    // float gx_scl = 1.174444;
    // float gy_scl = 1.163333;
    // float gz_scl = 1.201111;
    float gx_scl = 1;
    float gy_scl = 1;
    float gz_scl = 1;

    for (int i = 0; i < num_samp; i++){
        get_gyro_rates(MPU6050_ADDR,i2c,&gx_raw,&gy_raw,&gz_raw,&gx,&gy,&gz);
        gx_cal += gx_raw*gx_scl/65.5;
        gy_cal += gy_raw*gy_scl/65.5;
        gz_cal += gz_raw*gz_scl/65.5;
        sleep_ms(1);
    }

    // deg/s
    float gx_off = gx_cal/num_samp;
    float gy_off = gy_cal/num_samp;
    float gz_off = gz_cal/num_samp;

    while (1) {
        start = get_absolute_time();
        // hello_world(5);


        // accel func tests
        // get_accel_data(MPU6050_ADDR,i2c,&ax_raw,&ay_raw,&az_raw,&ax,&ay,&az);
        // printf("ax:%f, ay:%f, az:%f\n",ax,ay,az);

        // gyro func tests
        get_gyro_rates(MPU6050_ADDR,i2c,&gx_raw,&gy_raw,&gz_raw,&gx,&gy,&gz);

        // printf("%d,%f,%f,%f,%f,%f,%f\n",iter,ax,ay,az,gx,gy,gz);
        // printf("ax:%f, ay:%f, az:%f\n",ax,ay,az);
        // printf("gx:%f, gy:%f, gz:%f\n",gx,gy,gz);
        // iter++;


        // print gyro offsets
        // printf("gx_cal:%f, gy_cal:%f, gz_cal:%f\n",gx_cal,gy_cal,gz_cal);

        float gxx = gx_raw*gx_scl/65.5 - gx_off;
        float gyy = gy_raw*gy_scl/65.5 - gy_off;
        float gzz = gz_raw*gz_scl/65.5 - gz_off;
        // printf("gxx:%f, gyy:%f, gzz:%f\n",gxx,gyy,gzz);


        // pitch and roll from accelerometer
        // float p_ratio = ax/GRAVITY;
        // float p = asin(p_ratio)*180/3.1415926;
        // printf("pitch: %f\n",p);


        // pitch roll and yaw from gyro
        gx_ang += gxx*dt;
        gy_ang += gyy*dt;
        gz_ang += gzz*dt;
        printf("dt:%f, gx_ang:%f, gy_ang:%f, gz_ang:%f\n",dt,gx_ang,gy_ang,gz_ang);
        // printf("%f\n",gy_ang);

        // end = clock();

        // dt = end-start+0.004;
        // // dt = 0.004;
        // // dt = 1000;
        // sleep_ms(dt*1000);


        // // coast until next interval
        // while(timer <= 4){
        //     sleep_ms(0.1);
        //     timer += 0.1;
        // }

        // // reset timer
        // timer = 0;

        while(absolute_time_diff_us(start,get_absolute_time())/1000000.0 < dt);

    }
}