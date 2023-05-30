#include "mpu6050_lib.hpp"
#include "mahony_filter_lib.hpp"
#include "pwm_lib.hpp"

#include "hardware/adc.h"


double sigmoid(double n) {
    return (1.0 / (1.0 + pow(2.71828, -n)));
}

// maps the potentiometer value between 
double norm_map(double x)
{
    double z = (x - 0) / (4096.0 - 0) * 100.0;
    // uint16_t z = (x - 0) / (4096.0 - 0) * 10.0 + 3200.0;

    // z greater than 10% of the duty then keep it at 10%
    z = z > z*0.1 ? z*0.1 : z;

    return z;
}


int main()
{
    stdio_init_all();

    // imu set up
    // mpu6050 address
    static const uint8_t MPU6050_ADDR = 0x68;

    // i2c port
    i2c_inst_t * i2c = i2c0;

    // initialize ports for mpu
    int sda_pin = 4;
    int scl_pin = 5;

    // initialize the imu using the MPU6050 class
    // mpu6050.init(i2c0,MPU6050_ADDR,sda_pin,scl_pin);
    // MPU6050 mpu6050(i2c0,MPU6050_ADDR,sda_pin,scl_pin);

    // // set the full scale range of the gyro and accel
    // mpu6050.set_gyro_range(mpu6050.GYRO_RANGE_500);
    // mpu6050.set_accel_range(mpu6050.ACCEL_RANGE_8G);

    // // calibrate the gyro
    // mpu6050.cali_gyro(2000);
    // initialize the imu using the MPU6050 class

    // initialize the imu using the Mahony_Filter class
    Mahony_Filter mf(i2c0,MPU6050_ADDR,sda_pin,scl_pin);
    mf.set_gyro_range(mf.GYRO_RANGE_500);
    mf.set_accel_range(mf.ACCEL_RANGE_8G);
    mf.cali_gyro(2000);

    // gains
    double Kp = 3;
    double Ki = 0.2;   
    mf.set_gains(Kp,Ki);


    while(!stdio_usb_connected()) {
        sleep_ms(500);
    }
    printf("usb connected to pc\n");


    // loop time stuff
    absolute_time_t start;
    double dt = 0.004;       // 250 Hz
    // double dt = 0.5;


    uint pwm_pin = 15;
    uint servo_freq = 50;
    uint channel_b = 1; 
    PWM mot(pwm_pin,channel_b,servo_freq);


    // pot setup
    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);


    while(true)
    {

        /* testing mpu6050 class */
        // // output in seconds
        // start = get_absolute_time();

        // // mpu6050.get_accel_data();
        // // mpu6050.print_accel();

        // mpu6050.get_gyro_data();
        // mpu6050.print_gyro();

        // while( absolute_time_diff_us(start,get_absolute_time())/1000000.0 < dt );

        // compact for of above code
        // mpu6050.sample_imu(dt);


        /* testing mahony filter class */
        // mf.compute_mahony_filter(dt);
        // mf.print_euler_angles();



        /* testing potentiometer */
        start = get_absolute_time();

        uint16_t result = adc_read();
        double mod_result = norm_map(result);

        printf("result: %d, mapped result: %f \n", result, mod_result);

        while( absolute_time_diff_us(start,get_absolute_time())/1000000.0 < 0.005 );


        mot.change_duty(mod_result);

    }
}
