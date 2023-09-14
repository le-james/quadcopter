#include "mpu6050.h"

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

    // set the range
    set_accel_range(MPU6050_ADDR,i2c,0x00);
    set_gyro_range(MPU6050_ADDR,i2c,0x00);


// DOESN'T WORK!!!
// wasted time doing this shiet and found out that c doesn't have a file system implementation
// so this doesn't work - lucky there is an easy way to get the data just using the usb connection

/* 
    // //  number of imu data point to collect
    // int n_iter = 420;
    // int curr_iter = 0;

    // // stuct to store accel and gyro outputs
    // typedef struct imu_data{
    //     float ax, ay, az, gx, gy, gz;
    // } imu_data;


    // // create struct
    // imu_data ax_data[n_iter];
    // imu_data ay_data[n_iter];
    // imu_data az_data[n_iter];

    // imu_data gx_data[n_iter];
    // imu_data gy_data[n_iter];
    // imu_data gz_data[n_iter];


    // while (1) {

    //     // get accel data
    //     get_accel_data(MPU6050_ADDR,i2c,&ax_raw,&ay_raw,&az_raw,&ax,&ay,&az);

    //     // get gyro rates
    //     get_gyro_rates(MPU6050_ADDR,i2c,&gx_raw,&gy_raw,&gz_raw,&gx,&gy,&gz);

    //     // store imu data in the stuct
    //     ax_data[curr_iter].ax = ax;
    //     ay_data[curr_iter].ay = ay;
    //     az_data[curr_iter].az = az;

    //     gx_data[curr_iter].gx = gx;
    //     gy_data[curr_iter].gy = gy;
    //     gz_data[curr_iter].gz = gz;

    //     // continue or finish after collecting all data
    //     if (curr_iter < n_iter){
    //         curr_iter++;
    //     } else {
    //         break;
    //     }

    //     printf("az: %f\n", az);

    //     sleep_ms(1000);
    // }

    // // pointer to file to write into - "file" stores the pointer - new txt file created in the curr directory
    // FILE *file = fopen("newtext.txt","w");

    // // check if open the file
    // if (file == NULL){
    //     return 1;   // return 1 means error - displays in terminal
    // }

    // // output the struct data into a txt file
    // for (int i = 0; i < n_iter; i++){
    //     fprintf(file,
    //             "%f,%f,%f,%f,%f,%f\n",

    //             ax_data[i].ax,
    //             ay_data[i].ay,
    //             az_data[i].az,

    //             gx_data[i].gx,
    //             gy_data[i].gy,
    //             gz_data[i].gz);

    //     // return error (true) of the last function call to work on file i.e. fprintf
    //     if (ferror(file)){
    //         printf("error writing to file\n");
    //         return 1;
    //     }
    // }

    // fclose(file);
*/

    while (1){
        printf("sleepin\n");
        sleep_ms(1000);
    }
}
