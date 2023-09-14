#include "mpu6050.h"

#include <math.h>
#include "hardware/timer.h"
// #include <time.h>
// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "hardware/i2c.h"



// functions not used

// ZYX (YAW, PITCH, ROLL)
void setDCM(double roll, double pitch, double yaw){
    
    // row 1 of DCM
    double c11 = cos(pitch)*cos(yaw);
    double c12 = cos(pitch)*sin(yaw);
    double c13 = -sin(pitch);

    // row 2 of DCM
    double c21 = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
    double c22 = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw);
    double c23 = sin(roll)*cos(pitch);

    // row 3 of DCM
    double c31 = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
    double c32 = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
    double c33 = cos(roll)*cos(pitch);

}

// outputs a normalized quaternion given the ZYX DCM elements
void DCM2quat(double c11, double c12, double c13, double c21, double c22, double c23, double c31, double c32, double c33){

    // computer intermediate quaternion
    double q_int_0 = sqrt(0.25*(1 + c11 + c22 + c33));
    double q_int_1 = sqrt(0.25*(1 + c11 - c22 - c33));
    double q_int_2 = sqrt(0.25*(1 - c11 + c22 - c33));
    double q_int_3 = sqrt(0.25*(1 - c11 - c22 + c33));

    // array of intermediate quaternion
    double q_int[4] = {q_int_0, q_int_1, q_int_2, q_int_3};
    double max = q_int[0];

    // count
    int i;

    // find max element in the intermediate quaternion
    for(i = 1; i < 4; i++){
        if(max < q_int[i]){
            max = q_int[i];
        }
    }

    // determine the correct quaternion
    double q[4];
    if(max == q_int_0){
        q[0] = q_int_0;
        q[1] = 0.25*(c23-c32)/q[0];
        q[2] = 0.25*(c31-c13)/q[0];
        q[3] = 0.25*(c12-c21)/q[0];
    } else if(max == q_int_1) {
        q[0] = 0.25*(c23-c32)/q[1];
        q[1] = q[1];
        q[2] = 0.25*(c12-c21)/q[1];
        q[3] = 0.25*(c31-c13)/q[1];
    } else if(max == q_int_2) {
        q[0] = 0.25*(c31-c13)/q[2];
        q[2] = 0.25*(c12+c21)/q[2];
        q[1] = q[2];
        q[3] = 0.25*(c23+c32)/q[2];
    } else if(max == q_int_3) {
        q[0] = 0.25*(c12-c21)/q[3];
        q[1] = 0.25*(c31+c13)/q[3];
        q[2] = 0.25*(c23+c32)/q[3];
        q[3] = q[3];
    } 

    // compute norm of quaternion
    double q_norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[03]);

    // compute unit vector of quaternion
    for(i = 0; i < 4; i++){
        q[i] = q[i]/q_norm;
    }

}

// outputs DCM elements given a normalized quaternion
void quat2DCM(double q0, double q1, double q2, double q3){

    // roll
    double c23 = 2*(q2*q3 + q0*q1);
    double c33 = 1 - 2*(q1*q1 + q2*q2);

    // pitch
    double c13 = 2*(q1*q3 - q0*q2);

    // yaw
    double c11 = 1 - 2*(q2*q2 + q3*q3);
    double c12 = 2*(q1*q2 + q0*q3);

}

// functions not used



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
    set_gyro_range(MPU6050_ADDR,i2c,0x10);
    // read_gyro_range(MPU6050_ADDR,i2c,0);

    double scl = 32.8;

    absolute_time_t start;
    double time = 0;       // used to print to terminal every 1 sec
    double dt = 0.01;    // 100hz

    // initialize the gyro pitch and roll with accel pitch and roll
    get_accel_data(MPU6050_ADDR,i2c,&ax_raw,&ay_raw,&az_raw,&ax,&ay,&az);
    double p_ratio = ax/sqrt((ax*ax + ay*ay + az*az));
    double roll = atan2f(ay,az);
    double pitch = asin(p_ratio);
    double yaw = 0;

    // initialize quaternion
    double q0 = 1;
    double q1 = 0;
    double q2 = 0;
    double q3 = 0;

    // initialize bias error
    double b1 = 0;
    double b2 = 0;
    double b3 = 0;

    // gains
    int Kp = 0.04;
    // int Kp1 = 0.08;       
    // int Kp2 = 0.08;       
    // int Kp3 = 0;
 
    int Ki = 0.0002;     
    // int Ki1 = 0.1;      
    // int Ki2 = 0.1;      
    // int Ki3 = 0;      

    // calibrate gyro
    int num_samp = 2000;
    double gx_cal = 0;
    double gy_cal = 0;
    double gz_cal = 0;

    for (int i = 0; i < num_samp; i++){
        get_gyro_rates(MPU6050_ADDR,i2c,&gx_raw,&gy_raw,&gz_raw,&gx,&gy,&gz);
        gx_cal += gx_raw/scl;
        gy_cal += gy_raw/scl;
        gz_cal += gz_raw/scl;
        sleep_ms(1);
    }

    // deg/s
    double gx_off = gx_cal/num_samp;
    double gy_off = gy_cal/num_samp;
    double gz_off = gz_cal/num_samp;


    while (1) {
        start = get_absolute_time();
        // hello_world(5);


        // get accel data
        get_accel_data(MPU6050_ADDR,i2c,&ax_raw,&ay_raw,&az_raw,&ax,&ay,&az);

        // calibrated accel values
            // THIS DOES NOT WORK WITH THE MAHONY FILTER
        ax*1.004616 - 0.49043;
        ay*1.00279 + 0.141377;
        az*1.016184 + 0.645471;

        // step 1 compute unit vector of accelerometer vector
        double a_norm = sqrt(ax*ax + ay*ay + az*az);
        // if(a_norm > GRAVITY*1.3){
        //     Kp = 10;
        //     Ki = 1;   
        // } else {
        //     Kp = 0.2;
        //     Ki = 0.3;     
        // }

        ax = ax/a_norm;
        ay = ay/a_norm;
        az = az/a_norm;

        // step 2 previous estimated quaternion to DCM
        // quat2DCM(q0, q1, q2, q3);
        // third coloum of the DCM is the z axis basis vector - points upwards
        double c13 = 2*(q1*q3 - q0*q2);
        double c23 = 2*(q2*q3 + q0*q1);
        double c33 = 1 - 2*(q1*q1 + q2*q2);
        // double c33 = (q0*q0 - q1*q1 - q2*q2 + q3*q3);

        // [c13 c23 c33] and [ax ay az] both point upwards

        // step 3 compute error between accel vector and previous normalized estimate of inertial vector
        // double wmes1 = c33*ay - az*c23;
        // double wmes2 = c33*ax - ay*c13;
        // double wmes3 = c13*az - ax*c33;
        double wmes1 = c33*ay - az*c23;
        double wmes2 = c13*az - ax*c33;
        double wmes3 = c23*ax - ay*c13;

        // step 4 integrate bias
        b1 -= Ki*wmes1*dt;
        b2 -= Ki*wmes2*dt;
        b3 -= Ki*wmes3*dt;

        // b1 += -Ki1*wmes1*dt;
        // b2 += -Ki2*wmes2*dt;
        // b3 += -Ki3*wmes3*dt;


        // get gyro data
        get_gyro_rates(MPU6050_ADDR,i2c,&gx_raw,&gy_raw,&gz_raw,&gx,&gy,&gz);
        // printf("%d,%f,%f,%f,%f,%f,%f\n",iter,ax,ay,az,gx,gy,gz);
        // printf("ax:%f, ay:%f, az:%f\n",ax,ay,az);
        // printf("gx:%f, gy:%f, gz:%f\n",gx,gy,gz);

        // calibrated gyro rates
        double gxx = (gx_raw/scl - gx_off) * 3.14159265359/180;
        double gyy = (gy_raw/scl - gy_off) * 3.14159265359/180;
        double gzz = (gz_raw/scl - gz_off) * 3.14159265359/180;
        // printf("gxx:%f, gyy:%f, gzz:%f\n",gxx,gyy,gzz);

        // yaw += gzz*dt * 180/3.14159265359f;

        // integrate quaternion kinematics
        // q0 += ( 0.5*(-(gxx + b1 + Kp*wmes1)*q1 - (gyy + b2 + Kp*wmes2)*q2 - (gzz + b3 + Kp*wmes3)*q3) )*dt;
        // q1 += ( 0.5*((gxx + b1 + Kp*wmes1)*q0 + (gzz + b3 + Kp*wmes3)*q2 - (gyy + b2 + Kp*wmes2)*q3) )*dt;
        // q2 += ( 0.5*((gyy + b2 + Kp*wmes2)*q0 - (gzz + b3 + Kp*wmes3)*q1 + (gxx + b1 + Kp*wmes1)*q3) )*dt;
        // q3 += ( 0.5*((gzz + b3 + Kp*wmes3)*q0 + (gyy + b2 + Kp*wmes2)*q1 - (gxx + b1 + Kp*wmes1)*q2) )*dt;

        q0 += ( 0.5*(-(gxx - b1 + Kp*wmes1)*q1 - (gyy - b2 + Kp*wmes2)*q2 - (gzz - b3 + Kp*wmes3)*q3) )*dt;
        q1 += ( 0.5*((gxx - b1 + Kp*wmes1)*q0 + (gzz - b3 + Kp*wmes3)*q2 - (gyy - b2 + Kp*wmes2)*q3) )*dt;
        q2 += ( 0.5*((gyy - b2 + Kp*wmes2)*q0 - (gzz - b3 + Kp*wmes3)*q1 + (gxx - b1 + Kp*wmes1)*q3) )*dt;
        q3 += ( 0.5*((gzz - b3 + Kp*wmes3)*q0 + (gyy - b2 + Kp*wmes2)*q1 - (gxx - b1 + Kp*wmes1)*q2) )*dt;

        double q_norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);  
        q0 = q0/q_norm;
        q1 = q1/q_norm;
        q2 = q2/q_norm;
        q3 = q3/q_norm;

        // DCM elements given a normalized quaternion
        // roll
        c23 = 2*(q2*q3 + q0*q1);
        c33 = 1 - 2*(q1*q1 + q2*q2);
        // c33 = (q0*q0 - q1*q1 - q2*q2 + q3*q3);

        // pitch
        c13 = 2*(q1*q3 - q0*q2);

        // yaw
        double c11 = 1 - 2*(q2*q2 + q3*q3);
        // double c11 = (q0*q0 + q1*q1 - q2*q2 - q3*q3);
        double c12 = 2*(q1*q2 + q0*q3);

        // euler angles from quaternion
        roll = atan2(c23,c33) * 180/3.14159265359;
        pitch = asin(c13) * 180/3.14159265359;


        // if(pitch)
        yaw = atan2(c12,c11) * 180/3.14159265359;

        // yaw when pitch is at +-90 deg
        double yawp90 = 2*atan2(q1,q0) * 180/3.14159265359;
        // double yawn90 = -2*atan2(q1,q0) * 180/3.14159265359;

        time += dt;

        if(time > 0.5){
            // print roll pitch and yaw
            printf("roll: %0.2f, pitch: %0.2f, yaw: %0.2f\n",roll,pitch,yaw);
            printf("roll: %0.2f, pitch: %0.2f, yaw: %0.2f\n",roll,pitch,yawp90);

            // DCM elements
            // printf("c23: %0.2f, c33: %0.2f\n",c23,c33);

            // print yaw values
            // printf("yaw: %0.2f, yaw at 90: %0.2f, yaw: %0.2f\n",yaw,yawp90,yawn90);

            // quaternion vector
            // printf("q0: %0.2f, q1: %0.2f, q2: %0.2f, q3: %0.2f\n",q0,q1,q2,q3);
            time = 0;
        }

        while(absolute_time_diff_us(start,get_absolute_time())/1000000.0 < dt);

    }
}