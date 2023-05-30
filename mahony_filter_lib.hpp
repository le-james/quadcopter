#ifndef MAHONY_FILTER_LIB_HPP
#define MAHONY_FILTER_LIB_HPP

#include <math.h>
#include "mpu6050_lib.hpp"

/** \file mahony_filter_lib.hpp
 * Computes a quaternion and euler angles by fusing the accelerometer and gyroscope output.
 * compute_mahony_filter(double dt) loop solves just less than 1 millisecond
 */
class Mahony_Filter : public MPU6050
{
    public:
        // for initializer list of Mahony_Filter constructor
        // // imu parameters
        // i2c_inst_t *i2c;
        // const uint8_t MPU6050_ADDR;

        // // data line pins
        // const uint8_t PIN_SDA;
        // const uint8_t PIN_SCL;


        // initialize quaternion
        double q0 = 1, q1 = 0, q2 = 0, q3 = 0;

        // bias error
        double b1, b2, b3;
        double prev_b1 = 0, prev_b2 = 0, prev_b3 = 0;

        double time;

        // mahony filter gains
        double Kp, Ki;

        // euler angles
        double roll, pitch, yaw;

    public:
        // inherits all MPU6050 constructors
        // using MPU6050::MPU6050;
        // or
        Mahony_Filter(i2c_inst_t *i2c, const uint8_t MPU6050_ADDR, const uint8_t PIN_SDA, const uint8_t PIN_SCL) 
            : MPU6050(i2c, MPU6050_ADDR, PIN_SDA, PIN_SCL)
            // uncomment the line below only if we want to initialize this variables in this class
            // , i2c(i2c), MPU6050_ADDR(MPU6050_ADDR), PIN_SDA(PIN_SDA), PIN_SCL(PIN_SCL)
        {

        }

        /** \brief Set the proportional and integral gains of the Mahony Filter
         * \param Kp Proportional gain
         * \param Ki Integral gain
         */
        void set_gains(double Kp, double Ki)
        {
            this->Kp = Kp;
            this->Ki = Ki;
        }

        /** \brief Mahony Filter algorithm
         * \note Takes about 1 millisecond to solve
         * \param dt Time between samples. 1/dt is the freqency of the sample rate.
         */
        void compute_mahony_filter(double dt)
        {
            // output in seconds
            absolute_time_t start = get_absolute_time();

            // read accelerometer values
            get_accel_data();

            // step 1 compute unit vector of accelerometer vector
            // compute the norm of the acceleration vector
            ax = ax/sqrt(ax*ax + ay*ay + az*az);
            ay = ay/sqrt(ax*ax + ay*ay + az*az);
            az = az/sqrt(ax*ax + ay*ay + az*az);

            // step 2 previous estimated quaternion to DCM
            double c13 = 2*(q1*q3 - q0*q2);
            double c23 = 2*(q2*q3 + q0*q1);
            double c33 = 1 - 2*(q1*q1 + q2*q2);
            // double c33 = (q0*q0 - q1*q1 - q2*q2 + q3*q3);

            // step 3 compute error between accel vector and previous normalized estimate of inertial vector
            double wmes1 = c33*ay - az*c23;
            double wmes2 = c13*az - ax*c33;
            double wmes3 = c23*ax - ay*c13;

            // step 4 integrate bias (error dyanmics)
            b1 -= Ki*wmes1*dt;
            b2 -= Ki*wmes2*dt;
            b3 -= Ki*wmes3*dt;

            // clegg integrator - zero the integrator after crossing zero
            if(b1 < 0 && prev_b1 > 0) {         // crosses 0 from positive to negative
                b1 = 0;
            } else if(b1 > 0 && prev_b1 < 0) {  // crosses 0 from negative to positive 
                b1 = 0;  
            }

            if(b2 < 0 && prev_b2 > 0) {         // crosses 0 from positive to negative
                b2 = 0;
            } else if(b2 > 0 && prev_b2 < 0) {  // crosses 0 from negative to positive 
                b2 = 0;  
            }

            if(b3 < 0 && prev_b3 > 0) {         // crosses 0 from positive to negative
                b3 = 0;
            } else if(b3 > 0 && prev_b3 < 0) {  // crosses 0 from negative to positive 
                b3 = 0;  
            }

            prev_b1 = b1;
            prev_b2 = b2;
            prev_b3 = b3;

            // step 5 integrate quaternion kinematics
            // get gyro rates [deg/s]
            get_gyro_data();

            // convert to [rad/s]
            double gxx = gx * 3.14159265359/180;
            double gyy = gy * 3.14159265359/180;
            double gzz = gz * 3.14159265359/180;

            // integrate the quaternion kinematics
            q0 += ( 0.5*(-(gxx - b1 + Kp*wmes1)*q1 - (gyy - b2 + Kp*wmes2)*q2 - (gzz - b3 + Kp*wmes3)*q3) )*dt;
            q1 += ( 0.5*( (gxx - b1 + Kp*wmes1)*q0 + (gzz - b3 + Kp*wmes3)*q2 - (gyy - b2 + Kp*wmes2)*q3) )*dt;
            q2 += ( 0.5*( (gyy - b2 + Kp*wmes2)*q0 - (gzz - b3 + Kp*wmes3)*q1 + (gxx - b1 + Kp*wmes1)*q3) )*dt;
            q3 += ( 0.5*( (gzz - b3 + Kp*wmes3)*q0 + (gyy - b2 + Kp*wmes2)*q1 - (gxx - b1 + Kp*wmes1)*q2) )*dt;

            // compute norm of quaternion
            q0 = q0/sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
            q1 = q1/sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
            q2 = q2/sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
            q3 = q3/sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

            // step 6 update the DCM elements given a new normalized quaternion to compute the euler angles

            // roll
            c23 = 2*(q2*q3 + q0*q1);
            c33 = 1 - 2*(q1*q1 + q2*q2);
            // c33 = (q0*q0 - q1*q1 - q2*q2 + q3*q3);

            // pitch
            c13 = 2*(q1*q3 - q0*q2);

            // yaw
            double c11 = 1 - 2*(q2*q2 + q3*q3);
            double c12 = 2*(q1*q2 + q0*q3);


            /* NEED TO IMPLEMENT A GIMBAL LOCK CHECK OR SOMETHING*/


            // euler angles [deg] from quaternion
            roll = atan2(c23,c33) * 180/3.14159265359;
            pitch = asin(c13) * 180/3.14159265359;
            yaw = atan2(c12,c11) * 180/3.14159265359;

            while(absolute_time_diff_us(start,get_absolute_time())/1000000.0 < dt);
        }

        /** \brief Prints the Euler angles to the terminal 
         * \note Need to enable usb and or uart stdio for the pico
        */
        void print_euler_angles()
        {
            printf("roll: %f, pitch: %f, yaw: %f \n", roll, pitch, yaw);
        }
};

#endif