#ifndef MPU6050_LIB_HPP
#define MPU6050_LIB_HPP

#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"

/** \file mpu6050_lib.hpp
 * Simply reads the IMU's accelerometer and gyroscope output.
 */
class MPU6050
{
    protected:
        // earth gravity
        const double GRAVITY = 9.80665;

        // imu parameters
        i2c_inst_t *i2c;
        const uint8_t MPU6050_ADDR;

        // data line pins
        const uint8_t PIN_SDA;
        const uint8_t PIN_SCL;


        /* gyroscope parameters */

        int16_t gx_raw, gy_raw, gz_raw;
        double gx, gy, gz;

        // gyro calibration offsets 
        double gx_off, gy_off, gz_off;

        // gyro config register - used to change the gyro range
        const uint8_t GYRO_CONFIG = 0x1B;


        // store gyro data - 16 bits each for x y z       
        uint8_t gyro[6];

        // gyro sensitiviy scale
        double gyro_scl;

        // Start reading gyroscope registers from register 0x43 for 6 bytes
        uint8_t gyro_reg = 0x43;

    public:
        // gyro ranges
        const uint8_t GYRO_RANGE_250 = 0x00;
        const uint8_t GYRO_RANGE_500 = 0x08;
        const uint8_t GYRO_RANGE_1000 = 0x10;
        const uint8_t GYRO_RANGE_2000 = 0x18;

        // gyroscope sensitivity scale modifiers
        const double GYRO_SEN_SCL_FAC_250 = 131.0;
        const double GYRO_SEN_SCL_FAC_500 = 65.5;
        const double GYRO_SEN_SCL_FAC_1000 = 32.8;
        const double GYRO_SEN_SCL_FAC_2000 = 16.4;

        /* gyroscope parameters */


        /* accelerometer parameters */

        int16_t ax_raw, ay_raw, az_raw;
        double ax, ay, az;

        // accel config register - used to change the accel range
        const uint8_t ACCEL_CONFIG = 0x1C;


        // store accel data - 16 bits each for x y z
        uint8_t accel[6];

        // accel sensitiviy scale
        int accel_scl;

        // start reading acceleration registers from register 0x3B for 6 bytes
        uint8_t accel_reg = 0x3B;

    public:
        // accel ranges
        const uint8_t ACCEL_RANGE_2G = 0x00;
        const uint8_t ACCEL_RANGE_4G = 0x08;
        const uint8_t ACCEL_RANGE_8G = 0x10;
        const uint8_t ACCEL_RANGE_16G = 0x18;

        // accelerometer sensitivity scale modifiers
        const int ACCEL_SEN_SCL_FAC_2G = 16384;
        const int ACCEL_SEN_SCL_FAC_4G = 8192;
        const int ACCEL_SEN_SCL_FAC_8G = 4096;
        const int ACCEL_SEN_SCL_FAC_16G = 2048;

        /* accelerometer parameters */

    public:

        MPU6050(i2c_inst_t *i2c, const uint8_t MPU6050_ADDR, const uint8_t PIN_SDA, const uint8_t PIN_SCL)
            : i2c(i2c), MPU6050_ADDR(MPU6050_ADDR), PIN_SDA(PIN_SDA), PIN_SCL(PIN_SCL)
        {
            // initialize i2c hw block to run at 400kHz
            i2c_init(i2c, 400 * 1000);

            // set gpio to use i2c function
            gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
            gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);

            // pull i2c lines high
            gpio_pull_up(PIN_SDA);
            gpio_pull_up(PIN_SCL);

            // two byte reset. first byte register, second byte data
            // buf[] = { address, bits sent (wake up mpu6050 becuz it starts in sleep mode) }
            uint8_t buf[] = {0x6B, 0x00};
            i2c_write_blocking(i2c, MPU6050_ADDR, buf, 2, false);
        }

        // // initialize the imu - setup gpio - ALT FUNCTION
        // void init(i2c_inst_t *i2c, uint8_t MPU6050_ADDR, uint8_t PIN_SDA, uint8_t PIN_SCL)
        // {
        //     // initialize i2c hw block to run at 400kHz
        //     i2c_init(i2c, 400 * 1000);

        //     // set gpio to use i2c function
        //     gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
        //     gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);

        //     // pull i2c lines high
        //     gpio_pull_up(PIN_SDA);
        //     gpio_pull_up(PIN_SCL);

        //     // two byte reset. first byte register, second byte data
        //     // buf[] = { address, bits sent (wake up mpu6050 becuz it starts in sleep mode) }
        //     uint8_t buf[] = {0x6B, 0x00};
        //     i2c_write_blocking(i2c, MPU6050_ADDR, buf, 2, false);
        // }

        /* GYROSCOPE FUNCTIONS */

  
        /** \brief Set gyroscope range
         * \param gyro_range IMU's gyroscope ranges from the datasheet
         */
        void set_gyro_range(uint8_t gyro_range)
        {
            // write to register the default range first (range 250)
            uint8_t default_range[] = {GYRO_CONFIG, GYRO_RANGE_250};

            // true to keep master control of bus
            i2c_write_blocking(i2c, MPU6050_ADDR, default_range, 2, true);

            uint8_t new_range[] = {GYRO_CONFIG, gyro_range};
            i2c_write_blocking(i2c, MPU6050_ADDR, new_range, 2, false);

            // select the gyro scale modifier to use based on the range being used
            if (gyro_range == 0)
            {
                gyro_scl = GYRO_SEN_SCL_FAC_250;
            }
            else if (gyro_range == 8)
            {
                gyro_scl = GYRO_SEN_SCL_FAC_500;
            }
            else if (gyro_range == 16)
            {
                gyro_scl = GYRO_SEN_SCL_FAC_1000;
            }
            else if (gyro_range == 24)
            {
                gyro_scl = GYRO_SEN_SCL_FAC_2000;
            }
            else
            {
                printf("WARN: Using none pre-defined gyro range (decimal) of: %d\n", gyro_scl);
            }
        }

        /** \brief Read gyroscope range
         * \param isPrint Enables printing range to the terminal if true
         * \returns the gyro range
         */
        uint8_t read_gyro_range(bool isPrint)
        {
            // range from register
            uint8_t gyro_range_out[1];

            // Start reading gyroscope registers from register 0x1B for 1 byte
            i2c_write_blocking(i2c, MPU6050_ADDR, &GYRO_CONFIG, 1, true);
            i2c_read_blocking(i2c, MPU6050_ADDR, gyro_range_out, 1, false);

            // output gyro range that is being used if flag isPrint is true
            if (isPrint)
            {
                if (gyro_range_out[0] == 0)
                {
                    printf("Current gyro range: 0x00\n");
                }
                else if (gyro_range_out[0] == 8)
                {
                    printf("Current gyro range: 0x08\n");
                }
                else if (gyro_range_out[0] == 16)
                {
                    printf("Current gyro range: 0x10\n");
                }
                else if (gyro_range_out[0] == 24)
                {
                    printf("Current gyro range: 0x18\n");
                }
                else
                {
                    printf("WARN: Using none pre-defined gyro range (decimal), %d\n", gyro_range_out[0]);
                }
            }
            return gyro_range_out[0];
        }

        /** \brief Read gyroscope axis values
         */
        // void get_gyro_rates(i2c_inst_t *i2c, uint8_t MPU6050_ADDR, \
        //                     int16_t *gx_raw, int16_t *gy_raw, int16_t *gz_raw, \
        //                     double *gx, double *gy, double *gz)
        void get_gyro_data()
        {
            i2c_write_blocking(i2c, MPU6050_ADDR, &gyro_reg, 1, true);
            i2c_read_blocking(i2c, MPU6050_ADDR, gyro, 6, false);

            // raw gyro values
            gx_raw = (int16_t)(gyro[0] << 8 | gyro[1]);
            gy_raw = (int16_t)(gyro[2] << 8 | gyro[3]);
            gz_raw = (int16_t)(gyro[4] << 8 | gyro[5]);

            // gyro values deg/s
            // gx = gx_raw / gyro_scl;
            // gy = gy_raw / gyro_scl;
            // gz = gz_raw / gyro_scl;

            // calibrated gyro values [deg/s]
            gx = gx_raw / gyro_scl - gx_off;
            gy = gy_raw / gyro_scl - gy_off;
            gz = gz_raw / gyro_scl - gz_off;
        }

        /** \brief Calibrate the gyroscope
         * \param num_samp Number of samples gyro values to sample and average
         */
        void cali_gyro(int num_samp)
        {
            // calibrate gyro
            double gx_cal = 0;
            double gy_cal = 0;
            double gz_cal = 0;

            for (int i = 0; i < num_samp; i++){
                get_gyro_data();
                gx_cal += gx_raw / gyro_scl;
                gy_cal += gy_raw / gyro_scl;
                gz_cal += gz_raw / gyro_scl;
                sleep_ms(1);
            }

            // gyro offset in deg/s
            gx_off = gx_cal / num_samp;
            gy_off = gy_cal / num_samp;
            gz_off = gz_cal / num_samp;
        }

        /** \brief Print gyro calibration offset values
         */
        void print_gyro_cali_offset()
        {
            printf("gx_off: %f, gy_off: %f, gz_off: %f \n", gx_off, gy_off, gz_off);
        }

        /** \brief Print the raw gyro values
         */
        void print_gyro_raw()
        {
            printf("gx_raw: %f, gy_raw: %f, gz_raw: %f \n", gx_raw, gy_raw, gz_raw);
        }

        /** \brief Print gyro values [deg/s]
         */
        void print_gyro()
        {
            printf("gx: %f, gy: %f, gz: %f \n", gx, gy, gz);
        }

        /* GYROSCOPE FUNCTIONS */


        /* ACCELEROMETER FUNCTIONS */

        /** \brief Set accelerometer range
         * \param accel_range IMU's acceleration ranges from the datasheet
         */
        void set_accel_range(uint8_t accel_range)
        {
            // write to register the default range first (range 2G)
            uint8_t default_range[] = {ACCEL_CONFIG, ACCEL_RANGE_2G};
            i2c_write_blocking(i2c, MPU6050_ADDR, default_range, 2, true);

            uint8_t new_range[] = {ACCEL_CONFIG, accel_range};
            i2c_write_blocking(i2c, MPU6050_ADDR, new_range, 2, false);

            // select the accel scale modifier to use based on the range being used
            if (accel_range == 0)
            {
                accel_scl = ACCEL_SEN_SCL_FAC_2G;
            }
            else if (accel_range == 8)
            {
                accel_scl = ACCEL_SEN_SCL_FAC_4G;
            }
            else if (accel_range == 16)
            {
                accel_scl = ACCEL_SEN_SCL_FAC_8G;
            }
            else if (accel_range == 24)
            {
                accel_scl = ACCEL_SEN_SCL_FAC_16G;
            }
            else
            {
                printf("WARN: Using none pre-defined accel range (decimal), %d\n", accel_range);
            }
        }

        /** \brief Read the accelerometer values
         * \param isPrint Enables printing range to the terminal if true
         * \returns the acceleration range
         */
        uint8_t read_accel_range(bool isPrint)
        {
            // range from register
            uint8_t accel_range_out[1];

            // Start reading acceleration registers from register 0x1C for 1 byte
            i2c_write_blocking(i2c, MPU6050_ADDR, &ACCEL_CONFIG, 1, true);
            i2c_read_blocking(i2c, MPU6050_ADDR, accel_range_out, 1, false);

            // output accel range that is being used if flag isPrint is true
            if (isPrint)
            {
                if (accel_range_out[0] == 0)
                {
                    printf("Current accel range: 0x00\n");
                }
                else if (accel_range_out[0] == 8)
                {
                    printf("Current accel range: 0x08\n");
                }
                else if (accel_range_out[0] == 16)
                {
                    printf("Current accel range: 0x10\n");
                }
                else if (accel_range_out[0] == 24)
                {
                    printf("Current accel range: 0x18\n");
                }
                else
                {
                    printf("WARN: Using none pre-defined accel range (decimal), %d\n", accel_range_out[0]);
                }
            }
            return accel_range_out[0];
        }

        /** \brief Get the accelerometer values
         */
        // void get_accel_data(i2c_inst_t *i2c, uint8_t MPU6050_ADDR, \
        //                     int16_t *ax_raw, int16_t *ay_raw, int16_t *az_raw, \
        //                     double *ax, double *ay, double *az)
        void get_accel_data()
        {
            i2c_write_blocking(i2c, MPU6050_ADDR, &accel_reg, 1, true); // true to keep master control of bus
            i2c_read_blocking(i2c, MPU6050_ADDR, accel, 6, false);

            // raw accel values
            ax_raw = (int16_t)(accel[0] << 8 | accel[1]);
            ay_raw = (int16_t)(accel[2] << 8 | accel[3]);
            az_raw = (int16_t)(accel[4] << 8 | accel[5]);

            // accle values m/s^2
            ax = ax_raw * GRAVITY / accel_scl;
            ay = ay_raw * GRAVITY / accel_scl;
            az = az_raw * GRAVITY / accel_scl;
        }

        /** \brief Print the gyroscope values [deg/s]
         */
        void print_accel_raw()
        {
            printf("ax_raw: %f, ay_raw: %f, az_raw: %f \n", ax_raw, ay_raw, az_raw);
        }

        /** \brief Print the accerlation values [m/s^2]
         */
        void print_accel()
        {
            printf("ax: %f, ay: %f, az: %f \n", ax, ay, az);
        }

        /* ACCELEROMETER FUNCTIONS */


        /** \brief Sample the imu for accel and gyro data every dt interval
         * \param dt Time between samples. 1/dt is the freqency of the sample rate.
         */
        void sample_imu(double dt)
        {
            // output in seconds
            absolute_time_t start = get_absolute_time();

            get_accel_data();
            print_accel();

            get_gyro_data();
            print_gyro();

            while( absolute_time_diff_us(start,get_absolute_time())/1000000.0 < dt );
        }
};

#endif