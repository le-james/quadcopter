#include "mpu6050.h"

void hello_world(uint8_t a)
{
    printf("hello world, from library %d\n", a);
}


// initialize the imu
void init_mpu6050(uint8_t MPU6050_ADDR, i2c_inst_t *i2c, uint8_t PIN_SDA, uint8_t PIN_SCL)
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


/* GYROSCOPE FUNCTIONS */
// set gyroscope range
void set_gyro_range(uint8_t MPU6050_ADDR, i2c_inst_t *i2c, uint8_t gyro_range)
{

    // write to default first (range 250)
    uint8_t default_range[] = {GYRO_CONFIG, 0x00};
    i2c_write_blocking(i2c, MPU6050_ADDR, default_range, 2, true);

    uint8_t new_range[] = {GYRO_CONFIG, gyro_range};
    i2c_write_blocking(i2c, MPU6050_ADDR, new_range, 2, false);
}

// read gyroscope range
uint8_t read_gyro_range(uint8_t MPU6050_ADDR, i2c_inst_t *i2c, bool isPrint)
{

    // range from register
    uint8_t gyro_range_out[1];

    // Start reading gyroscope registers from register 0x1B for 1 byte
    i2c_write_blocking(i2c, MPU6050_ADDR, &GYRO_CONFIG, 1, true); // true to keep master control of bus
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

// read gyroscope axis values
void get_gyro_rates(uint8_t MPU6050_ADDR, i2c_inst_t *i2c, \
                    int16_t *gx_raw, int16_t *gy_raw, int16_t *gz_raw, \
                    double *gx, double *gy, double *gz)
{

    // 16 bits each for x y z
    uint8_t gyro[6];

    // Start reading gyroscope registers from register 0x43 for 6 bytes
    uint8_t val = 0x43;
    i2c_write_blocking(i2c, MPU6050_ADDR, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c, MPU6050_ADDR, gyro, 6, false);

    // get the accel range being used (decimal)
    uint8_t curr_gyro_range = read_gyro_range(MPU6050_ADDR, i2c, 0);

    // declare
    double sclMod;

    // select the gyro scale modifier to use based on the range being used
    if (curr_gyro_range == 0)
    {
        sclMod = GYRO_SEN_SCL_FAC_250;
    }
    else if (curr_gyro_range == 8)
    {
        sclMod = GYRO_SEN_SCL_FAC_500;
    }
    else if (curr_gyro_range == 16)
    {
        sclMod = GYRO_SEN_SCL_FAC_1000;
    }
    else if (curr_gyro_range == 24)
    {
        sclMod = GYRO_SEN_SCL_FAC_2000;
    }
    else
    {
        printf("WARN: Using none pre-defined gyro range (decimal), %d\n", curr_gyro_range);
    }

    // raw gyro values
    *gx_raw = (int16_t)(gyro[0] << 8 | gyro[1]);
    *gy_raw = (int16_t)(gyro[2] << 8 | gyro[3]);
    *gz_raw = (int16_t)(gyro[4] << 8 | gyro[5]);

    // gyro values deg/s
    *gx = *gx_raw / sclMod + gx_corr;
    *gy = *gy_raw / sclMod + gy_corr;
    *gz = *gz_raw / sclMod + gz_corr;

    // need to convert to 2 bytes for each axis
    // printf("raw gyro values: ");
    // printf("Gx: %d ", gx_raw);
    // printf("Gy: %d ", gy_raw);
    // printf("Gz: %d\n", gz_raw);

    // printf("deg/s gyro values: ");
    // printf("Gx: %f ", gx);
    // printf("Gy: %f ", gy);
    // printf("Gz: %f\n", gz);
}

/* GYROSCOPE FUNCTIONS */



/* ACCELEROMETER FUNCTIONS */
// set accelerometer range
void set_accel_range(uint8_t MPU6050_ADDR, i2c_inst_t *i2c, uint8_t accel_range)
{

    // write to default first (range 2G)
    uint8_t default_range[] = {ACCEL_CONFIG, 0x00};
    i2c_write_blocking(i2c, MPU6050_ADDR, default_range, 2, true);

    uint8_t new_range[] = {ACCEL_CONFIG, accel_range};
    i2c_write_blocking(i2c, MPU6050_ADDR, new_range, 2, false);
}

// read accelerometer range
uint8_t read_accel_range(uint8_t MPU6050_ADDR, i2c_inst_t *i2c, bool isPrint)
{

    // range from register
    uint8_t accel_range_out[1];

    // Start reading acceleration registers from register 0x1C for 1 byte
    i2c_write_blocking(i2c, MPU6050_ADDR, &ACCEL_CONFIG, 1, true); // true to keep master control of bus
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

// read accelerometer axis values
void get_accel_data(uint8_t MPU6050_ADDR, i2c_inst_t *i2c, \
                    int16_t *ax_raw, int16_t *ay_raw, int16_t *az_raw, \
                    double *ax, double *ay, double *az)
{

    // 16 bits each for x y z
    uint8_t accel[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c, MPU6050_ADDR, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c, MPU6050_ADDR, accel, 6, false);

    // get the accel range being used (decimal)
    uint8_t curr_accel_range = read_accel_range(MPU6050_ADDR, i2c, 0);

    // declare
    int sclMod;

    // select the accel scale modifier to use based on the range being used
    if (curr_accel_range == 0)
    {
        sclMod = ACCEL_SEN_SCL_FAC_2G;
    }
    else if (curr_accel_range == 8)
    {
        sclMod = ACCEL_SEN_SCL_FAC_4G;
    }
    else if (curr_accel_range == 16)
    {
        sclMod = ACCEL_SEN_SCL_FAC_8G;
    }
    else if (curr_accel_range == 24)
    {
        sclMod = ACCEL_SEN_SCL_FAC_16G;
    }
    else
    {
        printf("WARN: Using none pre-defined accel range (decimal), %d\n", curr_accel_range);
    }

    // raw accel values
    *ax_raw = (int16_t)(accel[0] << 8 | accel[1]);
    *ay_raw = (int16_t)(accel[2] << 8 | accel[3]);
    *az_raw = (int16_t)(accel[4] << 8 | accel[5]);

    // accle values m/s^2
    *ax = *ax_raw * GRAVITY / sclMod + ax_corr;
    *ay = *ay_raw * GRAVITY / sclMod + ay_corr;
    *az = *az_raw * GRAVITY / sclMod + az_corr;

    // need to convert to 2 bytes for each axis
    // printf("raw accel values: ");
    // printf("Ax: %d ", (int16_t)(accel[0] << 8 | accel[1]));
    // printf("Ay: %d ", (int16_t)(accel[2] << 8 | accel[3]));
    // printf("Az: %d\n", (int16_t)(accel[4] << 8 | accel[5]));

    // printf("m/s^s accel values: ");
    // printf("Ax: %f ", (int16_t)(accel[0] << 8 | accel[1]) * (1.0 / sclMod) * GRAVITY);
    // printf("Ay: %f ", (int16_t)(accel[2] << 8 | accel[3]) * (1.0 / sclMod) * GRAVITY);
    // printf("Az: %f\n", (int16_t)(accel[4] << 8 | accel[5]) * (1.0 / sclMod) * GRAVITY);
}
/* ACCELEROMETER FUNCTIONS */
