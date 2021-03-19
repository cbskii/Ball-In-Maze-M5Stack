/*
 * Basic ESP32 driver for getting accelerometer and gyroscope data from the MPU9250 IMU. A lot of
 * MPU9250 functionality is not supported by this driver, only what's necessary for basic
 * roll/pitch usage. The MPU9250 has a FIFO buffer that can be used to store accel and gyro
 * data, but the preferred method here is to just poll from the application to get latest data
 * and update position as necessary.
 *
 * DATASHEET: https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
 * REGISTER MAP: https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf
 */

#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>

// TODO figure out what units are needed for Madgewick IMU algorithm
// Example: https://github.com/m5stack/M5Stack/blob/master/src/utility/MPU9250.cpp
// Other example: https://github.com/bolderflight/mpu9250/blob/main/README.md
// Useful: https://longnight975551865.wordpress.com/2018/02/11/how-to-read-data-from-mpu9250/
// IMU algorithms: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

#define MPU9250_I2C_ADDR (0x68)
#define MPU9250_EXPECTED_WHOAMI (0x71)

/* Gyro and Accelerometer registers */
#define SELF_TEST_X_GYRO (0x00)
#define SELF_TEST_Y_GYRO (0x01)
#define SELF_TEST_Z_GYRO (0x02)
#define SELF_TEST_X_ACCEL (0x0D)
#define SELF_TEST_Y_ACCEL (0x0E)
#define SELF_TEST_Z_ACCEL (0x0F)
#define XG_OFFSET_H (0x13)
#define XG_OFFSET_L (0x14)
#define YG_OFFSET_H (0x15)
#define YG_OFFSET_L (0x16)
#define ZG_OFFSET_H (0x17)
#define ZG_OFFSET_L (0x18)
#define SMPLRT_DIV (0x19)
#define CONFIG (0x1A)
#define GYRO_CONFIG (0x1B)
#define ACCEL_CONFIG (0x1C)
#define ACCEL_CONFIG_2 (0x1D)
#define LP_ACCEL_ODR (0x1E)
#define WOM_THR (0x1F)
#define FIFO_EN (0x23)
#define I2C_MST_CTRL (0x24)
#define I2C_SLV0_ADDR (0x25)
#define I2C_SLV0_REG (0x26)
#define I2C_SLV0_CTRL (0x27)
#define I2C_SLV1_ADDR (0x28)
#define I2C_SLV1_REG (0x29)
#define I2C_SLV1_CTRL (0x2A)
#define I2C_SLV2_ADDR (0x2B)
#define I2C_SLV2_REG (0x2C)
#define I2C_SLV2_CTRL (0x2D)
#define I2C_SLV3_ADDR (0x2E)
#define I2C_SLV3_REG (0x2F)
#define I2C_SLV3_CTRL (0x30)
#define I2C_SLV4_ADDR (0x31)
#define I2C_SLV4_REG (0x32)
#define I2C_SLV4_DO (0x33)
#define I2C_SLV4_CTRL (0x34)
#define I2C_SLV4_DI (0x35)
#define I2C_MST_STATUS (0x36)
#define INT_PIN_CFG (0x37)
#define INT_ENABLE (0x38)
#define INT_STATUS (0x3A)
#define ACCEL_XOUT_H (0x3B)
#define ACCEL_XOUT_L (0x3C)
#define ACCEL_YOUT_H (0x3D)
#define ACCEL_YOUT_L (0x3E)
#define ACCEL_ZOUT_H (0x3F)
#define ACCEL_ZOUT_L (0x40)
#define TEMP_OUT_H (0x41)
#define TEMP_OUT_L (0x42)
#define GYRO_XOUT_H (0x43)
#define GYRO_XOUT_L (0x44)
#define GYRO_YOUT_H (0x45)
#define GYRO_YOUT_L (0x46)
#define GYRO_ZOUT_H (0x47)
#define GYRO_ZOUT_L (0x48)
#define EXT_SENS_DATA_00 (0x49)
#define EXT_SENS_DATA_01 (0x4A)
#define EXT_SENS_DATA_02 (0x4B)
#define EXT_SENS_DATA_03 (0x4C)
#define EXT_SENS_DATA_04 (0x4D)
#define EXT_SENS_DATA_05 (0x4E)
#define EXT_SENS_DATA_06 (0x4F)
#define EXT_SENS_DATA_07 (0x50)
#define EXT_SENS_DATA_08 (0x51)
#define EXT_SENS_DATA_09 (0x52)
#define EXT_SENS_DATA_10 (0x53)
#define EXT_SENS_DATA_11 (0x54)
#define EXT_SENS_DATA_12 (0x55)
#define EXT_SENS_DATA_13 (0x56)
#define EXT_SENS_DATA_14 (0x57)
#define EXT_SENS_DATA_15 (0x58)
#define EXT_SENS_DATA_16 (0x59)
#define EXT_SENS_DATA_17 (0x5A)
#define EXT_SENS_DATA_18 (0x5B)
#define EXT_SENS_DATA_19 (0x5C)
#define EXT_SENS_DATA_20 (0x5D)
#define EXT_SENS_DATA_21 (0x5E)
#define EXT_SENS_DATA_22 (0x5F)
#define EXT_SENS_DATA_23 (0x60)
#define I2C_SLV0_DO (0x63)
#define I2C_SLV1_DO (0x64)
#define I2C_SLV2_DO (0x65)
#define I2C_SLV3_DO (0x66)
#define I2C_MST_DELAY_CTRL (0x67)
#define SIGNAL_PATH_RESET (0x68)
#define MOT_DETECT_CTRL (0x69)
#define USER_CTRL (0x6A)
#define PWR_MGMT_1 (0x6B)
#define PWR_MGMT_2 (0x6C)
#define FIFO_COUNTH (0x72)
#define FIFO_COUNTL (0x73)
#define FIFO_R_W (0x74)
#define WHO_AM_I (0x75)
#define XA_OFFSET_H (0x77)
#define XA_OFFSET_L (0x78)
#define YA_OFFSET_H (0x7A)
#define YA_OFFSET_L (0x7B)
#define ZA_OFFSET_H (0x7D)
#define ZA_OFFSET_L (0x7E)

/*
 * Acceleration in x, y and z axes in units of g (e.g. when placed on a flat table x and y
 * will be 0g and z will be +1g).
 */
struct accel {
	float x;
	float y;
	float z;
};

/*
 * Gryoscope data indicates rotation around a specific axis represented in degrees/second.
 */
struct gyro {
	float x;
	float y;
	float z;
};

// esp_err_t get_raw_accel_data(struct accel_data* raw_accel_data) {
//     // TODO accel data is 16 bits, H << 8 | L
//     // TODO can use a data length of 6 bytes to read all
//     //      x, y and z at once - cast into int16_t
//     uint8_t raw_data;
//     esp_err_t err;
//     err = i2c_master_read()

// TODO docstrings
esp_err_t mpu9250_init();
esp_err_t mpu9250_deinit();
void mpu9250_get_accel_data(struct accel *const accel_data);
void mpu9250_get_gyro_data(struct gyro *const gyro_data);

#endif /* MPU9250_H */
