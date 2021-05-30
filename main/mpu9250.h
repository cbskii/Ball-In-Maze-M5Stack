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
#include "esp_err.h"

/*
 * Acceleration in x, y and z axes in units of g (e.g. when placed on a flat table x and y
 * will be 0g and z will be +1g).
 */
typedef struct {
	float x;
	float y;
	float z;
} accel_t;

/*
 * Gryoscope data indicates rotation around a specific axis represented in degrees/second.
 */
typedef struct {
	float x;
	float y;
	float z;
} gyro_t;

/**
 * @brief Initializes the mpu9250 for accelerometer and gyroscope usage along
 * with I2C necessary to communicate with the chip.
 */
esp_err_t mpu9250_init();

/**
 * @brief Teardown of mpu9250.
 */
esp_err_t mpu9250_shutdown();

/**
 * @brief Returns accelerometer sensitivity in g per lsb
 *
 * @return accelerometer sensitivity
 */
float mpu9250_get_accel_sensitivity();

/**
 * @brief Returns gyroscope sensitivity in degrees per second per lsb
 *
 * @return gyroscope sensitivity
 */
float mpu9250_get_gyro_sensitivity();

/**
 * @brief Returns most recent accelerometer data in units of lsb.
 *
 * @param[out] accel_data: raw accel data
 */
void mpu9250_get_raw_accel_data(accel_t *const accel_data);

/**
 * @brief Returns most recent gyroscope data in units of degrees per second.
 *
 * @param[out] gyro_data: raw gyro data
 */
void mpu9250_get_raw_gyro_data(gyro_t *const gyro_data);

/**
 * @brief Returns most recent accelerometer data in units of g.
 *
 * @param[out] accel_data: accel data
 */
void mpu9250_get_accel_data(accel_t *const accel_data);

/**
 * @brief Returns most recent gyroscope data in units of degrees per second.
 *
 * @param[out] gyro_data: gyro data
 */
void mpu9250_get_gyro_data(gyro_t *const gyro_data);

#endif /* MPU9250_H */
