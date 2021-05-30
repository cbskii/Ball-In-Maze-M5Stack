#include <stdio.h>
#include "freertos/FreeRTOS.h"

#include "buttons.h"
#include "display.h"
#include "mpu9250.h"
#include "Fusion.h" // TODO replace with motion.h

// TODO move into motion.c and rename/cleanup fusion stuff
FusionBias fusionBias;
FusionAhrs fusionAhrs;
FusionVector3 accelerometerSensitivity;
FusionVector3 gyroscopeSensitivity;
float samplePeriod = 0.0025f; // TODO not sure which value to use here, using accel sample rate now

// TODO test movement in horizontal line; eventually use acceleromter to move position
static display_coord_t get_new_ball_pos(display_coord_t prev_pos)
{
    display_coord_t new_pos;
    if (prev_pos.x + 5 > DEFAULT_TFT_DISPLAY_WIDTH - 20) {
        new_pos.x = 20;
    } else {
        new_pos.x = prev_pos.x + 5;
    }
    new_pos.y = prev_pos.y + 0;
    return new_pos;
}

static void motion_init()
{
    float accel_sensitivity;
    float gyro_sensitivity;

     // Initialise gyroscope bias correction algorithm
    FusionBiasInitialise(&fusionBias, 0.5f, samplePeriod); // stationary threshold = 0.5 degrees per second

    // Initialise AHRS algorithm
    FusionAhrsInitialise(&fusionAhrs, 0.5f); // gain = 0.5

    accel_sensitivity = mpu9250_get_accel_sensitivity();
    gyro_sensitivity = mpu9250_get_gyro_sensitivity();

    accelerometerSensitivity.axis.x = accel_sensitivity;
    accelerometerSensitivity.axis.y = accel_sensitivity;
    accelerometerSensitivity.axis.z = accel_sensitivity;

    gyroscopeSensitivity.axis.x = gyro_sensitivity;
    gyroscopeSensitivity.axis.y = gyro_sensitivity;
    gyroscopeSensitivity.axis.z = gyro_sensitivity;
}

static void motion_get_euler_angles()
{
    accel_t raw_accel;
    gyro_t raw_gyro;

    // Calibrate gyroscope
    /* mpu9250_get_raw_gyro_data(&raw_gyro); */
    mpu9250_get_gyro_data(&raw_gyro);
    FusionVector3 uncalibratedGyroscope = {
        .axis.x = raw_gyro.x,
        .axis.y = raw_gyro.y,
        .axis.z = raw_gyro.z,
    };
    FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

    // Calibrate accelerometer
    /* mpu9250_get_raw_accel_data(&raw_accel); */
    mpu9250_get_accel_data(&raw_accel);
    FusionVector3 uncalibratedAccelerometer = {
        .axis.x = raw_accel.x,
        .axis.y = raw_accel.y,
        .axis.z = raw_accel.z,
    };
    FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

    // Update gyroscope bias correction algorithm
    calibratedGyroscope = FusionBiasUpdate(&fusionBias, calibratedGyroscope);

    // Update AHRS algorithm
    FusionAhrsUpdateWithoutMagnetometer(&fusionAhrs, calibratedGyroscope, calibratedAccelerometer, samplePeriod);

    // Print Euler angles
    FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs));
    printf("Roll = %0.1f, Pitch = %0.1f, Yaw = %0.1f\r\n", eulerAngles.angle.roll, eulerAngles.angle.pitch, eulerAngles.angle.yaw);
}

void app_main(void)
{
    // TODO get working power on/off button
    // TODO do stuff with buttons
    // TODO generate different mazes
    // TODO turn off sound
    display_coord_t prev_ball_pos;
    display_coord_t new_ball_pos;

    buttons_init();
    mpu9250_init();
    motion_init();
    display_init();
    display_draw_maze(&prev_ball_pos);

    while (true) {
        motion_get_euler_angles();
        new_ball_pos = get_new_ball_pos(prev_ball_pos);
        display_move_ball(prev_ball_pos, new_ball_pos);
        prev_ball_pos = new_ball_pos;
        vTaskDelay(50 / portTICK_RATE_MS); // TODO adjust as necessary
    }

    mpu9250_shutdown();
    buttons_exit();
    display_shutdown();
}
