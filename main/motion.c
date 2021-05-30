#include "driver/timer.h"

#include "tft.h"
#include "motion.h"
#include "mpu9250.h"
#include "Fusion.h"

static FusionBias g_fusion_bias;
static FusionAhrs g_fusion_ahrs;
static FusionVector3 g_accel_sensitivity;
static FusionVector3 g_gyro_sensitivity;
static float g_fusion_sample_period = 0.0025f; // TODO not sure which value to use here, using accel sample rate now

// TODO need to calibrate mpu9250, current numbers are wrong
void motion_init()
{
    float accel_sensitivity;
    float gyro_sensitivity;

    mpu9250_init();

     /* Initialise gyroscope bias correction algorithm */
    FusionBiasInitialise(&g_fusion_bias, 0.5f, g_fusion_sample_period);

    /* Initialise AHRS algorithm */
    FusionAhrsInitialise(&g_fusion_ahrs, 0.5f);

    accel_sensitivity = mpu9250_get_accel_sensitivity();
    gyro_sensitivity = mpu9250_get_gyro_sensitivity();

    g_accel_sensitivity.axis.x = accel_sensitivity;
    g_accel_sensitivity.axis.y = accel_sensitivity;
    g_accel_sensitivity.axis.z = accel_sensitivity;

    g_gyro_sensitivity.axis.x = gyro_sensitivity;
    g_gyro_sensitivity.axis.y = gyro_sensitivity;
    g_gyro_sensitivity.axis.z = gyro_sensitivity;

    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = 16,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = false,
    }; /* default clock source is APB */

    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
}

void motion_shutdown()
{
    mpu9250_shutdown();
}

static void motion_get_euler_angles(float *roll, float *pitch, float *yaw)
{
    accel_t raw_accel;
    gyro_t raw_gyro;

    /* Calibrate gyroscope */
    mpu9250_get_raw_gyro_data(&raw_gyro);
    /* mpu9250_get_gyro_data(&raw_gyro); */
    FusionVector3 uncalibratedGyroscope = {
        .axis.x = raw_gyro.x,
        .axis.y = raw_gyro.y,
        .axis.z = raw_gyro.z,
    };
    FusionVector3 calibratedGyroscope = FusionCalibrationInertial(
            uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY,
            g_gyro_sensitivity, FUSION_VECTOR3_ZERO);

    /* Calibrate accelerometer */
    mpu9250_get_raw_accel_data(&raw_accel);
    /* mpu9250_get_accel_data(&raw_accel); */
    FusionVector3 uncalibratedAccelerometer = {
        .axis.x = raw_accel.x,
        .axis.y = raw_accel.y,
        .axis.z = raw_accel.z,
    };
    FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(
            uncalibratedAccelerometer,FUSION_ROTATION_MATRIX_IDENTITY,
            g_accel_sensitivity, FUSION_VECTOR3_ZERO);

    /* Update gyroscope bias correction algorithm */
    calibratedGyroscope = FusionBiasUpdate(&g_fusion_bias, calibratedGyroscope);

    /* Update AHRS algorithm */
    FusionAhrsUpdateWithoutMagnetometer(
            &g_fusion_ahrs, calibratedGyroscope, calibratedAccelerometer, g_fusion_sample_period);

    /* Print Euler angles */
    // TODO instead of euler angles, use rotation matrix instead?
    FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(
            FusionAhrsGetQuaternion(&g_fusion_ahrs));

    printf("Roll = %0.1f, Pitch = %0.1f, Yaw = %0.1f\r\n", eulerAngles.angle.roll, eulerAngles.angle.pitch, eulerAngles.angle.yaw);
    *roll = eulerAngles.angle.roll;
    *pitch = eulerAngles.angle.pitch;
    *yaw = eulerAngles.angle.yaw;
}

// TODO currently just hardcode horizontal motion, eventually remove DEBUG code when ready to use
// IMU readings for movement
//#define DEBUG
#ifdef DEBUG
display_coord_t motion_get_new_ball_pos(display_coord_t prev_pos)
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
#else
// TODO needs to take a pointer to maze layout (const) so it can handle bouncing off walls
// If new position of ball is going to hit wall, modify new position to right before wall
// Then reverse direction of velocity (make negative) and divide by some factor (e.g. 3).
// TODO really only need accelerometer value because nothing is being rotated? -- does this mean
// we can skip using the Fusion library? If so remove all Fusion dependencies.
void motion_update_ball_pos(ball_t *ball)
{
    accel_t accel;
    double elapsed_time_sec;
    static double time_delta_sec;
    static double last_update_time_sec;
    static bool first_update = true;

    if (!ball) {
        printf("%s: ball is NULL.\n", __func__);
        return;
    }

    if (first_update) {
        /* Start timer and skip first update */
        timer_start(TIMER_GROUP_0, TIMER_0);
        timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &last_update_time_sec);
        first_update = false;
        return;
    }

    /* Save previous position before modifying */
    ball->prev_pos = ball->pos;

    /* Collect latest accelerometer data and measure time delta between now and last update */
    mpu9250_get_accel_data(&accel);
    timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &elapsed_time_sec);
    time_delta_sec = elapsed_time_sec - last_update_time_sec;

    printf("accel: x=%0.1f, y=%0.1f, z=%0.1f\n", accel.x, accel.y, accel.z);
    printf("time delta: %ds\n", (int)time_delta_sec);

    /* Calculate velocity from acceleration, ignoring Z axis values */
    ball->velocity.x += accel.x * time_delta_sec;
    ball->velocity.y += accel.y * time_delta_sec;

    /* Clamp velocity to avoid ball moving too fast */
    // TODO figure out good velocity limits once things are working
    if (ball->velocity.x > 3 ) {
        ball->velocity.x = 3;
    } else if (ball->velocity.x < -3 ) {
        ball->velocity.x = -3;
    }

    if (ball->velocity.y > 3 ) {
        ball->velocity.y = 3;
    } else if (ball->velocity.y < -3 ) {
        ball->velocity.y = -3;
    }

    /* Calculate position from velocity, ignoring Z axis values */
    ball->pos.x += ball->velocity.x * time_delta_sec;
    ball->pos.y += ball->velocity.y * time_delta_sec;

    // TODO temporary bounds checking, replace with check for maze wall
    if (ball->pos.x > DEFAULT_TFT_DISPLAY_WIDTH - 20) {
        ball->pos.x = DEFAULT_TFT_DISPLAY_WIDTH - 20;
    } else if (ball->pos.x < 20) {
        ball->pos.x = 20;
    }

    if (ball->pos.y > DEFAULT_TFT_DISPLAY_HEIGHT - 20) {
        ball->pos.y = DEFAULT_TFT_DISPLAY_HEIGHT - 20;
    } else if (ball->pos.y < 20) {
        ball->pos.y = 20;
    }

    return;
}
#endif

