#include "mpu9250.h"
#include "Fusion.h"

static FusionBias g_fusion_bias;
static FusionAhrs g_fusion_ahrs;
static FusionVector3 g_accel_sensitivity;
static FusionVector3 g_gyro_sensitivity;
static float g_fusion_sample_period = 0.0025f; // TODO not sure which value to use here, using accel sample rate now

void motion_init()
{
    float accel_sensitivity;
    float gyro_sensitivity;

    mpu9250_init();

     // Initialise gyroscope bias correction algorithm
    FusionBiasInitialise(&g_fusion_bias, 0.5f, g_fusion_sample_period); // stationary threshold = 0.5 degrees per second

    // Initialise AHRS algorithm
    FusionAhrsInitialise(&g_fusion_ahrs, 0.5f); // gain = 0.5

    accel_sensitivity = mpu9250_get_accel_sensitivity();
    gyro_sensitivity = mpu9250_get_gyro_sensitivity();

    g_accel_sensitivity.axis.x = accel_sensitivity;
    g_accel_sensitivity.axis.y = accel_sensitivity;
    g_accel_sensitivity.axis.z = accel_sensitivity;

    g_gyro_sensitivity.axis.x = gyro_sensitivity;
    g_gyro_sensitivity.axis.y = gyro_sensitivity;
    g_gyro_sensitivity.axis.z = gyro_sensitivity;
}

void motion_shutdown()
{
    mpu9250_shutdown();
}

void motion_get_euler_angles()
{
    accel_t raw_accel;
    gyro_t raw_gyro;

    // Calibrate gyroscope
    mpu9250_get_raw_gyro_data(&raw_gyro);
    /* mpu9250_get_gyro_data(&raw_gyro); */
    FusionVector3 uncalibratedGyroscope = {
        .axis.x = raw_gyro.x,
        .axis.y = raw_gyro.y,
        .axis.z = raw_gyro.z,
    };
    FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, g_gyro_sensitivity, FUSION_VECTOR3_ZERO);

    // Calibrate accelerometer
    mpu9250_get_raw_accel_data(&raw_accel);
    /* mpu9250_get_accel_data(&raw_accel); */
    FusionVector3 uncalibratedAccelerometer = {
        .axis.x = raw_accel.x,
        .axis.y = raw_accel.y,
        .axis.z = raw_accel.z,
    };
    FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, g_accel_sensitivity, FUSION_VECTOR3_ZERO);

    // Update gyroscope bias correction algorithm
    calibratedGyroscope = FusionBiasUpdate(&g_fusion_bias, calibratedGyroscope);

    // Update AHRS algorithm
    FusionAhrsUpdateWithoutMagnetometer(&g_fusion_ahrs, calibratedGyroscope, calibratedAccelerometer, g_fusion_sample_period);

    // Print Euler angles
    FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&g_fusion_ahrs));
    printf("Roll = %0.1f, Pitch = %0.1f, Yaw = %0.1f\r\n", eulerAngles.angle.roll, eulerAngles.angle.pitch, eulerAngles.angle.yaw);
}

