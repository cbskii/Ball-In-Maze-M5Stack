#include "driver/i2c.h"
#include "hal/i2c_types.h"
#include "mpu9250.h"

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

/* M5stack I2C settings */
#define I2C_MASTER_SDA_GPIO (21)
#define I2C_MASTER_SCL_GPIO (22)
#define I2C_MASTER_CLK_SPEED (100000)

/* Acccelerometer full scale settings */
#define ACCEL_FS_2G (0x00) /* +/- 2g */
#define ACCEL_FS_4G (0x01) /* +/- 4g */
#define ACCEL_FS_8G (0x10) /* +/- 8g */
#define ACCEL_FS_16G (0x11) /* +/- 16g */

/* Gyro full scale settings */
#define GYRO_FS_250DPS (0x00) /* +/- 250dps */
#define GYRO_FS_500DPS (0x01) /* +/- 500dps */
#define GYRO_FS_1000DPS (0x10) /* +/- 1000dps */
#define GYRO_FS_2000DPS (0x11) /* +/- 2000dps */

static i2c_port_t g_i2c_master_port = 0;

/*
 * Sensor resolution based on configured full scale range.
 * Raw sensor data is provided as a signed 16-bit value representing data within the sensor's
 * configured range. Example: configuring GYRO_FS_250DPS will cause the gyroscope to return a
 * value between -32768 and 32767 representing -250dps to 250dps. This means resolution is (2 *
 * FS value) / 65535 for dps/num or 65535 / (2 * FS value) for num/dps which makes integer math
 * possible for quick and less accurate calculations. This is where the 131 LSB/dps value in the
 * datasheet comes from when selecting GYRO_FS_250DPS (FS_SEL = 0).
 */
static float g_accel_res_num_per_g;
static float g_gyro_res_num_per_dps;

static void calculate_accel_resolution(uint8_t accel_fs)
{
    switch(accel_fs) {
    case ACCEL_FS_2G:
        g_accel_res_num_per_g = UINT16_MAX / (float)(2 * 2);
        break;
    case ACCEL_FS_4G:
        g_accel_res_num_per_g = UINT16_MAX / (float)(2 * 4);
        break;
    case ACCEL_FS_8G:
        g_accel_res_num_per_g = UINT16_MAX / (float)(2 * 8);
        break;
    case ACCEL_FS_16G:
        g_accel_res_num_per_g = UINT16_MAX / (float)(2 * 16);
        break;
    }
}

static void calculate_gyro_resolution(uint8_t gyro_fs)
{
    switch(gyro_fs) {
    case GYRO_FS_250DPS:
        g_gyro_res_num_per_dps = UINT16_MAX / (float)(2 * 250);
        break;
    case GYRO_FS_500DPS:
        g_gyro_res_num_per_dps = UINT16_MAX / (float)(2 * 500);
        break;
    case GYRO_FS_1000DPS:
        g_gyro_res_num_per_dps = UINT16_MAX / (float)(2 * 1000);
        break;
    case GYRO_FS_2000DPS:
        g_gyro_res_num_per_dps = UINT16_MAX / (float)(2 * 2000);
        break;
    }
}

/* Configure device as I2C master */
static esp_err_t mpu9250_init_i2c()
{
    esp_err_t err;

    static i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_CLK_SPEED,
    };

    err = i2c_param_config(g_i2c_master_port, &conf);
    if (err != ESP_OK) {
        printf("Error configuring i2c params: %d", err);
        return err;
    }

    err = i2c_driver_install(g_i2c_master_port, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        printf("Error installing i2c: %d", err);
        return err;
    }

    return ESP_OK;
}

static esp_err_t mpu9250_deinit_i2c()
{
    return i2c_driver_delete(g_i2c_master_port);
}

static esp_err_t mpu9250_write_byte(uint8_t addr, uint8_t data)
{
    esp_err_t err;

    /* S -> AD+W -> RA -> DATA -> P */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(g_i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    if (err) {
        printf("Error writing byte 0x%X to address 0x%X: %d", data, addr, err);
    }

    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t mpu9250_read_byte(uint8_t addr, uint8_t *data)
{
    esp_err_t err;

    /* S -> AD+W -> RA -> S -> AD+R -> READ DATA -> NACK -> P */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, false);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(g_i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    if (err) {
        printf("Error reading byte from address 0x%X: %d", addr, err);
    }

    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t mpu9250_read_bytes(uint8_t addr, uint8_t *data, uint8_t num_bytes)
{
    esp_err_t err;

    /* S -> AD+W -> RA -> S -> AD+R -> RDATA (n-1) -> ACK -> RDATA (1) -> NACK P */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, num_bytes - 1, true);
    i2c_master_read_byte(cmd, data + (num_bytes - 1), false);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(g_i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    if (err) {
        printf("Error reading %d bytes starting from address 0x%X: %d", num_bytes, addr, err);
    }

    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t mpu9250_init()
{
    /* Initialize I2C first */
    mpu9250_init_i2c();

    /* Initialize MPU9250 power by resetting all registers and waking the device up */
    mpu9250_write_byte(PWR_MGMT_1, 0x00);
    vTaskDelay(100 / portTICK_RATE_MS);

    /* Verify WHO_AM_I */
    uint8_t who_am_i;
    mpu9250_read_byte(WHO_AM_I, &who_am_i);
    if (who_am_i != MPU9250_EXPECTED_WHOAMI) {
        printf("MPU9250 WHOAMI byte (%02X) does not match expected value (%02X)!",
                who_am_i, MPU9250_EXPECTED_WHOAMI);
        return ESP_FAIL;
    }

    /* Set clock source based on a Gyro axis if available, else default to internal oscillator */
    mpu9250_write_byte(PWR_MGMT_1, 0x01);
    vTaskDelay(200 / portTICK_RATE_MS);

    /* Configure gyroscope range/sensitivity and enable low pass filter */
    uint8_t gyro_fs = GYRO_FS_500DPS;
    uint8_t gyro_dlpf_cfg = 0x04; /* 20Hz, 9.9ms delay */
    mpu9250_write_byte(GYRO_CONFIG, gyro_fs << 3);
    mpu9250_write_byte(CONFIG, gyro_dlpf_cfg);
    calculate_gyro_resolution(gyro_fs);

    /* Configure accelerometer range/sensitivity and enable low pass filter */
    uint8_t accel_fs = ACCEL_FS_4G;
    uint8_t accel_dlpf_cfg = 0x03; /* 41Hz, 11.80ms delay */
    mpu9250_write_byte(ACCEL_CONFIG, accel_fs << 3);
    mpu9250_write_byte(ACCEL_CONFIG_2, accel_dlpf_cfg);
    calculate_accel_resolution(accel_fs);
}

void mpu9250_calibrate()
{
    // TODO see the following for reference:
    // https://github.com/m5stack/M5Stack/blob/master/src/utility/MPU9250.cpp
    // https://github.com/kriswiner/MPU9250/issues/306
    return;
}

bool mpu9250_self_test()
{
    // TODO see the following for reference:
    // https://github.com/m5stack/M5Stack/blob/master/src/utility/MPU9250.cpp
    return true;
}

esp_err_t mpu9250_deinit()
{
    mpu9250_deinit_i2c();
}

void mpu9250_get_accel_data(struct accel *const accel_data)
{
    esp_err_t err;
    uint8_t raw_data[6];

    /* XYZ high and low bytes are stored at consecutive addresses so read all at once */
    err = mpu9250_read_bytes(ACCEL_XOUT_H, raw_data, sizeof(raw_data));
    if ((err == ESP_OK) && accel_data) {
        accel_data->x = ((raw_data[0] << 8) | raw_data[1]) / g_accel_res_num_per_g;
        accel_data->y = ((raw_data[2] << 8) | raw_data[3]) / g_accel_res_num_per_g;
        accel_data->z = ((raw_data[4] << 8) | raw_data[5]) / g_accel_res_num_per_g;
    }
}

void mpu9250_get_gyro_data(struct gyro *const gyro_data)
{
    esp_err_t err;
    uint8_t raw_data[6];

    /* XYZ high and low bytes are stored at consecutive addresses so read all at once */
    err = mpu9250_read_bytes(GYRO_XOUT_H, raw_data, sizeof(raw_data));
    if ((err == ESP_OK) && gyro_data) {
        gyro_data->x = ((raw_data[0] << 8) | raw_data[1]) / g_gyro_res_num_per_dps;
        gyro_data->y = ((raw_data[2] << 8) | raw_data[3]) / g_gyro_res_num_per_dps;
        gyro_data->z = ((raw_data[4] << 8) | raw_data[5]) / g_gyro_res_num_per_dps;
    }
}
