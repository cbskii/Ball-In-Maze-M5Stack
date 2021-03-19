#include "mpu9250.h"

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
static uint16_t g_accel_res_num_per_g;
static uint16_t g_gyro_res_num_per_dps;

static void calculate_accel_resolution(uint8_t accel_fs)
{
    switch(accel_fs) {
    case ACCEL_FS_2G:
        g_accel_res = UINT16_MAX / (2 * 2);
        break;
    case ACCEL_FS_4G:
        g_accel_res = UINT16_MAX / (2 * 4);
        break;
    case ACCEL_FS_8G:
        g_accel_res = UINT16_MAX / (2 * 8);
        break;
    case ACCEL_FS_16G:
        g_accel_res = UINT16_MAX / (2 * 16);
        break;
    }
}

static void calculate_gyro_resolution(uint8_t gyro_fs)
{
    switch(accel_fs) {
    case GYRO_FS_250DPS:
        g_gyro_res = UINT16_MAX / (2 * 250);
        break;
    case GYRO_FS_500DPS:
        g_gyro_res = UINT16_MAX / (2 * 500);
        break;
    case GYRO_FS_1000DPS:
        g_gyro_res = UINT16_MAX / (2 * 1000);
        break;
    case GYRO_FS_2000DPS:
        g_gyro_res = UINT16_MAX / (2 * 2000);
        break;
    }
}

/* Configure device as I2C master */
// TODO SDA=21, SCL=22? Need to configure somewhere?
static esp_err_t mpu9250_init_i2c()
{
    esp_err_t err;

    static i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    err = i2c_param_config(g_i2c_master_port, &conf);
    if (err != ESP_OK) {
        printf("Error configuring i2c params: %d", err);
        return err;
    }

    err = i2c_driver_install(g_i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE, 0);
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

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9250_I2C_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, addr, ACK_CHECK_EN);
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

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9250_I2C_ADDR, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, addr, ACK_CHECK_EN);
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

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9250_I2C_ADDR, ACK_CHECK_EN);
    i2c_master_read(cmd, addr, num_bytes, ACK_CHECK_EN);
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

    // TODO rest of init
    // TODO try out different accel and gryo sensitivity configurations, start with 4g and 500dps
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
        accel_data->x = ((raw_data[0] << 8) | raw_data[1]) / g_gyro_res_num_per_dps;
        accel_data->y = ((raw_data[2] << 8) | raw_data[3]) / g_gyro_res_num_per_dps;
        accel_data->z = ((raw_data[4] << 8) | raw_data[5]) / g_gyro_res_num_per_dps;
    }
}
