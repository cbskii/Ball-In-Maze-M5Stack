// DATASHEET: https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
// REGISTER MAP: https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf

// TODO the goal of this driver is to be able to return raw
//      accelerometer data - since we only care about roll and pitch,
//      we don't need to use the gyroscope (needed for yaw) or
//      the magnetometer. There are also more features available like
//      temperature sensing which will not be accounted for.
//      Useful reading: https://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf

// TODO use FIFO to store accelerometer data and read from that?

// TODO document all project stuff using doxygen syntax
// TODO add all defines etc. to header file specific to chip - mpu9250.[h|c]
// -- then use accelerometer.c as a general abstraction

#define REGISTER_RESET 0x00  /* reset value for almost all registers */
#define MPU9250_I2C_ADDR 0x68

/* register definitions */
#define FIFO_EN 0x23
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define ACCEL_INTEL_CTRL 0x69
#define USER_CTRL 0x6A
#define FIFO_COUNTH 0x72
#define FIFO_COUNTL 0x73
#define FIFO_R_W 0x74
#define WHO_AM_I 0x75
#define XA_OFFSET_H 0x77
#define XA_OFFSET_L 0x78
#define YA_OFFSET_H 0x7A
#define YA_OFFSET_L 0x7B
#define ZA_OFFSET_H 0x7D
#define ZA_OFFSET_L 0x7E

// struct accel_data {
//     int16_t x;
//     int16_t y;
//     int16_t z;
// };

// void accel_init(void) {
//     // TODO read WHOAMI and verify 0x71 or don't init driver and print error
//     // TODO wait 50ms for accelerometer data to be ready
//     // TODO calibrate accelerometer data here?

// }

// void accel_exit(void) {

// }

// esp_err_t get_raw_accel_data(struct accel_data* raw_accel_data) {
//     // TODO accel data is 16 bits, H << 8 | L
//     // TODO can use a data length of 6 bytes to read all
//     //      x, y and z at once - cast into int16_t
//     uint8_t raw_data;
//     esp_err_t err;
//     err = i2c_master_read()

// }
