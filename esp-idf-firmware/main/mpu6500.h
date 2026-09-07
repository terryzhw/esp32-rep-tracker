#ifndef MPU6500_H
#define MPU6500_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c.h"

#define MPU6500_I2C_ADDR 0x68

#define MPU6500_ACCEL_RANGE_2G  0x00
#define MPU6500_ACCEL_RANGE_4G  0x08
#define MPU6500_ACCEL_RANGE_8G  0x10
#define MPU6500_ACCEL_RANGE_16G 0x18

#define MPU6500_GYRO_RANGE_250DPS  0x00
#define MPU6500_GYRO_RANGE_500DPS  0x08
#define MPU6500_GYRO_RANGE_1000DPS 0x10
#define MPU6500_GYRO_RANGE_2000DPS 0x18

#define MPU6500_DLPF_184HZ 0x01
#define MPU6500_DLPF_92HZ  0x02
#define MPU6500_DLPF_41HZ  0x03
#define MPU6500_DLPF_20HZ  0x04
#define MPU6500_DLPF_10HZ  0x05
#define MPU6500_DLPF_5HZ   0x06

typedef struct {
    i2c_port_t i2c_port;
    float accel_scale;
    float gyro_scale;
    float accel[3];
    float gyro[3];

    float temp;
} mpu6500_t;

bool mpu6500_init(mpu6500_t *dev, i2c_port_t port);
bool mpu6500_set_accel_range(mpu6500_t *dev, uint8_t range);
bool mpu6500_set_gyro_range(mpu6500_t *dev, uint8_t range);
bool mpu6500_set_dlpf(mpu6500_t *dev, uint8_t dlpf);
bool mpu6500_set_srd(mpu6500_t *dev, uint8_t srd);
bool mpu6500_read(mpu6500_t *dev);

#endif
