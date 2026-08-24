#include "mpu6500.h"
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define G_MPS2      9.80665f
#define DEG2RAD     (M_PI / 180.0f)

#define REG_PWR_MGMT_1   0x6B
#define REG_WHOAMI        0x75
#define REG_ACCEL_CONFIG  0x1C
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG2 0x1D
#define REG_CONFIG        0x1A
#define REG_SMPLRT_DIV    0x19
#define REG_INT_STATUS    0x3A

#define CLKSEL_PLL        0x01
#define WHOAMI_MPU6500    0x70
#define RAW_DATA_RDY_INT  0x01

#define I2C_TIMEOUT_MS    100

static bool write_reg(mpu6500_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(dev->i2c_port, MPU6500_I2C_ADDR,
                                      buf, 2,
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == ESP_OK;
}

static bool read_regs(mpu6500_t *dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(dev->i2c_port, MPU6500_I2C_ADDR,
                                        &reg, 1, data, len,
                                        pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == ESP_OK;
}

bool mpu6500_init(mpu6500_t *dev, i2c_port_t port)
{
    dev->i2c_port = port;

    // PLL is more stable than internal oscillator
    if (!write_reg(dev, REG_PWR_MGMT_1, CLKSEL_PLL))
        return false;

    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t who;
    if (!read_regs(dev, REG_WHOAMI, &who, 1))
        return false;
    if (who != WHOAMI_MPU6500)
        return false;

    // safe defaults — caller overrides these after init
    if (!mpu6500_set_accel_range(dev, MPU6500_ACCEL_RANGE_16G))
        return false;
    if (!mpu6500_set_gyro_range(dev, MPU6500_GYRO_RANGE_2000DPS))
        return false;
    if (!mpu6500_set_dlpf(dev, MPU6500_DLPF_184HZ))
        return false;
    if (!mpu6500_set_srd(dev, 0))
        return false;

    return true;
}

bool mpu6500_set_accel_range(mpu6500_t *dev, uint8_t range)
{
    switch (range) {
        case MPU6500_ACCEL_RANGE_2G:  dev->accel_scale = 2.0f  / 32767.5f; break;
        case MPU6500_ACCEL_RANGE_4G:  dev->accel_scale = 4.0f  / 32767.5f; break;
        case MPU6500_ACCEL_RANGE_8G:  dev->accel_scale = 8.0f  / 32767.5f; break;
        case MPU6500_ACCEL_RANGE_16G: dev->accel_scale = 16.0f / 32767.5f; break;
        default: return false;
    }
    return write_reg(dev, REG_ACCEL_CONFIG, range);
}

bool mpu6500_set_gyro_range(mpu6500_t *dev, uint8_t range)
{
    switch (range) {
        case MPU6500_GYRO_RANGE_250DPS:  dev->gyro_scale = 250.0f  / 32767.5f; break;
        case MPU6500_GYRO_RANGE_500DPS:  dev->gyro_scale = 500.0f  / 32767.5f; break;
        case MPU6500_GYRO_RANGE_1000DPS: dev->gyro_scale = 1000.0f / 32767.5f; break;
        case MPU6500_GYRO_RANGE_2000DPS: dev->gyro_scale = 2000.0f / 32767.5f; break;
        default: return false;
    }
    return write_reg(dev, REG_GYRO_CONFIG, range);
}

bool mpu6500_set_dlpf(mpu6500_t *dev, uint8_t dlpf)
{
    if (!write_reg(dev, REG_ACCEL_CONFIG2, dlpf))
        return false;
    return write_reg(dev, REG_CONFIG, dlpf);
}

bool mpu6500_set_srd(mpu6500_t *dev, uint8_t srd)
{
    return write_reg(dev, REG_SMPLRT_DIV, srd);
}

bool mpu6500_read(mpu6500_t *dev)
{
    uint8_t buf[15];
    if (!read_regs(dev, REG_INT_STATUS, buf, sizeof(buf)))
        return false;

    if (!(buf[0] & RAW_DATA_RDY_INT))
        return false;

    int16_t ax = (int16_t)(buf[1]  << 8 | buf[2]);
    int16_t ay = (int16_t)(buf[3]  << 8 | buf[4]);
    int16_t az = (int16_t)(buf[5]  << 8 | buf[6]);
    int16_t tc = (int16_t)(buf[7]  << 8 | buf[8]);
    int16_t gx = (int16_t)(buf[9]  << 8 | buf[10]);
    int16_t gy = (int16_t)(buf[11] << 8 | buf[12]);
    int16_t gz = (int16_t)(buf[13] << 8 | buf[14]);

    // swap X/Y and negate Z to match Bolder Flight convention
    dev->accel[0] = (float)ay * dev->accel_scale * G_MPS2;
    dev->accel[1] = (float)ax * dev->accel_scale * G_MPS2;
    dev->accel[2] = (float)az * dev->accel_scale * -G_MPS2;

    dev->temp = ((float)tc - 21.0f) / 333.87f + 21.0f;

    dev->gyro[0] = (float)gy * dev->gyro_scale * DEG2RAD;
    dev->gyro[1] = (float)gx * dev->gyro_scale * DEG2RAD;
    dev->gyro[2] = (float)gz * dev->gyro_scale * -DEG2RAD;

    return true;
}
