#ifndef __DRV_MPU6500_H__
#define __DRV_MPU6500_H__

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6500_CONFIG                0x1A
#define MPU6500_GYRO_CONFIG           0x1B
#define MPU6500_ACCEL_CONFIG          0x1C
#define MPU6500_ACCEL_XOUT_H          0x3B
#define MPU6500_ACCEL_XOUT_L          0x3C
#define MPU6500_ACCEL_YOUT_H          0x3D
#define MPU6500_ACCEL_YOUT_L          0x3E
#define MPU6500_ACCEL_ZOUT_H          0x3F
#define MPU6500_ACCEL_ZOUT_L          0x40
#define MPU6500_TEMP_OUT_H            0x41
#define MPU6500_TEMP_OUT_L            0x42
#define MPU6500_GYRO_XOUT_H           0x43
#define MPU6500_GYRO_XOUT_L           0x44
#define MPU6500_GYRO_YOUT_H           0x45
#define MPU6500_GYRO_YOUT_L           0x46
#define MPU6500_GYRO_ZOUT_H           0x47
#define MPU6500_GYRO_ZOUT_L           0x48

#define MPU6500_SIGNAL_PATH_RESET     0x68
#define MPU6500_PWR_MGMT_1            0x6B
#define MPU6500_PWR_MGMT_2            0x6C
#define MPU6500_WHO_AM_I              0x75

typedef struct
{
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t tem;
    float acc_x_f;
    float acc_y_f;
    float acc_z_f;
} imu_raw_data;

typedef struct
{
    void    *bus_dev;
    uint8_t  bus_index;
    int32_t  cs_pin;

    uint32_t dma_buf_size;
    uint8_t *dma_buf;

    uint8_t  is_busy;
    volatile uint32_t dma_buf_offset;

    imu_raw_data imu_data;

    uint8_t is_init;
} imu_mpu6500_dev_t;

void imu_6500_test(int pin);

#ifdef __cplusplus
}
#endif
#endif
