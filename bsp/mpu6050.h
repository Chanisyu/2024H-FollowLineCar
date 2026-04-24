#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "main.h"
#include "math.h"

#define MPU6050_ADDR        (0x68 << 1)   // AD0鎺ュ湴鏃跺湴鍧€0x68锛孒AL閲屽乏绉?浣?

#define SMPLRT_DIV          0x19
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C

#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40

#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42

#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48

#define INT_ENABLE          0x38
#define PWR_MGMT_1          0x6B
#define PWR_MGMT_2          0x6C
#define WHO_AM_I            0x75

#define USER_CTRL    0x6A
#define INT_PIN_CFG  0x37

#define MPU6050_SMPLRT_DIV_VALUE 0x13
#define MPU6050_CONFIG_VALUE 0x03

extern int16_t ax, ay, az, gx, gy, gz;
extern float roll_gyro, pitch_gyro, yaw_gyro;
extern float roll_acc, pitch_acc, yaw_acc;
extern float roll_Kalman, pitch_Kalman, yaw_Kalman;
extern float gyro_zero_z;

HAL_StatusTypeDef MPU6050_Write(uint8_t addr, uint8_t dat);
HAL_StatusTypeDef MPU6050_Read(uint8_t addr, uint8_t *dat);
HAL_StatusTypeDef MPU6050_ReadBytes(uint8_t addr, uint8_t *buf, uint16_t len);

HAL_StatusTypeDef MPU6050_Init(void);
HAL_StatusTypeDef MPU6050_GetData(void);
void calibrate_gyro(void);

#endif
