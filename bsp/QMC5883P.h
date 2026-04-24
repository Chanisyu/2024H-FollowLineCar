/* QMC5883P.h */
#ifndef __QMC5883P_H__
#define __QMC5883P_H__

#include "main.h"

typedef enum
{
    QMC5883P_AXIS_X = 0,
    QMC5883P_AXIS_Y = 1,
    QMC5883P_AXIS_Z = 2
} QMC5883P_Axis_t;

/* 
 * QMC5883P 默认 7-bit I2C 地址是 0x2C。
 * STM32 HAL 的 DevAddress 通常传入左移 1 位后的值，
 * 所以这里定义为 0x58。
 */
#define QMC5883P_ADDR                    (0x2CU << 1)
#define QMC5883P_CHIP_ID                 0x80U

/* 官方寄存器表中明确给出的寄存器 */
#define QMC5883P_REG_CHIP_ID             0x00U
#define QMC5883P_REG_XOUT_LSB            0x01U
#define QMC5883P_REG_XOUT_MSB            0x02U
#define QMC5883P_REG_YOUT_LSB            0x03U
#define QMC5883P_REG_YOUT_MSB            0x04U
#define QMC5883P_REG_ZOUT_LSB            0x05U
#define QMC5883P_REG_ZOUT_MSB            0x06U
#define QMC5883P_REG_STATUS              0x09U
#define QMC5883P_REG_CONTROL_1           0x0AU
#define QMC5883P_REG_CONTROL_2           0x0BU

/* 状态寄存器 0x09 */
#define QMC5883P_STATUS_DRDY             0x01U
#define QMC5883P_STATUS_OVFL             0x02U

/* Control Register 1 (0x0A): OSR2<1:0> OSR1<1:0> ODR<1:0> MODE<1:0> */
#define QMC5883P_OSR2_1                  0x00U
#define QMC5883P_OSR2_2                  0x40U
#define QMC5883P_OSR2_4                  0x80U
#define QMC5883P_OSR2_8                  0xC0U

#define QMC5883P_OSR1_8                  0x00U
#define QMC5883P_OSR1_4                  0x10U
#define QMC5883P_OSR1_2                  0x20U
#define QMC5883P_OSR1_1                  0x30U

#define QMC5883P_ODR_10HZ                0x00U
#define QMC5883P_ODR_50HZ                0x04U
#define QMC5883P_ODR_100HZ               0x08U
#define QMC5883P_ODR_200HZ               0x0CU

#define QMC5883P_MODE_SUSPEND            0x00U
#define QMC5883P_MODE_NORMAL             0x01U
#define QMC5883P_MODE_SINGLE             0x02U
#define QMC5883P_MODE_CONTINUOUS         0x03U

/* Control Register 2 (0x0B): SOFT_RST SELF_TEST - - RNG<1:0> SET/RESET_MODE<1:0> */
#define QMC5883P_SOFT_RST                0x80U
#define QMC5883P_SELF_TEST_EN            0x40U

#define QMC5883P_RNG_30G                 0x00U
#define QMC5883P_RNG_12G                 0x04U
#define QMC5883P_RNG_8G                  0x08U
#define QMC5883P_RNG_2G                  0x0CU

#define QMC5883P_SET_RESET_ON            0x00U
#define QMC5883P_SET_ONLY_ON             0x01U
#define QMC5883P_SET_RESET_OFF           0x02U

/*
 * 这两个初始化值直接对应手册里的 Normal Mode Setup Example：
 *   0x0B = 0x08  -> Set/Reset On + 8G
 *   0x0A = 0xCD  -> OSR2=8, OSR1=8, ODR=200Hz, MODE=Normal
 */
#define QMC5883P_CTRL2_INIT              (QMC5883P_RNG_8G | QMC5883P_SET_RESET_ON)
#define QMC5883P_CTRL1_INIT              (QMC5883P_OSR2_8 | QMC5883P_OSR1_8 | QMC5883P_ODR_200HZ | QMC5883P_MODE_NORMAL)

/*
 * 手册示例里出现了 0x29 = 0x06，但 0x29 不在官方寄存器表中。
 * 为了“默认只使用官方明确定义的寄存器”，这个功能默认关闭。
 * 如果你实测发现你的板子必须写这条，再把下面宏改成 1。
 */
#define QMC5883P_USE_UNDOCUMENTED_SIGN_REG   0U
#define QMC5883P_REG_SIGN                    0x29U
#define QMC5883P_SIGN_REG_VALUE              0x06U

/*
 * 下面这组参数专门给偏航角解算用：
 * 1) offset/scale 用于后续磁力计校准
 * 2) YAW_X/YAW_Y 用于把模块坐标系映射到小车前向/左向平面
 */
#define QMC5883P_X_OFFSET                0.0f
#define QMC5883P_Y_OFFSET                0.0f
#define QMC5883P_Z_OFFSET                0.0f

#define QMC5883P_X_SCALE                 1.0f
#define QMC5883P_Y_SCALE                 1.0f
#define QMC5883P_Z_SCALE                 1.0f

#define QMC5883P_YAW_X_AXIS              QMC5883P_AXIS_X
#define QMC5883P_YAW_Y_AXIS              QMC5883P_AXIS_Y
#define QMC5883P_YAW_X_SIGN              1.0f
#define QMC5883P_YAW_Y_SIGN              1.0f

HAL_StatusTypeDef QMC5883P_Write(uint8_t addr, uint8_t dat);
HAL_StatusTypeDef QMC5883P_Read(uint8_t addr, uint8_t *dat);
HAL_StatusTypeDef QMC5883P_Init(void);
HAL_StatusTypeDef QMC5883P_GetData(int16_t *x, int16_t *y, int16_t *z);
float QMC5883P_GetHeadingDeg(int16_t raw_x, int16_t raw_y, int16_t raw_z);
float QMC5883P_GetContinuousYawDeg(int16_t raw_x, int16_t raw_y, int16_t raw_z);

extern int16_t qmc_x, qmc_y, qmc_z;
extern float yaw_qmc;

#endif
