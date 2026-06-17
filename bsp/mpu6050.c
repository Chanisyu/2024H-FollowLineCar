/**
 * @file    mpu6050.c
 * @brief   MPU6050 初始化、原始数据读取和陀螺仪零漂校准。
 *
 * 文件结构：
 *   1. MPU6050_Write()      - 单字节写寄存器
 *   2. MPU6050_Read()       - 单字节读寄存器
 *   3. MPU6050_Init()       - 配置采样率、量程、中断和旁路模式
 *   4. MPU6050_ReadBytes()  - 批量读取连续寄存器
 *   5. MPU6050_GetData()    - 读取加速度计和陀螺仪原始值
 *   6. calibrate_gyro()     - 采样平均得到 Z 轴零偏
 */

#include "mpu6050.h"
#include "i2c.h"

/*
 * 变量说明：
 * 1. ax/ay/az：MPU6050 加速度计三轴原始值。
 * 2. gx/gy/gz：MPU6050 陀螺仪三轴原始值。
 * 3. gyro_zero_z：Z 轴陀螺仪零偏平均值，用于修正 gz。
 * 4. roll_gyro/pitch_gyro/yaw_gyro：纯陀螺仪积分得到的姿态角，短期快、长期漂。
 * 5. roll_acc/pitch_acc/yaw_acc：由加速度计粗算的角度，抗漂移但受运动干扰。
 * 6. hmc_x/hmc_y/hmc_z：磁力计原始值，在 HMC5883L 模块中定义。
 * 7. hmc_x_cal/hmc_y_cal/hmc_z_cal：磁力计校准后的值。
 * 8. yaw_hmc：由 atan2f(hmc_y_cal,hmc_x_cal) 计算的磁力计航向角。
 * 9. HMC5883L_Cali_Res：磁力计校准结果，包含 offset 和 scale。
 * 10. offset_x/y/z 与 scale_x/y/z：分别修正硬铁偏移和软铁比例差异。
 */


// 这些是原始数据
int16_t ax,ay,az,gx,gy,gz;
float roll_gyro,pitch_gyro,yaw_gyro;
float roll_acc,pitch_acc,yaw_acc;
float roll_Kalman,pitch_Kalman,yaw_Kalman;
float gyro_zero_z=0.0f;  // Z 轴陀螺仪零偏，跨帧保存，供偏航角积分修正使用。

// 我重写了 MPU6050 的模块，具体修改了什么，请查看 `HMC5883L.c`。

HAL_StatusTypeDef MPU6050_Write(uint8_t addr,uint8_t dat)
{
  return HAL_I2C_Mem_Write(&hi2c2,
                           MPU6050_ADDR,
                           addr,  // 内部寄存器地址。
                           I2C_MEMADD_SIZE_8BIT,
                           &dat,
                           1,
                           100);
}

HAL_StatusTypeDef MPU6050_Read(uint8_t addr,uint8_t *dat)
{
  return HAL_I2C_Mem_Read(&hi2c2,
                          MPU6050_ADDR,
                          addr,
                          I2C_MEMADD_SIZE_8BIT,
                          dat,
                          1,
                          100);
}

HAL_StatusTypeDef MPU6050_Init(void)
{
  uint8_t id=0;

  HAL_Delay(100);  // 上电后等待内部时钟和寄存器状态稳定。

  if(MPU6050_Read(WHO_AM_I,&id)!=HAL_OK)  return HAL_ERROR;
  if(id!=0x68)  return HAL_ERROR;

  // 退出睡眠，使用 Y 轴陀螺仪 PLL 时钟。
  if(MPU6050_Write(PWR_MGMT_1,0x02)!=HAL_OK)  return HAL_ERROR;

  // 使能所有轴。
  if(MPU6050_Write(PWR_MGMT_2,0x00)!=HAL_OK)  return HAL_ERROR;

  // 采样率会被 DLPF 影响；当前配置对应 100Hz，外部中断周期约 10ms。
  if(MPU6050_Write(SMPLRT_DIV,MPU6050_SMPLRT_DIV_VALUE)!=HAL_OK)  return HAL_ERROR;

  // DLPF=3 时既保留一定带宽，又能削弱陀螺仪高频噪声。
  if(MPU6050_Write(CONFIG,MPU6050_CONFIG_VALUE)!=HAL_OK)  return HAL_ERROR;

  // 陀螺仪量程 ±2000 dps，适合小车快速转向场景。
  if(MPU6050_Write(GYRO_CONFIG,0x18)!=HAL_OK)  return HAL_ERROR;

  // 加速度计量程 ±2g，分辨率最高，适合姿态融合。
  if(MPU6050_Write(ACCEL_CONFIG,0x00)!=HAL_OK)  return HAL_ERROR;

  // 数据就绪中断使能，PB5 外部中断由 main.c 里统一处理。
  if(MPU6050_Write(INT_ENABLE,0x01)!=HAL_OK)  return HAL_ERROR;

  // 关闭辅助 I2C Master，避免与旁路模式互斥。
  if(MPU6050_Write(USER_CTRL,0x00)!=HAL_OK)  return HAL_ERROR;

  // 打开 bypass 旁路模式，让主控可直接访问 XDA/XCL 上的 HMC5883L。
  if(MPU6050_Write(INT_PIN_CFG,0x02)!=HAL_OK)  return HAL_ERROR;

  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadBytes(uint8_t addr,uint8_t *buf,uint16_t len)
{
  return HAL_I2C_Mem_Read(&hi2c2,
                          MPU6050_ADDR,
                          addr,
                          I2C_MEMADD_SIZE_8BIT,
                          buf,
                          len,
                          100);
}

HAL_StatusTypeDef MPU6050_GetData(void)
{
  uint8_t buf[14];

  if(MPU6050_ReadBytes(ACCEL_XOUT_H,buf,14)!=HAL_OK)  return HAL_ERROR;

  ax=(int16_t)((buf[0]<<8)|buf[1]);
  ay=(int16_t)((buf[2]<<8)|buf[3]);
  az=(int16_t)((buf[4]<<8)|buf[5]);

  // buf[6], buf[7] 是温度，如果后面要用可以再单独保存。
  gx=(int16_t)((buf[8]<<8)|buf[9]);
  gy=(int16_t)((buf[10]<<8)|buf[11]);
  gz=(int16_t)((buf[12]<<8)|buf[13]);

  return HAL_OK;
}

// 标定零漂：连续采样 50 次取平均，得到 Z 轴陀螺仪静止偏置。
void calibrate_gyro(void)
{
  float sum=0;
  volatile int a=0;

  for(a=0;a<50;a++)
  {
    if(MPU6050_GetData()!=HAL_OK)
    {
      gz=0;
    }
    sum+=gz;
  }

  gyro_zero_z=(sum/50);
}
