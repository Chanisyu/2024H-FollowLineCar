#include "mpu6050.h"
#include "i2c.h"

// 这些是原始数据
int16_t ax, ay, az, gx, gy, gz;
// 这些是要通过计算算出来的具体数据
float roll_gyro, pitch_gyro, yaw_gyro;
float roll_acc, pitch_acc, yaw_acc;
float roll_Kalman, pitch_Kalman, yaw_Kalman;
float gyro_zero_z = 0.0f;

// 我重写了MPU6050的模块，具体修改了什么，请查看 `HMC5883L.c` 

HAL_StatusTypeDef MPU6050_Write(uint8_t addr, uint8_t dat)
{
    return HAL_I2C_Mem_Write(&hi2c2,
                             MPU6050_ADDR,
                             addr,						// 内部要操作的地址
                             I2C_MEMADD_SIZE_8BIT,		// 寄存器有多宽
                             &dat,						
                             1,							// 要操作几字节
                             100);
}

HAL_StatusTypeDef MPU6050_Read(uint8_t addr, uint8_t *dat)
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
    uint8_t id = 0;

    HAL_Delay(100);

    if (MPU6050_Read(WHO_AM_I, &id) != HAL_OK)
        return HAL_ERROR;

    if (id != 0x68)
        return HAL_ERROR;

    // 退出睡眠，使用Y轴陀螺仪PLL时钟
    if (MPU6050_Write(PWR_MGMT_1, 0x02) != HAL_OK)
        return HAL_ERROR;

    // 所有轴都使能
    if (MPU6050_Write(PWR_MGMT_2, 0x00) != HAL_OK)
        return HAL_ERROR;

		// 采样率会被底下的DLPF影响
    // 采样率 = 1k / (1 + MPU6050_SMPLRT_DIV_VALUE) = 50Hz，50Hz的周期就是20ms，所以配置了INT的外部中断后，
		// 就会20ms进一次外部中断
    if (MPU6050_Write(SMPLRT_DIV, MPU6050_SMPLRT_DIV_VALUE) != HAL_OK)
        return HAL_ERROR;

    // 开启DLPF，并设置为3，DLPF这玩意本质上就是个低通滤波器
		// 写0或者7就是关闭，此时采样频率是8kHZ，如果开启的话，采样频率就是1kHZ
    if (MPU6050_Write(CONFIG, MPU6050_CONFIG_VALUE) != HAL_OK)
        return HAL_ERROR;

    // 陀螺仪量程 ±2000 dps
    if (MPU6050_Write(GYRO_CONFIG, 0x18) != HAL_OK)
        return HAL_ERROR;

    // 加速度量程 ±2g
    if (MPU6050_Write(ACCEL_CONFIG, 0x00) != HAL_OK)
        return HAL_ERROR;

    // 数据就绪中断使能（如果INT引脚没接，也可以先不写这一句）
    if (MPU6050_Write(INT_ENABLE, 0x01) != HAL_OK)
        return HAL_ERROR;

	  // 关闭 MPU6050 的辅助 I2C Master ( 辅助I2CMaster 和 旁路模式 是互斥的 )
    if (MPU6050_Write(USER_CTRL, 0x00) != HAL_OK)     // USER_CTRL
        return HAL_ERROR;

    // 打开 bypass 旁路模式，让主控可直接访问 XDA/XCL 上的 HMC5883L
    if (MPU6050_Write(INT_PIN_CFG, 0x02) != HAL_OK)   // INT_PIN_CFG, I2C_BYPASS_EN=1
        return HAL_ERROR;
	
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadBytes(uint8_t addr, uint8_t *buf, uint16_t len)
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

    if (MPU6050_ReadBytes(ACCEL_XOUT_H, buf, 14) != HAL_OK)
        return HAL_ERROR;

    ax = (int16_t)((buf[0]  << 8) | buf[1]);
    ay = (int16_t)((buf[2]  << 8) | buf[3]);
    az = (int16_t)((buf[4]  << 8) | buf[5]);

    // buf[6], buf[7] 是温度，如果你后面要用可以再加变量保存
    gx = (int16_t)((buf[8]  << 8) | buf[9]);
    gy = (int16_t)((buf[10] << 8) | buf[11]);
    gz = (int16_t)((buf[12] << 8) | buf[13]);

    return HAL_OK;
}

// 标定零漂
void calibrate_gyro(void)
{
  float sum = 0;
	volatile int a=0;
  for(a=0; a<50; a++) 
	{    
		if(MPU6050_GetData() != HAL_OK)
		{	
			gz = 0;
		}
		sum += gz;
	}
	gyro_zero_z = (sum / 50);
}
