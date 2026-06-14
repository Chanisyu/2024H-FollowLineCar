#include "HMC5883L.h"
#include "i2c.h"

int16_t hmc_x, hmc_y, hmc_z;
float hmc_x_cal, hmc_y_cal, hmc_z_cal;
float yaw_hmc;

// 我重写了 HMC5883L 的模块，把它从软件 I2C + 私有库的形式，改成硬件 I2C + HAL 库的形式
// 函数返回值重写为 HAL_StatusTypeDef，使函数可以返回执行状态
// 硬件地址宏定义在 .h 文件里，可右键跳转查看，硬件地址在寄存器手册里可以查看

/*
    参数一：硬件内目标写入地址
    参数二：要写入的字节
*/

HAL_StatusTypeDef HMC5883L_Write(uint8_t addr, uint8_t dat)
{
	// Mem 就是把“找硬件地址、找硬件内地址、写/读”三件事合在一起的硬件 I2C 函数。
	
	return HAL_I2C_Mem_Write(&hi2c2,
                             HMC5883L_ADDR,
                             addr,										// 内部要操作的地址
                             I2C_MEMADD_SIZE_8BIT,		// 寄存器有多宽
                             &dat,						
                             1,												// 要操作几字节
                             10);
}

// 原来读取函数是直接返回读取值，
// 更改成了返回执行状态，增加了一个指针参数，用于存储返回值

/*
    参数一：硬件内目标读取地址
    参数二：返回值将要写入的变量的指针
*/

HAL_StatusTypeDef HMC5883L_Read(uint8_t addr, uint8_t *dat)
{
  return HAL_I2C_Mem_Read(&hi2c2,
													HMC5883L_ADDR,
													addr,
													I2C_MEMADD_SIZE_8BIT,
													dat,
													1,
													10);
}

HAL_StatusTypeDef HMC5883L_ReadBytes(uint8_t addr, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c2,
                            HMC5883L_ADDR,
                            addr,
                            I2C_MEMADD_SIZE_8BIT,
                            buf,
                            len,
                            10);
}

HAL_StatusTypeDef HMC5883L_Init()
{
	HAL_Delay(100);
	
	// 最大输出速率：75Hz
	if (HMC5883L_Write(HMC5883L_CRA, 0x78) != HAL_OK)
        return HAL_ERROR;
	
	// 默认增益
	if (HMC5883L_Write(HMC5883L_CRB, 0x20) != HAL_OK)
        return HAL_ERROR;
	
	// 连续测量
	if (HMC5883L_Write(HMC5883L_MR, 0x00) != HAL_OK)
        return HAL_ERROR;

	HAL_Delay(20); 
	
	return HAL_OK;
}

HAL_StatusTypeDef HMC5883L_GetData(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];

    if (HMC5883L_ReadBytes(HMC5883L_DOXMR, buf, 6) != HAL_OK)
        return HAL_ERROR;

    // HMC5883L 连续寄存器顺序是:
		// 0x03 X_MSB
		// 0x04 X_LSB
		// 0x05 Z_MSB
		// 0x06 Z_LSB
		// 0x07 Y_MSB
		// 0x08 Y_LSB

    *x = (int16_t)((buf[0] << 8) | buf[1]);
    *z = (int16_t)((buf[2] << 8) | buf[3]);
    *y = (int16_t)((buf[4] << 8) | buf[5]);

    return HAL_OK;
}




	


