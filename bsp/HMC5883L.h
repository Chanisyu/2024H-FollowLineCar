#ifndef __HMC5883L_H__
#define __HMC5883L_H__
#include "main.h"
#include "math.h"

#define HMC5883L_ADDR 0x3c
#define HMC5883L_CRA 0x00
#define HMC5883L_CRB 0x01
#define HMC5883L_MR 0x02
#define HMC5883L_DOXMR 0x03
#define HMC5883L_DOXLR 0x04
#define HMC5883L_DOZMR 0x05
#define HMC5883L_DOZLR 0x06
#define HMC5883L_DOYMR 0x07
#define HMC5883L_DOYLR 0x08
#define HMC5883L_SR 0x09
#define HMC5883L_IRA 0x0A
#define HMC5883L_IRB 0x0B
#define HMC5883L_IRC 0x0C

#define OFFSET_X -15
#define OFFSET_Y -285
#define OFFSET_Z -214
#define SCALE_X 1.05
#define SCALE_Y 1.00
#define SCALE_Z 1.01

HAL_StatusTypeDef HMC5883L_Write(uint8_t addr, uint8_t dat);
HAL_StatusTypeDef HMC5883L_Read(uint8_t addr, uint8_t *dat);
HAL_StatusTypeDef HMC5883L_Init(void);
HAL_StatusTypeDef HMC5883L_GetData(int16_t *x, int16_t *y, int16_t *z);

extern int16_t hmc_x, hmc_y, hmc_z;
extern float  hmc_x_cal, hmc_y_cal, hmc_z_cal;
extern float yaw_hmc;

#endif
