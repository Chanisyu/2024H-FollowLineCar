/**
 * @file    QMC5883P.c
 * @brief   QMC5883P 三轴磁力计硬件 I2C 驱动和航向角解算。
 *
 * 文件结构：
 *   1. QMC5883P_NormalizeAngleDeg()      - 将角度限制到 0~360 度
 *   2. QMC5883P_UnwrapAngleDeg()         - 将航向角展开为连续角度
 *   3. QMC5883P_GetHeadingDeg()          - 用校准参数计算磁力计航向角
 *   4. QMC5883P_Init()                   - 复位并配置 QMC5883P
 *   5. QMC5883P_GetData()                - 读取 X/Y/Z 三轴原始数据
 */

#include "QMC5883P.h"
#include "i2c.h"
#include "math.h"

int16_t qmc_x,qmc_y,qmc_z;
float yaw_qmc;

static float QMC5883P_NormalizeAngleDeg(float angle_deg)
{
  while(angle_deg<0.0f)
  {
    angle_deg+=360.0f;
  }

  while(angle_deg>=360.0f)
  {
    angle_deg-=360.0f;
  }

  return angle_deg;
}

static float QMC5883P_UnwrapAngleDeg(float current_deg)
{
  static uint8_t initialized=0U;      // 跨帧标记：第一帧没有上一角度可比较。
  static float last_deg=0.0f;         // 跨帧保存上一轮 0~360 度航向角。
  static float continuous_deg=0.0f;   // 跨帧保存展开后的连续角度，便于做角度环。
  float delta_deg;

  if(initialized==0U)
  {
    initialized=1U;
    last_deg=current_deg;
    continuous_deg=current_deg;
    return continuous_deg;
  }

  delta_deg=current_deg-last_deg;

  if(delta_deg>180.0f)
  {
    delta_deg-=360.0f;
  }
  else if(delta_deg<-180.0f)
  {
    delta_deg+=360.0f;
  }

  continuous_deg+=delta_deg;
  last_deg=current_deg;

  return continuous_deg;
}

float QMC5883P_GetHeadingDeg(int16_t raw_x,int16_t raw_y,int16_t raw_z)
{
  float axes[3];
  float heading_x;
  float heading_y;
  float heading_deg;

  axes[QMC5883P_AXIS_X]=((float)raw_x-QMC5883P_X_OFFSET)*QMC5883P_X_SCALE;
  axes[QMC5883P_AXIS_Y]=((float)raw_y-QMC5883P_Y_OFFSET)*QMC5883P_Y_SCALE;
  axes[QMC5883P_AXIS_Z]=((float)raw_z-QMC5883P_Z_OFFSET)*QMC5883P_Z_SCALE;

  heading_x=axes[QMC5883P_YAW_X_AXIS]*QMC5883P_YAW_X_SIGN;
  heading_y=axes[QMC5883P_YAW_Y_AXIS]*QMC5883P_YAW_Y_SIGN;

  heading_deg=atan2f(heading_y,heading_x)*57.2957795f;

  return QMC5883P_NormalizeAngleDeg(heading_deg);
}

float QMC5883P_GetContinuousYawDeg(int16_t raw_x,int16_t raw_y,int16_t raw_z)
{
  return QMC5883P_UnwrapAngleDeg(QMC5883P_GetHeadingDeg(raw_x,raw_y,raw_z));
}

//// 这个QMC5883P模块纯是AI帮我写的，我把 QMC5883P的数据手册 和 我重写的HMC5883L 给了它
//// 让它照葫芦画瓢给我写一个QMC5883P的，写完一测能用，那就别动它了好吧。


/*
 * 内部静态函数：一次性连续读取多个字节。
 * QMC5883P 的 X/Y/Z 数据寄存器是 0x01~0x06 连续排列，
 * 用 burst read 比单字节读更合理，也更不容易在跨更新边界时读裂。
 */
static HAL_StatusTypeDef QMC5883P_ReadBytes(uint8_t addr,uint8_t *buf,uint16_t len)
{
  return HAL_I2C_Mem_Read(&hi2c2,
                          QMC5883P_ADDR,
                          addr,
                          I2C_MEMADD_SIZE_8BIT,
                          buf,
                          len,
                          100);
}

/*
 * 写单字节寄存器
 */
HAL_StatusTypeDef QMC5883P_Write(uint8_t addr,uint8_t dat)
{
  return HAL_I2C_Mem_Write(&hi2c2,
                           QMC5883P_ADDR,
                           addr,
                           I2C_MEMADD_SIZE_8BIT,
                           &dat,
                           1,
                           100);
}

/*
 * 读单字节寄存器
 */
HAL_StatusTypeDef QMC5883P_Read(uint8_t addr,uint8_t *dat)
{
  return HAL_I2C_Mem_Read(&hi2c2,
                          QMC5883P_ADDR,
                          addr,
                          I2C_MEMADD_SIZE_8BIT,
                          dat,
                          1,
                          100);
}

/*
 * 初始化流程：
 * 1) 等待上电稳定
 * 2) 读取 CHIPID，确认总线和器件都对
 * 3) 执行 soft reset，保证寄存器回到已知状态
 * 4) （可选）写入示例里出现但寄存器表未定义的 0x29=0x06
 * 5) 按手册 normal mode example 写入 0x0B 和 0x0A
 * 6) 回读校验，确保配置真正写进去
 */
HAL_StatusTypeDef QMC5883P_Init(void)
{
  uint8_t id=0;
  uint8_t verify=0;

    /* 
     * 数据手册给出外部电源典型上升时间 50ms，POR 完成时间典型 250us。
     * 这里保守等待 50ms，避免电源刚起来就访问寄存器。
     */
  HAL_Delay(50);

  /* 先读 CHIPID，QMC5883P 默认值应为 0x80 */
  if(QMC5883P_Read(QMC5883P_REG_CHIP_ID,&id)!=HAL_OK)  return HAL_ERROR;

  if(id!=QMC5883P_CHIP_ID)  return HAL_ERROR;

  /* soft reset：手册明确给了 0x0B[7] = 1 */
  if(QMC5883P_Write(QMC5883P_REG_CONTROL_2,QMC5883P_SOFT_RST)!=HAL_OK)  return HAL_ERROR;

  /* soft reset 后给器件一点恢复时间，远大于手册给的 250us */
  HAL_Delay(1);

#if QMC5883P_USE_UNDOCUMENTED_SIGN_REG
    /*
     * 注意：
     * 0x29=0x06 只出现在 datasheet 的 example 中，
     * 不在正式寄存器表里，所以默认关闭。
     * 只有你实测确认板级轴向需要这条时，才建议打开。
     */
  if(QMC5883P_Write(QMC5883P_REG_SIGN,QMC5883P_SIGN_REG_VALUE)!=HAL_OK)  return HAL_ERROR;
#endif

  /* 先配 Control_2：Set/Reset On + 8G */
  if(QMC5883P_Write(QMC5883P_REG_CONTROL_2,QMC5883P_CTRL2_INIT)!=HAL_OK)  return HAL_ERROR;

  /* 再配 Control_1：Normal mode + 200Hz + 最大 OSR */
  if(QMC5883P_Write(QMC5883P_REG_CONTROL_1,QMC5883P_CTRL1_INIT)!=HAL_OK)  return HAL_ERROR;

  /* 回读校验 Control_2 */
  if(QMC5883P_Read(QMC5883P_REG_CONTROL_2,&verify)!=HAL_OK)  return HAL_ERROR;

  if(verify!=QMC5883P_CTRL2_INIT)  return HAL_ERROR;

  /* 回读校验 Control_1 */
  if(QMC5883P_Read(QMC5883P_REG_CONTROL_1,&verify)!=HAL_OK)  return HAL_ERROR;

  if(verify!=QMC5883P_CTRL1_INIT)  return HAL_ERROR;

    /*
     * 当前 ODR=200Hz，1 个输出周期约 5ms。
     * 这里等 5ms，保证后续第一次取数时更有机会拿到新样本。
     */
  HAL_Delay(5);

  return HAL_OK;
}

/*
 * 读取三轴数据
 *
 * 返回值语义：
 * - HAL_OK   : 成功读到一组新数据
 * - HAL_BUSY : 当前还没有新数据（DRDY=0）
 * - HAL_ERROR: I2C 出错，或状态寄存器报告溢出（OVFL=1）
 */
HAL_StatusTypeDef QMC5883P_GetData(int16_t *x,int16_t *y,int16_t *z)
{
  uint8_t status=0;
  uint8_t buf[6];

  /* 防御式检查，避免空指针 */
  if((x==0)||(y==0)||(z==0))  return HAL_ERROR;

    /*
     * 先读状态寄存器：
     * bit0 = DRDY，bit1 = OVFL。
     * 手册说明读取状态寄存器会清 DRDY/OVFL 标志，
     * 所以这里先判断状态，再立即读数据。
     */
  if(QMC5883P_Read(QMC5883P_REG_STATUS,&status)!=HAL_OK)  return HAL_ERROR;

  /* 溢出说明本次数据已不可信 */
  if((status&QMC5883P_STATUS_OVFL)!=0U)  return HAL_ERROR;

  /* 没有新数据时，不硬等，直接返回 HAL_BUSY 让上层决定是否继续轮询 */
  if((status&QMC5883P_STATUS_DRDY)==0U)  return HAL_BUSY;

    /*
     * QMC5883P 的输出寄存器是连续的：
     * 0x01~0x06 依次为 X_L, X_H, Y_L, Y_H, Z_L, Z_H
     */
  if(QMC5883P_ReadBytes(QMC5883P_REG_XOUT_LSB,buf,6)!=HAL_OK)  return HAL_ERROR;

  /* 数据是 16-bit 二补码，低字节在前，高字节在后 */
  *x=(int16_t)((uint16_t)buf[0]|((uint16_t)buf[1]<<8));
  *y=(int16_t)((uint16_t)buf[2]|((uint16_t)buf[3]<<8));
  *z=(int16_t)((uint16_t)buf[4]|((uint16_t)buf[5]<<8));

  return HAL_OK;
}
