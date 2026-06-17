/**
 * @file    gray_track.c
 * @brief   感为无 MCU 八路灰度传感器的直接 ADC 读取与循迹误差计算。
 *
 * 现在没有再使用 MSPM0 辅助板，也没有 DAT/CLK 串行协议。
 * STM32 直接连接传感器本体：
 *   OUT -> PA4 / ADC1_IN4   传感器当前被选中通道的模拟电压输出
 *   AD0 -> PA3              传感器地址位 0
 *   AD1 -> PB0              传感器地址位 1
 *   AD2 -> PB1              传感器地址位 2
 *
 * 这个传感器可以理解成“8 个灰度探头 + 1 个 8 选 1 模拟开关”。
 * STM32 先用 AD0/AD1/AD2 选中某一路，再从 OUT 读取该路 ADC 值。
 */

#include "main.h"
#include "gray_track.h"
#include "adc.h"
#include "pid.h"
#include "mpu6050.h"
#include "math.h"
#include "Motor.h"

/* 上一次循迹误差。丢线时用它判断该向哪边找线，D 项也依赖它计算误差变化量。 */
float last_err=0;
/* 连续丢线计数。pid_control() 按 10ms 节拍调用 track()，所以 100 次约等于 1s。 */
uint16_t lose_cnt=0;

/* 循迹输出的中间变量：now_out 是本次 PID 差速，last_out/fina_out 用于低通滤波。 */
float now_out=0;
float last_out=0;
float fina_out=0;

/* 循迹环参数。Kp/Kd 是主要参数，Kpp/Kdd 保留给大偏差非线性和陀螺仪阻尼。 */
float Kp=5.0f;
float Kd=1.0f;
float Kpp=0.0f;
float Kdd=0.0f;

/* 循迹基础速度。track() 只修改左右轮目标速度，不直接控制 PWM。 */
#define TRACK_BASE_SPEED 18.0f
#define TRACK_SPEED_MAX  36.0f

/* 每一路连续采样次数。次数越大，ADC 噪声越小，但一次灰度更新越耗时。 */
#define GRAY_ADC_SAMPLES       8u
/* STM32F103 ADC 是 12 位，输出范围 0~4095；这里用 4096 做归一化满量程。 */
#define GRAY_ADC_FULL_SCALE    4096u
/* 8 路 black strength 之和低于该值时，认为没有可靠检测到黑线。 */
#define GRAY_LOST_STRENGTH_MIN 800u

/*
 * gray_analog：每路原始 ADC 值，下标 0~7 对应 O1~O8。
 * gray_normal：按黑白标定归一化后的“白色强度”，黑线附近约 0，白底附近约 4096。
 * gray_dark：由 gray_normal 反算出的“黑线强度”，黑线附近约 4096，白底附近约 0。
 *
 * 后面计算循迹误差时使用 gray_dark，而不是只使用二值 0/1。
 * 这样黑线位于两个探头之间时，误差会平滑变化，不会在两个探头权重之间跳变。
 */
uint16_t gray_analog[8];
uint16_t gray_normal[8];
uint16_t gray_dark[8];

/* 兼容旧数字灰度接口：bit=1 表示白底，bit=0 表示黑线。 */
uint8_t gray_raw=0xFF;

/*
 * 当前使用的黑白标定值。
 * white：传感器放在白底上时的 ADC 值。
 * black：传感器放在黑线上时的 ADC 值。
 *
 * 这里先用官方 STM32 例程里的粗略默认值，让程序可以先跑起来。
 * 真正调车前应当把车架空，用 OLED/VOFA 看 gray_analog[8]，再填入你自己这块传感器的黑白实测值。
 */
uint16_t gray_white_cal[8]={1600,1600,1600,1600,1600,1600,1600,1600};
uint16_t gray_black_cal[8]={100,100,100,100,100,100,100,100};

/**
 * @brief  地址线切换后的短延时。
 *
 * AD0/AD1/AD2 改变后，传感器内部模拟开关会切到另一只探头，OUT 电压不会瞬间稳定。
 * 如果刚切地址马上 ADC，读到的可能是上一通道残留电压和当前通道电压的混合值。
 */
static void gray_address_settle_delay(void)
{
  for(volatile uint16_t i=0;i<400;i++)
  {
    __NOP();
  }
}

/**
 * @brief  选择要读取的灰度通道地址。
 * @param  index 传感器内部地址，范围 0~7。
 *
 * 官方无 MCU 灰度例程里地址线写法是：
 *   AD0 = !(index & 0x01)
 *   AD1 = !(index & 0x02)
 *   AD2 = !(index & 0x04)
 *
 * 也就是说地址位是反相控制：
 *   index=0 -> AD2 AD1 AD0 = 1 1 1
 *   index=7 -> AD2 AD1 AD0 = 0 0 0
 *
 * 这里保持官方逻辑，避免和传感器板上的模拟选通逻辑相反。
 */
static void gray_set_address(uint8_t index)
{
  HAL_GPIO_WritePin(GRAY_AD0_PORT,GRAY_AD0_PIN,(index&0x01u)?GPIO_PIN_RESET:GPIO_PIN_SET);
  HAL_GPIO_WritePin(GRAY_AD1_PORT,GRAY_AD1_PIN,(index&0x02u)?GPIO_PIN_RESET:GPIO_PIN_SET);
  HAL_GPIO_WritePin(GRAY_AD2_PORT,GRAY_AD2_PIN,(index&0x04u)?GPIO_PIN_RESET:GPIO_PIN_SET);
  gray_address_settle_delay();
}

/**
 * @brief  对当前 OUT 通道做多次 ADC 采样并取平均。
 * @return 当前被选中灰度通道的平均 ADC 值。
 *
 * OUT 是模拟量，受环境光、电源噪声、传感器本身噪声影响会有小抖动。
 * 连续采样平均可以降低这种随机噪声，不改变读取逻辑。
 */
static uint16_t gray_adc_average(void)
{
  uint32_t sum=0;

  for(uint8_t i=0;i<GRAY_ADC_SAMPLES;i++)
  {
    sum+=ADC1_ReadValue();
  }

  return (uint16_t)(sum/GRAY_ADC_SAMPLES);
}

/**
 * @brief  把一路原始 ADC 值归一化为“白色强度”。
 * @param  value 当前原始 ADC 值。
 * @param  black 该路黑线标定值。
 * @param  white 该路白底标定值。
 * @return 0~4096，越大表示越接近白底，越小表示越接近黑线。
 *
 * 感为例程的默认假设是：白底 ADC 值大，黑线 ADC 值小。
 * 所以归一化公式是：
 *   normal = (value - black) * 4096 / (white - black)
 */
static uint16_t gray_normalize_one(uint16_t value,uint16_t black,uint16_t white)
{
  if(white<=black)
  {
    return 0;
  }

  if(value<=black)
  {
    return 0;
  }

  if(value>=white)
  {
    return GRAY_ADC_FULL_SCALE;
  }

  return (uint16_t)(((uint32_t)(value-black)*GRAY_ADC_FULL_SCALE)/(uint32_t)(white-black));
}

/**
 * @brief  完整更新 8 路灰度数据。
 *
 * 执行流程：
 *   1. addr 从 0 到 7，逐个选择传感器内部通道。
 *   2. 读取该通道 ADC 平均值，保存到 gray_analog[index]。
 *   3. 根据黑白标定值计算 gray_normal[index]。
 *   4. 用 4096 - gray_normal 得到 gray_dark[index]。
 *   5. 同时生成兼容旧逻辑的 gray_raw 数字量。
 *
 * 注意 addr 和 index 不一定相同。官方例程 Direction=1 时会把地址 0 存到数组最右侧。
 * 这里通过 GRAY_REVERSE_ORDER 保持同样方向。如果实测左右反了，只改头文件里的宏即可。
 */
void gray_sensor_update(void)
{
  uint8_t digital=0;

  for(uint8_t addr=0;addr<8;addr++)
  {
#if GRAY_REVERSE_ORDER
    uint8_t index=7u-addr;
#else
    uint8_t index=addr;
#endif

    gray_set_address(addr);
    gray_analog[index]=gray_adc_average();
    gray_normal[index]=gray_normalize_one(gray_analog[index],gray_black_cal[index],gray_white_cal[index]);
    gray_dark[index]=GRAY_ADC_FULL_SCALE-gray_normal[index];

    /*
     * 生成旧版二值灰度：
     *   gray_dark 大，说明接近黑线，bit 保持 0。
     *   gray_dark 小，说明接近白底，bit 置 1。
     */
    if(gray_dark[index]<((uint16_t)GRAY_ADC_FULL_SCALE/2u))
    {
      digital|=(1u<<index);
    }
  }

  gray_raw=digital;
}

/**
 * @brief  兼容旧代码的 8 路数字灰度读取接口。
 * @return bit0~bit7 对应 O1~O8，1=白底，0=黑线。
 *
 * 旧的辅助板代码通过 DAT/CLK 一次读出 8 个数字位。
 * 现在虽然底层变成 ADC 直接读取，但保留这个函数名，方便原来的 O1~O8 宏继续工作。
 */
uint8_t gray_board_read(void)
{
  gray_sensor_update();
  return gray_raw;
}

/**
 * @brief  计算循迹误差。
 * @return 连续误差值，左偏为负，右偏为正，中心附近为 0。
 *
 * 原来的二值算法是：哪些探头压黑线，就把那些探头的位置平均。
 * 现在改为模拟重心算法：
 *   err = sum(位置权重 * 黑线强度) / sum(黑线强度)
 *
 * 这样做的目的：
 *   黑线从 O4 移到 O5 的过程中，误差会从 -0.5 平滑过渡到 +0.5，
 *   而不是突然跳变。这对抑制循迹摇摆很关键。
 */
float track_error(void)
{
  /*
   * 完全按 Car_OPEN 的权重风格计算误差：
   *   weights = {-7,-5,-3,-1,1,3,5,7}
   *   err = sum(1024 * weight[i] * gray_dark[i]) / sum(gray_dark[i])
   *
   * 这里额外保留当前工程原本的“弱信号/长时间丢线停车”保护。
   * 但在短暂丢线时，不再强行返回 +/-4，而是返回上一帧误差，
   * 这和 Car_OPEN 的“original_sum==0 时返回上一次值”策略一致。
   */
  static const int8_t weights[8]={-7,-5,-3,-1,1,3,5,7};
  uint32_t strength_sum=0;
  int32_t weighted_sum=0;

  gray_sensor_update();

  for(uint8_t i=0;i<8;i++)
  {
    strength_sum+=gray_dark[i];
    weighted_sum+=(int32_t)(1024L*weights[i]*(int32_t)gray_dark[i]);
  }

  if(strength_sum<GRAY_LOST_STRENGTH_MIN)
  {
    lose_cnt++;
    if(lose_cnt>=100)
    {
      pid_set_tar_speed(0,0);
      return 0;
    }
    return last_err;
  }

  lose_cnt=0;
  return (float)weighted_sum/(float)strength_sum;
}

/**
 * @brief  清空循迹控制的跨帧状态。
 *
 * 切换状态、重新起跑、停车后重新循迹时应调用，避免上一次的误差和滤波输出影响下一次起步。
 */
void track_reset(void)
{
  last_err=0.0f;
  lose_cnt=0;
  now_out=0.0f;
  last_out=0.0f;
  fina_out=0.0f;
  gray_raw=0xFF;
}

/**
 * @brief  循迹控制主函数。
 *
 * 这里不直接控制电机 PWM，而是根据灰度误差计算左右轮目标速度：
 *   err > 0：黑线偏右，右轮目标降低、左轮目标提高，让车向右修正。
 *   err < 0：黑线偏左，左轮目标降低、右轮目标提高，让车向左修正。
 *
 * 最后的 PWM 输出仍然交给速度环 pid_cal_motor() 去完成。
 */
void track(void)
{
  float err=track_error();
  float derr=err-last_err;
  last_err=err;

  now_out=Kp*err+Kd*derr+Kpp*(err*fabsf(err))+Kdd*(float)((float)gz-gyro_zero_z)/16.4f;
  fina_out=now_out*0.3f+last_out*0.7f;
  last_out=fina_out;

  if(fina_out>TRACK_BASE_SPEED)  fina_out=TRACK_BASE_SPEED;
  if(fina_out<-TRACK_BASE_SPEED)  fina_out=-TRACK_BASE_SPEED;

  int right=(int)(TRACK_BASE_SPEED-fina_out);
  int left=(int)(TRACK_BASE_SPEED+fina_out);

  if(right<0)  right=0;
  if(left<0)  left=0;
  if(right>(int)TRACK_SPEED_MAX)  right=(int)TRACK_SPEED_MAX;
  if(left>(int)TRACK_SPEED_MAX)  left=(int)TRACK_SPEED_MAX;

  pid_set_tar_speed(right,left);
}

