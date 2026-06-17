/**
 * @file    OLED.c
 * @brief   SSD1306 OLED 软件 I2C 显示驱动和反白显示扩展。
 *
 * 文件结构：
 *   1. OLED_I2C_Init()          - 初始化 PB8/PB9 开漏输出
 *   2. OLED_I2C_Start/Stop()    - 模拟 I2C 起始和停止时序
 *   3. OLED_I2C_SendByte()      - 软件 I2C 发送 1 字节
 *   4. OLED_WriteCommand/Data() - 写 OLED 命令或显示数据
 *   5. OLED_Show*()             - 字符、字符串和数字显示
 *   6. OLED_Init()              - SSD1306 初始化命令序列
 *   7. OLED_*Reverse()          - 反白显示扩展
 */

#include "main.h"
#include "OLED_Font.h"

// PB8=SCL，PB9=SDA，软件 I2C 开漏输出；写 1 释放总线，写 0 拉低总线。
#define OLED_W_SCL(x)  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,(x)?GPIO_PIN_SET:GPIO_PIN_RESET)
#define OLED_W_SDA(x)  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,(x)?GPIO_PIN_SET:GPIO_PIN_RESET)

void OLED_I2C_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStructure={0};
  GPIO_InitStructure.Mode=GPIO_MODE_OUTPUT_OD;
  GPIO_InitStructure.Speed=GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pin=GPIO_PIN_8;
  HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);
  GPIO_InitStructure.Pin=GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);

  OLED_W_SCL(1);
  OLED_W_SDA(1);
}

// I2C 起始：SCL 为高时 SDA 从高到低，OLED 将其识别为一次传输开始。
void OLED_I2C_Start(void)
{
  OLED_W_SDA(1);
  OLED_W_SCL(1);
  OLED_W_SDA(0);
  OLED_W_SCL(0);
}

// I2C 停止：SCL 为高时 SDA 从低到高，OLED 将其识别为一次传输结束。
void OLED_I2C_Stop(void)
{
  OLED_W_SDA(0);
  OLED_W_SCL(1);
  OLED_W_SDA(1);
}

void OLED_I2C_SendByte(uint8_t Byte)
{
  uint8_t i;

  for(i=0;i<8;i++)
  {
    OLED_W_SDA(!!(Byte&(0x80>>i)));  // 先发送高位，符合 SSD1306 I2C 字节时序。
    OLED_W_SCL(1);
    OLED_W_SCL(0);
  }

  OLED_W_SCL(1);  // 额外时钟位用于跳过 ACK，本驱动不读取应答信号。
  OLED_W_SCL(0);
}

void OLED_WriteCommand(uint8_t Command)
{
  OLED_I2C_Start();
  OLED_I2C_SendByte(0x78);  // SSD1306 7-bit 地址 0x3C 左移 1 位后的写地址。
  OLED_I2C_SendByte(0x00);  // 控制字节：后续字节解释为命令。
  OLED_I2C_SendByte(Command);
  OLED_I2C_Stop();
}

void OLED_WriteData(uint8_t Data)
{
  OLED_I2C_Start();
  OLED_I2C_SendByte(0x78);  // SSD1306 7-bit 地址 0x3C 左移 1 位后的写地址。
  OLED_I2C_SendByte(0x40);  // 控制字节：后续字节解释为显示数据。
  OLED_I2C_SendByte(Data);
  OLED_I2C_Stop();
}

void OLED_SetCursor(uint8_t Y,uint8_t X)
{
  OLED_WriteCommand(0xB0|Y);             // 页地址，Y 范围 0~7。
  OLED_WriteCommand(0x10|((X&0xF0)>>4)); // 列地址高 4 位。
  OLED_WriteCommand(0x00|(X&0x0F));      // 列地址低 4 位。
}

void OLED_Clear(void)
{
  uint8_t i,j;

  for(j=0;j<8;j++)
  {
    OLED_SetCursor(j,0);
    for(i=0;i<128;i++)
    {
      OLED_WriteData(0x00);
    }
  }
}

void OLED_ShowChar(uint8_t Line,uint8_t Column,char Char)
{
  uint8_t i;

  OLED_SetCursor((Line-1)*2,(Column-1)*8);
  for(i=0;i<8;i++)
  {
    OLED_WriteData(OLED_F8x16[Char-' '][i]);
  }

  OLED_SetCursor((Line-1)*2+1,(Column-1)*8);
  for(i=0;i<8;i++)
  {
    OLED_WriteData(OLED_F8x16[Char-' '][i+8]);
  }
}

void OLED_ShowString(uint8_t Line,uint8_t Column,char *String)
{
  uint8_t i;

  for(i=0;String[i]!='\0';i++)
  {
    OLED_ShowChar(Line,Column+i,String[i]);
  }
}

uint32_t OLED_Pow(uint32_t X,uint32_t Y)
{
  uint32_t Result=1;

  while(Y--)
  {
    Result*=X;
  }

  return Result;
}

void OLED_ShowNum(uint8_t Line,uint8_t Column,uint32_t Number,uint8_t Length)
{
  uint8_t i;

  for(i=0;i<Length;i++)
  {
    OLED_ShowChar(Line,Column+i,Number/OLED_Pow(10,Length-i-1)%10+'0');
  }
}

void OLED_ShowSignedNum(uint8_t Line,uint8_t Column,int32_t Number,uint8_t Length)
{
  uint8_t i;
  uint32_t Number1;

  if(Number>=0)
  {
    OLED_ShowChar(Line,Column,'+');
    Number1=Number;
  }
  else
  {
    OLED_ShowChar(Line,Column,'-');
    Number1=-Number;
  }

  for(i=0;i<Length;i++)
  {
    OLED_ShowChar(Line,Column+i+1,Number1/OLED_Pow(10,Length-i-1)%10+'0');
  }
}

void OLED_ShowHexNum(uint8_t Line,uint8_t Column,uint32_t Number,uint8_t Length)
{
  uint8_t i,SingleNumber;

  for(i=0;i<Length;i++)
  {
    SingleNumber=Number/OLED_Pow(16,Length-i-1)%16;
    if(SingleNumber<10)
    {
      OLED_ShowChar(Line,Column+i,SingleNumber+'0');
    }
    else
    {
      OLED_ShowChar(Line,Column+i,SingleNumber-10+'A');
    }
  }
}

void OLED_ShowBinNum(uint8_t Line,uint8_t Column,uint32_t Number,uint8_t Length)
{
  uint8_t i;

  for(i=0;i<Length;i++)
  {
    OLED_ShowChar(Line,Column+i,Number/OLED_Pow(2,Length-i-1)%2+'0');
  }
}

// ================================================================
// 1. OLED 初始化命令序列
//
//    上电后先等待供电稳定，再按 SSD1306 常用配置写入页扫描方向、
//    对比度、预充电周期、VCOMH 和充电泵等寄存器。
// ================================================================
void OLED_Init(void)
{
  uint32_t i,j;

  for(i=0;i<1000;i++)
  {
    for(j=0;j<1000;j++);
  }

  OLED_I2C_Init();

  OLED_WriteCommand(0xAE);  // 关闭显示，避免初始化过程中屏幕闪烁。

  OLED_WriteCommand(0xD5);
  OLED_WriteCommand(0x80);

  OLED_WriteCommand(0xA8);
  OLED_WriteCommand(0x3F);

  OLED_WriteCommand(0xD3);
  OLED_WriteCommand(0x00);

  OLED_WriteCommand(0x40);

  OLED_WriteCommand(0xA1);  // 左右方向：0xA1 正常，0xA0 左右反置。

  OLED_WriteCommand(0xC8);  // 上下方向：0xC8 正常，0xC0 上下反置。

  OLED_WriteCommand(0xDA);
  OLED_WriteCommand(0x12);

  OLED_WriteCommand(0x81);
  OLED_WriteCommand(0xCF);

  OLED_WriteCommand(0xD9);
  OLED_WriteCommand(0xF1);

  OLED_WriteCommand(0xDB);
  OLED_WriteCommand(0x30);

  OLED_WriteCommand(0xA4);

  OLED_WriteCommand(0xA6);

  OLED_WriteCommand(0x8D);
  OLED_WriteCommand(0x14);

  OLED_WriteCommand(0xAF);  // 开启显示。

  OLED_Clear();
}

// ================================================================
// 2. 反白显示扩展
//
//    一行文本对应两个 SSD1306 page，清行/填充/反白都按两个 page 处理。
//    反白字符通过对 8x16 字模取反实现，便于菜单选中项高亮。
// ================================================================
void OLED_ClearLine(uint8_t Line)
{
  uint8_t i;
  uint8_t page=(Line-1)*2;

  OLED_SetCursor(page,0);
  for(i=0;i<128;i++)
  {
    OLED_WriteData(0x00);
  }

  OLED_SetCursor(page+1,0);
  for(i=0;i<128;i++)
  {
    OLED_WriteData(0x00);
  }
}

void OLED_FillLine(uint8_t Line)
{
  uint8_t i;
  uint8_t page=(Line-1)*2;

  OLED_SetCursor(page,0);
  for(i=0;i<128;i++)
  {
    OLED_WriteData(0xFF);
  }

  OLED_SetCursor(page+1,0);
  for(i=0;i<128;i++)
  {
    OLED_WriteData(0xFF);
  }
}

void OLED_ShowCharReverse(uint8_t Line,uint8_t Column,char Char)
{
  uint8_t i;

  OLED_SetCursor((Line-1)*2,(Column-1)*8);
  for(i=0;i<8;i++)
  {
    OLED_WriteData(~OLED_F8x16[Char-' '][i]);
  }

  OLED_SetCursor((Line-1)*2+1,(Column-1)*8);
  for(i=0;i<8;i++)
  {
    OLED_WriteData(~OLED_F8x16[Char-' '][i+8]);
  }
}

void OLED_ShowStringReverse(uint8_t Line,uint8_t Column,char *String)
{
  uint8_t i;

  for(i=0;String[i]!='\0';i++)
  {
    OLED_ShowCharReverse(Line,Column+i,String[i]);
  }
}