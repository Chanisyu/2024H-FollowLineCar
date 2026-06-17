# AGENT.md

本文件用于约束 AI 在本项目中生成、修改、重构 C / 嵌入式代码时的注释风格。

目标：
让 AI 写出的代码注释保持**中文、结构清晰、面向硬件逻辑、解释充分、便于调试和交接**，风格参考当前项目中的 `gray_track.c`。

------

## 0. 规则优先级与示例说明

本文件同时约束两类内容：

1. **注释内容规范**：说明注释应该解释什么、写到什么程度。
2. **C 代码格式规范**：说明 C 代码中的括号、空格、缩进、换行等格式要求。

当注释示例和 C 代码格式之间出现理解冲突时，按以下原则处理：

- 注释要表达的硬件行为、协议流程、算法含义，以前文各注释章节为准。
- C 代码的括号、空格、缩进、换行、参数逗号等格式，必须以本文末尾 **C code style（C 代码风格）** 为准。
- 生成或修改代码时，既要保持注释解释充分，也要严格匹配项目现有 Keil-style 紧凑格式。
- 修改已有文件时，优先保持 nearby code 的既有风格，不要为了套用示例而重排无关代码。

------

## 1. 总体注释风格

所有代码注释必须使用中文，使用GB2312编码，除非是芯片手册、寄存器名、函数名、宏名、协议名等必须保留英文的内容。

注释应做到：

- 解释“为什么这样写”，而不是只翻译代码本身
- 明确硬件引脚、时序、协议、单位、方向、取值含义
- 对控制算法、状态机、通信协议等逻辑写出完整流程
- 对关键参数写清作用、调参方向和副作用
- 对边界情况和保护逻辑写清触发条件与处理方式

禁止出现以下低质量注释：

```c
i++;        // i 加一
flag=1;    // 设置 flag 为 1
```

应改为：

```c
i++;        // 进入下一个采样通道
flag=1;    // 标记本轮数据已接收完成，主循环可以开始处理
```

------

## 2. 文件头注释格式

每个 `.c` 文件开头必须包含文件说明，使用如下格式：

```c
/**
 * @file    xxx.c
 * @brief   模块功能简述
 *
 * 文件结构：
 *   1. xxx_init()       — 初始化硬件外设 / GPIO / 定时器
 *   2. xxx_read()       — 读取传感器 / 通信数据
 *   3. xxx_control()    — 控制算法 / 状态机 / 输出执行
 */
```

要求：

- `@brief` 用一句话说明该文件负责什么
- “文件结构”按代码中的主要函数顺序列出
- 每一项后面用 `—` 简短说明职责
- 如果文件包含协议、控制算法、硬件驱动，应在文件头中体现

------

## 3. 大段功能区注释格式

主要模块、复杂函数或重要功能区前应使用分隔线注释。普通 helper 函数、简单 getter / setter、显而易见的局部代码块不强制使用大段分隔线，避免为了注释而注释。

推荐格式如下：

```c
// ================================================================
// 1. 模块名称
//
//    简要说明该模块解决什么问题
//    如果涉及硬件，应写清引脚、方向、电平、时序、单位
//    如果涉及算法，应写清输入、处理流程、输出含义
// ================================================================
```

示例：

```c
// ================================================================
// 2. 串行读取 8 路灰度传感器
//
//    引脚：
//      PB1 (CLK) — 推挽输出，MCU 发出时钟
//      PB0 (DAT) — 上拉输入，读取传感器数据
//
//    协议：
//      ① CLK 拉低 → 传感器输出当前位
//      ② MCU 读取 DAT
//      ③ CLK 拉高 → 传感器移位，准备下一位
//
//    返回数据：
//      bit0 = 探头1，bit7 = 探头8
//      bit=0 → 黑线，bit=1 → 白纸
// ================================================================
```

------

## 4. 函数内部步骤注释

函数内部如果逻辑超过 3 个明确阶段，必须使用编号步骤注释：

```c
// ------- 1. 初始化参数 -------
// ------- 2. 读取输入数据 -------
// ------- 3. 判断边界情况 -------
// ------- 4. 计算控制量 -------
// ------- 5. 输出执行结果 -------
```

要求：

- 步骤编号从 1 开始
- 每个步骤名称简短明确
- 步骤注释用于划分逻辑，不要滥用
- 状态机、控制算法、通信解析、传感器融合等必须使用步骤注释
- 简单赋值、普通循环、局部变量准备等即使行数较多，也不应强行拆成编号步骤

------

## 5. 硬件相关注释要求

涉及 GPIO、UART、I2C、SPI、ADC、PWM、定时器、中断时，必须写清楚：

- 使用的引脚
- 输入 / 输出方向
- 高低电平含义
- 是否上拉 / 下拉 / 开漏 / 推挽
- 时序约束
- 单位，例如 `us`、`ms`、`Hz`、`tick`、`rpm`
- 与硬件手册相关的特殊要求

示例：

```c
// PB1 (CLK) — 推挽输出，MCU 主动产生时钟
// PB0 (DAT) — 上拉输入，传感器为开漏输出，低电平表示有效
```

涉及延时必须说明原因：

```c
Delay_us(5);  // 手册要求 CLK 高电平保持 ≥5us，保证传感器完成移位
```

------

## 6. 协议 / 时序注释要求

串行协议、通信协议、传感器读数流程必须写成“动作 → 结果”的形式。

推荐格式：

```c
// ① 拉低 CLK → 从机把当前 bit 放到 DAT 上
// ② MCU 读取 DAT → 得到本轮数据
// ③ 拉高 CLK → 从机锁存并准备下一 bit
```

不要只写：

```c
// 拉低时钟
// 读取数据
// 拉高时钟
```

应说明每一步对硬件的影响。

------

## 7. 数据位、逻辑电平、方向必须说明

凡是涉及 bit、mask、方向、左右轮、正负号的逻辑，必须明确说明。

示例：

```c
// raw: bit0 = 最左侧探头，bit7 = 最右侧探头
// 原始数据中 bit=0 表示黑线，bit=1 表示白纸
// 反转后 s=1 表示压线，便于后续 active_count 统计
```

涉及方向时必须写清正负含义：

```c
// error<0 → 黑线偏左，需要向左修正
// error>0 → 黑线偏右，需要向右修正
```

涉及电机差速时必须写清物理效果：

```c
// 左轮=base+turn
// 右轮=base-turn
// turn>0 时左轮更快、右轮更慢，小车向右修正
```

------

## 8. 算法注释要求

控制算法、滤波、PID / PD、加权平均、状态机等不能只写公式，必须解释物理意义。

示例：

```c
// P 项：偏差越大，转向越强，用于快速把车拉回中心
// D 项：偏差变化越快，阻尼越强，用于抑制左右震荡
float raw_turn=(error*Kp)+((error-last_error)*Kd);
```

滤波必须说明响应与平滑的关系：

```c
// filter_k 越小越平滑，但响应越慢
// filter_k 越大越灵敏，但更容易抖动
float smooth=last_smooth*(1.0f-filter_k)+raw*filter_k;
```

------

## 9. 参数注释要求

所有可调参数附近必须说明：

- 参数作用
- 增大后会发生什么
- 减小后会发生什么
- 是否可能导致震荡、迟钝、超调等问题

示例：

```c
float Kp=4.8f;        // 比例系数：越大转向越猛，过大会左右震荡
float Kd=1.0f;        // 微分系数：越大阻尼越强，过大会反应迟钝
float filter_k=0.3f;  // 低通滤波系数：越小越平滑，越大响应越快
```

------

## 10. 边界保护注释要求

遇到异常、保护、提前返回时，必须说明：

- 什么情况下触发
- 为什么要保护
- 返回值 / 后续行为由谁处理

示例：

```c
if(active_count==0)
{
  // 全白：没有任何探头检测到黑线，说明小车可能已经脱离赛道
  // 此处不直接转向，返回 0 交给上层决定刹车、搜索还是停车
  return 0;
}
```

------

## 11. 静态变量 / 全局变量注释要求

静态变量和全局变量必须说明“为什么需要跨帧保存”。

示例：

```c
// 静态变量：跨帧保存上一轮状态
// last_error 用于计算 D 项，last_smooth_turn 用于低通滤波
static float last_error=0.0f;
static float last_smooth_turn=0.0f;
```

------

## 12. 行内注释风格

行内注释用于补充该行代码的关键含义。

推荐：

```c
ret|=(1<<i);  // 当前探头为白纸，对应 bit 置 1
```

不推荐：

```c
ret|=(1<<i);  // 设置第 i 位
```

行内注释应对齐整洁，但不要求为了对齐破坏代码可读性。

------

## 13. 示例注释模板

新增传感器读取函数时，建议使用以下结构。此模板已经按项目 Keil-style 紧凑格式书写：

```c
uint8_t sensor_read(void)
{
  // ------- 1. 准备读取 -------
  // 复位数据缓存，确保本轮读取不会受到上一轮残留 bit 影响
  uint8_t data=0;

  // ------- 2. 按协议逐位读取 -------
  for(uint8_t i=0;i<8;i++)
  {
    // 拉低 CLK → 传感器输出当前 bit
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);

    // DAT 为高电平表示无效 / 白色区域，对应 bit 置 1
    if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==GPIO_PIN_SET)  data|=(1<<i);

    // 拉高 CLK → 传感器锁存并准备下一 bit
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
    Delay_us(5);  // 满足手册要求的最小高电平保持时间
  }

  // ------- 3. 结束通信 -------
  // 保持 CLK 低电平，让传感器内部状态机回到初始位置
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);

  return data;
}
```

------

## 14. AI 写代码时必须遵守

AI 在本项目中写代码时必须遵守以下规则：

1. 优先写清楚硬件行为、协议流程、算法含义，而不是机械翻译代码。
2. 重要模块、复杂函数和关键功能区要有功能区注释；普通 helper 函数按需简写。
3. 复杂函数内部要有编号步骤注释；简单线性代码不要为了凑格式而强行编号。
4. 所有可调参数都要说明作用和调参方向。
5. 所有保护逻辑都要说明触发原因和返回含义。
6. 所有正负号、左右方向、高低电平、bit 位映射都要明确说明。
7. 不要写无意义注释，不要为了注释而注释。
8. 注释风格要稳定、清晰、适合嵌入式项目长期维护。
9. C 代码的格式、空格、缩进、括号位置必须遵守本文末尾的 **C code style（C 代码风格）**。

------

## 15. 注释密度原则

本项目允许较高注释密度，尤其是以下场景：

- 传感器驱动
- 通信协议
- 中断服务函数
- 定时器 / PWM / 编码器
- PID / PD / 滤波算法
- 状态机
- 安全保护逻辑
- 调参相关代码

对于非常简单的赋值、普通循环、显而易见的语句，可以少写或不写注释。

原则是：

> 注释应该帮助后来的人理解硬件、时序、算法和调试思路，而不是重复 C 语言语法。

当“复杂逻辑需要步骤注释”和“简单代码避免过度注释”看起来冲突时，按以下判断：

- 如果代码包含状态切换、异常分支、硬件时序、通信解析、控制算法等，应使用步骤注释。
- 如果代码只是连续赋值、普通遍历、变量搬运、简单封装，即使行数较多，也不必强行使用步骤注释。
- 注释密度应服务于调试和交接，不应干扰阅读代码主流程。

------

## 16. C code style（C 代码风格）

这是一个 Keil/STM32 embedded C project。

在生成或修改 C code 时，请严格遵循以下紧凑的 Keil-style formatting。

Rules（规则）：

1. 使用 Allman braces。Opening braces 必须另起一行。

Correct：

```c
if(angle_deg<0.0f)
{
  motor->busy=1;
}
```

Incorrect：

```c
if (angle_deg < 0.0f) {
    motor->busy = 1;
}
```

2. 不要在 control keywords 后添加空格。

Correct：

```c
if(angle_deg<0.0f)
while(motor->busy)
for(i=0;i<10;i++)
```

Incorrect：

```c
if (angle_deg < 0.0f)
while (motor->busy)
for (i = 0; i < 10; i++)
```

3. 不要在 operators 两侧添加空格。

Correct：

```c
motor->busy=1;
angle_deg=-angle_deg;
motor->step_count=0;
if(angle_deg<0.0f)
```

Incorrect：

```c
motor->busy = 1;
angle_deg = -angle_deg;
motor->step_count = 0;
if(angle_deg < 0.0f)
```

4. 不要在 function arguments 或 parameter lists 的逗号后添加空格。

Correct：

```c
void Stepper_MoveAngle(StepperMotor_t *motor,float angle_deg,float rpm,uint32_t ramp_steps)
HAL_GPIO_WritePin(DIR1_PORT,DIR1_PIN,GPIO_PIN_RESET);
```

Incorrect：

```c
void Stepper_MoveAngle(StepperMotor_t *motor, float angle_deg, float rpm, uint32_t ramp_steps)
HAL_GPIO_WritePin(DIR1_PORT, DIR1_PIN, GPIO_PIN_RESET);
```

5. 使用 two-space indentation，不要使用 tab indentation，也不要使用 4-space indentation。

Correct：

```c
if(angle_deg<0.0f)
{
  motor->busy=1;
}
```

Incorrect：

```c
if(angle_deg<0.0f)
{
	motor->busy=1;
}
```

6. 如果 `if()` body 只包含一个 simple statement，请写在同一行。closing parenthesis 和 statement 之间必须正好有两个空格。

Correct：

```c
if(angle_deg<0.0f)  motor->busy=1;
```

Incorrect：

```c
if(angle_deg<0.0f)
{
  motor->busy=1;
}
```

Incorrect：

```c
if(angle_deg<0.0f) motor->busy=1;
```

7. 如果 `if()` body 包含多条 statements，请使用 Allman braces。

Correct：

```c
if(angle_deg<0.0f)
{
  HAL_GPIO_WritePin(DIR1_PORT,DIR1_PIN,GPIO_PIN_RESET);
  angle_deg=-angle_deg;
}
```

8. 保留 nearby code 的 style。
9. 不要 reformat unrelated existing code。
10. 不要使用 Google、LLVM、K&R、clang-format default，或 modern spaced C style。
11. 返回 code 时，请匹配以下 sample style：

```c
void Stepper_MoveAngle(StepperMotor_t *motor,float angle_deg,float rpm,uint32_t ramp_steps)
{
  motor->busy=1;
  if(angle_deg<0.0f)
  {
    HAL_GPIO_WritePin(DIR1_PORT,DIR1_PIN,GPIO_PIN_RESET);
    angle_deg=-angle_deg;
  }
  else
  {
    HAL_GPIO_WritePin(DIR1_PORT,DIR1_PIN,GPIO_PIN_SET);
  }

  motor->step_target=(uint32_t)(angle_deg*MOTOR_STEPS_PER_REV/360.0f+0.5f);
  motor->step_count=0;
}
```
