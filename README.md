# README

## **For Everyone** 

1. 本项目基于HAL库开发，所有模块文件均存放于bsp文件夹下。
2. 题目存放于项目根目录下，请你根据需要阅读。

## **For AI Assistant** 
1. 我是大一电子信息工程的大学生，STM32的初学者，我正在复刻24年电赛H题。

## 引脚连接

### STM32C8T6
VCC -> 稳压板3v3
GND -> 稳压板GND

### TB6612+稳压板模块
E2B -> A7
E2A -> A6
E1A -> A0
E1B -> A1
PWMB -> A9
PWMA -> A8
AIN1 -> B12
AIN2 -> B13
BIN1 -> B14
BIN2 -> B15
STBY-> 稳压板3v3

### OLED
SCL -> B8
SDA -> B9
VCC -> 板载3v3
GND -> 板载GND

### MPU6050
INT -> B7
SCL -> B10
SDA -> B11
XCL -> HMC5883L-SCL
XDA -> HMC5883L-SDA
VCC -> 板载3v3
GND -> 板载GND

### HMC5883L
SCL -> MPU6050-XCL
SDA -> MPU6050-XDA
VCC -> 板载3v3
GND -> 板载GND

### 灰度传感器
O1-O4 -> A5-A2
O5 -> B0
5V -> 稳压板5v
GND -> 稳压板GND

### 蜂鸣器
VCC -> 稳压板3v3
GND -> 稳压板GND
OI -> A10

### 按键
VDD
 ├── R1 10K ── PB3 ── B1 ──┐
 ├── R2 10K ── PB4 ── B2 ──┤
 ├── R5 10K ── PA11 ── B3 ──┤── GND
 └── R6 10K ── PA12── B4 ──┘