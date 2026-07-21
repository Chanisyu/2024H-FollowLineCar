/**
 * @file    OLED.h
 * @brief   OLED 显示驱动接口，提供字符、数字及反白显示功能。
 */

#ifndef __OLED_H
#define __OLED_H

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line,uint8_t Column,char Char);
void OLED_ShowString(uint8_t Line,uint8_t Column,char *String);
void OLED_ShowNum(uint8_t Line,uint8_t Column,uint32_t Number,uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line,uint8_t Column,int32_t Number,uint8_t Length);
void OLED_ShowHexNum(uint8_t Line,uint8_t Column,uint32_t Number,uint8_t Length);
void OLED_ShowBinNum(uint8_t Line,uint8_t Column,uint32_t Number,uint8_t Length);

// 行操作接口：Line 取 1~4 表示显示行，Column 取 1~16 表示起始字符列。
void OLED_ClearLine(uint8_t Line);
void OLED_FillLine(uint8_t Line);
void OLED_ShowCharReverse(uint8_t Line,uint8_t Column,char Char);
void OLED_ShowStringReverse(uint8_t Line,uint8_t Column,char *String);

#endif
