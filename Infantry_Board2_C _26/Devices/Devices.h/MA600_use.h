/**
 * @file MA600.h
 * @author xhf
 * @brief 
 * @version 0.1
 * @date 2025-05-06
 * 
 */
#ifndef _MA600_USE_H
#define _MA600_USE_H

#include "MA600_base.h"

//编码器片选引脚设置
#define MA600_CS1_Port GPIOB
#define MA600_CS1_Pin  GPIO_PIN_12
#define MA600_CS2_Port GPIOF
#define MA600_CS2_Pin  GPIO_PIN_0
#define MA600_CS3_Port GPIOF
#define MA600_CS3_Pin  GPIO_PIN_1
#define MA600_CS4_Port GPIOI
#define MA600_CS4_Pin  GPIO_PIN_7

/********全局变量声明*********/
extern hMA600_TypeDef MA600s[4];
extern uint8_t MA600_flag[2];

/********函数声明*********/
void MA600sInit(void);
void setChrysanthemumChains(void);
void MA600s_Read_DaisyChain(void);
void MA600s_Read_Chassis(void);

#endif

