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
#define MA600_CS_Port GPIOB
#define MA600_CS_Pin  GPIO_PIN_12

//初始位置编码器角度值
#define MA600_Roll_InitAngle 34456.0f

/********全局变量声明*********/
extern hMA600_TypeDef MA600s_J1;
/********函数声明*********/
void MA600sInit(void);
void MA600_J1_Location(void);

#endif

