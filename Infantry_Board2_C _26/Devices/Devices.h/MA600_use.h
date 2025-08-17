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

#define MA600_CS_Port GPIOB
#define MA600_CS_Pin  GPIO_PIN_12

/********全局变量声明*********/
extern hMA600_TypeDef MA600s[4];
extern uint8_t MA600_flag[2];
/********函数声明*********/
void MA600sInit(void);
void setChrysanthemumChains(void);
void MA600s_Read_DaisyChain(void);

#endif

