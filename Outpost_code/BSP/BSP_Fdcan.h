#ifndef _BSP_FDCAN_H
#define _BSP_FDCAN_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "typedef.h"

// 定义 FDCAN 句柄
extern FDCAN_HandleTypeDef hfdcan1; // 云台 FDCAN
extern FDCAN_HandleTypeDef hfdcan2; // 底盘 FDCAN

// CAN ID 定义
#define CAN_CHASSIS_ALL_ID   0x200  // 底盘所有电机的控制ID（广播）
#define CAN_3508_M1_ID       0x201  // 底盘电机1的反馈ID
#define CAN_3508_M2_ID       0x202  // 底盘电机2的反馈ID
#define CAN_3508_M3_ID       0x203  // 底盘电机3的反馈ID
#define CAN_3508_M4_ID       0x204  // 底盘电机4的反馈ID
#define CAN_YAW_MOTOR_ID     0x205  // 云台YAW电机的反馈ID
#define CAN_PIT_MOTOR_ID     0x206  // 云台PITCH电机的反馈ID
#define CAN_TRIGGER_MOTOR_ID 0x207  // 拨弹电机的反馈ID
#define CAN_GIMBAL_ALL_ID    0x1FF  // 云台所有电机的控制ID（广播）

// RM电机数据结构体
typedef struct
{
    uint16_t ecd;            // 编码器值（0-8191）
    int16_t speed_rpm;       // 电机转速（RPM）
    int16_t given_current;   // 电机实际输出电流
    uint8_t temperate;       // 电机温度
    int16_t last_ecd;        // 上一次的编码器值
} motor_measure_t;

// 函数声明
void FDCAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
void FDCAN_cmd_motor_speed(uint8_t motor_id, float v_des);
void J4310_Enable();
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;           /* 保存上次编码器值 */   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);  /* 当前编码器值 */ \
        (ptr)->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);  /* 转速RPM */ \
        (ptr)->given_current = (int16_t)((data)[4] << 8 | (data)[5]); /* 给定电流 */ \
        (ptr)->temperate = (data)[6];           /* 电机温度 */         \
    }
#define MAX_MOTOR_SPEED_RPM  2000.0f
#define MAX_MOTOR_SPEED_RAD  (MAX_MOTOR_SPEED_RPM * 2 * 3.14159265 / 60) 

#endif /* _BSP_FDCAN_H */
