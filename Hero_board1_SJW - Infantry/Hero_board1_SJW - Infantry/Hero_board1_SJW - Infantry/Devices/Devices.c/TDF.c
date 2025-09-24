/**
 * @file TDF.c
 * @author SJW
 * @brief
 * @version .01
 * @date 2025-9-23
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "TDF.h"


/* ----------------------- Internal Data ----------------------------------- */
SBUS_Buffer SBUS;
RC_Ctl_TDF_t RC_Ctrl_TDFData;
uint8_t SBUS_RX_Finish = 0;
uint8_t SBUS_RXBuffer[SBUS_RX_LEN] = {0}; // 接收缓冲
uint8_t SBUS_Rx_Data[25];
TDF_Export_Data_t TDF_Export_Data = TDF_ExportDataGroundInit;

/* ----------------------- Internal Function ----------------------------------- */
void RemoteControl_PC_Update(void);

/**
 * @brief  SBUS初始化，开启接收空闲中断
 * @param  void
 * @retval void
 */
void TDF_Init()
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, SBUS_Rx_Data, sizeof(SBUS_Rx_Data));
}

/**
 * @brief 线性映射函数：将353~1695映射到-1024~1024
 * @param sbus_val: SBUS原始通道值
 * @retval 映射后的数值
 */
static int16_t map_to_1024(int16_t sbus_val)
{
    // 限制输入范围，防止异常值
    if (sbus_val < SBUS_MIN)
        sbus_val = SBUS_MIN;
    if (sbus_val > SBUS_MAX)
        sbus_val = SBUS_MAX;

    // 线性映射公式：y = (x - x_min) * (y_max - y_min)/(x_max - x_min) + y_min
    return (int16_t)((sbus_val - SBUS_MIN) * 2048.0f / (SBUS_MAX - SBUS_MIN) - 1024);
}

/**
 * @brief 5-10通道档位转换：353→1，1024→2，1695→3
 * @param sbus_val: SBUS原始通道值
 * @retval 1/2/3档位
 */
static uint8_t map_to_3levels(int16_t sbus_val)
{
    if (sbus_val <= SBUS_MIN + TOLERANCE)
    {
        return 1;
    }
    else if (sbus_val >= SBUS_MAX - TOLERANCE)
    {
        return 3;
    }
    else if (sbus_val >= MID_VALUE - TOLERANCE && sbus_val <= MID_VALUE + TOLERANCE)
    {
        return 2;
    }
    return 0;
}

/**
 * @brief 11-12通道档位转换：353→1，1695→2
 * @param sbus_val: SBUS原始通道值
 * @retval 1/2档位
 */
static uint8_t map_to_2levels(int16_t sbus_val)
{
    if (sbus_val <= SBUS_MIN + TOLERANCE)
    {
        return 1;
    }
    else if (sbus_val >= SBUS_MAX - TOLERANCE)
    {
        return 2;
    }
    return 0;
}

/**
 * @brief 解析遥控器数据并完成通道映射
 * @param  void
 * @retval void
 */
void TDF_Handle()
{
    if (SBUS_RX_Finish == 1)
    {
        SBUS_RX_Finish = 0;

        // 帧头帧尾校验
        if (SBUS_RXBuffer[0] == 0x0F && SBUS_RXBuffer[24] == 0x00)
        {
            // 1. 解析原始通道数据（保持原解析逻辑）
            SBUS.Ch1 = ((uint16_t)SBUS_RXBuffer[1]) | ((uint16_t)((SBUS_RXBuffer[2] & 0x07) << 8));
            SBUS.Ch2 = ((uint16_t)((SBUS_RXBuffer[2] & 0xf8) >> 3)) | (((uint16_t)(SBUS_RXBuffer[3] & 0x3f)) << 5);
            SBUS.Ch3 = ((uint16_t)((SBUS_RXBuffer[3] & 0xc0) >> 6)) | ((((uint16_t)SBUS_RXBuffer[4]) << 2)) | (((uint16_t)(SBUS_RXBuffer[5] & 0x01)) << 10);
            SBUS.Ch4 = ((uint16_t)((SBUS_RXBuffer[5] & 0xfe) >> 1)) | (((uint16_t)(SBUS_RXBuffer[6] & 0x0f)) << 7);
            SBUS.Ch5 = ((uint16_t)((SBUS_RXBuffer[6] & 0xf0) >> 4)) | (((uint16_t)(SBUS_RXBuffer[7] & 0x7f)) << 4);
            SBUS.Ch6 = ((uint16_t)((SBUS_RXBuffer[7] & 0x80) >> 7)) | (((uint16_t)SBUS_RXBuffer[8]) << 1) | (((uint16_t)(SBUS_RXBuffer[9] & 0x03)) << 9);
            SBUS.Ch7 = ((uint16_t)((SBUS_RXBuffer[9] & 0xfc) >> 2)) | (((uint16_t)(SBUS_RXBuffer[10] & 0x1f)) << 6);
            SBUS.Ch8 = ((uint16_t)((SBUS_RXBuffer[10] & 0xe0) >> 5)) | (((uint16_t)(SBUS_RXBuffer[11])) << 3);
            SBUS.Ch9 = ((uint16_t)SBUS_RXBuffer[12]) | (((uint16_t)(SBUS_RXBuffer[13] & 0x07)) << 8);
            SBUS.Ch10 = ((uint16_t)((SBUS_RXBuffer[13] & 0xf8) >> 3)) | (((uint16_t)(SBUS_RXBuffer[14] & 0x3f)) << 5);
            SBUS.Ch11 = ((uint16_t)((SBUS_RXBuffer[14] & 0xc0) >> 6)) | (((uint16_t)SBUS_RXBuffer[15]) << 2) | (((uint16_t)(SBUS_RXBuffer[16] & 0x01)) << 10);
            SBUS.Ch12 = ((uint16_t)((SBUS_RXBuffer[16] & 0xfe) >> 1)) | (((uint16_t)(SBUS_RXBuffer[17] & 0x0f)) << 7);
            SBUS.Ch13 = ((uint16_t)((SBUS_RXBuffer[17] & 0xf0) >> 4)) | (((uint16_t)(SBUS_RXBuffer[18] & 0x7f)) << 4);
            SBUS.Ch14 = ((uint16_t)((SBUS_RXBuffer[18] & 0x80) >> 7)) | (((uint16_t)SBUS_RXBuffer[19]) << 1) | (((uint16_t)(SBUS_RXBuffer[20] & 0x03)) << 9);
            SBUS.Ch15 = ((uint16_t)((SBUS_RXBuffer[20] & 0xfc) >> 2)) | (((uint16_t)(SBUS_RXBuffer[21] & 0x1f)) << 6);
            SBUS.Ch16 = ((uint16_t)((SBUS_RXBuffer[21] & 0xe0) >> 5)) | (((uint16_t)SBUS_RXBuffer[22]) << 3);

            // 2. 通道数据映射
						RC_Ctrl_TDFData.rc.Ch1 = map_to_1024(SBUS.Ch1);
            RC_Ctrl_TDFData.rc.Ch2 = map_to_1024(SBUS.Ch2);
            RC_Ctrl_TDFData.rc.Ch3 = map_to_1024(SBUS.Ch3);
            RC_Ctrl_TDFData.rc.Ch4 = map_to_1024(SBUS.Ch4);
            RC_Ctrl_TDFData.rc.Ch5 = map_to_3levels(SBUS.Ch5);
            RC_Ctrl_TDFData.rc.Ch6 = map_to_3levels(SBUS.Ch6);
            RC_Ctrl_TDFData.rc.Ch7 = map_to_3levels(SBUS.Ch7);
            RC_Ctrl_TDFData.rc.Ch8 = map_to_3levels(SBUS.Ch8);
            RC_Ctrl_TDFData.rc.Ch9 = map_to_3levels(SBUS.Ch9);
            RC_Ctrl_TDFData.rc.Ch10 = map_to_2levels(SBUS.Ch10);
            RC_Ctrl_TDFData.rc.Ch11 = map_to_3levels(SBUS.Ch11);
            RC_Ctrl_TDFData.rc.Ch12 = map_to_2levels(SBUS.Ch12);
            RC_Ctrl_TDFData.rc.Ch13 = map_to_1024(SBUS.Ch13);
            RC_Ctrl_TDFData.rc.Ch14 = map_to_1024(SBUS.Ch14);
            RC_Ctrl_TDFData.rc.Ch15 = map_to_1024(SBUS.Ch15);
            RC_Ctrl_TDFData.rc.Ch16 = map_to_1024(SBUS.Ch16);

            /**************************** control code ****************************/
            ControlMes.shoot_state = RC_Ctrl_TDFData.rc.Ch7;
            /*发射信息处理*/
            if (ControlMes.shoot_state == RC_UP)
            {
                Dial_Data.Shoot_Mode = Single_Shoot;
                Shoot_Data.Shoot_Switch = 1;
            }
            else if (ControlMes.shoot_state == RC_MID)
            {
                Dial_Data.Shoot_Mode = No_Shoot;
                Shoot_Data.Shoot_Switch = 0;
            }
            else if (ControlMes.shoot_state == RC_DOWN)
            {
                Dial_Data.Shoot_Mode = Continuous_Shoot;
                Shoot_Data.Shoot_Switch = 1;
            }
            /*通道5*/
            /*中正常遥控；下自瞄；上键鼠*/
            if (RC_Ctrl_TDFData.rc.Ch5 == RC_MID)
            {
                /******************************遥控器数值传递******************************/
                // 底盘运动控制
                ControlMes.x_velocity = -RC_Ctrl_TDFData.rc.Ch2; // 左手上下
                ControlMes.y_velocity = -RC_Ctrl_TDFData.rc.Ch4; // 左手左右
                // 云台运动控制
                ControlMes.AutoAimFlag = 0;
                ControlMes.pitch_velocity = RC_Ctrl_TDFData.rc.Ch3;       // 右手上下
                ControlMes.yaw_velocity = RC_Ctrl_TDFData.rc.Ch1;         // 右手左右
                ControlMes.z_rotation_velocity = RC_Ctrl_TDFData.rc.Ch13; // 滑轮左右
                ControlMes.yaw_position = Auto_Aim_Yaw;
                // 发射状态设置（右拨杆）（UP 单发模式（顺时针右滚动滚轮拨弹，逆时针左退弹）；MID 禁止发射 ；DOWN 部署模式）

                if (ControlMes.shoot_state == RC_UP) // 单发模式
                {
                    Fric_Data.Fric_Switch = Fric_On;
                    ControlMes.fric_Flag = 1;
                    if (RC_Ctrl_TDFData.rc.Ch8 == RC_UP)
                    {
                        Dial_Data.Dial_Switch = Dial_On;
                    }
                    else if (RC_Ctrl_TDFData.rc.Ch8 == RC_MID)
                    {
                        Dial_Data.Dial_Switch = Dial_Off;
                    }
                    else if (RC_Ctrl_TDFData.rc.Ch8 == RC_DOWN)
                    {
                        Dial_Data.Dial_Switch = Dial_Back;
                    }
                }

                else if (ControlMes.shoot_state == RC_MID) // 正常模式
                {
                    Dial_Data.Dial_Switch = Dial_Off;
                    Dial_Data.Speed_Dial = 0;
                    Dial_Data.Number_ToBeFired = 0;
                    Fric_Data.Fric_Switch = Fric_Off;
                    ControlMes.fric_Flag = 0;
                    ControlMes.modelFlag = 0;
                }

                else if (ControlMes.shoot_state == RC_DOWN) // （步兵连发模式）(英雄部署模式待完善)
                {
                    Fric_Data.Fric_Switch = Fric_On;
                    ControlMes.fric_Flag = 1;
                    if (RC_Ctrl_TDFData.rc.Ch8 == RC_UP)
                    {
                        Dial_Data.Dial_Switch = Dial_On;
                    }
                    else if (RC_Ctrl_TDFData.rc.Ch8 == RC_MID)
                    {
                        Dial_Data.Dial_Switch = Dial_Off;
                    }
                    else if (RC_Ctrl_TDFData.rc.Ch8 == RC_DOWN)
                    {
                        Dial_Data.Dial_Switch = Dial_Back;
                    }
                }
            }

            // 自瞄模式
            else if (RC_Ctrl_TDFData.rc.Ch5 == RC_DOWN)
            {
                /******************************遥控器数值传递******************************/
                // 底盘运动控制
                ControlMes.x_velocity = -RC_Ctrl_TDFData.rc.Ch2; // 左手上下
                ControlMes.y_velocity = -RC_Ctrl_TDFData.rc.Ch4; // 左手左右
                // 自瞄云台运动控制（这里添加额外的遥控器控制是为了补偿自瞄精度，自己用遥控器微调一下辅助瞄准）
                ControlMes.pitch_velocity = RC_Ctrl_TDFData.rc.Ch3 * 0.2; // 右手上下
                ControlMes.yaw_velocity = RC_Ctrl_TDFData.rc.Ch1;         // 右手左右
                ControlMes.z_rotation_velocity = RC_Ctrl_TDFData.rc.Ch13; // 滑轮左右
                ControlMes.AutoAimFlag = 1;
                // 上位机视觉得到的正负与电机的正负是一样的，通信两边的正负号要对好
                ControlMes.yaw_position = Auto_Aim_Yaw;
                Cloud.AutoAim_Pitch = Auto_Aim_Pitch;

                /*************************发射状态设置*********************/
                // （UP 单发模式 ； MID 禁止发射；DOWN 部署模式）
                if (ControlMes.shoot_state == RC_UP)
                {
                    Fric_Data.Fric_Switch = Fric_On;
                    ControlMes.fric_Flag = 1;
                    if (RC_Ctrl_TDFData.rc.Ch8 == RC_UP)
                    {
                        Dial_Data.Dial_Switch = Dial_On;
                    }
                    else if (RC_Ctrl_TDFData.rc.Ch8 == RC_MID)
                    {
                        Dial_Data.Dial_Switch = Dial_Off;
                    }
                    else if (RC_Ctrl_TDFData.rc.Ch8 == RC_DOWN)
                    {
                        Dial_Data.Dial_Switch = Dial_Back;
                    }
                }

                else if (ControlMes.shoot_state == RC_MID)
                {
                    Dial_Data.Dial_Switch = Dial_Off;
                    Dial_Data.Speed_Dial = 0;
                    Dial_Data.Number_ToBeFired = 0;
                    Fric_Data.Fric_Switch = Fric_Off;
                    ControlMes.fric_Flag = 0;
                    ControlMes.modelFlag = 0;
                }

                else if (ControlMes.shoot_state == RC_DOWN) // （步兵连发模式）(英雄部署模式待完善)
                {
                    Fric_Data.Fric_Switch = Fric_On;
                    ControlMes.fric_Flag = 1;
                    if (RC_Ctrl_TDFData.rc.Ch8 == RC_UP)
                    {
                        Dial_Data.Dial_Switch = Dial_On;
                    }
                    else if (RC_Ctrl_TDFData.rc.Ch8 == RC_MID)
                    {
                        Dial_Data.Dial_Switch = Dial_Off;
                    }
                    else if (RC_Ctrl_TDFData.rc.Ch8 == RC_DOWN)
                    {
                        Dial_Data.Dial_Switch = Dial_Back;
                    }
                }
            }

            else if (RC_Ctrl_TDFData.rc.Ch5 == RC_UP)
            {
                /*键鼠数据处理*/
                KeyMouseFlag_Update();
                RemoteControl_PC_Update();
            }

            else
            {
                ControlMes.AutoAimFlag = 0;
                ControlMes.x_velocity = 0;          // 左手上下
                ControlMes.y_velocity = 0;          // 左手左右
                ControlMes.z_rotation_velocity = 0; // 右手上下
                ControlMes.yaw_velocity = 0;
                ControlMes.pitch_velocity = 0;
                ControlMes.shoot_state = RC_MID;
            }
        }
    }
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, SBUS_Rx_Data, sizeof(SBUS_Rx_Data));
    // 用board1 CAN2发送给board2。
    Board1_FUN.Board1_To_2();
}
