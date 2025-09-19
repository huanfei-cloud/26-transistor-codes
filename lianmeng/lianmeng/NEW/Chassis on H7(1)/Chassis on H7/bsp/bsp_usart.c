#include "bsp_usart.h"
#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "DT7.h"
#include "IMU.h"
#include "Protocol_Judgement.h"
struct Struct_UART_Manage_Object UART1_Manage_Object = {0};
struct Struct_UART_Manage_Object UART6_Manage_Object = {0};
uint8_t Count = 0;
uint8_t last_rsnum = 0;
uint8_t rsimu_flag = 0;
uint8_t rsacc_flag = 0;
int rs_imutype =0;
int rs_ahrstype =0;
uint8_t Fd_data[64];
uint8_t Fd_rsimu[64];
uint8_t Fd_rsahrs[56];

// 空闲中断处理sbaer数据
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	// IMU数据
	if (huart->Instance == USART1)
	{
        Fd_data[Count] = IMU_Receive;  // 将串口数据填入数组
        if (((last_rsnum == FRAME_END) && (IMU_Receive == FRAME_HEAD)) || Count > 0) {
            Count++;
            if ((Fd_data[1] == TYPE_IMU) && (Fd_data[2] == IMU_LEN)) {
                rsimu_flag = 1;
            }
            if ((Fd_data[1] == TYPE_AHRS) && (Fd_data[2] == AHRS_LEN)) {
                rsacc_flag = 1;
            }
        } else {
            Count = 0;
        }
        last_rsnum = IMU_Receive;

        if (rsimu_flag == 1 && Count == IMU_RS) {  // 保存 IMU 数据
            Count = 0;
            rsimu_flag = 0;
            rs_imutype = 1;
            if (Fd_data[IMU_RS - 1] == FRAME_END) {  // 帧尾校验
                memcpy(Fd_rsimu, Fd_data, sizeof(Fd_data));
            }
        }

        if (rsacc_flag == 1 && Count == AHRS_RS) {  // 保存 AHRS 数据
            Count = 0;
            rsacc_flag = 0;
            rs_ahrstype = 1;
            if (Fd_data[AHRS_RS - 1] == FRAME_END) {
                memcpy(Fd_rsahrs, Fd_data, sizeof(Fd_data));
            }
            for (int i = 0; i < sizeof(Fd_data); i++) Fd_data[i] = 0;  // 清空数据数组
        }
	
   }
	
	 	// 遥控器数据
	if (huart->Instance == USART2)
	{
		DT7_RX_Finish = 1; // 已接受完一包数据
	}

	
	if (huart->Instance == USART3)
	{
		JudgeSystem_Handler(&huart3);
	}
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef*huart) //接收回调函数
//{
//	if(huart->Instance == USART1)
//	{
//		 for (uint16_t i = 0; i < 100; i++)
//     {
//         CopeCmdData(IMU_Buffer[i]);  // 逐个处理接收到的字节
//     }
//		 HAL_UART_Receive_IT(&huart1,IMU_Buffer,sizeof(IMU_Buffer));
//	}
//}





