#include "Task_UsartSend.h"
#include "BSP_Test.h"
#include "BSP_CAN.h"
//extern M2006s_t M2006_Array[4];
//extern Omni_Data_t Omni_Data;
extern fp32 INS_angle[3];

/**
  * @brief  串口发送显示数据
  * @param  void
  * @retval void
  */
void Show_Data(void const *argument)
{
    for(;;)
    {
		//unsigned int time = 1;
	
//		uint8_t m[10];
//		sprintf(m,"%f,%f\r\n",Mecanum_Data.Angle_ChassisToCloud,bmi088_real_data.temp);
//		sprintf(m,"%f\r\n",INS_angle[0]);
//		HAL_UART_Transmit(&huart1,m,strlen(m),0xff);
//		vTaskDelay(1);
		
			//遥控器控制模式
//		SBUS_Handle();//处理遥控器数据
	//	vTaskDelay(1000);
		
		//超级电容调试
//		uint8_t data[2];
//		uint16_t Set_Power;
//		Set_Power = 3000;
//		data[0] = Set_Power>>8;
//		data[1] = Set_Power;
//		Can_Fun.CAN_SendData(CAN_SendHandle,&hcan1,CAN_ID_STD,0x210,data);	
//		vTaskDelay(100);
		//FIRE调试模式
//	Fire_Handle();
//		int a,b,c;
//		a=M6020s_Pitch.realAngle;
//		b=M6020s_Pitch.targetSpeed;
//		c=M6020s_Pitch.realSpeed;
////		Fire_Set_Computer_Value(RETURN_RealValue,0x01,&a,1);//传回当前通道的实际值
//		Fire_Set_Computer_Value(RETURN_RealValue,0x01,&a,1);//传回当前通道的实际值
//	 	Fire_Set_Computer_Value(RETURN_RealValue,0x02,&b,1);//传回当前通道的目标值
//		Fire_Set_Computer_Value(RETURN_RealValue,0x03,&c,1);//传回当前通道的实际值
//	 	Fire_Set_Computer_Value(RETURN_Target,0x02,&M6020s_Pitch.targetSpeed,1);//传回当前通道的目标值
//	 	Fire_Set_Computer_Value(RETURN_Target,0x01,&M3508_Array[FricL_Wheel].targetSpeed,1);//传回当前通道的目标值
//		Fire_Set_Computer_Value(RETURN_RealValue,0x02,&b,1);//传回当前通道的实际值
//	//	Fire_Set_Computer_Value(RETURN_Target,0x02,&M3508_Array[FricR_Wheel].targetSpeed,1);//传回当前通道的目标值

//		
//		
//		uint8_t data[8];
//						M3508_Array[FricL_Wheel].targetSpeed = Fric_Data.Required_Speed;
//						M3508_Array[FricR_Wheel].targetSpeed = -1 * Fric_Data.Required_Speed;
//						M3508_Array[FricL_Wheel].outCurrent = Incremental_PID(&M3508_FricL_Pid, 
//															  M3508_Array[FricL_Wheel].targetSpeed, 
//															  M3508_Array[FricL_Wheel].realSpeed);
//						M3508_Array[FricR_Wheel].outCurrent = Incremental_PID(&M3508_FricR_Pid, 
//															  M3508_Array[FricR_Wheel].targetSpeed,
//															  M3508_Array[FricR_Wheel].realSpeed);
//				
//						M3508_FUN.M3508_setCurrent(M3508_Array[FricL_Wheel].outCurrent,
//															M3508_Array[FricR_Wheel].outCurrent,
//															0, 0, data);
//						Can_Fun.CAN_SendData(CAN_SendHandle, &hcan2, CAN_ID_STD, M3508_SENDID_Fric_Dial, data);
//		
		//sprintf((char *)temp1, "RealSpeed = %d\n", M3508_Array[FricL_Wheel].realSpeed);
		//HAL_UART_Transmit(&huart1, temp1, sizeof(temp1), 0xff);

	}
	

}

