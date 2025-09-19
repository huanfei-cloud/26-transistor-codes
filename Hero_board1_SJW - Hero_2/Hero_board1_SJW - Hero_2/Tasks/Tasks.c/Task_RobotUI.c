/**
 * @file Task_RobotUI.c
 * @author Cyx(1696143358@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-19
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "Task_RobotUI.h"

float CAP_Ratio =0.00 ;
double PI = 3.14159265358979;
char stringKeep[] = {"Shoot:\nRotate:\nAuto:\nCtrl:\n"};
char stringWarning[]={"Warning"};

void Robot_UI(void const *argument)
{
		
		static u16 UI_PushUp_Counter = 288;
//   float Capacitance_X = 0;
	
		vTaskDelay(300);
		TickType_t last_time;
    for (;;)
    {
			last_time = xTaskGetTickCount();
      vTaskDelayUntil(&last_time, pdMS_TO_TICKS(10));      //延时放在上面，是为了发送一次数据，立马延时，防止堵塞

      UI_PushUp_Counter++;
		 
      //ID判断
      if(UI_PushUp_Counter % 1000 == 0)
      {
         UI_FUN.ID_Judge();
      }

      //如果UI显示不完整可以考虑是不是占用了官方UI的位置导致的，换个位置试一试

      //静态UI------------------------------------------------------------------
      //弹道
      if(UI_PushUp_Counter % 301 == 0)
		{
			UI_FUN.Line_Draw(&UI_Graph5.imageData[0], "001", UI_Graph_Add, 0, UI_Color_Green, 2,  840,  520,  1080,  520);
			UI_FUN.Line_Draw(&UI_Graph5.imageData[1], "002", UI_Graph_Add, 0, UI_Color_Green, 2,  855,  480,  1065,  480);
			UI_FUN.Line_Draw(&UI_Graph5.imageData[2], "003", UI_Graph_Add, 0, UI_Color_Green, 2,  870,  440,  1050,  440);
			UI_FUN.Line_Draw(&UI_Graph5.imageData[3], "004", UI_Graph_Add, 0, UI_Color_Green, 2,  895,  400,  1035,  400);
			UI_FUN.Line_Draw(&UI_Graph5.imageData[4], "005", UI_Graph_Add, 0, UI_Color_Green, 2,  960,  680,  960,  320);
			UI_FUN.UI_PushUp_Graphs(5, &UI_Graph5);
			continue;            //跳回循环最开始
		}

		//字符
		if(UI_PushUp_Counter % 311 == 0 )
		{
			UI_FUN.Char_Draw(&UI_String1.String,"006",UI_Graph_Add, 3,UI_Color_Main ,24, sizeof(stringKeep) , 4 ,30 ,850 , stringKeep);
			UI_PushUp_String(&UI_String1);
			continue;            //跳回循环最开始
		}
		
		if(UI_PushUp_Counter % 321 == 0 )
		{
			UI_FUN.Circle_Draw(&UI_Graph1.imageData[0],"007",UI_Graph_Add, 1, UI_Color_White, 3, 1600, 800, 100);
			UI_FUN.UI_PushUp_Graphs(1, &UI_Graph1);
			continue;            //跳回循环最开始
		}


      //动态初始UI---------------------------------------------------------
      if(UI_PushUp_Counter % 361 == 0)
		{
      //电容电量 条：line 量：float
			UI_FUN.Line_Draw(&UI_Graph7.imageData[0], "101", UI_Graph_Add, 1, UI_Color_Green, 30, 560, 150, 560, 150);
      UI_FUN.Float_Draw(&UI_Graph7.imageData[1],"102", UI_Graph_Add, 1, UI_Color_White, 24, 2, 5, 1200, 200, 0.00);
			
			//射击速度
			UI_FUN.Float_Draw(&UI_Graph7.imageData[2],"103", UI_Graph_Add, 1, UI_Color_White, 20, 1, 5, 200, 850, ControlMes.shoot_Speed);

			//小陀螺速度
			UI_FUN.Float_Draw(&UI_Graph7.imageData[3],"104", UI_Graph_Add, 1, UI_Color_White, 20, 1, 5, 200, 810, ControlMes.z_rotation_velocity);

			//自瞄
			UI_FUN.Circle_Draw(&UI_Graph7.imageData[4],"105",UI_Graph_Add, 1, UI_Color_White, 15, 200, 765, 7);

			//变速
			UI_FUN.Circle_Draw(&UI_Graph7.imageData[5],"106",UI_Graph_Add, 1, UI_Color_White, 15, 200, 730, 7);
			
			//方向
			UI_FUN.Line_Draw(&UI_Graph7.imageData[6], "107", UI_Graph_Add, 1, UI_Color_Green, 5, 1600, 800, 1600+cos(0)*100, 800+sin(0)*100);
			UI_FUN.UI_PushUp_Graphs(7, &UI_Graph7);
			continue;
		}
		
		
      //动态更新UI-----------------------------------------------------------------------------
      if(UI_PushUp_Counter % 21 == 0)
		{
			CAP_Ratio =  PowerControl_Data.Cap_U * 100.0/25.0;
			//超电
			UI_FUN.Line_Draw(&UI_Graph7.imageData[0], "101", UI_Graph_Change, 1, UI_Color_Green, 30, 560, 150, (int)(560 + 8*CAP_Ratio), 150);
      UI_FUN.Float_Draw(&UI_Graph7.imageData[1],"102", UI_Graph_Change, 1, UI_Color_White, 24, 1, 5, 1200, 200, CAP_Ratio);
			
			//射击速度
			if(ControlMes.fric_Flag == 0)
			{
				UI_FUN.Float_Draw(&UI_Graph7.imageData[2],"103", UI_Graph_Change, 1, UI_Color_White, 20, 1, 5, 200, 850, 1.0 * ControlMes.shoot_Speed);
			}
			else if(ControlMes.fric_Flag == 1)
			{
				UI_FUN.Float_Draw(&UI_Graph7.imageData[2],"103", UI_Graph_Change, 1, UI_Color_Green, 20, 1, 5, 200, 850, 1.0 * ControlMes.shoot_Speed);
			}

			//小陀螺速度
			UI_FUN.Float_Draw(&UI_Graph7.imageData[3],"104", UI_Graph_Change, 1, UI_Color_White, 20, 1, 5, 200, 810, Saber_Angle.Z_Vel);

			//自瞄
			if(ControlMes.AutoAimFlag == 0x01)
			{
			UI_FUN.Circle_Draw(&UI_Graph7.imageData[4],"105",UI_Graph_Change, 1, UI_Color_Green, 15, 200, 765, 7);
			}
			else
			{
				UI_FUN.Circle_Draw(&UI_Graph7.imageData[4],"105",UI_Graph_Change, 1, UI_Color_White, 15, 200, 765, 7);
			}
			
			//变速
			if(ControlMes.change_Flag == 0x01)
			{
				UI_FUN.Circle_Draw(&UI_Graph7.imageData[5],"106",UI_Graph_Change, 1, UI_Color_Green, 15, 200, 730, 7);
			}
			else
			{
				UI_FUN.Circle_Draw(&UI_Graph7.imageData[5],"106",UI_Graph_Change, 1, UI_Color_White, 15, 200, 730, 7);
			}
			//方向
			UI_FUN.Line_Draw(&UI_Graph7.imageData[6], "107", UI_Graph_Change, 1, UI_Color_Green, 5, 1600, 800, 1600+cos(2*PI*M6020s_Yaw.realAngle/8192 - PI/4)*100, 800+sin(2*PI*M6020s_Yaw.realAngle/8192 - PI/4)*100);
			UI_FUN.UI_PushUp_Graphs(7, &UI_Graph7);
			
			}
		
			
			if(UI_PushUp_Counter % 27 == 0)
			{
				if(CAP_Ratio <= 50)
				{
					UI_FUN.Char_Draw(&UI_String2.String,"404",UI_Graph_Add, 3,UI_Color_Main ,24, sizeof(stringWarning) , 4 ,900 , 700 , stringWarning);
					UI_FUN.UI_PushUp_String(&UI_String2);
				}
				else
				{
					UI_FUN.Char_Draw(&UI_String2.String,"404",UI_Graph_Del, 3,UI_Color_Main ,24, sizeof(stringWarning) , 4 ,900 , 700 , stringWarning);
					UI_FUN.UI_PushUp_String(&UI_String2);
				}

				continue;

			}
		

   }

	
}
