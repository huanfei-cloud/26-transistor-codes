#include "DT7.h"
#include "Dial.h"
#include "shoot.h"
#include "Cloud_Control.h"
#include "SpeedRamp.h"
#include "UI.h"

//底盘遥控前后斜坡
SpeedRamp_t ChassisRamp_ForwardBack = ForwardBackGroundInit;
#undef ForwardBackGroundInit

//底盘遥控左右斜坡
SpeedRamp_t ChassisRamp_LeftRight = LeftRightGroundInit;
#undef LeftRightGroundInit

//底盘遥控左右转斜坡
SpeedRamp_t ChassisRamp_Rotate = RotateGroundInit;
#undef RotateGroundInit


static int FBLR_RampRate = 30;              //前后、左右斜坡函数叠加值
static int Rotate_RampRate = 30;
/* ----------------------- Internal Data ----------------------------------- */
RC_Ctl_t RC_CtrlData;
static uint8_t DT7_Rx_Data[RC_FRAME_LENGTH];
uint8_t DT7_RX_Finish;

void DT7_Init(void);
void DT7_Handle(void);
int DR16_DataCheck(void);
void Online_Check_DR16(void);
bool GetKeyMouseAction(KeyList_e KeyMouse, KeyAction_e Action);
bool IT_GetKeyMouseAction(KeyList_e KeyMouse, KeyAction_e Action);
void KeyMouseFlag_Update(void);
void IT_KeyMouseFlag_Update(void);
void RemoteControl_PC_Update(void);
void IT_RemoteControl_PC_Update(void);


DT7_Fun_t DT7_Fun = DT7_FunGroundInit;
#undef DT7_FunGroundInit
DR16_Export_Data_t DR16_Export_Data = DR16_ExportDataGroundInit;
Image_Transmission_Export_Data_t Image_Transmission_Export_Data = Image_Transmission_ExportDataGroundInit;
#undef DR16_ExportDataGroundInit
/**
  * @brief  DT7初始化，开启接收空闲中断
  * @param  void
  * @retval void
  */
void DT7_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2,DT7_Rx_Data,sizeof(DT7_Rx_Data));
}

/**
  * @brief  DT7数据解析
  * @param 	void
  * @retval void
  */
void DT7_Handle(void)
{
	if(DT7_RX_Finish == 1)
	{
		DT7_RX_Finish = 0;
		
		RC_CtrlData.rc.ch0 = (DT7_Rx_Data[0] | DT7_Rx_Data[1] << 8) & 0x07FF;
		RC_CtrlData.rc.ch0 -= 1024;
		RC_CtrlData.rc.ch1 = (DT7_Rx_Data[1] >> 3 | DT7_Rx_Data[2] << 5) & 0x07FF;
		RC_CtrlData.rc.ch1 -= 1024;
		RC_CtrlData.rc.ch2 = (DT7_Rx_Data[2] >> 6 | DT7_Rx_Data[3] << 2 | DT7_Rx_Data[4] << 10) & 0x07FF;
		RC_CtrlData.rc.ch2 -= 1024;
		RC_CtrlData.rc.ch3 = (DT7_Rx_Data[4] >> 1 | DT7_Rx_Data[5] << 7) & 0x07FF;
		RC_CtrlData.rc.ch3 -= 1024;

		/* prevent remote control zero deviation */
		if (RC_CtrlData.rc.ch0 <= 20 && RC_CtrlData.rc.ch0 >= -20)
		{
			RC_CtrlData.rc.ch0 = 0;
		}
		if (RC_CtrlData.rc.ch1 <= 10 && RC_CtrlData.rc.ch1 >= -10)
		{
			RC_CtrlData.rc.ch1 = 0;
		}
		if (RC_CtrlData.rc.ch2 <= 10 && RC_CtrlData.rc.ch2 >= -10)
		{
			RC_CtrlData.rc.ch2 = 0;
		}
		if (RC_CtrlData.rc.ch3 <= 10 && RC_CtrlData.rc.ch3 >= -10)
		{
			RC_CtrlData.rc.ch3 = 0;
		}

		RC_CtrlData.rc.s1 = ((DT7_Rx_Data[5] >> 4) & 0x000C) >> 2;
		RC_CtrlData.rc.s2 = (DT7_Rx_Data[5] >> 4) & 0x0003;

		RC_CtrlData.mouse.x = DT7_Rx_Data[6] | (DT7_Rx_Data[7] << 8); // x axis
		RC_CtrlData.mouse.y = DT7_Rx_Data[8] | (DT7_Rx_Data[9] << 8);
		RC_CtrlData.mouse.z = DT7_Rx_Data[10] | (DT7_Rx_Data[11] << 8);

		RC_CtrlData.mouse.press_l = DT7_Rx_Data[12];
		RC_CtrlData.mouse.press_r = DT7_Rx_Data[13];

		RC_CtrlData.key.key_code = DT7_Rx_Data[14] | DT7_Rx_Data[15] << 8; // key borad code
		RC_CtrlData.wheel = (DT7_Rx_Data[16] | DT7_Rx_Data[17] << 8) - 1024;
		
		if(RC_CtrlData.wheel< -660 || RC_CtrlData.wheel > 660)
		{
			RC_CtrlData.wheel = 0;
		}
	
		//control code
		
		static uint8_t sigle_bullet_flag =1;//用于检录单发
		static uint8_t deployment_count = 0;//用于部署模式计数
		
		//左摇杆拨上 电脑操作模式
		if(RC_CtrlData.rc.s1 == RC_SW_UP)
		{
			ControlMes.Climb_Whole_Flag = CLIMB_WITHDRAW;
			ControlMes.Climb_Back_Flag = CLIMB_WITHDRAW;
			ControlMes.Climb_Front_Flag = CLIMB_WITHDRAW;  
			
			KeyMouseFlag_Update();
			RemoteControl_PC_Update();
			ControlMes.x_velocity = Mecanum_Data.Speed_ToCloud.vx;
			ControlMes.y_velocity = -1*Mecanum_Data.Speed_ToCloud.vy;
			ControlMes.z_rotation_velocity = Mecanum_Data.Speed_ToCloud.vw;
			ControlMes.yaw_velocity = -1*RC_CtrlData.mouse.x*6;
			ControlMes.pitch_velocity = RC_CtrlData.mouse.y*27;
		}

		//左摇杆拨中 遥控器模式—底盘移动控制
		else if(RC_CtrlData.rc.s1 == RC_SW_MID)	
		{
			ControlMes.Climb_Whole_Flag = CLIMB_WITHDRAW;
			ControlMes.Climb_Back_Flag = CLIMB_WITHDRAW;
			ControlMes.Climb_Front_Flag = CLIMB_WITHDRAW;      
			//底盘运动控制
			ControlMes.x_velocity = -0.1*RC_CtrlData.rc.ch3; //左手上下
		  ControlMes.y_velocity = -0.1*RC_CtrlData.rc.ch2; //左手左右

			//发射状态设置（右拨杆）（UP 单发模式（顺时针右滚动滚轮拨弹，逆时针左退弹）；MID 禁止发射 ；DOWN 部署模式）
			ControlMes.shoot_state = RC_CtrlData.rc.s2;
			if(ControlMes.shoot_state == RC_SW_UP)
			{
//				ControlMes.Check_In_Flag = 1;
				Fric_Data.Fric_Switch = Fric_On;
				if(RC_CtrlData.wheel > 500 && sigle_bullet_flag == 1)
				{
					 Dial_Data.Dial_Switch = Dial_On;
					 Dial_Fun.Dial_Back_OneBullet();
					 sigle_bullet_flag = 0;
				}
				else if(RC_CtrlData.wheel < 500 && RC_CtrlData.wheel > -500)
				{
					Dial_Data.Dial_Switch = Dial_Off;
					sigle_bullet_flag = 1;
				}
				else if(RC_CtrlData.wheel < -500 && sigle_bullet_flag == 1)
				{
					Dial_Data.Dial_Switch = Dial_Back;
					Dial_Fun.Dial_OneBullet();
					//M3508_Array[Dial_Motor].targetAngle -= (float)Angle_DialOneBullet_42mm;
					sigle_bullet_flag = 0;
				}
			}
			else if(ControlMes.shoot_state == RC_SW_MID)     //正常模式
			{
				Dial_Data.Dial_Switch = Dial_Off;
				Dial_Data.Speed_Dial = 0;
				Dial_Data.Number_ToBeFired =0;
				Fric_Data.Fric_Switch = Fric_Off;
				ControlMes.Check_In_Flag = 0;
				sigle_bullet_flag = 0;
				if(ControlMes.shoot_state != RC_SW_UP)
				ControlMes.z_rotation_velocity = 0.3*RC_CtrlData.wheel; //滑轮左右
			}
			else if(ControlMes.shoot_state == RC_SW_DOWN)    //部署模式
			{
				deployment_count++;
				if(deployment_count % 2 == 1)
				{
					ControlMes.Deployment_Flag = 1;
				}
				else if(deployment_count % 2 == 0)
				{
					ControlMes.Deployment_Flag = 0;
				}
			}
			//云台运动控制
			ControlMes.AutoAimFlag = 0;
			ControlMes.pitch_velocity = RC_CtrlData.rc.ch1;		//右手上下
			//Cloud.Target_Pitch += RC_CtrlData.rc.ch1*0.05;
			ControlMes.yaw_velocity = -RC_CtrlData.rc.ch0;		//右手左右


		}
		//左摇杆拨下 自瞄模式 （发射设置与MID时一致）
		else if(RC_CtrlData.rc.s1 == RC_SW_DOWN)
		{
			ControlMes.Climb_Whole_Flag = CLIMB_WITHDRAW;
			ControlMes.Climb_Back_Flag = CLIMB_WITHDRAW;
			ControlMes.Climb_Front_Flag = CLIMB_WITHDRAW;  
			
			ControlMes.pitch_velocity = RC_CtrlData.rc.ch1 ;		//右手上下
			ControlMes.yaw_velocity = -RC_CtrlData.rc.ch0  ;		//右手左右
			ControlMes.z_rotation_velocity = RC_CtrlData.wheel; //滑轮左右
			ControlMes.AutoAimFlag = 1;
			Cloud.AutoAim_Yaw = Auto_Aim_Yaw;   
			Cloud.AutoAim_Pitch = Auto_Aim_Pitch;
			

			/*************************发射状态设置（右拨杆）*********************/
			//（UP 单发模式 ； MID 禁止发射；DOWN 退弹模式）
			ControlMes.shoot_state = RC_CtrlData.rc.s2;
			if(ControlMes.shoot_state == RC_SW_UP)
			{
//				ControlMes.Check_In_Flag = 1;
				Fric_Data.Fric_Switch = Fric_On;
				if(RC_CtrlData.wheel > 400 && sigle_bullet_flag == 1)
				{
					 Dial_Data.Dial_Switch = Dial_On;
					 //M3508_Array[Dial_Motor].targetAngle -= (float)Angle_DialOneBullet_42mm;
					 Dial_Fun.Dial_Back_OneBullet();
					 sigle_bullet_flag = 0;
				}
				else if(RC_CtrlData.wheel < 400 && RC_CtrlData.wheel > -400)
				{
					Dial_Data.Dial_Switch = Dial_Off;
					sigle_bullet_flag = 1;
				}
				else if(RC_CtrlData.wheel < -400 && sigle_bullet_flag == 1)
				{
					Dial_Data.Dial_Switch = Dial_Back;
					M3508_Array[Dial_Motor].targetAngle -= (float)Angle_DialOneBullet_42mm;
					sigle_bullet_flag = 0;
				}
			}
			else if(ControlMes.shoot_state == RC_SW_MID)
			{
				Dial_Data.Dial_Switch = Dial_Off;
				Dial_Data.Speed_Dial = 0;
				Dial_Data.Number_ToBeFired =0;
				Fric_Data.Fric_Switch = Fric_Off;
				ControlMes.Check_In_Flag = 0;
				sigle_bullet_flag = 0;
				if(ControlMes.shoot_state != RC_SW_UP)
				ControlMes.z_rotation_velocity = RC_CtrlData.wheel; //滑轮左右
			}
			else if(ControlMes.shoot_state == RC_SW_DOWN)
			{
				deployment_count++;
				if(deployment_count % 2 == 1)
				{
					ControlMes.Deployment_Flag = 1;
				}
				else if(deployment_count % 2 == 0)
				{
					ControlMes.Deployment_Flag = 0;
				}
			}
			
		}
	
	}
	//用board1 FDCAN2发送给board2
	Board1_FUN.Board1_To_2();	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2,DT7_Rx_Data,sizeof(DT7_Rx_Data));

}

/**
	* @brief  检测遥控数据（遥控器掉线数据并没有出错，所以加多一个0判断，防止掉线重启mcu，但是比赛时需要去掉这个判断）
  * @param	void
  * @retval int   0:数据没有出错      1：数据出错了
  */
int DR16_DataCheck(void)
{
    if ((RC_CtrlData.rc.s1 != RC_SW_UP && RC_CtrlData.rc.s1 != RC_SW_MID && RC_CtrlData.rc.s1 != RC_SW_DOWN && RC_CtrlData.rc.s1 != 0)        /* 左拨杆 */
        || (RC_CtrlData.rc.s2 != RC_SW_UP && RC_CtrlData.rc.s2 != RC_SW_MID && RC_CtrlData.rc.s2 != RC_SW_DOWN && RC_CtrlData.rc.s2 != 0) /* 右拨杆 */
        || (RC_CtrlData.rc.ch0 > 660 || RC_CtrlData.rc.ch0 < -660)                                                                                             /* 通道0 */
        || (RC_CtrlData.rc.ch1 > 660 || RC_CtrlData.rc.ch1 < -660)                                                                                             /* 通道1 */
        || (RC_CtrlData.rc.ch2 > 660 || RC_CtrlData.rc.ch2 < -660)                                                                                             /* 通道2 */
        || (RC_CtrlData.rc.ch3 > 660 || RC_CtrlData.rc.ch3 < -660)                                                                                             /* 通道3 */
        || (RC_CtrlData.wheel > 660 || RC_CtrlData.wheel < -660))                                                                                      /* 波轮 */
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/**
 * @brief DR16掉线检测
 * 
 */
void Online_Check_DR16(void)
{
    //遥控器 ---------------------------
    if (RC_CtrlData.infoUpdateFrame < 1)
    {
        RC_CtrlData.OffLineFlag = 1;
    }
    else
    {
        RC_CtrlData.OffLineFlag = 0;
    }
    RC_CtrlData.infoUpdateFrame = 0;
}

/**
	* @brief  获取鼠标键盘某个键当前的动作
  * @param	键值  动作
  * @retval 返回键鼠动作状态  0 没有该动作 1 有该动作
  */
bool GetKeyMouseAction(KeyList_e KeyMouse, KeyAction_e Action)
{
    uint8_t action = 0;
    switch (Action)
    {
    case KeyAction_CLICK: //单击

        action = ((DR16_Export_Data.KeyMouse.Click_Press_Flag >> KeyMouse) & 1);
        break;
    case KeyAction_PRESS: //按下
        action = ((DR16_Export_Data.KeyMouse.Press_Flag >> KeyMouse) & 1);
        break;
    case KeyAction_LONG_PRESS: //长按
        action = ((DR16_Export_Data.KeyMouse.Long_Press_Flag >> KeyMouse) & 1);
        break;
    default:
        action = 0;
        break;
    }
    return action;
}

static int Q_Lock=0;
static int E_Lock=0;
static int F_Lock=0;
static int ctrl_Lock=0;
static int add_Q = 0;
static int add_E = 0;

/**
  * @Data    2024/3/16
  * @brief   键盘标志位更新 
  * @param   void
  * @retval  void
  */
void KeyMouseFlag_Update(void)
{
    uint32_t KeyMouse = (uint32_t)RC_CtrlData.key.key_code| RC_CtrlData.mouse.press_l<< 16 | RC_CtrlData.mouse.press_r << 17; // 把键盘鼠标的标志位合并。

    for (int Index = 0; Index < KEYMOUSE_AMOUNT; Index++) //遍历全部键位，更新他们的状态。
    {
        if (KeyMouse & (1 << Index)) //判断第index位是否为1。
        {
            DR16_Export_Data.KeyMouse.PressTime[Index]++;
            if (DR16_Export_Data.KeyMouse.PressTime[Index] > TIME_KeyMouse_Press) //满足按下的时间，视为按下
            {
                DR16_Export_Data.KeyMouse.Press_Flag |= 1 << Index; //设置该键的标志位为1
            }

            if (DR16_Export_Data.KeyMouse.PressTime[Index] > TIME_KeyMouse_LongPress) //长按判断
            {

                DR16_Export_Data.KeyMouse.Long_Press_Flag |= 1 << Index; //设置长按标志位
            }
        }
        else
        {
            if ((DR16_Export_Data.KeyMouse.PressTime[Index] > TIME_KeyMouse_Press) && (DR16_Export_Data.KeyMouse.PressTime[Index] < TIME_KeyMouse_LongPress)) //时间处于两者之间，为单击。
            {
                DR16_Export_Data.KeyMouse.Click_Press_Flag |= 1 << Index; //设置单击标志位
            }
            else
            {
                DR16_Export_Data.KeyMouse.Click_Press_Flag &= ~(1 << Index); //取反操作，将该键的标志位设为0
            }

            //已经松开，将按下标志位置空。
            DR16_Export_Data.KeyMouse.Press_Flag &= ~(1 << Index);
            DR16_Export_Data.KeyMouse.Long_Press_Flag &= ~(1 << Index);
            DR16_Export_Data.KeyMouse.PressTime[Index] = 0;
        }
    }
}



void RemoteControl_PC_Update(void)
{
	//前后
	if(GetKeyMouseAction(KEY_W,KeyAction_PRESS)) //w
	{
		ChassisRamp_ForwardBack.rate = FBLR_RampRate;
	}
	else if(GetKeyMouseAction(KEY_S,KeyAction_PRESS))
	{
		ChassisRamp_ForwardBack.rate = -FBLR_RampRate;
	}
	else
	{
		CountReset(&ChassisRamp_ForwardBack);
        ChassisRamp_ForwardBack.rate = 0;
	}
	Mecanum_Data.Speed_ToCloud.vx = -SpeedRampCalc(&ChassisRamp_ForwardBack);
	
	//左右
	if(GetKeyMouseAction(KEY_A,KeyAction_PRESS))
	{
		ChassisRamp_LeftRight.rate = FBLR_RampRate;
	}
	else if(GetKeyMouseAction(KEY_D,KeyAction_PRESS))
	{
		ChassisRamp_LeftRight.rate = -FBLR_RampRate;
	}
	else
	{
		CountReset(&ChassisRamp_LeftRight);
		ChassisRamp_LeftRight.rate = 0;
	}
	Mecanum_Data.Speed_ToCloud.vy = -SpeedRampCalc(&ChassisRamp_LeftRight);
	

	/*小陀螺控制*/	/*QE*/
	if(GetKeyMouseAction(KEY_Q,KeyAction_PRESS) && Q_Lock == 0) //q
	{
		Q_Lock = 1;
	}
	else if(GetKeyMouseAction(KEY_Q,KeyAction_PRESS)!=1 && Q_Lock == 1 && add_Q >=40)
	{
		add_Q = 0;
		Q_Lock = 0;
	}
		
		if(ControlMes.z_rotation_velocity < ChassisRamp_Rotate.maxcount && Q_Lock == 1)
	{
		if(add_Q <40)
		{
			Mecanum_Data.Speed_ToCloud.vw += 10;
			add_Q++;
		}
	}

		
	if(GetKeyMouseAction(KEY_E,KeyAction_PRESS) && E_Lock == 0) //e
	{
		E_Lock = 1;
	}
	else if(GetKeyMouseAction(KEY_E,KeyAction_PRESS)!=1 && E_Lock == 1 && add_E >=40 )
	{
		add_E = 0;
		E_Lock = 0;
	}
	
		if(ControlMes.z_rotation_velocity > ChassisRamp_Rotate.mincount && E_Lock == 1)
	{
		if(add_E < 40)
		{
			Mecanum_Data.Speed_ToCloud.vw -= 10;
			add_E ++;
		}
	}

	
	//急停
	/*清空*/
	if(ControlMes.game_start == 1)
	{
		if(GetKeyMouseAction(KEY_R,KeyAction_PRESS)||ControlMes.Blood_Volume == 0) //r
		{
			Mecanum_Data.Speed_ToCloud.vw = 0;
			Mecanum_Data.Speed_ToCloud.vx = 0;
			Mecanum_Data.Speed_ToCloud.vy = 0;
			add_E = 0;
			add_Q = 0;
			E_Lock = 0;
			Q_Lock = 0;
		}
	}
	else if(ControlMes.game_start == 0)
	{
		if(GetKeyMouseAction(KEY_R,KeyAction_PRESS)) //r
		{
			Mecanum_Data.Speed_ToCloud.vw = 0;
			Mecanum_Data.Speed_ToCloud.vx = 0;
			Mecanum_Data.Speed_ToCloud.vy = 0;
			add_E = 0;
			add_Q = 0;
			E_Lock = 0;
			Q_Lock = 0;
		}
	}
	
	//底盘跟随云台归中
	if(GetKeyMouseAction(KEY_SHIFT,KeyAction_PRESS))
	{
		Mecanum_Fun.Mecanum_Chassis_Follow_Gimbal();
	}
	
//	//shift加速
//	if(GetKeyMouseAction(KEY_SHIFT,KeyAction_PRESS))
//	{
//		FBLR_RampRate = 60;
//	}
//	else
//	{
//		FBLR_RampRate = 30;
//	}
//	
	//鼠标调节云台
	//Cloud.Target_Yaw -= RC_CtrlData.mouse.x;ws
	//Cloud.Target_Pitch += RC_CtrlData.mouse.y*0.6f;
	
	if(GetKeyMouseAction(KEY_X,KeyAction_CLICK))
	{
		Fric_Data.Fric_Switch = !Fric_Data.Fric_Switch ;
		ControlMes.fric_Flag = Fric_Data.Fric_Switch;
	}
	
	//左键射击
	if(GetKeyMouseAction(MOUSE_Left,KeyAction_CLICK) ||GetKeyMouseAction(KEY_G,KeyAction_CLICK) )
	{
		Dial_Data.Shoot_Mode = Single_Shoot;
		Dial_Data.Dial_Switch = Dial_On;
		//if(Fric_Data.Fric_Ready)//如果摩擦轮转速满足 发单量++
		Fric_Data.Fric_Ready = true;
			Dial_Data.Number_ToBeFired=1;
		if(ControlMes.fric_Flag == 1)
		{		
			Dial_Fun.Dial_OneBullet();
		}
	//	Shoot_Data.Shoot_Switch = Shoot_On;
		//Shoot_Data.Shoot_Switch = 1;
	}
	else
	{
	//	Shoot_Data.Shoot_Switch = Shoot_Off;
		Dial_Data.Dial_Switch = Dial_Off;
	}
	
	if(GetKeyMouseAction(KEY_F,KeyAction_CLICK))
	{
		Dial_Fun.Dial_Back_OneBullet();
		Dial_Data.Dial_Switch = Dial_Back;
	}
	//右键开启自瞄
	if(GetKeyMouseAction(MOUSE_Right,KeyAction_CLICK))
	{
		if(ControlMes.AutoAimFlag == 0)
		{
			ControlMes.AutoAimFlag = 1;
			//上位机视觉得到的正负与电机的正负是一样的，通信两边的正负号要对好	
			ControlMes.yaw_position = Auto_Aim_Yaw;
			Cloud.AutoAim_Pitch = Auto_Aim_Pitch;

		}
		else if(ControlMes.AutoAimFlag == 1)
		{
			ControlMes.AutoAimFlag = 0;
		}
	}
	
	if(ControlMes.AutoAimFlag == 1)
	{
		ControlMes.yaw_position = Auto_Aim_Yaw;
		Cloud.AutoAim_Pitch = Auto_Aim_Pitch;
	}
	
	//调节底盘速度档位
	if(GetKeyMouseAction(KEY_C,KeyAction_CLICK))
	{
		ChassisRamp_ForwardBack.maxcount = 30;
		ChassisRamp_ForwardBack.mincount = -30;
		ChassisRamp_LeftRight.maxcount = 30;
		ChassisRamp_LeftRight.mincount = -30;
		ChassisRamp_Rotate.maxcount = 40;
		ChassisRamp_Rotate.mincount = -40;
	}
	else if(GetKeyMouseAction(KEY_V,KeyAction_CLICK))
	{
		ChassisRamp_ForwardBack.maxcount = 50;
		ChassisRamp_ForwardBack.mincount = -50;
		ChassisRamp_LeftRight.maxcount = 50;
		ChassisRamp_LeftRight.mincount = -50;
		ChassisRamp_Rotate.maxcount = 100;
		ChassisRamp_Rotate.mincount = -100;
	}
	else if(GetKeyMouseAction(KEY_B,KeyAction_CLICK))
	{
		ChassisRamp_ForwardBack.maxcount = 80;
		ChassisRamp_ForwardBack.mincount = -80;
		ChassisRamp_LeftRight.maxcount = 80;
		ChassisRamp_LeftRight.mincount = -80;	
		ChassisRamp_Rotate.maxcount = 155;
		ChassisRamp_Rotate.mincount = -155;
	}
	
	if(GetKeyMouseAction(KEY_Z,KeyAction_CLICK))
	{
		ChassisRamp_ForwardBack.rate = 3;
		ChassisRamp_LeftRight.rate = 3;
	}
	else if(GetKeyMouseAction(KEY_CTRL,KeyAction_CLICK))
	{
		ChassisRamp_ForwardBack.rate = 30;
		ChassisRamp_LeftRight.rate = 30;
	}
	
}
