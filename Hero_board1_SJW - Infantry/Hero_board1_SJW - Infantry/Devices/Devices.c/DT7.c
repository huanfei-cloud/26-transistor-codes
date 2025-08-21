/**
 * @file DT7.c
 * @author lxr(784457420@qq.com),cyx(1686143358@qq.com)
 * @brief
 * @version 2.0
 * @date 2024-3-9
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "DT7.h"

// 底盘遥控前后斜坡
SpeedRamp_t ChassisRamp_ForwardBack = ForwardBackGroundInit;
#undef ForwardBackGroundInit

// 底盘遥控左右斜坡
SpeedRamp_t ChassisRamp_LeftRight = LeftRightGroundInit;
#undef LeftRightGroundInit

// 底盘遥控左右转斜坡
SpeedRamp_t ChassisRamp_Rotate = RotateGroundInit;
#undef RotateGroundInit

/* ----------------------- Internal Data ----------------------------------- */
RC_Ctl_t RC_CtrlData;
static uint8_t DT7_Rx_Data[RC_FRAME_LENGTH];
uint8_t DT7_RX_Finish;

void RemoteControl_PC_Update(void);
void IT_RemoteControl_PC_Update(void);

DR16_Export_Data_t DR16_Export_Data = DR16_ExportDataGroundInit;
Image_Transmission_Export_Data_t Image_Transmission_Export_Data = Image_Transmission_ExportDataGroundInit;

/**
 * @brief  DT7初始化，开启接收空闲中断
 * @param  void
 * @retval void
 */
void DT7_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, DT7_Rx_Data, sizeof(DT7_Rx_Data));
}

/**
 * @Data    2024/3/9
 * @brief   键盘标志位更新
 * @param   void
 * @retval  void
 */
void KeyMouseFlag_Update(void)
{
	uint32_t KeyMouse = (uint32_t)RC_CtrlData.key.key_code | RC_CtrlData.mouse.press_l << 16 | RC_CtrlData.mouse.press_r << 17; // 把键盘鼠标的标志位合并。

	for (int Index = 0; Index < KEYMOUSE_AMOUNT; Index++) // 遍历全部键位，更新他们的状态。
	{
		if (KeyMouse & (1 << Index)) // 判断第index位是否为1。
		{
			DR16_Export_Data.KeyMouse.PressTime[Index]++;
			if (DR16_Export_Data.KeyMouse.PressTime[Index] > TIME_KeyMouse_Press) // 满足按下的时间，视为按下
			{
				DR16_Export_Data.KeyMouse.Press_Flag |= 1 << Index; // 设置该键的标志位为1
			}

			if (DR16_Export_Data.KeyMouse.PressTime[Index] > TIME_KeyMouse_LongPress) // 长按判断
			{

				DR16_Export_Data.KeyMouse.Long_Press_Flag |= 1 << Index; // 设置长按标志位
			}
		}
		else
		{
			if ((DR16_Export_Data.KeyMouse.PressTime[Index] > TIME_KeyMouse_Press) && (DR16_Export_Data.KeyMouse.PressTime[Index] < TIME_KeyMouse_LongPress)) // 时间处于两者之间，为单击。
			{
				DR16_Export_Data.KeyMouse.Click_Press_Flag |= 1 << Index; // 设置单击标志位
			}
			else
			{
				DR16_Export_Data.KeyMouse.Click_Press_Flag &= ~(1 << Index); // 取反操作，将该键的标志位设为0
			}

			// 已经松开，将按下标志位置空。
			DR16_Export_Data.KeyMouse.Press_Flag &= ~(1 << Index);
			DR16_Export_Data.KeyMouse.Long_Press_Flag &= ~(1 << Index);
			DR16_Export_Data.KeyMouse.PressTime[Index] = 0;
		}
	}
}

/**
 * @brief  DT7数据解析
 * @param  void
 * @retval void
 */
void DT7_Handle(void)
{
	/*DT7数据处理*/
	if (DT7_RX_Finish == 1)
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
		if (RC_CtrlData.rc.ch0 <= 100 && RC_CtrlData.rc.ch0 >= -100)
		{
			RC_CtrlData.rc.ch0 = 0;
		}
		if (RC_CtrlData.rc.ch1 <= 5 && RC_CtrlData.rc.ch1 >= -5)
		{
			RC_CtrlData.rc.ch1 = 0;
		}
		if (RC_CtrlData.rc.ch2 <= 5 && RC_CtrlData.rc.ch2 >= -5)
		{
			RC_CtrlData.rc.ch2 = 0;
		}
		if (RC_CtrlData.rc.ch3 <= 5 && RC_CtrlData.rc.ch3 >= -5)
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

		if (RC_CtrlData.wheel < -660 || RC_CtrlData.wheel > 660)
		{
			RC_CtrlData.wheel = 0;
		}

		/**************************** control code ****************************/
		/*通道1*/
		/*中正常遥控；下自瞄；上键鼠*/
		if (RC_CtrlData.rc.s1 == RC_SW_MID && RC_CtrlData.rc.s2 != 0)
		{
			/******************************遥控器数值传递******************************/
			// 底盘运动控制
			ControlMes.x_velocity = -RC_CtrlData.rc.ch3; // 左手上下
			ControlMes.y_velocity = -RC_CtrlData.rc.ch2; // 左手左右

			static int countFric = 0;
			// 发射状态设置（UP 发射 ； MID 禁止发射；DOWN 检录） //检录模式是停止小陀螺，让Yaw轴与底盘相对静止（检录校准测速用）
			if (RC_CtrlData.rc.s2 == RC_SW_DOWN)
			{
				ControlMes.modelFlag = model_Record;
				ControlMes.fric_Flag = 0;
				countFric = 0;
				ControlMes.shoot_state = RC_SW_MID;
			}
			else if (RC_CtrlData.rc.s2 == RC_SW_MID)
			{
				ControlMes.modelFlag = model_Normal;
				ControlMes.fric_Flag = 0;
				ControlMes.shoot_state = RC_CtrlData.rc.s2;
				countFric = 0;
			}
			else if (RC_CtrlData.rc.s2 == RC_SW_UP)
			{
				ControlMes.modelFlag = model_Normal;
				ControlMes.fric_Flag = 1;
				if (countFric < 50)
				{
					countFric++;
				}
				else
				{
					ControlMes.shoot_state = RC_CtrlData.rc.s2;
				}
			}

			// 云台运动控制
			ControlMes.AutoAimFlag = 0;
			ControlMes.pitch_velocity = RC_CtrlData.rc.ch1;			// 右手上下
			ControlMes.yaw_velocity = RC_CtrlData.rc.ch0;			// 右手左右
			ControlMes.z_rotation_velocity = RC_CtrlData.wheel * 2; // 滑轮左右
			ControlMes.yaw_position = Auto_Aim_Yaw;
		}

		// 自瞄模式
		else if (RC_CtrlData.rc.s1 == RC_SW_DOWN && RC_CtrlData.rc.s2 != 0)
		{
			/******************************遥控器数值传递******************************/
			// 底盘运动控制
			ControlMes.x_velocity = -RC_CtrlData.rc.ch3; // 左手上下
			ControlMes.y_velocity = -RC_CtrlData.rc.ch2; // 左手左右

			// 发射状态设置（UP 连发模式 ； MID 禁止发射；DOWN 单发模式）
			ControlMes.shoot_state = RC_CtrlData.rc.s2;

			// 自瞄云台运动控制（这里添加额外的遥控器控制是为了补偿自瞄精度，自己用遥控器微调一下辅助瞄准）
			ControlMes.pitch_velocity = RC_CtrlData.rc.ch1 * 0.2; // 右手上下
			ControlMes.yaw_velocity = RC_CtrlData.rc.ch0;		  // 右手左右
			ControlMes.AutoAimFlag = 1;
			// 上位机视觉得到的正负与电机的正负是一样的，通信两边的正负号要对好
			ControlMes.yaw_position = Auto_Aim_Yaw;
			Cloud.AutoAim_Pitch = Auto_Aim_Pitch;
			if (ControlMes.shoot_state == RC_SW_DOWN)
			{
				Dial_Data.Speed_Dial = (RC_CtrlData.wheel) * 5;
				ControlMes.modelFlag = model_Normal;
				ControlMes.fric_Flag = 1;
			}
			else if (ControlMes.shoot_state == RC_SW_UP)
			{
				ControlMes.modelFlag = model_Normal;
				ControlMes.fric_Flag = 1;
				Dial_Data.Speed_Dial = -(RC_CtrlData.wheel) * 5;
			}
			else if (ControlMes.shoot_state == RC_SW_MID)
			{
				ControlMes.fric_Flag = 0;
			}
		}
		else if (RC_CtrlData.rc.s1 == RC_SW_UP && RC_CtrlData.rc.s2 == RC_SW_MID)
		{
			/*键鼠数据处理*/
			KeyMouseFlag_Update();
			RemoteControl_PC_Update();
		}
		else
		{
			ControlMes.AutoAimFlag = 0;
			ControlMes.x_velocity = 0;			// 左手上下
			ControlMes.y_velocity = 0;			// 左手左右
			ControlMes.z_rotation_velocity = 0; // 右手上下
			ControlMes.yaw_velocity = 0;
			ControlMes.pitch_velocity = 0;
			ControlMes.shoot_state = RC_SW_MID;
		}

		/*发射信息处理*/
		if (ControlMes.shoot_state == RC_SW_UP)
		{
			Dial_Data.Shoot_Mode = Continuous_Shoot;
			Shoot_Data.Shoot_Switch = 1;
		}
		else if (ControlMes.shoot_state == RC_SW_MID)
		{
			Shoot_Data.Shoot_Switch = 0;
		}
	}
	// 用board1 CAN2发送给board2。
	Board1_FUN.Board1_To_2();
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
	case KeyAction_CLICK: // 单击
		action = ((DR16_Export_Data.KeyMouse.Click_Press_Flag >> KeyMouse) & 1);
		break;
	case KeyAction_PRESS: // 按下
		action = ((DR16_Export_Data.KeyMouse.Press_Flag >> KeyMouse) & 1);
		break;
	case KeyAction_LONG_PRESS: // 长按
		action = ((DR16_Export_Data.KeyMouse.Long_Press_Flag >> KeyMouse) & 1);
		break;
	default:
		action = 0;
		break;
	}
	return action;
}

bool IT_GetKeyMouseAction(KeyList_e KeyMouse, KeyAction_e Action)
{
	uint8_t action = 0;
	switch (Action)
	{
	case KeyAction_CLICK: // 单击

		action = ((Image_Transmission_Export_Data.KeyMouse.Click_Press_Flag >> KeyMouse) & 1);
		break;
	case KeyAction_PRESS: // 按下
		action = ((Image_Transmission_Export_Data.KeyMouse.Press_Flag >> KeyMouse) & 1);
		break;
	case KeyAction_LONG_PRESS: // 长按
		action = ((Image_Transmission_Export_Data.KeyMouse.Long_Press_Flag >> KeyMouse) & 1);
		break;
	default:
		action = 0;
		break;
	}
	return action;
}

static int RampRate_ForwardBack = 80; // 斜坡函数叠加值
static int RampRate_LeftRight = 80;	  // 斜坡函数叠加值
// static float RampRate_Rotate = 0.2;              //斜坡函数叠加值

static int Q_Lock = 0;
static int E_Lock = 0;
static int F_Lock = 0;
static int ctrl_Lock = 0;
static int add_Q = 0;
static int add_E = 0;

/**
 * @brief  控制更新
 * @param	none
 * @retval 修改控制
 */
void RemoteControl_PC_Update(void)
{
	/*清空*/
	if (ControlMes.game_start == 1)
	{
		if (GetKeyMouseAction(KEY_R, KeyAction_PRESS) || ControlMes.Blood_Volume == 0) // r
		{
			ControlMes.z_rotation_velocity = 0;
			add_E = 0;
			add_Q = 0;
			E_Lock = 0;
			Q_Lock = 0;
			ControlMes.change_Flag = 0;
		}
	}
	else if (ControlMes.game_start == 0)
	{
		if (GetKeyMouseAction(KEY_R, KeyAction_PRESS)) // r
		{
			ControlMes.z_rotation_velocity = 0;
			add_E = 0;
			add_Q = 0;
			E_Lock = 0;
			Q_Lock = 0;
			ControlMes.change_Flag = 0;
		}
	}

	/*前后运动控制*/
	if (GetKeyMouseAction(KEY_W, KeyAction_PRESS)) // w
	{
		ChassisRamp_ForwardBack.rate = RampRate_ForwardBack;
	}
	else if (GetKeyMouseAction(KEY_S, KeyAction_PRESS))
	{
		ChassisRamp_ForwardBack.rate = -RampRate_ForwardBack;
	}
	else
	{
		CountReset(&ChassisRamp_ForwardBack);
		ChassisRamp_ForwardBack.rate = 0;
	}
	ControlMes.x_velocity = -SpeedRampCalc(&ChassisRamp_ForwardBack);

	/*左右运动控制*/
	if (GetKeyMouseAction(KEY_A, KeyAction_PRESS)) // a
	{
		ChassisRamp_LeftRight.rate = -RampRate_LeftRight;
	}
	else if (GetKeyMouseAction(KEY_D, KeyAction_PRESS))
	{
		ChassisRamp_LeftRight.rate = RampRate_LeftRight;
	}
	else
	{
		CountReset(&ChassisRamp_LeftRight);
		ChassisRamp_LeftRight.rate = 0;
	}
	ControlMes.y_velocity = -SpeedRampCalc(&ChassisRamp_LeftRight);

	/*小陀螺控制*/												  /*QE*/
	if (GetKeyMouseAction(KEY_Q, KeyAction_PRESS) && Q_Lock == 0) // q
	{
		Q_Lock = 1;
	}
	else if (GetKeyMouseAction(KEY_Q, KeyAction_PRESS) != 1 && Q_Lock == 1 && add_Q >= 40)
	{
		add_Q = 0;
		Q_Lock = 0;
	}

	if (ControlMes.z_rotation_velocity < ChassisRamp_Rotate.maxcount && Q_Lock == 1)
	{
		if (add_Q < 40)
		{
			ControlMes.z_rotation_velocity += 10;
			add_Q++;
		}
	}

	if (GetKeyMouseAction(KEY_E, KeyAction_PRESS) && E_Lock == 0) // e
	{
		E_Lock = 1;
	}
	else if (GetKeyMouseAction(KEY_E, KeyAction_PRESS) != 1 && E_Lock == 1 && add_E >= 40)
	{
		add_E = 0;
		E_Lock = 0;
	}

	if (ControlMes.z_rotation_velocity > ChassisRamp_Rotate.mincount && E_Lock == 1)
	{
		if (add_E < 40)
		{
			ControlMes.z_rotation_velocity -= 10;
			add_E++;
		}
	}

	/*右键自瞄*/
	if (GetKeyMouseAction(MOUSE_Right, KeyAction_CLICK))
	{
		if (ControlMes.AutoAimFlag == 0)
		{
			ControlMes.AutoAimFlag = 1;
			// 上位机视觉得到的正负与电机的正负是一样的，通信两边的正负号要对好
			ControlMes.yaw_position = Auto_Aim_Yaw;
			Cloud.AutoAim_Pitch = Auto_Aim_Pitch;
		}
		else if (ControlMes.AutoAimFlag == 1)
		{
			ControlMes.AutoAimFlag = 0;
		}
	}

	if (ControlMes.AutoAimFlag == 1)
	{
		ControlMes.yaw_position = Auto_Aim_Yaw;
		Cloud.AutoAim_Pitch = Auto_Aim_Pitch;
	}

	/*鼠标左键发射控制*/
	if ((GetKeyMouseAction(MOUSE_Left, KeyAction_PRESS) || GetKeyMouseAction(MOUSE_Left, KeyAction_LONG_PRESS)) && ControlMes.fric_Flag == 1)
	{
		ControlMes.shoot_state = RC_SW_UP;
	}
	else
	{
		ControlMes.shoot_state = RC_SW_MID;
	}

	/*运动档位控制*/ /*CVB*/
	if (GetKeyMouseAction(KEY_C, KeyAction_PRESS))
	{
		ChassisRamp_ForwardBack.maxcount = 600;
		ChassisRamp_ForwardBack.mincount = -600;
		ChassisRamp_LeftRight.maxcount = 600;
		ChassisRamp_LeftRight.mincount = -600;
		ChassisRamp_Rotate.maxcount = 800;
		ChassisRamp_Rotate.mincount = -800;
	}

	if (GetKeyMouseAction(KEY_V, KeyAction_PRESS))
	{
		ChassisRamp_ForwardBack.maxcount = 900;
		ChassisRamp_ForwardBack.mincount = -900;
		ChassisRamp_LeftRight.maxcount = 900;
		ChassisRamp_LeftRight.mincount = -900;
		ChassisRamp_Rotate.maxcount = 1200;
		ChassisRamp_Rotate.mincount = -1200;
	}

	if (GetKeyMouseAction(KEY_B, KeyAction_PRESS))
	{
		ChassisRamp_ForwardBack.maxcount = 1200;
		ChassisRamp_ForwardBack.mincount = -1200;
		ChassisRamp_LeftRight.maxcount = 1200;
		ChassisRamp_LeftRight.mincount = -1200;
		ChassisRamp_Rotate.maxcount = 1600;
		ChassisRamp_Rotate.mincount = -1600;
	}

	/*发射档位控制*/
	/*2发/s*/
	if (GetKeyMouseAction(KEY_Z, KeyAction_CLICK))
	{
		// 至少 2发/s -540
		if (Dial_Data.Speed_Dial < -550)
		{
			Dial_Data.Speed_Dial += 540;
			ControlMes.shoot_Speed -= 2;
		}
	}

	if (GetKeyMouseAction(KEY_X, KeyAction_CLICK))
	{
		// 至多 12发/s -3240
		if (Dial_Data.Speed_Dial > -3000)
		{
			Dial_Data.Speed_Dial -= 540;
			ControlMes.shoot_Speed += 2;
		}
	}

	// 云台控制
	ControlMes.yaw_velocity = RC_CtrlData.mouse.x * 5;
	ControlMes.pitch_velocity = RC_CtrlData.mouse.y * 8;

	if (GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS))
	{
		if (ctrl_Lock == 0)
		{
			if (ControlMes.change_Flag == 0)
			{
				ControlMes.change_Flag = 1;
			}
			else if (ControlMes.change_Flag == 1)
			{
				ControlMes.change_Flag = 0;
			}
			ctrl_Lock = 1;
		}
	}
	else
	{
		ctrl_Lock = 0;
	}

	if (GetKeyMouseAction(KEY_F, KeyAction_PRESS))
	{
		if (F_Lock == 0)
		{
			if (ControlMes.fric_Flag == 0)
			{
				ControlMes.fric_Flag = 1;
			}
			else if (ControlMes.fric_Flag == 1)
			{
				ControlMes.fric_Flag = 0;
			}
			F_Lock = 1;
		}
	}
	else
	{
		F_Lock = 0;
	}

	if (GetKeyMouseAction(KEY_G, KeyAction_PRESS))
	{
		ControlMes.redial = 1;
	}
	else
	{
		ControlMes.redial = 0;
	}
}
