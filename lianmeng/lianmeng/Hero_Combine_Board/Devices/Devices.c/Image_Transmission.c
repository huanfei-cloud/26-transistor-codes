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


static int FBLR_RampRate = 3;              //前后、左右斜坡函数叠加值
static int Rotate_RampRate = 30;

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

void Image_Transmission_Update(void)
{
	
	
}
