#include "DT7.h"
#include "Dial.h"
#include "shoot.h"
#include "Cloud_Control.h"
#include "SpeedRamp.h"
#include "UI.h"

//����ң��ǰ��б��
SpeedRamp_t ChassisRamp_ForwardBack = ForwardBackGroundInit;
#undef ForwardBackGroundInit

//����ң������б��
SpeedRamp_t ChassisRamp_LeftRight = LeftRightGroundInit;
#undef LeftRightGroundInit

//����ң������תб��
SpeedRamp_t ChassisRamp_Rotate = RotateGroundInit;
#undef RotateGroundInit


static int FBLR_RampRate = 3;              //ǰ������б�º�������ֵ
static int Rotate_RampRate = 30;

/**
	* @brief  ��ȡ������ĳ������ǰ�Ķ���
  * @param	��ֵ  ����
  * @retval ���ؼ�����״̬  0 û�иö��� 1 �иö���
  */
bool GetKeyMouseAction(KeyList_e KeyMouse, KeyAction_e Action)
{
    uint8_t action = 0;
    switch (Action)
    {
    case KeyAction_CLICK: //����

        action = ((DR16_Export_Data.KeyMouse.Click_Press_Flag >> KeyMouse) & 1);
        break;
    case KeyAction_PRESS: //����
        action = ((DR16_Export_Data.KeyMouse.Press_Flag >> KeyMouse) & 1);
        break;
    case KeyAction_LONG_PRESS: //����
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
