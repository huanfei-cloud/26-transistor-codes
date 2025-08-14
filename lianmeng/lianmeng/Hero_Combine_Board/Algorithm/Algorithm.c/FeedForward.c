/**
 * @file FeedForward.c
 * @author Why
 * @brief ǰ������,ȡʵ�����İٷ�֮��ʮ
 * @version 0.1
 * @date 2023-08-28
 * @attention һ��Ҫ��PID�������֮�����������������������Ϊ��ֱ�Ӽ��������������
 *
 * @copyright
 *
 */

#include "M3508_Motor.h"
#include "M6020_Motor.h"
#include "Cloud_Control.h"
#include "Dial.h"
#include "FeedForward.h"
#include "Saber_C3.h"

/**************�û����ݶ���****************/
void FeedForward_Fric(void);
void FeedForward_Dial(void);
void FeedForward_Pitch(void);
void FeedForward_Pitch_Accelerate(void);

/****************�ӿڶ���******************/
FeedForward_FUN_t FeedForward_FUN = FeedForward_FunGroundInit;
#undef FeedForward_FunGroundInit

#include "FeedForward.h"

fp32 param=2500;
fp32 max=0;
fp32 min=0;
fp32 angle_hd;
fp32 Forward_Accelerate;
/**
  * @brief   ��Ħ���ַ���ǰ��
  * @param   M3508���ⲿ�ӿ�
  * @retval  void
  */
void FeedForward_Fric()
{
    /* ƽ��Ħ���� */
    if(M3508_Array[FricL_Wheel].realSpeed < 1000)
        M3508_Array[FricL_Wheel].outCurrent +=
            (212 - 0.124f * (float)M3508_Array[FricL_Wheel].realSpeed);
    else
        M3508_Array[FricL_Wheel].outCurrent +=
            (88 - 0.01f * (float)(M3508_Array[FricL_Wheel].realSpeed - 1000));

    if(M3508_Array[FricR_Wheel].realSpeed < 1000)
        M3508_Array[FricR_Wheel].outCurrent +=
            (212 - 0.124f * (float)M3508_Array[FricR_Wheel].realSpeed);
    else
        M3508_Array[FricR_Wheel].outCurrent +=
            (88 - 0.01f * (float)(M3508_Array[FricR_Wheel].realSpeed - 1000));
		if(M3508_Array[FricU_Wheel].realSpeed < 1000)
        M3508_Array[FricU_Wheel].outCurrent +=
            (212 - 0.124f * (float)M3508_Array[FricR_Wheel].realSpeed);
    else
        M3508_Array[FricR_Wheel].outCurrent +=
            (88 - 0.01f * (float)(M3508_Array[FricR_Wheel].realSpeed - 1000));

    /* ƽ�ⵯ���Ħ���ֵľ޴����������ٵ��� */
    M3508_Array[FricL_Wheel].outCurrent += 5000;
    M3508_Array[FricR_Wheel].outCurrent += 5000;
	  M3508_Array[FricU_Wheel].outCurrent += 5000;
}

/**
  * @brief   �����̵������ǰ��
  * @param
  * @retval  void
  * @attention �����С����Ĳ�����һ��,�������ֶ�Ҫ��
  */
void FeedForward_Dial()
{
    static uint8_t temp;
    static int16_t temp_v = 0;  // ��һ�ε��ٶȣ���������ٶ�
    /* ƽ������ */
    if(Dial_Data.Bullet_Dialed <= MAX_GRAVITY)
        temp = Dial_Data.Bullet_Dialed;
    else temp = MAX_GRAVITY;

    if(temp >= 4) temp -= Num_InTube;
    else temp = 0;
    M3508_Array[Dial_Motor].outCurrent += temp * Gravity_Para;

    /* ƽ����ٶ� */
    if(Dial_Data.Bullet_Dialed <= MAX_GRAVITY)
        temp = Dial_Data.Bullet_Dialed + 6;
    else temp = MAX_GRAVITY + 6;
    M3508_Array[Dial_Motor].outCurrent = temp * (float)(M3508_Array[Dial_Motor].targetSpeed\
                                         - temp_v) * Accelerate_Para;
    temp_v = M3508_Array[Dial_Motor].targetSpeed;

}

#if Pitch_Fitting == 2
/**
  * @brief   ����̨�������������ǰ��,pitch�ö��κ������
  * @param   ��̨����̨������ⲿ�ӿ�
  * @retval  void
  */
void FeedForward_Pitch()
{
    static float exp1, exp2, exp3, exp4; // ��ʱ����
    /* ƽ��Pitch���� */
    /* �Ӵ�ת�ǵ�Сת�� y = 0.01609 * x ^ 2 -279.1 * x + 1.207e+06 */
    exp1 = M6020s_Pitch.realAngle;
    if(Cloud.Target_Pitch > M6020s_Pitch.realAngle + Pitch_Margin)
    {
        arm_mult_f32(&exp1, &exp1, &exp3, 1);
        exp2 = 0.0160f;
        arm_mult_f32(&exp3, &exp2, &exp3, 1);
        exp2 = 279.1f;
        arm_mult_f32(&exp2, &exp1, &exp4, 1);
        arm_sub_f32(&exp3, &exp4, &exp3, 1);
        exp2 = 1.207e+06f;
        arm_add_f32(&exp3, &exp2, &exp4, 1);
        M6020s_Pitch.outCurrent += 1.125f * exp4;
    }
    /* ��Сת�ǵ���ת�� y = 0.003091 * x ^ 2 -63.3 * x + 3.127e+05 */
    else if(Cloud.Target_Pitch < M6020s_Pitch.realAngle - Pitch_Margin)
    {
        arm_mult_f32(&exp1, &exp1, &exp3, 1);
        exp2 = 0.003091;
        arm_mult_f32(&exp3, &exp2, &exp3, 1);
        exp2 = 63.3f;
        arm_mult_f32(&exp2, &exp1, &exp4, 1);
        arm_sub_f32(&exp3, &exp4, &exp3, 1);
        exp2 = 3.127e+05f;
        arm_add_f32(&exp3, &exp2, &exp4, 1);
        M6020s_Pitch.outCurrent += exp4;
    }
    else M6020s_Pitch.outCurrent += 1.125f * exp4;

    /* ƽ��YawĦ���� */

}

#elif Pitch_Fitting == 3
/**
  * @brief   ����̨�������������ǰ����pitch�����Ǻ������
  * @param   ��̨����̨������ⲿ�ӿ�
  * @retval  void
  */
void FeedForward_Pitch()
{
    static float exp1, exp2, exp3, exp4; // ��ʱ����
    /* ƽ��Pitch���� */
    /* �Ӵ�ת�ǵ�Сת�� y = 4154*sin(0.004271x + 37.18)���� */
    exp1 = M6020s_Pitch.realAngle;
    if(Cloud.Target_Pitch > M6020s_Pitch.realAngle + Pitch_Margin)
    {
        exp2 = 0.004271f;
        arm_mult_f32(&exp1, &exp2, &exp3, 1);
        exp2 = 37.18;
        arm_add_f32(&exp2, &exp3, &exp4, 1);
        exp3 = arm_sin_f32(exp4);
        exp2 = 4154.0f;
        arm_mult_f32(&exp2, &exp3, &exp1, 1);
    }
    /* ��Сת�ǵ���ת�� y = 18090*sin(0.0006841x + 66.55)���� */
    else if(Cloud.Target_Pitch < M6020s_Pitch.realAngle - Pitch_Margin)
    {
        exp2 = 0.0006841f;
        arm_mult_f32(&exp1, &exp2, &exp3, 1);
        exp2 = 66.55;
        arm_add_f32(&exp2, &exp3, &exp4, 1);
        exp3 = arm_sin_f32(exp4);
        exp2 = 18090.0f;
        arm_mult_f32(&exp2, &exp3, &exp1, 1);
    }
    else M6020s_Pitch.outCurrent += exp4; //ȷ��ǰ���ı�����һ��ֻȡ���ֵ�İٷ�֮��ʮ

    /* ƽ��YawĦ���� */

}
#elif Pitch_Fitting == 6
/**
  * @brief   ����̨�������������ǰ����pitch�����ζ���ʽ�������
  * @param   ��̨����̨������ⲿ�ӿ�
  * @retval  void
  * @attention 
  */
void FeedForward_Pitch()
{
	 static float exp1, exp2, exp3, exp4; // ��ʱ����
    /* ƽ��Pitch���� */
    /* �Ӵ�ת�ǵ�Сת�� ����ģ�� Poly3:
     f(x) = p1*x^3 + p2*x^2 + p3*x + p4
ϵ��(���ű߽�Ϊ 95%):
       p1 =   3.302e-05  (2.445e-05, 4.159e-05)
       p2 =     -0.7656  (-0.9635, -0.5676)
       p3 =        5921  (4398, 7444)
       p4 =  -1.527e+07  (-1.918e+07, -1.137e+07)

����Ŷ�:
  SSE: 2.729e+06
  R ��: 0.9879
  ���� R ��: 0.9857
  RMSE: 400.7 */

	if(Cloud.Target_Pitch > M6020s_Pitch.realAngle + Pitch_Margin)
    {
		exp1 = 1.527e+07;
		exp4 = 0 ;
		arm_sub_f32(&exp4,&exp1,&exp4,1);//exp4=p4
		if(M6020s_Pitch.realAngle>7000)
		exp2 = M6020s_Pitch.realAngle;
		else
		exp2 = M6020s_Pitch.realAngle+8192;
		exp1 = 5921;
        arm_mult_f32(&exp1, &exp2, &exp1, 1);//exp1=x*p3
		arm_add_f32(&exp1,&exp4,&exp4,1);//exp4=p4+x*p3
		arm_mult_f32(&exp2, &exp2, &exp3, 1);//exp3=x*x
		exp1=0.7656;
		arm_mult_f32(&exp1, &exp3, &exp1, 1);//exp1=p2*x*x
		arm_sub_f32(&exp4,&exp1,&exp4,1);//exp4=p4+x*p3+p2*x*x
		arm_mult_f32(&exp3, &exp2, &exp3, 1);//exp3=x*x*x
        exp1=3.302e-05;
        arm_mult_f32(&exp1, &exp3, &exp1, 1);//exp1=p1*x*x*x
        arm_add_f32(&exp1, &exp4, &exp4, 1);//exp4=p4+x*p3+x*x*p2+x*x*x*p1
        M6020s_Pitch.outCurrent += 1.5f * exp4;
    }
    /* ��Сת�ǵ���ת�� ����ģ�� Poly3:
     f(x) = p1*x^3 + p2*x^2 + p3*x + p4
ϵ��(���ű߽�Ϊ 95%):
       p1 =   2.142e-05  (1.355e-05, 2.928e-05)
       p2 =     -0.4911  (-0.6727, -0.3095)
       p3 =        3762  (2364, 5159)
       p4 =  -9.627e+06  (-1.321e+07, -6.046e+06)

����Ŷ�:
  SSE: 2.298e+06
  R ��: 0.992
  ���� R ��: 0.9905
  RMSE: 367.6
*/
    else if(Cloud.Target_Pitch < M6020s_Pitch.realAngle - Pitch_Margin)
    {
		exp1 = 9.627e+06;
		exp4 = 0 ;
        arm_sub_f32(&exp4,&exp1,&exp4,1);//exp4=p4
		if(M6020s_Pitch.realAngle>7000)
			exp2 = M6020s_Pitch.realAngle;
		else
			exp2 = M6020s_Pitch.realAngle+8192;
		exp1 = 3762;
        arm_mult_f32(&exp1, &exp2, &exp1, 1);//exp1=x*p3
		arm_add_f32(&exp1,&exp4,&exp4,1);//exp4=p4+x*p3
		arm_mult_f32(&exp2, &exp2, &exp3, 1);//exp3=x*x
		exp1=0.4911;
		arm_mult_f32(&exp1, &exp3, &exp1, 1);//exp1=p2*x*x
		arm_sub_f32(&exp4,&exp1,&exp4,1);//exp4=p4+x*p3+p2*x*x
		arm_mult_f32(&exp3, &exp2, &exp3, 1);//exp3=x*x*x
        exp1= 2.142e-05;
        arm_mult_f32(&exp1, &exp3, &exp1, 1);//exp1=p1*x*x*x
        arm_add_f32(&exp1, &exp4, &exp4, 1);//exp4=p4+x*p3+x*x*p2+x*x*x*p1
        M6020s_Pitch.outCurrent += 1.5f * exp4;
    }
  
}

#endif

void FeedForward_Pitch_Accelerate(void)
{
		float Angle_Cloud = ControlMes.yaw_realAngle-Cloud_Yaw_Center;
		if(Angle_Cloud>4096)
		{
			Angle_Cloud-=8192;
		}
		angle_hd=Angle_Cloud/8192.0f*2*PI;
	 
		Forward_Accelerate =Saber_Angle.X_Acc * cos(angle_hd) + Saber_Angle.Y_Acc* sin(angle_hd);
		if(Forward_Accelerate * param > max )
			max=Forward_Accelerate * param;
		if(Forward_Accelerate *param < max)
			min=Forward_Accelerate *param ;
		M6020s_Pitch.outCurrent += Forward_Accelerate * param;
}
