/**
 * @file FeedForward.c
 * @author Why
 * @brief 前馈控制,取实测量的百分之八十
 * @version 0.1
 * @date 2023-08-28
 * @attention 一定要在PID算出电流之后再重新引用这个函数，因为是直接加在输出电流上了
				这里只是放这个文件，里边的东西都是没有用到的旧版本的另一个板子的东西
 *
 * @copyright
 *
 */

#include "FeedForward.h"
// #include "Saber_C3.h"

/**************用户数据定义****************/
void FeedForward_Fric(void);
void FeedForward_Chassis(void);
void FeedForward_Pitch(void);
void FeedForward_Yaw(void);
void Compensator_Yaw(void);

/****************接口定义******************/
FeedForward_FUN_t FeedForward_FUN = FeedForward_FunGroundInit;
#undef FeedForward_FunGroundInit

#include "FeedForward.h"

/**
 * @brief   给摩擦轮发送前馈
 * @param   M3508的外部接口
 * @retval  void
 */
void FeedForward_Fric()
{
	/* 平衡摩擦力 */
	if (M3508_Array[FricL_Wheel].realSpeed < 1000)
		M3508_Array[FricL_Wheel].outCurrent +=
			(212 - 0.124f * (float)M3508_Array[FricL_Wheel].realSpeed);
	else
		M3508_Array[FricL_Wheel].outCurrent +=
			(88 - 0.01f * (float)(M3508_Array[FricL_Wheel].realSpeed - 1000));

	if (M3508_Array[FricR_Wheel].realSpeed < 1000)
		M3508_Array[FricR_Wheel].outCurrent +=
			(212 - 0.124f * (float)M3508_Array[FricR_Wheel].realSpeed);
	else
		M3508_Array[FricR_Wheel].outCurrent +=
			(88 - 0.01f * (float)(M3508_Array[FricR_Wheel].realSpeed - 1000));

	/* 平衡弹丸进摩擦轮的巨大阻力，减少掉速 */
	M3508_Array[FricL_Wheel].outCurrent += 5000;
	M3508_Array[FricR_Wheel].outCurrent += 5000;
}

/**
 * @brief   给摩擦轮发送前馈
 * @param
 * @retval  void
 */
void FeedForward_Chassis()
{
	/* 平衡摩擦力 */
}

/**
 * @brief   给pitch轴电机发送前馈
 * @param   云台和云台电机的外部接口
 * @retval  void
 */
// void FeedForward_Pitch()
//{
//	/* 临时变量 */
//	float exp1,exp2, exp3, exp4;

/* 平衡Pitch重力 */
/* 从大转角到小转角 y = 0.074 * x ^ 2 -20.9418 * x + 1291.57 */
//	exp1 = M6020s_Pitch.realAngle;
//	if(Cloud.Target_Pitch > M6020s_Pitch.realAngle + Pitch_Margin)
//	{
//		//arm_mult_f32(&exp1, &exp1, &exp3, 1);
//		exp2 = 0.074f;
//		//arm_mult_f32(&exp3, &exp2, &exp3, 1);
//		exp2 = 20.9418f;
//		//arm_mult_f32(&exp2, &exp1, &exp4, 1);
//		//arm_sub_f32(&exp3, &exp4, &exp3, 1);
//		exp2 = 1291.57f;
//		//arm_add_f32(&exp3, &exp2, &exp4, 1);
//		M6020s_Pitch.outCurrent += 1.125f * exp4;
//	}
//	/* 从小转角到大转角 y = 0.0065 * x ^ 2 -19.0685 * x + 1225.57 */
//	else if(Cloud.Target_Pitch < M6020s_Pitch.realAngle - Pitch_Margin)
//	{
//		//arm_mult_f32(&exp1, &exp1, &exp3, 1);
//		exp2 = 0.0065f;
//		//arm_mult_f32(&exp3, &exp2, &exp3, 1);
//		exp2 = 19.0685f;
//		//arm_mult_f32(&exp2, &exp1, &exp4, 1);
//		//arm_sub_f32(&exp3, &exp4, &exp3, 1);
//		exp2 = 1225.57f;
//		//arm_add_f32(&exp3, &exp2, &exp4, 1);
//		M6020s_Pitch.outCurrent += exp4;
//	}
//	else M6020s_Pitch.outCurrent += 1.125f * exp4;
//}

/**
 * @brief   给Yaw轴电机发送前馈
 * @param   云台和云台电机的外部接口
 * @retval  void
 */
// void FeedForward_Yaw()
//{
//	/* Yaw轴的前馈 */
//	M6020s_Yaw.targetSpeed -= Saber_Angle.Z_Vel / 6.0f;  // 从角度每秒转化到转每分钟
// }

/**
 * @brief   给Yaw轴电机进行扰动的补偿
 * @param   云台和云台电机的外部接口
 * @retval  void
 */
void Compensator_Yaw()
{
	/* 临时变量 */
	float exp1, exp2, exp3, exp4;
	/* constant for yaw */
	static float Coefficient_2 = Yaw_J * Yaw_L / (Yaw_K * Yaw_K);
	static float Coefficient_1 = (Yaw_J * Yaw_R + Yaw_L * Yaw_b) / (Yaw_K * Yaw_K);
	static float Coefficient_0 = (Yaw_b * Yaw_R) / (Yaw_K * Yaw_K) + 1;
	static float speed_last = 0;
	static float error = 0;
	static float error_last = 0;
	static float eerror = 0;

	/* 依次是二阶导、一阶导、零阶导的值,此处得到的值应为负值，单位伏特 */
	//	exp4 = Saber_Angle.Z_Vel / 57.29578f; // 陀螺仪是角度每秒，转化为弧度
	error = exp4 - speed_last;
	eerror = error - error_last;
	exp3 = Coefficient_2 * eerror;
	exp2 = Coefficient_1 * error;
	exp1 = Coefficient_0 * exp4;
	speed_last = exp4;
	error_last = error;

	M6020s_Yaw.outCurrent += 1000 * (exp1 + exp2 + exp3);
}
