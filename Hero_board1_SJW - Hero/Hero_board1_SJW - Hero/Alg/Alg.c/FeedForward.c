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

/**************用户数据定义****************/
void FeedForward_Fric(void);
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
	if (M3508_Array[Fric_Front_1].realSpeed < 1000)
		M3508_Array[Fric_Front_1].outCurrent +=
			(212 - 0.124f * (float)M3508_Array[Fric_Front_1].realSpeed);
	else
		M3508_Array[Fric_Front_1].outCurrent +=
			(88 - 0.01f * (float)(M3508_Array[Fric_Front_1].realSpeed - 1000));

	if (M3508_Array[Fric_Front_2].realSpeed < 1000)
		M3508_Array[Fric_Front_2].outCurrent +=
			(212 - 0.124f * (float)M3508_Array[Fric_Front_2].realSpeed);
	else
		M3508_Array[Fric_Front_2].outCurrent +=
			(88 - 0.01f * (float)(M3508_Array[Fric_Front_2].realSpeed - 1000));

	if (M3508_Array[Fric_Front_3].realSpeed < 1000)
		M3508_Array[Fric_Front_3].outCurrent +=
			(212 - 0.124f * (float)M3508_Array[Fric_Front_3].realSpeed);
	else
		M3508_Array[Fric_Front_3].outCurrent +=
			(88 - 0.01f * (float)(M3508_Array[Fric_Front_3].realSpeed - 1000));

	if (M3508_Array[Fric_Back_1].realSpeed < 1000)
		M3508_Array[Fric_Back_1].outCurrent +=
			(212 - 0.124f * (float)M3508_Array[Fric_Back_1].realSpeed);
	else
		M3508_Array[Fric_Back_1].outCurrent +=
			(88 - 0.01f * (float)(M3508_Array[Fric_Back_1].realSpeed - 1000));

	if (M3508_Array[Fric_Back_2].realSpeed < 1000)
		M3508_Array[Fric_Back_2].outCurrent +=
			(212 - 0.124f * (float)M3508_Array[Fric_Back_2].realSpeed);
	else
		M3508_Array[Fric_Back_2].outCurrent +=
			(88 - 0.01f * (float)(M3508_Array[Fric_Back_2].realSpeed - 1000));

	if (M3508_Array[Fric_Back_3].realSpeed < 1000)
		M3508_Array[Fric_Back_3].outCurrent +=
			(212 - 0.124f * (float)M3508_Array[Fric_Back_3].realSpeed);
	else
		M3508_Array[Fric_Back_3].outCurrent +=
			(88 - 0.01f * (float)(M3508_Array[Fric_Back_3].realSpeed - 1000));

	/* 平衡弹丸进摩擦轮的巨大阻力，减少掉速 */
	M3508_Array[Fric_Front_1].outCurrent += 5000;
	M3508_Array[Fric_Front_2].outCurrent += 5000;
	M3508_Array[Fric_Front_3].outCurrent += 5000;
	M3508_Array[Fric_Back_1].outCurrent += 5000;
	M3508_Array[Fric_Back_2].outCurrent += 5000;
	M3508_Array[Fric_Back_3].outCurrent += 5000;
}


/**
 * @brief   给Yaw轴电机进行扰动的补偿
 * @param   云台和云台电机的外部接口
 * @retval  void
 */
void Compensator_Yaw()
{
//	/* 临时变量 */
//	float exp1, exp2, exp3, exp4;
//	/* constant for yaw */
//	static float Coefficient_2 = Yaw_J * Yaw_L / (Yaw_K * Yaw_K);
//	static float Coefficient_1 = (Yaw_J * Yaw_R + Yaw_L * Yaw_b) / (Yaw_K * Yaw_K);
//	static float Coefficient_0 = (Yaw_b * Yaw_R) / (Yaw_K * Yaw_K) + 1;
//	static float speed_last = 0;
//	static float error = 0;
//	static float error_last = 0;
//	static float eerror = 0;

//	/* 依次是二阶导、一阶导、零阶导的值,此处得到的值应为负值，单位伏特 */
//	//	exp4 = Saber_Angle.Z_Vel / 57.29578f; // 陀螺仪是角度每秒，转化为弧度
//	error = exp4 - speed_last;
//	eerror = error - error_last;
//	exp3 = Coefficient_2 * eerror;
//	exp2 = Coefficient_1 * error;
//	exp1 = Coefficient_0 * exp4;
//	speed_last = exp4;
//	error_last = error;

//	M6020s_Yaw.outCurrent += 1000 * (exp1 + exp2 + exp3);
}
