/**
 * @file Gimbal_Chassis_Pitch_Angle.c
 * @author ZS
 * @brief
 * @version 1.0
 * @date 2024-11-2
 *
 * @copyright 
 *
 */

#include <math.h>  
#include "Saber_C3.h"
#include "BSP_BoardCommunication.h"
#include "Gimbal_Chassis_Pitch_Angle.h"
  
// 定义PI常量  
#define M_PI 3.1415926536f 


// 将角度从度转换为弧度  
float degreesToRadians(float degrees) 
{  
    return degrees * M_PI / 180.0f;  
}  
    
float Pitch_Compensation;
float Gimbal_Chassis_offset = 38;   //GM6020的零度位置和底盘yaw轴零度之间的夹角（偏移量）
    
/**
 * @brief         计算云台pitch轴补偿量的函数
 * @param         底盘Saber各轴角度
 * @param         云台相对于底盘的yaw角度
 * @retval        云台pitch轴补偿量
 */ 
float calculateGimbalPitchCompensation(IMUData chassisIMUData, float gimbalYawAngle) 
{  
  
    // 底盘的pitch和roll角度将影响云台的实际pitch角度  
    // 当云台绕yaw轴旋转后，底盘的pitch将影响云台的roll（相对于云台自身坐标系）  
    // 而底盘的roll将影响云台的pitch（但需要通过云台yaw角度的旋转矩阵来变换）  
  
    // 由于云台只绕yaw轴旋转，我们可以使用二维旋转公式来近似计算   
  
    // 云台pitch轴的补偿量可以看作是底盘pitch和roll角度在云台坐标系中的投影  
    // 这个投影可以通过旋转矩阵来计算，但在这里我们使用近似方法  
  
    // 近似计算云台pitch轴的补偿量  
    // 由于云台绕yaw旋转了gimbalYawAngle，底盘的pitch现在影响云台的"等效roll"  
    // 而底盘的roll现在（通过旋转）影响云台的"等效pitch"  
    // 但由于我们想要保持云台相对于地面的pitch不变，我们只需要考虑底盘pitch的"等效影响"  
    // 这个"等效影响"可以通过底盘pitch乘以一个与云台yaw角度相关的系数来近似（这个系数是cos(gimbalYawAngle)）  
    // 同时，我们还需要加上一个由于底盘roll引起的"等效pitch"变化，这个变化可以通过底盘roll乘以sin(gimbalYawAngle)来近似   
  
    float compensation = chassisIMUData.pitch * cos(gimbalYawAngle) + chassisIMUData.roll * sin(gimbalYawAngle);  
  
    return compensation;   // 返回补偿量  
}  
 
/**
 * @brief  将Saber的pitch角的值赋给ControlMes结构体中的对应量
 * @param  None
 * @retval None
 */
void Gimbal_Chassis_Pitch_Translate(void) 
{  
    IMUData chassisIMUData = { .pitch = Saber_Angle.Pitch, .yaw = Saber_Angle.Yaw, .roll = Saber_Angle.RoLL }; // 底盘Saber数据
		/***********死区begin************/
		if(chassisIMUData.pitch < 1.2f && chassisIMUData.pitch > -1.2f)
		{
			chassisIMUData.pitch = 0;
		}
		if(chassisIMUData.roll < 1.2f && chassisIMUData.roll > -1.2f)
		{
			chassisIMUData.roll = 0;
		}
		/***********死区end************/
		
		chassisIMUData.pitch = degreesToRadians(chassisIMUData.pitch);   //角度转为弧度
		chassisIMUData.roll = degreesToRadians(chassisIMUData.roll);
		chassisIMUData.yaw = degreesToRadians(chassisIMUData.yaw);
		
    float gimbalYawAngle = ControlMes.yaw_realAngle / 8192.f * 360.f - Saber_Angle.Yaw - Gimbal_Chassis_offset; // 云台相对于底盘的yaw角度 
		gimbalYawAngle = degreesToRadians(gimbalYawAngle);
  
    // 调用函数计算补偿量  
    Pitch_Compensation = calculateGimbalPitchCompensation(chassisIMUData, gimbalYawAngle);
    ControlMes.Gimbal_Chassis_Pitch_Angle = (int16_t)(Pitch_Compensation / M_PI *8192.f);
    
}
