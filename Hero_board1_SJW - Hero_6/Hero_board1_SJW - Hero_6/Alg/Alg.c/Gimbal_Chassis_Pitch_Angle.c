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
#ifndef M_PI  
#define M_PI 3.1415926536 
#endif  
  
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
		if(chassisIMUData.pitch < 1.0f && chassisIMUData.pitch > -1.0f)
		{
			chassisIMUData.pitch = 0;
		}
		if(chassisIMUData.roll < 1.0f && chassisIMUData.roll > -1.0f)
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


  
  
// 计算旋转矩阵（基于ZYX欧拉角，即先绕Z轴转yaw，再绕Y轴转pitch，最后绕X轴转roll）  
//RotationMatrix createRotationMatrixFromEulerAngles(float yaw, float pitch, float roll) 
//{  
//    yaw = degreesToRadians(yaw);  
//    pitch = degreesToRadians(pitch);  
//    roll = degreesToRadians(roll);  
//  
//    RotationMatrix R;  
//    R.row1.x = cos(yaw) * cos(roll) - sin(yaw) * sin(pitch) * sin(roll);  
//    R.row1.y = sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll);  
//    R.row1.z = -sin(roll) * cos(pitch);  
//  
//    R.row2.x = -cos(yaw) * sin(roll) - sin(yaw) * sin(pitch) * cos(roll);  
//    R.row2.y = -sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll);  
//    R.row2.z = cos(roll) * cos(pitch);  
//  
//    R.row3.x = sin(yaw) * cos(pitch);  
//    R.row3.y = cos(yaw) * cos(pitch);  
//    R.row3.z = sin(pitch);  
//  
//    return R;  
//}  
//  
//// 计算云台pitch轴补偿量的函数  
//float calculateGimbalPitchCompensation(RotationMatrix chassisRotationMatrix, float gimbalYawAngle, Vector3 chassisPitchVector) 
//{  
//    // gimbalYawAngle是云台相对于底盘的yaw旋转角度（以度为单位）  
//    // chassisPitchVector是底盘pitch轴在底盘坐标系中的表示（通常为{0, 1, 0}）  
//  
//    gimbalYawAngle = degreesToRadians(gimbalYawAngle);  
//  
//    // 创建表示云台相对于底盘旋转的旋转矩阵（仅绕yaw轴旋转）  
//    RotationMatrix gimbalYawRotationMatrix = {  
//        {cos(gimbalYawAngle), 0, sin(gimbalYawAngle)},  
//        {0, 1, 0},  
//        {-sin(gimbalYawAngle), 0, cos(gimbalYawAngle)}  
//    };  
//  
//    // 将底盘的旋转矩阵与云台的yaw旋转矩阵相乘，得到云台在世界坐标系中的旋转矩阵  
//    RotationMatrix gimbalWorldRotationMatrix = {0};  
//    for (int i = 0; i < 3; i++) {  
//        for (int j = 0; j < 3; j++) {  
//            gimbalWorldRotationMatrix.row1.x += gimbalYawRotationMatrix.row1.x * chassisRotationMatrix.row1.x +  
//                                               gimbalYawRotationMatrix.row1.y * chassisRotationMatrix.row2.x +  
//                                               gimbalYawRotationMatrix.row1.z * chassisRotationMatrix.row3.x;  
//            if (i == 0) {  
//                gimbalWorldRotationMatrix.row1.y += gimbalYawRotationMatrix.row1.x * chassisRotationMatrix.row1.y +  
//                                                   gimbalYawRotationMatrix.row1.y * chassisRotationMatrix.row2.y +  
//                                                   gimbalYawRotationMatrix.row1.z * chassisRotationMatrix.row3.y;  
//                gimbalWorldRotationMatrix.row1.z += gimbalYawRotationMatrix.row1.x * chassisRotationMatrix.row1.z +  
//                                                   gimbalYawRotationMatrix.row1.y * chassisRotationMatrix.row2.z +  
//                                                   gimbalYawRotationMatrix.row1.z * chassisRotationMatrix.row3.z;  
//            } else if (i == 1) {  
//                gimbalWorldRotationMatrix.row2.x = gimbalYawRotationMatrix.row2.x * chassisRotationMatrix.row1.x +  
//                                                  gimbalYawRotationMatrix.row2.y * chassisRotationMatrix.row2.x +  
//                                                  gimbalYawRotationMatrix.row2.z * chassisRotationMatrix.row3.x;  
//                gimbalWorldRotationMatrix.row2.y = gimbalYawRotationMatrix.row2.x * chassisRotationMatrix.row1.y +  
//                                                  gimbalYawRotationMatrix.row2.y * chassisRotationMatrix.row2.y +  
//                                                  gimbalYawRotationMatrix.row2.z * chassisRotationMatrix.row3.y;  
//                gimbalWorldRotationMatrix.row2.z = gimbalYawRotationMatrix.row2.x * chassisRotationMatrix.row1.z +  
//                                                  gimbalYawRotationMatrix.row2.y * chassisRotationMatrix.row2.z +  
//                                                  gimbalYawRotationMatrix.row2.z * chassisRotationMatrix.row3.z;  
//            } else if (i == 2) {  
//                gimbalWorldRotationMatrix.row3.x = gimbalYawRotationMatrix.row3.x * chassisRotationMatrix.row1.x +  
//                                                  gimbalYawRotationMatrix.row3.y * chassisRotationMatrix.row2.x +  
//                                                  gimbalYawRotationMatrix.row3.z * chassisRotationMatrix.row3.x;  
//                gimbalWorldRotationMatrix.row3.y = gimbalYawRotationMatrix.row3.x * chassisRotationMatrix.row1.y +  
//                                                  gimbalYawRotationMatrix.row3.y * chassisRotationMatrix.row2.y +  
//                                                  gimbalYawRotationMatrix.row3.z * chassisRotationMatrix.row3.y;  
//                gimbalWorldRotationMatrix.row3.z = gimbalYawRotationMatrix.row3.x * chassisRotationMatrix.row1.z +  
//                                                  gimbalYawRotationMatrix.row3.y * chassisRotationMatrix.row2.z +  
//                                                  gimbalYawRotationMatrix.row3.z * chassisRotationMatrix.row3.z;  
//            }  
//        }  
//    }  
//  
//    // 计算云台在世界坐标系中想要保持的pitch方向向量（通常为{0, 1, 0}）  
//    Vector3 desiredPitchVector = {0, 1, 0};  
//  
//    // 将底盘的pitch轴向量转换到世界坐标系中  
//    Vector3 chassisPitchVectorInWorld = {  
//        chassisPitchVector.x * gimbalWorldRotationMatrix.row1.x + chassisPitchVector.y * gimbalWorldRotationMatrix.row2.x + chassisPitchVector.z * gimbalWorldRotationMatrix.row3.x,  
//        chassisPitchVector.x * gimbalWorldRotationMatrix.row1.y + chassisPitchVector.y * gimbalWorldRotationMatrix.row2.y + chassisPitchVector.z * gimbalWorldRotationMatrix.row3.y,  
//        chassisPitchVector.x * gimbalWorldRotationMatrix.row1.z + chassisPitchVector.y * gimbalWorldRotationMatrix.row2.z + chassisPitchVector.z * gimbalWorldRotationMatrix.row3.z  
//    };  
//  
//    // 计算补偿量，即底盘pitch轴在世界坐标系中的z分量（表示倾斜程度）的相反数  
//    // 因为我们想要保持云台相对于地面的水平，所以需要抵消底盘的倾斜  
//    float compensation = -chassisPitchVectorInWorld.z;  
//  
//    return compensation;  
//}  
//  

//void Gimbal_Chassis_Pitch_Translate(void) 
//{  
//    float chassisYaw = Saber_Angle.Yaw; // 底盘yaw角度（通常可以忽略，因为我们只关心pitch和roll对云台pitch的影响）  
//    float chassisPitch = Saber_Angle.Pitch ; // 底盘pitch角度  
//    float chassisRoll = Saber_Angle.RoLL; // 底盘roll角度  
//    float gimbalYawAngle = ControlMes.yaw_realAngle /8192.f *360.f - Saber_Angle.Yaw ; // 云台相对于底盘的yaw旋转角度  
//  
//    // 创建底盘旋转矩阵  
//    RotationMatrix chassisRotationMatrix = createRotationMatrixFromEulerAngles(chassisYaw, chassisPitch, chassisRoll);  
//  
//    // 底盘pitch轴在底盘坐标系中的表示  
//    Vector3 chassisPitchVector = {0, 1, 0};
//  
//    // 调用函数计算补偿量  
//    Pitch_Compensation = calculateGimbalPitchCompensation(chassisRotationMatrix, gimbalYawAngle, chassisPitchVector) + 1.f;  
//    ControlMes.Gimbal_Chassis_Pitch_Angle = (int16_t)(Pitch_Compensation / M_PI *8192.f);
//    
//}