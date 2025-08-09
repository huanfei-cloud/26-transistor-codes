#ifndef GIMBAL_CHASSIS_PITCH_ANGLE
#define	GIMBAL_CHASSIS_PITCH_ANGLE

//以弧度为单位
typedef struct 
{  
    float pitch;  
    float yaw;  
    float roll;  
} IMUData;

// 定义一个表示三维向量的结构体  
typedef struct {  
    float x;  
    float y;  
    float z;  
} Vector3;  
  
// 定义一个表示旋转矩阵的结构体  
typedef struct {  
    Vector3 row1;  
    Vector3 row2;  
    Vector3 row3;  
} RotationMatrix;  

extern float degreesToRadians(float degrees); 
extern float calculateGimbalPitchCompensation(IMUData chassisIMUData, float gimbalYawAngle) ;
//extern float calculateGimbalPitchCompensation(RotationMatrix chassisRotationMatrix, float gimbalYawAngle, Vector3 chassisPitchVector) ;

extern RotationMatrix createRotationMatrixFromEulerAngles(float yaw, float pitch, float roll) ;

extern void Gimbal_Chassis_Pitch_Translate(void) ;


#endif
