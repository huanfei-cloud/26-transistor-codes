#ifndef GIMBAL_CHASSIS_PITCH_ANGLE
#define	GIMBAL_CHASSIS_PITCH_ANGLE

//�Ի���Ϊ��λ
typedef struct 
{  
    float pitch;  
    float yaw;  
    float roll;  
} IMUData;

// ����һ����ʾ��ά�����Ľṹ��  
typedef struct {  
    float x;  
    float y;  
    float z;  
} Vector3;  
  
// ����һ����ʾ��ת����Ľṹ��  
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