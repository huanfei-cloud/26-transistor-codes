/**
 * @file Cloud_control.c
 * @author Cyx
 * @brief ��̨����ģ�飬������̨ƫ��������PID���ƺͿ������˲�
 * @version 0.1
 * @date 2023-08-15
 * @copyright ��Ȩ��Ϣ
 */
#include "Cloud_Control.h"  // ������̨����ͷ�ļ�

/************���PID����������***********/
positionpid_t M6020s_YawIPID;          // ƫ�������ڻ�PID����������
positionpid_t M6020s_Yaw_SpeedPID;     // ƫ�����ٶȻ�PID��δʹ�ã�
positionpid_t M6020s_YawOPID;          // ƫ�����⻷PID��λ�û���
positionpid_t AutoAim_M6020s_YawIPID;  // �Զ���׼ģʽ�µ��ڻ�PID
positionpid_t AutoAim_M6020s_YawOPID;  // �Զ���׼ģʽ�µ��⻷PID
/************���PID END***********/

/****************�������˲����ṹ�崴��*****************/
One_Kalman_t Cloud_YawMotorAngle_Error_Kalman;  // ƫ����Ƕ��������˲�
One_Kalman_t Cloud_YawCurrent_Kalman;           // �Զ�ģʽ�����������˲�
One_Kalman_t Cloud_YawCurrent_Kalman_manul;     // �ֶ�ģʽ�����������˲�
/****************�������˲��ṹ�崴�� End*****************/

/********��������********/
Cloud_t Cloud;            // ��̨����ȫ�ֽṹ��
float Control_Self_Yaw;   // �������ƫ���ǣ�δʹ�ã�
float shit;               // δʹ�õ���ʱ����������ɾ����
/********��������********/
float Linear = 2.75f;             // �������ͺ󲹳�ϵ��
float Setup_Angleoffset = -3624;  // ��̨��װ�Ƕ�ƫ����
uint8_t kk = 8;                   // �⻷PID����Ƶ�ʿ��Ʋ���
/********ȫ�ֱ�������********/
extern M6020s_t* M6020_Array[1];  // �������ָ�루����0��ӦID1��
extern Saber_Angle_t Saber_Angle; // �ⲿ����������ǽǶ�����

/********��������********/
void Cloud_Init(void);            // ��̨��ʼ��
void Cloud_Yaw_Angle_Set(void);   // ƫ����Ƕȿ���
void Cloud_Sport_Out(void);       // ��̨�������
void Cloud_Self_Yaw(void);        // ���鹦�ܣ�δʵ�֣�
void PID_Clear_Yaw(void);         // PID�������

/***************����ӿڶ���***************/
Cloud_FUN_t Cloud_FUN = Cloud_FUNGroundInit;  // ��̨���ܺ����ṹ���ʼ��
#undef Cloud_FUNGroundInit  // ȡ��������ʼ���궨��

/**
 * @brief  ��̨��ʼ�������ò�������λ��̨
 * @param  None
 * @retval None
 */
void Cloud_Init(void)
{
    // ��ʼ��Ŀ��Ƕȣ���ǰ����Ƕ� + ������ƫ����ת��ֵ
    // �����ǽǶ�(��)ת��Ϊ������ֵ(0-8191��Χ)
    Cloud.Target_Yaw = M6020s_Yaw.realAngle + Saber_Angle.Yaw / 360.0f * 8192.0f; 

    // ��ʼ���������˲���
    One_Kalman_Create(&Cloud_YawMotorAngle_Error_Kalman, 1, 10);  // Q=1, R=10
    One_Kalman_Create(&Cloud_YAWODKalman, 1, 10);                // δ�������˲����������Ǳ���
    One_Kalman_Create(&Cloud_YawCurrent_Kalman, 1, 6);           // �Զ�ģʽ�˲���
    One_Kalman_Create(&Cloud_YawCurrent_Kalman_manul, 1, 6);     // �ֶ�ģʽ�˲���
}

/**
  * @brief  ���ƫ����PID����������
  * @param  void
  * @retval void
  */
void PID_Clear_Yaw(void)
{
    Clear_PositionPIDData(&M6020s_YawIPID);  // ����ڻ�PID����
    Clear_PositionPIDData(&M6020s_YawOPID);  // ����⻷PID����
}

/**
  * @brief  ƫ����Ƕȿ��ƣ����ĺ�����
  * @param  void
  * @retval void
  */
void Cloud_Yaw_Angle_Set(void)
{
    /*********************** ��̨�Ƕȳ�ʼ������ ***********************/
    // ��������ݸ���֡����Чʱ����30֡��������Ŀ��Ƕ�
    if(M6020s_Yaw.InfoUpdateFrame <= 30) {
        Cloud.Target_Yaw = Saber_Angle.Yaw /360.0f * 8192.0f + M6020s_Yaw.realAngle;
    }

    // Ŀ��Ƕȹ�һ����0-8191��Χ����Ӧ������һȦ��
    if (Cloud.Target_Yaw > 8192) Cloud.Target_Yaw -= 8192;
    else if (Cloud.Target_Yaw < -8192) Cloud.Target_Yaw += 8192;

    /*********************** ����ϵת�� ***********************/
    // ��������ƫ����ת��Ϊ������ֵ��Χ
    float Angle_Yaw_Chassis = Saber_Angle.Yaw / 360.0f * 8192.0f;
    // ������������ϵ�µ���̨�Ƕȣ����̽Ƕ� + ����Ƕȣ�
    float Angle_Yaw_Cloud = M6020s_Yaw.realAngle + Angle_Yaw_Chassis;
    
    // �Ƕȹ�һ��������-4096~4096��Χ��
    if (Angle_Yaw_Cloud > 4096) Angle_Yaw_Cloud -= 8192;
    else if (Angle_Yaw_Cloud < -4096) Angle_Yaw_Cloud += 8192;
    
    ControlMes.yaw_realAngle = Angle_Yaw_Cloud;  // ���¿�����Ϣ�е�ʵʱ�Ƕ�

    /*********************** �Ƕ�ƫ����� ***********************/
    // ���㵱ǰ�Ƕ���Ŀ��Ƕȵ�ƫ������������Ǵ����ӳ�
    float Delta_Yaw = Angle_Yaw_Cloud - Cloud.Target_Yaw + Linear * Saber_Angle.Z_Vel;
    
    // ƫ��Ƕȹ�һ������
    if (Delta_Yaw <= -4096) Delta_Yaw += 8192;
    else if (Delta_Yaw >= 4096) Delta_Yaw -= 8192; 

    /*********************** ����ģʽѡ�� ***********************/
    static uint8_t time = 5;  // �⻷PID�����ʱ��
    
    // �ֶ�����ģʽ
    if(ControlMes.AutoAimFlag == 0) {
        // ����������С�Ƕ�ƫ����Ϊ��
        if(Delta_Yaw < 5 && Delta_Yaw > -5) Delta_Yaw = 0; 
        
        // �������˲�ƽ���Ƕ�ƫ��
        Delta_Yaw = One_Kalman_Filter(&Cloud_YawMotorAngle_Error_Kalman, Delta_Yaw);
        
        // �⻷PID���㣨��Ƶִ�У�
        if(time >= kk) {
            M6020s_Yaw.targetSpeed = Position_PID(&M6020s_YawOPID, 0, Delta_Yaw);
            time = 0;  // ���ü�ʱ��
        }
        time++;  // ��ʱ������
        
        // �ڻ�PID���㣨��ģ��PID��
        M6020s_Yaw.outCurrent = Position_PID_Yaw(&M6020s_YawIPID, &FuzzyPID_Yaw, 
                                                M6020s_Yaw.targetSpeed, M6020s_Yaw.realSpeed);
        // ��������˲�
        M6020s_Yaw.outCurrent = One_Kalman_Filter(&Cloud_YawCurrent_Kalman_manul, M6020s_Yaw.outCurrent);
    } 
    // �Զ���׼ģʽ
    else if(ControlMes.AutoAimFlag == 1) {
        // ��������
        if(Delta_Yaw < 10 && Delta_Yaw > -10) Delta_Yaw = 0;
        
        // ʹ���Զ���׼ר��PID����Ŀ���ٶ�
        M6020s_Yaw.targetSpeed = Position_PID(&AutoAim_M6020s_YawOPID, 0, Delta_Yaw);
        
        // �ڻ�PID���㣨���Զ���׼ģ��PID��
        M6020s_Yaw.outCurrent = Position_PID_Yaw(&AutoAim_M6020s_YawIPID, &FuzzyPID_AimYaw,
                                                M6020s_Yaw.targetSpeed, M6020s_Yaw.realSpeed);
        // ��������˲�
        M6020s_Yaw.outCurrent = One_Kalman_Filter(&Cloud_YawCurrent_Kalman, M6020s_Yaw.outCurrent);
    }
}

/**
  * @brief  ��̨�����������
  * @param  void
  * @retval void
  */
void Cloud_Sport_Out(void)
{
    /********** ״̬��� **********/
    // ¼��ģʽֱ�ӷ���
    if(ControlMes.modelFlag == model_Record) {
        M6020s_Yaw.InfoUpdateFrame = 0;
        return;
    } 
    // ������ݸ���ʱִ�нǶȿ���
    else if(M6020s_Yaw.InfoUpdateFlag == 1) {
        Cloud_FUN.Cloud_Yaw_Angle_Set();  // ���ýǶȿ��ƺ���
    } 
    // �޸���ʱֱ�ӷ���
    else {
        return;
    }

    uint8_t data[8] = {0};  // CAN�������ݻ�����

    /********** ��̨�Ƕȷ��� **********/
    // �����ƫ��������̨�Ƕ�
    float Angle_Cloud = M6020s_Yaw.realAngle + Setup_Angleoffset;
    // �Ƕȹ�һ����-4096~4096��
    if(Angle_Cloud > 4096) Angle_Cloud -= 8192;
    else if (Angle_Cloud < -4096) Angle_Cloud += 8192;
    // ���ͽǶȸ�����ģ�飨ת��Ϊ�ȣ�
    steer_getangle( Angle_Cloud / 8192.0f * 360);

    /********** ���������� **********/
    // �����ĸ�����ĵ���ֵ
    M6020_Fun.M6020_setVoltage(M6020s_Yaw.outCurrent, 
                              M6020s_Yaw.outCurrent,
                              M6020s_Yaw.outCurrent, 
                              M6020s_Yaw.outCurrent, 
                              data);
    // ͨ��CAN���͵���ָ��
    Can_Fun.CAN_SendData(CAN_SendHandle, &hcan1, CAN_ID_STD, M6020_SENDID, data);
}