#include "kalman_filter.h"

/**    
  * @author  Cyx
  * 一阶卡尔曼滤波器来自RoboMaster论坛  
  * 一维卡尔曼滤波器                     
  * 使用时先定义一个kalman指针，然后调用Kalman_Create()创建一个滤波器 
  * 每次读取到传感器数据后即可调用KalmanFilter()来对数据进行滤波
  *         一阶使用示例                                             
  *         One_Kalman_t p;               //定义一个卡尔曼滤波器结构体                                                 
  *         float SersorData;             //需要进行滤波的数据                                          
  *         Kalman_Create(&p,20,200);      //初始化该滤波器的Q=20 R=200参数                                                  
  *         while(1)                                                                
  *         {                                                                            
  *            SersorData = sersor();                     //获取数据                                           
  *            SersorData = One_Kalman_Filter(&p,SersorData);  //对数据进行滤波                                                                            
  *         }     
  *			二阶使用示例
  *	
  *	  
  */

/**
  * @name   kalman_Create
  * @brief  创建一个卡尔曼滤波器
  * @param  p:  滤波器
  *         T_Q:系统噪声协方差
  *         T_R:测量噪声协方差
  *         
  * @retval none
  * @attention 
  *	R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
  *	反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
  */
void One_Kalman_Create(One_Kalman_t *Kal, float T_Q, float T_R)
{
  Kal->X_Last = (float)0;
  Kal->P_Last = 0;
  Kal->Q = T_Q;
  Kal->R = T_R;
  Kal->A = 1;
  Kal->B = 0;
  Kal->H = 1;
  Kal->X_Mid = Kal->X_Last;
}

/**
  * @name   Kalman_Filter
  * @brief  卡尔曼滤波器
  * @param  Kal:  滤波器
  *         Data:待滤波数据
  * @retval 滤波后的数据
  * @attention 
  *	Data(Z(k))是系统输入,即测量值   
  *	X(k|k)是卡尔曼滤波后的值,即最终输出
  *	A=1 B=0 H=1 I=1  W(K)(过程噪声)/V(k)(观测/系统噪声)是高斯白噪声,叠加在测量值上了,可以不用管
  *	以下是卡尔曼的5个核心公式
  *	一阶H'即为它本身,否则为转置矩阵
  *	公式(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
  *	公式(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
  *	公式(3)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  *	公式(4)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  *	公式(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
  */

float One_Kalman_Filter(One_Kalman_t *Kal, float Data)
{
  Kal->X_Mid = Kal->A * Kal->X_Last;                    	//(1)当阶段的先验估计
  Kal->P_Mid = Kal->A * Kal->P_Last + Kal->Q;             	//(2)当阶段先验估计的协方差
  Kal->Kt = Kal->P_Mid / (Kal->P_Mid + Kal->R);           	//(3)卡尔曼增益更新(取一阶H=1)
  Kal->X_Now = Kal->X_Mid + Kal->Kt * (Data - Kal->X_Mid); 	//(4)计算当阶段的最优估计
  Kal->P_Now = (1 - Kal->Kt) * Kal->P_Mid;              	//(5)计算当阶段最优估计的协方差
  Kal->P_Last = Kal->P_Now;                           		//状态更新
  Kal->X_Last = Kal->X_Now;									//状态更新
  return Kal->X_Now; 										//输出最优估计x(k|k)
}
/**    
  * 二阶卡尔曼滤波器
  * 
  */
/************************************************/

float matrix_value1;
float matrix_value2;
/**  
  * @name   kalman_filter_init
  * @brief  二阶卡尔曼滤波器创建
  * @param  F:滤波器
  *         I:待输入数据
  * @retval 
  */
void Two_Kalman_Filter_Init(Two_Kalman_Filter_t *F, Two_Kalman_Filter_Init_t *I)
{
  mat_init(&F->xhat, 2, 1, (float *)I->xhat_data);
  mat_init(&F->xhatminus, 2, 1, (float *)I->xhatminus_data);
  mat_init(&F->z, 2, 1, (float *)I->z_data);
  mat_init(&F->A, 2, 2, (float *)I->A_data);
  mat_init(&F->H, 2, 2, (float *)I->H_data);
  mat_init(&F->Q, 2, 2, (float *)I->Q_data);
  mat_init(&F->R, 2, 2, (float *)I->R_data);
  mat_init(&F->P, 2, 2, (float *)I->P_data);
  mat_init(&F->Pminus, 2, 2, (float *)I->Pminus_data);
  mat_init(&F->K, 2, 2, (float *)I->K_data);
  mat_init(&F->AT, 2, 2, (float *)I->AT_data);
  mat_trans(&F->A, &F->AT);
  mat_init(&F->HT, 2, 2, (float *)I->HT_data);
  mat_trans(&F->H, &F->HT);
  //  matrix_value2 = F->A.pData[1];
}

// xhatminus==x(k|k-1)  xhat==X(k-1|k-1)
// Pminus==p(k|k-1)     P==p(k-1|k-1)    AT==A'
// HT==H'   K==kg(k)    I=1
//

/**
  *@param 卡尔曼参数结构体
  *@param 角度
  *@param 速度
*/

/*********************************************/
float *Two_Kalman_Filter_Calc(Two_Kalman_Filter_t *F, float signal1, float signal2)
{
  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
  mat TEMP, TEMP21;

  mat_init(&TEMP, 2, 2, (float *)TEMP_data);     //
  mat_init(&TEMP21, 2, 1, (float *)TEMP_data21); //

  F->z.pData[0] = signal1; //z(k)
  F->z.pData[1] = signal2; //z(k)

  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus); //  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)

  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);  //   p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_mult(&F->Pminus, &F->AT, &TEMP); //  p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_add(&TEMP, &F->Q, &F->Pminus);   //    p(k|k-1) = A*p(k-1|k-1)*A'+Q

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &F->K); //  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_mult(&F->K, &F->HT, &TEMP);     //      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_add(&TEMP, &F->R, &F->K);       //        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

  mat_inv(&F->K, &F->P);               //
  mat_mult(&F->Pminus, &F->HT, &TEMP); //
  mat_mult(&TEMP, &F->P, &F->K);       //

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21);   //      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_sub(&F->z, &TEMP21, &F->xhat);         //            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_mult(&F->K, &F->xhat, &TEMP21);        //           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_add(&F->xhatminus, &TEMP21, &F->xhat); //    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P); //            p(k|k) = (I-kg(k)*H)*P(k|k-1)
  mat_sub(&F->Q, &F->P, &TEMP);  //
  mat_mult(&TEMP, &F->Pminus, &F->P);

  matrix_value1 = F->xhat.pData[0];
  matrix_value2 = F->xhat.pData[1];

  F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];
  return F->filtered_value;
}

