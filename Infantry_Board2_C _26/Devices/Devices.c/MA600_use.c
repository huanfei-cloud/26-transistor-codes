/**
 * @file MA600.c
 * @author xhf
 * @brief 
 * @version 0.1
 * @date 2025-05-06
 * 
 */
#include "MA600_use.h"

/********全局变量定义********/
hMA600_TypeDef MA600s[4];
uint8_t MA600_flag[2] = {0};

/**
 * @brief 四个编码器初始化
 * @param None
 * @retval None
 */
void MA600sInit(void)
{
	MA600_HandleInit(&MA600s[0],&hspi2, MA600_CS1_Port, MA600_CS1_Pin);
	MA600_HandleInit(&MA600s[1],&hspi2, MA600_CS2_Port, MA600_CS2_Pin);
	MA600_HandleInit(&MA600s[2],&hspi2, MA600_CS3_Port, MA600_CS3_Pin);
	MA600_HandleInit(&MA600s[3],&hspi2, MA600_CS4_Port, MA600_CS4_Pin);
}

/**
 * @brief 单个编码器设置为菊花链模式
 * @param None
 * @retval None
 */
void setChrysanthemumChains(void)
{
	//将NVM中的值恢复到寄存器里
	MA600_Restore_All(&MA600s[0]);
	//解锁寄存器10
	MA600_Write_Reg(&MA600s[0],MA600_Reg_UR10, 0x01);
	MA600_flag[0] = MA600_Read_Reg(&MA600s[0],MA600_Reg_UR10);
	
	//将DAISY设置为1
	MA600_Write_Reg(&MA600s[0],MA600_Reg_DAISY_RWM , 0x01);
	MA600_flag[1] = MA600_Read_Reg(&MA600s[0],MA600_Reg_DAISY_RWM);
	
	//将块0存储到NVM里
	MA600_Store_Single(&MA600s[0], 0x00);

	//锁定寄存器10
	MA600_Write_Reg(&MA600s[0],MA600_Reg_UR10, 0x00);
	MA600_flag[0] = MA600_Read_Reg(&MA600s[0],MA600_Reg_UR10);
}

/**
 * @brief 读取菊花链四个编码器的角度值
 * @param None
 * @retval None
 */
void MA600s_Read_DaisyChain(void)
{
	// 共用同一个SPI接口，假设所有编码器共享CS线
    GPIO_TypeDef* cs_gpio = MA600s[0].spi_CS_GPIOx;
    uint16_t cs_pin = MA600s[0].spi_CS_Pin;
	
	// 片选使能
    cs_gpio->ODR &= ~cs_pin;
    
  // 菊花链读取流程
    uint16_t dummyTx = 0xFFFF; // 全1作为填充数据
    uint16_t rxBuffer[4] = {0};
		
		// 发送4个16位数据（触发4个编码器输出）
    for(int i = 0; i < 4; i++)
    {
        HAL_SPI_TransmitReceive(MA600s[i].hspi, (uint8_t*)&dummyTx, (uint8_t*)&rxBuffer[i],1,HAL_MAX_DELAY);
    }
		
		 // 片选失能
    cs_gpio->ODR |= cs_pin;
		
		// 数据解析（注意：菊花链中第一个发送的数据对应最后一个编码器）
    for(int i = 0; i < 4; i++)
    {
        MA600s[3-i].Angle = rxBuffer[i] & 0x3FFF; // MA600有效数据为14位并更新句柄中的角度值
			  MA600s[3-i].Circle = MA600s[3-i].Angle / 360.0f;  //转化成圈数
    }

}

/**
 * @brief 读取底盘四个编码器的角度值
 * @param None
 * @retval None
 */
void MA600s_Read_Chassis(void)
{
		MA600_Get_Angle(&MA600s[0]);
		MA600_Get_Angle(&MA600s[1]);
		MA600_Get_Angle(&MA600s[2]);
		MA600_Get_Angle(&MA600s[3]);
}










