/*************************************************
Copyright (C),  Li_Jiang
File name: MA600.c
Author: Li_Jiang
Version: 1.0.2 
Date: 2024��6��10��
Description:  
		* ��Ҫʹ�ô������⣬��Ҫ����Ӧ��SPI�����ʼ��Ϊ16bitģʽ������
		
		��������Ϊÿ��MA600оƬ����һ�������hMA600_TypeDef��
		�ھ���м�¼�ű����ڵ�SPI�������Լ�Ƭѡ������Ϣ
		ͬʱ�ھ���м�¼����һ�εĵ�Ȧ�Ƕ�ֵ����ȦȦ��*�Լ���ת�ٶ�*�����*����Ŀ��Ҫͨ���Դű�Ĵ�����������������
		
		ÿ�������Ҫͨ������ MA600_HandleInit() �������г�ʼ�����˺���ʹ��ʱ������ݾ���ڵ���������оƬ����ͨ��
		
		������ĺ������� MA600_Get_Angle() ͨ�����ô˺������ԴӴű��ж������µĽǶ�ֵ����ʱΪ16��SPIʱ�����ڣ����Ҷ��صĽǶ�ֵ��ͬʱ���ڶ�Ӧ�ľ����
		��Ҫʵ�ֶ���תȦ���Ķ�ȡ���ǶԵ�ǰ�ٶȵĶ�ȡ������ͨ������MA600_Get_Speed_rpm() �� MA600_Get_MultiTurn() ������ǰ������ȷ���õ�оƬ��MA600_Reg_MTSP_PRT_PRTS_APRT_FTA_FTM �Ĵ���
		
		��һ��ʹ�ô�����������ͨ������ MA600_Read_Reg() ����������ָ���Ĵ����������Լ�ʹ�� MA600_Write_Reg() ��������ָ���Ĵ���д������
		���߽�ϱ����ʵ�ָ�����Ҫ�Դű���������Ը��ӷ���ʵ��ʹ������
		
		MA600�ű�����֧�ֶԼĴ�������ʵ�ַ���ʧ�Դ洢��ʵ�ַ�ʽΪ�ڶԼĴ�����ִֵ�����޸�֮����� MA600_Store_Single() �������Ĵ�����ֵ�洢���洢��Ԫ�У���ʱ��ֵ��������һ���ϵ�ʱ�Զ�����
		��Ҫ�ֶ�����ǰ���мĴ�����ֵ�ָ�Ϊ�洢��Ԫ�е�ֵ������Ե��� MA600_Restore_All() ����ʵ��
		
		
		
Others:      
Function List:
History:
<author>    <time>          <version>       <desc>
Li_Jiang    2023_12_23      v1.0.0          ����
Li_Jiang    2024_6_10       v1.0.2          ��λ��װ����֤BCT����
Li_Jiang    2025_1_8        v1.0.3          �����ת������λ�����á���ȡ
**************************************************/
#include "MA600_base.h"

/* ============ ȫ�ֱ������� ============= */
hMA600_TypeDef TestMA600 = {0};

/* ============ �ֲ��������� ============= */

/* ============ �ڲ��������� ============= */
// ˵��������ʹ�ö�ʱ��5��ʵ����ʱ���ܣ���ֲʱ����ʵ�����ʵ��us��ms����ʱ
// ��Ƶ72MHz PSC��73 ARR��0xFFFF ��������Up
// �൱��ÿ1us CNTֵ��һ

static void MA600_Delay_ns(uint32_t ns);
static void MA600_Delay_us(uint16_t us);
static void MA600_Delay_ms(uint16_t ms);

/* ============ �������� ================= */

/**
  * @name   MA600_Delay_ns
  * @brief  MA600ʵ��������ʱ
  * @call   Internal
  * @param  ns  Ҫ�ӳٵ�ʱ����ns��
  * @RetVal NULL
  */
static void MA600_Delay_ns(uint32_t ns)
{
    ns *= 100;
    for( ; ns > 0; ns--);
}/* MA600_Delay_ns() */

/**
  * @name   MA600_Delay_us
  * @brief  MA600ʵ��΢����ʱ
  * @call   Internal
  * @param  us  Ҫ�ӳٵ�ʱ����us��
  * @RetVal NULL
  */
static void MA600_Delay_us(uint16_t us)
{
//    UNUSED(us);
    
    uint16_t start = TIM6->CNT;
    uint16_t end;
    
    if(0xFFFF - start < us)
    {
        end = us - (0xFFFF - start);
        
        while(TIM6->CNT < end || TIM6->CNT < start);
    }
    else
    {
        end = start + us;
        
        while(TIM6->CNT < end);
    }
}/* MA600_Delay_us() */

/**
  * @name   MA600_Delay_ms
  * @brief  MA600ʵ�ֺ�����ʱ
  * @call   Internal
  * @param  ms  Ҫ�ӳٵ�ʱ����ms��
  * @RetVal NULL
  */
static void MA600_Delay_ms(uint16_t ms)
{
    for( ; ms > 0; ms--)
    {
        MA600_Delay_us(1000);
    }
}/* MA600_Delay_ms() */

/**
  * @name   MA600_HandleInit()
  * @brief  MA600 �����ʼ��
  * @call   External
  * @param  hMA600          Ҫ��ʼ���ľ����ָ��
  * @param  hspi            Ҫ��ʼ���Ĵű����ڵ�SPI���ߵľ��ָ��
  * @param  spi_CS_GPIOx    �ű��Ӧ��Ƭѡ���ŵĶ˿�
  * @param  spi_CS_Pin      �ű��Ӧ��Ƭѡ���ŵ�����
  * @RetVal Return value
  */
void MA600_HandleInit(hMA600_TypeDef *hMA600, SPI_HandleTypeDef *hspi, GPIO_TypeDef *spi_CS_GPIOx, uint32_t spi_CS_Pin)
{
    hMA600->hspi            = hspi;
    hMA600->spi_CS_GPIOx    = spi_CS_GPIOx;
    hMA600->spi_CS_Pin      = spi_CS_Pin;
    
    hMA600->Angle           = 0;
	  hMA600->Circle          = 0;
}/* MA600_HandleInit() */

/**
  * @name   MA600_Get_Angle()
  * @brief  ��ȡ��ǰ�Ƕ�
  * @call   Internal or External
  * @param  hMA600          MA600���ָ��
  * @RetVal Angle           ��ǰ�Ƕ�
  */
uint16_t MA600_Get_Angle(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    
    uint16_t TxData = 0, RxData;
    // Ƭѡʹ��
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    // ʱ���߷���16�����壬����16λ�Ƕ�����
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)&TxData, (uint8_t *)&RxData, 1, 0xFF);
    
    hMA600->Angle = RxData;
    // Ƭѡʧ��
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    return hMA600->Angle;
}/* MA600_Get_Angle() */

/**
  * @name   MA600_Read_Reg()
  * @brief  ��ȡָ���Ĵ�����ֵ
  * @call   Internal or External
  * @param  hMA600          MA600���ָ��
  *         RegAddr         Ҫ��ȡ�ļĴ�����ַ
  * @RetVal RegData         �Ĵ�����ֵ
  */
uint8_t MA600_Read_Reg(hMA600_TypeDef *hMA600, uint8_t RegAddr)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    
    // Ƭѡʹ��
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    uint16_t TxData = 0xD200, RxData = 0;
    int8_t   RegData = 0;
    
    TxData |= (RegAddr & 0xFF);
    
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)&TxData, (uint8_t *)&RxData, 1, 0xFF);
    
    // Ƭѡʧ��
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    hMA600->Angle = RxData;
    TxData = 0;
    
    // ���δ���֮����Ҫ���t_IdleCommand
    MA600_Delay_ns(MA600_t_IdleCommand);
    
    // Ƭѡʹ��
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)&TxData, (uint8_t *)&RxData, 1, 0xFF);
    
    RegData = RxData & 0xFF;
    
    // Ƭѡʧ��
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    return RegData;
}/* MA600_Read_Reg() */

/**
  * @name   MA600_Write_Reg
  * @brief  ��ָ���Ĵ���д������
  * @call   Internal or External
  * @param  hMA600          MA600���ָ��
  *         RegAddr         Ҫд��ļĴ�����ַ
  *         Value           Ҫд���ֵ
  * @RetVal RegData         д���Ĵ����ڵ�����
  */
uint8_t MA600_Write_Reg(hMA600_TypeDef *hMA600, uint8_t RegAddr, uint8_t Value)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    
    // Ƭѡʹ��
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    uint16_t TxData = 0xEA54, RxData = 0;
    int8_t   RegData = 0;
    
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)&TxData, (uint8_t *)&RxData, 1, 0xFF);
    hMA600->Angle = RxData;
    
    // Ƭѡʧ��
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    // ���δ���֮����Ҫ���t_IdleCommand
    MA600_Delay_ns(MA600_t_IdleCommand);
    
    // Ƭѡʹ��
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    TxData = (RegAddr << 8) | (Value & 0xFF);
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)&TxData, (uint8_t *)&RxData, 1, 0xFF);
    
    // Ƭѡʧ��
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    // ���δ���֮����Ҫ���t_IdleCommand
    MA600_Delay_ns(MA600_t_IdleCommand);
    
    // Ƭѡʹ��
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    HAL_SPI_Receive(hMA600->hspi, (uint8_t *)&RxData, 1, 0xFF);
    
    RegData = RxData & 0xFF;
    
    // Ƭѡʧ��
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    return RegData;
}/* MA600_Write_Reg() */

/**
  * @name   MA600_Store_Single()
  * @brief  �洢�����Ĵ�����ֵ��NVM��
  * @call   Internal or External
  * @param  hMA600          MA600���ָ��
  *         BlockIndex      Ҫ�洢�ļĴ�����ı��
  * @RetVal Angle           ��ǰ�Ƕ�
  */
uint16_t MA600_Store_Single(hMA600_TypeDef *hMA600, uint8_t BlockIndex)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    
    if(0 != BlockIndex && 1 != BlockIndex)
    {
        return 1;
    }
    
    uint16_t TxData = 0xEA55, RxData = 0;
    
    // Ƭѡʹ��
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)&TxData, (uint8_t *)&RxData, 1, 0xFF);
    hMA600->Angle = RxData;
    
    // Ƭѡʧ��
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    // ���δ���֮����Ҫ���t_IdleCommand
    MA600_Delay_ns(MA600_t_IdleCommand);
    
    // Ƭѡʹ��
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    TxData = 0xEA00 | (0 == BlockIndex ? 0x0000: 0x0001);
    HAL_SPI_Transmit(hMA600->hspi, (uint8_t *)&TxData, 1, 0xFF);
    
     // Ƭѡʧ��
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    // ���δ���֮����Ҫ���t_StoreRegBlock
    MA600_Delay_ms(MA600_t_StoreRegBlock);
    
    // Ƭѡʹ��
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    TxData = 0x0000;
    HAL_SPI_Transmit(hMA600->hspi, (uint8_t *)&TxData, 1, 0xFF);
    
    // Ƭѡʧ��
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    return hMA600->Angle;
}/* MA600_Store_Single() */

/**
  * @name   MA600_Restore_All()
  * @brief  ��NVM�е�����ֵ�ָ����Ĵ�����
  * @call   Internal or External
  * @param  hMA600          MA600���ָ��
  * @RetVal Angle           ��ǰ�Ƕ�
  */
uint16_t MA600_Restore_All(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    
    uint16_t TxData = 0, RxData = 0;
    
    // Ƭѡʹ��
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    TxData = 0xEA56;
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)&TxData, (uint8_t *)&RxData, 1, 0xFF);
    hMA600->Angle = RxData;
    
    // Ƭѡʧ��
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    // ���δ���֮����Ҫ���t_RestoreAllRegBlocks
    MA600_Delay_us(MA600_t_RestoreAllRegBlocks);
    
    // Ƭѡʹ��
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    TxData = 0x0000;
    HAL_SPI_Transmit(hMA600->hspi, (uint8_t *)&TxData, 1, 0xFF);
    
    // Ƭѡʧ��
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    return hMA600->Angle;
}/* MA600_Restore_All() */

/**
  * @name   MA600_Clear_ErrFlags()
  * @brief  ������д����־
  * @call   Internal or External
  * @param  hMA600          MA600���ָ��
  * @RetVal Angle           ��ǰ�Ƕ�
  */
uint16_t MA600_Clear_ErrFlags(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    
    uint16_t TxData = 0, RxData = 0;
    
    // Ƭѡʹ��
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    TxData = 0xD700;
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)&TxData, (uint8_t *)&RxData, 1, 0xFF);
    hMA600->Angle = RxData;
    
    // Ƭѡʧ��
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    // ���δ���֮����Ҫ���t_IdleCommand
    MA600_Delay_ns(MA600_t_IdleCommand);
    
    // Ƭѡʹ��
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    TxData = 0x0000;
    HAL_SPI_Transmit(hMA600->hspi, (uint8_t *)&TxData, 1, 0xFF);
    
    // Ƭѡʧ��
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    return hMA600->Angle;
}/* MA600_Clear_ErrFlags() */


/**
  * @name   MA600_SetMask()
  * @brief  ���ö�ȡ�Ƕ�ʱʹ�õ�����
  * @call   External
  * @param  hMA600          MA600���ָ��
  * @param  Mask            ����ֵ
  * @RetVal hMA600->Mask    ��ǰ��Ч��mask
  */
uint16_t MA600_SetMask(hMA600_TypeDef *hMA600, uint16_t Mask)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    
    hMA600->Mask = Mask;
    
    return hMA600->Mask;

}/* MA600_SetMask() */


/**
  * @name   MA600_SetValueBits()
  * @brief  ���ö�ȡ�Ƕ�ʱʹ�õ�����(ͨ�����ȣ�
  * @call   External
  * @param  hMA600          MA600���ָ��
  * @param  bits            ��Чλ����
  * @RetVal hMA600->Mask    ��ǰ��Ч��mask
  */
uint16_t MA600_SetValueBits(hMA600_TypeDef *hMA600, uint16_t bits)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    
    hMA600->Mask = 0xFFFF << (16 - bits);
    
    return hMA600->Mask;
}/* MA600_SetValueBits() */


// ���еļĴ������ö�Ҫ��ѭ    ����->�޸�->д��->��� �Ĳ���


/**
  * @name   MA600_Read_PPT()
  * @brief  ��ȡָ���ű��ÿ�����������������������
  * @call   External
  * @param  hMA600      Ҫ�����Ĵű�ľ��ָ��
  * @RetVal ָ���ű��ÿ����������
  */
uint16_t MA600_Read_PPT(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    uint8_t RegValue[2] = {0};
    uint16_t PPTValue = 0;
    
    RegValue[0] = MA600_Read_Reg(hMA600, MA600_Reg_PPT_ILIP);
    RegValue[1] = MA600_Read_Reg(hMA600, MA600_Reg_PPT);
    
    PPTValue = (RegValue[1] << 3) | ((RegValue[0] >> 5) & 0x07) | ((RegValue[0] << 11) & 0x0800);
    PPTValue = PPTValue + 1;
    
    return PPTValue;
}/* MA600_Read_PPT() */

/**
  * @name   MA600_Set_PPT()
  * @brief  ����ָ���ű��ÿ�����������������������
  * @call   External
  * @param  hMA600      Ҫ�����Ĵű�ľ��ָ��
  * @param  NewPPTValue Ҫ���õ�ÿ����������
  * @RetVal ָ���ű��ÿ����������
  */
uint16_t MA600_Set_PPT(hMA600_TypeDef *hMA600, uint16_t NewPPTValue)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    uint8_t RegValue[2] = {0};
    uint8_t TxData[2];
    
    
    // ������ǰ�Ĵ����е�ֵ
    RegValue[0] = MA600_Read_Reg(hMA600, MA600_Reg_PPT_ILIP);
    RegValue[1] = MA600_Read_Reg(hMA600, MA600_Reg_PPT);
    
    // �����µ�ֵ
    NewPPTValue = NewPPTValue - 1;
    TxData[0] = RegValue[0] & MA600_Msk_ILIP;
    TxData[0] |= ((NewPPTValue >> 11U) & 0x01);
    TxData[0] |= ((NewPPTValue << 5U)  & 0xE0);
    TxData[1] = ((NewPPTValue >> 3U) & 0xFF);
    
    // ����ֵд�ص��Ĵ�����
    RegValue[0] = MA600_Write_Reg(hMA600, MA600_Reg_PPT_ILIP, TxData[0]);
    RegValue[1] = MA600_Write_Reg(hMA600, MA600_Reg_PPT, TxData[1]);
    
    // �ж�д���Ƿ�ɹ�
    if(RegValue[0] == TxData[0] && RegValue[1] == TxData[1])
    {
        // �ɹ�
        return NewPPTValue + 1;
    }
    else
    {
        return 1;
    }
}/* MA600_Set_PPT() */

/**
  * @name   MA600_Read_IOMatrix()
  * @brief  ��ȡָ���ű��IO����ѡ��
  * @call   External
  * @param  hMA600      Ҫ�����Ĵű�ľ��ָ��
  * @RetVal ָ���ű��IO����ѡ��     @ IO Matrix Type
  */
uint8_t MA600_Read_IOMatrix(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    uint8_t RegValue = 0;
    uint8_t INFT_SEL_Value = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_INTF_SEL_DAZ_CK100);
    
    INFT_SEL_Value = (RegValue & MA600_Msk_INTF_SEL) >> 5U;
    
    return INFT_SEL_Value;
}

/**
  * @name   MA600_Set_IOMatrix()
  * @brief  ����ָ���ű��IO����ѡ��
  * @call   External
  * @param  hMA600      Ҫ�����Ĵű�ľ��ָ��
  * @param  Type        IO����ѡ��  @ IO Matrix Type
  * @RetVal ָ���ű��IO����ѡ��
  */
uint8_t MA600_Set_IOMatrix(hMA600_TypeDef *hMA600, uint8_t Type)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    uint8_t RegValue = 0;
    uint8_t TxData = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_INTF_SEL_DAZ_CK100);
    
    TxData = RegValue & ~MA600_Msk_INTF_SEL;
    
    TxData |= ((Type << 5U) & MA600_Msk_INTF_SEL);
    
    RegValue = MA600_Write_Reg(hMA600, MA600_Reg_INTF_SEL_DAZ_CK100, TxData);
    
    // ���ص�ǰ�Ĵ����ڵ��������
    return (RegValue & MA600_Msk_INTF_SEL) >> 5U;
}/* MA600_Set_IOMatrix() */

/**
  * @name   MA600_Set_MTSP()
  * @brief  ����ָ���ű�ķ���ֵ���ͣ���ȦȦ������ת�ٶȣ�
  * @call   External
  * @param  hMA600      Ҫ�����Ĵű�ľ��ָ��
  * @param  Type        ����ֵ���ͣ���ȦȦ������ת�ٶȣ�  @ MTSP Type
  * @RetVal ָ���ű�ķ���ֵ����      @ MTSP Type
  */
uint8_t MA600_Set_MTSP(hMA600_TypeDef *hMA600, uint8_t Type)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    uint8_t RegValue = 0;
    uint8_t TxData = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_MTSP_PRT_PRTS_APRT_FTA_FTM);
    
    TxData = RegValue & ~MA600_Msk_MTSP;
    Type &= 0x01U;
    TxData = TxData | (Type << 7U);
    
    RegValue = MA600_Write_Reg(hMA600, MA600_Reg_MTSP_PRT_PRTS_APRT_FTA_FTM, TxData);
    
    Type = (RegValue & MA600_Msk_MTSP) >> 7U;
    
    // ���ص�ǰ�Ĵ����ڵ��������
    return Type;
}/* MA600_Set_MTSP() */

/**
  * @name   MA600_Get_Speed_rpm()
  * @brief  ��ȡָ���ű������ת�٣�rpm��
  * @call   External
  * @param  hMA600      Ҫ�����Ĵű�ľ��ָ��
  * @RetVal Omega_rpm   ָ���ű������ת�٣�rpm��
  */
float MA600_Get_Speed_rpm(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    
    uint16_t TxData[2] = {0};
    uint16_t RxData[2] = {0};
    float    Omega_rpm = 0.0f;
    
    // Ƭѡʹ��
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)TxData, (uint8_t *)RxData, 2, 0xFF);
    
    // Ƭѡʧ��
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    hMA600->Angle = RxData[0];
    Omega_rpm     = (int16_t)RxData[1] * 5.722;
    hMA600->Speed = Omega_rpm;
    
    // ��������˲�������õ����ٶ���Ϣ������ʱ����Ư��ֵ��ԭ������ǰ�װ��಻���ʣ��ӽǶ�ֵ�Ͽ�������2��3������
    // ���������û�в��ԣ�Ŀǰ���Լ��ϵ�3508ƨ��û�е����ת������
    
    return Omega_rpm;
}/* MA600_Get_Speed_rpm() */


/**
  * @name   MA600_Get_MultiTurn()
  * @brief  ��ȡָ���ű�����Ķ�ȦȦ��
  * @call   External
  * @param  hMA600      Ҫ�����Ĵű�ľ��ָ��
  * @RetVal MultiTurn   ָ���ű�����Ķ�ȦȦ��
  */
int16_t MA600_Get_MultiTurn(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return 0;
    }
    
    uint16_t TxData[2] = {0};
    uint16_t RxData[2] = {0};
    int16_t  MultiTurn = 0.0f;
    
    // Ƭѡʹ��
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)TxData, (uint8_t *)RxData, 2, 0xFF);
    
    // Ƭѡʧ��
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    hMA600->Angle       = RxData[0];
    MultiTurn           = (int16_t)RxData[1];
    hMA600->MultiTurn   = MultiTurn;
    
    return MultiTurn;
}/* MA600_Get_MultiTurn() */

/**
  * @name   MA600_Read_BCT()
  * @brief  ��ȡָ���ű��BCT����ֵ
  * @call   External
  * @param  hMA600      Ҫ�����Ĵű�ľ��ָ��
  * @RetVal Omega_rpm   ָ���ű��BCT����ֵ
  */
uint8_t MA600_Read_BCT(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    uint8_t RegValue = 0;
    uint8_t BCT_Value = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_BCT);
    
    BCT_Value = RegValue;
    
    return BCT_Value;
}/* MA600_Read_BCT() */

/**
  * @name   MA600_Set_BCT()
  * @brief  ����ָ���ű��BCT����ֵ
  * @call   External
  * @param  hMA600      Ҫ�����Ĵű�ľ��ָ��
  * @param  BCTValue    Ҫ���õ�BCTֵ
  * @RetVal Omega_rpm   ָ���ű��BCT����ֵ
  */
uint8_t MA600_Set_BCT(hMA600_TypeDef *hMA600, uint8_t BCTValue)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    uint8_t RegValue = 0;
    uint8_t TxData = 0;
    
//    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_BCT);
    
    TxData = BCTValue;
    
    RegValue = MA600_Write_Reg(hMA600, MA600_Reg_BCT, TxData);
    
    // ���ص�ǰ�Ĵ����ڵ��������
    return RegValue;
}/* MA600_Set_BCT() */

/**
  * @name   MA600_Read_ETYETX()
  * @brief  ��ȡָ���ű��ͨ��˥��ʹ�����
  * @call   External
  * @param  hMA600  Ҫ�����Ĵű�ľ��ָ��
  * @RetVal ָ���ű��ͨ��˥��ʹ�����
  */
uint8_t MA600_Read_ETYETX(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    uint8_t RegValue = 0;
    uint8_t ETYETX_Value = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_ETY_ETX);
    
    ETYETX_Value = RegValue & (MA600_Msk_ETY | MA600_Msk_ETX);
    
    return ETYETX_Value;
}/* MA600_Read_ETYETX() */

/**
  * @name   MA600_Set_ETYETX()
  * @brief  ����ָ���ű��ͨ��˥��ʹ�����
  * @call   External
  * @param  hMA600  Ҫ�����Ĵű�ľ��ָ��
  * @param  ETY     Y�ᴫ������˥��ʹ��
  * @param  ETX     X�ᴫ������˥��ʹ��
  * @RetVal ָ���ű��ͨ��˥��ʹ�����
  */
uint8_t MA600_Set_ETYETX(hMA600_TypeDef *hMA600, uint8_t ETY, uint8_t ETX)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    uint8_t RegValue = 0;
    uint8_t TxData = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_ETY_ETX);
    
    TxData = RegValue & ~(MA600_Msk_ETY | MA600_Msk_ETX);
    TxData |= (0 == ETY) ? 0: MA600_Msk_ETY;
    TxData |= (0 == ETX) ? 0: MA600_Msk_ETX;
    
    RegValue = MA600_Write_Reg(hMA600, MA600_Reg_ETY_ETX, TxData);
    
    // ���ص�ǰ�Ĵ����ڵ��������
    return RegValue;
}/* MA600_Set_ETYETX() */

/**
  * @name   MA600_Read_FW()
  * @brief  ��ȡָ���ű���˲�����ֵ
  * @call   External
  * @param  hMA600  Ҫ�����Ĵű�ľ��ָ��
  * @RetVal ָ���ű���˲�����ֵ
  */
uint8_t MA600_Read_FW(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    uint8_t RegValue = 0;
    uint8_t FW_Value = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_FW);
    
    FW_Value = RegValue & MA600_Msk_FW;
    
    return FW_Value;
}/* MA600_Read_FW() */

/**
  * @name   MA600_Set_FW()
  * @brief  ����ָ���ű���˲�����ֵ
  * @call   External
  * @param  hMA600  Ҫ�����Ĵű�ľ��ָ��
  * @param  FWValue Ҫ���õ��˲�����ֵ
  * @RetVal ָ���ű���˲�����ֵ
  */
uint8_t MA600_Set_FW(hMA600_TypeDef *hMA600, uint8_t FWValue)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    uint8_t RegValue = 0;
    uint8_t TxData = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_FW);
    
    TxData = RegValue & ~MA600_Msk_FW;
    TxData |= FWValue;
    
    RegValue = MA600_Write_Reg(hMA600, MA600_Reg_FW, TxData);
    
    // ���ص�ǰ�Ĵ����ڵ��������
    return RegValue;
}/* MA600_Set_FW() */


/**
  * @name   MA600_Read_RotationDirection()
  * @brief  ��ȡָ���ű����ת������(��ֵ��������)
  * @call   External
  * @param  hMA600  Ҫ�����Ĵű�ľ��ָ��
  * @RetVal RD_Value    ��ת����(0 - CW | 1 - CCW)
  */
uint8_t MA600_Read_RotationDirection(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    uint8_t RegValue = 0;
    uint8_t RD_Value = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_RD);
    
    RD_Value = (RegValue & MA600_Msk_RD) >> 7U;
    
    return RD_Value;
}/* MA600_Read_RotationDirection() */

/**
  * @name   MA600_Set_RotationDirection()
  * @brief  ����ָ���ű����ת������(��ֵ��������)
  * @call   External
  * @param  hMA600  Ҫ�����Ĵű�ľ��ָ��
  * @param  RD      Ҫ���õ���ת������[MA600_RD_CW || MA600_RD_CW]
  * @RetVal ָ���ű����ת������  (0 - CW | 1 - CCW)
  */
uint8_t MA600_Set_RotationDirection(hMA600_TypeDef *hMA600, uint8_t RD)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    uint8_t RegValue = 0;
    uint8_t TxData = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_RD);
    
    TxData = RegValue & ~MA600_Msk_RD;
    TxData |= (RD << 7U);
    
    RegValue = MA600_Write_Reg(hMA600, MA600_Reg_RD, TxData);
    
    RegValue = (RegValue & MA600_Msk_RD) >> 7U;
    
    // ���ص�ǰ�Ĵ����ڵ��������
    return RegValue;
}/* MA600_Set_RotationDirection() */

/**
  * @name   MA600_Read_Zero()
  * @brief  ��ȡָ���ű�����λ��
  * @call   External
  * @param  hMA600  Ҫ�����Ĵű�ľ��ָ��
  * @RetVal ZeroValue   
  */
uint16_t MA600_Read_Zero(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    uint8_t  RegValue = 0;
    uint16_t ZeroValue = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_Z_H);
    ZeroValue = RegValue;
    ZeroValue = ZeroValue << 8;
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_Z_L);
    ZeroValue |= RegValue;
    
    return ZeroValue;
}/* MA600_Read_Zero() */

/**
  * @name   MA600_Set_Zero()
  * @brief  ����ָ���ű�����λ��
  * @call   External
  * @param  hMA600  Ҫ�����Ĵű�ľ��ָ��
  * @param  Zero    Ҫ���õ����λ��
  * @RetVal ָ���ű�����λ��
  */
uint16_t MA600_Set_Zero(hMA600_TypeDef *hMA600, uint16_t Zero)
{
    if(NULL == hMA600)
    {
        return 1;
    }
    uint8_t RegValue = 0;
    uint8_t TxData = 0;
    uint16_t ZeroValue = 0;
    
    // ��8λ
    TxData = (Zero >> 8) & 0xFFU;
    RegValue = MA600_Write_Reg(hMA600, MA600_Reg_Z_H, TxData);
    ZeroValue = RegValue;
    ZeroValue = ZeroValue << 8;
    
    // ��8λ
    TxData = Zero & 0xFFU;
    RegValue = MA600_Write_Reg(hMA600, MA600_Reg_Z_L, TxData);
    ZeroValue |= RegValue;
    
    // ���ص�ǰ�Ĵ����ڵ��������
    return ZeroValue;
}/* MA600_Set_RotationDirection() */


