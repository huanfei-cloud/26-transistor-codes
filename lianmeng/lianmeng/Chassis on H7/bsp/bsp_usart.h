#ifndef BSP_USART_H
#define BSP_USART_H
#include "struct_typedef.h"
#include "stm32h7xx_hal.h"

#define UART_BUFFER_SIZE 256
#define Usart_DMA_Idle_Length 128
extern uint8_t Fd_data[64];
extern uint8_t Fd_rsimu[64];
extern uint8_t Fd_rsahrs[56];
extern int rs_imutype;
extern int rs_ahrstype;
extern uint8_t Count;
extern uint8_t last_rsnum;
extern uint8_t rsimu_flag ;
extern uint8_t rsacc_flag ;

/**
 * @brief UARTͨ�Ž��ջص�������������
 *
 */
typedef void (*UART_Call_Back)(uint8_t Buffer[]);

/**
 * @brief UARTͨ�Ŵ���ṹ��
 */
struct Struct_UART_Manage_Object
{
    UART_HandleTypeDef *UART_Handler;
    uint8_t Tx_Buffer[UART_BUFFER_SIZE];
    uint8_t Rx_Buffer[UART_BUFFER_SIZE];
    uint16_t Rx_Buffer_Length;
    UART_Call_Back Callback_Function;
};


/* Exported variables --------------------------------------------------------*/

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;//��ң����ռ�ã��Ҳ���USART_IRQHandler���������Ǵ��ڽ����жϻص�������DMA�����жϻص�����
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

extern struct Struct_UART_Manage_Object UART1_Manage_Object;
extern struct Struct_UART_Manage_Object UART6_Manage_Object;


/* Exported function declarations --------------------------------------------*/


#endif
