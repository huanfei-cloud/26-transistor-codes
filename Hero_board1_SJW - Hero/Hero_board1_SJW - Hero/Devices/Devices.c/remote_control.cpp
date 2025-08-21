#include "remote_control.h"
#include "usart.h"
#include "string.h"
RemoteCtrl rc(&huart2);
void RemoteCtrl::Init()
{

    HAL_UARTEx_ReceiveToIdle_DMA(this->huart_, this->sbus_buf_, 18);
    __HAL_DMA_DISABLE_IT(this->huart_->hdmarx, DMA_IT_HT);
}

void RemoteCtrl::sbus_to_rc()
{
    this->rc_ctrl_.rc.ch[0] = (this->sbus_buf_[0] | (this->sbus_buf_[1] << 8)) & 0x07ff;        //!< Channel 0
    this->rc_ctrl_.rc.ch[1] = ((this->sbus_buf_[1] >> 3) | (this->sbus_buf_[2] << 5)) & 0x07ff; //!< Channel 1
    this->rc_ctrl_.rc.ch[2] = ((this->sbus_buf_[2] >> 6) | (this->sbus_buf_[3] << 2) |          //!< Channel 2
                               (this->sbus_buf_[4] << 10)) &
                              0x07ff;
    this->rc_ctrl_.rc.ch[3] = ((this->sbus_buf_[4] >> 1) | (this->sbus_buf_[5] << 7)) & 0x07ff; //!< Channel 3
    this->rc_ctrl_.rc.s[0] = ((this->sbus_buf_[5] >> 4) & 0x0003);                              //!< Switch left
    this->rc_ctrl_.rc.s[1] = ((this->sbus_buf_[5] >> 4) & 0x000C) >> 2;                         //!< Switch right
    this->rc_ctrl_.mouse.x = this->sbus_buf_[6] | (this->sbus_buf_[7] << 8);                    //!< Mouse X axis
    this->rc_ctrl_.mouse.y = this->sbus_buf_[8] | (this->sbus_buf_[9] << 8);                    //!< Mouse Y axis
    this->rc_ctrl_.mouse.z = this->sbus_buf_[10] | (this->sbus_buf_[11] << 8);                  //!< Mouse Z axis
    this->rc_ctrl_.mouse.press_l = this->sbus_buf_[12];                                         //!< Mouse Left Is Press ?
    this->rc_ctrl_.mouse.press_r = this->sbus_buf_[13];                                         //!< Mouse Right Is Press ?
    this->rc_ctrl_.key.v = this->sbus_buf_[14] | (this->sbus_buf_[15] << 8);                    //!< KeyBoard value
    this->rc_ctrl_.rc.ch[4] = this->sbus_buf_[16] | (this->sbus_buf_[17] << 8);                 // NULL

    this->rc_ctrl_.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    this->rc_ctrl_.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    this->rc_ctrl_.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    this->rc_ctrl_.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    this->rc_ctrl_.rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

rc_ctrl_t *RemoteCtrl::getRcCtrl()
{
    return &this->rc_ctrl_;
}

uint8_t *RemoteCtrl::getSbusBuf()
{
    return this->sbus_buf_;
}

extern "C"
{
    void Rc_Init()
    {
        rc.Init();
    }
    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
    {

        if (huart == &huart2)
        {
            rc.sbus_to_rc();
            memset(rc.getSbusBuf(), 0, 18);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rc.getSbusBuf(), 18);
            __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
        }
	
    }
	
//	//如果数据来自USART1
//	if(huart->Instance == USART1)
//	{	
//		memcpy(Fire_RxBuffer,Fire_Rx_Data,sizeof(Fire_Rx_Data));
//		Fire_Rx_Finish = 1;//已接收完一包数据
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Fire_Rx_Data,FIRE_MAX_BUF_LEN);
//    }
}

