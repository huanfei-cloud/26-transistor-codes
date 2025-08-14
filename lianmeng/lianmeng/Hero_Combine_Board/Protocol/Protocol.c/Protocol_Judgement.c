/**
 * @file Protocol_Judgement.c
 * @author Why (1091537104@qq)
 * @brief ����ϵͳ�Ľ��㺯��
 * @version 0.1
 * @date 2023-08-16
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "BSP_BoardCommunication.h"
#include "Protocol_Judgement.h"
#include "main.h"
#include "CRC.h"

void JudgeSystem_USART_Receive_DMA(UART_HandleTypeDef *huartx);
void JudgeSystem_Handler(UART_HandleTypeDef *huart);
void Check_Judge(void);

JudgeSystem_FUN_t JudgeSystem_FUN = JudgeSystem_FUNGroundInit;
#undef JudgeSystem_FUNGroundInit
RM_Judge_t RM_Judge;

/****************************************�Զ���DMA-���ڽ���****************************************/
/**
 * @brief USART_DMA���տ������ض���
 * 
 * @param huart 
 * @param pData 
 * @param Size 
 * @return  
 */
static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
{

    /*��⵱ǰhuart״̬*/
    if (huart->RxState == HAL_UART_STATE_READY)
    {
        /*����ĵ�ַ��������������Ļ�*/
        if ((pData == NULL) || (Size == 0))
        {
            return HAL_ERROR;
        }

        /*huart�����Ӧ��Rx�����ض���*/
        huart->pRxBuffPtr = pData;
        huart->RxXferSize = Size;
        huart->ErrorCode = HAL_UART_ERROR_NONE;

        /*����huart1�ϵ�RX_DMA*/
        HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->RDR, (uint32_t)pData, Size);

        /*ֻ������ӦDMA�����Rx���ܣ�����ǿ���Tx�Ļ�����USART_CR3_DMAT��*/
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    }
    else
    {
        return HAL_BUSY;
    }

    return HAL_OK;
}

/**
  * @Data    2021/3/26
  * @brief   ����ϵͳ������ 
  * @param   UART_HandleTypeDef *huart
  * @retval  void
  */
void JudgeSystem_USART_Receive_DMA(UART_HandleTypeDef *huartx)
{
    __HAL_UART_CLEAR_IDLEFLAG(huartx);
    __HAL_UART_ENABLE(huartx);
    __HAL_UART_ENABLE_IT(huartx, UART_IT_IDLE);
    // assert(JUDGESYSTEM_PACKSIZE == 341u);
    USART_Receive_DMA_NO_IT(huartx, JudgeSystem_rxBuff, JUDGESYSTEM_PACKSIZE);
}

uint16_t DMA_Counter;
void JudgeSystem_Handler(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_DMA_DISABLE(huart->hdmarx);

    DMA_Counter = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    Judge_GetMessage(JUDGESYSTEM_PACKSIZE - DMA_Counter);

    __HAL_DMA_SET_COUNTER(huart->hdmarx, JUDGESYSTEM_PACKSIZE);
    __HAL_DMA_ENABLE(huart->hdmarx);

    RM_Judge.InfoUpdateFrame++;
}

void Check_Judge(void)
{
    if (RM_Judge.InfoUpdateFrame < 1)
    {
        RM_Judge.OffLineFlag = 1;
    }
    else
    {
        RM_Judge.OffLineFlag = 0;
    }
    RM_Judge.InfoUpdateFrame = 0;
}

/*************************************2023��������ϵͳ*********************************************/
RM_Judge_t JudgeSystem;

ext_game_status_t 				 ext_game_status;
ext_game_result_t 				 ext_game_result;
ext_game_robot_HP_t 			 ext_game_robot_HP;
ext_event_data_t 				 ext_even_data;
ext_supply_projectile_action_t 	 ext_supply_projectile_action;
ext_referee_warning_t 			 ext_referee_warning;
ext_dart_remaining_time_t 		 ext_dart_remaining_time;
ext_game_robot_status_t 		 ext_game_robot_state;
ext_power_heat_data_t 			 ext_power_heat_data;
ext_game_robot_pos_t 			 ext_game_robot_pos;
ext_buff_t 						 Buff;
aerial_robot_energy_t 			 aerial_robot_energy;
ext_robot_hurt_t 				 ext_robot_hurt;
ext_shoot_data_t 				 ext_shoot_data;
ext_bullet_remaining_t 			 ext_bullet_remaining;
ext_rfid_status_t 			 	 ext_rfid_status;
ext_dart_client_cmd_t 			 ext_dart_client_cmd;
ext_ground_robot_position_t 	 ext_ground_robot_position;
ext_radar_mark_data_t 			 ext_radar_mark_data;
custom_robot_data_t 			 custom_robot_data;
ext_robot_command_t 			 ext_robot_command;
ext_robot_keycommand_t 			 ext_robot_keycommand;
ext_client_map_command_t 		 ext_client_map_command;
custom_client_data_t 			 custom_client_data;
ext_map_sentry_data_t            ext_map_sentry_data;
ext_user_data_t                  ext_user_data;
ext_custom_info_t                ext_custom_info;
ext_sentry_info_t                ext_sentry_info;
ext_radar_info_t                 ext_radar_info;

uint8_t JudgeSystem_rxBuff[JUDGESYSTEM_PACKSIZE]; //����buff
uint8_t Robot_Commute[128];

/*****************************************************
 ** @brief  ��ȡ����ϵͳ��Ϣ                        **
 ** @param  Data_Length 	��ȡ�������ݳ���          **
 ** **************************************************
 */
void Judge_GetMessage(uint16_t Data_Length)
{
    for (int n = 0; n < Data_Length;)
    {
        if (JudgeSystem_rxBuff[n] == JUDGE_FRAME_HEADER)
        {
            switch (JudgeSystem_rxBuff[n + 5] | JudgeSystem_rxBuff[n + 6] << 8)
            {
            case Judge_Game_StatusData: //����״̬����
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Game_StatusData))
                {
                    memcpy(ext_game_status.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Game_StatusData-9]));
                    n += JudgeLength_Game_StatusData;
                    ext_game_status.infoUpdateFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Game_ResultData: //�������
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Game_ResultData))
                {
                    memcpy(ext_game_result.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Game_ResultData-9]));
                    n += JudgeLength_Game_ResultData;
                    ext_game_result.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Robot_HP: //������Ѫ������
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_HP))
                {
                    memcpy(&ext_game_robot_HP.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Robot_HP-9]));
                    n += JudgeLength_Robot_HP;
                    ext_game_robot_HP.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Event_Data: //�����¼�����
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Event_Data))
                {
                    memcpy(&ext_even_data.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Event_Data-9]));
                    n += JudgeLength_Event_Data;
                    ext_even_data.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Supply_Station: //����վ������ʶ
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Supply_Station))
                {
                    memcpy(&ext_supply_projectile_action.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Supply_Station-9]));
                    n += JudgeLength_Supply_Station;
                    ext_supply_projectile_action.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Referee_Warning: //����ϵͳ������Ϣ
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Referee_Warning))
                {
                    memcpy(&ext_referee_warning.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Referee_Warning-9]));
                    n += JudgeLength_Referee_Warning;
                    ext_referee_warning.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Dart_Countdown: //���ڷ���ڵ���ʱ
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Dart_Countdown))
                {
                    memcpy(&ext_dart_remaining_time.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Dart_Countdown-9]));
                    n += JudgeLength_Dart_Countdown;
                    ext_dart_remaining_time.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Robot_State: //����������״̬
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_State))
                {
                    memcpy(&ext_game_robot_state.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Robot_State-9]));
					ControlMes.shooter_42mm_heat_limit =ext_game_robot_state.data.shooter_barrel_heat_limit;//ǹ����������
					ControlMes.chassis_power_limit = ext_game_robot_state.data.chassis_power_limit;         //���̹�������
					if (ext_game_robot_state.data.robot_id < 100)
						ControlMes.tnndcolor = 1;															//����ID
					else if (ext_game_robot_state.data.robot_id > 100)
						ControlMes.tnndcolor = 2;	
//					ControlMes.HeatLimit_2 = ext_game_robot_state.data.shooter_id2_17mm_barrel_heat_limit;
                    n += JudgeLength_Robot_State;
                    ext_game_robot_state.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Power_Heat: //ʵʱ��������
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Power_Heat))
                {
                    memcpy(&ext_power_heat_data.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Power_Heat-9]));
					ControlMes.shooter_42mm_heat_now = ext_power_heat_data.data.shooter_id1_42mm_barrel_heat;//ǹ��ʵʱ����
					ControlMes.chassis_power_buffer = ext_power_heat_data.data.chassis_power_buffer;//���̹��ʻ���
					ControlMes.chassis_power = ext_power_heat_data.data.chassis_power;//����ʵʱ����
//					ControlMes.Heat_2 = ext_power_heat_data.data.shooter_id2_17mm_barrel_heat;
                    n += JudgeLength_Power_Heat;
                    ext_power_heat_data.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Robot_Position: //������λ��
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_Position))
                {
                    memcpy(&ext_game_robot_pos.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Robot_Position-9]));
                    n += JudgeLength_Robot_Position;
                    ext_game_robot_pos.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Robot_Buff: //����������
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_Buff))
                {
                    memcpy(&Buff.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Robot_Buff-9]));
                    n += JudgeLength_Robot_Buff;
                    Buff.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Aerial_Time: //���л���������״̬
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Aerial_Time))
                {
                    memcpy(&aerial_robot_energy.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Aerial_Time-9]));
                    n += JudgeLength_Aerial_Time;
                    aerial_robot_energy.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Injury_State: //�˺�״̬
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Injury_State))
                {
                    memcpy(&ext_robot_hurt.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Injury_State-9]));
                    n += JudgeLength_Injury_State;
                    ext_robot_hurt.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_RealTime_Shoot: //ʵʱ�������
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_RealTime_Shoot))
                {
                    memcpy(&ext_shoot_data.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_RealTime_Shoot-9]));
	//				if (ext_shoot_data.data.shooter_id == 1)
	//					ControlMes.BulletSpeed_1 = ext_shoot_data.data.bullet_speed;
//					else if (ext_shoot_data.data.shooter_id == 2)
	//					ControlMes.BulletSpeed_2 = ext_shoot_data.data.bullet_speed;			
                    n += JudgeLength_RealTime_Shoot;
                    ext_shoot_data.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Remaining_Rounds: //�ӵ�ʣ����
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Remaining_Rounds))
                {
                    memcpy(&ext_bullet_remaining.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Remaining_Rounds-9]));
                    n += JudgeLength_Remaining_Rounds;
                    ext_bullet_remaining.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Robot_RFID: //������RFID״̬
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_RFID))
                {
                    memcpy(&ext_rfid_status.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Robot_RFID-9]));
                    n += JudgeLength_Robot_RFID;
                    ext_rfid_status.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Dart_Client: //���ڻ����˿ͻ���ָ������
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Dart_Client))
                {
                    memcpy(&ext_dart_client_cmd.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Dart_Client-9]));
                    n += JudgeLength_Dart_Client;
                    ext_dart_client_cmd.InfoUpdataFlag = 1;
                }
                else
                    n++; //26
                break;
			case Judge_Ground_Position: //���������λ������
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Ground_Position))
                {
                    memcpy(&ext_ground_robot_position.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Ground_Position-9]));
                    n += JudgeLength_Ground_Position;
                    ext_ground_robot_position.InfoUpdataFlag = 1;
                }
                else
                    n++; 
                break;
			case Judge_Radar_Marking: //�״��ǽ�������
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Radar_Marking))
                {
                    memcpy(&ext_radar_mark_data.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Radar_Marking-9]));
                    n += JudgeLength_Radar_Marking;
                    ext_radar_mark_data.InfoUpdataFlag = 1;
                }
                else
                    n++; 
                break;
			case Judge_Sentry_Info: //�ڱ���Ϣ����
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Sentry_Info))
                {
                    memcpy(&ext_sentry_info.sentry_info, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Radar_Marking-9]));
                    n += JudgeLength_Sentry_Info;
                    ext_radar_mark_data.InfoUpdataFlag = 1;
                }
                else
                    n++; 
                break;
			case Judge_Radar_Info: //�״���Ϣ����
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Radar_Info))
                {
                    memcpy(&ext_radar_info.radar_info, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Radar_Marking-9]));
                    n += JudgeLength_Radar_Info;
                    ext_radar_mark_data.InfoUpdataFlag = 1;
                }
                else
                    n++; 
                break;
            case Judge_Robot_Communicate: //��������Ϣ����(����һ��д������ֱ��case����ID ��case������)
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_Communicate))
                {
                    memcpy(&Robot_Commute, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Robot_Communicate-9]));
                    n += JudgeLength_Robot_Communicate;
                }
                else
                    n++;
                break;
			case Judge_User_Defined: //�Զ��������������˽�������
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_User_Defined))
                {
                    memcpy(&custom_robot_data.data, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_User_Defined-9]));
                    n += JudgeLength_User_Defined;
					custom_robot_data.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
			case Judge_Map_Interaction: //�ͻ���С��ͼ��������
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Map_Interaction))
                {
                    memcpy(&ext_robot_command.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Map_Interaction-9]));
                    n += JudgeLength_Map_Interaction;
					ext_robot_command.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
			case Judge_KeyMouse_Message: //�������ͼ������
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_KeyMouse_Message))
                {
                    memcpy(&ext_robot_keycommand.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_KeyMouse_Message-9]));
                    n += JudgeLength_KeyMouse_Message;
					ext_robot_keycommand.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
			case Judge_Client_Map: //�ͻ���С��ͼ�����״���Ϣ
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Client_Map))
                {
                    memcpy(&ext_client_map_command.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Client_Map-9]));
                    n += JudgeLength_Client_Map;
					ext_client_map_command.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
			case Judge_User_Interaction: //�Զ����������ѡ�ֶ˽�������
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_User_Interaction))
                {
                    memcpy(&custom_client_data.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[8]));
                    n += JudgeLength_User_Interaction;
					custom_client_data.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
			case Judge_Sentinal_Map: //ѡ�ֶ�С��ͼ�����ڱ�����
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Sentinal_Map))
                {
                    memcpy(&ext_map_sentry_data.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Sentinal_Map-9]));
                    n += JudgeLength_Sentinal_Map;
					ext_map_sentry_data.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
			case Judge_Custom_Info: //ѡ�ֶ�С��ͼ�����û��Զ�������
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Custom_Info))
                {
                    memcpy(&ext_custom_info.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Custom_Info-9]));
                    n += JudgeLength_Custom_Info;
					ext_map_sentry_data.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            default:
                n++;
                break;
            }
        }
        else
            n++;
    }
    JudgeSystem.InfoUpdateFrame++;
}
