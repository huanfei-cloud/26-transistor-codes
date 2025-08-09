#ifndef PROTOCL_JUDGEMENT_H
#define PROTOCL_JUDGEMENT_H
#include "usart.h"
typedef enum
{
    ID_GAME_STATUS = 0x0001,
    ID_GAME_RESULT = 0x0002,
    ID_GAME_ROBOT_HP = 0x0003,
    ID_EVENT_DATA = 0x0101,
    ID_REFEREE_WARNING = 0x0104,
    ID_DART_INFO = 0x0105,
    ID_ROBOT_STATUS = 0x0201,
    ID_POWER_HEAT_DATA = 0x0202,
    ID_ROBOT_POS = 0x0203,
    ID_BUFF = 0x0204,
    ID_HURT_DATA = 0x0206,
    ID_SHOOT_DATA = 0x0207,
    ID_PROJECTILE_ALLOWANCE = 0x0208,
    ID_RFID_STATUS = 0x0209,
    ID_DART_CLIENT_CMD = 0x020A,
    ID_GROUND_ROBOT_POSITION = 0x020B,
    ID_RADAR_MARK_DATA = 0x020C,
    ID_SNETRY_INFO = 0x020D,
    ID_RADAR_INFO = 0x20E,
    ID_ROBOT_INTERACTION_DATA = 0x301,
    ID_MAP_COMMAND = 0x303,
    ID_MAP_ROBOT_DATA = 0x0305,
    ID_MAP_DATA = 0x0307,
    ID_CUSTOM_INFO = 0x0308,
    ID_CUSTOM_ROBOT_DATA = 0x0302,
    ID_ROBOT_CUSTOM_DATA = 0x0309,
    ID_REMOTE_CONTROL = 0x0304
} CmdID_e;

#pragma pack(1)
typedef struct
{
    uint8_t SOF;
    uint16_t DataLength;
    uint8_t Seq;
    uint8_t CRC8;
} xFrameHeader;

// 常规链路
typedef struct // 0x001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} game_status_t;

typedef struct // 0x002
{
    uint8_t winner;
} game_result_t;

typedef struct // 0x003
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t reserved1;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t reserved2;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} game_robot_HP_t;

typedef struct // 0x101
{
    uint32_t event_data;
} event_data_t;

typedef struct // 0x104
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} referee_warning_t;

typedef struct // 0x105
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} dart_info_t;

typedef struct // 0x201
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} robot_status_t;

typedef struct // 0x202
{
    uint16_t reserved1;
    uint16_t reserved2;
    float reserved3;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

typedef struct // 0x203
{
    float x;
    float y;
    float angle;
} robot_pos_t;

typedef struct // 0x204
{
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
} buff_t;

typedef struct // 0x206
{
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
} hurt_data_t;

typedef struct // 0x207
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
} shoot_data_t;

typedef struct // 0x208
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
} projectile_allowance_t;

typedef struct // 0x209
{
    uint32_t rfid_status;
} rfid_status_t;

typedef struct // 0x20A
{
    uint8_t dart_launch_opening_status;
    uint8_t reserved;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
} dart_client_cmd_t;

typedef struct // 0x20B
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reserved1;
    float reserved2;
} ground_robot_position_t;

typedef struct // 0x20C
{
    uint8_t mark_progress;
} radar_mark_data_t;

typedef struct // 0x20D
{
    uint32_t sentry_info;
    uint16_t sentry_info_2;
} sentry_info_t;

typedef struct // 0x20E
{
    uint8_t radar_info;
} radar_info_t;

typedef struct // 0x301
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[10]; // define the num
} robot_interaction_data_t;

// 子内容ID TODO

typedef struct // 0x303
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint16_t cmd_source;
} map_command_t;

typedef struct // 0x305
{
    uint16_t hero_position_x;
    uint16_t hero_position_y;
    uint16_t engineer_position_x;
    uint16_t engineer_position_y;
    uint16_t infantry_3_position_x;
    uint16_t infantry_3_position_y;
    uint16_t infantry_4_position_x;
    uint16_t infantry_4_position_y;
    uint16_t infantry_5_position_x;
    uint16_t infantry_5_position_y;
    uint16_t sentry_position_x;
    uint16_t sentry_position_y;
} map_robot_data_t;

typedef struct // 0x307
{
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;
} map_data_t;

typedef struct // 0x308
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
} custom_info_t;

// 图传链路
typedef struct // 0x302
{
    uint8_t data[10];
} custom_robot_data_t;

typedef struct // 0x309
{
    uint8_t data[10];
} robot_custom_data_t;

typedef struct // 0x304
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
} remote_control_t;

#pragma pack()


typedef struct
{

    UART_HandleTypeDef *huart_;
    xFrameHeader header_;
    uint16_t cmdID_;

    game_status_t game_status_;
    game_result_t game_result_;
    game_robot_HP_t game_robot_HP_;
    event_data_t event_data_;
    referee_warning_t referee_warning_;
    dart_info_t dart_info_;
    robot_status_t robot_status_;
    power_heat_data_t power_heat_;
    robot_pos_t robot_pos_;
    buff_t buff_;
    hurt_data_t hurt_data_;
    shoot_data_t shoot_data_;
    projectile_allowance_t projectile_allowance_;
    rfid_status_t rfid_status_;
    dart_client_cmd_t dart_client_cmd_;
    ground_robot_position_t ground_robot_position_;
    radar_mark_data_t radar_mark_data_;
    sentry_info_t sentry_info_;
    radar_info_t radar_info_;

    robot_interaction_data_t robot_interaction_data_;
    map_command_t map_command_;
    map_robot_data_t map_robot_data_;
    map_data_t map_data_;
    custom_info_t custom_info_;
    custom_robot_data_t custom_robot_data_;
    robot_custom_data_t robot_custom_data_;

    remote_control_t remote_control_;
} Referee_t;


#define JUDGESYSTEM_PACKSIZE 136
extern uint8_t JudgeSystem_rxBuff[JUDGESYSTEM_PACKSIZE];
extern Referee_t g_referee;

extern void JudgeSystem_USART_Receive_DMA(UART_HandleTypeDef *huartx);
extern void JudgeSystem_Handler(UART_HandleTypeDef *huart);

#endif
