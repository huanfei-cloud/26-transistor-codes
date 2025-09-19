/**
 * @file Protocol_Judgement.h
 * @author Why (1091537104@qq)
 * @brief 裁判系统的详细协议,1.6.1版本
 * @version 0.2
 * @date 2023-02-21
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef _RM_JUDGESYSTEM_H
#define _RM_JUDGESYSTEM_H

#include "main.h"

#pragma anon_unions                         //允许编译匿名结构体 **后边的__packed表示不使用系统对齐**

#define JudgeSystem_FUNGroundInit       \
    {                                   \
        &JudgeSystem_USART_Receive_DMA, \
        &JudgeSystem_Handler,           \
        &Check_Judge,                   \
    }

typedef struct
{
    void (*JudgeSystem_USART_Receive_DMA)(UART_HandleTypeDef *huartx);
    void (*JudgeSystem_Handler)(UART_HandleTypeDef *huart);
    void (*Check_Judge)(void);
} JudgeSystem_FUN_t;

extern JudgeSystem_FUN_t JudgeSystem_FUN;

typedef struct
{
    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} RM_Judge_t;

extern RM_Judge_t RM_Judge;

float GetVision_CP(void);

#define JUDGE_DATA_ERROR 0
#define JUDGE_DATA_CORRECT 1

//2023 裁判系统官方接口协议

//对应通信协议格式   frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16,整包校验)
#define       LEN_HEADER        5/*frame_header*/
#define       LEN_CMDID         2/*cmd_id*/
#define       LEN_TAIL          2/*frame_tail*/

//起始字节，协议固定为0xA5
#define      JUDGE_FRAME_HEADER             0xA5
#define      JUDGESYSTEM_PACKSIZE           389u        //裁判系统包大小(354+35)

//红蓝方
#define BLUE 0
#define RED  1

//信息传输
#define TRUE 1
#define FALSE 0


/***********************************命令码ID*************************************/
/*

ID: 0x0001          Byte: 11      比赛状态数据                   发送频率 1Hz
ID: 0x0002          Byte: 1       比赛结果数据                   比赛结束后发送
ID: 0x0003          Byte: 32      比赛机器人血量数据             发送频率 3Hz
ID: 0x0101          Byte: 4       场地事件数据                   固定1HZ频率发送
ID: 0x0102          Byte: 4       场地补给站动作标识数据         动作改变后发送
ID: 0X0104          Byte: 3       裁判警告数据                   警告发生后发送
ID: 0x0105          Byte: 3       飞镖发射口倒计时               发送频率 1Hz
ID: 0X0201          Byte: 13      机器人状态数据                 发送频率 10Hz
ID: 0X0202          Byte: 16      实时功率热量数据               发送频率 50Hz
ID: 0X0203          Byte: 16      机器人位置数据                 发送频率 1Hz
ID: 0X0204          Byte: 6       机器人增益数据                 固定3HZ发送
ID: 0X0205          Byte: 2       空中机器人支援时间数据         发送频率1Hz，只有空中机器人主控发送
ID: 0X0206          Byte: 1       伤害状态数据                   伤害发生后发送
ID: 0X0207          Byte: 7       实时射击数据                   子弹发射后发送
ID: 0X0208          Byte: 6       子弹允许发送数                 英雄步兵哨兵和空中机器人被发送  发送频率 10Hz
ID: 0X0209          Byte: 4       机器人RFID状态                 发送频率 3Hz
ID: 0x020A          Byte: 6       飞镖机器人客户端指令书         发送频率 3Hz
ID: 0x020B          Byte: 40      地面机器人位置数据             发送频率 1Hz
ID: 0x020C          Byte: 6       雷达标记进度数据               发送频率 1Hz
ID: 0x020D          Byte: 4       哨兵信息数据                   发送频率 1Hz
ID: 0x020E          Byte: 1       雷达信息数据                   发送频率 1Hz
ID: 0X0301          Byte: 128     机器人间交互数据               发送方触发发送    上限频率 10Hz
ID: 0x0302          Byte: 30      自定义控制器交互数据接口       客户端触发发送    上限频率 30Hz
ID: 0x0303          Byte: 15      客户端小地图交互数据           选手端触发发送
ID: 0x0304          Byte: 12      键盘、鼠标信息                 通过图传串口发送，固定30HZ
ID: 0x0305          Byte: 10      地图接受雷达的数据             频率上限10HZ
ID: 0x0306          Byte: 8       自定义控制器与选手端交互数据   频率上限30HZ
ID: 0x0307          Byte: 103     选手端小地图接受哨兵数据       频率上限1HZ
ID: 0x0308          Byte: 34      选手端小地图接受机器人数据     频率上限3HZ

*/

#define       Judge_Game_StatusData              0x0001
#define       Judge_Game_ResultData              0x0002
#define       Judge_Robot_HP                     0x0003
#define       Judge_Event_Data                   0x0101
#define       Judge_Supply_Station               0x0102
#define       Judge_Referee_Warning              0x0104
#define       Judge_Dart_Countdown               0x0105
#define       Judge_Robot_State                  0x0201
#define       Judge_Power_Heat                   0x0202
#define       Judge_Robot_Position               0x0203
#define       Judge_Robot_Buff                   0x0204
#define       Judge_Aerial_Time                  0x0205
#define       Judge_Injury_State                 0x0206
#define       Judge_RealTime_Shoot               0x0207
#define       Judge_Remaining_Rounds             0x0208
#define       Judge_Robot_RFID                   0x0209
#define       Judge_Dart_Client                  0x020A
#define       Judge_Ground_Position              0x020B
#define       Judge_Radar_Marking                0x020C
#define       Judge_Sentry_Info                  0x020D
#define       Judge_Radar_Info                   0x020E
#define       Judge_Robot_Communicate            0x0301
#define       Judge_User_Defined                 0x0302
#define       Judge_Map_Interaction              0x0303
#define       Judge_KeyMouse_Message             0x0304
#define       Judge_Client_Map                   0x0305
#define       Judge_User_Interaction             0x0306
#define       Judge_Sentinal_Map                 0x0307
#define       Judge_Custom_Info                  0x0308


/***************************DATA_Length*************************/
/*Calculation:frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16)*/

#define       JudgeLength_Game_StatusData        20
#define       JudgeLength_Game_ResultData        10
#define       JudgeLength_Robot_HP               41
#define       JudgeLength_Event_Data             13
#define       JudgeLength_Supply_Station         13
#define       JudgeLength_Referee_Warning        11
#define       JudgeLength_Dart_Countdown         11
#define       JudgeLength_Robot_State            36
#define       JudgeLength_Power_Heat             25
#define       JudgeLength_Robot_Position         25
#define       JudgeLength_Robot_Buff             15
#define       JudgeLength_Aerial_Time            11
#define       JudgeLength_Injury_State           10
#define       JudgeLength_RealTime_Shoot         16
#define       JudgeLength_Remaining_Rounds       15
#define       JudgeLength_Robot_RFID             13
#define       JudgeLength_Dart_Client            15
#define       JudgeLength_Ground_Position        49
#define       JudgeLength_Radar_Marking          15
#define       JudgeLength_Sentry_Info            13      
#define       JudgeLength_Radar_Info             10
#define       JudgeLength_Robot_Communicate      137
#define       JudgeLength_User_Defined           39
#define       JudgeLength_Map_Interaction        24
#define       JudgeLength_KeyMouse_Message       21
#define       JudgeLength_Client_Map             19
#define       JudgeLength_User_Interaction       17
#define       JudgeLength_Sentinal_Map           112
#define       JudgeLength_Custom_Info            43

/**************************************自定义UI机器人指示信息文字*****************************************/

#define       LED_YELLOWMeaning                 "Fric:Manual"
#define       LED_GREENMeaning                  "Magazine:OFF"
#define       LED_ORANGEMeaning                 "ChassisMode:Normal"
#define       LED_PURPLEMeaning                 "VisionMode:None"
#define       LED_PINKMeaning                   "Pitch:Default"

#define       LED_CYANMeaning                   "Resever"
#define       LED_BLACKMeaning                  "SuperCap"
#define       LED_WHITEMeaning                  "Windmill"
#define       LED_Init                          "         "

extern int YellowFlag;
extern int GreenFlag;
extern int OrangeFlag;
extern int PurpleFlag;
extern int PinkFlag;
/**************************************裁判系统结构体*****************************************/
/* 自定义帧头 */
typedef __packed struct
{
    uint8_t  SOF;
    uint16_t DataLength;
    uint8_t  Seq;
    uint8_t  CRC8;

} xFrameHeader;

/* ID: 0x0001    Byte: 11     比赛状态数据 */
typedef struct
{
    union
    {
        uint8_t dataBuff[11];
        __packed struct
        {
            uint8_t game_type : 4;             //比赛类型
            uint8_t game_progress : 4;         //当前比赛阶段
            uint16_t stage_remain_time;        //当前阶段剩余时间  单位s
            uint64_t SyncTimeStamp;            //UNIX时间
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_game_status_t;

/* ID: 0x0002    Byte: 1       比赛结果数据 */
typedef struct
{
    union
    {
        uint8_t dataBuff[1];
        __packed struct
        {
            uint8_t winner;//0平局 1红胜 2蓝胜
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_game_result_t;

/* ID: 0x0003     Byte: 32     比赛机器人血量数据 */
typedef struct
{
    union
    {
        uint8_t dataBuff[32];
        __packed struct
        {
            uint16_t red_1_robot_HP;//红 1 英雄机器人血量，未上场以及罚下血量为 0
            uint16_t red_2_robot_HP;//红 2 工程机器人血量
            uint16_t red_3_robot_HP;//红 3 步兵机器人血量
            uint16_t red_4_robot_HP;//红 4 步兵机器人血量
            uint16_t red_5_robot_HP;//红 5 步兵机器人血量
            uint16_t red_7_robot_HP;//红 7 步兵机器人血量
            uint16_t red_outpost_HP;//红方前哨战血量
            uint16_t red_base_HP;   //红方基地血量
            uint16_t blue_1_robot_HP;
            uint16_t blue_2_robot_HP;
            uint16_t blue_3_robot_HP;
            uint16_t blue_4_robot_HP;
            uint16_t blue_5_robot_HP;
            uint16_t blue_7_robot_HP;
            uint16_t blue_outpost_HP;
            uint16_t blue_base_HP;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_game_robot_HP_t;

/* ID: 0x0101     Byte: 4       场地事件数据 */
typedef struct
{
    union
    {
        uint8_t dataBuff[4];
        __packed struct
        {
            uint32_t event_type;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_event_data_t;

/* ID: 0x0102     Byte: 4       场地补给站动作标识数据 */
typedef struct
{
    union
    {
        uint8_t dataBuff[4];
        __packed struct
        {
            uint8_t reserved;              //之前为补给站口 ID，现在保留了
            uint8_t supply_robot_id;       //补弹机器人 ID
            uint8_t supply_projectile_step;//出弹口开闭状态
            uint8_t supply_projectile_num; //补弹数量
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_supply_projectile_action_t;

/* ID: 0X0104          Byte: 2       裁判警告数据 */
typedef struct
{
    union
    {
        uint8_t dataBuff[3];
        __packed struct
        {
            uint8_t level;              //警告等级：
            uint8_t offending_robot_id; //犯规机器人 ID：判负时，机器人 ID 为 0 黄牌、红牌时，机器人 ID 为犯规机器人 ID
            uint8_t count;              //己方最后一次受到判罚的违规机器人对应判罚等级的违规次数
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_referee_warning_t;

/* ID: 0x0105          Byte: 1       飞镖发射口倒计时  */
typedef struct
{
    union
    {
        uint8_t dataBuff[3];
        __packed struct
        {
            uint8_t dart_remaining_time;//15s 倒计时
			uint16_t dart_info;         //射击信息
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_dart_remaining_time_t;


/* 这个是1.6.1版本的协议 */
/* ID: 0X0201          Byte: 13      机器人状态数据 */
typedef struct
{
    union
    {
        uint8_t dataBuff[13];
        __packed struct
        {
            uint8_t robot_id;
            uint8_t robot_level;
            uint16_t remain_HP;                     //机器人剩余血量
            uint16_t max_HP;                        //机器人上限血量
			uint16_t shooter_barrel_cooling_value;  //枪口每秒冷却值
			uint16_t shooter_barrel_heat_limit;     //枪口热量上限
			uint16_t chassis_power_limit; 			//机器人底盘功率限制上限         
            /*主控电源输出情况：
            0 bit：gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出；
            1 bit：chassis 口输出：1 为有 24V 输出，0 为无 24v 输出；
            2 bit：shooter 口输出：1 为有 24V 输出，0 为无 24v 输出；*/
            uint8_t mains_power_gimbal_output : 1;
            uint8_t mains_power_chassis_output : 1;
            uint8_t mains_power_shooter_output : 1;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_game_robot_status_t;

///* ID: 0X0201          Byte: 13      机器人状态数据 */
//typedef struct
//{
//    union
//    {
//        uint8_t dataBuff[27];
//        __packed struct
//        {
//            uint8_t robot_id;
//            uint8_t robot_level;
//            uint16_t remain_HP;                             //机器人剩余血量
//            uint16_t max_HP;                                //机器人上限血量
//			uint16_t shooter_id1_17mm_barrel_cooling_value; //枪口每秒冷却值
//			uint16_t shooter_id1_17mm_barrel_heat_limit;    //枪口热量上限
//			//机器人1号17mm 发射机构射击初速度上限(单位：m/s)
//			uint16_t shooter_id1_17mm_initial_launching_speed_limit; 
//			uint16_t shooter_id2_17mm_barrel_cooling_value;
//			uint16_t shooter_id2_17mm_barrel_heat_limit; 
//			uint16_t shooter_id2_17mm_initial_launching_speed_limit;
//			uint16_t shooter_id1_42mm_barrel_cooling_value;
//			uint16_t shooter_id1_42mm_barrel_heat_cooling_limit;
//			uint16_t shooter_id1_42mm_initial_launching_speed_limit;     
//			uint16_t chassis_power_limit; 			//机器人底盘功率限制上限         
//            /*主控电源输出情况：
//            0 bit：gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出；
//            1 bit：chassis 口输出：1 为有 24V 输出，0 为无 24v 输出；
//            2 bit：shooter 口输出：1 为有 24V 输出，0 为无 24v 输出；*/
//            uint8_t mains_power_gimbal_output : 1;
//            uint8_t mains_power_chassis_output : 1;
//            uint8_t mains_power_shooter_output : 1;
//        };
//    } data;
//    uint8_t InfoUpdataFlag;
//} ext_game_robot_status_t;

/* ID: 0X0202          Byte: 16      实时功率热量数据 */
typedef struct
{
    union
    {
        uint8_t dataBuff[16];
        __packed struct
        {
            uint16_t chassis_volt;                 //电源管理模块底盘输出电压 单位 毫伏
            uint16_t chassis_current;              //电源管理模块底盘输出电流 单位 毫安
            float chassis_power;                   //底盘输出功率 单位 W 瓦
            uint16_t chassis_power_buffer;         //底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
            uint16_t shooter_id1_17mm_barrel_heat; //枪口热量
            uint16_t shooter_id2_17mm_barrel_heat;
            uint16_t shooter_id1_42mm_barrel_heat;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_power_heat_data_t;

/* ID: 0X0203          Byte: 16      机器人位置数据  */
typedef struct
{
    union
    {
        uint8_t dataBuff[12];
        __packed struct
        {
            float x;  //位置 x 坐标
            float y;  //位置 y 坐标
            float angle;//枪口测速位置朝向
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_game_robot_pos_t;

/* ID: 0X0204          Byte: 5       机器人增益数据 */
typedef struct
{
    union
    {
        uint8_t dataBuff[1];
        __packed struct
        {
            uint8_t power_rune_buff;/*bit 0：机器人血量补血状态
                                      bit 1：枪口热量冷却加速
                                      bit 2：机器人防御加成
                                      bit 3：机器人攻击加成 */
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_buff_t;
/* ID: 0X0205          Byte: 2       空中机器人能量状态数据 */
typedef struct
{
    union
    {
        uint8_t dataBuff[2];
        __packed struct
        {
            uint8_t airforce_status;
            uint8_t time_remain;     //可攻击时间 单位 s。30s 递减至 0
        };
    } data;
    uint8_t InfoUpdataFlag;
} aerial_robot_energy_t;

/* ID: 0X0206          Byte: 1       伤害状态数据 */
typedef struct
{
    union
    {
        uint8_t dataBuff[1];
        __packed struct
        {
            uint8_t armor_id : 4;   /*bit 0-3：当血量变化类型为装甲伤害或测速模块，代表装甲或测速 ID，
                                            其他血量变化类型，该变量数值为 0。*/
            uint8_t hurt_type : 4;  /*bit 4-7：血量变化类型,0x0 装甲伤害扣血；
                                            0x1 模块掉线扣血；
                                            0x2 超射速扣血；
                                            0x3 超枪口热量扣血；
                                            0x4 超底盘功率扣血；
                                            0x5 装甲撞击扣血 */
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_robot_hurt_t;

/* ID: 0X0207          Byte: 7       实时射击数据 */
typedef struct
{
    union
    {
        uint8_t dataBuff[7];
        __packed struct
        {
            uint8_t bullet_type; //子弹类型: 1：17mm 弹丸 2：42mm 弹丸
            uint8_t shooter_id;  //发射机构 ID
            uint8_t bullet_freq; //子弹射频 单位 Hz
            float bullet_speed;  //子弹射速 单位 m/s
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_shoot_data_t;

/* ID: 0X0208          Byte: 6       子弹剩余发送数 */
typedef struct
{
    union
    {
        uint8_t dataBuff[6];
        __packed struct
        {
            uint16_t bullet_remaining_num_17mm; //17mm 子弹剩余发射数目
            uint16_t bullet_remaining_num_42mm; //42mm 子弹剩余发射数目
            uint16_t coin_remaining_num;        //剩余金币数量
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_bullet_remaining_t;

/* ID: 0X0209          Byte: 4       机器人RFID状态 */
/*  bit 位值为 1/0 的含义： 是否已检测到该增益点 RFID
    bit0： 己方基地增益点
    bit1： 己方环形高地增益点
    bit2： 对方环形高地增益点
    bit3： 己方 R3/B3 梯形高地增益点
    bit4： 对方 R3/B3 梯形高地增益点
    bit5： 己方 R4/B4 梯形高地增益点
    bit6： 对方 R4/B4 梯形高地增益点
    bit7： 己方能量机关激活点
    bit8： 己方飞坡增益点（靠近己方一侧飞坡前）
    bit9： 己方飞坡增益点（靠近己方一侧飞坡后）
    bit10： 对方飞坡增益点（靠近对方一侧飞坡前）
    bit11： 对方飞坡增益点（靠近对方一侧飞坡后）
    bit12： 己方前哨站增益点
    bit13： 己方补血点（检测到任一均视为激活）
    bit14： 己方哨兵巡逻区
    bit15： 对方哨兵巡逻区
    bit16： 己方大资源岛增益点
    bit17： 对方大资源岛增益点
    bit18： 己方控制区
    bit19： 对方控制区
    bit20-31：保留
注： 基地增益点，高地增益点，飞坡增益点，前哨站增益点，资源岛增益点，
补血点， 控制区和哨兵巡逻区的 RFID 仅在赛内才生效， 即在赛外，即使检
测到对应的 RFID 卡，对应值也将为 0。*/
typedef struct
{
    union
    {
        uint8_t dataBuff[4];
        __packed struct
        {
            uint32_t rfid_status;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_rfid_status_t;

/* ID: 0x020A          Byte: 6      飞镖机器人客户端指令书 */
typedef struct
{
    union
    {
        uint8_t dataBuff[6];
        __packed struct
        {
            uint8_t dart_launch_opening_status;//当前飞镖发射口的状态
            uint8_t reserved;                  //保留
            uint16_t target_change_time;       //切换打击目标时的比赛剩余时间，单位秒，从未切换默认为 0。
            uint16_t last_dart_launch_time;    //最后一次的发射飞镖的比赛剩余时间，单位秒，初始值为 0。
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_dart_client_cmd_t;

/* ID: 0x020B          Byte: 40      地面机器人位置数据 */
typedef struct
{
    union
    {
        uint8_t dataBuff[40];
        __packed struct
        {
            float hero_x;        //以下均分别表示对应机器人的xy轴坐标
            float hero_y;
            float engineer_x;
            float engineer_y;
            float standard_3_x;
            float standard_3_y;
            float standard_4_x;
            float standard_4_y;
            float standard_5_x;
            float standard_5_y;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_ground_robot_position_t;

/* ID: 0x020C          Byte: 6      雷达标记进度数据 */
typedef struct
{
    union
    {
        uint8_t dataBuff[40];
        __packed struct
        {
            uint8_t mark_hero_progress;          //以下均表示对应机器人的标记进度，值为0-120
            uint8_t mark_engineer_progress;
            uint8_t mark_standard_3_progress;
            uint8_t mark_standard_4_progress;
            uint8_t mark_standard_5_progress;
            uint8_t mark_sentry_progress;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_radar_mark_data_t;


/* 哨兵数据信息： 0x020D */
typedef struct
{
    uint32_t sentry_info;
    uint8_t InfoUpdataFlag;
} ext_sentry_info_t;

/* 雷达数据信息： 0x020E */
typedef struct
{
    uint32_t radar_info;
    uint8_t InfoUpdataFlag;
} ext_radar_info_t;


/*

交互数据包括一个统一的数据段头结构。
数据段包含了内容 ID，发送者以及接收者的 ID 和内容数据段，
整个交互数据的包总共长最大为 128 个字节，
减去 frame_header,cmd_id 和 frame_tail 共 9 个字节以及数据段头结构的 6 个字节，
故而发送的内容数据段最大为 113。
每个机器人交互数据与自定义控制器数据上下行合计带宽不超过 5000 Byte。上下行发送频率分别不超过
30Hz。

机器人 ID：
1，英雄(红)；
2，工程(红)；
3/4/5，步兵(红)；
6，空中(红)；
7，哨兵(红)；
9，雷达站（红）；

101，英雄(蓝)；
102，工程(蓝)；
103/104/105，步兵(蓝)；
106，空中(蓝)；
107，哨兵(蓝)；
109，雷达站（蓝）。

客户端 ID：
0x0101 为英雄操作手客户端(红)；
0x0102，工程操作手客户端((红)；
0x0103/0x0104/0x0105，步兵操作手客户端(红)；
0x0106，空中操作手客户端((红)；

0x0165，英雄操作手客户端(蓝)；
0x0166，工程操作手客户端(蓝)；
0x0167/0x0168/0x0169，步兵操作手客户端步兵(蓝)；
0x016A，空中操作手客户端(蓝)

*/

/*
  由于存在多个内容 ID，但整个 cmd_id 上行频率最大为 10Hz，
  学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
    交互数据 机器人间通信：0x0301。
    发送频率：上限 10Hz

    字节偏移量  大小    说明                       备注
        0        2      数据的内容 ID           0x0200~0x02FF
                                                可以在以上 ID 段选取，具体 ID 含义由参赛队自定义

        2        2      发送者的 ID             需要校验发送者的 ID 正确性，

        4        2      接收者的 ID             需要校验接收者的 ID 正确性，
                                                例如不能发送到敌对机器人的ID

        6        n      数据段                  n 需要小于 113
*/

/* ID: 0X0301          Byte: n       机器人间交互数据 */
typedef __packed struct
{
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

/* data */
typedef __packed struct
{
    uint8_t data[112];//数据段n小于113
} robot_interactive_data_t;

/*机器人交互信息：0x0301*/
typedef __packed struct
{
    xFrameHeader                            txFrameHeader;//帧头
    uint16_t                                CmdID;//命令码
    ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
    robot_interactive_data_t                interactData;//数据段
    uint16_t                                FrameTail;//帧尾
} ext_CommunatianData_t;

/*机器人交互信息：0x0301*/
typedef struct
{
    union
    {
        uint8_t dataBuff[128];
        __packed struct
        {
            ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
			robot_interactive_data_t                interactData;//数据段
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_user_data_t;

/*
    客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0x0100   0x0101  0x0102  0x0103  0x0110  0x0104
    发送频率：上限 10Hz
*/

/* 客户端删除图形 机器人间通信：0x0100 */
typedef __packed struct
{
    uint8_t operate_tpye;
    uint8_t layer;
} ext_client_custom_graphic_delete_t;


/* 图形数据  ID 0x0101*/
typedef __packed struct
{
    uint8_t figure_name[3];
    uint32_t operate_tpye: 3;
    uint32_t figure_tpye: 3;
    uint32_t layer: 4;
    uint32_t color: 4;
    uint32_t details_a: 9;
    uint32_t details_b: 9;
    uint32_t width: 10;
    uint32_t start_x: 11;
    uint32_t start_y: 11;
    uint32_t details_c: 10;
    uint32_t details_d: 11;
    uint32_t details_e: 11;
} interaction_figure_t;

/* 图形数据  ID 0x0102*/
typedef __packed struct
{
    interaction_figure_t interaction_figure[2];
} interaction_figure_2_t;

/* 图形数据  ID 0x0103*/
typedef __packed struct
{
    interaction_figure_t interaction_figure[5];
} interaction_figure_3_t;

/* 图形数据  ID 0x0104*/
typedef __packed struct
{
    interaction_figure_t interaction_figure[7];
} interaction_figure_4_t;

/* 图形数据  ID 0x0110*/
typedef __packed struct
{
    ext_student_interactive_header_data_t grapic_head;
    interaction_figure_t grapic_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;


/* 自定义控制器与机器人交互数据 Byte：上限30  ID 0x0302*/
typedef __packed struct
{
    uint8_t data[30];
    uint8_t InfoUpdataFlag;
} custom_robot_data_t;

/* 选手端小地图交互数据：0x0303 (Byte 15)*/
typedef struct
{
    union
    {
        uint8_t dataBuff[15];
        __packed struct
        {
            float target_position_x;     //目标位置的x坐标
			float target_position_y;     //目标位置的y坐标
			uint8_t cmd_keyboard;        //按下的键盘通用值，没有按下则为0
			uint8_t target_robot_id;     //对方机器人ID，没有则为0
			uint16_t cmd_source;         //信息来源的ID，官方文件疑似有错
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_robot_command_t;

/* 图传遥控客户端下发键鼠遥控信息:0x0304  Byte 12*/
typedef struct
{
    union
    {
        uint8_t dataBuff[12];
        __packed struct
        {
            int16_t mouse_x;
            int16_t mouse_y;
            int16_t mouse_z;        //鼠标滚轮
            int8_t left_button_down;
            int8_t right_button_down;
            uint16_t keyboard_value;
            uint16_t reserved;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_robot_keycommand_t;

/* 选手端接收:0x0305 Byte 10*/
typedef struct
{
    union
    {
        uint8_t dataBuff[10];
        __packed struct
        {
            uint16_t target_robot_ID;
            float target_position_x;
            float target_position_y;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_client_map_command_t;

typedef struct
{
    xFrameHeader                            txFrameHeader;//帧头
    uint16_t                                CmdID;//命令码
    //ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
    ext_client_map_command_t                interactData;//数据段
    uint16_t                                FrameTail;//帧尾
} ext_MapCommunate_t;

/* 自定义控制器与选手端交互数据:0x0306  Byte 8*/
typedef struct
{
    union
    {
        uint8_t dataBuff[8];
        __packed struct
        {
            uint16_t key_value;
            uint16_t x_position: 12;
            uint16_t mouse_left: 4;
            uint16_t y_position: 12;
            uint16_t mouse_right: 4;
            uint16_t reserved;
        };
    } data;
    uint8_t InfoUpdataFlag;
} custom_client_data_t;


/* 选手端小地图接收哨兵数据:0x0307 Byte 103*/
typedef struct
{
    union
    {
        uint8_t dataBuff[105];
        __packed struct
        {
            uint8_t intention;
            uint16_t start_position_x;
            uint16_t start_position_y;
            int8_t delta_x[49];
            int8_t delta_y[49];
			uint16_t sender_id;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_map_sentry_data_t;

/* 己方机器人可通过常规链路向己方任意选手端发送自定义的消息
*该消息会在己方选手端特定位置显示:0x0308 Byte 34         */
typedef struct
{
    union
    {
        uint8_t dataBuff[34];
        __packed struct
        {
            uint16_t sender_id;
			uint16_t receiver_id;
			uint8_t user_data[30];
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_custom_info_t;


void Judge_GetMessage(uint16_t Data_Length);

extern RM_Judge_t JudgeSystem;

extern ext_game_status_t                ext_game_status;
extern ext_game_result_t                ext_game_result;
extern ext_game_robot_HP_t              ext_game_robot_HP;
extern ext_event_data_t ext_even_data;
extern ext_supply_projectile_action_t   ext_supply_projectile_action;
extern ext_referee_warning_t            ext_referee_warning;
extern ext_dart_remaining_time_t        ext_dart_remaining_time;
extern ext_game_robot_status_t          ext_game_robot_state;
extern ext_power_heat_data_t            ext_power_heat_data;
extern ext_game_robot_pos_t             ext_game_robot_pos;
extern ext_buff_t Buff;
extern aerial_robot_energy_t            aerial_robot_energy;
extern ext_robot_hurt_t                 ext_robot_hurt;
extern ext_shoot_data_t                 ext_shoot_data;
extern ext_bullet_remaining_t           ext_bullet_remaining;
extern ext_rfid_status_t                ext_rfid_status;
extern ext_dart_client_cmd_t            ext_dart_client_cmd;
extern ext_ground_robot_position_t      ext_ground_robot_position;
extern ext_radar_mark_data_t            ext_radar_mark_data;
extern ext_user_data_t                  ext_user_data;
extern ext_custom_info_t                ext_custom_info;
extern ext_sentry_info_t                ext_sentry_info;
extern ext_radar_info_t                 ext_radar_info;
extern ext_robot_keycommand_t 			    ext_robot_keycommand;

extern uint8_t JudgeSystem_rxBuff[JUDGESYSTEM_PACKSIZE];

#endif //头文件
