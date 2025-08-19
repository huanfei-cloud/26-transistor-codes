# Draft

#### 介绍
机器人队全向轮以实践模板框架

#### 软件架构
总的来说就是每个文件夹中，.c文件放.c，.h放.h文件。keil内的工程结构与电脑上的文件结构一致

1. Algorithm文件夹中放算法文件，包括pid、滤波等
2. APPs文件夹中放具体实现的功能，比如小车的运动，运动学解算等
3. Devices文件夹中放对设备的控制文件，比如电机协议以及控制
4. BSP文件夹中放对板子上外设的操作，比如CAN通信的具体实现，串口通信每帧数据的编写等
5. Tasks文件中放最终的简洁的任务函数。

变量名字不需要特别统一，最后用结构体的方法连在一起就可以。变量命名尽量用下划线连接的办法。具体实现方法如下：

```c
// 假设现在要实现一个任务Robot_Control，它需要用到底盘的各种数据和函数。
// 第一步在APPs对应的具体功能的.h文件中建立两个结构体，一个装数据，一个装函数，如：
/* 底盘运动 */
typedef struct
{
    float targetXRaw;         //底盘x轴原始数据
    float targetYRaw;         //底盘y轴原始数据
    float targetZRaw;         //底盘z轴原始数据
    float LpfAttFactor;       //底盘滤波系数
    float targetXLPF;         //底盘x轴滤波后数据
    float targetYLPF;         //底盘y轴滤波后数据
    float targetZLPF;         //底盘z轴滤波后数据
    float speedLimit;         //底盘速度限制
    uint16_t OmegaLimit;      //底盘速度限制
    float Trace_Distance;     //跟随装甲板距离
    float FollowtargetYawRaw; //底盘目标Yaw轴跟随云台原始数据
    float FollowtargetYawLPF; //底盘Yaw轴跟随云台滤波后数据

    float SpeedChange_Factor;  //速度改变因子
    float SpeedLimit_Factor;   //限速因子
    uint8_t mode;              //底盘控制模式
    uint8_t swingFlag;         //扭腰标志位
    float spinSpeed;           //自旋速度
    float swingSpeed;          //扭腰速度
    uint8_t PowerOverflowFlag; //超功率标志位
} Chassis_t;

typedef struct
{
    void (*Chassis_Init)(void);
    void (*Chassis_processing)(float Vx, float Vy, float VOmega);
    void (*ChassisCapControl)(void);
} Chassis_FUN_t;
// 强烈建议在必要的时候使用枚举、位域、联合体等数据结构精简代码和内存

// 第二步编写对应的代码即可
// 此处省略数据的获取以及具体的函数

// 第三步创建初始化宏定义，即对应着结构体的结构。如对应上方的两个结构体，初始化宏定义应该为（第一个没有找到对应的在哪里，找了个其他数据结构体的）：
#define LBWHEEL_PID_PARAM     \
    {                         \
        0,                    \
            0,                \
            0,                \
            0,                \
            0,                \
            13.0f,            \
            0.5f,             \
            0.0f,             \
            0,                \
            0,                \
            0,                \
            0,                \
            M3508_MaxOutput,  \
            3000,             \
            &Incremental_PID, \
    }

#define Chassis_FUNGroundInit    \
    {                            \
        &Chassis_Init,           \
            &Chassis_processing, \
            &ChassisCapControl,  \
    }

// 第四步，连接。也就是建立一个真正要用的对应的结构体变量，然后取等号就可以。需要注意的是，这一步之前所有宏定义中用到的东西都要定义好。尤其需要注意的就是函数，一定要在这一步之前先把对应的函数声明了。
// 最后也可以像他一样undef，把之前用的宏定义删了，节省空间。
Chassis_FUN_t Chassis_FUN = Chassis_FUNGroundInit;
#undef Chassis_FUNGroundInit
```



#### 参与贡献

1.  Fork 本仓库
2.  新建 Feat_xxx 分支
3.  提交代码
4.  新建 Pull Request
