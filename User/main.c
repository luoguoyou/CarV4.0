#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "jy61p.h"
#include "Motor.h"
#include "Key.h"
#include "usart.h"
#include "Encoder.h"
#include "track.h"
#include "Timer.h"
#include "openmv.h" 
#include <stdlib.h>
#include <stdio.h>

/* ---------- 循迹参数配置区 ---------- */
#define BASE_SPEED          25      // 小车基础直行速度 (略微提高以应对虚线)
#define PROPORTIONAL_COEFF  0.40f   // 正常循迹比例系数
#define MAX_SPEED           60      // 速度限幅
#define OPENMV_BASE_SPEED   10      // 转向时的速度偏差量
#define LOST_LINE_LIMIT     20       // 允许脱线/全黑的最大计数次数 (6 * 10ms = 60ms)，防虚线误判

/* ---------------------------------- */



// 指令定义
#define CMD_START   'S'  // 启动/开始识别
#define CMD_STOP    'E'  // 错误指令，需重新启动小车
#define CMD_LEFT    0  // 左转指令
#define CMD_RIGHT   1  // 右转指令
#define CMD_FINISH  'F'  // 任务完成/停车指令
#define CMD_KE      'K'  // 开启形状识别指令
#define CMD_NONE     1

// 外部引用
extern float Speed_L;
extern float Speed_R;

// 局部辅助变量（）

static uint16_t vertical_adj_cnt = 0;   // 垂直进入微调计数
static uint16_t delay_timer;     // 通用延时计数器 (如果需要在状态机中实现非阻塞延时)
static uint16_t cross_delay_cnt = 0;    // 路口判断非阻塞延时计数器
static uint8_t turn_init_flag = 0;      // 转向动作初始化标志位



//// 全局变量
static uint8_t white_counter = 0;   // 连续白底计数器 (用于区分虚线和真正脱线)
uint8_t runningflag = 0;        // 按键标志位
uint8_t initial_turn_cmd = CMD_NONE; // 存储OpenMV下发的第一个转向指令 ('L' or 'R')
uint8_t T_flag = 0;// T路口标志位
uint8_t turn_counter = 0;// 转向次数计数器
uint8_t turn_max = 5;  // 最大转向次数,大于这个数字都是右转
uint8_t turn_island = 200;//环岛转弯时间，后面也要改
uint8_t turn_time = 50;// 转向时间 (单位: 10ms, 50 = 500ms)，后面也要改
uint8_t island_flag = 0;// 环岛标志位，进入环岛后不再进行环岛判断




// 小车状态枚举

typedef enum {
    Wait_key,          // 等待按键启动
    Wait_CMD,       // 进入等待指令阶段
    Tracking,           // 循迹逻辑
    Crossing,           // 路口判断逻辑
    Rough_road,         // 粗糙路面逻辑
    Slope_road,         // 斜坡逻辑
    Island,             // 环岛逻辑
    Turn_right,         // 右转逻辑
    Turn_left,          // 左转逻辑
    Stop_car,           // 停车逻辑

} CarState_t;
const char* Get_State_String(CarState_t state)
{
    switch (state)
    {
        case Wait_key:    return "WaitKey";
        case Wait_CMD:    return "WaitCMD";
        case Tracking:    return "Tracking";
        case Crossing:    return "Crossing";
        case Rough_road:  return "Rough";
        case Slope_road:  return "Slope";
        case Island:      return "Island";
        case Turn_right:  return "TurnR";
        case Turn_left:   return "TurnL";
        case Stop_car:    return "Stop";
        default:          return "Unknown";
    }
}
extern CarState_t car_state; // 初始状态为等待按键
/**
 * @brief  TIM3定时器中断服务函数（10ms调用一次）
 * @note   核心控制逻辑在此执行
 */
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {
        Key_Scan(); // 按键扫描
        uint8_t val = Track_GetRaw();     //获取传感器数据 
        int pos = Track_GetWeightPos();   // 位置偏差
        Encoder_Sync_Data(&Speed_L, &Speed_R);// 同步编码器数据
        uint8_t cmd = OpenMV_GetCmd();// 3. 获取 OpenMV 指令 (非阻塞)
        // 如果连续多次检测到 0x00，才认为是真正的“脱线”或“路口”，否则视为虚线间隙。
        if (val == 0x00) // 全白/脱线特征，根据实际传感器调整
            { 
                white_counter++;//
            } 
            else 
            {
                white_counter = 0;//只有连续检测到全白才增加计数器，其他情况重置
            }
// ===================== 状态机逻辑 =====================
        switch (car_state)
    {
            case Wait_key:// 待机，等待按键启动
            Stop_All_Motors();
                white_counter = 0;// 重置连续白底计数器
                turn_init_flag = 0; // 重置转向标志
                if (runningflag == 1) // 按键触发启动
                {
                    car_state = Tracking; // 直接进入循迹状态，等待 OpenMV 指令调整转向
                    initial_turn_cmd = CMD_RIGHT; // 默认初始转向指令为右转，调试转弯
                    initial_turn_cmd = cmd;
                    /*该代码运行没问题，调试关闭二维码扫描功能，直接进入循迹状态，避免繁琐识别过程，便于调试其他功能
                    runningflag = 0; //
                    initial_turn_cmd = CMD_NONE; // 重置初始转向指令
                    car_state = Wait_CMD;// 进入等待指令阶段
                    OpenMV_SendChar(CMD_START); // 发送 'S' 通知 OpenMV 准备
                    */
                }
                break;

            case Wait_CMD:// 进入等待指令阶段, 等待 OpenMV 下发初始方向指令 ('L' 或 'R')
            //     Stop_All_Motors(); 
            //     // 获取转向指令后启动循迹
                // if (initial_turn_cmd == CMD_NONE) 
                // {
                    if (cmd == CMD_LEFT || cmd == CMD_RIGHT) 
                    {
                        //initial_turn_cmd = cmd; // 锁定指令直接进入循迹状态，等待路口特征触发转向
                        car_state = Tracking;   // 循迹逻辑
                    }
                //}
                break;

            case Tracking:// 循迹逻辑


                    // 虚线段 pos 可能波动大，使用稍小的系数或滤波
                    int offset = (int)(pos * PROPORTIONAL_COEFF);
                    int L_Speed = BASE_SPEED + offset;
                    int R_Speed = BASE_SPEED - offset;
                    // 限幅
                    if(L_Speed > MAX_SPEED) L_Speed = MAX_SPEED;
                    if(L_Speed < -MAX_SPEED) L_Speed = -MAX_SPEED;
                    if(R_Speed > MAX_SPEED) R_Speed = MAX_SPEED;
                    if(R_Speed < -MAX_SPEED) R_Speed = -MAX_SPEED;
                    // 设置电机速度
                    Motor_Left_SetSpeed(L_Speed);
                    Motor_Right_SetSpeed(R_Speed);
                
                break;
            case Crossing: // 路口判断逻辑
                Stop_All_Motors();
                // delay_s(1);
                if (cross_delay_cnt > 0) 
                {
                    cross_delay_cnt--;
                }
                else
                { // 延时结束后进行判断
                if((GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12)|GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5)|GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)|GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7))&&
				(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3)==0 & GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1)==0 ))//+字路口特征
                {
                  
                }
                else if(val == 0x00)//T字路口特征
                {

                }
                else if(val == 0xff)//坡道特征
                {
                    car_state = Slope_road;// 跳到斜坡
                }
                else
                {
                    car_state = Tracking;// 跳到循迹逻辑
                    //car_state = Stop_car;
                }
            }
                    break;

         
                case Rough_road://粗糙路面
                if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3) & GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) 
                & GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) & GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1)) // pb3 pb4 pb0 pb1同时为1则判断路口
                {
                    car_state = Tracking;// 跳到循迹逻辑
                    //T_flag = 0;// 恢复T_flag，以便下次进入T字路口时能正确切换状态
                }
                else
                {
                    white_counter = 0;//非脱线区域，防止计数器溢出
                    // 黑底上，循迹失效。保持直行
                    //==========================保持直行============================
                    Motor_Left_SetSpeed(30);
                    Motor_Right_SetSpeed(30);
                    
                }
                break;
            case Slope_road://斜坡
                if(val != 0xff)
                {
                    car_state = Tracking;// 跳到循迹逻辑
                    break;
                }
                else
                {
                    // 全黑上，循迹失效。保持直行
                    //==========================保持直行============================
                    Motor_Left_SetSpeed(30);
                    Motor_Right_SetSpeed(30);
                }
                
                break;
            case Island://环岛
                if(turn_island -- == 0 )
                {
                    Motor_Left_SetSpeed(0);
                    Motor_Right_SetSpeed(30);
                }
                else
                {
                    turn_island = 200;// 重置环岛转向时间
                    car_state = Tracking;// 结束跳到循迹
                    island_flag = 1;// 环岛标志位，进入环岛后不再进行环岛判断
                    break;
                }
                break;
            case Turn_right://右转逻辑
                // 初始化转向计时器
                if (turn_init_flag == 0) {
                    turn_time = 50; // 重置转向时间
                    turn_init_flag = 1;
                }

                 if(turn_time > 0)
                {      
                    Motor_Left_SetSpeed(45);
                    Motor_Right_SetSpeed(0); 
                    turn_time--;         
                }
                
                 else
                {
                    car_state = Tracking;// 跳到循迹逻辑
                    turn_counter++;// 记录转向次数
                    break;
                }
                break; 

           case Turn_left://左转逻辑
                // 初始化转向计时器
                if (turn_init_flag == 0) {
                    turn_time =50; // 重置转向计时器
                    turn_init_flag = 1;
                }

                if(turn_time > 0)
                {    
                    Motor_Left_SetSpeed(0);
                    Motor_Right_SetSpeed(45);
                    turn_time--;            
                }
                else
                {
                    car_state = Tracking;// 跳到循迹逻辑
                    turn_counter++;// 记录转向次数
                    break;
                }
                  break;

                    case Stop_car://停车
                        Stop_All_Motors();
                    break;

		}
               
                

        // 清除中断标志位
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
}
int main(void)
{
    // 1. 硬件初始化
    OLED_Init();
    Key_Init();
    Motor_Init();
    Timer_Init();       // 开启 TIM3 中断 (10ms)
    Encoder_Init();
    Track_Init();
    
    // 设置循迹模式（白底黑线）
    Track_SetMode(MODE_WHITE_PAPER_BLACK_LINE);
    
    // 初始化串口
    usart3_Init(9600);  // 陀螺仪 (如果用到)
    OpenMV_Init(9600);  // OpenMV 通信

    OLED_Clear();
    OLED_ShowString(0, 0, "System Ready", OLED_6X8);
    OLED_ShowString(0, 16, "Press Key1", OLED_6X8);
    OLED_Update();

    // 2. 主循环 (仅用于显示刷新，控制逻辑在中断中)
    while(1)
    {
        // 实时显示调试信息
        uint8_t val = Track_GetRaw();
        int pos = Track_GetWeightPos();
        
        OLED_ShowTrackStatus(0, 0, val);
        OLED_Printf(0, 16, OLED_6X8, "L:%.1f R:%.1f", Speed_L, Speed_R);
        OLED_Printf(0, 32, OLED_6X8, "Pos:%d St:%d", pos, car_state);
        //OLED_Printf(0, 48, OLED_6X8, "Cmd:%c", initial_turn_cmd ? initial_turn_cmd : '-');
        OLED_Printf(0, 48, OLED_6X8,"state:%s", Get_State_String(car_state));
        OLED_Update();
        
        delay_ms(50); // 降低刷新率，避免OLED刷屏占用过多CPU
    }
}
