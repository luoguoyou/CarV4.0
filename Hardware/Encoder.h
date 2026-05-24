#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

// 根据你的电机参数修改此值：一圈总脉冲 = 线数 * 倍频(4) * 减速比
#define TOTAL_PPR  1058.0f  

// extern  float Speed_L;
// extern   float Speed_R;
extern volatile int32_t total_L;
extern volatile int32_t total_R;
extern volatile int32_t current_pos;
void Encoder_Init(void);
int16_t Encoder_Get_Count(uint8_t timer_num);
float Encoder_Get_RPM(int16_t delta_count, float sample_time_s);
float Get_Left_Speed(float sample_time_s);
int32_t Encoder_Get_Total_Count(uint8_t timer_num);
float Get_Right_Speed(float sample_time_s);
void Encoder_Sync_Data(float *speedL, float *speedR);
#endif
