#ifndef __TRACK_H
#define __TRACK_H

#include "stm32f10x.h"

/* ---------------- 模式配置 ---------------- */
typedef enum {
    MODE_WHITE_PAPER_BLACK_LINE = 0, // 白底黑线 (高电平1代表发现线)
    MODE_BLACK_PAPER_WHITE_LINE = 1  // 黑底白线 (取反处理)
} TrackMode_t;

/* ---------------- 状态机定义 ---------------- */
typedef enum {
    STATE_FOLLOWING,    // 正常循迹中
    STATE_Y_JUNCTION,   // 遇到Y字/分叉口
    STATE_CROSS_ROAD,   // 遇到十字/横线
    STATE_LOST          // 脱轨/全白
} TrackState_t;

/* ---------------- 基础接口 ---------------- */
void Track_Init(void);
void Track_SetMode(TrackMode_t mode);
uint8_t Track_GetRaw(void);   // 获取原始逻辑位
int Track_GetWeightPos(void); // 获取加权位置 (-100 ~ 100)
void OLED_ShowTrackStatus(uint8_t x, uint8_t y, uint8_t sensor_val);

///* ---------------- 特殊路口判断 ---------------- */
//uint8_t Track_Check_Y(uint8_t sensor_val);
//uint8_t Track_Check_Lost(uint8_t sensor_val);

#endif
