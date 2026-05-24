#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h" // 替换为标准库头文件

void Motor_Init(void);
void Motor_Left_SetSpeed(int16_t speed);
void Motor_Right_SetSpeed(int16_t speed);
void Stop_All_Motors(void);

#endif /* __MOTOR_H */
