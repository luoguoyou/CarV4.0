#ifndef __OPENMV_H
#define __OPENMV_H

#include "stm32f10x.h"
#include "sys.h"


void OpenMV_Init(uint32_t baudrate);
uint8_t OpenMV_GetCmd(void);
void OpenMV_ProcessCmd(void);
void OpenMV_SendChar(uint8_t data); // 新增：发送函数

#endif
