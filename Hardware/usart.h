#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"   
//�¼ӵ�
#include "stdio.h"	
#include "sys.h" 

extern uint8_t openmv_rx_data;       // ���յ�������
extern uint8_t openmv_new_data_flag;  // �����ݵ����־

void usart3_Init(uint32_t bound);
void usart2_Init(uint32_t bound);


#endif

