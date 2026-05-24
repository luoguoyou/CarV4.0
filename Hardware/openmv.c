#include "OpenMv.h"
#include "Motor.h"
#include "usart.h"
#include "stdio.h"

#define CMD_START   'S'  // ??/????
#define CMD_STOP    'E'  // ????????????
#define CMD_LEFT    'L'  // ????
#define CMD_RIGHT   'R'  // ????
#define CMD_FINISH  'F'  // ????/????
#define CMD_KE      'K'  // ????????
#define CMD_NONE     0
// ȫ�ֱ�������
volatile uint8_t openmv_cmd = CMD_NONE;

typedef enum {
    Wait_key,          // ??????
    Wait_CMD,       // ????????
    Tracking,           // ????
    Off_line,           // ????
    Dotted_line,        // ????
    Crossing,           // ??????
    Cross_road,         // ??????
    T_road,             // T?????
    Rough_road,         // ??????
    Slope_road,         // ????
    Island,             // ????
    Turn_right,         // ????
    Turn_left,          // ????
    Straight,           // ????
    Stop_car,           // ????


} CarState_t;
//extern uint8_t runningflag;
//#define OPENMV_BASE_SPEED 20

CarState_t car_state = Wait_key; // ?????????
/**
 * @brief ��ʼ�� OpenMV ͨ�Ŵ��� (USART2)
 */
void OpenMV_Init(uint32_t baudrate) {
    usart2_Init(baudrate); // �������� usart.c ��д�õĳ�ʼ������
}


/**
 * @brief �� OpenMV �����ַ�
 */
void OpenMV_SendChar(uint8_t data) {
    USART_SendData(USART2, data);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

/**
 * @brief ��ȡ���������
 */
uint8_t OpenMV_GetCmd(void) {
    uint8_t cmd = openmv_cmd;
    if(cmd != CMD_NONE) {
        openmv_cmd = CMD_NONE;
    }
    return cmd;
}

/**
 * @brief �������� (����ֻ�����򵥵���ͣ��ת���߼���״̬���� TIM3 �д���)
 */
void OpenMV_ProcessCmd(void) {
    uint8_t cmd = OpenMV_GetCmd();
    if (cmd == CMD_NONE) return;

    switch(cmd) {
        case CMD_START:
            // ������ڿ���״̬���յ� S �����ȴ�ָ��״̬
            if(car_state == Wait_key) {
                car_state = Wait_CMD;
            }
            break;
        case CMD_STOP:
            Stop_All_Motors();
            car_state = Stop_car;
            break;
        case CMD_FINISH:
             car_state = Stop_car;
             Stop_All_Motors();
             break;
        default:
            break;
    }
}

// USART2 �жϣ�ֻ�����������
void USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART2);
        // �洢������Чָ����崦������״̬��
        if(data == CMD_LEFT || data == CMD_RIGHT || data == CMD_FINISH || data == CMD_START || data == CMD_STOP) {
            openmv_cmd = data;
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}
