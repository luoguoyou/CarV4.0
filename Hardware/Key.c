#include "stm32f10x.h"
#include "Delay.h"
#include "Key.h"

// 外部声明你的运行模式标志
//uint8_t runningflag;
extern float Target_yaw ,Yaw;
static uint8_t key1_cnt = 0;
static uint8_t key2_cnt = 0;

void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    // PB5 上拉输入
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // PA4 上拉输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// 必须放在 10ms 中断里
void Key_Scan(void)
{
    uint8_t key1 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);
    uint8_t key2 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4);

    // ==================== PB5 按下 → runningflag = 1 ====================
    if(key1 == 0)
    {
        key1_cnt++;
        if(key1_cnt >= 3)  // 按下 ≥30ms
        {
            runningflag = 1;
            key1_cnt = 10; // 防止重复触发
        }
    }
    else
    {
        key1_cnt = 0;
    }

    // ==================== PA4 按下 → runningflag = 2 ====================
    if(key2 == 0)
    {
        key2_cnt++;
        if(key2_cnt >= 3)  // 按下 ≥30ms
        {

            runningflag = 2;
            key2_cnt = 10;
        }
    }
    else
    {
        key2_cnt = 0;
    }
}

