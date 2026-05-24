#include "stm32f10x.h"
#include "Timer.h"

void Timer_Init(void)
{
    // 1. 开启时钟：TIM3 挂在 APB1 总线上
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    // 2. 选择内部时钟源
    TIM_InternalClockConfig(TIM3);
    
    // 3. 时基单元配置 (实现 10ms 中断)
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;      // ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler = 720 - 1;    // PSC (320分频得到100kHz)
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;  // TIM3 无重复计数器，设为 0
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
    
    // 4. 清除更新标志位，防止初始化后立即进入中断
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    
    // 5. 开启定时器更新中断
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    
    // 6. NVIC 优先级配置 (若项目中已统一配置过分组，此处可省略)
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;       // TIM3 中断通道
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
    
    // 7. 启动定时器
    TIM_Cmd(TIM3, ENABLE);
}

///**
// * @brief  TIM3 中断服务函数
// */
// void TIM3_IRQHandler(void)
// {
//    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
//    {

//        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
//    }
// }
