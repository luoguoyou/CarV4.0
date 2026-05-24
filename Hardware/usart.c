#include "usart.h"
#include "jy61p.h"


//新增 USART2 代码 (用于 OpenMV) 
// 定义全局变量用于 main.c 访问

/**
 * @brief       USART2 初始化 (连接 OpenMV)
 * @param       bound: 波特率 (建议与 OpenMV 端一致，如 9600 或 115200)
 * @retval      无
 */

 void usart2_Init(uint32_t bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    // 1. 开启 GPIOA 和 USART2 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // GPIOA 在 APB2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // USART2 在 APB1
    
    // 2. 配置 PA2 为 USART2_TX (复用推挽输出)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);    
    
    // 3. 配置 PA3 为 USART2_RX (浮空输入)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
     
    // 4. 串口参数配置
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure); 
    
    // 5. 开启接收中断
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
    USART_Cmd(USART2, ENABLE);                     
    
    // 6. NVIC 配置
    // 注意：优先级需根据系统整体调整，这里设为略低于 USART3 或相同
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


/**
 * @brief       USART2 中断服务函数 (接收 OpenMV 指令)
 */
 
// void USART2_IRQHandler(void)
// {
//     if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
//     {
//         // 读取数据
//         openmv_rx_data = USART_ReceiveData(USART2);
        
//         // 置位标志，通知主循环或定时器中断处理
//         openmv_new_data_flag = 1;
        
//         // 清除中断标志
//         USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//     }
// }



//usart3

/**
 * @brief       ????3?????????
 * @param       bound: ??????
 * @retval      ??
 */

 void usart3_Init(uint32_t bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    // 1. ???? GPIOB ?? USART3 ?????
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    
    // 2. ???? PB10 ???????????? (TX)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);    
    
    // 3. ???? PB11 ????????? (RX)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
     
    // 4. ???????????
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure); 
    
    // 5. ?ж?????
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // ?????????ж?
    USART_Cmd(USART3, ENABLE);                     // ??????
    
    // 6. NVIC ?????????
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // ????????
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;       // ????????
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief       ????3?????ж?????? 
 */
void USART3_IRQHandler(void)
{
    uint8_t RxData;
    if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
    {
        RxData = USART_ReceiveData(USART3); /* ??? USART3 ??????????? */
        
        jy61p_ReceiveData(RxData);          /* ????????????????? */
        
        // ???????????λ???????????????????????????????????
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}
