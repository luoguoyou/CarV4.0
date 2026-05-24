// 文件：Motor.c
// 功能：电机底层驱动（初始化、左右电机速度设置、全部停止）
// 说明：
//  - 使用 TIM1 产生 PWM 驱动电机（通道1 对应左电机，通道4 对应右电机）
//  - 使用 PB12~PB15 作为电机方向控制引脚（通过高/低电平组合决定电机正反转）
//  - SetSpeed 接口传入带符号的速度值：正数表示一个方向，负数表示相反方向，0 表示停止
//  - 实际输出到定时器的是占空比（使用 TIM_SetCompareX），参数应在 0~周期范围内
// 结果：
//  - Motor_Init() 配置并启动 PWM，初始状态为停止
//  - Motor_Left_SetSpeed / Motor_Right_SetSpeed 根据符号设置方向引脚并设置占空比
//  - Stop_All_Motors() 将两侧速度设为 0 并断开方向引脚输出（电机制动/空转视硬件电路） 

#include "motor.h"
#include <stdlib.h>

/**
 * @brief  电机底层硬件初始化
 * @note   包含 TIM1 PWM 通道和方向控制 GPIO 的配置
 * @result 初始化完成后 TIM1 启动，PWM 输出为 0，占空比为 0（电机停止）
 * @注意   请在系统时钟与 GPIO 时钟启用后调用；若使用不同的硬件引脚或定时器需相应修改
 */
void Motor_Init(void) {
    // 1. 开启时钟 (必须先开启时钟再配置寄存器)
    // 需要开启：TIM1(PWM), GPIOA(PWM引脚), GPIOB(方向引脚), AFIO(复用功能)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | 
                           RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    // 2. 配置方向控制引脚: PB12, PB13, PB14, PB15
    // 对应 HAL 库的 GPIO_MODE_OUTPUT_PP 和 GPIO_SPEED_FREQ_LOW
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   // 2MHz 对应 FREQ_LOW
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 3. 配置 PWM 输出引脚: PA8 (CH1), PA11 (CH4)
    // 注意：PWM 引脚必须配置为“复用推挽输出” (AF_PP)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 4. 定时器基础配置 (72MHz 主频下，10kHz PWM)
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Prescaler = 71;           // 预分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 99;              // 自动重装载值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // 5. PWM 通道配置
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);             // 填充默认值
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;                  // 初始占空比为 0
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);            // 初始化通道 1 (左)
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);            // 初始化通道 4 (右)

    // 6. 启动定时器 (对应 HAL_TIM_PWM_Start)
    TIM_Cmd(TIM1, ENABLE);                              // 开启计数器
    TIM_CtrlPWMOutputs(TIM1, ENABLE);                   // 高级定时器 MOE 主输出使能

    // 初始状态停止电机
    Stop_All_Motors();
}

/**
 * @brief  左电机速度设置
 * @param  speed: 带符号速度值（正为一个方向，负为相反方向，0 停止）
 * @note   该函数通过设置 PB14/PB15 两个引脚的高低电平来控制左电机转向
 *         并通过 TIM_SetCompare1 设置 PWM 占空比（绝对值作为占空比大小）
 * @result 调用后电机朝指定方向以对应占空比转动；speed 的绝对值越大占空比越大，电机转速越高
 */
void Motor_Left_SetSpeed(int16_t speed) {
    if (speed > 0) {
        GPIO_SetBits(GPIOB, GPIO_Pin_15);
        GPIO_ResetBits(GPIOB, GPIO_Pin_14);
    } else if (speed < 0) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);
        GPIO_SetBits(GPIOB, GPIO_Pin_14);
    } else {
        GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15);
    }
    TIM_SetCompare1(TIM1, (uint16_t)abs(speed));
}

/**
 * @brief  右电机速度设置
 * @param  speed: 带符号速度值（正为一个方向，负为相反方向，0 停止）
 * @note   该函数通过设置 PB12/PB13 两个引脚的高低电平来控制右电机转向
 *         并通过 TIM_SetCompare4 设置 PWM 占空比（绝对值作为占空比大小）
 * @result 调用后电机朝指定方向以对应占空比转动；speed 的绝对值越大占空比越大，电机转速越高
 */
void Motor_Right_SetSpeed(int16_t speed) {
    if (speed > 0) {
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        GPIO_ResetBits(GPIOB, GPIO_Pin_13);
    } else if (speed < 0) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        GPIO_SetBits(GPIOB, GPIO_Pin_13);
    } else {
        GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13);
    }
    TIM_SetCompare4(TIM1, (uint16_t)abs(speed));
}

/**
 * @brief  停止所有电机
 * @result 将左右电机占空比置为 0，并将方向引脚置为低电平（停止状态）
 * @note   具体停止效果（制动或空转）由电机驱动硬件决定
 */
void Stop_All_Motors(void) {
    Motor_Left_SetSpeed(0);
    Motor_Right_SetSpeed(0);
}
