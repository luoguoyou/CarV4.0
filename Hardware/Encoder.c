// 文件：Encoder.c
// 功能：编码器驱动与速度计算
// 说明：
//  - 使用定时器 TIM2（左）和 TIM4（右）工作在正交编码器模式读取 AB 相脉冲
//  - 不使用中断，直接读取定时器计数并清零来获取一定时间窗口内的脉冲增量
//  - 提供速度转换（RPM）与同步函数，用于在固定采样周期（建议 10ms）内更新速度变量
// 结果：
//  - total_L/total_R：累计脉冲数，可用于里程计算（单位：脉冲）
//  - current_pos：基于左右轮平均的当前位置（单位：脉冲）
//  - Speed_L/Speed_R：实时转速，单位 RPM（转/分钟），需每 10ms 调用同步函数以保证准确性

#include "Encoder.h"

// 全局变量：左右轮总脉冲数（volatile防止编译器优化，中断/定时读取要用）
// total_L/total_R 为累计脉冲数，单位：脉冲（pulse）, 可用于计算总里程或圈数
volatile int32_t total_L = 0;  // 左电机累计编码器脉冲
volatile int32_t total_R = 0;  // 右电机累计编码器脉冲
volatile int32_t current_pos = 0; // 小车当前位置（左右轮平均，单位：脉冲）

// L_Speed / R_Speed 为实时速度（RPM），由 Encoder_Sync_Data 更新
float Speed_L;
float Speed_R;
/**
  * @brief  编码器接口初始化（使用定时器2、定时器4的正交编码器模式）
  * @对应接线：
  *       TIM2 —— 左编码器 CH1=PA0  CH2=PA1
  *       TIM4 —— 右编码器 CH1=PB6  CH2=PB7
  * @原理   定时器编码器模式：AB相脉冲自动计数，不用写中断！
  * @结果   TIM2/TIM4 启动并计数，计数寄存器初始为 0
  * @注意   请在调用任何读取函数前先调用该函数完成硬件配置
  */
void Encoder_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    // 1. 开时钟
    // TIM2、TIM4 是APB1总线上的定时器
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE);
    // PA、PB口是APB2总线上的
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    // 2. 配置GPIO口：编码器A/B相 → 上拉输入模式
    // 编码器没有信号时默认为高电平，防止干扰
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    // 上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    // 左编码器：PA0、PA1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 右编码器：PB6、PB7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 3. 定时器时间基准配置（编码器模式不需要关心频率）
    TIM_TimeBaseStructure.TIM_Prescaler = 0;         // 预分频器 = 0，不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseStructure.TIM_Period = 65535;        // 自动重装载值最大（0~65535）
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // 4. 【核心】配置定时器为编码器模式
    // TIM_EncoderMode_TI12：A相、B相都计数（4倍频精度最高）
    // 两个边沿都计数 → 转一圈得到的脉冲数 = 电机线数 * 4

    // 左编码器 TIM2
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    // 右编码器 TIM4
    // 注意：这里用 Falling 是为了匹配小车电机正反转方向
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);

    // 5. 输入滤波：防止电机抖动干扰计数
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10; // 滤波强度 0~15，越大越稳
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    // 6. 计数器清零 + 启动定时器
    TIM_SetCounter(TIM2, 0);
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

/**
  * @brief  读取定时器的增量脉冲，并自动清零
  * @param  timer_num：2=左电机  4=右电机
  * @return 这段时间内的脉冲增量（带正负），类型 int16_t
  * @结果  返回的值是从上次读取到当前的脉冲差（单位：脉冲）
  * @注意  读取后会立即把计数器清零，适用于在固定时间窗口（例如 10ms）内采样
  */
int16_t Encoder_Get_Count(uint8_t timer_num) {
    int16_t count;
    if (timer_num == 2) {
        count = (int16_t)TIM_GetCounter(TIM2); // 读取当前计数值
        TIM_SetCounter(TIM2, 0);               // 读完立刻清零
    } else if (timer_num == 4) {
        count = (int16_t)TIM_GetCounter(TIM4);
        TIM_SetCounter(TIM4, 0);
    } else {
        count = 0;
    }
    return count;
}

/**
  * @brief  获取电机累计总脉冲（可用来算距离、圈数）
  * @param  timer_num：2=左电机  4=右电机
  * @return 累计脉冲数（int32_t）
  * @结果  可结合每圈脉冲数(TOTAL_PPR)换算为圈数或距离
  */
int32_t Encoder_Get_Total_Count(uint8_t timer_num) {
    if (timer_num == 2) return total_L;
    if (timer_num == 4) return total_R;
    return 0;
}

/**
  * @brief  根据脉冲数计算转速 RPM（每分钟多少转）
  * @param  delta_count：这段时间的脉冲数（int16_t）
  * @param  sample_time_s：采样时间（单位：秒）
  * @return 转速，单位 RPM（float）
  * @公式  RPM = (delta_count / TOTAL_PPR) * (60 / sample_time_s)
  * @结果  当 sample_time_s 为 0 时返回 0，避免除0
  * @注意  delta_count 为脉冲数，可能为负值（代表反转）
  */
float Encoder_Get_RPM(int16_t delta_count, float sample_time_s) {
    if (sample_time_s <= 0) return 0.0f;

    // 公式：
    // 1秒转多少圈 = 脉冲数 / 一圈总脉冲数
    // 1分钟转多少圈 = 上面 × 60
    return ((float)delta_count / TOTAL_PPR) * (60.0f / sample_time_s);
}

/**
  * @brief  获取左电机转速（RPM）
  * @param  sample_time_s：采样时间（秒），例如固定调用间隔 0.01f（10ms）
  * @return 左电机转速，单位 RPM
  * @结果  内部调用 Encoder_Get_Count(2) 会清零 TIM2 计数器
  */
float Get_Left_Speed(float sample_time_s) {
    return Encoder_Get_RPM(Encoder_Get_Count(2), sample_time_s);
}

/**
  * @brief  获取右电机转速（RPM）
  * @param  sample_time_s：采样时间（秒）
  * @return 右电机转速，单位 RPM
  * @结果  内部调用 Encoder_Get_Count(4) 会清零 TIM4 计数器
  */
float Get_Right_Speed(float sample_time_s) {
    return Encoder_Get_RPM(Encoder_Get_Count(4), sample_time_s);
}

/**
  * @brief  编码器数据同步函数（培训重点！）
  * @param  speedL 左电机速度（传出，单位 RPM）
  * @param  speedR 右电机速度（传出，单位 RPM）
  * @用法   必须每隔 10ms 调用一次！！！（例如放在 TIM3 10ms 中断内）
  * @步骤/结果：
  *    1. 读取 10ms 内左右脉冲增量（Encoder_Get_Count，会清零计数器）
  *    2. 累加到 total_L/total_R（用于里程统计）
  *    3. 更新 current_pos 为左右平均累计值（用于里程或位置估算）
  *    4. 根据 delta 和采样时间 0.01s 计算 RPM 并写入传出参数
  * @注意  如果调用间隔不是 10ms，请修改本函数中传入 sample_time 或在调用处调整使用公式
  */
void Encoder_Sync_Data(float *speedL, float *speedR) {
    // 1. 读取10ms内的脉冲增量
    int16_t delta_L = Encoder_Get_Count(2);
    int16_t delta_R = Encoder_Get_Count(4);

    // 2. 累加到总里程（小车跑了多远）
    total_L += delta_L;
    total_R += delta_R;

    // 3. 小车当前位置 = 左右轮平均里程
    current_pos = (total_L + total_R) / 2;

    // 4. 计算速度（固定10ms = 0.01s）
    *speedL = Encoder_Get_RPM(delta_L, 0.01f);
    *speedR = Encoder_Get_RPM(delta_R, 0.01f);
}



