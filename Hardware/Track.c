#include "track.h"    // 循迹模块驱动头文件
#include "OLED.h"     // OLED显示屏驱动头文件

// 静态全局变量：仅当前文件可访问，用于存储循迹模式（封装性，外部无法直接修改）
static TrackMode_t g_current_mode = MODE_WHITE_PAPER_BLACK_LINE;

/**
 * @brief  循迹传感器GPIO初始化函数
 * @note   重要知识点：STM32F103的PB3、PB4默认是JTAG调试引脚，
 *         必须关闭JTAG功能才能作为普通IO口使用！
 * @param  无
 * @retval 无
 */
void Track_Init(void) {
    GPIO_InitTypeDef gpio;  // 定义GPIO初始化结构体变量

    // 1. 开启外设时钟
    // GPIOA、GPIOB：循迹传感器使用的IO口
    // AFIO：复用功能时钟，必须开启才能进行引脚重映射（释放PB3/PB4）
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA | 
        RCC_APB2Periph_GPIOB | 
        RCC_APB2Periph_AFIO, 
        ENABLE
    );

    // 2. 关闭JTAG调试功能，释放 PB3、PB4 作为普通IO口使用
    // 这是使用PB3、PB4必须加的配置，否则IO口无法正常输入
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    // 3. 配置GPIO模式
    // 循迹传感器输出5V高电平表示检测到黑线
    // 配置为：下拉输入（GPIO_Mode_IPD），无信号时默认为低电平
    gpio.GPIO_Mode = GPIO_Mode_IPD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;  // IO口速度，输入模式下可随意配置

    // 4. 初始化GPIOA组循迹传感器引脚
    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_12;
    GPIO_Init(GPIOA, &gpio);

    // 5. 初始化GPIOB组循迹传感器引脚
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_Init(GPIOB, &gpio);
}

/**
 * @brief  设置循迹传感器工作模式
 * @param  mode：循迹模式
 *         MODE_WHITE_PAPER_BLACK_LINE：白底黑线模式（默认）
 *         MODE_BLACK_PAPER_WHITE_LINE：黑底白线模式
 * @retval 无
 */
void Track_SetMode(TrackMode_t mode) {
    g_current_mode = mode;  // 将用户设置的模式存入全局变量
}

/**
 * @brief  获取8路循迹传感器原始状态（8位二进制）
 * @param  无
 * @retval uint8_t：8路传感器状态
 *         bit0 ~ bit7 对应 8个传感器（左→右）
 *         1 = 检测到黑线 / 白线（由模式决定）
 *         0 = 未检测到
 */
uint8_t Track_GetRaw(void) {
    uint8_t raw = 0;  // 存储8路传感器原始数据，初始化为全0

    // ===================== 传感器物理排列 =====================
    // 顺序：最左 → 最右
    // PB3、PB4、PA12、PA5、PA6、PA7、PB0、PB1
    // ==========================================================

    // bit0：最左侧传感器 PB3
    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3))  raw |= 0x01;
    // bit1：左2传感器 PB4
    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4))  raw |= 0x02;
    // bit2：左3传感器 PA12
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12)) raw |= 0x04;
    // bit3：左4传感器 PA5
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5))  raw |= 0x08;
    // bit4：右4传感器 PA6
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6))  raw |= 0x10;
    // bit5：右3传感器 PA7
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7))  raw |= 0x20;
    // bit6：右2传感器 PB0
    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0))  raw |= 0x40;
    // bit7：最右侧传感器 PB1
    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1))  raw |= 0x80;

    // ===================== 模式切换 =====================
    // 黑底白线：需要将原始数据取反（0变1，1变0）
    if (g_current_mode == MODE_BLACK_PAPER_WHITE_LINE) {
        return ~raw;  // 按位取反
    }

    // 白底黑线：直接返回原始数据
    return raw;
}

/**
 * @brief  加权计算循迹偏差位置（核心算法）
 * @param  无
 * @retval int：偏差值
 *         负数 → 小车偏右
 *         正数 → 小车偏左
 *         0    → 小车在中心
 *         范围：-80 ~ +80
 */
int Track_GetWeightPos(void) {
    uint8_t v = Track_GetRaw();     // 获取8路传感器原始状态
    static int last_pos = 0;        // 静态变量：保存上一次有效位置（掉电不丢失）

    // ===================== 脱线处理 =====================
    // 所有传感器全0 → 小车完全脱离轨道
    if (v == 0x00) {
        return last_pos;  // 返回最后一次有效位置，保持转弯姿态防止冲出跑道
    }

    int32_t sum = 0;    // 加权总和
    int32_t count = 0;   // 检测到黑线的传感器数量

    // 8路传感器加权系数表（左负右正，越靠边权重绝对值越大）
    static const int weights[8] = {-80, -55, -25, -5, 5, 25, 55, 80};

    // 遍历8个传感器，计算加权平均值
    for (int i = 0; i < 8; i++) {
        // 判断当前传感器是否检测到黑线
        if (v & (1 << i)) {
            sum += weights[i];   // 累加权重
            count++;             // 统计检测到黑线的传感器数量
        }
    }

    last_pos = (int)(sum / count);  // 计算加权平均位置，并保存
    return last_pos;                // 返回最终偏差值
}

/**
 * @brief  在OLED上显示8路循迹传感器状态（0/1字符串）
 * @param  x：显示起始X坐标
 * @param  y：显示起始Y坐标
 * @param  sensor_val：传感器8位原始值
 * @retval 无
 */
void OLED_ShowTrackStatus(uint8_t x, uint8_t y, uint8_t sensor_val) {
    char str[9];  // 存储8位01字符串 + 结束符'\0'

    // 循环读取8位传感器状态，转为 '0' 和 '1'
    // 从 bit0（最左）到 bit7（最右）依次显示
    for (int i = 0; i < 8; i++) {
        if (sensor_val & (0x01 << i)) {
            str[i] = '1';  // 检测到黑线 → 显示1
        } else {
            str[i] = '0';  // 未检测到 → 显示0
        }
    }
    str[8] = '\0';  // 字符串结束符，必须加！

    // 在OLED上格式化输出循迹状态字符串
    OLED_Printf(x, y, OLED_6X8, "Track:[%s]", str);
}


