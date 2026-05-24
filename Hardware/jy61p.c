// 文件：jy61p.c
// 功能：解析 JY-61P（或类似）姿态传感器串口数据，提取 Roll/Pitch/Yaw（角度，单位：度）
// 说明：
//  - 本文件提供字节流解析函数 jy61p_ReceiveData(uint8_t RxData)，需在串口接收回调/中断中逐字节调用
//  - 支持的数据帧格式（本实现匹配常见 JY 系列格式）：
//      0x55 0x53 D2 D3 D4 D5 D6 D7 D8 D9 CHKSUM
//    其中 0x55 为帧头1，0x53 为帧头2（表示角度数据帧），后 8 字节为数据（按低字节在前，高字节在后），第 10 字节为校验和
//  - 校验方式：前 10 个字节求和与第 11 字节比较，相等则认为帧合法
//  - 角度计算公式：raw_int16 / 32768.0f * 180.0f -> 单位：度
// 结果：全局变量 Roll/Pitch/Yaw 存储解析得到的角度值（float，度）。调用者可直接读取用于姿态显示或控制。

#include "jy61p.h"

static uint8_t RxBuffer[11];    // 接收缓存：用于存储完整的一帧（长度 11）
static volatile uint8_t RxState = 0; // 解析状态机状态：0=等待首字节 0x55，1=等待类型字节，2=接收数据直至完整帧
static uint8_t RxIndex = 0;     // 当前写入 RxBuffer 的索引（0~10）

/* 解析后得到的角度值（单位：度，范围约 -180 ~ +180）
 * 注意：这些值为最新成功解析帧的结果，若在中断中更新，外部读取时如需原子性请加同步保护
 */
float Roll, Pitch, Yaw;
float Angle[3]; // 新增这一行，和Roll/Pitch/Yaw同步
/**
 * @brief  接收并解析来自姿态传感器的单字节数据
 * @param  RxData - 当前接收到的一个字节（在串口接收回调或 ISR 中逐字节调用）
 * @retval 无
 * 
 * 处理流程与结果说明：
 * 1. 使用简单的状态机识别帧头：先匹配 0x55，再匹配 0x53（表示角度帧）
 * 2. 在状态 2 时连续收集后续字节直至收到 11 字节完整帧
 * 3. 完整帧收到后计算前 10 字节之和，与第 11 字节比较校验和
 *    - 校验通过：按小端(short low byte first)合并对应字节得到三个 int16 原始值
 *      * Roll  = (int16_t)(RxBuffer[3] << 8 | RxBuffer[2]) / 32768.0f * 180.0f
 *      * Pitch = (int16_t)(RxBuffer[5] << 8 | RxBuffer[4]) / 32768.0f * 180.0f
 *      * Yaw   = (int16_t)(RxBuffer[7] << 8 | RxBuffer[6]) / 32768.0f * 180.0f
 *    - 校验失败：丢弃该帧（状态回到等待帧头）
 * 4. 解析结果：Roll/Pitch/Yaw 为角度（float，单位度），可直接用于显示或控制算法
 *
 * 注意：
 * - 该函数是字节级解析，需在串口中断或接收回调中被频繁调用
 * - 若串口存在错帧或丢字节情况，状态机会自动重同步（回到等待 0x55）
 * - 若在多线程/中断环境读取 Roll/Pitch/Yaw，请考虑禁用中断或加临界区以保证读取一致性
 */
void jy61p_ReceiveData(uint8_t RxData)
{
    uint8_t i, sum = 0;
    if (RxState == 0)    // 等待帧头 0x55
    {
        if (RxData == 0x55)    // 收到帧头1
        {
            RxBuffer[RxIndex] = RxData;
            RxState = 1;
            RxIndex = 1;    // 进入下一状态，准备接收类型字节
        }
    }
    else if (RxState == 1) // 等待类型字节
    {
        if (RxData == 0x53)    /* 判断是否为角度数据帧（0x53 为 Yaw/Pitch/Roll 类型） */
        {
            RxBuffer[RxIndex] = RxData;
            RxState = 2;
            RxIndex = 2;    // 进入数据接收状态
        }
        else
        {
            // 非期望类型，重置回等待头部（可视需要改为更复杂的同步策略）
            RxState = 0;
            RxIndex = 0;
        }
    }
    else if (RxState == 2)    // 接收数据直到完整帧
    {
        RxBuffer[RxIndex++] = RxData;
        if (RxIndex == 11)    // 接收到完整帧（11 字节）
        {
            for (i = 0; i < 10; i++)
            {
                sum = sum + RxBuffer[i];    // 计算前 10 字节和（无符号累加）
            }
            if (sum == RxBuffer[10])        // 校验和匹配
            {
                /* 解析数据：小端字节序，低字节在前，高字节在后 */
                Roll = ((int16_t)((int16_t)RxBuffer[3] << 8 | (int16_t)RxBuffer[2])) / 32768.0f * 180.0f;
                Pitch = ((int16_t)((int16_t)RxBuffer[5] << 8 | (int16_t)RxBuffer[4])) / 32768.0f * 180.0f;
                Yaw = ((int16_t)((int16_t)RxBuffer[7] << 8 | (int16_t)RxBuffer[6])) / 32768.0f * 180.0f;
                // 结果：Roll/Pitch/Yaw 以度为单位存储在全局变量中
            }
            // 无论校验是否成功，都回到初始状态等待下一个帧头
            RxState = 0;
            RxIndex = 0;
        }
    }
}
