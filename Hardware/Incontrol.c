/**
  ******************************************************************************
  * @file    Incontrol.c
  * @brief   红外遥控接收与 NEC 协议解码模块
  *
  * 功能概述：
  *   通过红外接收头（如 VS1838B/HX1838 等）接收红外遥控器信号，并按照
  *   NEC 红外编码协议进行解码，提取出 32 位数据码供主程序使用。
  *
  * NEC 红外协议简介（最常见的中外遥控器协议）：
  *
  *   一帧完整数据 = 引导码 + 地址码(8位) + 地址反码(8位)
  *                 + 数据码(8位) + 数据反码(8位)
  *   总计 32 位数据。
  *
  *   ┌──────────────────────────────────────────────────────┐
  *   │ 引导码    │ 地址码(8位) │ 地址反码(8位) │ 数据(8位) │ 数据反码(8位) │
  *   │ 9ms+4.5ms│   8位       │    8位       │   8位    │    8位      │
  *   └──────────────────────────────────────────────────────┘
  *
  *   位"0"编码：560μs 低电平 + 560μs 高电平  （总时长约 1120μs）
  *   位"1"编码：560μs 低电平 + 1680μs 高电平 （总时长约 2240μs）
  *
  *   引导码：9ms 低电平 + 4.5ms 高电平
  *   重复码：9ms 低电平 + 2.25ms 高电平（按键长按时发送，本项目未处理）
  *
  * 硬件连接：
  *   - 红外接收头数据引脚 → PA8
  *   - 使用外部中断 EXTI_Line8，下降沿触发（信号从高变低时触发中断）
  *   - 红外接收头输出为反相逻辑：有红外信号=低电平，无红外信号=高电平
  *
  * 解码策略（本项目的实现方式）：
  *   1. 外部中断检测到下降沿 → 进入 EXTI9_5_IRQHandler()
  *   2. 在中断服务函数中，通过测量高电平持续时间来区分：
  *      - 引导码的高电平（4.5ms ≈ 225 × 20μs → Tim=200~250）
  *      - 数据"1"的高电平（1680μs ≈ 84 × 20μs → Tim=60~90）
  *      - 数据"0"的高电平（560μs ≈ 28 × 20μs → Tim=10~50）
  *      - 重复码的高电平（2.25ms ≈ 112 × 20μs）→ 超时(Tim≥250)退出
  *   3. 接收满 32 位数据后，置位 IR_Receiveflag = 1，通知主循环处理
  *
  * 注意：本解码方法在中断中占用时间较长（约几十毫秒），不适合对
  *       实时性要求极高的场景。对于学习用途完全够用。
  ******************************************************************************
  */

#include "stm32f10x.h"    // STM32F10x 标准外设库头文件
#include "Delay.h"        // 延时函数（IRremote_Counttime 中使用 Delay_us(20)）

/* ── 【新增】外部全局变量声明 ──
 * g_sys_tick_ms:        系统毫秒计数器，由 SysTick_Handler 每1ms自增（定义于 main.c）
 * g_last_ir_activity_ms: 最后一次红外活动的时刻戳，在 EXTI 中断中更新（定义于 main.c）
 */
extern volatile uint32_t g_sys_tick_ms;
extern volatile uint32_t g_last_ir_activity_ms;

/* ==================== 红外接收硬件引脚宏定义 ==================== */
#define IRED_PORT      GPIOA                // 红外接收头连接的 GPIO 端口
#define IRED_PIN       GPIO_Pin_8           // 红外接收头连接的引脚（PA8）
#define IRED_PORT_RCC  RCC_APB2Periph_GPIOA // GPIOA 时钟使能宏

/* ==================== 全局变量 ==================== */

/*
 * 接收到的 32 位红外数据码（NEC 格式）
 * 位结构：Bit31~24=地址码, Bit23~16=地址反码, Bit15~8=数据码, Bit7~0=数据反码
 * 在主循环中使用 IR_Receivecode >> 8 即可提取出数据码
 */
uint32_t IR_Receivecode;

/*
 * 红外接收完成标志位
 * = 0 : 未接收到新数据（空闲/等待）
 * = 1 : 已成功接收完整一帧数据，等待主循环读取
 * 主循环读取后应手动清零
 */
uint8_t  IR_Receiveflag;

/* ==================== 函数实现 ==================== */

/**
  * @brief  红外遥控接收模块初始化
  * @param  无
  * @retval 无
  *
  * 初始化内容包括：
  *   1. 使能 GPIOA 和 AFIO 时钟（AFIO 用于配置外部中断引脚映射）
  *   2. 配置 PA8 为上拉输入模式（红外接收头空闲时输出高电平）
  *   3. 配置 PA8 为外部中断 EXTI_Line8，下降沿触发
  *   4. 配置 NVIC 中断优先级：抢占优先级0，响应优先级1
  *
  * PA8 中断通道归属：EXTI9_5_IRQn（EXTI Line5~9 共用一个中断向量）
  */
void IRremote_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* --- 第1步：开启 GPIOA 和 AFIO 时钟 ---
     * AFIO（复用功能IO）时钟必须开启，否则无法配置外部中断引脚映射
     */
    RCC_APB2PeriphClockCmd(IRED_PORT_RCC | RCC_APB2Periph_AFIO, ENABLE);

    /* --- 第2步：配置 PA8 为上拉输入 ---
     * 红外接收头空闲时输出高电平，有38KHz红外信号时输出低电平
     * 上拉输入可以确保无信号时引脚保持稳定的高电平
     */
    GPIO_InitStructure.GPIO_Pin  = IRED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    // 上拉输入 (Input Pull-Up)
    GPIO_Init(IRED_PORT, &GPIO_InitStructure);

    /* --- 第3步：配置外部中断线路 ---
     * GPIO_EXTILineConfig(): 将 PA8 引脚映射到 EXTI_Line8 中断线
     * STM32 的 EXTI 线路：PAx/PBx/PCx... 中相同的 x 共享同一条 EXTI_Line
     * 所以要指定是哪个端口的 Pin8 被映射到 EXTI_Line8
     */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);

    // 清除可能残留的中断挂起标志，避免误触发
    EXTI_ClearITPendingBit(EXTI_Line8);

    /* --- 第4步：配置外部中断为下降沿触发 ---
     * 红外接收头在有38KHz载波时输出低电平，所以：
     *   下降沿（高→低）= 红外信号开始
     *   上升沿（低→高）= 红外信号结束
     * 我们捕获下降沿进入中断，然后在中断中测量高电平时间来判断数据
     */
    EXTI_InitStructure.EXTI_Line    = EXTI_Line8;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;   // 中断模式（非事件模式）
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  // 下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                // 使能该中断线
    EXTI_Init(&EXTI_InitStructure);

    /* --- 第5步：配置 NVIC 中断优先级 ---
     * PA8 对应的中断向量是 EXTI9_5_IRQn（EXTI 5~9 线共用）
     * 抢占优先级 = 0（最高优先级，可打断其他低抢占优先级的中断）
     * 响应优先级 = 1（同抢占优先级下，越小越优先）
     */
    NVIC_InitStructure.NVIC_IRQChannel    = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  // 抢占优先级 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;  // 响应优先级 1
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  测量高电平持续时间
  * @param  无
  * @retval 高电平的计数值 t（每个计数单位 ≈ 20μs）
  *
  * 工作原理：
  *   1. 在 while 循环中不断检测引脚是否为高电平
  *   2. 每等待 20μs，计数器 t 加 1
  *   3. 当引脚变为低电平，或 t 达到 250（约 5ms），停止计数并返回
  *
  * 时间换算参考：
  *   - t = 10  → 约 200μs
  *   - t = 28  → 约 560μs  （位"0"的高电平）
  *   - t = 84  → 约 1680μs （位"1"的高电平）
  *   - t = 225 → 约 4500μs （引导码的高电平 4.5ms）
  *   - t ≥ 250→ 超时退出  （约 ≥5ms）
  */
uint8_t IRremote_Counttime(void)
{
    uint8_t t = 0;    // 计数器，每个单位 ≈ 20μs

    // 当引脚为高电平时持续计数
    while (GPIO_ReadInputDataBit(IRED_PORT, IRED_PIN) == 1)
    {
        t++;
        Delay_us(20);         // 延时 20μs，作为时间基准

        if (t >= 250)
        {
            return t;         // 超时（约 5ms），强制返回，防止死循环
        }
    }

    return t;                 // 返回测量的高电平持续时间
}

/**
  * @brief  EXTI9_5 中断服务函数（处理红外遥控接收）
  * @param  无
  * @retval 无
  *
  * 这是整个红外解码的核心函数！当 PA8 检测到下降沿时，硬件自动进入此函数。
  *
  * 解码流程详解：
  *   1. 等待 PA8 变为高电平（下降沿之后的上升沿到来）
  *   2. 测量高电平持续时间 Tim
  *   3. 根据 Tim 的值判断信号类型：
  *      - Tim ≥ 250          → 超时/无信号，退出
  *      - 200 ≤ Tim < 250    → 引导码（4.5ms高电平）, Ok=1 开始接收数据
  *      - 60 ≤ Tim < 90      → 数据位"1"（1680μs高电平）
  *      - 10 ≤ Tim < 50      → 数据位"0"（560μs高电平）
  *   4. 收到引导码后，将后续 32 位数据逐位移入 IR_Receivecode
  *   5. 收满 32 位 → IR_Receiveflag = 1 → 退出
  *
  * 数据接收方式：位拼接
  *   IR_Receivecode <<= 1;    // 左移一位，腾出最低位
  *   IR_Receivecode += Data;  // 将新接收的 bit 放在最低位
  *
  * 最终 IR_Receivecode 中存放的是一帧完整的 32 位 NEC 数据：
  *   [31:24] 地址码    [23:16] 地址反码    [15:8] 数据码    [7:0] 数据反码
  */
void EXTI9_5_IRQHandler(void)
{
    uint8_t Tim = 0;    // 高电平宽度计数值
    uint8_t Ok  = 0;    // 解码状态标志：0=等待引导码, 1=已收到引导码, 正在接收数据
    uint8_t Data = 0;   // 当前接收到的数据位（0 或 1）
    uint8_t Num  = 0;   // 已接收的数据位数（从0开始计数）

    /* ── 【新增】更新红外活动时间戳 ──
     * 每次触发 EXTI 中断（检测到红外信号的下降沿），说明遥控器正在发送信号。
     * 记录当前时刻，用于主循环中的超时检测：
     *   - 按键持续按住时，遥控器会持续发送重复码，不断触发此中断 → 时间戳持续更新
     *   - 松开按键后，重复码停止 → 时间戳不再更新 → 主循环检测超时后自动刹车
     *
     * 注意：此处在中断刚开始时更新，即使是重复码（无法被解码为有效数据）
     * 也会更新时间戳，确保长按按键时小车不会误判超时。
     */
    g_last_ir_activity_ms = g_sys_tick_ms;

    while (1)
    {
        /*
         * 等待红外信号变为高电平
         * 下降沿触发了中断，此时引脚是低电平。
         * 我们需要等待它变回高电平后，用 IRremote_Counttime() 测量高电平的宽度
         */
        if (GPIO_ReadInputDataBit(IRED_PORT, IRED_PIN) == 1)
        {
            // 测量这一段时间高电平的持续时间
            Tim = IRremote_Counttime();

            // 情况A：超时 → 信号异常或结束，放弃本次解码
            if (Tim >= 250) break;

            // 情况B：高电平持续 200~249（约 4~5ms）→ 引导码！
            // 收到引导码说明一帧新数据即将开始
            if (Tim >= 200 && Tim < 250)
            {
                Ok = 1;         // 标记：已收到引导码，后续数据有效
            }
            // 情况C：高电平持续 60~89（约 1200~1780μs）→ 数据位 "1"
            else if (Tim >= 60 && Tim < 90)
            {
                Data = 1;       // 接收到 bit 1
            }
            // 情况D：高电平持续 10~49（约 200~980μs）→ 数据位 "0"
            else if (Tim >= 10 && Tim < 50)
            {
                Data = 0;       // 接收到 bit 0
            }

            /*
             * 如果已经收到引导码（Ok == 1），将 Data 移入接收码
             *
             * 位移操作示例（假设连续收到 1, 0, 1, 0）：
             *   初始: IR_Receivecode = 0 (二进制 0000 0000)
             *   第1位 Data=1: 左移→0000 0000, +1→0000 0001
             *   第2位 Data=0: 左移→0000 0010, +0→0000 0010
             *   第3位 Data=1: 左移→0000 0100, +1→0000 0101
             *   第4位 Data=0: 左移→0000 1010, +0→0000 1010
             *   ...以此类推
             */
            if (Ok == 1)
            {
                IR_Receivecode <<= 1;    // 左移一位
                IR_Receivecode += Data;  // 最低位填入新数据

                // 收满 32 位？→ 一帧完整数据接收完毕！
                if (Num >= 32)
                {
                    IR_Receiveflag = 1;   // 置位接收完成标志，通知主循环
                    break;                // 退出 while 循环
                }
            }

            Num++;    // 计数器 +1（每检测到一个高电平脉冲就加1）
        }
    }

    // 清除 EXTI_Line8 的中断挂起标志位
    // 这是必须的！否则退出中断后会立即再次进入
    EXTI_ClearITPendingBit(EXTI_Line8);
}
