/**
  ******************************************************************************
  * @file    PWM.c
  * @brief   TIM4 四路 PWM 输出初始化模块
  *
  * 功能概述：
  *   配置 STM32 通用定时器 TIM4，在 PB6~PB9 四个引脚上输出 PWM 信号，
  *   用于驱动机器人小车的两个直流电机（通过 L298N 双H桥驱动模块）。
  *
  * PWM 基础知识：
  *   PWM（Pulse Width Modulation）= 脉冲宽度调制
  *   通过改变方波信号中高电平的占比（占空比）来调节输出的平均电压。
  *   占空比 = 高电平时间 / 周期 × 100%
  *   占空比 50% → 平均电压 = 电源电压 × 50%
  *
  * 定时器通道分配：
  *   TIM4_CH1 → PB6 → 左电机正转（控制左电机正转时的速度）
  *   TIM4_CH2 → PB7 → 左电机反转（控制左电机反转时的速度）
  *   TIM4_CH3 → PB8 → 右电机正转（控制右电机正转时的速度）
  *   TIM4_CH4 → PB9 → 右电机反转（控制右电机反转时的速度）
  *
  * PWM 参数计算：
  *   定时器时钟源: APB1 总线 = 72MHz（因 APB1 预分频器分频比为 ÷2）
  *
  *   PWM 频率 = TIM_CLK / (PSC + 1) / (ARR + 1)
  *            = 72,000,000 / 36 / 100
  *            = 20,000 Hz = 20 KHz
  *
  *   为什么选择 20KHz？
  *   - 远高于人耳可听范围（20Hz~20KHz），避免电机发出刺耳的啸叫声
  *   - 频率足够高，电机线圈的电感可以很好地滤除 PWM 纹波
  *
  *   占空比分辨率 = ARR + 1 = 100 级（0%~100%，每级1%）
  *
  * 输出极性：TIM_OCPolarity_High
  *   高电平 = 有效电平，即 TIM_Pulse/100 的值直接等于占空比百分比
  ******************************************************************************
  */

#include "stm32f10x.h"    // STM32F10x 标准外设库头文件

/**
  * @brief  TIM4 四路 PWM 初始化函数
  * @param  无
  * @retval 无
  *
  * 初始化步骤：
  *   1. 使能 TIM4 和 GPIOB 时钟
  *   2. 配置 PB6~PB9 为复用推挽输出（AF_PP）
  *   3. 配置 TIM4 时基参数（频率、计数方向等）
  *   4. 配置 TIM4 四路输出比较通道为 PWM 模式1
  *   5. 使能所有通道的预装载功能
  *   6. 启动 TIM4
  *
  * 注意：初始化后所有通道的 PWM 占空比均为 0（电机不转），
  *       需要通过 TIM_SetCompareX() 动态调节占空比。
  */
void PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    /* ======== 第1步：开启时钟 ======== */

    // TIM4 挂在 APB1 总线上，使能 TIM4 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // GPIOB 挂在 APB2 总线上，使能 GPIOB 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* ======== 第2步：配置 GPIOB PB6~PB9 为复用推挽输出 ======== */

    // PB6 → TIM4_CH1（左电机正转）
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;    // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // PB7 → TIM4_CH2（左电机反转）
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // PB8 → TIM4_CH3（右电机正转）
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // PB9 → TIM4_CH4（右电机反转）
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ======== 第3步：配置 TIM4 时基参数 ======== */

    /*
     * TIM_Period (ARR: 自动重装载值)
     *   设为 100-1=99，即计数器从 0 计数到 99 再溢出回到 0
     *   总共 100 个计数周期，对应 PWM 分辨率 100 级（1%~100%）
     *
     * TIM_Prescaler (PSC: 预分频器)
     *   设为 36-1=35，即 72MHz/(35+1) = 2MHz
     *   每个计数周期 = 1/2MHz = 0.5 μs
     *
     * PWM 频率 = 2MHz / 100 = 20KHz
     */
    TIM_TimeBaseStructure.TIM_Period        = 100 - 1;    // ARR = 99（100级分辨率）
    TIM_TimeBaseStructure.TIM_Prescaler     = 36 - 1;     // PSC = 35（分频后 2MHz）
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;          // 时钟分频因子（不进一步分频）
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;  // 向上计数模式
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /* ======== 第4步：配置四路 PWM 输出通道 ======== */

    /*
     * PWM 模式1：向上计数时
     *   当前计数值 < TIM_Pulse → 输出高电平
     *   当前计数值 ≥ TIM_Pulse → 输出低电平
     *
     * TIM_Pulse = 0 → 始终输出低电平（电机不转）
     * TIM_Pulse = 50 → 50%占空比（电机半速）
     * TIM_Pulse = 100 → 100%占空比（电机全速）
     */

    // --- TIM4_CH1 (PB6) 初始化为 PWM 模式1, 占空比 0% ---
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;            // PWM 模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;     // 使能输出
    TIM_OCInitStructure.TIM_Pulse       = 0;                          // 初始比较值 0（占空比 0%）
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;        // 高电平有效
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);

    // --- TIM4_CH2 (PB7) 初始化 ---
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);

    // --- TIM4_CH3 (PB8) 初始化 ---
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);

    // --- TIM4_CH4 (PB9) 初始化 ---
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);

    /* ======== 第5步：使能预装载功能 ======== */

    /*
     * 预装载（Preload）的作用：
     *   当在运行时修改比较值（TIM_Pulse）时，如果不使能预装载，
     *   新值会立即生效，可能导致当前 PWM 周期出现不完整波形（毛刺）。
     *
     *   使能预装载后，新值会在当前 PWM 周期结束后才更新到影子寄存器，
     *   保证每个 PWM 周期的波形完整。
     */
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  // CH1 预装载
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  // CH2 预装载
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  // CH3 预装载
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  // CH4 预装载
    TIM_ARRPreloadConfig(TIM4, ENABLE);                // ARR 自动重装载预装载

    /* ======== 第6步：启动 TIM4 ======== */
    TIM_Cmd(TIM4, ENABLE);    // 使能定时器，开始输出 PWM
}
