/**
  ******************************************************************************
  * @file    Delay.c
  * @brief   基于 SysTick 系统定时器的精确延时函数库
  *
  * 功能概述：
  *   利用 Cortex-M3 内核自带的 SysTick（系统滴答定时器，24位递减计数器），
  *   提供微秒级(us)、毫秒级(ms)和秒级(s)的延时功能。
  *
  * SysTick 定时器简介：
  *   - SysTick 是 ARM Cortex-M 内核内置的一个24位递减定时器
  *   - 时钟源可以选择 HCLK（72MHz）或 HCLK/8（9MHz）
  *   - 本代码使用 HCLK = 72MHz 作为时钟源，即每计数一次 = 1/72 微秒
  *   - 计数范围：0 ~ 16,777,215（2^24 - 1）
  *
  * 使用方法：
  *   Delay_us(100);   // 延时 100 微秒
  *   Delay_ms(500);   // 延时 500 毫秒（0.5秒）
  *   Delay_s(3);      // 延时 3 秒
  *
  * 注意事项：
  *   1. SysTick 被配置为只能单任务延时（非中断方式），延时期间CPU被完全占用
  *   2. 延时期间若发生中断，实际延时可能略长
  *   3. 本实现会影响 SysTick->CTRL 和 SysTick->LOAD/VAL 寄存器，
  *      如果项目中同时使用 SysTick 做 RTOS 时基，请勿使用此延时方案
  ******************************************************************************
  */

#include "stm32f10x.h"    // STM32F10x 标准外设库头文件

/**
  * @brief  微秒级延时（阻塞式）
  * @param  xus: 要延时的微秒数，范围 0 ~ 233015
  *         （最大值 = 2^24 / 72 ≈ 233015 微秒 ≈ 233 毫秒）
  * @retval 无
  *
  * 工作原理：
  *   1. SysTick 时钟源 = HCLK = 72MHz，即每个时钟周期 = 1/72 μs
  *   2. 要延时 xus 微秒，需要计数 72 * xus 个时钟周期
  *   3. 将 LOAD 寄存器设为 72*xus，启动定时器开始递减
  *   4. 当计数器减到 0 时，CTRL 寄存器的 bit16(COUNTFLAG) 自动置1
  *   5. 检测到 COUNTFLAG=1 后，关闭定时器，延时结束
  *
  * SysTick 控制寄存器 (CTRL) 位定义：
  *   bit0 (ENABLE)   : 1=使能定时器
  *   bit1 (TICKINT)  : 1=使能中断（本代码不使用中断）
  *   bit2 (CLKSOURCE): 1=HCLK, 0=HCLK/8（这里设为1，用72MHz）
  *   bit16(COUNTFLAG): 计数到0时硬件置1，读取后自动清零
  *
  *   0x00000005 = bit0 + bit2 (使能 + 选择HCLK时钟源，不产生中断)
  *   0x00000004 = bit2 only（关闭使能，保留时钟源配置）
  */
void Delay_us(uint32_t xus)
{
    /* ── 【新增】保存当前 SysTick 配置 ──
     * 如果 SysTick 正在以中断模式运行（作为系统滴答时钟），
     * 需要先保存其配置，等本次阻塞延时完成后再恢复。
     * 否则会破坏 SysTick 的周期中断功能。
     */
    uint32_t saved_ctrl = SysTick->CTRL;
    uint32_t saved_load = SysTick->LOAD;

    SysTick->LOAD = 72 * xus;               // 设置重装载值（72 × 微秒数）
    SysTick->VAL  = 0x00;                   // 清空当前计数值
    SysTick->CTRL = 0x00000005;             // HCLK时钟 + 无中断 + 启动定时器
    while (!(SysTick->CTRL & 0x00010000));  // 等待 COUNTFLAG(bit16) 置1 → 计数到0
    SysTick->CTRL = 0x00000004;             // 关闭定时器

    /* ── 【新增】恢复 SysTick 到之前的运行状态 ──
     * 如果之前 SysTick 是使能的（如周期中断模式），
     * 则恢复其 LOAD 值并重新使能，使系统滴答时钟继续运行。
     * VAL 清零意味着当前周期重新开始计时（损失不到1ms，可忽略）。
     */
    if (saved_ctrl & 0x01)                  // 之前 ENABLE 位是否为1？
    {
        SysTick->LOAD = saved_load;         // 恢复重装载值
        SysTick->VAL  = 0x00;              // 清空当前值，重新开始计数
        SysTick->CTRL = saved_ctrl;        // 恢复控制寄存器（含使能位）
    }
}

/**
  * @brief  毫秒级延时（阻塞式）
  * @param  xms: 要延时的毫秒数，范围 0 ~ 4294967295
  * @retval 无
  *
  * 实现方式：循环调用 Delay_us(1000)，即每循环一次延时 1000 μs = 1 ms
  * while(xms--) 代表重复 xms 次，每次减1直到为0
  */
void Delay_ms(uint32_t xms)
{
    while (xms--)
    {
        Delay_us(1000);    // 每次延时 1000 微秒 = 1 毫秒
    }
}

/**
  * @brief  秒级延时（阻塞式）
  * @param  xs: 要延时的秒数，范围 0 ~ 4294967295
  * @retval 无
  *
  * 实现方式：循环调用 Delay_ms(1000)，即每循环一次延时 1000 ms = 1 s
  */
void Delay_s(uint32_t xs)
{
    while (xs--)
    {
        Delay_ms(1000);    // 每次延时 1000 毫秒 = 1 秒
    }
}
