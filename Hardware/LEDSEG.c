/**
  ******************************************************************************
  * @file    LEDSEG.c
  * @brief   数码管显示驱动模块
  *
  * 功能概述：
  *   通过 GPIOA 的 8 个引脚（PA0~PA7）直接驱动一位共阳极数码管。
  *   - LEDSEG_Init()   : 初始化 GPIOA 引脚为推挽输出模式
  *   - Digital_Display(): 在数码管上显示 0~9 的数字
  *
  * 硬件连接：
  *   PA0 → a段  (DPY_a)        ____a____
  *   PA1 → b段  (DPY_b)       |         |
  *   PA2 → c段  (DPY_c)       f         b
  *   PA3 → d段  (DPY_d)       |____g____|
  *   PA4 → e段  (DPY_e)       |         |
  *   PA5 → f段  (DPY_f)       e         c
  *   PA6 → g段  (DPY_g)       |____d____|  ·dp(小数点)
  *   PA7 → dp段 (DPY_DP)                    (本项目未使用小数点)
  *
  * 注意事项：
  *   - 本代码驱动的是"共阳极"数码管（低电平点亮，高电平熄灭）
  *   - 如果是共阴极数码管，需要将 LedShowData[] 的每个值取反
  ******************************************************************************
  */

#include "stm32f10x.h"    // STM32F10x 标准外设库头文件

/*
 * 共阳极数码管段码表（0~9）
 * 数码管为共阳极，即公共端接 VCC，段选引脚低电平时对应的段点亮。
 *
 * 每一位代表一个段（1=熄灭，0=点亮）：
 *   Bit7: dp(小数点)  Bit6: g   Bit5: f   Bit4: e
 *   Bit3: d           Bit2: c   Bit1: b   Bit0: a
 *
 * 例如：数字"0" → a,b,c,d,e,f点亮，g和dp熄灭 → 11000000 = 0xC0
 *       数字"1" → b,c点亮，其余熄灭 → 11111001 = 0xF9
 */
uint8_t LedShowData[] = {
    0xC0,  // 0 → 1100 0000  (a,b,c,d,e,f 点亮)
    0xF9,  // 1 → 1111 1001  (b,c 点亮)
    0xA4,  // 2 → 1010 0100  (a,b,d,e,g 点亮)
    0xB0,  // 3 → 1011 0000  (a,b,c,d,g 点亮)
    0x99,  // 4 → 1001 1001  (b,c,f,g 点亮)
    0x92,  // 5 → 1001 0010  (a,c,d,f,g 点亮)
    0x82,  // 6 → 1000 0010  (a,c,d,e,f,g 点亮)
    0xF8,  // 7 → 1111 1000  (a,b,c 点亮)
    0x80,  // 8 → 1000 0000  (全部8段点亮)
    0x90   // 9 → 1001 0000  (a,b,c,d,f,g 点亮)
};

/*
 * GPIO 引脚号数组，对应 PA0~PA7
 * 用于 Digital_Display() 函数中逐位输出段码
 */
uint16_t GPIO_PIN_x[] = {
    GPIO_Pin_0,   // PA0 → a段
    GPIO_Pin_1,   // PA1 → b段
    GPIO_Pin_2,   // PA2 → c段
    GPIO_Pin_3,   // PA3 → d段
    GPIO_Pin_4,   // PA4 → e段
    GPIO_Pin_5,   // PA5 → f段
    GPIO_Pin_6,   // PA6 → g段
    GPIO_Pin_7    // PA7 → dp(小数点)
};

/**
  * @brief  数码管初始化函数
  * @param  无
  * @retval 无
  *
  * 初始化步骤：
  *   1. 使能 GPIOA 端口的时钟（挂在 APB2 总线上）
  *   2. 配置 PA0~PA7 为推挽输出模式（50MHz）
  *   3. 将所有引脚初始状态设为高电平（共阳极数码管高电平=熄灭）
  */
void LEDSEG_Init(void)
{
    // --- 1. 开启 GPIOA 时钟 ---
    // GPIOA 挂在 APB2（高速外设总线）上，使用前必须先使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // --- 2. 配置 GPIOA 的8个引脚 ---
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;      // 推挽输出模式（能输出高低电平）
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
                                  | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      // 输出速率 50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);                  // 写入配置

    // --- 3. 初始化所有引脚输出高电平（数码管熄灭）---
    GPIO_SetBits(GPIOA,
        GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
        GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
}

/**
  * @brief  数码管显示数字
  * @param  val: 要显示的数字（0~9）
  * @retval 无
  *
  * 工作原理：
  *   1. 从段码表中取出 val 对应的8位段码
  *   2. 逐位输出到 PA0~PA7（位 0→PA0, 位1→PA1, ..., 位7→PA7）
  *   3. 段码中 0=点亮, 1=熄灭（共阳数码管）
  *
  * 注意：若 val 超出 0~9 范围，会显示乱码（数组越界）
  */
void Digital_Display(uint8_t val)
{
    uint8_t i;
    uint16_t display_val = LedShowData[val];  // 从段码表中取出对应数字的8位段码

    // 逐位输出段码到对应引脚
    for (i = 0; i < 8; i++)
    {
        /*
         * display_val >> i  & 0x01  : 取段码的第 i 位的值（0或1）
         * GPIO_WriteBit() 将这一位写入到 PA0+i 引脚
         * 位值为1 → 引脚高电平 → 该段熄灭（共阳极特性）
         * 位值为0 → 引脚低电平 → 该段点亮
         */
        GPIO_WriteBit(GPIOA, GPIO_PIN_x[i], (display_val >> i) & 0x01);
    }
}
