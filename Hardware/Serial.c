/**
  ******************************************************************************
  * @file    Serial.c
  * @brief   USART1 串口通信模块（支持 printf 重定向）
  *
  * 功能概述：
  *   提供完整的 USART1 串口通信功能，包括：
  *   - 串口初始化（9600波特率，8数据位，1停止位，无校验）
  *   - 单字节发送/数组发送/字符串发送/数字发送/格式化字符串发送
  *   - 中断方式接收（每收到一个字节产生一次中断）
  *   - 重定向 C 标准库 printf() 到串口输出
  *
  * 硬件引脚：
  *   PA9  → USART1_TX（发送）
  *   PA10 → USART1_RX（接收）
  *
  * 使用方法：
  *   1. 调用 Serial_Init() 初始化串口
  *   2. 发送数据：printf() / Serial_SendString() / Serial_SendByte() 等
  *   3. 接收数据：在主循环中轮询 Serial_GetRxFlag()，收到后用 Serial_GetRxData() 读取
  *   4. 也可直接在中断中处理（修改 USART1_IRQHandler）
  *
  * 关于 printf 重定向：
  *   重写了 fputc() 函数，将 putchar 的标准输出重定向到 USART1。
  *   因此可以直接使用 printf() 像在电脑上一样输出调试信息到串口助手。
  ******************************************************************************
  */

#include "stm32f10x.h"    // STM32F10x 标准外设库头文件
#include <stdio.h>        // 标准输入输出库（printf 需要）
#include <stdarg.h>       // 可变参数宏（va_list, va_start, va_end 需要）

/* ==================== 全局变量 ==================== */

/*
 * 串口接收数据缓冲区
 * USART1_IRQHandler() 中断中接收到的单字节数据存放在此
 */
uint8_t Serial_RxData;

/*
 * 串口接收完成标志位
 * = 0 : 无新数据
 * = 1 : 接收到新数据，等待读取
 * 主程序调用 Serial_GetRxFlag() 时自动清零
 */
uint8_t Serial_RxFlag;

/* ==================== 串口初始化 ==================== */

/**
  * @brief  串口初始化函数
  * @param  无
  * @retval 无
  *
  * 配置内容：
  *   1. 使能 USART1 和 GPIOA 时钟
  *   2. 配置 PA9(TX) 为复用推挽输出，PA10(RX) 为上拉输入
  *   3. 配置 USART1：9600波特率, 8数据位, 1停止位, 无校验, 无硬件流控
  *   4. 使能 RXNE（接收非空）中断
  *   5. 配置 NVIC 中断优先级：抢占优先级1, 响应优先级1
  *   6. 使能 USART1
  *
  * 波特率计算（USART1挂在APB2上，时钟72MHz）：
  *   波特率 = 72MHz / (16 × USARTDIV)
  *   9600 ≈ 72MHz / (16 × 468.75)
  *   硬件自动计算，只需配置 USART_BaudRate = 9600
  */
void Serial_Init(void)
{
    // --- 1. 开启时钟 ---
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  // USART1 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,  ENABLE);  // GPIOA  时钟

    // --- 2. 配置 PA9(TX) 为复用推挽输出 ---
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;       // 复用推挽输出（USART外设控制引脚）
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;            // USART1_TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // --- 3. 配置 PA10(RX) 为上拉输入 ---
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;         // 上拉输入
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;           // USART1_RX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // --- 4. 配置 USART1 工作参数 ---
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate            = 9600;  // 波特率 9600bps
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控
    USART_InitStructure.USART_Mode               = USART_Mode_Tx | USART_Mode_Rx;  // 同时使能发送和接收
    USART_InitStructure.USART_Parity             = USART_Parity_No;     // 无校验位
    USART_InitStructure.USART_StopBits           = USART_StopBits_1;    // 1 位停止位
    USART_InitStructure.USART_WordLength         = USART_WordLength_8b;  // 8 位数据位
    USART_Init(USART1, &USART_InitStructure);

    // --- 5. 使能 RXNE 中断（接收缓冲区非空中断）---
    // 当 USART 收到一个字节时，硬件自动触发此中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    // --- 6. 配置 NVIC 中断优先级 ---
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  // 抢占优先级 1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority       = 1;  // 响应优先级 1
    NVIC_Init(&NVIC_InitStructure);

    // --- 7. 使能 USART1 ---
    USART_Cmd(USART1, ENABLE);
}

/* ==================== 串口发送函数 ==================== */

/**
  * @brief  发送单个字节
  * @param  Byte: 要发送的8位数据
  * @retval 无
  *
  * 发送流程：
  *   1. 调用 USART_SendData() 将数据写入发送数据寄存器 TDR
  *   2. while 循环等待 TXE（发送数据寄存器空）标志位置位
  *      TXE=1 表示数据已从 TDR 转移到移位寄存器，可以发送下一字节
  *      TXE=0 表示 TDR 中还有数据未转移，需等待
  */
void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART1, Byte);                           // 写入数据到发送寄存器
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);  // 等待发送完成
}

/**
  * @brief  发送字节数组
  * @param  Array : 指向要发送的数组的指针
  * @param  Length: 数组长度（要发送的字节数）
  * @retval 无
  *
  * 循环逐个调用 Serial_SendByte() 发送数组中的每个字节
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++)
    {
        Serial_SendByte(Array[i]);
    }
}

/**
  * @brief  发送字符串（以 '\0' 结尾的字符数组）
  * @param  String: 指向要发送的字符串指针
  * @retval 无
  *
  * 逐个字符发送，直到遇到字符串结束符 '\0'
  */
void Serial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        Serial_SendByte(String[i]);
    }
}

/**
  * @brief  计算 X 的 Y 次方（整数幂运算）
  * @param  X: 底数
  * @param  Y: 指数
  * @retval X 的 Y 次方结果
  *
  * 例如：Serial_Pow(10, 3) = 1000
  * 用途：在 Serial_SendNumber() 中用于数字按位提取
  */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

/**
  * @brief  发送数字（以十进制字符串形式）
  * @param  Number: 要发送的无符号32位整数
  * @param  Length: 数字的显示位数（不足时高位不补零，超过时截断高位）
  * @retval 无
  *
  * 工作原理示例（Number=123, Length=3）：
  *   i=0: Number / 10^(3-0-1) % 10 = 123/100%10 = 1 → 发送字符 '1'
  *   i=1: Number / 10^(3-1-1) % 10 = 123/10%10  = 2 → 发送字符 '2'
  *   i=2: Number / 10^(3-2-1) % 10 = 123/1%10   = 3 → 发送字符 '3'
  */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        /*
         * 公式：Number / 10^{Length-i-1} % 10
         * 即从最高位开始逐位提取每一位的数字
         * + '0' 将数字(0~9)转为对应的 ASCII 字符('0'~'9')
         */
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
  * @brief  重定向 C 库函数 fputc，使 printf 输出到串口
  * @param  ch: 要输出的字符
  * @param  f : 文件指针（未使用）
  * @retval 输出的字符
  *
  * 这是实现 printf 串口输出的关键！
  * printf() 内部最终调用 fputc() 来逐个输出字符。
  * 重写此函数，将字符通过串口发送，即可实现 printf 重定向。
  *
  * 使用条件（还需在工程选项中配置）：
  *   Keil: Target → Target → 勾选 "Use MicroLIB"
  *   或在编译器选项中添加: --keep=__use_no_semihosting
  *   本代码通过重写 fputc 来避免 semihosting 模式
  */
int fputc(int ch, FILE *f)
{
    Serial_SendByte(ch);    // 将字符通过串口发送
    return ch;              // 返回该字符（C标准要求）
}

/**
  * @brief  格式化字符串发送（类似 printf）
  * @param  format: 格式化字符串（如 "温度: %d ℃\r\n"）
  * @param  ...   : 可变参数列表（与格式化字符串中的 % 占位符一一对应）
  * @retval 无
  *
  * 实现原理：
  *   使用 C 标准库的 vsprintf() 将格式化后的字符串写入缓冲区 String，
  *   然后调用 Serial_SendString() 将缓冲区内容通过串口发送。
  *
  * 注意：缓冲区大小为 100 字节，格式化后的字符串不能超过此长度！
  *
  * 使用示例：
  *   Serial_Printf("ADC值: %d, 电压: %.2fV\r\n", adc_val, voltage);
  */
void Serial_Printf(char *format, ...)
{
    char String[100];              // 格式化字符串缓冲区（最大100字符）
    va_list arg;                   // 可变参数列表变量
    va_start(arg, format);         // 初始化 arg，使其指向 format 之后的第一个参数
    vsprintf(String, format, arg); // 格式化字符串写入 String
    va_end(arg);                   // 清理可变参数列表
    Serial_SendString(String);     // 将格式化后的字符串通过串口发送
}

/* ==================== 串口接收函数（与中断服务配合使用） ==================== */

/**
  * @brief  读取串口接收完成标志
  * @param  无
  * @retval 1=有新的接收数据, 0=无新数据
  *
  * 注意：调用此函数后，标志位会被自动清零！
  *       因此每次接收到的数据只能被读取一次。
  */
uint8_t Serial_GetRxFlag(void)
{
    if (Serial_RxFlag == 1)
    {
        Serial_RxFlag = 0;    // 清零标志位
        return 1;             // 返回"有新数据"
    }
    return 0;                 // 返回"无新数据"
}

/**
  * @brief  读取串口接收到的数据
  * @param  无
  * @retval 最近一次中断中接收到的字节数据
  *
  * 典型用法（在主循环中轮询）：
  *   if (Serial_GetRxFlag() == 1)
  *   {
  *       uint8_t data = Serial_GetRxData();
  *       // 处理收到的数据...
  *   }
  */
uint8_t Serial_GetRxData(void)
{
    return Serial_RxData;
}

/* ==================== USART1 中断服务函数 ==================== */

/**
  * @brief  USART1 中断服务函数
  * @param  无
  * @retval 无
  *
  * 触发条件：RXNE（接收数据寄存器非空）中断
  * 当 USART1 接收到一个完整字节的数据后，硬件自动进入此中断。
  *
  * 中断处理内容：
  *   1. 读取接收到的单字节数据 → 存入 Serial_RxData
  *   2. 置位 Serial_RxFlag = 1（通知主循环有新数据）
  *   3. 清除 RXNE 中断标志位
  *
  * 注意：此实现只缓存一个字节的数据。如果主程序读取速度慢于接收速度，
  *       旧数据会被新数据覆盖（没有环形缓冲区保护）。
  *       对于偶尔发送指令的场景来说足够使用。
  */
void USART1_IRQHandler(void)
{
    // 检查是否为 RXNE（接收非空）中断
    if (USART_GetFlagStatus(USART1, USART_IT_RXNE) == SET)
    {
        Serial_RxData = USART_ReceiveData(USART1);   // 读取接收到的数据字节
        Serial_RxFlag = 1;                           // 设置接收完成标志
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);  // 清除中断标志
    }
}
