/**
  ******************************************************************************
  * @file    main.c
  * @brief   红外遥控机器人小车 —— 主程序
  *
  * 功能概述：
  *   1. 通过红外接收头接收遥控器发来的 NEC 编码信号
  *   2. 解码红外信号，提取数据码（键值）
  *   3. 根据不同的键值，控制小车执行：前进、后退、左转、右转、停止
  *   4. 用数码管显示当前动作对应的编号（0~4）
  *
  * 硬件平台：STM32F103 系列（ARM Cortex-M3 内核）
  * 对应引脚：
  *   - 红外接收：PA8  (外部中断 EXTI_Line8)
  *   - 数码管段选：PA0~PA7
  *   - 电机 PWM 控制：PB6~PB9 (TIM4 四路 PWM)
  *   - 串口调试：USART1 (PA9-TX, PA10-RX), 波特率 9600
  *   - 按键：PA15
  *`    
  * 【2025年修改说明】
  *   增加了"松开按键即停"功能：
  *   - SysTick 配置为 1ms 周期中断，维护全局毫秒计数器
  *   - 每次红外中断（EXTI）触发时更新最后活动时间戳
  *   - 主循环检测超时（100ms 无红外信号）→ 自动刹车
  *   - Delay_us() 已修改为保存/恢复 SysTick 配置，互不干扰
  ******************************************************************************
  */

/* ==================== 头文件包含 ==================== */
#include "stm32f10x.h"    // STM32F10x 标准外设库主头文件
#include "LEDSEG.h"       // 数码管显示模块
#include "Delay.h"        // 延时函数（SysTick 微秒/毫秒/秒级延时）
#include "robot.h"        // 机器人运动控制（前进/后退/转向/刹车）
#include "Incontrol.h"    // 红外遥控接收解码模块
#include "Key.h"          // 按键输入模块
#include "Serial.h"       // 串口通信模块（USART1）


/* ==================== 【新增】全局变量 & 宏定义 ==================== */

/**
  * @brief 系统毫秒计数器（由 SysTick_Handler 每1ms自增1）
  *
  * 声明为 volatile uint32_t：
  *   - volatile: 在中断中修改，编译器不优化对其的访问
  *   - uint32_t: 32位，每1ms加1，可计时约 49.7 天后溢出回绕
  *     由于本项目中只用差值判断超时（差值 < 2^31），
  *     无符号回绕不会影响计算结果，无需额外处理。
  *
  * 此变量在以下位置被使用：
  *   - stm32f10x_it.c 的 SysTick_Handler()  → 自增
  *   - Incontrol.c 的 EXTI9_5_IRQHandler() → 读取并写入 g_last_ir_activity_ms
  *   - main.c 主循环 → 读取并做超时判断
  */
volatile uint32_t g_sys_tick_ms = 0;

/**
  * @brief 最后一次检测到红外信号的时刻（毫秒时间戳）
  *
  * 在 EXTI9_5_IRQHandler() 入口处更新（每次检测到红外信号下降沿，
  * 无论是否为有效数据帧或重复码）。
  *
  * 主循环中通过比较 (g_sys_tick_ms - g_last_ir_activity_ms) 与超时阈值
  * 来判断红外信号是否已经中断（遥控器按键是否已松开）。
  */
volatile uint32_t g_last_ir_activity_ms = 0;

/**
  * @brief 红外信号超时阈值（单位：毫秒）
  *
  * 超过此时间未收到任何红外信号（包括有效数据帧和重复码），
  * 认为遥控器按键已松开，小车自动刹车。
  *
  * 推荐值 100ms：
  *   - NEC 协议重复码间隔约 110ms，100ms 阈值能可靠区分"按键松开"
  *     和"两次重复码之间的短暂间隔"
  *   - 松开按键后 100ms 内停车，响应灵敏
  *   - 如果发现长按时误刹车，可适当增大此值（如 150ms）
  */
#define IR_TIMEOUT_MS   ((uint32_t)100)


/**
  * @brief  主函数
  * @param  无
  * @retval int (实际不会返回，因为 while(1) 死循环)
  *
  * 程序执行流程：
  *   1. 配置中断优先级分组为第2组（2位抢占优先级，2位响应优先级）
  *   2. 依次初始化各个外设模块：红外接收→数码管→串口→机器人
  *   3. 【新增】配置 SysTick 为 1ms 周期中断
  *   4. 进入主循环，不断检测是否收到红外信号
  *   5. 【新增】检测超时，自动刹车
  *   6. 收到信号后解析数据码，根据键值执行对应动作
  */
int main(void)
{
    /* ---------- 局部变量定义 ---------- */
    uint8_t buf[2];          // 存放解析后的数据码：buf[0]=高四位, buf[1]=低四位
    uint8_t data_code = 0;   // 存放从红外接收码中提取的8位数据码

    /* ── 【新增】小车运动状态标志 ──
     * car_moving = 0: 小车已停止，不执行任何运动指令（防止超时刹车后被旧按键值重新触发）
     * car_moving = 1: 小车受红外遥控控制中，可以执行运动指令
     */
    uint8_t car_moving = 0;

    /* ---------- 系统初始化 ---------- */

    // 配置 NVIC 中断优先级分组为第2组
    // 第2组：2位抢占优先级 + 2位响应优先级（范围各为0~3）
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    IRremote_Init();      // 初始化红外遥控接收（PA8 外部中断 + NVIC）
    LEDSEG_Init();        // 初始化数码管显示（PA0~PA7 推挽输出）
    Serial_Init();        // 初始化串口 USART1（9600波特率，用于调试输出）
    robot_Init();         // 初始化机器人（即初始化 TIM4 四路PWM输出）

    /* ── 【新增】配置 SysTick 为 1ms 周期中断 ──
     *
     * SysTick_Config() 是 CMSIS 提供的标准函数，内部操作：
     *   1. LOAD  = SystemCoreClock/1000 - 1  (72000-1=71999，对应1ms)
     *   2. VAL   = 0                          清空当前计数
     *   3. CTRL  = 0x00000007                 ENABLE + TICKINT + CLKSOURCE(HCLK)
     *   4. 设置 SysTick 中断优先级为最低
     *
     * 返回值：0=成功, 1=失败（reload值超出24位范围）
     *
     * 注意：必须在所有外设初始化完成后再启动 SysTick 中断，
     * 避免初始化过程中的 Delay 调用与 SysTick 中断冲突。
     * Delay.c 中的 Delay_us() 已修改为自动保存/恢复 SysTick 配置，
     * 因此后续运行中两者可以共存。
     */
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        // 配置失败（理论上不会发生，72000 < 2^24），死循环便于调试
        while (1);
    }

    // 【新增】初始化时间戳为当前时刻，避免开机瞬间误判超时
    g_last_ir_activity_ms = g_sys_tick_ms;

    //Digital_Display(0); // 可选：上电时数码管显示 0

    /* ==================== 主循环 ==================== */
    while (1)
    {
        /*
         * --- 第一步：检测是否收到了红外遥控信号 ---
         * IR_Receiveflag 是红外接收模块的全局标志位：
         *   = 1 表示接收到一帧完整的 NEC 红外数据
         *   = 0 表示无新数据
         * 该标志由外部中断服务函数 EXTI9_5_IRQHandler 在接收完成后置1
         */
        if (IR_Receiveflag == 1)
        {
            IR_Receiveflag = 0;  // 清除接收标志，准备接收下一帧

            // 通过串口打印接收到的原始红外码（32位，16进制格式）
            // 用于调试：可通过串口助手查看遥控器各按键对应的键值
            printf("红外接收码： %0.8X\r\n", IR_Receivecode);

            // 提取数据码：32位NEC码中，数据码位于第9~16位
            // 右移8位后取低8位，得到遥控器按键对应的数据码
            data_code = IR_Receivecode >> 8;

            IR_Receivecode = 0;   // 清除接收码，准备接收下一帧

            /*
             * --- 第二步：将数据码拆分为两个半字节 ---
             * buf[0] = 高4位（除以16取商），用于区分按键分组
             * buf[1] = 低4位（除以16取余），用于区分组内具体按键
             *
             * 例如：数据码 0x62（十进制98）
             *   → buf[0] = 98/16 = 6
             *   → buf[1] = 98%16 = 2
             * 对应遥控器"前进"键
             */
            buf[0] = data_code / 16;   // 高4位
            buf[1] = data_code % 16;   // 低4位

            // 通过串口打印拆分后的键值，方便调试
            printf("buf[0]:%d\r\n", buf[0]);
            printf("buf[1]:%d\r\n", buf[1]);

            /* ── 【新增】收到有效数据帧，标记小车进入受控运动状态 ── */
            car_moving = 1;
        }

        /* ── 【新增】红外信号超时检测 ──
         *
         * 工作原理：
         *   1. EXTI9_5_IRQHandler 每次触发（检测到红外信号下降沿）
         *      都会更新 g_last_ir_activity_ms 为当前时刻。
         *   2. 遥控器按键持续按住时，遥控器持续发送重复码（NEC协议约每110ms一次），
         *      时间戳持续更新，不会触发超时。
         *   3. 松开按键后，不再有红外信号，EXTI 中断不再触发，
         *      时间戳不再更新，(g_sys_tick_ms - g_last_ir_activity_ms) 逐渐增大。
         *   4. 差值超过 IR_TIMEOUT_MS(100ms) → 判定按键已松开 → 自动刹车。
         *
         * 无符号减法的回绕安全性：
         *   即使 g_sys_tick_ms 发生 32 位溢出回绕（约 49.7 天后），
         *   无符号减法的结果在语义上仍然是正确的"时间差"，
         *   因为 g_sys_tick_ms 和 g_last_ir_activity_ms 总是同时溢出，
         *   差值 ≤ 实际时间差（模 2^32 运算的性质）。
         */
        if (car_moving &&
            (g_sys_tick_ms - g_last_ir_activity_ms) > IR_TIMEOUT_MS)
        {
            /*
             * 超时处理：执行 L298N 刹车模式停止
             *
             * makerobo_brake(0) 内部调用 robot_speed(0,0,0,0)，
             * 将 TIM4 四路 PWM 占空比全部设为 0%。
             * 对于 L298N：
             *   - IN1~IN4 全部输出低电平
             *   - H 桥所有 MOS 管关闭
             *   - 电机两端被短接到 GND（快速刹车/能耗制动）
             *   - 小车立即停稳，惯性滑行距离极短
             */
            makerobo_brake(0);

            Digital_Display(4);     // 数码管显示"4"（停止/刹车）

            car_moving = 0;         // 清除运动标志，防止旧按键值重新触发运动
        }

        /*
         * --- 第三步：根据键值执行对应的机器人动作 ---
         * 以下各遥控器按键对应的 (buf[0], buf[1]) 键值对：
         *
         *   (6, 2)  → 前进   → Digital_Display(0)
         *   (10, 8) → 后退   → Digital_Display(1)
         *   (2, 2)  → 左转   → Digital_Display(2)
         *   (12, 2) → 右转   → Digital_Display(3)
         *   (0, 2)  → 停止   → Digital_Display(4)
         *
         * 注意：不同遥控器的键值映射可能不同！
         * 可以通过串口打印的实际键值来确认每个按钮对应的编码。
         *
         * 【修改】仅当 car_moving == 1 时才执行运动指令。
         * 超时刹车后 car_moving 被清零，此 if 块整体跳过，
         * 防止 buf[] 中残留的旧按键值在下一次循环中重新驱动电机。
         */
        if (car_moving)
        {
            if (buf[0] == 6 && buf[1] == 2)
            {
                // 遥控器"前进"键 → 小车以70%的速度前进，持续0ms（持续运行）
                // 第二个参数 time=0 表示不延时，持续执行该动作直到收到新指令
                makerobo_run(70, 0);    // 机器人前进，速度70%
                Digital_Display(0);     // 数码管显示"0"
            }
            else if (buf[0] == 10 && buf[1] == 8)
            {
                makerobo_back(70, 0);   // 机器人后退，速度70%
                Digital_Display(1);     // 数码管显示"1"
            }
            else if (buf[0] == 2 && buf[1] == 2)
            {
                makerobo_Spin_Left(70, 0);  // 机器人原地左转
                Digital_Display(2);         // 数码管显示"2"
            }
            else if (buf[0] == 12 && buf[1] == 2)
            {
                makerobo_Spin_Right(70, 0); // 机器人原地右转
                Digital_Display(3);         // 数码管显示"3"
            }
            else if (buf[0] == 0 && buf[1] == 2)
            {
                makerobo_brake(0);      // 机器人刹车停止
                Digital_Display(4);     // 数码管显示"4"
                car_moving = 0;         // 【新增】手动按停止键，清除运动标志
            }
            else
            {
                // 未匹配到任何已知按键 → 默认执行刹车，确保安全
                makerobo_brake(0);      // 停止
                Digital_Display(4);     // 数码管显示"4"
                car_moving = 0;         // 【新增】未知按键，清除运动标志
            }
        }
    }
}
