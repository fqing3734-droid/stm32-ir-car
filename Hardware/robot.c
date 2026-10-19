/**
  ******************************************************************************
  * @file    robot.c
  * @brief   双轮差速驱动机器人小车 —— 运动控制模块
  *
  * 功能概述：
  *   通过 TIM4 的四路 PWM 控制两个直流电机（左右各一个），实现小车的前进、
  *   后退、左转、右转、原地旋转和刹车停止等基本运动。
  *
  * 驱动方式：L298N 双H桥电机驱动模块（或兼容模块）
  *   每个电机由两路 PWM 控制：一路控制正转速度，另一路控制反转速度
  *   左电机：TIM4_CH1(PB6) 正转 + TIM4_CH2(PB7) 反转
  *   右电机：TIM4_CH3(PB8) 正转 + TIM4_CH4(PB9) 反转
  *
  *   ┌─────────────┬──────────┬──────────┬──────────┬──────────┐
  *   │   运动方式   │ 左正转   │ 左反转   │ 右正转   │ 右反转   │
  *   │             │ (PB6)    │ (PB7)    │ (PB8)    │ (PB9)    │
  *   ├─────────────┼──────────┼──────────┼──────────┼──────────┤
  *   │ 前进        │ speed    │ 0        │ speed    │ 0        │
  *   │ 后退        │ 0        │ speed    │ 0        │ speed    │
  *   │ 左转(前进)  │ 0        │ 0        │ speed    │ 0        │
  *   │ 右转(前进)  │ speed    │ 0        │ 0        │ 0        │
  *   │ 原地左转    │ 0        │ speed    │ speed    │ 0        │
  *   │ 原地右转    │ speed    │ 0        │ 0        │ speed    │
  *   │ 刹车/停止   │ 0        │ 0        │ 0        │ 0        │
  *   └─────────────┴──────────┴──────────┴──────────┴──────────┘
  *
  * PWM 参数说明：
  *   - 定时器：TIM4, 时钟 72MHz, 预分频36, 自动重装载100
  *   - PWM 频率 = 72MHz / 36 / 100 = 20KHz
  *   - 占空比 = TIM_Pulse / 100，取值范围 0~100 对应占空比 0%~100%
  ******************************************************************************
  */

#include "stm32f10x.h"    // STM32F10x 标准外设库头文件
#include "PWM.h"          // PWM 初始化（TIM4 四路 PWM 输出）
#include "Delay.h"        // 延时函数

/**
  * @brief  机器人初始化
  * @param  无
  * @retval 无
  *
  * 初始化 PWM 输出（TIM4四路PWM），为电机控制做好准备
  */
void robot_Init(void)
{
    PWM_Init();    // 初始化 TIM4 的四路 PWM 通道（PB6~PB9）
}

/**
  * @brief  设置四路 PWM 输出的速度值（底层函数）
  * @param  left1_speed  : 左电机正转 PWM 占空比（0~100）
  * @param  left2_speed  : 左电机反转 PWM 占空比（0~100）
  * @param  right1_speed : 右电机正转 PWM 占空比（0~100）
  * @param  right2_speed : 右电机反转 PWM 占空比（0~100）
  * @retval 无
  *
  * 注意：此函数直接设置四个通道的 PWM 比较值（占空比），
  *       不进行任何速度限制或逻辑判断。
  *       上层运动函数（如 makerobo_run）负责确保参数合理。
  */
void robot_speed(uint8_t left1_speed,  uint8_t left2_speed,
                 uint8_t right1_speed, uint8_t right2_speed)
{
    TIM_SetCompare1(TIM4, left1_speed);    // 设置 TIM4_CH1(PB6) 比较值 → 左电机正转
    TIM_SetCompare2(TIM4, left2_speed);    // 设置 TIM4_CH2(PB7) 比较值 → 左电机反转
    TIM_SetCompare3(TIM4, right1_speed);   // 设置 TIM4_CH3(PB8) 比较值 → 右电机正转
    TIM_SetCompare4(TIM4, right2_speed);   // 设置 TIM4_CH4(PB9) 比较值 → 右电机反转
}

/**
  * @brief  机器人前进
  * @param  speed: 前进速度，范围 0~100（对应PWM占空比 0%~100%）
  * @param  time : 持续时间（毫秒），设为 0 表示持续运行不延时
  * @retval 无
  *
  * 工作原理：左右两轮同时正转，产生向前运动。
  * 如果两个电机速度不同（如左轮70, 右轮50），小车会偏转行驶
  */
void makerobo_run(int8_t speed, uint16_t time)
{
    // --- 速度范围限制 ---
    if (speed > 100) { speed = 100; }   // 最大占空比 100%
    if (speed < 0)   { speed = 0;   }   // 最小占空比 0%（停止）

    // 左电机正转=speed, 右电机正转=speed → 两轮同速正转 → 前进
    robot_speed(speed, 0, speed, 0);

    Delay_ms(time);    // 运行指定时间（time=0 时立即返回）
}

/**
  * @brief  机器人刹车/停止
  * @param  time: 停止持续时间（毫秒），设为 0 表示不延时
  * @retval 无
  *
  * 四路 PWM 全部输出 0 → 所有电机停止转动 → 小车刹车
  */
void makerobo_brake(uint16_t time)
{
    robot_speed(0, 0, 0, 0);   // 四路 PWM 全部设为 0 → 刹车
    Delay_ms(time);             // 保持刹车状态的时间
}

/**
  * @brief  机器人前进左转（左轮不动，右轮正转）
  * @param  speed: 右轮速度，范围 0~100
  * @param  time : 持续时间（毫秒），设为 0 表示不延时
  * @retval 无
  *
  * 以右轮为动力、左轮为支点前进左转（转弯半径较小）
  */
void makerobo_Left(int8_t speed, uint16_t time)
{
    if (speed > 100) { speed = 100; }
    if (speed < 0)   { speed = 0;   }

    // 仅右轮正转，左轮不转 → 小车前进左转
    robot_speed(0, 0, speed, 0);

    Delay_ms(time);
}

/**
  * @brief  机器人原地左转（左轮反转，右轮正转）
  * @param  speed: 旋转速度，范围 0~100
  * @param  time : 持续时间（毫秒），设为 0 表示不延时
  * @retval 无
  *
  * 左轮反转 + 右轮正转 → 小车绕自身中心原地逆时针旋转
  */
void makerobo_Spin_Left(int8_t speed, uint16_t time)
{
    if (speed > 100) { speed = 100; }
    if (speed < 0)   { speed = 0;   }

    // 左轮反转=speed, 右轮正转=speed → 原地逆时针旋转
    robot_speed(0, speed, speed, 0);

    Delay_ms(time);
}

/**
  * @brief  机器人前进右转（左轮正转，右轮不动）
  * @param  speed: 左轮速度，范围 0~100
  * @param  time : 持续时间（毫秒），设为 0 表示不延时
  * @retval 无
  *
  * 以左轮为动力、右轮为支点前进右转（转弯半径较小）
  */
void makerobo_Right(int8_t speed, uint16_t time)
{
    if (speed > 100) { speed = 100; }
    if (speed < 0)   { speed = 0;   }

    // 仅左轮正转，右轮不转 → 小车前进右转
    robot_speed(speed, 0, 0, 0);

    Delay_ms(time);
}

/**
  * @brief  机器人原地右转（左轮正转，右轮反转）
  * @param  speed: 旋转速度，范围 0~100
  * @param  time : 持续时间（毫秒），设为 0 表示不延时
  * @retval 无
  *
  * 左轮正转 + 右轮反转 → 小车绕自身中心原地顺时针旋转
  */
void makerobo_Spin_Right(int8_t speed, uint16_t time)
{
    if (speed > 100) { speed = 100; }
    if (speed < 0)   { speed = 0;   }

    // 左轮正转=speed, 右轮反转=speed → 原地顺时针旋转
    robot_speed(speed, 0, 0, speed);

    Delay_ms(time);
}

/**
  * @brief  机器人后退
  * @param  speed: 后退速度，范围 0~100
  * @param  time : 持续时间（毫秒），设为 0 表示不延时
  * @retval 无
  *
  * 左右两轮同时反转 → 产生向后的运动
  */
void makerobo_back(int8_t speed, uint16_t time)
{
    if (speed > 100) { speed = 100; }
    if (speed < 0)   { speed = 0;   }

    // 左电机反转=speed, 右电机反转=speed → 两轮同速反转 → 后退
    robot_speed(0, speed, 0, speed);

    Delay_ms(time);
}
