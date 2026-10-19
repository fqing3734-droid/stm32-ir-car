# STM32 红外遥控机器人小车

基于 **STM32F103C8T6** + **标准外设库（StdPeriph Library）** 的双轮差速小车：红外遥控器（NEC 协议）发指令，单片机解码后驱动 L298N 控制两个直流电机完成前进 / 后退 / 左转 / 右转 / 刹车，同时用数码管显示当前动作编号。

> 从零手写：红外 NEC 解码、TIM4 四路 PWM、差速运动模型、SysTick 毫秒计时与「松手即停」超时刹车，全部自己实现，没有用 HAL 和第三方库。

## 功能特性

- **红外遥控**：PA8 外部中断（EXTI_Line8）捕获 NEC 编码，软件解码引导码 / 地址码 / 数据码 / 反码校验
- **五种动作**：前进、后退、左转（单轮）、右转（单轮）、停止；数码管同步显示动作编号 0~4
- **PWM 调速**：TIM4 四路 PWM 驱动 L298N，频率 20 kHz（72 MHz / 36 / 100），占空比 0~100 可调
- **松手即停**：「按下持续动、松手 100 ms 内自动刹车」，靠 SysTick 1 ms 节拍 + 最后活动时间戳实现
- **速度渐变**：起步 / 停车带斜坡加速，避免电机瞬间启停打滑（见 `robot.c`）
- **串口调试**：USART1（PA9/PA10，9600 8N1）打印键值与动作
- **红外避障预留**：PA11 / PA12 上拉输入，`Irobstacle.c` 提供左右避障电平读取

## 硬件与接线

| 功能 | 引脚 | 说明 |
| --- | --- | --- |
| 红外接收头 | **PA8** | EXTI_Line8，下降沿触发 |
| 数码管段选 | **PA0 ~ PA7** | 共 8 段，显示动作编号 |
| 左电机正转 / 反转 | **PB6 / PB7** | TIM4_CH1 / TIM4_CH2 → L298N |
| 右电机正转 / 反转 | **PB8 / PB9** | TIM4_CH3 / TIM4_CH4 → L298N |
| 红外避障 左 / 右 | **PA11 / PA12** | 上拉输入，低电平有障碍 |
| 独立按键 | **PA15** | 需关闭 JTAG、保留 SWD |
| 串口调试 | **PA9 / PA10** | USART1，9600 8N1 |

**器件清单**：STM32F103C8T6 最小系统板、L298N 电机驱动模块、红外接收头（HS0038 类）、NEC 红外遥控器、两位 / 一位数码管、TT 电机 ×2 + 轮子、7.4 V 电池组（电机与 MCU 分开供电，共地）、ST-Link。

### 运动状态表（`robot.c`）

| 运动 | 左正转 PB6 | 左反转 PB7 | 右正转 PB8 | 右反转 PB9 |
| --- | --- | --- | --- | --- |
| 前进 | speed | 0 | speed | 0 |
| 后退 | 0 | speed | 0 | speed |
| 左转（前进） | 0 | 0 | speed | 0 |
| 右转（前进） | speed | 0 | 0 | 0 |
| 原地左转 | 0 | speed | speed | 0 |
| 原地右转 | speed | 0 | 0 | speed |
| 刹车 / 停止 | 0 | 0 | 0 | 0 |

## 软件结构

```
stm32-ir-car/
├── User/                 # main.c、中断向量实现 stm32f10x_it.c、库配置 stm32f10x_conf.h
├── Hardware/             # 自写外设驱动
│   ├── Incontrol.c/.h    #   红外 NEC 解码（EXTI 中断状态机）
│   ├── PWM.c/.h          #   TIM4 四路 PWM 初始化
│   ├── robot.c/.h        #   差速运动控制（前进/后退/转向/刹车/斜坡）
│   ├── Key.c/.h          #   按键扫描
│   ├── LEDSEG.c/.h       #   数码管显示（段选 PA0~PA7）
│   ├── OLED.c/.h         #   I2C OLED（可选显示，含字库 OLED_Font.h）
│   ├── Irobstacle.c/.h   #   红外避障输入
│   └── Serial.c/.h       #   USART1 串口收发 + printf 重定向
├── System/Delay.c/.h     # SysTick 微秒/毫秒延时（会保存并恢复 SysTick 配置）
├── Start/                # 启动文件、core_cm3、system_stm32f10x、stm32f10x.h
├── Library/              # ST 标准外设库
└── Project.uvprojx       # Keil MDK 工程（Device = STM32F103C8）
```

### 关键实现

**1. NEC 解码（`Incontrol.c`）**
17 个状态由 EXTI 下降沿推进：9 ms 引导码高电平 + 4.5 ms 低电平判定起始，随后按 560 µs 高电平 + 0.56 ms / 1.69 ms 低电平区分 bit0 / bit1，32 位收齐后校验（数据码与其反码异或 0xFF），再与 24 位地址组合输出键值。

**2. 松手即停（`main.c`）**
SysTick 配 1 ms 中断维护 `volatile uint32_t` 毫秒计数器；每次红外中断刷新最后活动时间；主循环在 100 ms 内没收到新键值就调 `makerobo_brake()`，避免遥控信号丢失后小车一直往前冲。`Delay_us()` 里对 SysTick 做保存 / 恢复，保证 1 ms 节拍不被延时函数破坏。

**3. 差速转向**
左转不是「轮子反向」，而是单侧驱动（左右转速差），按上表组合四路 PWM 占空比，速度参数 `speed` 取 0~100。

## 编译与烧录

### Keil MDK（推荐）

1. Keil 5 打开 `Project.uvprojx`，器件已配 `STM32F103C8`，编译输出到 `Objects/`（`keilkill.bat` 可一键清理中间文件）。
2. 接 ST-Link（SWD：SWDIO/SWCLK/GND/3V3），`Flash → Download`。
3. 注意事项：PA15 做普通 IO 时必须先 `RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE)` + `GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE)` 关掉 JTAG（保留 SWD），否则 PA15 不能当输出用。

### VS Code + EIDE（可选）

工程自带 `.eide/` 与 `.vscode/tasks.json`，装 EIDE 插件后可直接 Build / Flash；芯片同样选 STM32F103C8。

## 已知问题与后续计划

- [ ] 遥控器键值映射目前写死在 `main.c` 的 switch 里，可改成表驱动方便换遥控器
- [ ] 红外避障只做了电平读取，还没接进主循环做自动绕障
- [ ] 增加编码器测速 + 闭环 PID，让直线行驶不跑偏
- [ ] 数码管目前只显示动作编号，可扩展成速度档位显示
