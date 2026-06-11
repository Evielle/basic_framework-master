# Basic Framework 框架分析文档

## 1. main() 函数执行流程

### 1.1 系统初始化顺序

```
main()
├── HAL_Init()                           // HAL库初始化
├── SystemClock_Config()                 // 系统时钟配置 (168MHz PLL)
├── MX_GPIO_Init()                       // GPIO初始化
├── MX_DMA_Init()                        // DMA初始化
├── MX_ADC1_Init()                       // ADC1初始化
├── MX_CAN1_Init()                       // CAN1初始化
├── MX_CAN2_Init()                       // CAN2初始化
├── MX_SPI1_Init()                       // SPI1初始化
├── MX_SPI2_Init()                       // SPI2初始化
├── MX_TIM1_Init()                       // 定时器1初始化
├── MX_TIM4_Init()                       // 定时器4初始化
├── MX_TIM5_Init()                       // 定时器5初始化
├── MX_TIM8_Init()                       // 定时器8初始化
├── MX_TIM10_Init()                      // 定时器10初始化
├── MX_USART1_UART_Init()                // USART1初始化 (遥控器)
├── MX_USART3_UART_Init()                // USART3初始化
├── MX_USART6_UART_Init()                // USART6初始化
├── MX_I2C2_Init()                       // I2C2初始化
├── MX_I2C3_Init()                       // I2C3初始化
├── MX_RNG_Init()                        // 随机数生成器初始化
├── MX_RTC_Init()                        // RTC初始化
├── MX_CRC_Init()                        // CRC初始化
├── MX_DAC_Init()                        // DAC初始化
├── RobotInit()                          // 机器人唯一初始化函数
├── MX_FREERTOS_Init()                   // FreeRTOS初始化
└── osKernelStart()                      // 启动RTOS调度器
```

### 1.2 系统时钟配置

- **时钟源**: HSE (外部高速晶振)
- **主频**: 168 MHz (PLL倍频)
- **AHB总线**: 168 MHz
- **APB1总线**: 42 MHz (4分频)
- **APB2总线**: 84 MHz (2分频)

---

## 2. 初始化函数详解

### 2.1 BSPInit()

**位置**: `bsp/bsp_init.h`

**功能**: 初始化基础BSP组件

```c
void BSPInit()
{
    DWT_Init(168);    // 调试计时器初始化,传入主频168MHz
    BSPLogInit();     // 日志系统初始化
}
```

**说明**: 其他外设如CAN、串口等采用注册制，不注册则不初始化。

### 2.2 RobotInit()

**位置**: `application/robot.c`

**功能**: 机器人统一初始化入口

```c
void RobotInit()
{
    __disable_irq();   // 关闭中断,防止初始化被打断

    BSPInit();         // 初始化BSP组件

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCMDInit();    // 机器人命令初始化 (可选)
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisInit();     // 底盘初始化 (可选)
#endif

    OSTaskInit();      // 创建FreeRTOS任务

    __enable_irq();    // 开启中断
}
```

---

## 3. FreeRTOS 任务结构

### 3.1 任务列表

| 任务名 | 句柄 | 优先级 | 栈大小 | 频率 | 功能 |
|--------|------|--------|--------|------|------|
| INS Task | insTaskHandle | osPriorityAboveNormal (最高) | 1024 | 1kHz | 惯性导航/姿态解算 |
| Motor Task | motorTaskHandle | osPriorityNormal | 256 | 1kHz | 电机控制 |
| Daemon Task | daemonTaskHandle | osPriorityNormal | 128 | 100Hz | 守护进程/蜂鸣器 |
| Robot Task | robotTaskHandle | osPriorityNormal | 1024 | 200-500Hz | 机器人核心控制 |
| UI Task | uiTaskHandle | osPriorityNormal | 512 | 可变 | 裁判系统UI |

### 3.2 各任务详解

#### INS Task (惯性导航任务)
- **频率**: 1kHz (最高优先级)
- **功能**:
  - 读取BMI088陀螺仪和加速度计数据
  - 执行姿态解算 (INS)
  - 发送视觉数据 (VisionSend)
- **异常检测**: 如果执行时间超过1ms则报错

#### Motor Task (电机控制任务)
- **频率**: 1kHz
- **功能**:
  - 执行电机控制循环
  - 处理功率限制
- **异常检测**: 如果执行时间超过1ms则报错

#### Daemon Task (守护进程任务)
- **频率**: 100Hz
- **功能**:
  - 执行Daemon任务(喂狗)
  - 蜂鸣器任务
- **异常检测**: 如果执行时间超过10ms则报错

#### Robot Task (机器人核心任务)
- **频率**: 200-500Hz
- **功能**:
  - RobotCMDTask: 处理机器人命令
  - ChassisTask: 底盘控制
  - GimbalTask: 云台控制 (可选)
  - ShootTask: 发射控制 (可选)
- **异常检测**: 如果执行时间超过5ms则报错

#### UI Task (裁判系统UI任务)
- **频率**: 可变
- **功能**:
  - 与裁判系统通信
  - 更新UI显示
- **特性**: 每发送一包数据会挂起一次

---

## 4. 外设初始化汇总

### 4.1 CAN总线
- **CAN1**: 电机通信
- **CAN2**: 电机通信

### 4.2 串口
- **USART1**: 遥控器通信 (SBUS协议)
- **USART3**: 扩展通信
- **USART6**: 视觉/USB通信

### 4.3 I2C
- **I2C2**: OLED显示屏、传感器
- **I2C3**: IMU (BMI088)

### 4.4 SPI
- **SPI1**: FLASH存储
- **SPI2**: 扩展外设

### 4.5 定时器
- **TIM1/TIM8**: PWM输出 (电机控制)
- **TIM4/TIM5**: 编码器输入
- **TIM10**: 通用定时器

### 4.6 其他
- **ADC1**: 电池电压检测
- **DAC**: 数模转换
- **RNG**: 随机数生成
- **RTC**: 实时时钟
- **CRC**: CRC校验

---

## 5. 框架架构图

```
┌─────────────────────────────────────────────────────────────┐
│                         main()                               │
│  ┌──────────────────────────────────────────────────────┐   │
│  │ HAL_Init() → SystemClock_Config() → MX_xxx_Init()   │   │
│  └──────────────────────────────────────────────────────┘   │
│                           ↓                                  │
│  ┌──────────────────────────────────────────────────────┐   │
│  │                    RobotInit()                        │   │
│  │  ├── BSPInit() (DWT, Log)                           │   │
│  │  ├── RobotCMDInit() (可选)                          │   │
│  │  ├── ChassisInit() (可选)                           │   │
│  │  └── OSTaskInit()                                   │   │
│  └──────────────────────────────────────────────────────┘   │
│                           ↓                                  │
│              osKernelStart() (启动RTOS)                      │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                     FreeRTOS Tasks                           │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │  INS Task   │  │ Motor Task  │  │ Daemon Task │         │
│  │   (1kHz)    │  │   (1kHz)    │  │   (100Hz)   │         │
│  └─────────────┘  └─────────────┘  └─────────────┘         │
│  ┌─────────────┐  ┌─────────────┐                           │
│  │ Robot Task  │  │   UI Task   │                           │
│  │ (200-500Hz) │  │   (可变)    │                           │
│  └─────────────┘  └─────────────┘                           │
└─────────────────────────────────────────────────────────────┘
```

---

## 6. 模块层次结构

### 6.1 应用层 (application/)
- `robot.c/h` - 机器人初始化和任务入口
- `chassis/` - 底盘控制
- `gimbal/` - 云台控制
- `shoot/` - 发射控制
- `cmd/` - 机器人命令处理

### 6.2 模块层 (modules/)
- `imu/` - 惯性测量单元 (BMI088, INS)
- `motor/` - 电机驱动 (DJI, DM, HT, LK)
- `referee/` - 裁判系统通信
- `remote/` - 遥控器控制
- `algorithm/` - 算法 (PID, 卡尔曼滤波, 四元数EKF)
- `oled/` - OLED显示
- `alarm/` - 蜂鸣器报警
- `daemon/` - 守护进程

### 6.3 BSP层 (bsp/)
- `can/` - CAN总线封装
- `usart/` - 串口封装
- `spi/` - SPI封装
- `iic/` - I2C封装
- `gpio/` - GPIO封装
- `adc/` - ADC封装
- `dwt/` - 调试计时器
- `log/` - 日志系统
- `usb/` - USB设备

---

## 7. 编译配置

框架支持多种编译配置 (定义在 `robot_def.h`):

- `ONE_BOARD` - 单板模式
- `CHASSIS_BOARD` - 底盘板模式
- `GIMBAL_BOARD` - 云台板模式

不同配置会编译不同的初始化代码和任务。

---

## 8. 关键特性

1. **注册制外设**: CAN、串口等外设采用注册制，不注册不初始化
2. **多板支持**: 支持底盘板和云台板分离部署
3. **实时系统**: 基于FreeRTOS的硬实时任务调度
4. **模块化设计**: BSP层、模块层、应用层分离
5. **消息中心**: 模块间通过发布-订阅模式通信
6. **功率控制**: 支持超级电容和功率限制
7. **裁判系统**: 支持RoboMaster裁判系统UI
