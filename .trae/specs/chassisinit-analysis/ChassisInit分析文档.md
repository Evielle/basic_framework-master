# ChassisInit 详细分析文档

## 概述

**ChassisInit** 是底盘应用的初始化函数，负责初始化底盘控制所需的所有模块，包括：
- 四个轮毂电机（M3508）
- 裁判系统接口
- 缓冲能量PID控制器
- 超级电容
- IMU（底盘板模式）
- CAN通信（双板模式）或发布订阅机制（单板模式）

---

## 一、函数调用流程

```
ChassisInit()
├── 1. 初始化4个轮毂电机 (M3508 × 4)
│   └── PowerControlInit() × 4 → motor_lf, motor_rf, motor_lb, motor_rb
├── 2. 裁判系统初始化
│   └── UITaskInit() → referee_data
├── 3. 缓冲能量PID初始化
│   └── PIDInit() → buffer_PID
├── 4. 超级电容初始化
│   └── SuperCapInit() → cap
└── 5. 通信方式初始化 (二选一)
    ├── CHASSIS_BOARD模式: INS_Init() + CANCommInit()
    └── ONE_BOARD模式: SubRegister() + PubRegister()
```

---

## 二、关键变量与结构体详解

### 2.1 静态全局变量

| 变量名 | 类型 | 作用 | 默认值 |
|--------|------|------|--------|
| `motor_lf` | DJIMotorInstance* | 左前轮电机实例 | NULL (初始化后指向实例) |
| `motor_rf` | DJIMotorInstance* | 右前轮电机实例 | NULL |
| `motor_lb` | DJIMotorInstance* | 左后轮电机实例 | NULL |
| `motor_rb` | DJIMotorInstance* | 右后轮电机实例 | NULL |
| `buffer_PID` | PIDInstance | 缓冲能量PID控制器 | 由PIDInit初始化 |
| `referee_data` | referee_info_t* | 裁判系统数据指针 | 由UITaskInit返回 |
| `ui_data` | Referee_Interactive_info_t | UI交互数据 | 传给UITaskInit |
| `cap` | SuperCapInstance* | 超级电容实例 | NULL |
| `chassis_cmd_recv` | Chassis_Ctrl_Cmd_s | 接收到的底盘控制命令 | 未初始化 |
| `chassis_feedback_data` | Chassis_Upload_Data_s | 底盘反馈数据 | 未初始化 |

**条件编译变量（CHASSIS_BOARD模式）:**

| 变量名 | 类型 | 作用 |
|--------|------|------|
| `chasiss_can_comm` | CANCommInstance* | 双板通信CAN实例 |
| `Chassis_IMU_data` | attitude_t* | 底盘IMU姿态数据 |

**条件编译变量（ONE_BOARD模式）:**

| 变量名 | 类型 | 作用 |
|--------|------|------|
| `chassis_pub` | Publisher_t* | 底盘反馈发布者 |
| `chassis_sub` | Subscriber_t* | 底盘命令订阅者 |

### 2.2 电机初始化配置结构体

```c
Motor_Init_Config_s chassis_motor_config = {
    .can_init_config.can_handle = &hcan2,      // CAN总线句柄
    .controller_param_init_config = {
        .speed_PID = {
            .Kp = 4.5,                         // 比例系数
            .Ki = 0,                           // 积分系数
            .Kd = 0,                           // 微分系数
            .IntegralLimit = 3000,             // 积分限幅
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .MaxOut = 15000,                   // 最大输出
            .Output_LPF_RC = 0.3,              // 输出低通滤波时间常数
        },
    },
    .controller_setting_init_config = {
        .angle_feedback_source = MOTOR_FEED,   // 角度反馈来源:电机编码器
        .speed_feedback_source = MOTOR_FEED,   // 速度反馈来源:电机编码器
        .outer_loop_type = SPEED_LOOP,         // 外环类型:速度环(开环控制)
        .close_loop_type = SPEED_LOOP,         // 闭环类型:速度环
    },
    .motor_type = M3508,                       // 电机类型: M3508
};
```

**PID改进选项说明:**

| 标志位 | 功能 |
|--------|------|
| `PID_Trapezoid_Intergral` | 梯形积分，避免积分突变 |
| `PID_Integral_Limit` | 积分限幅，防止积分饱和 |
| `PID_Derivative_On_Measurement` | 微分作用于测量值而非误差 |

### 2.3 电机配置参数表

| 电机 | tx_id | 反转标志 | CAN总线 |
|------|-------|----------|---------|
| motor_lf (左前) | 1 | MOTOR_DIRECTION_NORMAL | CAN2 |
| motor_rf (右前) | 2 | MOTOR_DIRECTION_NORMAL | CAN2 |
| motor_lb (左后) | 4 | MOTOR_DIRECTION_NORMAL | CAN2 |
| motor_rb (右后) | 3 | MOTOR_DIRECTION_NORMAL | CAN2 |

### 2.4 PID配置结构体

```c
PID_Init_Config_s Buffer_pid_conf = {
    .Kp = 0.1,                                 // 比例系数较小
    .Ki = 0,                                   // 无积分
    .Kd = 0,                                   // 无微分
    .IntegralLimit = 1000,                     // 积分限幅
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .MaxOut = 1000,                            // 最大输出
};
```

### 2.5 超级电容配置结构体

```c
SuperCap_Init_Config_s cap_conf = {
    .can_config = {
        .can_handle = &hcan2,                  // 使用CAN2
        .tx_id = 0x302,                        // 电容接收ID
        .rx_id = 0x301,                        // 电容发送ID(注意:与用户视角相反)
    }};
```

### 2.6 CAN通信配置结构体（CHASSIS_BOARD模式）

```c
CANComm_Init_Config_s comm_conf = {
    .can_config = {
        .can_handle = &hcan2,
        .tx_id = 0x311,                        // 底盘板发送ID
        .rx_id = 0x312,                        // 底盘板接收ID
    },
    .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),     // 接收数据长度
    .send_data_len = sizeof(Chassis_Upload_Data_s),  // 发送数据长度
};
```

---

## 三、调用的函数详解

### 3.1 PowerControlInit()

**位置**: `modules/motor/power_control.h`

**功能**: 初始化带功率控制的电机实例

**参数**: `Motor_Init_Config_s *config` - 电机初始化配置

**返回值**: `DJIMotorInstance*` - 电机实例指针

**特点**:
- 使用功率控制模式，而非传统PID
- 电机设定值由功率控制设定，不走普通PID
- 返回的实例用于后续速度设定

### 3.2 UITaskInit()

**位置**: `modules/referee/referee_task.h`

**功能**: 初始化裁判系统通信和UI显示

**参数**: 
- `&huart6` - USART6句柄（裁判系统串口）
- `&ui_data` - UI交互数据结构

**返回值**: `referee_info_t*` - 裁判系统数据指针

**特点**:
- 同时初始化UI显示功能
- 返回指向裁判系统数据的指针，供后续读取

### 3.3 PIDInit()

**位置**: `modules/algorithm/controller.h`

**功能**: 初始化PID控制器实例

**参数**:
- `&buffer_PID` - PID实例指针
- `&Buffer_pid_conf` - PID初始化配置

**返回值**: 无

**特点**:
- 支持多种PID改进算法
- 配置后PID实例即可用于计算

### 3.4 SuperCapInit()

**位置**: `modules/super_cap/super_cap.h`

**功能**: 初始化超级电容通信

**参数**: `SuperCap_Init_Config_s *supercap_config` - 电容配置

**返回值**: `SuperCapInstance*` - 超级电容实例指针

**特点**:
- 通过CAN2与超级电容通信
- 接收电容电压、电流、功率数据

### 3.5 INS_Init()（CHASSIS_BOARD模式）

**功能**: 初始化惯性导航系统

**返回值**: `attitude_t*` - 姿态数据指针

**特点**:
- 仅在底盘板模式下调用
- 用于获取底盘IMU数据

### 3.6 CANCommInit()（CHASSIS_BOARD模式）

**功能**: 初始化双板CAN通信

**参数**: `CANComm_Init_Config_s *comm_conf` - CAN通信配置

**返回值**: `CANCommInstance*` - CAN通信实例指针

**特点**:
- 用于底盘板与云台板之间的通信
- 发送底盘反馈数据，接收云台控制命令

### 3.7 SubRegister() / PubRegister()（ONE_BOARD模式）

**功能**: 注册发布订阅话题

**参数**:
- `"chassis_cmd"` / `"chassis_feed"` - 话题名称
- `sizeof(Chassis_Ctrl_Cmd_s)` / `sizeof(Chassis_Upload_Data_s)` - 数据长度

**返回值**: 订阅者/发布者实例指针

**特点**:
- 伪pubsub机制，实现模块间通信隔离
- 单板模式下使用

---

## 四、设计要点与注意事项

### 4.1 坐标系约定

底盘采用**右手系**：
- X轴：底盘纵向正前方
- Y轴：底盘横向右侧
- Z轴：垂直向上（符合右手定则）

### 4.2 电机控制模式

底盘电机使用**功率控制模式**，而非传统PID控制：

```c
.outer_loop_type = SPEED_LOOP,  // 设置为开环
.close_loop_type = SPEED_LOOP,  // 速度闭环
```

**注意**：注释明确说明"电机设定值由功率控制设定，不走普通的PID"

### 4.3 双板 vs 单板模式

| 模式 | 通信方式 | 适用场景 |
|------|----------|----------|
| `CHASSIS_BOARD` | CAN通信 | 底盘板与云台板分离 |
| `ONE_BOARD` | 发布订阅 | 单板控制整车 |

### 4.4 未完成功能

代码中标记的TODO项：

1. **电机正反转设置**（第83行）
   ```c
   // @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号
   ```

2. **缓冲能量PID**（第103行）
   ```c
   /* Buffer环暂未测试，逻辑是计算期望buffer与实际buffer的差值...待完善 */
   ```

### 4.5 使用注意事项

1. **初始化顺序**：必须在FreeRTOS启动前调用，由`RobotInit()`调用
2. **中断状态**：初始化期间会关闭中断（在`RobotInit`中处理）
3. **CAN总线冲突**：电机和超级电容都使用CAN2，需确保ID不冲突
4. **USART6占用**：裁判系统使用USART6，不能再用于其他用途
5. **底盘参数配置**：需在`robot_def.h`中正确配置轴距、轮距等参数

### 4.6 宏定义依赖

底盘初始化依赖以下宏定义（来自`robot_def.h`）：

| 宏 | 说明 |
|----|------|
| `WHEEL_BASE` | 轴距 |
| `TRACK_WIDTH` | 轮距 |
| `RADIUS_WHEEL` | 轮子半径 |
| `CENTER_GIMBAL_OFFSET_X` | 云台中心X偏移 |
| `CENTER_GIMBAL_OFFSET_Y` | 云台中心Y偏移 |

---

## 五、初始化完成后的任务循环

ChassisInit完成后，`ChassisTask()`会被放入FreeRTOS任务循环，执行：

```
ChassisTask() (200-500Hz)
├── 获取控制命令 (CAN或PubSub)
├── 设置功率限制
├── 判断急停模式 → 停止/使能电机
├── 根据模式设定旋转速度
├── 坐标系变换 (云台→底盘)
├── 正运动学解算 (MecanumCalculate)
├── 输出限幅与电机设定 (LimitChassisOutput)
├── 逆运动学估计 (EstimateSpeed)
└── 发布反馈数据
```

---

## 六、架构关系图

```
┌─────────────────────────────────────────────────────────────┐
│                      ChassisInit()                          │
├─────────────────────────────────────────────────────────────┤
│  电机层                                                     │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐          │
│  │motor_lf │ │motor_rf │ │motor_lb │ │motor_rb │          │
│  └────┬────┘ └────┬────┘ └────┬────┘ └────┬────┘          │
│       └───────────┴────┬──────┴───────────┘                │
│                        ↓                                   │
│              PowerControlInit()                            │
├─────────────────────────────────────────────────────────────┤
│  能量层                                                     │
│  ┌─────────────┐    ┌──────────┐                           │
│  │  buffer_PID │    │   cap    │                           │
│  └──────┬──────┘    └────┬─────┘                           │
│         ↓                ↓                                 │
│    PIDInit()      SuperCapInit()                           │
├─────────────────────────────────────────────────────────────┤
│  通信层                                                     │
│  ┌───────────────────────────────────────────────┐         │
│  │  CHASSIS_BOARD: CANCommInit() + INS_Init()   │         │
│  │  ONE_BOARD:    SubRegister() + PubRegister() │         │
│  └───────────────────────────────────────────────┘         │
├─────────────────────────────────────────────────────────────┤
│  裁判系统                                                   │
│  ┌─────────────────┐                                       │
│  │   referee_data  │                                       │
│  └────────┬────────┘                                       │
│           ↓                                                │
│    UITaskInit(&huart6, &ui_data)                           │
└─────────────────────────────────────────────────────────────┘
```
