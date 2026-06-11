# PowerControl vs DJIMotor vs LKMotor 模块对比分析

## 概述

框架中有三套电机控制模块：
1. **DJIMotor** (`modules/motor/DJImotor/`) — 标准DJI电机PID控制
2. **PowerControl** (`modules/motor/power_control.c`) — 带功率限制的DJI电机控制（底盘专用）
3. **LKMotor** (`modules/motor/LKmotor/`) — LK系列电机控制

本文档详细对比三者的关联与区别，说明使用场景和注意事项。

---

## 一、三套模块总览

| 模块 | 适用电机 | 控制方式 | 典型用途 |
|------|----------|----------|----------|
| DJIMotor | M3508, M2006, GM6020 | 标准三环PID | 云台、拨盘、通用 |
| PowerControl | M3508, M2008 | 功率模型+速度环 | 底盘轮毂电机 |
| LKMotor | LK9015, LK9025 | 标准三环PID | 轮向电机、通用 |

---

## 二、函数对比表

### 2.1 初始化函数

| 模块 | 函数 | 返回类型 | 说明 |
|------|------|----------|------|
| DJIMotor | `DJIMotorInit(Motor_Init_Config_s *config)` | `DJIMotorInstance*` | 标准初始化 |
| PowerControl | `PowerControlInit(Motor_Init_Config_s *config)` | `DJIMotorInstance*` | 功率控制初始化，返回相同类型 |
| LKMotor | `LKMotorInit(Motor_Init_Config_s *config)` | `LKMotorInstance*` | LK电机初始化 |

**关联点**：
- 三者参数类型相同：`Motor_Init_Config_s`
- PowerControlInit内部实现与DJIMotorInit几乎相同
- 都会注册CAN回调、Daemon看门狗、电机分组

**区别**：
- PowerControlInit将电机实例存入`power_control.c`内部的`dji_motor_instance[]`数组
- DJIMotorInit将电机实例存入`dji_motor.c`内部的`dji_motor_instance[]`数组
- **两者是独立的数组**，互不影响

### 2.2 控制函数

| 模块 | 函数 | 调用位置 | 说明 |
|------|------|----------|------|
| DJIMotor | `DJIMotorControl()` | `MotorControlTask()` | 标准PID计算+CAN发送 |
| PowerControl | `PowerControl()` | `MotorControlTask()` | 功率限制+速度环+CAN发送 |
| LKMotor | `LKMotorControl()` | `MotorControlTask()` | LK电机PID+CAN发送 |

**关键区别**：
- `DJIMotorControl()`：完整的**三环PID**（位置环→速度环→电流环）
- `PowerControl()`：只有**速度环**，然后根据**功率模型**计算输出，**无电流环**
- `LKMotorControl()`：完整的**三环PID**

### 2.3 设置参考值

| 模块 | 函数 | 说明 |
|------|------|------|
| DJIMotor | `DJIMotorSetRef(motor, ref)` | 设置`motor_controller.pid_ref` |
| PowerControl | `DJIMotorSetRef(motor, ref)` | **复用DJI的函数**，设置`pid_ref` |
| LKMotor | `LKMotorSetRef(motor, ref)` | 设置`pid_ref` |

**关联点**：PowerControl模块没有单独的SetRef函数，直接使用`DJIMotorSetRef()`

### 2.4 启停函数

| 模块 | 函数 | 说明 |
|------|------|------|
| DJIMotor | `DJIMotorStop/Enable(motor)` | 设置`stop_flag` |
| PowerControl | `DJIMotorStop/Enable(motor)` | **复用DJI的函数** |
| LKMotor | `LKMotorStop/Enable(motor)` | 设置`stop_flag` |

### 2.5 特有函数

| 模块 | 函数 | 说明 |
|------|------|------|
| DJIMotor | `DJIMotorChangeFeed()` | 切换反馈源（IMU/编码器） |
| DJIMotor | `DJIMotorOuterLoop()` | 修改外环类型 |
| PowerControl | `SetPowerLimit(power)` | **特有**：设置功率限制 |
| LKMotor | `LKMotorIsOnline(motor)` | 检查在线状态 |

---

## 三、内部实现对比

### 3.1 DJIMotorControl() 的控制流程

```
DJIMotorControl()
├── 遍历所有注册的DJI电机
├── 位置环（如果启用且外环是ANGLE_LOOP）
│   └── PIDCalculate(angle_PID, total_angle, ref) → 输出速度参考
├── 速度环（如果启用且外环是SPEED_LOOP或ANGLE_LOOP）
│   └── PIDCalculate(speed_PID, speed_aps, ref) → 输出电流参考
├── 电流环（如果启用）
│   └── PIDCalculate(current_PID, real_current, ref) → 输出set
├── 填充sender_assignment[group].tx_buff
└── CANTransmit()
```

**特点**：完整的串级PID，支持三环任意组合

### 3.2 PowerControl() 的控制流程

```
PowerControl()
├── 遍历所有通过PowerControlInit注册的电机
├── 速度环（如果启用）
│   ├── pid_measure = speed_aps / 6.0f  (转换为rpm)
│   ├── pid_ref = PIDCalculate(speed_PID, pid_measure, ref)
│   ├── initial_torque[i] = pid_ref
│   └── 计算期望功率: initial_give_power[i] = K1*torque² + K2*rpm² + C*rpm*torque + constant
├── 累加所有电机期望功率: initial_total_power
├── 功率限制判断
│   └── if (initial_total_power > chassis_max_power)
│       ├── 计算缩放比例: ratio = chassis_max_power / initial_total_power
│       └── 根据功率模型反解新的输出值（一元二次方程求解）
├── 填充sender_assignment[group].tx_buff
└── CANTransmit()
```

**特点**：
1. **只有速度环**，没有电流环和位置环
2. 使用**电机功率模型**：`P = K1*T² + K2*n² + C*n*T + constant`
3. 当总功率超限时，根据功率模型**反解**新的输出值
4. **不使用PID的电流环**，直接输出电流值

### 3.3 LKMotorControl() 的控制流程

```
LKMotorControl()
├── 遍历所有注册的LK电机
├── 位置环（如果启用）
├── 速度环（如果启用）
├── 电流环（如果启用）
├── 填充sender_instance->tx_buff（多电机发送，ID=0x280）
└── CANTransmit()
```

**特点**：与DJIMotor类似的三环PID，但CAN协议不同

---

## 四、关键变量对比

### 4.1 电机实例存储

| 模块 | 存储数组 | 定义位置 | 最大数量 |
|------|----------|----------|----------|
| DJIMotor | `dji_motor_instance[DJI_MOTOR_CNT]` | `dji_motor.c` | 12 |
| PowerControl | `dji_motor_instance[DJI_MOTOR_CNT]` | `power_control.c` | 12 |
| LKMotor | `lkmotor_instance[LK_MOTOR_MX_CNT]` | `LK9025.c` | 4 |

**注意**：DJI和PowerControl的数组同名但位于不同文件，是**独立的静态变量**！

### 4.2 CAN发送分组

| 模块 | 发送实例 | 说明 |
|------|----------|------|
| DJIMotor | `sender_assignment[6]` | 6组：CAN1×3 + CAN2×3 |
| PowerControl | `sender_assignment[6]` | **与DJI相同**，共享 |
| LKMotor | `sender_instance` | 单一实例，ID=0x280 |

**关联点**：DJI和PowerControl共享同一套`sender_assignment`，因此：
- 如果同时使用两者，CAN发送会冲突！
- **必须二选一**：要么全用DJIMotor，要么全用PowerControl

### 4.3 功率模型参数（PowerControl特有）

```c
// power_control.c
#define TORQUE_COEF 0.0003662109375f        // (20/16384)*(0.3)
#define POWER_COEF 187.0f / 3591.0f / 9.55f

const float K1[4] = {1.23e-07, 1.23e-07, 1.23e-07, 1.23e-07};
const float K2[4] = {1.453e-07, 1.453e-07, 1.453e-07, 1.453e-07};
const float constant[4] = {4.081f, 4.081f, 4.081f, 4.081f};

static float chassis_max_power;  // 最大功率限制
```

**功率模型公式**：
```
P = K1 × T² + K2 × n² + C × n × T + constant
其中：
  T = 转矩（Nm）
  n = 转速（rpm）
  P = 功率（W）
```

### 4.4 反馈数据结构

| 模块 | 结构体 | 速度单位 | 编码器范围 |
|------|--------|----------|------------|
| DJI/PowerControl | `DJI_Motor_Measure_s` | 度/秒 | 0~8191 |
| LKMotor | `LKMotor_Measure_t` | 弧度/秒 | 0~65535 |

---

## 五、使用场景选择

### 5.1 什么时候用 DJIMotor？

- 云台电机（GM6020）：需要位置环+速度环
- 拨盘电机（M2006）：需要位置环或速度环
- 测试/调试：需要完整的PID控制
- 不需要功率限制的场景

### 5.2 什么时候用 PowerControl？

- **底盘轮毂电机（M3508）**：需要功率限制
- 裁判系统有功率限制要求（RoboMaster比赛）
- 需要超级电容配合
- 只需要速度控制，不需要精确位置

### 5.3 什么时候用 LKMotor？

- LK系列电机（LK9015, LK9025）
- 需要使用0x280/0x141协议
- 与DJI电机共存于同一CAN总线（ID不冲突）

### 5.4 能否混用？

| 组合 | 是否可行 | 说明 |
|------|----------|------|
| DJIMotor + LKMotor | ✅ 可以 | CAN ID不冲突，各自独立 |
| PowerControl + LKMotor | ✅ 可以 | 底盘常用组合 |
| DJIMotor + PowerControl | ❌ 不可以 | 共享sender_assignment，会冲突 |

**推荐组合**：
- 底盘：PowerControl（轮毂）+ LKMotor（轮向）
- 云台：DJIMotor（yaw/pitch）+ DJIMotor（拨盘）

---

## 六、代码示例对比

### 6.1 DJIMotor 使用示例

```c
#include "dji_motor.h"

DJIMotorInstance *gimbal_yaw;

void GimbalInit(void) {
    Motor_Init_Config_s config = {
        .motor_type = GM6020,
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 4,  // GM6020拨码开关
        },
        .controller_setting_init_config = {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
        },
        .controller_param_init_config = {
            .angle_PID = {.Kp = 10, .Ki = 0, .Kd = 0.5, .MaxOut = 200},
            .speed_PID = {.Kp = 50, .Ki = 0.1, .Kd = 0, .MaxOut = 30000},
        },
    };
    gimbal_yaw = DJIMotorInit(&config);
}

void GimbalTask(void) {
    DJIMotorSetRef(gimbal_yaw, 45.0f);  // 目标角度45度
    // DJIMotorControl() 在 MotorControlTask() 中被调用
}
```

### 6.2 PowerControl 使用示例

```c
#include "power_control.h"
#include "dji_motor.h"  // 需要DJIMotorSetRef

DJIMotorInstance *wheel_fl, *wheel_fr, *wheel_bl, *wheel_br;

void ChassisInit(void) {
    Motor_Init_Config_s config = {
        .motor_type = M3508,
        .can_init_config = {.can_handle = &hcan2},
        .controller_setting_init_config = {
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,  // 只有速度环！
            .speed_feedback_source = MOTOR_FEED,
        },
        .controller_param_init_config = {
            .speed_PID = {.Kp = 4.5, .Ki = 0, .Kd = 0, .MaxOut = 15000},
        },
    };

    config.can_init_config.tx_id = 1;
    wheel_fl = PowerControlInit(&config);

    config.can_init_config.tx_id = 2;
    wheel_fr = PowerControlInit(&config);

    config.can_init_config.tx_id = 3;
    wheel_bl = PowerControlInit(&config);

    config.can_init_config.tx_id = 4;
    wheel_br = PowerControlInit(&config);
}

void ChassisTask(void) {
    // 设置功率限制（从裁判系统获取）
    SetPowerLimit(referee_data->GameRobotState.chassis_power_limit);

    // 设置速度参考（度/秒）
    DJIMotorSetRef(wheel_fl, 500.0f);
    DJIMotorSetRef(wheel_fr, 500.0f);
    DJIMotorSetRef(wheel_bl, 500.0f);
    DJIMotorSetRef(wheel_br, 500.0f);

    // PowerControl() 在 MotorControlTask() 中被调用
}
```

### 6.3 LKMotor 使用示例

```c
#include "LK9025.h"

LKMotorInstance *wheel_left, *wheel_right;

void WheelInit(void) {
    Motor_Init_Config_s config = {
        .can_init_config = {.can_handle = &hcan2},
        .controller_setting_init_config = {
            .outer_loop_type = CURRENT_LOOP,
            .close_loop_type = CURRENT_LOOP,
        },
        .controller_param_init_config = {
            .current_PID = {.Kp = 1, .Ki = 0, .Kd = 0, .MaxOut = 2000},
        },
    };

    config.can_init_config.tx_id = 1;
    wheel_left = LKMotorInit(&config);

    config.can_init_config.tx_id = 2;
    wheel_right = LKMotorInit(&config);
}

void WheelTask(void) {
    LKMotorSetRef(wheel_left, 500.0f);   // 电流值
    LKMotorSetRef(wheel_right, 500.0f);
    // LKMotorControl() 在 MotorControlTask() 中被调用
}
```

---

## 七、MotorControlTask() 的调用

位置：`modules/motor/motor_task.c`

```c
void MotorControlTask() {
    DJIMotorControl();   // 如果有DJI电机
    PowerControl();      // 如果有PowerControl电机
    LKMotorControl();    // 如果有LK电机
}
```

**注意**：
- 三个控制函数都会被调用
- 但只有注册了对应电机的函数才会实际工作
- 如果用PowerControl，就不要注册DJIMotor（避免冲突）

---

## 八、常见问题与注意事项

### 8.1 PowerControl的功率模型参数

当前参数是针对M3508的，如果换用其他电机需要修改：
```c
const float K1[4] = {...};      // 需要重新辨识
const float K2[4] = {...};      // 需要重新辨识
const float constant[4] = {...}; // 需要重新辨识
```

### 8.2 PowerControl没有电流环

- PowerControl只计算速度环，输出直接作为电流值
- 速度环的`MaxOut`应该设为电机最大电流（M3508为15000左右）
- 不像DJIMotor那样有电流环做内环

### 8.3 功率限制的触发条件

```c
if (initial_total_power > chassis_max_power) {
    // 触发功率限制，按比例缩小输出
}
```

- 需要通过`SetPowerLimit()`设置最大功率
- 通常从裁判系统获取：`referee_data->GameRobotState.chassis_power_limit`

### 8.4 DJIMotor和PowerControl不能混用

如果混用，会出现：
- CAN发送冲突（共享sender_assignment）
- 部分电机控制失效
- 功率计算错误

**解决方案**：底盘全用PowerControl，云台全用DJIMotor（不同CAN总线或分开控制）

### 8.5 LKMotor的反馈单位

- 速度是**弧度/秒**，不是度/秒
- 使用时注意单位转换

---

## 九、总结：如何让这些函数起作用

### Step 1：选择正确的模块

| 场景 | 选择 |
|------|------|
| 底盘轮毂M3508 + 功率限制 | PowerControl |
| 云台GM6020/M3508 | DJIMotor |
| LK系列电机 | LKMotor |

### Step 2：初始化

```c
// 在应用层Init函数中
motor = DJIMotorInit(&config);     // 或
motor = PowerControlInit(&config); // 或
motor = LKMotorInit(&config);
```

### Step 3：设置参考值

```c
DJIMotorSetRef(motor, ref);  // DJI和PowerControl共用
LKMotorSetRef(motor, ref);   // LK专用
```

### Step 4：确保控制任务运行

`MotorControlTask()` 在 `robot_task.c` 的 `StartMOTORTASK()` 中以500Hz运行，会自动调用：
- `DJIMotorControl()`
- `PowerControl()`
- `LKMotorControl()`

### Step 5：功率限制（仅PowerControl）

```c
SetPowerLimit(80.0f);  // 设置最大功率80W
```

---

## 十、文件索引

| 文件 | 路径 | 内容 |
|------|------|------|
| dji_motor.h | `modules/motor/DJImotor/dji_motor.h` | DJI电机API |
| dji_motor.c | `modules/motor/DJImotor/dji_motor.c` | DJI电机实现 |
| power_control.h | `modules/motor/power_control.h` | PowerControl API |
| power_control.c | `modules/motor/power_control.c` | PowerControl实现 |
| LK9025.h | `modules/motor/LKmotor/LK9025.h` | LK电机API |
| LK9025.c | `modules/motor/LKmotor/LK9025.c` | LK电机实现 |
| motor_task.c | `modules/motor/motor_task.c` | 电机控制总入口 |
| motor_def.h | `modules/motor/motor_def.h` | 通用定义 |
