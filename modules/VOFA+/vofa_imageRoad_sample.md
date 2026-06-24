# Vofa + imageRoad 使用参考

在 `RobotCMDTask()` 中通过 Vofa JustFloat 协议输出图传链路摇杆和开关数据，上位机（VOFA+）可实时观察各通道波形。

## 通道映射

| 通道 | 字段 | 说明 |
|------|------|------|
| ch0 | `rocker_r_` | 右摇杆水平 |
| ch1 | `rocker_r1` | 右摇杆竖直 |
| ch2 | `rocker_l_` | 左摇杆水平 |
| ch3 | `rocker_l1` | 左摇杆竖直 |
| ch4 | `mode_sw` | 档位开关 |

## 集成步骤

### 1. 添加 include

```c
#include "Vofa.h"
#include "imageRoad.h"
```

### 2. 声明全局变量

```c
static Vofa_HandleTypedef vofa_handle;
static float vofa_data[5] = {0};
```

### 3. 初始化

```c
Vofa_Init(&vofa_handle, VOFA_MODE_SKIP);
imageRoad_rc = ImageRoadTaskInit(&huart1);
```

### 4. 200Hz 任务中输出

```c
vofa_data[0] = (float)imageRoad_rc[TEMP].rc.rocker_r_;
vofa_data[1] = (float)imageRoad_rc[TEMP].rc.rocker_r1;
vofa_data[2] = (float)imageRoad_rc[TEMP].rc.rocker_l_;
vofa_data[3] = (float)imageRoad_rc[TEMP].rc.rocker_l1;
vofa_data[4] = (float)imageRoad_rc[TEMP].rc.mode_sw;
Vofa_JustFloat(&vofa_handle, vofa_data, 5);
```

## 完整示例（`robot_cmd.c` 中用法）

```c
// include
#include "Vofa.h"
#include "imageRoad.h"

// 全局变量
static Vofa_HandleTypedef vofa_handle;
static float data[5] = {0};
static ImageRoad_RC_t *imageRoad_rc;

// 初始化
void RobotCMDInit()
{
    imageRoad_rc = ImageRoadTaskInit(&huart1);
    Vofa_Init(&vofa_handle, VOFA_MODE_SKIP);
    // ... 其余初始化
}

// 200Hz 任务循环
void RobotCMDTask()
{
    data[0] = (float)imageRoad_rc[TEMP].rc.rocker_r_;
    data[1] = (float)imageRoad_rc[TEMP].rc.rocker_r1;
    data[2] = (float)imageRoad_rc[TEMP].rc.rocker_l_;
    data[3] = (float)imageRoad_rc[TEMP].rc.rocker_l1;
    data[4] = (float)imageRoad_rc[TEMP].rc.mode_sw;
    Vofa_JustFloat(&vofa_handle, data, 5);
    // ... 其余控制逻辑
}
```
