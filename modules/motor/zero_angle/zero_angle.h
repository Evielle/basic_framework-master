#ifndef ZERO_ANGLE_H
#define ZERO_ANGLE_H

#include "dji_motor.h"               // DJIMotorInstance
#include "stm32f4xx.h"
#include <stdlib.h>

#define ZERO_MOTOR_NUM  2        // 支持同时校准的电机数量

/* 单个电机的校准配置 */
typedef struct {
    DJIMotorInstance *motor;     // 电机实例指针
    uint16_t          gate_pin;  // 对应光电门引脚 (GPIO_PIN_x)
} ZeroMotorCfg_t;

/* 校准总配置 */
typedef struct {
    ZeroMotorCfg_t motor_cfg[ZERO_MOTOR_NUM];  // 两个电机的配置
    float          cali_speed;                 // 校准旋转速度(度/秒)
    uint32_t       timeout_ms;                 // 超时时间(毫秒)
} ZeroConfig_t;

/* 校准状态 */
typedef enum {
    ZERO_IDLE = 0,      /* 空闲 */
    ZERO_RUNNING,       /* 校准中 */
    ZERO_DONE,          /* 校准成功，两个电机都成功触发 */
    ZERO_TIMEOUT,       /* 超时失败，至少一个未触发 */
    ZERO_ERROR          /* 其它错误 */
} ZeroState_t;

/* 全局状态（外部可读） */
extern ZeroState_t zero_state;

/* 初始化：注册光电门，准备中断（在CubeMX已配置GPIO的前提下） */
void ZeroInit(ZeroConfig_t *config);

/* 启动校准（可在遥控器回调中调用） */
void ZeroStart(void);

/* 停止校准（手动中止） */
void ZeroStop(void);

/* 校准状态机，需以固定频率调用（如 100Hz 或 200Hz） */
void ZeroProcess(void);

/* 获取校准后的零位偏移（单位：度），只有 ZERO_DONE 时有效 */
float ZeroGetOffset(void);

/* 光电门中断回调，需在 HAL_GPIO_EXTI_Callback() 中调用 */
void ZeroEXTICallback(uint16_t GPIO_Pin);

#endif