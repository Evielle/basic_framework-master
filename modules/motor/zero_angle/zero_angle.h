#ifndef __ZERO_ANGLE_H
#define __ZERO_ANGLE_H

#include "stm32f4xx_hal.h"
#include "bsp_gpio.h"
#include "stm32f4xx_hal_gpio.h"
#include "dji_motor.h"

typedef enum {
    ZEROANGLE_STATE_IDLE,
    ZEROANGLE_STATE_RUNNING,
    ZEROANGLE_STATE_DONE,
} ZeroAngleState_t;

typedef struct {
    GPIOInstance *gpio_instance;
    GPIO_PinState new_state;      // 消抖后的稳定电平
    GPIO_PinState old_state;      // 上一次稳定电平
    uint8_t state;                // IDLE / RUNNING / DONE
    float zero_angle;             // 找到的零点角度
    DJIMotorInstance *motor;
    uint8_t debounce_cnt;         // 软件消抖计数器
    int8_t direction;             // 旋转方向: 1=CW, -1=CCW
} ZeroAngleInstance;

ZeroAngleInstance *ZeroAngleInit(uint16_t GPIO_Pin, DJIMotorInstance *motor, int8_t direction);

uint8_t ZeroAngleProcess(ZeroAngleInstance *left, ZeroAngleInstance *right);

#endif/* __ZERO_ANGLE_H__ */
