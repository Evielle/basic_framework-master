/**
 * @file zero_angle.h
 * @brief 光电门校准模块
 */
#ifndef __ZERO_ANGLE_H
#define __ZERO_ANGLE_H

#include "stm32f4xx_hal.h"
#include "bsp_gpio.h"
#include "stm32f4xx_hal_gpio.h"
#include "dji_motor.h"


typedef enum {
    ZEROANGLE_STATE_IDLE,         // 空闲(初始状态)
    ZEROANGLE_STATE_RUNNING,      //正在校准中
    ZEROANGLE_STATE_DONE,         // 校准完成，等待应用阈值
    ZEROANGLE_STATE_ERROR         // 出错（如校准值异常）,重新校准
} ZeroAngleState_t;

typedef struct{
    GPIOInstance* gpio_instance;
    GPIO_PinState new_state;    //经过软件消抖后的稳定电平状态
    GPIO_PinState old_state;

    uint8_t state;              // 0=空闲, 1=找零中, 2=完成, 3=超时恢复
    float zero_angle;          //零点角度，未加减速比，未补偿误差
    DJIMotorInstance *motor; 
    
    /* 新增：超时与换向控制 */
    uint32_t timeout_cnt;     // 当前方向已运行次数（以调用周期计）
    uint32_t timeout_limit;   // 超时阈值（例如3秒 * 200Hz = 600）
    int8_t   direction;       // 1: 正向 / -1: 反向   
}ZeroAngleInstance;

/**
 * @brief  初始化光电门零点校准实例
 * @param  GPIO_Pin : 光电门引脚号 (如 GPIO_PIN_11 / GPIO_PIN_13)
 * @param  motor    : 对应的 DJI 电机实例指针
 * @retval 返回初始化后的实例指针，失败返回 NULL
 */
ZeroAngleInstance *ZeroAngleInit(uint16_t GPIO_Pin,DJIMotorInstance *motor);

/**
 * @brief  处理两个光电门的找零状态机（需周期调用）
 * @param  left  : 左光电门实例 (PE11, motor_rf)
 * @param  right : 右光电门实例 (PE13, motor_lb)
 * @retval 0 : 至少一个电机尚未完成校准
 *         1 : 两个电机均校准完成，可以进入后续控制
 */
uint8_t ZeroAngleProcess(ZeroAngleInstance *left, ZeroAngleInstance *right);


#endif/* __ZERO_ANGLE_H__ */