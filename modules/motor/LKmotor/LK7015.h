#ifndef LK7015_H
#define LK7015_H

#include "stdint.h"
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define LK7015_MOTOR_MX_CNT 4

#define I_MIN -2000
#define I_MAX 2000
#define CURRENT_SMOOTH_COEF 0.9f
#define SPEED_SMOOTH_COEF 0.85f
#define ECD_ANGLE_COEF_LK (360.0f / 65536.0f)
#define CURRENT_TORQUE_COEF_LK 0.00512f

typedef struct
{
    uint16_t last_ecd;
    uint16_t ecd;
    float angle_single_round;
    float speed_rads;
    int16_t real_current;
    uint8_t temperature;

    float total_angle;
    int32_t total_round;

    float feed_dt;
    uint32_t feed_dwt_cnt;
} LK7015_Measure_t;

typedef struct
{
    LK7015_Measure_t measure;

    Motor_Control_Setting_s motor_settings;

    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float pid_ref;

    Motor_Working_Type_e stop_flag;

    CANInstance *motor_can_ins;

    DaemonInstance *daemon;

} LK7015Instance;

LK7015Instance *LK7015Init(Motor_Init_Config_s *config);

void LK7015SetRef(LK7015Instance *motor, float ref);

void LK7015Control(void);

void LK7015Stop(LK7015Instance *motor);

void LK7015Enable(LK7015Instance *motor);

uint8_t LK7015IsOnline(LK7015Instance *motor);

#endif
