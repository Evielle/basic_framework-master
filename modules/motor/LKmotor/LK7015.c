#include "LK7015.h"
#include "stdlib.h"
#include "general_def.h"
#include "daemon.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

static uint8_t idx;
static LK7015Instance *lkmotor_instance[LK7015_MOTOR_MX_CNT] = {NULL};
static CANInstance *sender_instance;

static void LK7015Decode(CANInstance *_instance)
{
    LK7015Instance *motor = (LK7015Instance *)_instance->id;
    LK7015_Measure_t *measure = &motor->measure;
    uint8_t *rx_buff = _instance->rx_buff;

    DaemonReload(motor->daemon);
    measure->feed_dt = DWT_GetDeltaT(&measure->feed_dwt_cnt);

    measure->last_ecd = measure->ecd;
    measure->ecd = (uint16_t)((rx_buff[7] << 8) | rx_buff[6]);

    measure->angle_single_round = ECD_ANGLE_COEF_LK * measure->ecd;

    measure->speed_rads = (1 - SPEED_SMOOTH_COEF) * measure->speed_rads +
                          DEGREE_2_RAD * SPEED_SMOOTH_COEF * (float)((int16_t)(rx_buff[5] << 8 | rx_buff[4]));

    measure->real_current = (1 - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rx_buff[3] << 8 | rx_buff[2]));

    measure->temperature = rx_buff[1];

    if (measure->ecd - measure->last_ecd > 32768)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -32768)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

static void LK7015LostCallback(void *motor_ptr)
{
    LK7015Instance *motor = (LK7015Instance *)motor_ptr;
    LOGWARNING("[LK7015] motor lost, id: %d", motor->motor_can_ins->tx_id);
}

LK7015Instance *LK7015Init(Motor_Init_Config_s *config)
{
    LK7015Instance *motor = (LK7015Instance *)malloc(sizeof(LK7015Instance));
    if (motor == NULL) {
        return NULL;
    }
    memset(motor, 0, sizeof(LK7015Instance));

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.id = motor;
    config->can_init_config.can_module_callback = LK7015Decode;
    config->can_init_config.rx_id = 0x140 + config->can_init_config.tx_id;
    config->can_init_config.tx_id = config->can_init_config.tx_id + 0x280 - 1;
    motor->motor_can_ins = CANRegister(&config->can_init_config);

    if (idx == 0)
    {
        sender_instance = motor->motor_can_ins;
        sender_instance->tx_id = 0x280;
    }

    LK7015Enable(motor);
    DWT_GetDeltaT(&motor->measure.feed_dwt_cnt);
    lkmotor_instance[idx++] = motor;

    Daemon_Init_Config_s daemon_config = {
        .callback = LK7015LostCallback,
        .owner_id = motor,
        .reload_count = 5,
    };
    motor->daemon = DaemonRegister(&daemon_config);

    return motor;
}

void LK7015Control(void)
{
    float pid_measure, pid_ref;
    int16_t set;
    LK7015Instance *motor;
    LK7015_Measure_t *measure;
    Motor_Control_Setting_s *setting;

    for (size_t i = 0; i < idx; ++i)
    {
        motor = lkmotor_instance[i];
        measure = &motor->measure;
        setting = &motor->motor_settings;
        pid_ref = motor->pid_ref;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;

        if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP)
        {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor->other_angle_feedback_ptr;
            else
                pid_measure = measure->total_angle;
            pid_ref = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
            if (setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor->speed_feedforward_ptr;
        }

        if ((setting->close_loop_type & SPEED_LOOP) && setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))
        {
            if (setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor->other_speed_feedback_ptr;
            else
                pid_measure = measure->speed_rads;
            pid_ref = PIDCalculate(&motor->speed_PID, pid_measure, pid_ref);
            if (setting->feedforward_flag & CURRENT_FEEDFORWARD)
                pid_ref += *motor->current_feedforward_ptr;
        }

        if (setting->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&motor->current_PID, measure->real_current, pid_ref);
        }

        set = (int16_t)pid_ref;

        memcpy(sender_instance->tx_buff + (motor->motor_can_ins->tx_id - 0x280) * 2, &set, sizeof(uint16_t));

        if (motor->stop_flag == MOTOR_STOP)
        {
            memset(sender_instance->tx_buff + (motor->motor_can_ins->tx_id - 0x280) * 2, 0, sizeof(uint16_t));
        }
    }

    if (idx)
        CANTransmit(sender_instance, 0.2);
}

void LK7015Stop(LK7015Instance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void LK7015Enable(LK7015Instance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void LK7015SetRef(LK7015Instance *motor, float ref)
{
    motor->pid_ref = ref;
}

uint8_t LK7015IsOnline(LK7015Instance *motor)
{
    if (motor == NULL) return 0;
    return DaemonIsOnline(motor->daemon);
}
