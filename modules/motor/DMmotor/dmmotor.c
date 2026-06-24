#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"
#include <math.h>
// #include "seekfree.h"

float  a11 = 0,a22 = 0;

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];
static uint8_t zero_buff[6] = {0};

#ifndef DM_LOCAL_POS_LIMIT_ENABLE
#define DM_LOCAL_POS_LIMIT_ENABLE 0
#endif
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    if (!isfinite(x))
        x = 0.0f;
    if (x < x_min)
        x = x_min;
    else if (x > x_max)
        x = x_max;

    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
// 判断符号位
#define sign(x) ((x > 0) ? 1 : -1)
#define abs(x) ((x > 0) ? x : -x)

void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
    memcpy(motor->motor_can_instace->tx_buff, zero_buff, 6);
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    if (motor->was_lost)
    {
        DMMotorSetMode(DM_CMD_MOTOR_MODE, motor); // 离线恢复后首帧补发使能，覆盖断电重上电场景
        motor->was_lost = 0;
        motor->lost_cnt = 0;
    }

    DaemonReload(motor->motor_daemon);

    measure->id = rxbuff[0] & 0x0F;
    measure->state = rxbuff[0]>>4;

    measure->last_position = measure->position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16) * RAD_2_DEGREE;

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12) * RAD_2_DEGREE;

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];

    if (measure->position - measure->last_position > 180)
        measure->total_round--;
    else if (measure->position - measure->last_position < -180)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->position;
}

static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    LOGWARNING("[dm_motor] motor %d lost\n", motor->motor_can_instace->tx_id);
    motor->was_lost = 1;
    if (++motor->lost_cnt % 10 == 0)
        DMMotorSetMode(DM_CMD_MOTOR_MODE, motor); // 尝试重新让电机进入控制模式
    // DWT_Delay(0.1);
}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{

    uint16_t p, v, kp, kd, t;
    p = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
    v = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
    kp = 0;
    kd = 0;
    t = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);

    uint8_t *buf = motor->motor_can_instace->tx_buff;
    buf[0] = p >> 8;
    buf[1] = p & 0xFF;
    buf[2] = v >> 4;
    buf[3] = ((v & 0xF) << 4) | (kp >> 8);
    buf[4] = kp & 0xFF;
    buf[5] = kd >> 4;
    buf[6] = ((kd & 0xF) << 4) | (t >> 8);
    buf[7] = t & 0xff;

    memcpy(zero_buff, buf, 6); // 初始化的时候至少调用一次,故将其他指令为0时发送的报文保存一下,详见ht04电机说明
    CANTransmit(motor->motor_can_instace, 1);
    DWT_Delay(0.1);

    // 设置零点
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)zmalloc(sizeof(DMMotorInstance));
    
    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    motor->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);
    motor->feedforward = 0;
    motor->pos_limit_enable = 0;
    motor->lost_cnt = 0;
    motor->was_lost = 0;
    // DMMotorEnable(motor);
    // DWT_Delay(0.1);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    // DMMotorCaliEncoder(motor);
    // DWT_Delay(0.1);
    dm_motor_instance[idx++] = motor;
    return motor;
}

/* 电流只能通过电机自带传感器监测,后续考虑加入力矩传感器应变片等 */
void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        LOGERROR("[dm_motor] loop type error, check memory access and func param"); // 检查是否传入了正确的LOOP类型,或发生了指针越界
}


/**
 * @brief 设pid参考值
 * 
 * @param motor 
 * @param ref 
 */
void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    if (!isfinite(ref))
        ref = 0.0f;
    motor->motor_controller.pid_ref = ref;
}

// 反向
void DMMotorSetReverse(DMMotorInstance *motor, uint8_t reverse_flag)
{
    motor->motor_settings.feedback_reverse_flag = reverse_flag;
}
// 设前馈
void DMMotorSetFed(DMMotorInstance *motor, float fed)
{
    motor->feedforward = fed;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
    if(motor->last_stop_flag==MOTOR_STOP&&motor->stop_flag==MOTOR_ENALBED)
    {
        DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    }
    motor->last_stop_flag=MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
    motor->last_stop_flag=MOTOR_STOP;
}
// 设外环
void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

DMMotor_Send_s motor_send_mai[2];
//@Todo: 目前只实现了力控，更多位控PID等请自行添加
void DMMotorTask(void)
{
    float set, pid_measure, pid_ref;
    DMMotorInstance *motor;
    Motor_Control_Setting_s *setting;
    Motor_Controller_s *motor_controller;   // 电机控制器
    DM_Motor_Measure_s *measure;           // 电机测量值
    DMMotor_Send_s motor_send_mailbox;
    CANInstance *motor_can;
    uint16_t tmp;

    for(size_t i = 0; i < idx; ++i)
    {
        motor = dm_motor_instance[i];
        setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        motor_can = motor->motor_can_instace;
        pid_ref = motor_controller->pid_ref;

        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;
        
        if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP)
        {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor->motor_controller.other_angle_feedback_ptr;
            else
                pid_measure = measure->total_angle;
            // measure单位是rad,ref是角度,统一到angle下计算,方便建模
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        if ((setting->close_loop_type & SPEED_LOOP) && setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))
        {
            if ((setting->feedforward_flag & SPEED_FEEDFORWARD) &&
                motor_controller->speed_feedforward_ptr != NULL)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else
                pid_measure = measure->velocity;
            // measure单位是rad / s ,ref是angle per sec,统一到angle下计算
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        if ((setting->feedforward_flag & CURRENT_FEEDFORWARD) &&
            motor_controller->current_feedforward_ptr != NULL)
            pid_ref += *motor_controller->current_feedforward_ptr;
        if (setting->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->torque, pid_ref);
        }

        if (setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        if (!isfinite(pid_ref))
            pid_ref = 0.0f;
        if (!isfinite(motor->feedforward))
            motor->feedforward = 0.0f;
        set = pid_ref + motor->feedforward;
        if (!isfinite(set))
            set = 0.0f;
        LIMIT_MIN_MAX(set, DM_T_MIN, DM_T_MAX);

        motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
        motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
        motor_send_mailbox.torque_des = float_to_uint(set, DM_T_MIN, DM_T_MAX, 12);
        motor_send_mailbox.Kp = 0;
        motor_send_mailbox.Kd = 0;    
        motor->debug_pos_limit_active = 0;

        float tar_p;
        if(DM_LOCAL_POS_LIMIT_ENABLE && motor->pos_limit_enable)
        {
            if (measure->total_angle <= motor->pos_limit_min + 1) //1度
            {
                tar_p = motor->pos_limit_min + 1;
                motor_send_mailbox.Kp = 40;
                motor_send_mailbox.Kd = 2;
                tar_p *= DEGREE_2_RAD;
                LIMIT_MIN_MAX(tar_p, DM_P_MIN, DM_P_MAX);
                motor_send_mailbox.position_des = float_to_uint(tar_p, DM_P_MIN, DM_P_MAX, 16);
                motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
                motor->debug_pos_limit_active = 1;
            }
            else if (measure->total_angle >= motor->pos_limit_max - 1)
            {
                tar_p = motor->pos_limit_max - 1;
                motor_send_mailbox.Kp = 40;
                motor_send_mailbox.Kd = 2;
                tar_p *= DEGREE_2_RAD;
                LIMIT_MIN_MAX(tar_p, DM_P_MIN, DM_P_MAX);
                motor_send_mailbox.position_des = float_to_uint(tar_p, DM_P_MIN, DM_P_MAX, 16);
                motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
                motor->debug_pos_limit_active = 1;
            }
            else
            {
                motor_send_mailbox.Kp = 0;
                motor_send_mailbox.Kd = 0;
            }
        }

        if(motor->stop_flag == MOTOR_STOP)
        {
            PIDReset(&motor_controller->angle_PID);
            PIDReset(&motor_controller->speed_PID);
            PIDReset(&motor_controller->current_PID);
            motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
            motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
            motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
            motor_send_mailbox.Kp = 0;
            motor_send_mailbox.Kd = 0;
            motor->debug_pos_limit_active = 0;
        }
        motor->debug_last_set_nm = set;
        motor->debug_last_kp = motor_send_mailbox.Kp;
        motor->debug_last_kd = motor_send_mailbox.Kd;
        motor->debug_last_torque_des = motor_send_mailbox.torque_des;
        motor_send_mai[i] = motor_send_mailbox;
        motor_can->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
        motor_can->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
        motor_can->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
        motor_can->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
        motor_can->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
        motor_can->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
        motor_can->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
        motor_can->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

        CANTransmit(motor->motor_can_instace, 1);
    }
}
