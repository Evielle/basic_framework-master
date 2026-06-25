#include "gimbal.h"
#include "robot_def.h"
#include "dmmotor.h"
#include "message_center.h"

static DMMotorInstance *pitch_down;
static Subscriber_t *gimbal_sub;

void GimbalInit()
{
    Motor_Init_Config_s cfg = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 1,
            .rx_id = 0x11,
        },
        .controller_setting_init_config = {
            .outer_loop_type = OPEN_LOOP,
            .close_loop_type = OPEN_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .feedforward_flag = FEEDFORWARD_NONE,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
        },
        .motor_type = DM4310,
    };
    pitch_down = DMMotorInit(&cfg);
    // DMMotorSetMode(DM_CMD_CLEAR_ERROR, pitch_down);
    // DWT_Delay(0.05);
    pitch_down->pos_limit_enable = 1;
    pitch_down->pos_limit_min = 0.0f;
    pitch_down->pos_limit_max = 75.0f;
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

void GimbalTask()
{
    Gimbal_Ctrl_Cmd_s cmd;
    SubGetMessage(gimbal_sub, &cmd);
    if (cmd.gimbal_mode == GIMBAL_ZERO_FORCE)
    {
        DMMotorStop(pitch_down);
        return;
    }
    DMMotorEnable(pitch_down);
    DMMotorSetRef(pitch_down, 1.0f);
}
