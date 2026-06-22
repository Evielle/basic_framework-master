#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include <math.h>

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static BMI088Instance *bmi088; // 云台IMU
void GimbalInit()
{   
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8, // 8
                .Ki = 0,
                .Kd = 0,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,

                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 50,  // 50
                .Ki = 200, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10, // 10
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 50,  // 50
                .Ki = 350, // 350
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[0]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // IMU数据有效性检测
    static float last_yaw_angle = 0.0f;
    static float last_pitch_angle = 0.0f;
    static uint8_t imu_freeze_count = 0;
    static uint8_t imu_error_flag = 0;
    static uint8_t imu_error_logged = 0;
    static uint16_t imu_recovery_count = 0;

    // 1. 检查IMU数据是否在合理范围内
    if (gimba_IMU_data->YawTotalAngle < -18000.0f || gimba_IMU_data->YawTotalAngle > 18000.0f ||
        gimba_IMU_data->Pitch < -90.0f || gimba_IMU_data->Pitch > 90.0f)
    {
        imu_error_flag = 1;
    }

    // 2. 检查IMU数据是否冻结(连续10帧相同), 仅在IMU正常时检测
    if (!imu_error_flag)
    {
        float yaw_diff = gimba_IMU_data->YawTotalAngle - last_yaw_angle;
        float pitch_diff = gimba_IMU_data->Pitch - last_pitch_angle;
        if (fabsf(yaw_diff) < 0.01f && fabsf(pitch_diff) < 0.01f)
        {
            imu_freeze_count++;
            if (imu_freeze_count >= 10)
            {
                imu_error_flag = 1;
            }
        }
        else
        {
            imu_freeze_count = 0;
            last_yaw_angle = gimba_IMU_data->YawTotalAngle;
            last_pitch_angle = gimba_IMU_data->Pitch;
        }
    }

    // 3. IMU异常时切换到电机编码器反馈
    if (imu_error_flag)
    {
        if (!imu_error_logged)
        {
            LOGERROR("[GIMBAL] IMU data invalid, switching to motor encoder feedback");
            imu_error_logged = 1;
        }
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, MOTOR_FEED);
    }

    // 4. IMU数据恢复检测
    if (imu_error_flag)
    {
        // 检查数据是否在合理范围内
        if (gimba_IMU_data->YawTotalAngle >= -18000.0f && gimba_IMU_data->YawTotalAngle <= 18000.0f &&
            gimba_IMU_data->Pitch >= -90.0f && gimba_IMU_data->Pitch <= 90.0f)
        {
            float yaw_diff = gimba_IMU_data->YawTotalAngle - last_yaw_angle;
            float pitch_diff = gimba_IMU_data->Pitch - last_pitch_angle;
            // 数据在变化（未冻结），认为IMU恢复正常
            if (fabsf(yaw_diff) > 0.01f || fabsf(pitch_diff) > 0.01f)
            {
                imu_recovery_count++;
                last_yaw_angle = gimba_IMU_data->YawTotalAngle;
                last_pitch_angle = gimba_IMU_data->Pitch;
            }
            else
            {
                imu_recovery_count = 0;
            }
        }
        else
        {
            imu_recovery_count = 0;
        }

        if (imu_recovery_count >= 100)
        {
            imu_error_flag = 0;
            imu_error_logged = 0;
            imu_recovery_count = 0;
            LOGERROR("[GIMBAL] IMU data recovered, switching back to IMU feedback");
            DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        }
    }
    else
    {
        imu_recovery_count = 0;
    }

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        if (!imu_error_flag)
        {
            DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        }
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    default:
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}