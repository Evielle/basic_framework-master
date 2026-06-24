/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "controller.h"
#include "dji_motor.h"
#include "motor_def.h"
#include "robot_def.h"
#include "power_control.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include "LK7015.h"
#include "zero_angle.h"
#include "robot_cmd.h"
#include "InverseKinematics.h"
#include "bsp_log.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define L_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X) /1000.f)//半轮距，转换为m
#define R_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X) /1000.f)//半轮距，转换为m
#define ECD_ANGLE_COEF_ZF 0.08789f // (360/4096),将舵轮M3508编码器值转化为角度制
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_ZF) // 对齐时的角度,0-360
#define RADIUS_WHEEL_M       0.0525f                          // 驱动轮，轮子半径，单位米
#define REDUCTION_RATIO_WHEEL 19.0f                          // 驱动减速比
#define REDUCTION_RATIO_STEER 8.0f                           // 转向减速比

// 舵轮安装方位角（弧度）
#define AZIMUTH_FRONT        0.0f
#define AZIMUTH_REAR         PI

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static PIDInstance buffer_PID;             // 用于底盘的缓冲能量PID
static referee_info_t *referee_data;       // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

// static SuperCapInstance *cap;                                       // 超级电容
static ChassisMotorContext motor_ctx;  // 在 .bss 段分配，生命周期等同全局
static DJIMotorInstance *motor_l, *motor_r; // 大疆M3508电机实例
static LK7015Instance *motor_l_lk,*motor_r_lk;   //LK7015电机实例
static ZeroAngleInstance *zero_angle_l,*zero_angle_r;//光电门实例

static uint8_t zero_angle_done = 0;                                 //为完成校准为零，完成校准为一
// static float chassis_wz_ref,chassis_vx_ref,chassis_vy_ref; //底盘角速度目标值
// static Chassis_Velocity_s chassis_velocity;
/* 用于自旋变速策略的时间变量 */
// static float t;
// static volatile float test_speed_measure = 0;
// static volatile float motor_rf_AngleRef = 0.0f;
// static volatile float motor_lb_AngleRef = 0.0f;
// static volatile float motor_rf_lk_SpeedRef = 0.0f;
// static volatile float motor_lb_lk_SpeedRef = 0.0f;
/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx=0;
static float chassis_vy=0;
static float chassis_wz=0;                      // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;                  // 底盘速度解算后的临时输出,待进行限幅


void ChassisInit()
{
     // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s motor_m3508_config = {
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0,
                .Ki = 1,
                .Kd = 0,
                .IntegralLimit = 6000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 16384,
            },
            .speed_PID = {
                .Kp = 0,
                .Ki = 1,
                .Kd = 0,
                .IntegralLimit = 2000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 3500,
            },
            .other_angle_feedback_ptr = NULL,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_AND_SPEED_LOOP,
        },
        .motor_type = M3508,
    };
    motor_m3508_config.can_init_config.tx_id = 1;
    motor_m3508_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_l = DJIMotorInit(&motor_m3508_config);//左舵机
    motor_m3508_config.can_init_config.tx_id = 2;
    motor_m3508_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_r = DJIMotorInit(&motor_m3508_config);//右舵机

    Motor_Init_Config_s motor_lk9025_config = {
    .can_init_config.can_handle = &hcan2,
    .controller_param_init_config = {
        .speed_PID = {
            .Kp = 0,
            .Ki = 1,
            .Kd = 0,
            .IntegralLimit = 2000,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
            .MaxOut = 8000,
            },
        },
        .controller_setting_init_config = {
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
        },
        .motor_type = LK7015,
    };
    motor_lk9025_config.can_init_config.tx_id = 1;
    motor_lk9025_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_l_lk = LK7015Init(&motor_lk9025_config);  // 左驱, CAN ID=1
    motor_lk9025_config.can_init_config.tx_id = 2;
    motor_lk9025_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_r_lk = LK7015Init(&motor_lk9025_config);  // 右驱, CAN ID=2

    zero_angle_l=ZeroAngleInit(LightGateL_Pin,motor_l);
    zero_angle_r=ZeroAngleInit(LightGateR_Pin,motor_r);
    
    motor_ctx.steer_left  = motor_l;
    motor_ctx.steer_right = motor_r;
    motor_ctx.drive_left  = motor_l_lk;
    motor_ctx.drive_right = motor_r_lk;
    motor_ctx.zero_left   = zero_angle_l;
    motor_ctx.zero_right  = zero_angle_r;
    InverseKinematics_Init(&motor_ctx);


    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}


/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 获取新的控制信息
    // static float last_cmd_time = DWT_GetTimeline_ms(); // 初始化避免上电即超时
    // static uint8_t cmd_timeout_flag = 0;
    // float now = DWT_GetTimeline_ms();

#ifdef ONE_BOARD
    if (SubGetMessage(chassis_sub, &chassis_cmd_recv)) {
        // last_cmd_time = now;
        // if (cmd_timeout_flag) {
        //     cmd_timeout_flag = 0;
            LOGINFO("[CHASSIS] Communication restored");
        // }
    }
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

    if(zero_angle_done==0){
        zero_angle_done=ZeroAngleProcess(zero_angle_l, zero_angle_r);
        // 检查是否有舵机进入 ERROR 状态
        if ((zero_angle_l != NULL && zero_angle_l->state == ZEROANGLE_STATE_ERROR) ||
            (zero_angle_r != NULL && zero_angle_r->state == ZEROANGLE_STATE_ERROR)) {
            InverseKinematics_EmergencyStop();
            DJIMotorStop(motor_ctx.steer_left);
            DJIMotorStop(motor_ctx.steer_right);
            LK7015Stop(motor_ctx.drive_left);
            LK7015Stop(motor_ctx.drive_right);
            return;
        }
        if(zero_angle_done)
            InverseKinematics_UpdateZeroAngle();
    }
#if 0
    InverseKinematics_ComputePhysicalAngle();
    // 云台坐标系->底盘坐标系变换: 底盘逆时针为正
    // 使用 offset_angle 进行旋转矩阵计算
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
    chassis_wz = chassis_cmd_recv.wz;

    // 直接使用档位值（robot_cmd.c 侧输出 [-10, 10] 档位）
    int8_t vx_level = (int8_t)(chassis_vx);
    int8_t vy_level = (int8_t)(chassis_vy);
    int8_t wz_level = (int8_t)(chassis_wz);

    InverseKinematics_Drive(vx_level, vy_level, wz_level);

    SetPowerLimit(referee_data->GameRobotState.chassis_power_limit);

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif
#endif // CHASSIS_BOARD
}

