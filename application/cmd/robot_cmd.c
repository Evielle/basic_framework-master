// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "imageRoad.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static ImageRoad_RC_t *imageRoad_rc;    // 图传遥控器数据,初始化时返回

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;
void RobotCMDInit()
{
    imageRoad_rc = ImageRoadTaskInit(&huart1); // 图传链路初始化,根据实际硬件修改串口号

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

/**
 * @brief 图传遥控器控制
 *        mode_sw: 0/2=全部零力, 1=正常控制
 *        左摇杆→底盘, 右摇杆→云台
 *        trigger→抬升云台, pause→小陀螺
 *
 */
static void ImageRoadRcSet()
{
    // mode_sw == 0 或 2: 全部电机失能
    if (imageRoad_rc[TEMP].rc.mode_sw == 0 ||
        imageRoad_rc[TEMP].rc.mode_sw == 2)
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        return;
    }

    // mode_sw == 1: 正常控制
    // 左摇杆 → 底盘 (rocker_l_ = ch_2, rocker_l1 = ch_3)
    chassis_cmd_send.vx = (float)imageRoad_rc[TEMP].rc.rocker_l_ / 66.0f;
    chassis_cmd_send.vy = (float)imageRoad_rc[TEMP].rc.rocker_l1 / 66.0f;

    // 右摇杆 → 云台 (rocker_r_ = ch_0, rocker_r1 = ch_1)
    gimbal_cmd_send.yaw += 0.005f * (float)imageRoad_rc[TEMP].rc.rocker_r_;
    gimbal_cmd_send.pitch += 0.001f * (float)imageRoad_rc[TEMP].rc.rocker_r1;

    // trigger → 持续抬升云台
    if (imageRoad_rc[TEMP].rc.trigger)
        gimbal_cmd_send.pitch += 0.01f;

    // pause → 小陀螺
    if (imageRoad_rc[TEMP].rc.pause)
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    else
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;

    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
/* 消息超时检测: 连续多次未收到消息则判定离线 */
    static uint8_t gimbal_miss_count = 0;
    static uint8_t chassis_miss_count = 0;

#ifdef ONE_BOARD
    if (SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data) == 0) {
        chassis_miss_count++;
    } else {
        chassis_miss_count = 0;
    }
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    if (CANCommIsOnline(cmd_can_comm)) {
        chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
    } else {
        chassis_miss_count++;
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] CANComm offline, emergency stop!");
    }
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);

    if (SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data) == 0) {
        gimbal_miss_count++;
    } else {
        gimbal_miss_count = 0;
    }

    /* 如果连续100次(500ms)未收到消息，触发急停 */
    if (gimbal_miss_count > 100 || chassis_miss_count > 100) {
        LOGERROR("[CMD] Message timeout, triggering emergency stop!");
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
    }

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    // 图传控制
    if (ImageRoadIsOnline())
        ImageRoadRcSet();

// 将云台姿态数据传递给底盘
    chassis_cmd_send.gimbal_yaw_angle = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
    chassis_cmd_send.gimbal_wz = gimbal_fetch_data.gimbal_imu_data.Gyro[2];

#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
