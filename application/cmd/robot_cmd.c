// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "imageRoad.h"
#include "message_center.h"
#include "Vofa.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

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

#ifdef GIMBAL_BOARD
static Vofa_HandleTypedef vofa_handle;
static float vofa_data[3];
#endif

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
    Vofa_Init(&vofa_handle, VOFA_MODE_SKIP);
#endif
    gimbal_cmd_send.pitch = 0;

    robot_state = ROBOT_READY;
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
    chassis_cmd_send.vx = (float)imageRoad_rc[TEMP].rc.rocker_l_ * 3000.0f / 660.0f;
    chassis_cmd_send.vy = (float)imageRoad_rc[TEMP].rc.rocker_l1 * 3000.0f / 660.0f;

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

static void CmdEmergencyStop(void)
{
    robot_state = ROBOT_STOP;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
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
        CmdEmergencyStop();
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
        CmdEmergencyStop();
    }

    // 图传控制
    if (ImageRoadIsOnline())
        ImageRoadRcSet();

#if defined(ONE_BOARD)
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#elif defined(GIMBAL_BOARD)
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
    vofa_data[0] = chassis_fetch_data.steer_left_ref;
    vofa_data[1] = chassis_fetch_data.steer_left_meas;
    vofa_data[2] = chassis_fetch_data.steer_left_out;
    Vofa_JustFloat(&vofa_handle, vofa_data, 3);
#endif

    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
