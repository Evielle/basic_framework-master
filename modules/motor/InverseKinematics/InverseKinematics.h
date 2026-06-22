/**
 * @file InverseKinematics.h
 * @brief 双舵轮底盘运动学逆解模块（对角布局）
 * 
 * @note  坐标系: 右手系，+X前进，+Y左移，+Wz逆时针
 *        两组舵轮在底盘对角线上对称分布（另一对角为万向轮）
 *        舵轮坐标由 robot_def.h 的 WHEEL_BASE/TRACK_WIDTH 计算得到
 *        驱动轮半径: 0.105m, 减速比 19:1 (LK7015)
 *        舵向减速比: 8:1 (M3508)
 *        使用速度档位(1~10)以保证低速高稳定性
 *        安装误差、减速比等物理参数统一定义在 robot_def.h
 */
#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include "stdint.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "LK7015.h"
#include "zero_angle.h"

/* 速度档位对应的实际最大速度 (m/s 和 rad/s) */
#define MAX_LINEAR_SPEED_X   0.25f
#define MAX_LINEAR_SPEED_Y   0.25f
#define MAX_ANGULAR_SPEED_Z  0.5f

/* 舵轮对角布局坐标 (m)，由 robot_def.h 的 WHEEL_BASE/TRACK_WIDTH 计算 */
#define STEER_HALF_X_M  ((WHEEL_BASE)  / 2000.0f)
#define STEER_HALF_Y_M  ((TRACK_WIDTH) / 2000.0f)

/* 驱动轮参数 (robot_def.h 的 RADIUS_WHEEL=105mm) */
#define WHEEL_RADIUS_M      ((RADIUS_WHEEL) / 1000.0f)
#define REDUCTION_RATIO_DRIVE REDUCTION_RATIO_WHEEL
#define REDUCTION_RATIO_STEER 8.0f

/* 堵转检测参数 */
#define STALL_CURRENT_THRESHOLD  8.0f   // 堵转电流阈值 (A)
#define STALL_SPEED_THRESHOLD    0.5f   // 堵转速度阈值 (rad/s)
#define STALL_TIME_MS            200    // 堵转持续时间阈值 (ms)

typedef struct {
    DJIMotorInstance  *steer_left;   // 左舵轮转向电机
    DJIMotorInstance  *steer_right;  // 右舵轮转向电机
    LK7015Instance    *drive_left;   // 左驱动轮电机
    LK7015Instance    *drive_right;  // 右驱动轮电机
    ZeroAngleInstance *zero_left;    // 左光电门
    ZeroAngleInstance *zero_right;   // 右光电门
} ChassisMotorContext;
/**
 * @brief 更新零点角度
 *        记录光电门触发时转向电机的编码器累计角度(motor端总角度)作为原始零点。
 *        查零完成后将转向电机 outer_loop 切换为 ANGLE_LOOP，
 *        并设置 other_angle_feedback_ptr 指向物理舵向角度变量。
 *        应在 ZeroAngleProcess 返回校准完成后调用。
 */
void InverseKinematics_UpdateZeroAngle(void);

/**
 * @brief 计算物理舵向角度
 *        从电机编码器累计角度和零点偏移计算真实的物理舵向角(度)，
 *        并写入 steer_now_physical_angle[]，供 OTHER_FEED 角度环使用。
 *        应在 ChassisTask 中每周期调用(200Hz)。
 *
 *        物理舵向角 = (total_angle - zero_total_angle) / REDUCTION_RATIO_STEER - light_gate_error
 */
void InverseKinematics_ComputePhysicalAngle(void);

/**
 * @brief 运动学逆解与电机驱动
 * @param vx_level  X方向线速度档位 [-10, 10] (正为前进)
 * @param vy_level  Y方向线速度档位 [-10, 10] (正为左移)
 * @param wz_level  Z轴角速度档位 [-10, 10] (正为逆时针)
 * @return uint8_t  1: 驱动成功  0: 有电机离线，已自动急停
 * 
 * @note  档位绝对值小于1时视为零输入，此时保持舵向角度不变，驱动电机停转
 */
uint8_t InverseKinematics_Drive(int8_t vx_level, int8_t vy_level, int8_t wz_level);

/* 堵转保护持久标志，堵转后置1，EmergencyStop 清零 */
extern uint8_t stall_persistent[2];

/**
 * @brief 紧急停止
 *        立即停止所有转向电机和驱动电机，并清零堵转持久标志。
 */
void InverseKinematics_EmergencyStop(void);

/**
 * @brief 一分钟稳定性展示
 *        低速自旋与斜向移动交替，用于测试PID参数与系统稳定性。
 *        应在底盘任务中每5ms(200Hz)调用一次，持续60秒后自动停止。
 *        若调用过程中电机离线，会停止展示并返回。
 */
void InverseKinematics_ShowMotion(void);

/**
 * @brief 向逆解模块注入电机实例（必须在 ChassisInit 中先调用一次）
 */
void InverseKinematics_Init(ChassisMotorContext *ctx);

#endif /* INVERSE_KINEMATICS_H */