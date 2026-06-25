/**
 * @file InverseKinematics.c
 * @brief 双舵轮底盘运动学逆解实现（对角布局）
 * 
 * @detail 运动学模型：
 *         两组舵轮在底盘对角线上，第i个舵轮位置 (xi, yi)
 *         左舵轮: ( +STEER_HALF_X_M,  +STEER_HALF_Y_M )
 *         右舵轮: ( -STEER_HALF_X_M,  -STEER_HALF_Y_M )
 *         刚体平面运动: (Vx - yi*Wz, Vy + xi*Wz)
 *         解出：delta_i = atan2(Vy + xi*Wz, Vx - yi*Wz)
 *               vi = sqrt((Vx - yi*Wz)^2 + (Vy + xi*Wz)^2)
 *         若 |delta_i| > PI/2, 则反转驱动并调整 delta_i 至 [-PI/2, PI/2]
 *         驱动电机目标角速度 = vi / WHEEL_RADIUS_M * REDUCTION_RATIO_DRIVE
 *         舵向电机目标总角度 = 零位电机角度 + delta_i(度) * REDUCTION_RATIO_STEER
 */
#include "InverseKinematics.h"
#include "dji_motor.h"
#include "LK7015.h"
#include "zero_angle.h"
#include "daemon.h"
#include "math.h"
#include "stddef.h"
#include "bsp_log.h"
#include "general_def.h"

/* ---------- 外部电机实例 (在 application 层定义) ---------- */
// extern DJIMotorInstance *motor_l;      // 左舵轮转向电机 (ID1, 第一组, +X)
// extern DJIMotorInstance *motor_r;      // 右舵轮转向电机 (ID2, 第二组, -X)
// extern LK7015Instance  *motor_l_lk;   // 左驱动轮电机 (ID1)
// extern LK7015Instance  *motor_r_lk;   // 右驱动轮电机 (ID2)

/* ---------- 光电门实例 (在 zero_angle 模块初始化) ---------- */
// extern ZeroAngleInstance *zero_angle_l;
// extern ZeroAngleInstance *zero_angle_r;

/* ---------- 模块内部静态变量 ---------- */
static float steer_zero_total_angle[2] = {0.0f, 0.0f};  // [左,右] 光电门触发时电机端累计角度(度)
static float steer_now_physical_angle[2] = {0.0f, 0.0f}; // [左,右] 物理舵向角度(度)，供 OTHER_FEED 角度环反馈
static uint8_t zero_calibrated = 0;                       // 零点是否已校准

/* 安装误差，物理角度(度) */
static const float light_gate_error[2] = {LEFT_STEER_ERROR, RIGHT_STEER_ERROR};

/* 堵转保护持久标志 */
uint8_t stall_persistent[2] = {0, 0};

/* 展示状态控制 */
static float show_timer = 0.0f;        // 秒
static uint8_t show_active = 0;       // 展示是否激活

/* ---------- 函数实现 ---------- */


static ChassisMotorContext *motor_ctx = NULL;

void InverseKinematics_Init(ChassisMotorContext *ctx)
{
    motor_ctx = ctx;
}

/**
 * @brief 停止所有电机 (内部使用)
 */
static void StopAllMotors(void)
{
    if (motor_ctx == NULL) return;
    DJIMotorStop(motor_ctx->steer_left);
    DJIMotorStop(motor_ctx->steer_right);
    LK7015Stop(motor_ctx->drive_left);
    LK7015Stop(motor_ctx->drive_right);
}

/**
 * @brief 计算并更新零点偏移角度
 */
void InverseKinematics_UpdateZeroAngle(void)
{
    if (motor_ctx == NULL) return;

    if (motor_ctx->zero_left == NULL || motor_ctx->zero_right == NULL) return;
    if (motor_ctx->zero_left->state != ZEROANGLE_STATE_DONE ||
        motor_ctx->zero_right->state != ZEROANGLE_STATE_DONE) {
        return;
    }

    steer_zero_total_angle[0] = motor_ctx->zero_left->zero_angle;
    steer_zero_total_angle[1] = motor_ctx->zero_right->zero_angle;

    zero_calibrated = 1;

    DJIMotorOuterLoop(motor_ctx->steer_left, ANGLE_LOOP);
    DJIMotorOuterLoop(motor_ctx->steer_right, ANGLE_LOOP);
    DJIMotorEnable(motor_ctx->steer_left);
    DJIMotorEnable(motor_ctx->steer_right);

    motor_ctx->steer_left->motor_controller.other_angle_feedback_ptr = &steer_now_physical_angle[0];
    motor_ctx->steer_right->motor_controller.other_angle_feedback_ptr = &steer_now_physical_angle[1];
}

/**
 * @brief 计算物理舵向角度，写入 steer_now_physical_angle[]
 */
void InverseKinematics_ComputePhysicalAngle(void)
{
    if (motor_ctx == NULL) return;
    if (!zero_calibrated) return;

    static float prev_total_angle[2] = {0.0f, 0.0f};
    static uint8_t first_run = 1;
    static uint8_t jump_count[2] = {0, 0};

    if (first_run) {
        prev_total_angle[0] = motor_ctx->steer_left->measure.total_angle;
        prev_total_angle[1] = motor_ctx->steer_right->measure.total_angle;
        first_run = 0;
    }

    float delta_angle[2];
    delta_angle[0] = fabsf(motor_ctx->steer_left->measure.total_angle - prev_total_angle[0]);
    delta_angle[1] = fabsf(motor_ctx->steer_right->measure.total_angle - prev_total_angle[1]);

    for (int i = 0; i < 2; i++) {
        if (delta_angle[i] > 180.0f) {
            jump_count[i]++;
            if (jump_count[i] > 3) {
                LOGERROR("[IK] Encoder persistent jump on steer motor %d, delta=%.2f deg, stopping!", i, delta_angle[i]);
                StopAllMotors();
            } else {
                LOGWARNING("[IK] Encoder jump detected on steer motor %d, delta=%.2f deg (count=%d)", i, delta_angle[i], jump_count[i]);
            }
            // 不更新 prev_total_angle，保持上一周期值
            return;
        }
    }

    /* 正常时清零跳变计数器，更新 prev_total_angle 和物理角度 */
    jump_count[0] = 0;
    jump_count[1] = 0;

    prev_total_angle[0] = motor_ctx->steer_left->measure.total_angle;
    prev_total_angle[1] = motor_ctx->steer_right->measure.total_angle;

    steer_now_physical_angle[0] = (motor_ctx->steer_left->measure.total_angle - steer_zero_total_angle[0])
                                  / REDUCTION_RATIO_STEER - light_gate_error[0];
    steer_now_physical_angle[1] = (motor_ctx->steer_right->measure.total_angle - steer_zero_total_angle[1])
                                  / REDUCTION_RATIO_STEER - light_gate_error[1];
}

/**
 * @brief 将档位值转换为实际速度
 */
static void LevelToSpeed(int8_t vx_level, int8_t vy_level, int8_t wz_level,
                         float *vx, float *vy, float *wz)
{
    /* 限制档位在[-10,10] */
    if (vx_level > 10) vx_level = 10;
    if (vx_level < -10) vx_level = -10;
    if (vy_level > 10) vy_level = 10;
    if (vy_level < -10) vy_level = -10;
    if (wz_level > 10) wz_level = 10;
    if (wz_level < -10) wz_level = -10;

    *vx = (float)vx_level / 10.0f * MAX_LINEAR_SPEED_X;
    *vy = (float)vy_level / 10.0f * MAX_LINEAR_SPEED_Y;
    *wz = (float)wz_level / 10.0f * MAX_ANGULAR_SPEED_Z;
}

/**
 * @brief 运动学逆解与驱动
 */
uint8_t InverseKinematics_Drive(int8_t vx_level, int8_t vy_level, int8_t wz_level)
{
    /* 0. 零点未校准则无法保证方向，返回失败 */
    if (!zero_calibrated) return 0;

    /* 1. 在线检测（调试期间暂时关闭） */
    // if (!AllMotorsOnline()) {
    //     StopAllMotors();
    //     return 0;
    // }

    /* 确保电机全部处于使能状态（堵转保护的电机除外） */
    DJIMotorEnable(motor_ctx->steer_left);
    DJIMotorEnable(motor_ctx->steer_right);
    if (!stall_persistent[0]) LK7015Enable(motor_ctx->drive_left);
    if (!stall_persistent[1]) LK7015Enable(motor_ctx->drive_right);

    /* 2. 档位转实际速度 */
    float Vx, Vy, Wz;
    LevelToSpeed(vx_level, vy_level, wz_level, &Vx, &Vy, &Wz);

    LK7015Instance *drive_motor[2] = {motor_ctx->drive_left, motor_ctx->drive_right};

    /* 3. 微小输入直接停止驱动，保持舵向 */
    if (fabsf(Vx) < 0.001f && fabsf(Vy) < 0.001f && fabsf(Wz) < 0.001f) {
        LK7015SetRef(motor_ctx->drive_left, 0.0f);
        LK7015SetRef(motor_ctx->drive_right, 0.0f);
        return 1;
    }

    /* 4. 计算两个舵轮的目标 (对角布局) */
    float x[2] = {  STEER_HALF_X_M, -STEER_HALF_X_M };
    float y[2] = {  STEER_HALF_Y_M, -STEER_HALF_Y_M };
    float delta_rad[2], v_mps[2];
    DJIMotorInstance *steer_motor[2] = {motor_ctx->steer_left, motor_ctx->steer_right};

    for (int i = 0; i < 2; i++) {
        float vx_eff = Vx - y[i] * Wz;
        float vy_eff = Vy + x[i] * Wz;
        delta_rad[i] = atan2f(-vy_eff, vx_eff);
        v_mps[i] = sqrtf(vx_eff * vx_eff + vy_eff * vy_eff);

        /* 保持转向角在 [-PI/2, PI/2] 范围内，必要时反转驱动速度 */
        if (fabsf(delta_rad[i]) > (PI / 2.0f)) {
            if (delta_rad[i] > 0.0f)
                delta_rad[i] -= PI;
            else
                delta_rad[i] += PI;
            v_mps[i] = -v_mps[i];
        }
    }

    /* 5. 设置电机指令 */
    for (int i = 0; i < 2; i++) {
        /* 驱动电机: 线速度 -> 电机角速度 (rad/s) */
        float motor_omega = v_mps[i] / WHEEL_RADIUS_M * REDUCTION_RATIO_DRIVE;
        LK7015SetRef(drive_motor[i], motor_omega);

        /* 转向电机: 目标物理角(度) → 直接作为位置环参考值 */
        float target_deg = delta_rad[i] * RAD_2_DEGREE;
        /* 软件限位: 舵向角 [-90°, 90°] */
        if (target_deg > 90.0f) {
            target_deg = 90.0f;
        } else if (target_deg < -90.0f) {
            target_deg = -90.0f;
        }
        DJIMotorSetRef(steer_motor[i], target_deg);
    }

    return 1;
}
