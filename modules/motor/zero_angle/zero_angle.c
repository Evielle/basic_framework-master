#include "zero_angle.h"
#include <string.h>
#include <stdlib.h>
#include "bsp_gpio.h"

/* ----- 内部变量 ----- */
static ZeroConfig_t   z_cfg;
static volatile uint8_t gate_triggered[ZERO_MOTOR_NUM] = {0}; // 0=未触发，1=已触发
static uint32_t start_tick = 0;
static float    offset_angle[ZERO_MOTOR_NUM] = {0.0f};        // 每个电机的零位偏移
ZeroState_t zero_state = ZERO_IDLE;

static GPIOInstance zero_gate[ZERO_MOTOR_NUM];                // 两个光电门注册实例
static void ZeroGateCallback(GPIOInstance *gpio)
{
    // 直接调用我们原有的处理逻辑，传入对应引脚
    ZeroEXTICallback(gpio->GPIO_Pin);
}
/* ----- 内部辅助 ----- */
/* ----- 电机单独控制 ----- */
static void MotorMove(DJIMotorInstance *motor, float speed_dps)
{
    if (motor) {
        DJIMotorSetRef(motor, speed_dps);
    }
}
static void MotorStop(DJIMotorInstance *motor)
{
    if (motor) {
        DJIMotorSetRef(motor, 0.0f);
    }
}

/* ----- 公共接口 ----- */
void ZeroInit(ZeroConfig_t *config)
{
    if (config == NULL) return;
    memcpy(&z_cfg, config, sizeof(ZeroConfig_t));

    // 清除状态
    for (int i = 0; i < ZERO_MOTOR_NUM; i++) {
        gate_triggered[i] = 0;
        offset_angle[i] = 0.0f;
    }
    zero_state = ZERO_IDLE;

    // 注册两个光电门
    for (int i = 0; i < ZERO_MOTOR_NUM; i++) {
        zero_gate[i].GPIO_Pin = z_cfg.motor_cfg[i].gate_pin;
        zero_gate[i].gpio_model_callback = ZeroGateCallback;
        GPIO_Register(&zero_gate[i]);
    }
}

void ZeroStart(void)
{
    if (zero_state == ZERO_RUNNING) return;   // 已在运行
    if (z_cfg.motor_cfg[0].motor == NULL ||
        z_cfg.motor_cfg[1].motor == NULL) {
        zero_state = ZERO_ERROR;
        return;
    }

    // 清除触发标志和偏移
    for (int i = 0; i < ZERO_MOTOR_NUM; i++) {
        gate_triggered[i] = 0;
        offset_angle[i] = 0.0f;
    }
    start_tick = HAL_GetTick();
    zero_state = ZERO_RUNNING;

    // 让两个电机同时低速旋转
    for (int i = 0; i < ZERO_MOTOR_NUM; i++) {
        MotorMove(z_cfg.motor_cfg[i].motor, z_cfg.cali_speed);
    }
}

void ZeroStop(void)
{
    if (zero_state == ZERO_RUNNING) {
        for (int i = 0; i < ZERO_MOTOR_NUM; i++) {
            MotorStop(z_cfg.motor_cfg[i].motor);
        }
    }
    zero_state = ZERO_IDLE;
}

void ZeroProcess(void)
{
    if (zero_state != ZERO_RUNNING) return;

    uint8_t all_triggered = 1;
    for (int i = 0; i < ZERO_MOTOR_NUM; i++) {
        if (gate_triggered[i] == 0) {
            all_triggered = 0;
            // 如果该电机尚未触发，但其光电门已触发（由中断置1），则记录偏移并停转该电机
            // 注意：标志由中断设置，这里检测并处理
            // 但因为检查时中断可能还没到来，所以这里只是检查标志
            // 实际上需要先检测标志，如果标志为1，则执行以下操作
            // 但是这里要在第一次检测到标志时执行一次记录和停止，因此需要双重检查。
        }
    }

    // 处理每个电机：当 gate_triggered[i] 从 0 变为 1 时，记录偏移并停止该电机
    // 这里使用简单的轮询方式，标志由中断置1，我们在这里进行“消费”
    for (int i = 0; i < ZERO_MOTOR_NUM; i++) {
        if (gate_triggered[i]) {
            // 只处理一次：如果偏移尚未记录，则记录
            if (offset_angle[i] == 0.0f && gate_triggered[i]) {  // 假设偏移不可能刚好为0.0（电机总会转），可加个额外标志
                offset_angle[i] = z_cfg.motor_cfg[i].motor->measure.total_angle;
                MotorStop(z_cfg.motor_cfg[i].motor);
            }
        }
    }

    // 全部触发成功
    if (all_triggered) {
        zero_state = ZERO_DONE;
        return;
    }

    // 超时检查
    if ((HAL_GetTick() - start_tick) >= z_cfg.timeout_ms) {
        // 停止所有电机
        for (int i = 0; i < ZERO_MOTOR_NUM; i++) {
            MotorStop(z_cfg.motor_cfg[i].motor);
            if (!gate_triggered[i]) {
                offset_angle[i] = 0.0f;  // 未触发的不记录
            }
        }
        zero_state = ZERO_TIMEOUT;
    }
}

float ZeroGetOffset(DJIMotorInstance *motor)
{
    if (zero_state == ZERO_DONE) {
        for (int i = 0; i < ZERO_MOTOR_NUM; i++) {
            if (z_cfg.motor_cfg[i].motor == motor) {
                return offset_angle[i];
            }
        }
    }
    return 0.0f;   // 未完成或电机未找到
}

/* 中断回调（由 GPIO 注册机制调用） */
void ZeroEXTICallback(uint16_t GPIO_Pin)
{
    if (zero_state != ZERO_RUNNING) return;

    for (int i = 0; i < ZERO_MOTOR_NUM; i++) {
        if (GPIO_Pin == z_cfg.motor_cfg[i].gate_pin) {
            gate_triggered[i] = 1;      // 置位触发标志，具体处理在 ZeroProcess 中完成
            break;
        }
    }
}