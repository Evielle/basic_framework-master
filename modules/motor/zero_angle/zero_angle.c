#include "zero_angle.h"

/* ---------- 参数宏 ---------- */
#define DEBOUNCE_THRESHOLD  5       // 软件消抖确认次数
#define FIND_ZERO_SPEED     1000.0f // 找零点转速 (ref)

/* ---------- 外部函数声明 ---------- */
extern void DJIMotorSetRef(DJIMotorInstance *motor, float ref);
extern void DJIMotorStop(DJIMotorInstance *motor);

/**
 * @brief  初始化光电门零点校准实例
 * @param  GPIO_Pin : 引脚号 (GPIO_PIN_11 / GPIO_PIN_13)
 * @param  motor    : 对应的 DJI 电机实例指针
 * @retval 实例指针，失败返回 NULL
 */
ZeroAngleInstance *ZeroAngleInit(uint16_t GPIO_Pin, DJIMotorInstance *motor)
{
    ZeroAngleInstance *instance = (ZeroAngleInstance *)malloc(sizeof(ZeroAngleInstance));
    if (instance == NULL) return NULL;

    /* 先清零，再赋值，避免脏数据 */
    memset(instance, 0, sizeof(ZeroAngleInstance));

    instance->motor = motor;

    /* 配置 GPIO 并注册 */
    GPIO_Init_Config_s GPIO_config = {
        .GPIO_Pin    = GPIO_Pin,
        .GPIOx       = GPIOE,
        .exti_mode   = GPIO_EXTI_MODE_NONE,
        .id          = NULL,
        .pin_state   = GPIO_PIN_RESET,      // 初始低电平（无遮挡）
        .gpio_model_callback = NULL
    };
    instance->gpio_instance = GPIORegister(&GPIO_config);

    /* 读取当前电平作为初始稳定状态 */
    GPIO_PinState init_level = GPIORead(instance->gpio_instance);
    instance->old_state = init_level;
    instance->new_state = init_level;

    instance->state      = ZEROANGLE_STATE_IDLE;
    instance->zero_angle = 0.0f;

    instance->timeout_cnt   = 0;
    instance->timeout_limit = 600;  // 3秒，可根据实际调整
    instance->direction     = 1;    // 先尝试正向

    return instance;
}

/**
 * @brief  处理两个光电门的找零状态机（周期调用）
 * @param  left  : 左光电门实例 (PE11, motor_rf)
 * @param  right : 右光电门实例 (PE13, motor_lb)
 * @retval 0 : 未全部完成
 *         1 : 两个电机均校准完成
 */
uint8_t ZeroAngleProcess(ZeroAngleInstance *left, ZeroAngleInstance *right)
{
    static uint8_t left_debounce_cnt  = 0;
    static uint8_t right_debounce_cnt = 0;

    /* ========== 处理左光电门 ========== */
    if (left != NULL) {
        switch (left->state) {
            case ZEROANGLE_STATE_IDLE:
                /* 启动找零：速度环，正向低速旋转 */
                DJIMotorOuterLoop(left->motor, SPEED_LOOP);
                DJIMotorSetRef(left->motor, FIND_ZERO_SPEED * left->direction);
                left->state = ZEROANGLE_STATE_RUNNING;
                left_debounce_cnt = 0;
                left->timeout_cnt  = 0;
                break;

            case ZEROANGLE_STATE_RUNNING:
            {
                uint8_t cur = (GPIORead(left->gpio_instance) == GPIO_PIN_SET) ? 1 : 0;

                /* 软件消抖：逻辑与原版一致 */
                if (cur == left->new_state) {
                    if (left_debounce_cnt < DEBOUNCE_THRESHOLD) {
                        left_debounce_cnt++;
                    }
                } else {
                    left->new_state = cur;
                    left_debounce_cnt = 0;
                }

                /* 消抖达到阈值，状态变化确认 */
                if (left_debounce_cnt >= DEBOUNCE_THRESHOLD) {
                    if (left->new_state != left->old_state) {
                        /* 边沿发生 -> 记录当前位置为机械零点 */
                        left->zero_angle = left->motor->measure.total_angle;
                        DJIMotorStop(left->motor);
                        left->state = ZEROANGLE_STATE_DONE;
                        left->old_state = left->new_state;
                        left_debounce_cnt = 0;
                        break;   // 完成，跳出当前 case
                    }
                    left_debounce_cnt = 0;
                }

                /* 超时换向（只有仍在 RUNNING 且未完成时才执行） */
                left->timeout_cnt++;
                if (left->timeout_cnt >= left->timeout_limit) {
                    left->timeout_cnt = 0;
                    left->direction *= -1;          // 反转方向

                    if (left->direction == 1) {
                        /* 正反两个方向都超时 -> 报错退出 */
                        DJIMotorStop(left->motor);
                        left->state = ZEROANGLE_STATE_ERROR;
                    } else {
                        /* 第一次超时，反转方向继续找 */
                        DJIMotorSetRef(left->motor, FIND_ZERO_SPEED * left->direction);
                    }
                }
                break;
            }

            case ZEROANGLE_STATE_DONE:
            case ZEROANGLE_STATE_ERROR:
            default:
                break;
        }
    }

    /* ========== 处理右光电门（逻辑完全对称） ========== */
    if (right != NULL) {
        switch (right->state) {
            case ZEROANGLE_STATE_IDLE:
                DJIMotorOuterLoop(right->motor, SPEED_LOOP);
                DJIMotorSetRef(right->motor, FIND_ZERO_SPEED * right->direction);
                right->state = ZEROANGLE_STATE_RUNNING;
                right_debounce_cnt = 0;
                right->timeout_cnt  = 0;
                break;

            case ZEROANGLE_STATE_RUNNING:
            {
                uint8_t cur = (GPIORead(right->gpio_instance) == GPIO_PIN_SET) ? 1 : 0;

                if (cur == right->new_state) {
                    if (right_debounce_cnt < DEBOUNCE_THRESHOLD) {
                        right_debounce_cnt++;
                    }
                } else {
                    right->new_state = cur;
                    right_debounce_cnt = 0;
                }

                if (right_debounce_cnt >= DEBOUNCE_THRESHOLD) {
                    if (right->new_state != right->old_state) {
                        right->zero_angle = right->motor->measure.total_angle;
                        DJIMotorStop(right->motor);
                        right->state = ZEROANGLE_STATE_DONE;
                        right->old_state = right->new_state;
                        right_debounce_cnt = 0;
                        break;
                    }
                    right_debounce_cnt = 0;
                }

                right->timeout_cnt++;
                if (right->timeout_cnt >= right->timeout_limit) {
                    right->timeout_cnt = 0;
                    right->direction *= -1;

                    if (right->direction == 1) {
                        DJIMotorStop(right->motor);
                        right->state = ZEROANGLE_STATE_ERROR;
                    } else {
                        DJIMotorSetRef(right->motor, FIND_ZERO_SPEED * right->direction);
                    }
                }
                break;
            }

            case ZEROANGLE_STATE_DONE:
            case ZEROANGLE_STATE_ERROR:
            default:
                break;
        }
    }

    /* 返回 1 表示两个舵机都成功找到零点 */
    if (left  != NULL && left->state  == ZEROANGLE_STATE_DONE &&
        right != NULL && right->state == ZEROANGLE_STATE_DONE) {
        return 1;
    }
    return 0;
}