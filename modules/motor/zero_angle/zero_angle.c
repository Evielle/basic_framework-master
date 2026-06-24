#include "zero_angle.h"

#define DEBOUNCE_THRESHOLD  5
#define FIND_ZERO_SPEED     1000.0f

extern void DJIMotorSetRef(DJIMotorInstance *motor, float ref);
extern void DJIMotorStop(DJIMotorInstance *motor);

ZeroAngleInstance *ZeroAngleInit(uint16_t GPIO_Pin, DJIMotorInstance *motor)
{
    ZeroAngleInstance *inst = (ZeroAngleInstance *)malloc(sizeof(ZeroAngleInstance));
    if (inst == NULL) return NULL;

    memset(inst, 0, sizeof(ZeroAngleInstance));
    inst->motor = motor;

    GPIO_Init_Config_s gpio_cfg = {
        .GPIO_Pin    = GPIO_Pin,
        .GPIOx       = GPIOE,
        .exti_mode   = GPIO_EXTI_MODE_NONE,
        .id          = NULL,
        .pin_state   = GPIO_PIN_RESET,
        .gpio_model_callback = NULL,
    };
    inst->gpio_instance = GPIORegister(&gpio_cfg);

    GPIO_PinState init = GPIORead(inst->gpio_instance);
    inst->old_state = init;
    inst->new_state = init;

    inst->state       = ZEROANGLE_STATE_IDLE;
    inst->zero_angle  = 0.0f;
    inst->debounce_cnt = 0;

    return inst;
}

static void ZeroAngleProcessSingle(ZeroAngleInstance *inst)
{
    switch (inst->state) {
    case ZEROANGLE_STATE_IDLE:
        DJIMotorOuterLoop(inst->motor, SPEED_LOOP);
        DJIMotorSetRef(inst->motor, FIND_ZERO_SPEED);
        inst->state = ZEROANGLE_STATE_RUNNING;
        inst->debounce_cnt = 0;
        break;

    case ZEROANGLE_STATE_RUNNING: {
        uint8_t cur = (GPIORead(inst->gpio_instance) == GPIO_PIN_SET) ? 1 : 0;

        if (cur == inst->new_state) {
            if (inst->debounce_cnt < DEBOUNCE_THRESHOLD)
                inst->debounce_cnt++;
        } else {
            inst->new_state = cur;
            inst->debounce_cnt = 0;
        }

        if (inst->debounce_cnt >= DEBOUNCE_THRESHOLD &&
            inst->new_state != inst->old_state) {
            inst->zero_angle = inst->motor->measure.total_angle;
            DJIMotorSetRef(inst->motor, 0.0f);
            inst->state = ZEROANGLE_STATE_DONE;
            inst->old_state = inst->new_state;
            inst->debounce_cnt = 0;
        }
        break;
    }

    default:
        break;
    }
}

uint8_t ZeroAngleProcess(ZeroAngleInstance *left, ZeroAngleInstance *right)
{
    if (left)  ZeroAngleProcessSingle(left);
    if (right) ZeroAngleProcessSingle(right);

    return (left && left->state == ZEROANGLE_STATE_DONE &&
            right && right->state == ZEROANGLE_STATE_DONE) ? 1 : 0;
}
