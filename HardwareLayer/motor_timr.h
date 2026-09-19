#ifndef MOTOR_TIMR_H
#define MOTOR_TIMR_H
#include "foc_cfg.h"

#ifdef __cplusplus
/** 电机 PWM 外设操作；不拥有定时器寄存器或 Motor_Data。 */
class MotorPwm final {
public:
    MotorPwm() = delete;
    static int init(Motor_Type motor);
    static int shut(Motor_Data& motor);
};
extern "C" {
#endif
int FOC_TMR_Init(Motor_Type motor);/*电机PWM定时器初始化*/
int Shut_PWM(Motor_Data* motor);/*关闭电机PWM输出*/
#ifdef __cplusplus
}
#endif
#endif

