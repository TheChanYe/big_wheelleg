#ifndef MOTOR_TIMR_H
#define MOTOR_TIMR_H
#include "foc_cfg.h"

int FOC_TMR_Init(Motor_Type motor);/*电机PWM定时器初始化*/
int Shut_PWM(Motor_Data* motor);/*关闭电机PWM输出*/
#endif

