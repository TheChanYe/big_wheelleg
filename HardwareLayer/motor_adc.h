#ifndef MOTOR_ADC_H
#define MOTOR_ADC_H
#include "foc_cfg.h"

float queryTemp(float r);

int FOC_ADC_Init(Motor_Type motor);
int Motor_ADC_UpdateBusVoltage(void);
float Motor_ADC_GetBusVoltage(void);
uint8_t Motor_ADC_BusVoltageValid(void);
#endif

