#ifndef MOTOR_ADC_H
#define MOTOR_ADC_H
#include "foc_cfg.h"

#ifdef __cplusplus
/** ADC 初始化与母线电压读取；中断入口保持独立 C ABI。 */
class MotorAdc final {
public:
    MotorAdc() = delete;
    static int init(Motor_Type motor);
    static int updateBusVoltage();
    static float busVoltage();
    static uint8_t busVoltageValid();
};
extern "C" {
#endif

float queryTemp(float r);
int FOC_ADC_Init(Motor_Type motor);
int Motor_ADC_UpdateBusVoltage(void);
float Motor_ADC_GetBusVoltage(void);
uint8_t Motor_ADC_BusVoltageValid(void);

#ifdef __cplusplus
}
#endif

#endif

