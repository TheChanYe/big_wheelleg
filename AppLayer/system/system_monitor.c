/**
 * 文件用途：基础系统状态监控实现。
 * 所属层级：AppLayer/system。
 * 主要职责：保留原有 LED 闪烁规则和两路 MOS 温度采集调用。
 */

#include "system_monitor.h"
#include "switch.h"
#include "foc.h"
#include "motor_adc.h"
#include "motor_fault.h"
#include "safety_limit.h"

extern Motor_Data g_motor1;
extern Motor_Data g_motor2;

#define THERMAL_VALID_SAMPLE_COUNT  3u

static uint8_t g_thermal_sample_count[2] = {0u};

static void SystemMonitor_ThermalProtect(uint8_t motor_id,
                                         Motor_Data *motor,
                                         MotorFaultBits fault_bit)
{
    float limit_a = MOTOR_COMMAND_IQ_LIMIT_A;
    float derate_ratio;

    if (Get_Mos_Temp(motor) != E_OK)
    {
        g_thermal_sample_count[motor_id] = 0u;
        return;
    }

    /* 上电初始 ADC 值可能尚未稳定，先完成少量有效采样。 */
    if (g_thermal_sample_count[motor_id] < THERMAL_VALID_SAMPLE_COUNT)
    {
        g_thermal_sample_count[motor_id]++;
        return;
    }

    if (motor->mos_temp >= MOS_TEMP_FAULT_C)
    {
        MotorFault_Enter(motor_id, motor, fault_bit);
        return;
    }

    if (motor->mos_temp >= MOS_TEMP_DERATE_C)
    {
        derate_ratio = (MOS_TEMP_FAULT_C - motor->mos_temp)
            / (MOS_TEMP_FAULT_C - MOS_TEMP_DERATE_C);
        if (derate_ratio < MOS_TEMP_DERATE_MIN_RATIO)
            derate_ratio = MOS_TEMP_DERATE_MIN_RATIO;
        limit_a *= derate_ratio;
    }

    SafetyLimit_SetIqLimit(motor_id, limit_a);
}

void SystemMonitor_Process(void)
{
    static c_switch led = {0};
    static uint8_t initialized = 0u;

    if (!initialized)
    {
        led = switch_create(GPIOD, GPIO_PINS_2);
        initialized = 1u;
    }

    if (g_motor1.run_state == RUN && g_motor2.run_state == RUN)
        led.flicker(&led, 200);
    else if (g_motor1.run_state == FAULT)
        led.flicker(&led, 500);
    else if (g_motor2.run_state == FAULT)
        led.flicker(&led, 100);

    SystemMonitor_ThermalProtect(0u, &g_motor1, MOTOR0_OVERTEMP);
    SystemMonitor_ThermalProtect(1u, &g_motor2, MOTOR1_OVERTEMP);
    Motor_ADC_UpdateBusVoltage();
}
