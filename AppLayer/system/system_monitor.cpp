/**
 * 文件用途：基础系统状态监控实现。
 * 所属层级：AppLayer/system。
 * 主要职责：保留原有 LED 闪烁规则和两路 MOS 温度采集调用。
 */

#include "system_monitor.h"
#include "motor_adc.h"
#include "safety_limit.h"

#define THERMAL_VALID_SAMPLE_COUNT 3u
#define THERMAL_ADC_SETTLE_MS      1000u

/*
 * 两个电机对象仍然定义在 C 文件 foc.c 中，
 * 因此这里必须使用 C linkage。
 */
extern "C" {
extern Motor_Data g_motor1;
extern Motor_Data g_motor2;

int Get_Mos_Temp(Motor_Data *motor);
}

SystemMonitor::SystemMonitor()
    : led_{},
      initialized_(false),
      thermal_sample_count_{0u, 0u},
      temp_invalid_count_{0u, 0u},
      temp_fault_count_{0u, 0u},
      thermal_start_tick_(0)
{

}

SystemMonitor::~SystemMonitor()
{
    /*
     * switch 模块当前没有公开销毁接口。
     * SystemMonitor 生命周期覆盖整个程序运行期，因此不在此释放。
     */
}

/**
 * 功能：初始化系统监控模块。
 * 参数：无。
 * 返回值：无。
 */
void SystemMonitor::initialize()
{
    led_ = switch_create(GPIOD, GPIO_PINS_2);
    thermal_start_tick_ = xTaskGetTickCount();
    initialized_ = true;
}

/**
 * 功能：根据电机运行状态更新 LED 闪烁。
 * 参数：无。
 * 返回值：无。
 */
void SystemMonitor::updateLed() {
    if (g_motor1.run_state == RUN
        && g_motor2.run_state == RUN)
    {
        led_.flicker(&led_, 200);
    }
    else if (g_motor1.run_state == FAULT)
    {
        led_.flicker(&led_, 500);
    }
    else if (g_motor2.run_state == FAULT)
    {
        led_.flicker(&led_, 100);
    }
}
/**
 * 功能：对单个电机进行热保护。
 * 参数：
 *   motor_id    - 电机编号。
 *   motor      - 电机数据指针。
 *   fault_bit  - 对应的故障位。
 * 返回值：无。
 */
void SystemMonitor::thermalProtect(
    uint8_t motor_id,
    Motor_Data *motor,
    MotorFaultBits fault_bit)
{
    float limit_a = MOTOR_COMMAND_IQ_LIMIT_A;
    float derate_ratio;

    if (Get_Mos_Temp(motor) != E_OK)
    {
        thermal_sample_count_[motor_id] = 0u;

        if ((xTaskGetTickCount() - thermal_start_tick_)
            < pdMS_TO_TICKS(THERMAL_ADC_SETTLE_MS))
        {
            return;
        }

        if (++temp_invalid_count_[motor_id]
            >= TEMP_SENSOR_FAULT_CONFIRM_COUNT)
        {
            MotorFault_Enter(
                motor_id,
                motor,
                (motor_id == 0u)
                    ? MOTOR0_TEMP_SENSOR_FAULT
                    : MOTOR1_TEMP_SENSOR_FAULT);
        }
        return;
    }

    temp_invalid_count_[motor_id] = 0u;
    
    if (thermal_sample_count_[motor_id]
        < THERMAL_VALID_SAMPLE_COUNT)
    {
        ++thermal_sample_count_[motor_id];
        return;
    }
    
    if (motor->mos_temp >= MOS_TEMP_FAULT_C)
    {
        if (++temp_fault_count_[motor_id]
            >= MOS_TEMP_FAULT_CONFIRM_COUNT)
        {
            MotorFault_Enter(motor_id, motor, fault_bit);
        }

        return;
    }

    temp_fault_count_[motor_id] = 0u;

    if (motor->mos_temp >= MOS_TEMP_DERATE_C)
    {
        derate_ratio = 
            (MOS_TEMP_FAULT_C - motor->mos_temp)
            / (MOS_TEMP_FAULT_C - MOS_TEMP_DERATE_C);
        
        if (derate_ratio < MOS_TEMP_DERATE_MIN_RATIO)
        {
            derate_ratio = MOS_TEMP_DERATE_MIN_RATIO;
        }

        limit_a *= derate_ratio;
    }

    SafetyLimit_SetIqLimit(motor_id, limit_a);
}

/**
 * 功能：系统监控的主处理函数。
 * 参数：无。
 * 返回值：无。
 */
void SystemMonitor::process()
{
    if (!initialized_) {
        initialize();
    }

    updateLed();

    thermalProtect(
        0u,
        &g_motor1,
        MOTOR0_OVERTEMP
    );

    thermalProtect(
        1u,
        &g_motor2,
        MOTOR1_OVERTEMP
    );

    Motor_ADC_UpdateBusVoltage();
}