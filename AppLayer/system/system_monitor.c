/**
 * 文件用途：基础系统状态监控实现。
 * 所属层级：AppLayer/system。
 * 主要职责：保留原有 LED 闪烁规则和两路 MOS 温度采集调用。
 */

#include "system_monitor.h"
#include "switch.h"
#include "foc.h"
#include "motor_adc.h"

extern Motor_Data g_motor1;
extern Motor_Data g_motor2;

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

    Get_Mos_Temp(&g_motor1);
    Get_Mos_Temp(&g_motor2);
}
