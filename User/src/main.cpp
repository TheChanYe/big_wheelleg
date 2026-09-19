/**
 * 文件用途：副板程序入口。
 * 所属层级：User。
 * 主要职责：完成系统基础初始化、创建应用任务并启动 FreeRTOS。
 */

#include "main.h"
#include "system_init.h"
#include "app_tasks.h"
#include "i2c2_bus.h"
#include "my_usart.h"


int main(void)
{
    SystemManager system;
    TaskManager task_manager;
    
    if (system.init() != E_OK)
    {
        /* 基础资源创建失败时不能继续启动控制任务。 */
        while (1);
    }

    /* 调试日志共用现有 USART2 DMA 队列。调度器启动后才实际发送。 */
    get_uart_driver()->init();
    log_inform("USART2 serial logger ready, baud=115200");

    I2c2Bus i2c2;
    uint8_t who_am_i = 0u;  

    int probe_result = i2c2.init();
    if (probe_result == E_OK) {
        probe_result = i2c2.readRegister(0x68u, 0x75u, who_am_i);

        // AD0 若为高电平，设备地址可能是 0x69。
        if (probe_result != E_OK) {
            probe_result = i2c2.readRegister(0x69u, 0x75u, who_am_i);
        }
    }

    log_inform("MPU6050 probe: result=%d, WHO_AM_I=0x%02X",
               probe_result, who_am_i);

    if (task_manager.create() != E_OK)
    {
        /* 任务不完整会破坏原有控制时序，因此保持停机。 */
        while (1);
    }

    vTaskStartScheduler();

    while (1);
}
