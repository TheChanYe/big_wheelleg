/**
 * 文件用途：副板程序入口。
 * 所属层级：User。
 * 主要职责：完成系统基础初始化、创建应用任务并启动 FreeRTOS。
 */

#include "main.h"
#include "system_init.h"
#include "app_tasks.h"

int main(void)
{
    if (System_Init() != E_OK)
    {
        /* 基础资源创建失败时不能继续启动控制任务。 */
        while (1);
    }

    if (AppTasks_Create() != E_OK)
    {
        /* 任务不完整会破坏原有控制时序，因此保持停机。 */
        while (1);
    }

    vTaskStartScheduler();

    while (1);
}
