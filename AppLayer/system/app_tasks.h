/**
 * 文件用途：应用任务统一管理接口。
 * 所属层级：AppLayer/system。
 * 主要职责：公开 ADC 中断需要使用的电机任务句柄，并提供任务创建入口。
 */

#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ADC ISR 使用这两个句柄通知对应电机任务，不能定义为 static。 */
extern TaskHandle_t g_motor0_task_handle;
extern TaskHandle_t g_motor1_task_handle;

/** 功能：创建已启用的应用任务；参数：无；返回值：E_OK 或 E_ERROR。 */
int AppTasks_Create(void);

#ifdef __cplusplus
}
#endif

#endif
