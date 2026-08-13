/**
 * 文件用途：FreeRTOS 应用任务统一管理。
 * 所属层级：AppLayer/system。
 * 主要职责：创建任务并维护调度入口，不保存 CAN、UART 或 FOC 的业务实现。
 */

#include "app_tasks.h"
#include "task_config.h"
#include "motor_service.h"
#include "can_business.h"
#include "uart_business.h"
#include "system_monitor.h"
#include "drv_fault.h"

#define WATCHDOG_MOTOR0_ALIVE  0x01u
#define WATCHDOG_MOTOR1_ALIVE  0x02u
#define WATCHDOG_CAN_ALIVE     0x04u
#define WATCHDOG_STATUS_ALIVE  0x08u
#define WATCHDOG_REQUIRED      (WATCHDOG_MOTOR0_ALIVE | WATCHDOG_MOTOR1_ALIVE \
                                | WATCHDOG_CAN_ALIVE | WATCHDOG_STATUS_ALIVE)

static volatile uint8_t g_watchdog_ready = 0u;
static volatile uint8_t g_watchdog_alive = 0u;
static uint8_t g_watchdog_enabled = 0u;

static void Watchdog_MarkAlive(uint8_t bit, uint8_t ready)
{
    if (ready)
        g_watchdog_ready |= bit;
    g_watchdog_alive |= bit;
}

static void Watchdog_Process(void)
{
    if (!g_watchdog_enabled)
    {
        if (g_watchdog_ready != WATCHDOG_REQUIRED)
            return;

        wdt_register_write_enable(TRUE);
        wdt_divider_set(WDT_CLK_DIV_256);
        wdt_reload_value_set(625u); /* LICK 40kHz 下约 4 秒。 */
        while (wdt_flag_get(WDT_DIVF_UPDATE_FLAG | WDT_RLDF_UPDATE_FLAG));
        wdt_enable();
        g_watchdog_enabled = 1u;
    }

    if ((g_watchdog_alive & WATCHDOG_REQUIRED) == WATCHDOG_REQUIRED)
    {
        wdt_counter_reload();
        g_watchdog_alive = 0u;
    }
}

TaskHandle_t g_motor0_task_handle = NULL;
TaskHandle_t g_motor1_task_handle = NULL;
static TaskHandle_t g_uart_task_handle = NULL;
static TaskHandle_t g_status_task_handle = NULL;
static TaskHandle_t g_can_task_handle = NULL;
static TaskHandle_t g_can_test_task_handle = NULL;
static TaskHandle_t g_can_loopback_task_handle = NULL;

static void MotorTask(void *argument)
{
    uint8_t motor_id = (uint8_t)(uintptr_t)argument;

    if (MotorService_Init(motor_id) != E_OK)
    {
        log_error("Motor Init Faild!");
        vTaskDelete(NULL);
    }
    Watchdog_MarkAlive((motor_id == 0u) ? WATCHDOG_MOTOR0_ALIVE
                                        : WATCHDOG_MOTOR1_ALIVE, 1u);

    while (1)
    {
        uint32_t notifications;

        /* 控制环仍严格由 ADC 中断通知驱动。 */
        notifications = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        MotorService_RecordControlCycle(motor_id, notifications);
        MotorService_Run(motor_id);
        Watchdog_MarkAlive((motor_id == 0u) ? WATCHDOG_MOTOR0_ALIVE
                                            : WATCHDOG_MOTOR1_ALIVE, 0u);
    }
}

static void CanBusinessTask(void *argument)
{
    (void)argument;
    if (can_business_init() != E_OK)
    {
        log_error("CAN business: my_can_init failed");
        vTaskDelete(NULL);
    }
    Watchdog_MarkAlive(WATCHDOG_CAN_ALIVE, 1u);
    while (1)
    {
        can_business_process();
        DrvFault_Process();
        Watchdog_MarkAlive(WATCHDOG_CAN_ALIVE, 0u);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void UartBusinessTask(void *argument)
{
    (void)argument;
    uart_business_init();
    while (1)
    {
        uart_business_process();
        vTaskDelay(1);
    }
}

static void CanTxTestTask(void *argument)
{
    (void)argument;
    if (can_business_init() != E_OK)
        vTaskDelete(NULL);
    while (1)
    {
        can_business_tx_test_process();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void CanLoopbackTestTask(void *argument)
{
    (void)argument;
    if (can_business_init() != E_OK)
        vTaskDelete(NULL);
    while (1)
    {
        can_business_loopback_test_process();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void StatusTask(void *argument)
{
    (void)argument;
    while (1)
    {
        SystemMonitor_Process();
        Watchdog_MarkAlive(WATCHDOG_STATUS_ALIVE, 1u);
        Watchdog_Process();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static int AppTasks_CreateOne(TaskFunction_t function, const char *name,
                              uint16_t stack_size, UBaseType_t priority,
                              void *argument, TaskHandle_t *handle)
{
    return (xTaskCreate(function, name, stack_size, argument, priority, handle) == pdPASS)
        ? E_OK : E_ERROR;
}

int AppTasks_Create(void)
{
    int ret = E_OK;

    taskENTER_CRITICAL();
#if APP_TASK_UART_ENABLE
    ret |= AppTasks_CreateOne(UartBusinessTask, "USART_Send_Task", USART_SEND_TASK_STACK_SIZE,
                              USART_SEND_TASK_PRIORITY, NULL, &g_uart_task_handle);
#endif
#if APP_TASK_CAN_ENABLE
    ret |= AppTasks_CreateOne(CanBusinessTask, "CAN_Business", CAN_BUSINESS_TASK_STACK_SIZE,
                              CAN_BUSINESS_TASK_PRIORITY, NULL, &g_can_task_handle);
#endif
#if APP_TASK_CAN_TX_TEST_ENABLE
    ret |= AppTasks_CreateOne(CanTxTestTask, "CAN_Test_Task", CAN_BUSINESS_TASK_STACK_SIZE,
                              CAN_BUSINESS_TASK_PRIORITY, NULL, &g_can_test_task_handle);
#endif
#if APP_TASK_CAN_LOOPBACK_TEST_ENABLE
    ret |= AppTasks_CreateOne(CanLoopbackTestTask, "CAN_Loopback", CAN_BUSINESS_TASK_STACK_SIZE,
                              CAN_BUSINESS_TASK_PRIORITY, NULL, &g_can_loopback_task_handle);
#endif
#if APP_TASK_MOTOR0_ENABLE
    ret |= AppTasks_CreateOne(MotorTask, "Motor1_Task", MOTOR1_TASK_STACK_SIZE,
                              MOTOR1_TASK_PRIORITY, (void *)0, &g_motor0_task_handle);
#endif
#if APP_TASK_MOTOR1_ENABLE
    ret |= AppTasks_CreateOne(MotorTask, "Motor2_Task", MOTOR2_TASK_STACK_SIZE,
                              MOTOR2_TASK_PRIORITY, (void *)1, &g_motor1_task_handle);
#endif
#if APP_TASK_STATUS_ENABLE
    ret |= AppTasks_CreateOne(StatusTask, "Status_Monitoring_Task",
                              STATUS_MOMITORING_TASK_STACK_SIZE,
                              STATUS_MOMITORING_TASK_PRIORITY, NULL, &g_status_task_handle);
#endif
    taskEXIT_CRITICAL();

    if (ret != E_OK)
        log_error("Failed to create application task");
    return ret;
}
