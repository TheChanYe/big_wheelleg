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
#define WATCHDOG_STARTUP_GRACE_MS 8000u

namespace 
{
    TaskManager* g_task_manager = nullptr;  // 全局任务管理器实例
}

TaskManager::TaskManager()  // 构造函数
    : watchdog_ready_(0u),
      watchdog_alive_(0u),
      watchdog_start_tick_(0u),
      motor0_task_handle_(nullptr),
      motor1_task_handle_(nullptr),
      uart_task_handle_(nullptr),
      status_task_handle_(nullptr),
      can_task_handle_(nullptr),
      can_test_task_handle_(nullptr),
      can_loopback_task_handle_(nullptr),
      motor0_service_(0u),
      motor1_service_(1u),
      can_business_(),
      safety_limiter_(),
      motor_fault_manager_()
{
    /*
     * FreeRTOS 的任务入口必须是普通函数或静态成员函数。
     * 静态任务入口通过该指针访问当前 TaskManager 对象。
     */
    g_task_manager = this;
}

TaskManager::~TaskManager()  // 析构函数
{
    /*
     * 任务由 FreeRTOS 调度器管理。main() 正常情况下不会返回，
     * 因此这里不主动删除任务，避免删除正在运行的任务。
     */
    if (g_task_manager == this) {
        g_task_manager = nullptr;
    }
}

// 获取电机任务句柄
TaskHandle_t TaskManager::motorTaskHandle(uint8_t motor_id) const
{
    if (motor_id == 0u) 
    {
        return motor0_task_handle_;
    }

    if (motor_id == 1u) 
    {
        return motor1_task_handle_;
    }

    return nullptr;
}

/**
 * 函数用途：初始化看门狗硬件。
 * 主要职责：配置看门狗时钟分频、重载值，并启动看门狗。
 */
void TaskManager::watchdogHardwareInit()
{
    wdt_register_write_enable(TRUE);
    wdt_divider_set(WDT_CLK_DIV_256);
    wdt_reload_value_set(625u);

    while (wdt_flag_get(WDT_DIVF_UPDATE_FLAG | WDT_RLDF_UPDATE_FLAG));
    
    wdt_enable();
    watchdog_start_tick_ = xTaskGetTickCount();
}

/**
 * 函数用途：看门狗标记存活。
 * 参数说明：
 *     bit: 待标记的看门狗位。
 *     ready: 待标记的看门狗位是否就绪。
 */
void TaskManager::watchdogMarkAlive(uint8_t bit, uint8_t ready)
{
    taskENTER_CRITICAL();

    if (ready != 0u)
    {
        watchdog_ready_ |= bit;
    }

    watchdog_alive_ |= bit;

    taskEXIT_CRITICAL();
}

/**
 * 函数用途：看门狗处理。
 * 主要职责：检查看门狗状态，并重新加载看门狗。
 */
void TaskManager::watchdogProcess()
{
    uint8_t ready_snapshot;
    uint8_t alive_snapshot;

    const TickType_t now = xTaskGetTickCount();

    taskENTER_CRITICAL();

    ready_snapshot = watchdog_ready_;
    alive_snapshot = watchdog_alive_;
    watchdog_alive_ = 0u;
    
    taskEXIT_CRITICAL();

    if ((now - watchdog_start_tick_) 
            < pdMS_TO_TICKS(WATCHDOG_STARTUP_GRACE_MS)
        || (ready_snapshot == WATCHDOG_REQUIRED
            && (alive_snapshot & WATCHDOG_REQUIRED)
            == WATCHDOG_REQUIRED))
    {
        wdt_counter_reload();
    }
}
/**
 * @brief 两个电机共用的 FreeRTOS 任务入口。
 * @param argument 由 create() 传入的物理电机编号。
 *
 * FreeRTOS 要求任务入口为普通函数指针，因此该函数必须是 static。
 * 找到对应的 MotorService 对象后，后续控制使用对象成员函数完成。
 */
void TaskManager::motorTask(void *argument)
{
    const uint8_t motor_id =
        static_cast<uint8_t>(reinterpret_cast<uintptr_t>(argument));    // 获取电机 ID

    /*
     * 两个任务共用同一个入口，通过 motor_id 选择各自的服务对象。
     *
     * create() 只会传入 0 或 1，因此这里不会出现其他编号。
     */
    MotorService& service =
        (motor_id == 0u)
            ? g_task_manager->motor0_service_
            : g_task_manager->motor1_service_;
            
    if (service.init() != E_OK) {
        log_error("Motor Init Failed!");
        /*
         * 当前电机初始化失败后只删除当前任务。
         * 其他任务和硬件看门狗继续按照原有逻辑运行。
         */
        vTaskDelete(nullptr);
    }
    /*
     * 电机初始化完成后，通知看门狗该任务已经准备就绪。
     */
    g_task_manager->watchdogMarkAlive( // 标记电机任务存活
        (motor_id == 0u)
            ? WATCHDOG_MOTOR0_ALIVE
            : WATCHDOG_MOTOR1_ALIVE,
            1u
    );

    while (true) 
    {
        /*
         * 控制任务严格由 ADC 中断通知驱动。
         * 不添加固定延时，也不改变控制周期来源。
         */
        const uint32_t notifications = 
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // 等待通知
        
        service.recordControlCycle(notifications); // 记录控制周期
        service.run(); // 执行电机控制逻辑

        /*
         * 标记本轮控制已经完成，但不重复设置 ready 状态。
         */
        g_task_manager->watchdogMarkAlive(
            (motor_id == 0u)
                ? WATCHDOG_MOTOR0_ALIVE
                : WATCHDOG_MOTOR1_ALIVE,
                0U
        );
    }
}

/**
 * 函数用途：CAN 业务任务。
 * 参数说明：
 *     argument: 任务参数。
 * 
 * 主要职责：处理 CAN 业务并监控任务存活。
 */
void TaskManager::canBusinessTask(void *argument)
{
    (void)argument;

    if (g_task_manager->can_business_.init() != E_OK) // 初始化 CAN 业务失败
    {
        log_error("CAN Business Init Failed!");
        vTaskDelete(nullptr);
    }

    g_task_manager->watchdogMarkAlive(WATCHDOG_CAN_ALIVE, 1u);

    while (true)
    {
        g_task_manager->can_business_.process(); // 处理 CAN 业务
        /*
         * 驱动故障状态寄存器读取继续保留在 CAN 任务中延迟处理，
         * 避免在外部中断中执行 SPI 访问。
         */
        DrvFault_Process(); // 处理驱动故障

        g_task_manager->watchdogMarkAlive(
            WATCHDOG_CAN_ALIVE, 0u
        );

        vTaskDelay(pdMS_TO_TICKS(1));

    }
}

/**
 * 函数用途：UART 业务任务。
 * 参数说明：
 *     argument: 任务参数。
 * 
 * 主要职责：处理 UART 业务。
 */
void TaskManager::uartBusinessTask(void *argument)
{
    (void)argument;

    uart_business_init();

    while (true)
    {
        /* MPU6050 调试阶段暂时关闭原来的高速浮点遥测，避免刷屏。 */
        // uart_business_process();
        vTaskDelay(pdMS_TO_TICKS(500)); // 延时 500 毫秒
    }
}

/**
 * 函数用途：CAN 测试任务。
 * 参数说明：
 *     argument: 任务参数。
 * 
 * 主要职责：进行 CAN 测试。
 */
void TaskManager::canTxTestTask(void *argument)
{
    (void)argument;

    if (g_task_manager->can_business_.init() != E_OK)
    {
        vTaskDelete(nullptr);
    }

    while (true)
    {
        g_task_manager->can_business_.processTxTest();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * 函数用途：CAN 回环测试任务。
 * 参数说明：
 *     argument: 任务参数。
 * 
 * 主要职责：进行 CAN 回环测试。
 */
void TaskManager::canLoopbackTestTask(void *argument)
{
    (void)argument;

    if (g_task_manager->can_business_.init() != E_OK)
    {
        vTaskDelete(nullptr);
    }
    while (true)
    {
        g_task_manager->can_business_.processLoopbackTest();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
/**
 * 函数用途：状态任务。
 * 参数说明：
 *     argument: 任务参数。
 * 
 * 主要职责：处理系统状态。
 */
void TaskManager::statusTask(void *argument)
{
    (void)argument;

    while (true)
    {
        g_task_manager->system_monitor_.process();

        g_task_manager->watchdogMarkAlive(
            WATCHDOG_STATUS_ALIVE, 1u
        );

        g_task_manager->watchdogProcess();

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
/**
 * 函数用途：创建任务。
 * 参数说明：
 *     function: 任务函数。
 *     name: 任务名称。
 *     stack_size: 任务栈大小。
 *     priority: 任务优先级。
 *     argument: 任务参数。
 *     handle: 任务句柄。
 * 
 * 主要职责：封装 FreeRTOS 的 xTaskCreate 函数，创建一个任务。
 */
int TaskManager::createOne(TaskFunction_t function,
                            const char *name,
                            uint16_t stack_size,
                            UBaseType_t priority,
                            void *argument,
                            TaskHandle_t *handle)
{
    return (xTaskCreate(function,
                        name,
                        stack_size,
                        argument,
                        priority,
                        handle
    ) == pdPASS) ? E_OK : E_ERROR;
}


int TaskManager::create()
{
    int result = E_OK;

    taskENTER_CRITICAL();

#if APP_TASK_UART_ENABLE
    result |= createOne(uartBusinessTask,
                        "USART_Send_Task",
                        USART_SEND_TASK_STACK_SIZE,
                        USART_SEND_TASK_PRIORITY,
                        nullptr,
                        &uart_task_handle_);
#endif

#if APP_TASK_CAN_ENABLE
    result |= createOne(canBusinessTask,
                        "CAN_Business",
                        CAN_BUSINESS_TASK_STACK_SIZE,
                        CAN_BUSINESS_TASK_PRIORITY,
                        nullptr,
                        &can_task_handle_);
#endif

#if APP_TASK_CAN_TX_TEST_ENABLE
    result |= createOne(canTxTestTask,
                        "CAN_Test_Task",
                        CAN_BUSINESS_TASK_STACK_SIZE,
                        CAN_BUSINESS_TASK_PRIORITY,
                        nullptr,
                        &can_test_task_handle_);
#endif

#if APP_TASK_CAN_LOOPBACK_TEST_ENABLE
    result |= createOne(canLoopbackTestTask,
                        "CAN_Loopback",
                        CAN_BUSINESS_TASK_STACK_SIZE,
                        CAN_BUSINESS_TASK_PRIORITY,
                        nullptr,
                        &can_loopback_task_handle_);
#endif

#if APP_TASK_MOTOR0_ENABLE
    result |= createOne(
        motorTask,
        "Motor1_Task",
        MOTOR1_TASK_STACK_SIZE,
        MOTOR1_TASK_PRIORITY,
        reinterpret_cast<void*>(static_cast<uintptr_t>(0u)),
        &motor0_task_handle_);
#endif

#if APP_TASK_MOTOR1_ENABLE
    result |= createOne(
        motorTask,
        "Motor2_Task",
        MOTOR2_TASK_STACK_SIZE,
        MOTOR2_TASK_PRIORITY,
        reinterpret_cast<void*>(static_cast<uintptr_t>(1u)),
        &motor1_task_handle_);
#endif

#if APP_TASK_STATUS_ENABLE
    result |= createOne(statusTask,
                        "Status_Monitoring_Task",
                        STATUS_MOMITORING_TASK_STACK_SIZE,
                        STATUS_MOMITORING_TASK_PRIORITY,
                        nullptr,
                        &status_task_handle_);
#endif

    taskEXIT_CRITICAL();

    /*
     * 即使状态任务创建失败，硬件看门狗也会启动并自行复位。
     */
    watchdogHardwareInit();

    if (result != E_OK)
    {
        log_error("Failed to create application task");
    }

    return result;
}

extern "C" TaskHandle_t AppTasks_GetMotorTaskHandle(
    uint8_t motor_id)
{
    if (g_task_manager == nullptr)
    {
        return nullptr;
    }

    return g_task_manager->motorTaskHandle(motor_id);
}
