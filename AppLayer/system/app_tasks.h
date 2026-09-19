/**
 * 文件用途：应用任务统一管理接口。
 * 所属层级：AppLayer/system。
 * 主要职责：公开 ADC 中断需要使用的电机任务句柄，并提供任务创建入口。
 */

#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "main.h"

#ifdef __cplusplus
#include "system_monitor.h"
#include "motor_service.h"
#include "can_business.h"
#include "safety_limit.h"
#include "motor_fault.h"
#endif

#ifdef __cplusplus

class TaskManager final {
    public:
        TaskManager();
        ~TaskManager();

        TaskManager(const TaskManager &) = delete; // 禁止拷贝构造
        TaskManager& operator=(const TaskManager &) = delete; // 禁止拷贝赋值

        int create();   // 创建任务
        TaskHandle_t motorTaskHandle(uint8_t motor_id) const;   // 获取指定电机的任务句柄

    private:
        int createOne(TaskFunction_t function,
                        const char *name,
                        uint16_t stack_size,
                        UBaseType_t priority,
                        void *argument,
                        TaskHandle_t *handle);   // 创建单个任务
        
        void watchdogHardwareInit();   // 初始化看门狗硬件
        void watchdogMarkAlive(uint8_t bit, uint8_t ready);   // 标记看门狗存活状态
        void watchdogProcess();   // 处理看门狗逻辑

        static void motorTask(void *argument);   // 电机任务入口函数
        static void canBusinessTask(void *argument);    // CAN 业务任务入口函数
        static void uartBusinessTask(void *argument);   // UART 业务任务入口函数
        static void canTxTestTask(void *argument);
        static void canLoopbackTestTask(void *argument);
        static void statusTask(void *argument);

        volatile uint8_t watchdog_ready_;   // 看门狗就绪标志
        volatile uint8_t watchdog_alive_;   // 看门狗存活标志
        TickType_t watchdog_start_tick_;   // 看门狗启动时的系统滴答计数器

        TaskHandle_t motor0_task_handle_;   // 电机0的任务句柄
        TaskHandle_t motor1_task_handle_;   // 电机1的任务句柄
        TaskHandle_t uart_task_handle_;     // UART 业务任务的任务句柄
        TaskHandle_t status_task_handle_;   // 状态任务的任务句柄
        TaskHandle_t can_task_handle_;      // CAN 业务任务的任务句柄
        TaskHandle_t can_test_task_handle_;   // CAN 测试任务的任务句柄
        TaskHandle_t can_loopback_task_handle_;   // CAN 回环测试任务的任务句柄
        SystemMonitor system_monitor_;      // 系统监控对象

        /**
        * 两个物理电机的业务服务对象。
        *
        * 这些对象的生命周期与 TaskManager 一致，并覆盖所有电机任务
        * 和 FreeRTOS 调度器的运行期。
        */
        MotorService motor0_service_;
        MotorService motor1_service_;
        /**
         * @brief 唯一的 CAN 业务对象。
         *
         * 该对象保存命令、序号、通信状态和遥测统计。
         * 构造函数不访问硬件，CAN 外设由 CAN 任务调用 init() 初始化。
         */
        CanBusiness can_business_;

        /**
         * @brief 全局双电机安全限流器。
         *
         * SystemManager 初始化安全模块时，会通过 C ABI 调用这个对象。
         */
        SafetyLimiter safety_limiter_;

        /**
         * @brief 全局双电机锁存故障管理器。
         *
         * 对象在 SystemManager 调用 MotorFault_Init() 前已经构造完成。
         */
        MotorFaultManager motor_fault_manager_;    
};

extern "C" {
#endif

/**
 * ADC 中断通过该接口取得对应电机任务句柄。
 */
TaskHandle_t AppTasks_GetMotorTaskHandle(uint8_t motor_id);


#ifdef __cplusplus
}
#endif

#endif
