/**
 * 文件用途：系统基础初始化接口。
 * 所属层级：AppLayer/system。
 * 主要职责：初始化时钟、日志、同步资源和安全模块。
 */

#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

#include "main.h"

#ifdef __cplusplus

/**
 * 系统基础资源管理器。
 *
 * 对象必须在 main() 中创建，并在启动 FreeRTOS 调度器前调用 init()。
 */
class SystemManager final {
    public:
        SystemManager();    // 构造函数
        ~SystemManager();   

        SystemManager(const SystemManager&) = delete; // 禁止拷贝构造
        SystemManager& operator=(const SystemManager&) = delete; // 禁止拷贝赋值

        /**
        * 完成业务任务启动前的基础初始化。
        *
        * 返回：
        * E_OK：初始化成功
        * E_OUT_OF_MEMORY：互斥锁创建失败
        * E_ERROR：安全模块或驱动故障模块初始化失败
        */
        int init();
        /**
        * 获取系统共用互斥锁。
        */
        SemaphoreHandle_t  mutex() const;
    
    private:
        int createResources();   // 创建系统基础资源（互斥锁等）
        void enablePeripheralClocks(); // 启用外设时钟
        int initializeSafety();
        SemaphoreHandle_t mutex_;   // 系统共用互斥锁
};

extern "C" {
#endif

/**
 * 提供给尚未迁移的 C 模块使用。
 */
SemaphoreHandle_t System_GetMutex(void);
#ifdef __cplusplus
}
#endif

#endif // SYSTEM_INIT_H