/**
 * 文件用途：系统基础初始化实现。
 * 所属层级：AppLayer/system。
 * 主要职责：集中保存与具体业务无关的启动顺序和全局互斥锁。
 */

#include "system_init.h"
#include "safety_limit.h"
#include "motor_fault.h"
#include "drv_fault.h"

namespace 
{
    SystemManager* g_system_manager = nullptr;
}

SystemManager::SystemManager()
    : mutex_(nullptr)
{
    g_system_manager = this;
}

SystemManager::~SystemManager()
{
    if (mutex_ != nullptr)
    {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }

    if (g_system_manager == this)
    {
        g_system_manager = nullptr;
    }

}

int SystemManager::init()
{ 
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4); // 配置 NVIC 优先级分组
    SystemInit();
    system_clock_config();
    log_init();

    int result = createResources();
    if (result != E_OK)
    {
        return result;
    }
    enablePeripheralClocks();

    return initializeSafety();
}

SemaphoreHandle_t SystemManager::mutex() const
{
    return mutex_;
}

int SystemManager::createResources()
{
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == nullptr)
    {
        log_error("Failed to create mutex!");
        return E_OUT_OF_MEMORY;
    }

    return E_OK;
}

void SystemManager::enablePeripheralClocks()
{
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);
}

int SystemManager::initializeSafety()
{
    SafetyLimit_Init();
    MotorFault_Init();

    if (DrvFault_Init() != E_OK)
    {
        return E_ERROR;
    }
    return E_OK;
}

extern "C" SemaphoreHandle_t System_GetMutex(void)
{
    if (g_system_manager == nullptr)
    {
        return nullptr;
    }

    return g_system_manager->mutex();
}