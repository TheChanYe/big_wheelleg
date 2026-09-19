/**
 * 文件用途：系统基础初始化实现。
 * 所属层级：AppLayer/system。
 * 主要职责：集中保存与具体业务无关的启动顺序和全局互斥锁。
 */

#include "system_init.h"
#include "safety_limit.h"
#include "motor_fault.h"
#include "drv_fault.h"

SemaphoreHandle_t mutex = NULL;

int System_Init(void)
{
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    SystemInit();
    system_clock_config();
    log_init();

    /* 所有任务创建前完成互斥锁创建，避免任务抢先访问未就绪资源。 */
    mutex = xSemaphoreCreateMutex();
    if (mutex == NULL)
    {
        log_error("Failed to create mutex!");
        return E_OUT_OF_MEMORY;
    }

    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);

    SafetyLimit_Init();
    MotorFault_Init();
    if (DrvFault_Init() != E_OK)
        return E_ERROR;

    return E_OK;
}
