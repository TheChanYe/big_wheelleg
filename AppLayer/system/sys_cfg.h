#ifndef __SYS_CFG_H__
#define __SYS_CFG_H__
/*中断优先级,优先级值越小优先级越高*/
typedef enum __INTERRUPT_PRO
{
    DMA2_1_PRO    = 5,  // SPI3发送中断
    DMA2_2_PRO    = 5,  // SPI3接收中断
    ADC_1_2_PRO   = 6,  // ADC采样中断
    ADC_3_PRO     = 6,  // ADC采样中断
    DRV_FAULT_5_9_PRO   = 5,
    DRV_FAULT_10_15_PRO = 5,
    DMA1_7_PRO    = 7,	//串口DMA中断
    USART2_PRO    = 8,  // 串口中断	
    EXTI_0_PRO    = 9,
    EXTI_1_PRO    = 9,
    EXTI_2_PRO    = 9,
    EXTI_3_PRO    = 9,
    EXTI_4_PRO    = 9,
    EXTI_5_9_PRO  = 9,
    EXTI_10_15_PRO= 9,
    DMA1_1_PRO    = 10,
    DMA1_2_PRO    = 10,
    DMA1_3_PRO    = 10,
    DMA1_4_PRO    = 10,
    DMA1_5_PRO    = 10,
    DMA1_6_PRO    = 10,
    TIM_7_UP_PRO  = 10,
} interrupt_pro;

#endif



