#ifndef __SYS_CFG_H__
#define __SYS_CFG_H__
/*任务优先级*/
/*
优先级值越大 优先级越高
相同优先级采用时间片轮转原则
高优先级任务必须进 vTaskDelay() 才会执行低优先级任务
*/
typedef enum __TASK_PRO
{
	CAN_BUSINESS_TASK_PRIORITY = 1,			// CAN业务任务优先级（低于电机任务）
	DISPALY_TASK_PRIORITY,					// 定义显示屏任务优先级
	USART_SEND_TASK_PRIORITY,				// 定义串口发送任务优先级
	UART_TASK_PRIORITY,						// 串口任务优先级
	STATUS_MOMITORING_TASK_PRIORITY,    //状态监测任务优先级
	MOTOR1_TASK_PRIORITY ,				//电机1任务
	MOTOR2_TASK_PRIORITY,					//电机2任务

}task_pro;

/*中断优先级,优先级值越小优先级越高*/
typedef enum __INTERRUPT_PRO
{
    DMA2_1_PRO    = 5,  // SPI3发送中断
    DMA2_2_PRO    = 5,  // SPI3接收中断
    ADC_1_2_PRO   = 6,  // ADC采样中断
    ADC_3_PRO     = 6,  // ADC采样中断
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

/*任务堆栈大小*/
#define CAN_BUSINESS_TASK_STACK_SIZE             256     // CAN业务任务堆栈大小
#define UART_TX_TASK_STACK_SIZE 						512 	// 串口发送任务的栈大小
#define UART_RX_TASK_STACK_SIZE 						512 	// 串口接收任务的栈大小
#define USART_SEND_TASK_STACK_SIZE 					512		// 发送任务堆栈大小
#define DISPALY_TASK_STACK_SIZE 						512		// 显示屏任务堆栈大小	
#define MOTOR1_TASK_STACK_SIZE 							1024	// 定义电机任务堆栈大小
#define MOTOR2_TASK_STACK_SIZE 							1024	// 定义电机任务堆栈大小
#define STATUS_MOMITORING_TASK_STACK_SIZE 	512 	// 状态监测任务堆栈大小

#endif



