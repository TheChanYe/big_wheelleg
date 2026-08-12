#include "main.h"
#include "switch.h"
#include "lcd.h"
#include "foc.h"
#include "my_usart.h"
#include "my_can.h"
#include "can_business.h"

#define MODULE_NAME       "main"

#ifdef  MODE_LOG_TAG
#undef  MODE_LOG_TAG
#endif
#define MODE_LOG_TAG          MODULE_NAME

TaskHandle_t usartSendTaskHandle = NULL;// 声明串口发送任务句柄
void usart_send_task(void *pvParameters);//声明串口发送任务

TaskHandle_t DisplayTaskHandle = NULL;// 声明显示屏任务句柄
void display_task(void *pvParameters);//声明显示屏任务

TaskHandle_t Motor1TaskHandle = NULL;// 声明电机1任务句柄
void motor1_task(void *pvParameters);//声明电机1任务

TaskHandle_t Motor2TaskHandle = NULL;// 声明电机2任务句柄
void motor2_task(void *pvParameters);//声明电机2任务

TaskHandle_t StatusMonitoringTaskHandle = NULL;// 声明状态监测任务句柄
void status_monitoring_task(void *pvParameters);//声明状态监测任务

TaskHandle_t CanTestTaskHandle = NULL;           // CAN测试任务句柄
void can_test_task(void *pvParameters);           // CAN测试任务

TaskHandle_t CanLoopbackTestTaskHandle = NULL;     // CAN回环测试任务句柄
void can_loopback_test_task(void *pvParameters);   // CAN回环测试任务

TaskHandle_t CanBusinessTaskHandle = NULL;          // CAN业务任务句柄
void can_business_task(void *pvParameters);         // CAN业务任务


Motor_Data g_motor2 = {0};//电机2全局数据
Motor_Data g_motor1 = {0};//电机1全局数据

SemaphoreHandle_t mutex;//互斥锁句柄

UBaseType_t  uxHighWaterMark;


/**************************数据发送接收任务***************************/
char send_buf[256] ;

// 串口接收数据回调函数
void usart_receive_callback(char *data, size_t length) {
 // 打印接收到的数据，使用 %.*s 来限制输出的长度
    log_inform("Received data: %zu %.*s", length, (int)length, data);
}

// 串口发送任务
void usart_send_task(void *pvParameters) {
    // 获取UART驱动
    UartDriver_t *uart_driver = get_uart_driver();
    // 初始化USART2
    uart_driver->init();
    // 设置接收回调函数
    uart_driver->set_receive_callback(usart_receive_callback);
    while (1) {
			memset(send_buf,0,strlen(send_buf));           
			snprintf(send_buf,sizeof(send_buf), "data:%f,%f,%f\n",g_motor1.control.speed_target,g_motor1.velocity,g_motor1.control.iq_current_target);	
			uart_driver->send(send_buf, strlen(send_buf));
        // 每隔1秒发送一次
        vTaskDelay(1);
    }
}



//    // 获取空闲任务堆栈剩余空间
//    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
//log_inform("High water mark: %lu\n", (unsigned long)uxHighWaterMark);

//		sprintf(send_buf,"current_abc:%f,%f,%f\n",g_motor1.current_abc.Ia,g_motor1.current_abc.Ib,g_motor1.current_abc.Ic);
//		sprintf(send_buf,"current_dq:%f,%f,%f,%f\n",g_motor1.current_dq.Id, g_motor1.current_dq.Iq, g_motor1.control.id_current_feedback, g_motor1.control.iq_current_feedback );
//		sprintf(send_buf,"pid_dq:%f,%f,%f,%f,%f,%f\n",g_motor1.control.id_current_target, g_motor1.control.id_current_feedback, g_motor1.voltage_dq.Vd, g_motor1.control.iq_current_target, g_motor1.control.iq_current_feedback, g_motor1.voltage_dq.Vq);
//		sprintf(send_buf,"pid_dq:%f,%f,%f,%f,%f,%f\n",g_motor1.control.id_current_target, g_motor1.control.id_current_feedback, g_motor1.voltage_dq.Vd,g_motor1.control.iq_current_target, g_motor1.control.iq_current_feedback, g_motor1.voltage_dq.Vq);
//		sprintf(send_buf,"data:%f,%f\n",g_motor1.theta,g_motor1.filter_angle);				
//		sprintf(send_buf,"pid:%f,%f,%f,%f,%f\n",g_motor1.control.iq_current_target,g_motor1.control.iq_current_feedback,current_pid.error, current_pid.integral, current_pid.output);

//		sprintf(send_buf,"data:%f,%f,%f\n",g_motor1.control.position_target,g_motor1.control.position_feedback,g_motor1.filter_angle);	
//		uart_driver->send(send_buf, strlen(send_buf));
//		vTaskList(buffer);
//		log_inform("%s", buffer);

/**************************显示屏任务***************************/
void display_task(void *pvParameters) {
		
    while (1) {

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
/***********************电机1任务****************************/
void motor1_task(void *pvParameters) {
    // 保存任务句柄供中断使用
    Motor1TaskHandle = xTaskGetCurrentTaskHandle();
	int ret = 0;
	ret = Motor_Init(MOTOR_1);
	if(ret != E_OK)
	{
		log_error("Motor Init Faild!");
	}
	g_motor1.run_state = RUN;

    while (1)
	{
#if 1	
    // 等待任务通知（阻塞直到ADC中断触发）
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if(g_motor1.run_state == RUN)
		{
			if (can_business_get_motor_mode() == CAN_MOTOR_MODE_SPEED)
			{
				ret = CascadeControl_Run(&g_motor1, Speed_loop,
					                        can_business_get_motor0_speed_target());
			}
			else
			{
				ret = CascadeControl_Run(&g_motor1, Current_loop,
					                        can_business_get_motor0_iq_output());
			}
				if(ret != E_OK)
				{
					g_motor1.run_state = FAULT;				
				}
		}
		else if(g_motor1.run_state == STOP)
		{
			ret = CascadeControl_Run(&g_motor1, Current_loop, 0.0f);
				if(ret != E_OK)
				{
					g_motor1.run_state = FAULT;				
				}
		}

#else
        vTaskDelay(pdMS_TO_TICKS(10));
#endif
    }
}
/***********************电机2任务****************************/
void motor2_task(void *pvParameters) {
    // 保存任务句柄供中断使用
    Motor2TaskHandle = xTaskGetCurrentTaskHandle();
	int ret = 0;
	ret = Motor_Init(MOTOR_2);
	if(ret != E_OK)
	{
		log_error("Motor Init Faild!");
	}
		g_motor2.run_state = RUN;
    while (1)
	{
#if 1	
    // 等待任务通知（阻塞直到ADC中断触发）
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if(g_motor2.run_state == RUN)
		{
			if (can_business_get_motor_mode() == CAN_MOTOR_MODE_SPEED)
			{
				ret = CascadeControl_Run(&g_motor2, Speed_loop,
					                        can_business_get_motor1_speed_target());
			}
			else
			{
				ret = CascadeControl_Run(&g_motor2, Current_loop,
					                        can_business_get_motor1_iq_output());
			}
				if(ret != E_OK)
				{
					g_motor2.run_state = FAULT;		
				}
		}
		else if(g_motor2.run_state == STOP)
		{
			ret = CascadeControl_Run(&g_motor2, Current_loop, 0.0f);
				if(ret != E_OK)
				{
					g_motor2.run_state = FAULT;				
				}
		}		
#else
        vTaskDelay(pdMS_TO_TICKS(10));
#endif
    }
}
/***********************CAN测试任务****************************/
void can_test_task(void *pvParameters) {
    int         ret;
    uint16_t    counter = 0;
    uint8_t     data[8];

    /* 初始化 CAN1 */
    ret = my_can_init();
    if (ret != E_OK)
    {
        log_error("CAN test task: my_can_init failed");
        vTaskDelete(NULL);
    }

    log_inform("CAN test task started (ID=0x%03X, 100ms, 500Kbps)", CAN_TEST_ID);

    while (1)
    {
        data[0] = 0xCA;
        data[1] = 0x4E;
        data[2] = (uint8_t)(counter & 0xFF);
        data[3] = (uint8_t)((counter >> 8) & 0xFF);
        data[4] = 0x11;
        data[5] = 0x22;
        data[6] = 0x33;
        data[7] = 0x44;

        ret = my_can_send_std(CAN_TEST_ID, data, CAN_TEST_DLC);
        if (ret != E_OK)
        {
            log_error("CAN TX failed (counter=%u)", counter);
        }

        counter++;
        vTaskDelay(pdMS_TO_TICKS(100));   /* 100ms = 10Hz */
    }
}

/***********************CAN回环测试任务****************************/
void can_loopback_test_task(void *pvParameters) {
    uint16_t    id;
    uint8_t     data[8];
    uint8_t     len;
    int         ret;

    /* init CAN1 with RX filter */
    ret = my_can_init();
    if (ret != E_OK)
    {
        log_error("CAN loopback: my_can_init failed");
        vTaskDelete(NULL);
    }

    log_inform("CAN loopback test task started (RX 0x124 -> TX 0x125)");

    while (1)
    {
        ret = my_can_receive_std(&id, data, &len);

        if (ret == E_OK)
        {
            if (id == 0x124)
            {
                my_can_send_std(0x125, data, len);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));   /* 1ms poll interval */
    }
}

/***********************CAN业务任务****************************/
void can_business_task(void *pvParameters) {
    uint16_t    id;
    uint8_t     data[8];
    uint8_t     len;
    int         ret;
    TickType_t  last_state_tick = 0;

    /* CAN init once from business task */
    ret = my_can_init();
    if (ret != E_OK)
    {
        log_error("CAN business: my_can_init failed");
        vTaskDelete(NULL);
    }

    log_inform("CAN business task started (RX 0x101/0x102)");

    while (1)
    {
        /* drain FIFO0 completely each poll cycle */
        while (my_can_receive_std(&id, data, &len) == E_OK)
        {
            can_business_process_frame(id, data, len);
        }

        /* safety: zero outputs on command timeout */
        can_business_tick();

        /* MOTOR0/MOTOR1 speed telemetry @ 50 Hz (20 ms) */
        if ((xTaskGetTickCount() - last_state_tick)
            >= pdMS_TO_TICKS(20))
        {
            last_state_tick = xTaskGetTickCount();
            can_business_send_motor0_speed_state();
            can_business_send_motor0_speed_diag();
            can_business_send_motor1_speed_state();
            can_business_send_motor1_speed_diag();
        }

        vTaskDelay(pdMS_TO_TICKS(1));   /* 1ms poll interval */
    }
}

/***********************状态监测任务****************************/
void status_monitoring_task(void *pvParameters) {
	c_switch led = {0};
	led = switch_create(GPIOD,GPIO_PINS_2);
    while (1)
	{			
		if(g_motor1.run_state == RUN && g_motor2.run_state == RUN )
			led.flicker(&led,200);		// 1000
		else if(g_motor1.run_state == FAULT)
			led.flicker(&led,500);
		else if(g_motor2.run_state == FAULT)
			led.flicker(&led,100);

		Get_Mos_Temp(&g_motor1);
		Get_Mos_Temp(&g_motor2);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
/*******************任务创建*******************************/
int main(void)
{
   
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4); // 配置NVIC优先级组
		SystemInit();    // 初始化系统时钟
    system_clock_config();
		log_init();	//日志系统初始化
    // 创建全局互斥锁（必须放在任务创建之前！）
    mutex = xSemaphoreCreateMutex();
    if (mutex == NULL) {
        log_error("Failed to create mutex!");
        while(1); // 互斥锁创建失败，系统挂起
    }
    // 启用GPIO和DMA时钟
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);
    // 进入临界区
    taskENTER_CRITICAL();
#if 1
    // 创建串口任务
    if (xTaskCreate((TaskFunction_t )usart_send_task,
										(const char*    )"USART_Send_Task", 
                    (uint16_t       )USART_SEND_TASK_STACK_SIZE,
										(void*          )NULL, 
                    (UBaseType_t    )USART_SEND_TASK_PRIORITY,
										(TaskHandle_t*  )&usartSendTaskHandle) != pdPASS) {
        log_inform("Failed to create USART send task\n");
    }
#endif

#if 0
    // 创建TFT显示屏任务
    if (xTaskCreate((TaskFunction_t )display_task,
										(const char*    )"Dispaly_Task", 
                    (uint16_t       )DISPALY_TASK_STACK_SIZE,
										(void*          )NULL, 
                    (UBaseType_t    )DISPALY_TASK_PRIORITY,
										(TaskHandle_t*  )&DisplayTaskHandle) != pdPASS) {
        log_inform("Failed to create Dispaly task\n");
    }
#endif

#if ENABLE_CAN_TX_TEST
    // 创建CAN测试任务
    if (xTaskCreate((TaskFunction_t )can_test_task,
                    (const char*    )"CAN_Test_Task",
                    (uint16_t       )CAN_BUSINESS_TASK_STACK_SIZE,
                    (void*          )NULL,
                    (UBaseType_t    )CAN_BUSINESS_TASK_PRIORITY,
                    (TaskHandle_t*  )&CanTestTaskHandle) != pdPASS) {
        log_inform("Failed to create CAN test task\n");
    }
#endif

#if ENABLE_CAN_LOOPBACK_TEST
    /* 创建CAN 回环测试任务 */
    if (xTaskCreate((TaskFunction_t )can_loopback_test_task,
                    (const char*    )"CAN_Loopback",
                    (uint16_t       )CAN_BUSINESS_TASK_STACK_SIZE,
                    (void*          )NULL,
                    (UBaseType_t    )CAN_BUSINESS_TASK_PRIORITY,
                    (TaskHandle_t*  )&CanLoopbackTestTaskHandle) != pdPASS) {
        log_inform("Failed to create CAN loopback test task\n");
    }
#endif

#if ENABLE_CAN_BUSINESS
    /* 创建CAN 业务任务 */
    if (xTaskCreate((TaskFunction_t )can_business_task,
                    (const char*    )"CAN_Business",
                    (uint16_t       )CAN_BUSINESS_TASK_STACK_SIZE,
                    (void*          )NULL,
                    (UBaseType_t    )CAN_BUSINESS_TASK_PRIORITY,
                    (TaskHandle_t*  )&CanBusinessTaskHandle) != pdPASS) {
        log_inform("Failed to create CAN business task\n");
    }
#endif

#if 1
    // 创建电机1任务
    if (xTaskCreate((TaskFunction_t )motor1_task,
										(const char*    )"Motor1_Task", 
                    (uint16_t       )MOTOR1_TASK_STACK_SIZE,
										(void*          )NULL, 
                    (UBaseType_t    )MOTOR1_TASK_PRIORITY,
										(TaskHandle_t*  )&Motor1TaskHandle) != pdPASS) {
        log_inform("Failed to create Motor1 task\n");
    }
#endif

#if 1
    // 创建电机2任务
    if (xTaskCreate((TaskFunction_t )motor2_task,
										(const char*    )"Motor2_Task", 
                    (uint16_t       )MOTOR2_TASK_STACK_SIZE,
										(void*          )NULL, 
                    (UBaseType_t    )MOTOR2_TASK_PRIORITY,
										(TaskHandle_t*  )&Motor2TaskHandle) != pdPASS) {
        log_inform("Failed to create Motor2 task\n");
    }
#endif

#if 1
    // 创建状态监测任务
    if (xTaskCreate((TaskFunction_t )status_monitoring_task,
										(const char*    )"Status_Monitoring_Task", 
                    (uint16_t       )STATUS_MOMITORING_TASK_STACK_SIZE,
										(void*          )NULL, 
                    (UBaseType_t    )STATUS_MOMITORING_TASK_PRIORITY,
										(TaskHandle_t*  )&StatusMonitoringTaskHandle) != pdPASS) {
        log_inform("Failed to create Status Monitoring task\n");
    }
#endif

    // 退出临界区
    taskEXIT_CRITICAL();

    // 启动调度器
    vTaskStartScheduler();

    // 永远不会到达这里
    while (1);
}
