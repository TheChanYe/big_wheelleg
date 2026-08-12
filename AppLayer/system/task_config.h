/**
 * 文件用途：FreeRTOS 应用任务配置。
 * 所属层级：AppLayer/system。
 * 主要职责：集中管理任务启用开关、优先级与栈大小，不包含中断优先级。
 */

#ifndef TASK_CONFIG_H
#define TASK_CONFIG_H

#include "my_can.h"

/* 保持原任务优先级数值和启用状态不变。 */
#define APP_TASK_CAN_ENABLE              1
#define APP_TASK_UART_ENABLE             1
#define APP_TASK_MOTOR0_ENABLE           1
#define APP_TASK_MOTOR1_ENABLE           1
#define APP_TASK_STATUS_ENABLE           1
#define APP_TASK_DISPLAY_ENABLE          0

#define APP_TASK_CAN_TX_TEST_ENABLE      ENABLE_CAN_TX_TEST
#define APP_TASK_CAN_LOOPBACK_TEST_ENABLE ENABLE_CAN_LOOPBACK_TEST

#define CAN_BUSINESS_TASK_PRIORITY       1
#define DISPALY_TASK_PRIORITY            2
#define USART_SEND_TASK_PRIORITY         3
#define UART_TASK_PRIORITY               4
#define STATUS_MOMITORING_TASK_PRIORITY  5
#define MOTOR1_TASK_PRIORITY             6
#define MOTOR2_TASK_PRIORITY             7

#define CAN_BUSINESS_TASK_STACK_SIZE             256
#define UART_TX_TASK_STACK_SIZE                  512
#define UART_RX_TASK_STACK_SIZE                  512
#define USART_SEND_TASK_STACK_SIZE               512
#define DISPALY_TASK_STACK_SIZE                  512
#define MOTOR1_TASK_STACK_SIZE                   1024
#define MOTOR2_TASK_STACK_SIZE                   1024
#define STATUS_MOMITORING_TASK_STACK_SIZE        512

#endif
