/**
 * 文件用途：UART 协议业务接口。
 * 所属层级：Communication。
 * 主要职责：封装当前 UART 初始化、接收日志回调和测试遥测字符串生成。
 */

#ifndef UART_BUSINESS_H
#define UART_BUSINESS_H

#include "main.h"

/** 功能：初始化 UART BSP 并注册接收回调；参数：无；返回值：E_OK。 */
int uart_business_init(void);
/** 功能：发送一次当前测试遥测；参数：无；返回值：无。 */
void uart_business_process(void);

#endif
