#ifndef __MY_USART_H__
#define __MY_USART_H__

#include "main.h"

// 定义缓冲区长度
#define BUF_LEN 128

// 定义UART驱动接口结构体
typedef struct {
    void (*init)(void); // 初始化函数指针
    void (*send)(char *data, uint16_t data_len); // 发送函数指针
    void (*set_receive_callback)(void (*callback)(char *data, size_t length)); // 设置接收回调函数指针
} UartDriver_t;

// 获取UART驱动实例
UartDriver_t* get_uart_driver(void);

#endif // __MY_USART_H__
