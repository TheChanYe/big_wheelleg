/**
 * 文件用途：UART 协议业务实现。
 * 所属层级：Communication。
 * 主要职责：保持原有文本遥测格式和发送周期，不改变 UART 硬件驱动。
 */

#include "uart_business.h"
#include "my_usart.h"
#include "foc.h"

extern Motor_Data g_motor1;

static char g_send_buf[256];

static void uart_receive_callback(char *data, size_t length)
{
    log_inform("Received data: %zu %.*s", length, (int)length, data);
}

int uart_business_init(void)
{
    UartDriver_t *uart_driver = get_uart_driver();
    uart_driver->init();
    uart_driver->set_receive_callback(uart_receive_callback);
    return E_OK;
}

void uart_business_process(void)
{
    UartDriver_t *uart_driver = get_uart_driver();
    memset(g_send_buf, 0, strlen(g_send_buf));
    snprintf(g_send_buf, sizeof(g_send_buf), "data:%f,%f,%f\n",
             g_motor1.control.speed_target, g_motor1.velocity,
             g_motor1.control.iq_current_target);
    uart_driver->send(g_send_buf, strlen(g_send_buf));
}
