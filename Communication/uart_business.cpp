/**
 * 文件用途：UART 协议业务实现。
 * 所属层级：Communication。
 * 主要职责：保持原有文本遥测格式和发送周期，不改变 UART 硬件驱动。
 */

#include "uart_business.h"
#include "my_usart.h"
#include "foc_cfg.h" // 仅访问 Motor_Data，不引入尚未 C++ 化的驱动头。

extern Motor_Data g_motor1;

static char g_send_buf[256];

void UartBusiness::receiveCallback(char *data, size_t length)
{
    log_inform("Received data: %zu %.*s", length, (int)length, data);
}

int UartBusiness::init()
{
    UartDriver_t *uart_driver = get_uart_driver();
    uart_driver->init();
    uart_driver->set_receive_callback(receiveCallback);
    return E_OK;
}

void UartBusiness::process()
{
    UartDriver_t *uart_driver = get_uart_driver();
    memset(g_send_buf, 0, strlen(g_send_buf));
    snprintf(g_send_buf, sizeof(g_send_buf), "data:%f,%f,%f\n",
             g_motor1.control.speed_target, g_motor1.velocity,
             g_motor1.control.iq_current_target);
    uart_driver->send(g_send_buf, strlen(g_send_buf));
}

// 原 C 入口留给现有任务调用；硬件驱动仍以 C 编译。
extern "C" int uart_business_init(void)
{
    return UartBusiness::init();
}

extern "C" void uart_business_process(void)
{
    UartBusiness::process();
}
