#include "log.h"
#include "my_usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 临时串口调试日志：USART2 与 DMA 仍由原 UART 驱动独占管理。 */
class SerialLogger final
{
public:
    SerialLogger() = delete;

    static void write(const char *module, const char *level, int line,
                      const char *function, const char *format, va_list args)
    {
        /* 普通 FreeRTOS 队列接口不能在中断中使用。 */
        if (__get_IPSR() != 0u || format == nullptr)
        {
            return;
        }

        char buffer[BUF_LEN];
        int prefix = snprintf(buffer, sizeof(buffer), "%s %s %d %s: ",
                              module != nullptr ? module : "sys",
                              level != nullptr ? level : "[I]", line,
                              function != nullptr ? function : "?");
        if (prefix < 0)
        {
            return;
        }

        /* 为 CRLF 和结尾 NUL 留空间，过长日志直接截断。 */
        size_t used = static_cast<size_t>(prefix);
        if (used > sizeof(buffer) - 3u)
        {
            used = sizeof(buffer) - 3u;
        }
        if (vsnprintf(buffer + used, sizeof(buffer) - used - 2u,
                      format, args) < 0)
        {
            return;
        }
        used += strlen(buffer + used);
        buffer[used++] = '\r';
        buffer[used++] = '\n';

        UartDriver_t *uart = get_uart_driver();
        if (uart != nullptr && uart->try_send != nullptr)
        {
            /* 队列尚未建立或已满时丢弃，不能阻塞实时任务。 */
            (void)uart->try_send(buffer, static_cast<uint16_t>(used));
        }
    }
};

extern "C" int log_init(void)
{
    /* 系统初始化早期尚未启用 GPIO/DMA 时钟，UART 在 main 中初始化。 */
    return E_OK;
}

extern "C" void log_printf(const char *module, const char *level, int line,
                           const char *function, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    SerialLogger::write(module, level, line, function, format, args);
    va_end(args);
}
