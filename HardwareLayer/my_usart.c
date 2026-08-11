#include "my_usart.h"
#define MODULE_NAME "my_usart"

#ifdef MODE_LOG_TAG
#undef MODE_LOG_TAG
#endif
#define MODE_LOG_TAG MODULE_NAME

#define BUFFER_SIZE 128 // 数据缓冲区大小


#define UART_QUEUE_LENGTH 10       // 定义队列长度

// 定义UART缓冲区结构体
typedef struct {
    char buffer[BUFFER_SIZE]; // 数据缓冲区
    size_t length; // 数据长度
} UartBuffer_t;

// 发送和接收队列
static QueueHandle_t uartTxQueue; // 用于发送数据的队列
static QueueHandle_t uartRxQueue; // 用于接收数据的队列

// 发送和接收任务句柄
static TaskHandle_t uartTxTaskHandle = NULL; // 发送任务句柄
static TaskHandle_t uartRxTaskHandle = NULL; // 接收任务句柄

// 外部数据接收处理回调函数
static void (*external_process_received_data)(char *data, size_t length) = NULL; // 数据接收回调函数

// 发送和接收缓冲区
static UartBuffer_t txBuffer; // 发送缓冲区结构体
static UartBuffer_t rxBuffer; // 接收缓冲区结构体

// 函数声明
static void usart_configuration(void); // USART配置函数
static void m_send(char *data, uint16_t data_len); // 发送函数
static void UART_RxCallback(void); // 接收回调函数
static void set_receive_callback(void (*callback)(char *data, size_t length)); // 设置接收回调函数
static void UART_TxTask(void *pvParameters); // 发送任务函数
static void UART_RxTask(void *pvParameters); // 接收任务函数

// 中断服务程序声明
void USART2_IRQHandler(void); // USART2中断服务程序
void DMA1_Channel7_IRQHandler(void); // DMA1通道7中断服务程序

// USART配置函数
static void usart_configuration(void)
{
    gpio_init_type gpio_init_struct; // GPIO初始化结构体
    dma_init_type dma_init_struct; // DMA初始化结构体
    usart_type *usartx = USART2; // 选择USART2
    // 启用USART时钟
    crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);

    // 初始化GPIO默认参数
    gpio_default_para_init(&gpio_init_struct);

    // 配置USART TX引脚
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER; // 设置引脚驱动强度
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL; // 设置引脚输出类型
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX; // 设置引脚模式
    gpio_init_struct.gpio_pins = GPIO_PINS_2; // 设置引脚号
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE; // 设置引脚上拉/下拉
    gpio_init(GPIOA, &gpio_init_struct); // 初始化GPIOA引脚

    // 配置USART RX引脚
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT; // 设置引脚模式为输入
    gpio_init_struct.gpio_pins = GPIO_PINS_3; // 设置引脚号
    gpio_init_struct.gpio_pull = GPIO_PULL_UP; // 设置引脚上拉
    gpio_init(GPIOA, &gpio_init_struct); // 初始化GPIOA引脚

    // 配置USART参数
    usart_init(usartx, 115200, USART_DATA_8BITS, USART_STOP_1_BIT); // 初始化USART
    usart_transmitter_enable(usartx, TRUE); // 启用USART发送器
    usart_receiver_enable(usartx, TRUE); // 启用USART接收器
    usart_dma_transmitter_enable(usartx, TRUE); // 启用USART DMA发送
    usart_dma_receiver_enable(usartx, TRUE); // 启用USART DMA接收
    usart_interrupt_enable(usartx, USART_IDLE_INT, TRUE); // 启用空闲中断
    usart_enable(usartx, TRUE); // 启用USART
    // 启用USART全局中断
    nvic_irq_enable(USART2_IRQn, USART2_PRO, 0); // 启用USART2中断

				// 配置DMA用于USART TX传输
		dma_reset(DMA1_CHANNEL7); // 复位DMA1的通道7，确保之前的配置不会影响新配置
		dma_default_para_init(&dma_init_struct); // 初始化DMA默认参数结构体，清除结构体中的旧配置

		// 配置DMA传输的具体参数
		dma_init_struct.buffer_size = BUFFER_SIZE; // 设置DMA传输的数据量大小
		dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL; // 设置DMA传输方向为内存到外设
		dma_init_struct.memory_base_addr = (uint32_t)txBuffer.buffer; // 设置内存基地址，指向发送缓冲区
		dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE; // 设置内存数据宽度为字节
		dma_init_struct.memory_inc_enable = TRUE; // 启用内存增量模式，每次传输后内存地址递增
		dma_init_struct.peripheral_base_addr = (uint32_t)&usartx->dt; // 设置外设基地址，指向USART的数据寄存器
		dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE; // 设置外设数据宽度为字节
		dma_init_struct.peripheral_inc_enable = FALSE; // 禁用外设增量模式，外设地址保持不变
		dma_init_struct.priority = DMA_PRIORITY_MEDIUM; // 设置DMA优先级为中等
		dma_init_struct.loop_mode_enable = FALSE; // 禁用循环模式，即非循环模式
		dma_init(DMA1_CHANNEL7, &dma_init_struct); // 使用上述参数初始化DMA1的通道7

		// 启用DMA传输完成中断
		dma_interrupt_enable(DMA1_CHANNEL7, DMA_FDT_INT, TRUE); // 启用DMA传输完成中断
		nvic_irq_enable(DMA1_Channel7_IRQn, DMA1_7_PRO, 1); // 设置并启用DMA通道7的中断，优先级为0

		// 配置DMA用于USART RX接收
		dma_reset(DMA1_CHANNEL6); // 复位DMA1的通道6，确保之前的配置不会影响新配置
		dma_default_para_init(&dma_init_struct); // 初始化DMA默认参数结构体，清除结构体中的旧配置

		// 配置DMA传输的具体参数
		dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY; // 设置DMA传输方向为外设到内存
		dma_init_struct.buffer_size = BUFFER_SIZE; // 设置DMA传输的数据量大小
		dma_init_struct.memory_base_addr = (uint32_t)rxBuffer.buffer; // 设置内存基地址，指向接收缓冲区
		dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE; // 设置内存数据宽度为字节
		dma_init_struct.memory_inc_enable = TRUE; // 启用内存增量模式，每次传输后内存地址递增
		dma_init_struct.peripheral_base_addr = (uint32_t)&usartx->dt; // 设置外设基地址，指向USART的数据寄存器
		dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE; // 设置外设数据宽度为字节
		dma_init_struct.peripheral_inc_enable = FALSE; // 禁用外设增量模式，外设地址保持不变
		dma_init_struct.priority = DMA_PRIORITY_MEDIUM; // 设置DMA优先级为中等
		dma_init_struct.loop_mode_enable = FALSE; // 禁用循环模式，即非循环模式
		dma_init(DMA1_CHANNEL6, &dma_init_struct); // 使用上述参数初始化DMA1的通道6

    // 初始化队列
    uartTxQueue = xQueueCreate(UART_QUEUE_LENGTH, sizeof(UartBuffer_t)); // 创建发送队列，长度为UART_QUEUE_LENGTH
    uartRxQueue = xQueueCreate(UART_QUEUE_LENGTH, sizeof(UartBuffer_t)); // 创建接收队列，长度为UART_QUEUE_LENGTH

//    // 进入临界区
//    taskENTER_CRITICAL();

    // 创建发送任务
    if (xTaskCreate(UART_TxTask, "UART_TxTask", UART_TX_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, &uartTxTaskHandle) != pdPASS)
    {
        log_error("Failed to create UART_TxTask"); // 打印错误信息
    }

    // 创建接收任务
    if (xTaskCreate(UART_RxTask, "UART_RxTask", UART_RX_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, &uartRxTaskHandle) != pdPASS)
    {
        log_error("Failed to create UART_RxTask"); // 打印错误信息
    }

//    // 退出临界区
//    taskEXIT_CRITICAL();

//    // 启动调度器
//    vTaskStartScheduler();


}

// 发送任务函数
static void UART_TxTask(void *pvParameters)
{
    UartBuffer_t txData; // 定义发送数据缓冲区结构体
    while (1)
    {
        // 从发送队列中获取数据
        if (xQueueReceive(uartTxQueue, &txData, portMAX_DELAY) == pdPASS)
        {
            dma_channel_enable(DMA1_CHANNEL7, FALSE); // 禁用DMA通道7

            // 设置DMA传输长度和内存地址
            DMA1_CHANNEL7->dtcnt = txData.length; // 设置传输数据长度
            DMA1_CHANNEL7->maddr = (uint32_t)txData.buffer; // 设置内存地址

            dma_channel_enable(DMA1_CHANNEL7, TRUE); // 启用DMA通道7
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // 等待传输完成通知
        }
        else
        {
            log_error("UART_TxTask: 从uartTxQueue接收失败"); // 打印错误信息
        }
    }
}

// 接收任务函数
static void UART_RxTask(void *pvParameters)
{
    UartBuffer_t rxData; // 定义接收数据缓冲区结构体
    while (1)
    {
        // 从接收队列中获取数据
        if (xQueueReceive(uartRxQueue, &rxData, portMAX_DELAY) == pdPASS)
        {
            // 调用外部回调函数处理接收到的数据
            if (external_process_received_data != NULL)
            {
                external_process_received_data(rxData.buffer, rxData.length); // 处理接收到的数据
            }
        }
        else
        {
            log_error("UART_RxTask: 从uartRxQueue接收失败"); // 打印错误信息
        }
    }
}

// 发送函数
static void m_send(char *data, uint16_t data_len)
{
    // 检查数据长度是否超出缓冲区大小
    if (data_len > BUFFER_SIZE)
    {
        data_len = BUFFER_SIZE; // 如果超出，截断数据
    }

    UartBuffer_t txData; // 定义发送数据缓冲区结构体
    memcpy(txData.buffer, data, data_len); // 拷贝数据到缓冲区
    txData.length = data_len; // 设置数据长度

    // 将数据发送到发送队列
    if (xQueueSend(uartTxQueue, &txData, portMAX_DELAY) != pdPASS)
    {
        log_error("m_send: 发送到uartTxQueue失败"); // 打印错误信息
    }
}

// 接收回调函数
static void UART_RxCallback(void)
{
     BaseType_t xHigherPriorityTaskWoken = pdFALSE; // 高优先级任务唤醒标志

    // 计算接收到的数据长度
    size_t length = BUFFER_SIZE - DMA1_CHANNEL6->dtcnt;

    // 确保清空接收缓冲区前保留有效数据
    if (length > 0 && length <= BUFFER_SIZE)
    {
        UartBuffer_t rxData; // 定义接收数据缓冲区结构体
        memcpy(rxData.buffer, rxBuffer.buffer, length); // 拷贝数据到接收缓冲区
        rxData.length = length; // 设置数据长度

        // 将接收到的数据发送到接收队列
        if (xQueueSendFromISR(uartRxQueue, &rxData, &xHigherPriorityTaskWoken) != pdPASS)
        {
            log_error("UART_RxCallback: 发送到uartRxQueue失败"); // 打印错误信息
        }
    }

    // 重置DMA传输计数
    DMA1_CHANNEL6->dtcnt = BUFFER_SIZE; // 重置DMA计数器
    dma_channel_enable(DMA1_CHANNEL6, TRUE); // 重新启用DMA通道6

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // 根据情况进行任务切换
}

// 设置接收回调函数
static void set_receive_callback(void (*callback)(char *data, size_t length))
{
    external_process_received_data = callback; // 设置外部回调函数
}

// USART2中断服务程序
void USART2_IRQHandler(void)
{
    // 检查是否为空闲中断
    if (usart_flag_get(USART2, USART_IDLEF_FLAG) != RESET)
    {
        usart_flag_clear(USART2, USART_IDLEF_FLAG); // 清除空闲中断标志
        dma_channel_enable(DMA1_CHANNEL6, FALSE); // 禁用DMA通道6

        UART_RxCallback(); // 调用接收回调函数
    }
}

// DMA1通道7中断服务程序
void DMA1_Channel7_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; // 高优先级任务唤醒标志
    // 检查是否为传输完成中断
    if (dma_interrupt_flag_get(DMA1_FDT7_FLAG))
    {
        dma_flag_clear(DMA1_FDT7_FLAG); // 清除传输完成中断标志
        vTaskNotifyGiveFromISR(uartTxTaskHandle, &xHigherPriorityTaskWoken); // 通知发送任务
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // 根据情况进行任务切换
}

// 获取UART驱动实例
UartDriver_t* get_uart_driver(void)
{
    static UartDriver_t uart_driver = {
        .init = usart_configuration, // 初始化函数
        .send = m_send, // 发送函数
        .set_receive_callback = set_receive_callback // 设置接收回调函数
    };
    return &uart_driver; // 返回UART驱动
}
