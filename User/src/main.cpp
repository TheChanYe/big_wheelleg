/**
 * 文件用途：副板程序入口。
 * 所属层级：User。
 * 主要职责：完成系统基础初始化、创建应用任务并启动 FreeRTOS。
 */

#include "main.h"
#include "system_init.h"
#include "app_tasks.h"
#include "i2c2_bus.h"
#include "mpu6050.h"
#include "my_usart.h"
#include "delay.h"

namespace
{
/**
 * @brief MPU6050 临时验证任务。
 *
 * 每 500 ms 连续读取一次 14 字节测量寄存器，并通过 USART2 输出
 * 原始整数。暂不使用 %f，避免 newlib-nano 未启用浮点格式化时显示为空。
 */
void mpu6050LogTask(void *argument)
{
    Mpu6050 *mpu = static_cast<Mpu6050 *>(argument);
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true)
    {
        Mpu6050::RawSample sample{};
        const int result = mpu->readRaw(sample);

        if (result == E_OK)
        {
            log_inform("MPU raw A[%d,%d,%d] G[%d,%d,%d] T=%d",
                       static_cast<int>(sample.accel_x),
                       static_cast<int>(sample.accel_y),
                       static_cast<int>(sample.accel_z),
                       static_cast<int>(sample.gyro_x),
                       static_cast<int>(sample.gyro_y),
                       static_cast<int>(sample.gyro_z),
                       static_cast<int>(sample.temperature));
        }
        else
        {
            log_error("MPU6050 read failed, result=%d", result);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(500));
    }
}
} // namespace

int main(void)
{
    SystemManager system;
    
    if (system.init() != E_OK)
    {
        /* 基础资源创建失败时不能继续启动控制任务。 */
        while (1);
    }

    /* 调试日志共用现有 USART2 DMA 队列。调度器启动后才实际发送。 */
    get_uart_driver()->init();
    log_inform("USART2 serial logger ready, baud=115200");

    /* MPU6050 复位和唤醒阶段需要毫秒级等待。 */
    delay_init();

    /* static 保证调度器启动后任务持有的对象地址始终有效。 */
    static I2c2Bus i2c2;
    static Mpu6050 mpu6050(i2c2, 0x68u);

    int mpu_result = i2c2.init();
    if (mpu_result == E_OK)
    {
        mpu_result = mpu6050.init();
    }

    log_inform("MPU6050 driver init result=%d", mpu_result);

    /* 临时寄存器诊断：确认配置写入及测量寄存器单字节访问结果。 */
    uint8_t who_am_i = 0u;
    uint8_t power_management = 0u;
    uint8_t configuration = 0u;
    uint8_t gyro_configuration = 0u;
    uint8_t accel_configuration = 0u;
    uint8_t accel_x_high = 0u;
    uint8_t accel_x_low = 0u;

    const int who_result = i2c2.readRegister(0x68u, 0x75u, who_am_i);
    const int power_result = i2c2.readRegister(0x68u, 0x6Bu, power_management);
    const int config_result = i2c2.readRegister(0x68u, 0x1Au, configuration);
    const int gyro_config_result = i2c2.readRegister(0x68u, 0x1Bu, gyro_configuration);
    const int accel_config_result = i2c2.readRegister(0x68u, 0x1Cu, accel_configuration);
    const int accel_h_result = i2c2.readRegister(0x68u, 0x3Bu, accel_x_high);
    const int accel_l_result = i2c2.readRegister(0x68u, 0x3Cu, accel_x_low);

    log_inform("REG who=%d/0x%02X pwr=%d/0x%02X cfg=%d/0x%02X",
               who_result, who_am_i,
               power_result, power_management,
               config_result, configuration);
    log_inform("REG gyro=%d/0x%02X accel=%d/0x%02X AX=%d,%d/%02X%02X",
               gyro_config_result, gyro_configuration,
               accel_config_result, accel_configuration,
               accel_h_result, accel_l_result,
               accel_x_high, accel_x_low);

    if (mpu_result == E_OK)
    {
        if (xTaskCreate(mpu6050LogTask,
                        "MPU6050_Log",
                        512u,
                        &mpu6050,
                        2u,
                        nullptr) != pdPASS)
        {
            log_error("Failed to create MPU6050 log task");
        }
    }

    /*
     * MPU6050 驱动验证期间不创建电机、CAN 和系统看门狗任务。
     * 当前只运行 UART 收发任务与 MPU6050 日志任务，避免测试中被看门狗复位。
     */

    vTaskStartScheduler();

    while (1);
}
 
