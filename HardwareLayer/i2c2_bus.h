#ifndef I2C2_BUS_H
#define I2C2_BUS_H

#include "main.h"

/**
 * @brief 板载 I2C2 总线：PB10=SCL，PB11=SDA。
 *
 * 使用 AT32 硬件 I2C，采用阻塞轮询方式。当前阶段只允许一个调用者，
 * 不要同时从多个 FreeRTOS 任务调用；后续接入任务时再增加互斥保护。
 *
 * 对外使用 7 位设备地址，例如 MPU6050 的 0x68；
 * 地址左移由本类负责，调用者不要传 0xD0。
 */
class I2c2Bus final {
  public:
    I2c2Bus() = default;
    ~I2c2Bus() = default;

    I2c2Bus(const I2c2Bus&) = delete;
    I2c2Bus& operator = (const I2c2Bus&) = delete;

    /** 配置 GPIO e和 I2C2， 成功返回 E_OK */
    int init();

    /** 向ee设备的一个寄存器写入一个字节 */
    int writeRegister(uint8_t address7, uint8_t reg, uint8_t value);

    /** 从设备的一个寄存器读取一个字节，成功时写入 value */
    int readRegister(uint8_t address7, uint8_t reg, uint8_t& value);

    /**
     * @brief 从连续寄存器中读取多个字节。
     *
     * @param address7 7 位 I²C 地址，例如 MPU6050 为 0x68。
     * @param start_reg 第一个寄存器地址。
     * @param data      接收缓冲区。
     * @param length    要读取的字节数。
     *
     * @return E_OK 表示成功，其他值表示失败。
     */
    int readRegisters(uint8_t address7,
                      uint8_t start_reg,
                      uint8_t *data,
                      uint16_t length);
                      
    
  private:
    static constexpr uint32_t kWaitLimit = 200000u;

    bool initialized_ = false;

    /** 等待标志达到期望状态；同时检测 NACK、总线错误和仲裁丢失。 */
    bool waitFlag(uint32_t flag, bool expectedSet) const;

    /**
     * 产生 START 并发送设备地址。
     * address7 是 7 位地址；这里只转换一次为库函数要求的地址字节。
     * 成功返回时 ADDR7F 尚未清除，由调用方决定接收 ACK 的时机。
    */
    bool startAddress(uint8_t address7, i2c_direction_type direction) const;

    /** 出错时发送 STOP、清错误标志并恢复接收 ACK。 */
    void abortTransfer() const;
};

#endif // I2C2_BUS_H