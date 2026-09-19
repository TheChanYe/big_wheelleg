#include "i2c2_bus.h"

int I2c2Bus::init()
{
    // 独立打开时钟，不依赖其他模块恰好先初始化 GPIOB。
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_I2C2_PERIPH_CLOCK, TRUE);

    // I2C 的 SCL/SDA 必须是复用开漏输出；高电平依靠上拉电阻。
    gpio_init_type gpio_config;
    gpio_default_para_init(&gpio_config);
    gpio_config.gpio_pins = GPIO_PINS_10 | GPIO_PINS_11;
    gpio_config.gpio_mode = GPIO_MODE_MUX;
    gpio_config.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
    gpio_config.gpio_pull = GPIO_PULL_NONE;
    gpio_config.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOB, &gpio_config);

    // 只复位 I2C2 外设，不影响其他 I2C 总线。
    i2c_reset(I2C2);
    i2c_init(I2C2, I2C_FSMODE_DUTY_2_1, 100000u);
    i2c_own_address1_set(I2C2, I2C_ADDRESS_MODE_7BIT, 0x00u);
    i2c_master_receive_ack_set(I2C2, I2C_MASTER_ACK_CURRENT);
    i2c_ack_enable(I2C2, TRUE);
    i2c_enable(I2C2, TRUE);

    initialized_ = true;
    return E_OK;
}

int I2c2Bus::readRegisters(uint8_t address7, uint8_t start_reg, 
                           uint8_t *data, uint16_t length) {
    if (!initialized_ || address7 > 0x7Fu 
        || data == nullptr || length == 0u) {
        return E_PARAM;
    }

    /*
     * 临时诊断实现：逐个使用已经验证通过的单字节读取事务。
     * 若这样能获得正常数据，即可确认问题只位于 AT32 多字节接收时序。
     * 注意：这种方式不能保证 14 个字节来自同一个采样时刻。
     */
    for (uint16_t index = 0u; index < length; ++index) {
        const uint8_t reg = static_cast<uint8_t>(start_reg + index);
        const int result = readRegister(address7, reg, data[index]);

        if (result != E_OK) {
            return result;
        }
    }
    return E_OK;
}



bool I2c2Bus::waitFlag(uint32_t flag, bool expectedSet) const
{
    for (uint32_t count = 0; count < kWaitLimit; ++count) {
        const bool isSet = (i2c_flag_get(I2C2, flag) == SET);
        if (isSet == expectedSet) {
            return true;
        }

        // 从机未应答、总线错误或仲裁丢失时，不继续等到超时。
        if (i2c_flag_get(I2C2, I2C_ACKFAIL_FLAG) == SET ||
            i2c_flag_get(I2C2, I2C_BUSERR_FLAG) == SET ||
            i2c_flag_get(I2C2, I2C_ARLOST_FLAG) == SET ||
            i2c_flag_get(I2C2, I2C_TMOUT_FLAG) == SET) {
            return false;
        }
    }
    return false;
}

bool I2c2Bus::startAddress(uint8_t address7,
                           i2c_direction_type direction) const
{
    i2c_start_generate(I2C2);
    if (!waitFlag(I2C_STARTF_FLAG, true)) {
        return false;
    }

    // 本工程所用 AT32 库函数接收的是包含 R/W 位的地址字节。
    // 例如 7 位地址 0x68 要传 0xD0，函数再设置最低位。
    i2c_7bit_address_send(
        I2C2, static_cast<uint8_t>(address7 << 1), direction);

    return waitFlag(I2C_ADDR7F_FLAG, true);
}

void I2c2Bus::abortTransfer() const
{
    i2c_stop_generate(I2C2);

    i2c_flag_clear(I2C2,
                   I2C_ACKFAIL_FLAG |
                   I2C_BUSERR_FLAG |
                   I2C_ARLOST_FLAG |
                   I2C_TMOUT_FLAG);

    // 接收会暂时改变 ACK 行为；无论哪里出错都恢复默认状态。
    i2c_master_receive_ack_set(I2C2, I2C_MASTER_ACK_CURRENT);
    i2c_ack_enable(I2C2, TRUE);
}

int I2c2Bus::writeRegister(uint8_t address7, uint8_t reg, uint8_t value)
{
    if (!initialized_ || address7 > 0x7Fu) {
        return E_PARAM;
    }

    // 不在忙碌总线上插入新的 START。
    if (!waitFlag(I2C_BUSYF_FLAG, false)) {
        return E_ERROR;
    }

    if (!startAddress(address7, I2C_DIRECTION_TRANSMIT)) {
        abortTransfer();
        return E_ERROR;
    }
    i2c_flag_clear(I2C2, I2C_ADDR7F_FLAG);

    if (!waitFlag(I2C_TDBE_FLAG, true)) {
        abortTransfer();
        return E_ERROR;
    }
    i2c_data_send(I2C2, reg);

    if (!waitFlag(I2C_TDBE_FLAG, true)) {
        abortTransfer();
        return E_ERROR;
    }
    i2c_data_send(I2C2, value);

    // TDC 表示最后一个字节传输完成，再生成 STOP。
    if (!waitFlag(I2C_TDC_FLAG, true)) {
        abortTransfer();
        return E_ERROR;
    }

    i2c_stop_generate(I2C2);
    return E_OK;
}

int I2c2Bus::readRegister(uint8_t address7, uint8_t reg, uint8_t& value)
{
    if (!initialized_ || address7 > 0x7Fu) {
        return E_PARAM;
    }

    if (!waitFlag(I2C_BUSYF_FLAG, false)) {
        return E_ERROR;
    }

    // 第一段：写入要读取的寄存器地址，但不发 STOP。
    if (!startAddress(address7, I2C_DIRECTION_TRANSMIT)) {
        abortTransfer();
        return E_ERROR;
    }
    i2c_flag_clear(I2C2, I2C_ADDR7F_FLAG);

    if (!waitFlag(I2C_TDBE_FLAG, true)) {
        abortTransfer();
        return E_ERROR;
    }
    i2c_data_send(I2C2, reg);

    if (!waitFlag(I2C_TDC_FLAG, true)) {
        abortTransfer();
        return E_ERROR;
    }

    // 第二段：重复 START，切换为读取方向。
    if (!startAddress(address7, I2C_DIRECTION_RECEIVE)) {
        abortTransfer();
        return E_ERROR;
    }

    /*
     * 只收一个字节：必须在清除地址标志前关闭 ACK，
     * 然后立刻生成 STOP。短暂关中断是为了防止任务/中断
     * 插入这三个紧邻的寄存器操作；原来的中断状态会恢复。
     */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    i2c_ack_enable(I2C2, FALSE);
    i2c_flag_clear(I2C2, I2C_ADDR7F_FLAG);
    i2c_stop_generate(I2C2);

    if (primask == 0u) {
        __enable_irq();
    }

    if (!waitFlag(I2C_RDBF_FLAG, true)) {
        abortTransfer();
        return E_ERROR;
    }

    value = i2c_data_receive(I2C2);
    i2c_ack_enable(I2C2, TRUE);
    return E_OK;
}
