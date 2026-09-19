#include "mpu6050.h"
#include "delay.h"

Mpu6050::Mpu6050(I2c2Bus& bus, uint8_t address7)
	: bus_(bus), address7_(address7), initialized_(false) {}


int Mpu6050::writeRegister(Register reg, uint8_t value) {

	return bus_.writeRegister(
		address7_, static_cast<uint8_t>(reg), value
	);
}

int Mpu6050::readRegister(Register reg, uint8_t& value)
{
    return bus_.readRegister(
        address7_,
        static_cast<uint8_t>(reg),
        value
    );
}

int Mpu6050::checkIdentity() {

	uint8_t who_am_i = 0u;

	const int result = readRegister(Register::WhoAmI, who_am_i);
	if (result != E_OK) {
		return result;
	}

	return who_am_i == kExpectedWhoAmI ? E_OK : E_ERROR;
}

int Mpu6050::init() {

	initialized_ = false;

	if (address7_ > 0x7Fu) {
		return E_PARAM;
	}

	int result = checkIdentity();
	if (result != E_OK) {
		return result;
	}

	/*
     * PWR_MGMT_1 bit7 写 1，执行设备复位。
     * 复位完成后该位由芯片自动清零。
     */
	result = writeRegister(Register::PowerManagement1, 0x80u);
	if (result != E_OK) {
		return result;
	}

	/* 数据手册要求设备复位后等待内部电路重新启动。 */
	delay_ms(100u);

	/*
     * 不使用固定延时，而是轮询复位位。
     * 这样驱动既可以在调度器启动前使用，也不依赖 FreeRTOS 延时。
     */
	bool reset_complete = false;

	for (uint32_t attempt = 0u; attempt < 1000u; ++attempt) {
		
		uint8_t power_management = 0u;

		result = readRegister(Register::PowerManagement1, power_management);

		if (result != E_OK) {
			/* 芯片复位期间可能短暂 NACK，继续轮询直到超时。 */
			continue;
		}

		if ((power_management & 0x80u) == 0u) {
			reset_complete = true;
			break;
		}
	}

	if (!reset_complete) {
		return E_ERROR;
	}

	/*
     * PWR_MGMT_1：
     * SLEEP = 0，解除睡眠；
     * CLKSEL = 1，使用 X 轴陀螺仪 PLL 作为时钟源。
     */
	result = writeRegister(Register::PowerManagement1, 0x01u);
	if (result != E_OK) {
		return result;
	}

	/* 等待 PLL 和传感器模拟电路稳定，再校验 SLEEP 位确实清除。 */
	delay_ms(10u);
	uint8_t power_management = 0u;
	result = readRegister(Register::PowerManagement1, power_management);
	if (result != E_OK || (power_management & 0x40u) != 0u) {
		return E_ERROR;
	}

	/*
     * CONFIG.DLPF_CFG = 3：
     * 对加速度和陀螺仪启用数字低通滤波。
     */
	result = writeRegister(Register::Configuration, 0x03u);
	if (result != E_OK) {
		return result;
	}

	/*
     * 启用 DLPF 后，陀螺仪内部输出频率为 1 kHz。
     *
     * SampleRate = 1000 / (1 + SMPLRT_DIV)
     * SMPLRT_DIV = 9，因此采样率为 100 Hz。
     */
    result = writeRegister(Register::SampleRateDivider, 9u);
    if (result != E_OK)
    {
        return result;
    }

    /*
     * GYRO_CONFIG.FS_SEL = 0：
     * 陀螺仪量程为 ±250 °/s，灵敏度为 131 LSB/(°/s)。
     */
    result = writeRegister(Register::GyroscopeConfig, 0x00u);
    if (result != E_OK)
    {
        return result;
    }

    /*
     * ACCEL_CONFIG.AFS_SEL = 0：
     * 加速度量程为 ±2 g，灵敏度为 16384 LSB/g。
     */
    result = writeRegister(Register::AccelerometerConfig, 0x00u);
    if (result != E_OK)
    {
        return result;
    }

    initialized_ = true;
    return E_OK;
}


int16_t Mpu6050::decodeInt16(uint8_t high, uint8_t low) {

	const uint16_t unsigned_value = (static_cast<uint16_t>(high) <<8u) |
									 static_cast<uint16_t>(low);

	return static_cast<int16_t>(unsigned_value);
}

int Mpu6050::readRaw(RawSample& sample) {

	if (!initialized_) {
		return E_ERROR;
	}

	/*
     * 从 ACCEL_XOUT_H 开始连续读取：
     *
     * 0～5：三轴加速度
     * 6～7：温度
     * 8～13：三轴陀螺仪
     */
	uint8_t data[14] = {};

	const int result = bus_.readRegisters(address7_,
										  static_cast<uint8_t>(Register::AccelXOutputHigh),
										  data, sizeof(data));
	

	if (result != E_OK) {
		return result;
	}

	sample.accel_x = decodeInt16(data[0], data[1]);
    sample.accel_y = decodeInt16(data[2], data[3]);
    sample.accel_z = decodeInt16(data[4], data[5]);

    sample.temperature = decodeInt16(data[6], data[7]);

    sample.gyro_x = decodeInt16(data[8], data[9]);
    sample.gyro_y = decodeInt16(data[10], data[11]);
    sample.gyro_z = decodeInt16(data[12], data[13]);

    return E_OK;
}

int Mpu6050::readSample(Sample& sample)
{
    RawSample raw{};

    const int result = readRaw(raw);
    if (result != E_OK)
    {
        return result;
    }

    sample.accel_x_g =
        static_cast<float>(raw.accel_x) / kAccelSensitivity;
    sample.accel_y_g =
        static_cast<float>(raw.accel_y) / kAccelSensitivity;
    sample.accel_z_g =
        static_cast<float>(raw.accel_z) / kAccelSensitivity;

    sample.temperature_c =
        static_cast<float>(raw.temperature) /
        kTemperatureSensitivity +
        kTemperatureOffset;

    sample.gyro_x_dps =
        static_cast<float>(raw.gyro_x) / kGyroSensitivity;
    sample.gyro_y_dps =
        static_cast<float>(raw.gyro_y) / kGyroSensitivity;
    sample.gyro_z_dps =
        static_cast<float>(raw.gyro_z) / kGyroSensitivity;

    return E_OK;
}
