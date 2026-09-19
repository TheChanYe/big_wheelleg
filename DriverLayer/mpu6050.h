#ifndef MPU6050_H
#define MPU6050_H

#include "i2c2_bus.h"

#include <stdint.h>


/**
 * @brief MPU6050 六轴惯性传感器驱动。
 *
 * 本类负责：
 * 1. 检查设备身份；
 * 2. 配置时钟、采样率、数字低通滤波器和量程；
 * 3. 读取三轴加速度、温度和三轴角速度；
 * 4. 将原始数据转换为 g、摄氏度和 °/s。
 *
 * 本阶段暂不负责零偏校准、滤波和姿态解算。
 */
class Mpu6050 final {

    public:
        /**
        * @brief MPU6050 原始测量数据。
        *
        * 数值来自芯片寄存器，均为有符号 16 位整数。
        */
        struct RawSample {
            int16_t accel_x; // X 轴加速度
            int16_t accel_y; // Y 轴加速度
            int16_t accel_z; // Z 轴加速度

            int16_t temperature; // 芯片温度传感器原始 ADC 数据

            int16_t gyro_x; // 三轴角速度 X 分量
            int16_t gyro_y; // 三轴角速度 Y 分量
            int16_t gyro_z; // 三轴角速度 Z 分量
        };

        /**
        * @brief 转换为物理单位后的测量数据。
        */
        struct Sample
        {
            float accel_x_g;
            float accel_y_g;
            float accel_z_g;

            float temperature_c;

            float gyro_x_dps;
            float gyro_y_dps;
            float gyro_z_dps;
        };

        /**
        * @param bus MPU6050 所连接的 I²C 总线。
        * @param address7 MPU6050 的 7 位地址，AD0 为低时是 0x68。
        */
        explicit Mpu6050(I2c2Bus& bus, uint8_t address7 = 0x68u);

        ~Mpu6050() = default;

        Mpu6050(const Mpu6050&) = delete;
        Mpu6050& operator=(const Mpu6050&) = delete;

        /**
         * @brief 初始化 MPU6050。
         *
         * 固定配置：
         * - 陀螺仪量程：±250 °/s
         * - 加速度量程：±2 g
         * - DLPF：配置 3
         * - 采样率：100 Hz
         */
        int init();

        /** 检查 WHO_AM_I 是否为 0x68 */
        int checkIdentity();

        /** 一次连续读取全部 14 字节原始数据 */
        int readRaw(RawSample& sample);

        /** 读取并转换为物理单位 */
        int readSample(Sample& sample);

        bool initialized() const {
            return initialized_;
        }

        uint8_t address() const {
            return address7_;
        }

    private:
        /**
         * MPU6050 寄存器地址。
         *
         * enum class 可以限制作用域，避免寄存器名称污染全局命名空间。
         */
        enum class Register : uint8_t
        {
            SampleRateDivider = 0x19u,
            Configuration     = 0x1Au,
            GyroscopeConfig   = 0x1Bu,
            AccelerometerConfig = 0x1Cu,

            AccelXOutputHigh  = 0x3Bu,

            PowerManagement1  = 0x6Bu,
            WhoAmI            = 0x75u
        };

        static constexpr uint8_t kExpectedWhoAmI = 0x68u;

        /* ±2 g 时每 g 对应 16384 LSB。 */
        static constexpr float kAccelSensitivity = 16384.0f;

        /* ±250 °/s 时每 °/s 对应 131 LSB。 */
        static constexpr float kGyroSensitivity = 131.0f;

        /* MPU6050 数据手册规定的温度换算参数。 */
        static constexpr float kTemperatureSensitivity = 340.0f;
        static constexpr float kTemperatureOffset = 36.53f;

        I2c2Bus& bus_;
        uint8_t address7_;
        bool initialized_;

        int writeRegister(Register reg, uint8_t value);
        int readRegister(Register reg, uint8_t& value);

        /**
         * @brief 将两个大端字节组合为 int16_t。
         *
         * MPU6050 所有测量寄存器都是高字节在前。
         */
        static int16_t decodeInt16(uint8_t high, uint8_t low);
};





#endif
