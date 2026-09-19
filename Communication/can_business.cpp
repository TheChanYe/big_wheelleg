/**
 * @file can_business.cpp
 * @brief 双电机 CAN 命令处理和遥测业务实现。
 *
 * 本文件只迁移业务状态和函数组织方式，不修改任何 CAN ID、
 * 字节排列、缩放倍率、发送周期或命令超时阈值。
 */

#include "can_business.h"
#include "my_can.h"
#include "safety_limit.h"
#include "motor_fault.h"
#include "motor_adc.h"
#include "drv_fault.h"
#include "motor_service.h"

/*
 * g_motor1/g_motor2 定义在 motor_service.cpp 中，但继续使用 C linkage，
 * 以便尚未迁移的 ADC、FOC 和故障模块共享相同的 Motor_Data。
 */
extern "C" {
extern Motor_Data g_motor1;
extern Motor_Data g_motor2;

/*
 * 下列控制器接口仍由 foc.c 实现。
 * 不直接包含 foc.h，避免引入尚未兼容 C++ 的旧驱动头文件。
 */
int Motor_Reset_Current_Controller(Motor_Data *motor);
int Motor_Reset_Speed_Controller(Motor_Data *motor);

uint8_t Motor_Is_Speed_Startup_Boost_Active(
    const Motor_Data *motor);
}

#define CAN_CMD_TIMEOUT_MS       100u
#define CAN_CMD_STALE_MS          20u

namespace
{
/*
 * 临时注册指针供 C ABI getter 使用。
 * 该指针不拥有对象，不能 delete。
 */
CanBusiness *g_can_business = nullptr;
}


CanBusiness::CanBusiness()
    : wheel_command_{},
      speed_command_{},
      motor_mode_(CAN_MOTOR_MODE_CURRENT),  
      comm_state_(CAN_COMM_TIMEOUT),
      wheel_sequence_{},
      speed_sequence_{},
      last_new_command_tick_(0),
      command_active_(0u),
      telemetry_tx_fail_count_(0u),
      tx_test_counter_(0u)
{
    /**
     * 构造函数只初始化软件状态，步初始化 CAN 外设
     * 底层硬件初始化仍由 init() 在 CAN 任务启动完成
     */

    g_can_business = this;
}

CanBusiness::~CanBusiness() {
    /*
     * CAN 外设由 MCU 和底层驱动持有，不在析构函数中复位。
     * 正常固件运行期间该析构函数不会执行。
    */
    
    if (g_can_business == this) {
        g_can_business = nullptr;
    }
}

int CanBusiness::init() {
    const int result = my_can_init();

    if (result == E_OK) {
        log_inform(
            "CAN business task started (RX 0x101/0x102)"
        );
    }

    return result;
}

void CanBusiness::process() {
    uint16_t id;
    uint8_t data[CAN_CMD_DLC];
    uint8_t len;

    /*
     * 每个 1ms 周期清空接收 FIFO，保证命令接收优先于遥测发送。
    */
    while (my_can_receive_std(&id, data, &len) == E_OK) {
        processFrame(id, data, len);
    }

    updateTimeout();

    /*
     * 保持原来的 1ms phase 调度，同一周期最多发送一个遥测帧。
     */
    sendTelemetry(xTaskGetTickCount());
}

void CanBusiness::processTxTest()
{
    uint8_t data[CAN_CMD_DLC];

    data[0] = 0xCAu;
    data[1] = 0x4Eu;
    data[2] = static_cast<uint8_t>(
        tx_test_counter_ & 0xFFu);
    data[3] = static_cast<uint8_t>(
        (tx_test_counter_ >> 8) & 0xFFu);
    data[4] = 0x11u;
    data[5] = 0x22u;
    data[6] = 0x33u;
    data[7] = 0x44u;

    if (my_can_send_std(
            CAN_TEST_ID,
            data,
            CAN_TEST_DLC) != E_OK)
    {
        log_error(
            "CAN TX failed (counter=%u)",
            tx_test_counter_);
    }

    ++tx_test_counter_;
}

void CanBusiness::processLoopbackTest() {
    uint16_t id;
    uint8_t data[CAN_CMD_DLC];
    uint8_t len;

    if (my_can_receive_std(&id, data, &len) == E_OK
        && id == 0x124u)
    {
        my_can_send_std(0x125u, data, len);
    }
}

int16_t CanBusiness::readI16Le(const uint8_t *data) {
    return static_cast<int16_t>(
        static_cast<uint16_t>(data[0])
        | (static_cast<uint16_t>(data[1]) << 8)
    );
}

uint16_t CanBusiness::readU16Le(const uint8_t *data) {
    return static_cast<uint16_t>(
        static_cast<uint16_t>(data[0])
        | (static_cast<uint16_t>(data[1]) << 8)
    );
}

int16_t CanBusiness::toI16Saturated(float value)
{
    if (value > 32767.0f)
    {
        return 32767;
    }

    if (value < -32768.0f)
    {
        return -32768;
    }

    return static_cast<int16_t>(value);
}

bool CanBusiness::acceptSequence(
    CanSequenceStats& stats,
    uint16_t sequence)
{
    if (stats.valid == 0u) {
        stats.last_sequence = sequence;
        stats.valid = 1u;
        return true;
    }

    const uint16_t delta = 
        static_cast<uint16_t>(
            sequence - stats.last_sequence
        );

    if (delta == 0u) {
        ++stats.duplicate_count;
        return false;
    }
    if (delta >= 0x8000u) {
        ++stats.out_of_order_count;
        return false;
    }

    if (delta > 1u) {
        stats.drop_count +=
            static_cast<uint32_t>(delta - 1u);
    }

    stats.last_sequence = sequence;
    return true;
}
/**
 *  获取当前激活的 CAN 命令序列号
 */
uint16_t CanBusiness::activeSequence() const {
    return (motor_mode_ == CAN_MOTOR_MODE_SPEED)
        ? speed_command_.sequence
        : wheel_command_.sequence;
}
/**
 *  接受新的 CAN 命令
 */
void CanBusiness::acceptNewCommand() {
    last_new_command_tick_ = xTaskGetTickCount();
    comm_state_ = CAN_COMM_OK;
    command_active_ = 1u;
}
/**
 *  重置电机控制器
 */
void CanBusiness::resetControllers() {
    Motor_Reset_Current_Controller(&g_motor1);
    Motor_Reset_Current_Controller(&g_motor2);
    Motor_Reset_Speed_Controller(&g_motor1);
    Motor_Reset_Speed_Controller(&g_motor2);
}
/**
 *  禁用当前的 CAN 命令
 */
void CanBusiness::disableCommand() {
    /*
     * 保持原行为：关闭任何命令后切回电流模式，并强制输出归零。
    */
    speed_command_.motor0_speed_ref = 0.0f;
    speed_command_.motor1_speed_ref = 0.0f;
    motor_mode_ = CAN_MOTOR_MODE_CURRENT;

    SafetyLimit_ForceZeroAll();

    if (command_active_ != 0u) {
        resetControllers();
        command_active_ = 0u;
    }
}

void CanBusiness::processFaultClear(uint8_t flags) {
    if ((flags & CAN_CMD_FLAG_CLEAR_MOTOR0) != 0u) {
        MotorService_ClearFault(0u);
    }

    if ((flags & CAN_CMD_FLAG_CLEAR_MOTOR1) != 0u) {
        MotorService_ClearFault(1u);
    }
}

/**
 * @brief 向主控回复一帧电流命令确认帧。
 * @param command 已解析的 0x101 电流命令。
 *
 * ACK 保持原协议格式：
 * data[0..1]：Motor0 原始电流命令
 * data[2..3]：Motor1 原始电流命令
 * data[4..5]：命令序号
 * data[6]：使能状态
 * data[7]：保留
 *
 * 原代码不统计 ACK 发送失败，因此这里继续保持该行为。
 */
void CanBusiness::sendAck(const WheelCommand& command) {
    uint8_t data[CAN_CMD_DLC];

    data[0] = static_cast<uint8_t>(
        command.motor0_iq_raw & 0xFF);
    data[1] = static_cast<uint8_t>(
        (command.motor0_iq_raw >> 8) & 0xFF);

    data[2] = static_cast<uint8_t>(
        command.motor1_iq_raw & 0XFF);
    data[3] = static_cast<uint8_t>(
        (command.motor1_iq_raw >> 8) & 0xFF);
    
    data[4] = static_cast<uint8_t>(
        command.sequence & 0xFFu);
    data[5] = static_cast<uint8_t>(
        (command.sequence >> 8) & 0xFFu);
    
    data[6] = command.enable;
    data[7] = 0x00u;

    my_can_send_std(
        CAN_ID_WHEEL_ACK,
        data,
        CAN_CMD_DLC
    );
}

/**
 * @brief 解析一帧 CAN 电机命令。
 * @param id 标准帧 ID。
 * @param data CAN 数据区。
 * @param len 数据长度。
 * @return E_OK 表示识别并处理；CAN_RX_EMPTY 表示不是目标命令；
 *         E_PARAM 表示数据指针无效。
 *
 * 本函数只接受：
 * - 0x101：双电机电流命令；
 * - 0x102：双电机速度命令；
 * - DLC 必须等于 8。
 */
int CanBusiness::processFrame(
    uint16_t id,
    const uint8_t *data,
    uint8_t len) {
    
    if (data == nullptr) {
        return E_PARAM;
    }

    if ((id != CAN_ID_WHEEL_COMMAND 
        && id != CAN_ID_MOTOR_SPEED_COMMAND)
        || len != CAN_CMD_DLC) {
        
        return CAN_RX_EMPTY;
    }

    if (id == CAN_ID_WHEEL_COMMAND) {
        WheelCommand command{};
        /*
         * 0x101 数据格式：
         * byte 0~1：Motor0 电流，单位 0.01A；
         * byte 2~3：Motor1 电流，单位 0.01A；
         * byte 4~5：16 位命令序号；
         * byte 6 bit0：命令使能；
         * byte 7：故障清除位。
         */
        command.motor0_iq_raw = readI16Le(&data[0]);
        command.motor1_iq_raw = readI16Le(&data[2]);
        command.sequence = readU16Le(&data[4]);

        command.enable = 
            ((data[6] & CAN_CMD_FLAG_ENABLE) != 0u)
                ? 1u
                : 0u;

        command.motor0_iq_ref = 
            static_cast<float>(command.motor0_iq_raw) * 0.01f;
        
        command.motor1_iq_ref = 
            static_cast<float>(command.motor1_iq_raw) * 0.01f;

        if (command.enable == 0u) {
            /*
             * 禁用命令不经过序号过滤，必须立即关闭当前输出。
             */
            wheel_command_ = command;
            motor_mode_ = CAN_MOTOR_MODE_CURRENT;

            disableCommand();
            processFaultClear(data[7]);

            /*
             * 主动禁用不是通信超时，因此通信状态仍记为正常。
             */
            comm_state_ = CAN_COMM_OK;
        } else if (acceptSequence(
                        wheel_sequence_,
                        command.sequence
        )) {
            /*
             * 只有新序号命令才更新当前控制目标和超时计时。
             */
            wheel_command_ = command;
            motor_mode_ = CAN_MOTOR_MODE_CURRENT;
            acceptNewCommand();
        }
        /*
         * 即使命令重复或乱序，也保持原行为发送 ACK。
         */
        sendAck(command);

        return E_OK;
    }

    /*
     * 到达这里说明收到的是 0x102 速度命令。
     */
    MotorSpeedCommand command{};

    /*
     * 0x102 数据格式：
     * byte 0~1：Motor0 速度，单位 0.1rad/s；
     * byte 2~3：Motor1 速度，单位 0.1rad/s；
     * byte 4~5：16 位命令序号；
     * byte 6 bit0：命令使能；
     * byte 7：故障清除位。
     */
    command.motor0_speed_raw = readI16Le(&data[0]);
    command.motor1_speed_raw = readI16Le(&data[2]);
    command.sequence = readU16Le(&data[4]);

    command.enable = 
        ((data[6] & CAN_CMD_FLAG_ENABLE) != 0u)
            ? 1u
            : 0u;
    
    command.motor0_speed_ref = 
        static_cast<float>(command.motor0_speed_raw) * 0.1f;

    command.motor1_speed_ref = 
        static_cast<float>(command.motor1_speed_raw) * 0.1f;

    if (command.enable == 0u) {
        /*
         * 保持原程序顺序：
         * 先记录速度命令和速度模式，再调用 disableCommand()。
         * disableCommand() 最终会把控制模式切回电流模式。
         */
        speed_command_ = command;
        motor_mode_ = CAN_MOTOR_MODE_SPEED;

        disableCommand();
        processFaultClear(data[7]);
        
        comm_state_ = CAN_COMM_OK;
    } else if (acceptSequence(
                    speed_sequence_,
                    command.sequence
    )) {
        speed_command_ = command;
        motor_mode_ = CAN_MOTOR_MODE_SPEED;

        /*
         * 速度目标变为 0 时清空对应电机的速度控制器状态，
         * 防止下次启动继续使用旧积分值。
         */
        if (speed_command_.motor0_speed_raw == 0.0f) {
            Motor_Reset_Speed_Controller(&g_motor1);
        }

        if (speed_command_.motor1_speed_raw == 0.0f) {
            Motor_Reset_Speed_Controller(&g_motor2);
        }

        acceptNewCommand();
    }

    return E_OK;
}
/**
 * @brief 检查命令是否陈旧或超时。
 *
 * 20ms 以上标记为 STALE；
 * 100ms 以上关闭命令并标记为 TIMEOUT。
 *
 * TickType_t 的无符号减法能够正确处理系统 tick 回绕。
 */
void CanBusiness::updateTimeout() {
    if (command_active_ == 0u) {
        return;
    }

    const TickType_t elapsed =
        xTaskGetTickCount() - last_new_command_tick_;
    
    if (elapsed >= pdMS_TO_TICKS(CAN_CMD_TIMEOUT_MS)) {
        disableCommand();
        comm_state_ = CAN_COMM_TIMEOUT;
    } else if (elapsed >= pdMS_TO_TICKS(CAN_CMD_STALE_MS)) {
        comm_state_ = CAN_COMM_STALE;
    } else {
        comm_state_ = CAN_COMM_OK;
    }
}

/**
 * @brief 记录遥测帧发送失败次数。
 *
 * 当前计数只用于系统诊断状态位，不在运行期间清零。
 */
void CanBusiness::recordTransmitResult(int result) {
    if (result != E_OK) {
        ++telemetry_tx_fail_count_;
    }
}

/**
 * @brief 发送单个电机的速度状态帧。
 *
 * 数据格式保持原协议：
 * byte 0~1：目标速度，0.1rad/s；
 * byte 2~3：实际速度，0.1rad/s；
 * byte 4~5：当前控制模式对应的命令序号；
 * byte 6：电机运行状态；
 * byte 7 bit0：命令有效；
 * byte 7 bit1：当前为速度模式。
 */
void CanBusiness::sendSpeedState(
    uint16_t id,
    Motor_Data& motor,
    float target) 
{
    uint8_t data[CAN_CMD_DLC];

    const float state_target = 
        (motor_mode_ == CAN_MOTOR_MODE_SPEED)
            ? target
            : 0.0f;
    const uint16_t sequence = activeSequence();

    const int16_t command_raw = 
        toI16Saturated(state_target * 10.0f);
    const int16_t measured_raw = 
        toI16Saturated(motor.velocity * 10.0f);

    data[0] = static_cast<uint8_t>(
        command_raw & 0xFF
    );
    data[1] = static_cast<uint8_t>(
        (command_raw >> 8) & 0xFF
    );

    data[2] = static_cast<uint8_t>(
        measured_raw & 0xFF
    );
    data[3] = static_cast<uint8_t>(
        (measured_raw >> 8) & 0xFF
    );

    data[4] = static_cast<uint8_t>(
        sequence & 0xFFu
    );
    data[5] = static_cast<uint8_t>(
        (sequence >> 8) & 0xFFu
    );

    data[6] = motor.run_state;

    data[7] = 
        ((command_active_ != 0u) ? 0x01 : 0x00u)
        | ((motor_mode_ == CAN_MOTOR_MODE_SPEED)
                ? 0x02u
                : 0x00u);
    
    recordTransmitResult(
        my_can_send_std(id, data, CAN_CMD_DLC)
    );
}

/**
 * @brief 发送单个电机的速度控制诊断帧。
 *
 * byte 0~1：Iq 目标值，单位 0.001A；
 * byte 2~3：Iq 反馈值，单位 0.001A；
 * byte 4~5：Vq，缩放 1000；
 * byte 6 bit0：速度启动增强正在工作；
 * byte 7 bit0：速度启动失败。
 */
void CanBusiness::sendSpeedDiagnostic(
    uint16_t id,
    Motor_Data& motor
) {
    uint8_t data[CAN_CMD_DLC];

    const int16_t iq_target_raw = 
        toI16Saturated(
            motor.control.iq_current_target * 1000.0f);
    const int16_t iq_feedback_raw = 
        toI16Saturated(
            motor.control.iq_current_feedback * 1000.0f);
    
    const int16_t vq_raw =
        toI16Saturated(
            motor.voltage_dq.Vq * 1000.0f);
    
    data[0] = static_cast<uint8_t>(
        iq_target_raw & 0xFF
    );
    data[1] = static_cast<uint8_t>(
        (iq_target_raw >> 8) & 0xFF
    );

    data[2] = static_cast<uint8_t>(
        iq_feedback_raw & 0xFF
    );
    data[3] = static_cast<uint8_t>(
        (iq_feedback_raw >> 8) & 0xFF
    );

    data[4] = static_cast<uint8_t>(
        vq_raw & 0XFF
    );
    data[5] = static_cast<uint8_t>(
        (vq_raw >> 8) & 0xFF
    );

    data[6] = (Motor_Is_Speed_Startup_Boost_Active(&motor) != 0u)
                ? 0x01u
                : 0x00u;
    data[7] = (motor.control.speed_startup_failed != 0u)
                ? 0x01u
                : 0x00u;
    
    recordTransmitResult(
        my_can_send_std(id, data, CAN_CMD_DLC)
    );
}

/**
 * @brief 发送温度、母线电压和低 16 位故障信息。
 */
void CanBusiness::sendSafetyDiagnostic() {
    uint8_t data[CAN_CMD_DLC];

    const uint16_t bus_raw = static_cast<uint16_t>(
        toI16Saturated(Motor_ADC_GetBusVoltage() * 100.0f)
    );
    const MotorFaultBits fault_bits = MotorFault_GetBits();

    const int16_t temp0_raw = toI16Saturated(g_motor1.mos_temp * 10.0f);
    const int16_t temp1_raw = toI16Saturated(g_motor2.mos_temp * 10.0f);

    data[0] = static_cast<uint8_t>(
        temp0_raw & 0xFF
    );
    data[1] = static_cast<uint8_t>(
        (temp0_raw >> 8) & 0xFF
    );

    data[2] = static_cast<uint8_t>(
        temp1_raw & 0xFF
    );
    data[3] = static_cast<uint8_t>(
        (temp1_raw >> 8) & 0XFF
    );

    data[4] = static_cast<uint8_t>(
        bus_raw & 0xFFu
    );
    data[5] = static_cast<uint8_t>(
        (bus_raw >> 8) & 0xFFu
    );

    data[6] = static_cast<uint8_t>(
        fault_bits & 0xFFu
    );
    data[7] = static_cast<uint8_t>(
        (fault_bits >> 8) & 0xFFu
    );

    recordTransmitResult(my_can_send_std(
                            CAN_ID_SAFETY_DIAG,
                            data,
                            CAN_CMD_DLC));
}

/**
 * @brief 发送两路 DRV8301 的状态寄存器。
 */
void CanBusiness::sendDriverDiagnostic() {
    uint8_t data[CAN_CMD_DLC];

    const uint16_t status0_1 = DrvFault_GetStatus1(0u);
    const uint16_t status0_2 = DrvFault_GetStatus2(0u);
    const uint16_t status1_1 = DrvFault_GetStatus1(1u);
    const uint16_t status1_2 = DrvFault_GetStatus2(1u);

    data[0] = static_cast<uint8_t>(status0_1 & 0xFFu);
    data[1] = static_cast<uint8_t>(status0_1 >> 8);

    data[2] = static_cast<uint8_t>(status0_2 & 0xFFu);
    data[3] = static_cast<uint8_t>(status0_2 >> 8);

    data[4] = static_cast<uint8_t>(status1_1 & 0xFFu);
    data[5] = static_cast<uint8_t>(status1_1 >> 8);

    data[6] = static_cast<uint8_t>(status1_2 & 0xFFu);
    data[7] = static_cast<uint8_t>(status1_2 >> 8);

    recordTransmitResult(my_can_send_std(
                            CAN_ID_DRV_DIAG,
                            data,
                            CAN_CMD_DLC)
    );
}

/**
 * @brief 发送系统通信及故障状态。
 *
 * byte 0~3：完整 32 位故障位；
 * byte 4：CAN 通信状态；
 * byte 5：Motor0 运行状态；
 * byte 6：Motor1 运行状态；
 * byte 7：系统状态标志。
 */
void CanBusiness::sendSystemDiagnostic() {
    uint8_t data[CAN_CMD_DLC];

    const MotorFaultBits fault_bits = MotorFault_GetBits();

    data[0] = static_cast<uint8_t>(
        fault_bits & 0xFFu
    );
    data[1] = static_cast<uint8_t>(
        (fault_bits >> 8) & 0xFFu
    );
    data[2] = static_cast<uint8_t>(
        (fault_bits >> 16) & 0xFFu
    );
    data[3] = static_cast<uint8_t>(
        (fault_bits >> 24) & 0xFFu
    );

    data[4] = static_cast<uint8_t>(comm_state_);
    data[5] = g_motor1.run_state;
    data[6] = g_motor2.run_state;

    data[7] = ((command_active_ != 0u) ? 0x01u : 0x00u)
                | ((motor_mode_ == CAN_MOTOR_MODE_SPEED)
                        ? 0x02u
                        : 0x00u)
                | ((Motor_ADC_BusVoltageValid() != 0u)
                        ? 0x04u
                        : 0x00u)
                | ((telemetry_tx_fail_count_ != 0u)
                        ? 0x08u
                        : 0x00u
    );

    recordTransmitResult(my_can_send_std(
                            CAN_ID_SYSTEM_DIAG,
                            data,
                            CAN_CMD_DLC
    ));
}

/**
 * @brief 发送轮子位置、速度和电流状态。
 *
 * 方向系数根据电机定时器确定，保持原有左右轮符号定义。
 */
void CanBusiness::sendWheelState(
    uint16_t id,
    Motor_Data& motor)
{
    uint8_t data[CAN_CMD_DLC];

    const float direction =
        (motor.tmr == TMR1)
            ? MOTOR0_COMMAND_DIRECTION
            : MOTOR1_COMMAND_DIRECTION;

    const int32_t position_raw =
        static_cast<int32_t>(
            (motor.filter_angle
             + static_cast<float>(motor.circle_num) * cpr)
            * direction
            * 1000.0f);

    const int16_t velocity_raw =
        toI16Saturated(
            motor.velocity * direction * 10.0f);

    const int16_t iq_raw =
        toI16Saturated(
            motor.control.iq_current_feedback
            * direction
            * 100.0f);

    data[0] = static_cast<uint8_t>(
        position_raw & 0xFF);
    data[1] = static_cast<uint8_t>(
        (position_raw >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>(
        (position_raw >> 16) & 0xFF);
    data[3] = static_cast<uint8_t>(
        (position_raw >> 24) & 0xFF);

    data[4] = static_cast<uint8_t>(
        velocity_raw & 0xFF);
    data[5] = static_cast<uint8_t>(
        (velocity_raw >> 8) & 0xFF);

    data[6] = static_cast<uint8_t>(
        iq_raw & 0xFF);
    data[7] = static_cast<uint8_t>(
        (iq_raw >> 8) & 0xFF);

    recordTransmitResult(
        my_can_send_std(id, data, CAN_CMD_DLC));
}

/**
 * @brief 根据系统 tick 的低两位十进制相位发送遥测。
 *
 * 保持原有策略：
 * - 每 10ms 发送两路轮状态；
 * - 每 20ms 发送两路速度状态和速度诊断；
 * - 100ms 周期内发送一次安全、驱动和系统诊断；
 * - 单次 process() 最多发送一帧遥测。
 */
void CanBusiness::sendTelemetry(TickType_t now)
{
    switch (static_cast<uint32_t>(now) % 100u)
    {
    case 0u:
    case 10u:
    case 20u:
    case 30u:
    case 40u:
    case 50u:
    case 60u:
    case 70u:
    case 80u:
    case 90u:
        sendWheelState(
            CAN_ID_MOTOR0_WHEEL_STATE,
            g_motor1);
        break;

    case 5u:
    case 15u:
    case 25u:
    case 35u:
    case 45u:
    case 55u:
    case 65u:
    case 75u:
    case 85u:
    case 95u:
        sendWheelState(
            CAN_ID_MOTOR1_WHEEL_STATE,
            g_motor2);
        break;

    case 2u:
    case 22u:
    case 42u:
    case 62u:
    case 82u:
        sendSpeedState(
            CAN_ID_MOTOR0_SPEED_STATE,
            g_motor1,
            speed_command_.motor0_speed_ref);
        break;

    case 7u:
    case 27u:
    case 47u:
    case 67u:
    case 87u:
        sendSpeedState(
            CAN_ID_MOTOR1_SPEED_STATE,
            g_motor2,
            speed_command_.motor1_speed_ref);
        break;

    case 3u:
    case 23u:
    case 43u:
    case 63u:
    case 83u:
        sendSpeedDiagnostic(
            CAN_ID_MOTOR0_SPEED_DIAG,
            g_motor1);
        break;

    case 8u:
    case 28u:
    case 48u:
    case 68u:
    case 88u:
        sendSpeedDiagnostic(
            CAN_ID_MOTOR1_SPEED_DIAG,
            g_motor2);
        break;

    case 1u:
        sendSafetyDiagnostic();
        break;

    case 4u:
        sendDriverDiagnostic();
        break;

    case 6u:
        sendSystemDiagnostic();
        break;

    default:
        break;
    }
}

/*
 * 以下是只读状态接口。
 */

const WheelCommand& CanBusiness::wheelCommand() const
{
    return wheel_command_;
}

float CanBusiness::motor0IqOutput() const
{
    return wheel_command_.motor0_iq_ref;
}

float CanBusiness::motor1IqOutput() const
{
    return wheel_command_.motor1_iq_ref;
}

bool CanBusiness::currentCommandIsActive() const
{
    return command_active_ != 0u
        && motor_mode_ == CAN_MOTOR_MODE_CURRENT;
}

CanMotorMode CanBusiness::motorMode() const
{
    return motor_mode_;
}

CanCommState CanBusiness::commState() const
{
    return comm_state_;
}

const CanSequenceStats&
CanBusiness::wheelSequenceStats() const
{
    return wheel_sequence_;
}

const CanSequenceStats&
CanBusiness::speedSequenceStats() const
{
    return speed_sequence_;
}

float CanBusiness::motor0SpeedTarget() const
{
    return speed_command_.motor0_speed_ref;
}

float CanBusiness::motor1SpeedTarget() const
{
    return speed_command_.motor1_speed_ref;
}

/*
 * 以下是临时 C ABI 桥接。
 *
 * MotorService 和可能存在的 C 测试代码仍通过这些函数访问唯一的
 * CanBusiness 对象。通信相关模块迁移完成后再评估删除。
 */

extern "C" int can_business_init(void)
{
    return (g_can_business != nullptr)
        ? g_can_business->init()
        : E_ERROR;
}

extern "C" void can_business_process(void)
{
    if (g_can_business != nullptr)
    {
        g_can_business->process();
    }
}

extern "C" void can_business_tx_test_process(void)
{
    if (g_can_business != nullptr)
    {
        g_can_business->processTxTest();
    }
}

extern "C" void can_business_loopback_test_process(void)
{
    if (g_can_business != nullptr)
    {
        g_can_business->processLoopbackTest();
    }
}

extern "C" int can_business_process_frame(
    uint16_t id,
    const uint8_t *data,
    uint8_t len)
{
    return (g_can_business != nullptr)
        ? g_can_business->processFrame(id, data, len)
        : E_ERROR;
}

extern "C" const WheelCommand *
can_business_get_wheel_command(void)
{
    return (g_can_business != nullptr)
        ? &g_can_business->wheelCommand()
        : nullptr;
}

extern "C" float
can_business_get_motor0_iq_output(void)
{
    return (g_can_business != nullptr)
        ? g_can_business->motor0IqOutput()
        : 0.0f;
}

extern "C" float
can_business_get_motor1_iq_output(void)
{
    return (g_can_business != nullptr)
        ? g_can_business->motor1IqOutput()
        : 0.0f;
}

extern "C" uint8_t
can_business_current_command_is_active(void)
{
    return (g_can_business != nullptr
            && g_can_business->currentCommandIsActive())
        ? 1u
        : 0u;
}

extern "C" CanMotorMode
can_business_get_motor_mode(void)
{
    return (g_can_business != nullptr)
        ? g_can_business->motorMode()
        : CAN_MOTOR_MODE_CURRENT;
}

extern "C" CanCommState
can_business_get_comm_state(void)
{
    return (g_can_business != nullptr)
        ? g_can_business->commState()
        : CAN_COMM_TIMEOUT;
}

extern "C" const CanSequenceStats *
can_business_get_wheel_sequence_stats(void)
{
    return (g_can_business != nullptr)
        ? &g_can_business->wheelSequenceStats()
        : nullptr;
}

extern "C" const CanSequenceStats *
can_business_get_speed_sequence_stats(void)
{
    return (g_can_business != nullptr)
        ? &g_can_business->speedSequenceStats()
        : nullptr;
}

extern "C" float
can_business_get_motor0_speed_target(void)
{
    return (g_can_business != nullptr)
        ? g_can_business->motor0SpeedTarget()
        : 0.0f;
}

extern "C" float
can_business_get_motor1_speed_target(void)
{
    return (g_can_business != nullptr)
        ? g_can_business->motor1SpeedTarget()
        : 0.0f;
}