/**
 * @file can_business.h
 * @brief 双电机 CAN 命令、通信状态和遥测业务接口。
 *
 * 底层 CAN 外设收发仍由 my_can.c 实现，本模块只负责：
 * - 解析电流和速度命令；
 * - 检查命令序号；
 * - 处理命令超时；
 * - 执行故障清除命令；
 * - 按固定相位发送遥测数据。
 */

#ifndef CAN_BUSINESS_H
#define CAN_BUSINESS_H

#include "main.h"
#include "foc_cfg.h"

/* 接收命令帧。 */
#define CAN_ID_WHEEL_COMMAND           0x101u
#define CAN_ID_MOTOR_SPEED_COMMAND     0x102u

/* 发送状态帧。 */
#define CAN_ID_WHEEL_ACK               0x201u
#define CAN_ID_MOTOR0_SPEED_STATE      0x202u
#define CAN_ID_MOTOR0_SPEED_DIAG       0x203u
#define CAN_ID_MOTOR1_SPEED_STATE      0x204u
#define CAN_ID_MOTOR1_SPEED_DIAG       0x205u
#define CAN_ID_SAFETY_DIAG             0x206u
#define CAN_ID_DRV_DIAG                0x207u
#define CAN_ID_MOTOR0_WHEEL_STATE      0x208u
#define CAN_ID_MOTOR1_WHEEL_STATE      0x209u
#define CAN_ID_SYSTEM_DIAG             0x20Au

#define CAN_CMD_DLC                    8u

/* data[6] 的控制使能位。 */
#define CAN_CMD_FLAG_ENABLE            0x01u

/* data[7] 的故障清除位。 */
#define CAN_CMD_FLAG_CLEAR_MOTOR0      0x02u
#define CAN_CMD_FLAG_CLEAR_MOTOR1      0x04u

/**
 * @brief 0x101 电流命令解析结果。
 */
typedef struct {
    int16_t motor0_iq_raw;
    int16_t motor1_iq_raw;

    float motor0_iq_ref;
    float motor1_iq_ref;

    uint16_t sequence;
    uint8_t enable;
} WheelCommand;

/**
 * @brief 0x102 速度命令解析结果
 */
typedef struct {
    int16_t motor0_speed_raw;
    int16_t motor1_speed_raw;

    float motor0_speed_ref;
    float motor1_speed_ref;

    uint16_t sequence;
    uint8_t enable;
} MotorSpeedCommand;

/**
 * @brief 当前生效的电机控制命令类型。
 */
typedef enum {
    CAN_MOTOR_MODE_CURRENT = 0,
    CAN_MOTOR_MODE_SPEED
} CanMotorMode;

/**
 * @brief CAN 命令链路状态
 */
typedef enum {
    CAN_COMM_OK = 0,
    CAN_COMM_STALE,
    CAN_COMM_TIMEOUT
} CanCommState;

/**
 * @brief 单类命令帧的序号诊断数据
 */
typedef struct {
    uint16_t last_sequence;
    uint32_t duplicate_count;
    uint32_t drop_count;
    uint32_t out_of_order_count;
    uint8_t valid;
} CanSequenceStats;

#ifdef __cplusplus

/**
 * @brief CAN 业务管理对象。
 *
 * 固件中只创建一个 CanBusiness 对象，由 TaskManager 持有。
 * 类不拥有底层 CAN 外设，也不直接操作 CAN 寄存器。
 */
class CanBusiness final {
    public:
        CanBusiness();
        ~CanBusiness();

        CanBusiness(const CanBusiness&) = delete;
        CanBusiness& operator=(const CanBusiness&) = delete;

        /** 初始化底层CAN外设 */
        int init();

        /**
        * 执行一次 CAN 业务周期：
        * 1. 清空接收 FIFO；
        * 2. 更新命令超时；
        * 3. 按当前 tick 相位发送至多一个遥测帧。
        */
        void process();

        /** 执行一次 CAN 发送测试 */
        void processTxTest();

        /** 执行一次 CAN 回环测试 */
        void processLoopbackTest();

        /** 解析一帧标准 CAN 命令 */
        int processFrame(
            uint16_t id,
            const uint8_t *data,
            uint8_t len
        );

        const WheelCommand& wheelCommand() const;

        float motor0IqOutput() const;
        float motor1IqOutput() const;

        bool currentCommandIsActive() const;

        CanMotorMode motorMode() const;
        CanCommState commState() const;

        const CanSequenceStats& wheelSequenceStats() const;
        const CanSequenceStats& speedSequenceStats() const;

        float motor0SpeedTarget() const;
        float motor1SpeedTarget() const;
    
    private:
        /* 小端字节序转换工具*/
        static int16_t readI16Le(const uint8_t *data);
        static uint16_t readU16Le(const uint8_t *data);

        /* 将浮点遥测值饱和到 int16_t */
        static int16_t toI16Saturated(float value);

        bool acceptSequence(
            CanSequenceStats& stats,
            uint16_t sequence
        );

        uint16_t activeSequence() const;

        void acceptNewCommand();
        void disableCommand();
        void resetControllers();
        void processFaultClear(uint8_t flags);
        void updateTimeout();

        void sendAck(const WheelCommand& command);
        void sendTelemetry(TickType_t now);

        void sendSpeedState(
            uint16_t id,
            Motor_Data& motor,
            float target
        );

        void sendSpeedDiagnostic(
            uint16_t id,
            Motor_Data& motor
        );

        void sendSafetyDiagnostic();
        void sendDriverDiagnostic();
        void sendSystemDiagnostic();

        void sendWheelState(
            uint16_t id,
            Motor_Data& motor
        );

        void recordTransmitResult(int result);

        WheelCommand wheel_command_;
        MotorSpeedCommand speed_command_;

        CanMotorMode motor_mode_;
        CanCommState comm_state_;

        CanSequenceStats wheel_sequence_;
        CanSequenceStats speed_sequence_;

        TickType_t last_new_command_tick_;

        uint8_t command_active_;
        uint32_t telemetry_tx_fail_count_;
        uint16_t tx_test_counter_;
};

/*
 * 以下接口为尚未迁移完成的 C 模块提供兼容访问。
 */
extern "C"{
#endif


int can_business_init(void);
void can_business_process(void);
void can_business_tx_test_process(void);
void can_business_loopback_test_process(void);

int can_business_process_frame(
    uint16_t id,
    const uint8_t *data,
    uint8_t len);

const WheelCommand *can_business_get_wheel_command(void);

float can_business_get_motor0_iq_output(void);
float can_business_get_motor1_iq_output(void);

uint8_t can_business_current_command_is_active(void);

CanMotorMode can_business_get_motor_mode(void);
CanCommState can_business_get_comm_state(void);

const CanSequenceStats *
can_business_get_wheel_sequence_stats(void);

const CanSequenceStats *
can_business_get_speed_sequence_stats(void);

float can_business_get_motor0_speed_target(void);
float can_business_get_motor1_speed_target(void);

#ifdef __cplusplus
}
#endif

#endif