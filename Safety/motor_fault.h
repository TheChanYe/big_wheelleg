/**
 * @file motor_fault.h
 * @brief 双电机锁存故障管理及安全停机接口。
 *
 * 故障一旦进入便保持锁存，只有 CAN 明确发送清除命令，
 * 并且所有恢复条件满足后才能清除。
 */

#ifndef MOTOR_FAULT_H
#define MOTOR_FAULT_H

#include "foc_cfg.h"

/** 故障状态使用 32 位位图保存。 */
typedef uint32_t MotorFaultBits;

/* Motor0 独立故障。 */
#define MOTOR0_ENCODER_FAULT       (1u << 0)
#define MOTOR0_DRV_FAULT           (1u << 2)
#define MOTOR0_OVERCURRENT         (1u << 4)
#define MOTOR0_OVERTEMP            (1u << 6)
#define MOTOR0_TEMP_SENSOR_FAULT   (1u << 16)
#define MOTOR0_CONTROL_FAULT       (1u << 18)

/* Motor1 独立故障。 */
#define MOTOR1_ENCODER_FAULT       (1u << 1)
#define MOTOR1_DRV_FAULT           (1u << 3)
#define MOTOR1_OVERCURRENT         (1u << 5)
#define MOTOR1_OVERTEMP            (1u << 7)
#define MOTOR1_TEMP_SENSOR_FAULT   (1u << 17)
#define MOTOR1_CONTROL_FAULT       (1u << 19)

/* 两路电机共享的系统级故障。 */
#define VBUS_UNDERVOLTAGE          (1u << 8)
#define VBUS_OVERVOLTAGE           (1u << 9)
#define CAN_TIMEOUT_FAULT          (1u << 10)
#define CAN_SEQUENCE_FAULT         (1u << 11)
#define ADC_FAULT                  (1u << 12)
#define CALIBRATION_FAULT          (1u << 13)
#define CONTROL_FAULT              (1u << 14)
#define INTERNAL_FAULT             (1u << 15)

#ifdef __cplusplus

/**
 * @brief 双电机锁存故障管理器。
 *
 * 该对象保存整个系统唯一的故障位图，并提供两种故障入口：
 *
 * - enter()：普通任务上下文，可关闭电机和调用普通 FreeRTOS API；
 * - enterFromIsr()：中断上下文，只执行 ISR 安全操作。
 *
 * 本类不拥有 Motor_Data，也不负责释放电机资源。
 */
class MotorFaultManager final {
    public:
        MotorFaultManager();
        ~MotorFaultManager();

        /**
         *  故障位图只能存一份，禁止复制
         */
        MotorFaultManager(const MotorFaultManager&) = delete;
        MotorFaultManager& operator=(const MotorFaultManager&) = delete;

        /** 清空系统故障位图，仅用于启动初始化。 */
        void init();

        /**
        * @brief 从普通任务上下文进入故障。
        *
        * 执行顺序保持原逻辑：
        * 1. 锁存故障位；
        * 2. 强制安全电流归零；
        * 3. 复位电流环和速度环；
        * 4. 设置电机 FAULT 状态；
        * 5. 关闭电机 PWM。
        */
        void enter(
            uint8_t motor_id,
            Motor_Data *motor,
            MotorFaultBits bits
        );

        /**
        * @brief 从中断上下文进入故障。
        *
        * ISR 中不调用 Close_Motor()，硬件快速关断由对应中断处理程序完成。
        */
        void enterFromIsr(
            uint8_t motor_id,
            Motor_Data *motor,
            MotorFaultBits bits
        );

        /** 在线程安全的临界区内读取故障位快照。 */
        MotorFaultBits bits() const;

        /** 判断指定电机是否存在独立故障或系统级故障。 */
        bool motorHasFault(uint8_t motor_id) const;

        /** 清除指定电机的独立故障位，不清除系统级故障。 */
        bool clearMotor(uint8_t motor_id);

        /**
        * @brief 检查指定电机当前是否满足故障恢复条件。
        *
        * 本函数只做检查，不修改故障位，也不重新打开电机。
        */
        bool canClear(
            uint8_t motor_id,
            Motor_Data *motor
        ) const;

    private:
        /** 在普通任务临界区中锁存故障位。 */
        void setBits(MotorFaultBits bits);

        /** 在 ISR 临界区中锁存故障位 */
        void setBitsFromIsr(MotorFaultBits bits);

        /** 在普通任务e临界区中清除故障位 */
        void clearBits(MotorFaultBits bits);

        /** 返回指定电机的全部故障判断掩码 */
        static MotorFaultBits faultMask(uint8_t motor_id);

        /**  返回指定电机允许通过命令清除的独立故障掩码 */
        static MotorFaultBits clearableMask(uint8_t motor_id);

        /**
        * 整个系统唯一的锁存故障位图。
        *
        * 该成员同时由任务和 ISR 访问，写操作必须在对应临界区内完成。
        */
        volatile MotorFaultBits fault_bits_;

};

/*
 * 以下接口暂时提供给尚未迁移的 C 模块。
 */
extern "C" {
#endif

void MotorFault_Init(void);

void MotorFault_Enter(
    uint8_t motor_id,
    Motor_Data *motor,
    MotorFaultBits bits);

void MotorFault_EnterFromISR(
    uint8_t motor_id,
    Motor_Data *motor,
    MotorFaultBits bits);

MotorFaultBits MotorFault_GetBits(void);

uint8_t MotorFault_MotorHasFault(uint8_t motor_id);
uint8_t MotorFault_ClearMotor(uint8_t motor_id);

uint8_t MotorFault_CanClear(
    uint8_t motor_id,
    Motor_Data *motor);

#ifdef __cplusplus
}
#endif

#endif