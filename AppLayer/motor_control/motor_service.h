/**
 * @file motor_service.h
 * @brief 双电机业务服务的 C++ 接口及临时 C 兼容接口。
 *
 * MotorService 负责连接上层任务调度和底层 FOC 控制，包括：
 * - 初始化指定物理电机；
 * - 根据 CAN 命令选择速度模式或电流模式；
 * - 执行一次电机控制；
 * - 处理控制错误和电流采样错误；
 * - 清除满足恢复条件的锁存故障；
 * - 统计控制周期和通知堆积次数。
 *
 * 当前迁移阶段仍有多个 C 文件使用该模块，因此文件末尾继续保留
 * extern "C" 接口。等 CAN、安全和硬件层迁移后，再评估删除这些接口。
 */

#ifndef MOTOR_SERVICE_H
#define MOTOR_SERVICE_H

#include "foc_cfg.h"

#ifdef __cplusplus

/**
 * @brief 单个物理电机的业务服务对象。
 *
 * 每个 MotorService 对象只对应一个物理电机：
 *
 * motor_id == 0：
 * - 历史变量 g_motor1
 * - 物理 Motor0
 * - 左轮
 * - TMR1
 *
 * motor_id == 1：
 * - 历史变量 g_motor2
 * - 物理 Motor1
 * - 右轮
 * - TMR8
 *
 * 本类不直接实现 FOC 算法。FOC 仍由底层 C 模块实现，本类只负责
 * 业务状态判断、控制命令选择、错误处理和统计。
 */
class MotorService final {
    public:
        /**
         * @brief 创建指定物理电机的业务服务对象。
         * @param motor_id 物理电机编号，只允许为 0 或 1。
         *
         * 构造函数只建立对象与 Motor_Data 的对应关系，不初始化 ADC、
         * PWM、编码器或驱动芯片。真正的硬件初始化由 init() 完成。
         */
        explicit MotorService(uint8_t motor_id);
        /**
         * @brief 销毁电机服务对象。
         *
         * 固件正常运行期间该对象不会被销毁。析构函数只解除临时的
         * C 接口注册关系，不关闭电机，也不释放底层硬件资源。
         */
        ~MotorService();
        /**
         * @brief 禁止复制构造和赋值。
         *
         * 一个服务对象代表唯一物理硬件。
         * 复制对象会导致多个对象同时控制同一电机，因此禁止复制。
         */
        MotorService(const MotorService&) = delete;
        MotorService& operator=(const MotorService&) = delete;
        /**
        * @brief 初始化当前对象对应的物理电机。
        * @return E_OK 表示成功，其他值表示底层初始化失败。
        *
        * 如果电机已经处于锁存故障状态，则保持故障状态并返回 E_OK，
        * 行为与迁移前代码保持一致。
        */
        int init();
        /**
        * @brief 执行一次电机业务控制周期。
        * @return E_OK 表示本次处理完成，其他值表示控制失败。
        *
        * 根据 CAN 当前模式选择：
        * - CAN_MOTOR_MODE_SPEED：执行速度控制；
        * - 其他模式：执行带安全限幅的电流控制。
        *
        * 该函数由 ADC 中断通知唤醒的电机任务调用，不应在 ISR 中调用。
        */
        int run();
        /**
        * @brief 尝试清除当前电机的锁存故障。
        * @return E_OK 表示清除完成或当前没有故障；E_ERROR 表示不允许恢复。
        *
        * 清除前会检查温度、母线电压、驱动故障等恢复条件。
        */
        int clearFault();
        /**
        * @brief 记录一次控制任务处理情况。
        * @param notifications 本次任务取得的累计通知数量。
        *
        * notifications 大于 1 表示 ADC 中断产生通知的速度超过任务处理速度。
        * 多出的通知会计入控制周期堆积统计。
        */
        void recordControlCycle(uint32_t notifications);
        /** @brief 获取已处理的控制周期总数。 */
        uint32_t controlCount() const;
        /** @brief 获取累计通知堆积数量。 */
        uint32_t controlOverrunCount() const;

        /**
        * @brief 获取当前对象对应的可写电机数据。
        *
        * 返回的内存由兼容期全局变量持有，MotorService 不负责释放。
        */
        Motor_Data *motor();

        /** @brief 获取当前对象对应的只读电机数据。 */
        const Motor_Data *motor() const;

        /**
        * @brief 检查电流采样数据是否有效。
        * @param motor 待检查的电机数据。
        * @return true 表示数据有限且 ADC 电压处于可测量范围。
        *
        * 该函数不修改 Motor_Data，因此使用 const 引用。
        */
        static bool currentSenseIsValid(const Motor_Data& motor);

    private:
        /**
        * @brief 获取当前物理电机的命令方向系数。
        *
        * 用于处理左右轮机械安装方向不同的问题。
        */
        float commandDirection() const;
        /** 当前对象对应的物理电机编号，只允许为 0 或 1。 */
        uint8_t motor_id_;
        /**
        * 指向当前电机的运行数据。
        *
        * 目前指向 g_motor1 或 g_motor2。该指针不拥有内存，不能 delete。
        */
        Motor_Data *motor_;
        /**
        * 已处理的控制周期数量。
        *
        * CAN 诊断任务可能读取该值，因此保留原代码的 volatile 属性。
        */
        volatile uint32_t control_count_;
        /**
        * ADC 通知堆积数量。
        *
        * 用于判断控制任务是否来不及处理每次 ADC 中断。
        */
        volatile uint32_t control_overrun_count_;
};

/*
 * 下面开始声明给 C 文件调用的临时兼容接口。
 */
extern "C" {
#endif


/**
 * @brief 临时 C ABI 初始化接口。
 *
 * 新的 C++ 任务代码应直接调用 MotorService::init()。
 */
int MotorService_Init(uint8_t motor_id);

/**
 * @brief 临时 C ABI 控制接口。
 *
 * 新的 C++ 任务代码应直接调用 MotorService::run()。
 */
int MotorService_Run(uint8_t motor_id);

/**
 * @brief 临时 C ABI 故障恢复接口。
 *
 * 当前由 can_business.c 使用。
 */
int MotorService_ClearFault(uint8_t motor_id);

/**
 * @brief 临时 C ABI 控制周期记录接口。
 */
void MotorService_RecordControlCycle(
    uint8_t motor_id,
    uint32_t notifications);

/** @brief 获取指定电机的控制周期总数。 */
uint32_t MotorService_GetControlCount(uint8_t motor_id);

/** @brief 获取指定电机的通知堆积总数。 */
uint32_t MotorService_GetControlOverrunCount(uint8_t motor_id);

/**
 * @brief 检查指定 Motor_Data 的电流采样是否有效。
 *
 * 当前由 motor_fault.c 使用。
 */
uint8_t MotorService_CurrentSenseIsValid(
    const Motor_Data *motor);

#ifdef __cplusplus
}
#endif

#endif