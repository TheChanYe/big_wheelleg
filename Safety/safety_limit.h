/**
 * @file safety_limit.h
 * @brief 双电机电流命令限幅、斜率限制和紧急归零接口。
 *
 * SafetyLimiter 位于业务命令与底层电流控制之间，负责：
 * - 限制最大电流命令；
 * - 根据实际调用间隔限制电流变化率；
 * - 根据温度等条件动态降低电流上限；
 * - 在故障、超时或停机时立即强制命令归零。
 */

 #ifndef SAFETY_LIMIT_H
#define SAFETY_LIMIT_H

#include "main.h"

/** 当前系统的物理电机数量。 */
#define MOTOR_COMMAND_COUNT                  2u

/** 正常情况下允许的最大绝对 Iq 命令，单位 A。 */
#define MOTOR_COMMAND_IQ_LIMIT_A             0.8f

/** Iq 命令最大变化率，单位 A/s。 */
#define MOTOR_COMMAND_IQ_SLEW_A_PER_S        8.0f

/*
 * 初始热保护阈值。
 * 这些数值保持现有配置，不在 C++ 迁移阶段调整。
 */
#define MOS_TEMP_WARN_C                      70.0f
#define MOS_TEMP_DERATE_C                    80.0f
#define MOS_TEMP_FAULT_C                     90.0f
#define MOS_TEMP_DERATE_MIN_RATIO            0.25f
#define MOS_TEMP_FAULT_CONFIRM_COUNT         3u
#define MOS_TEMP_RECOVER_C                   75.0f
#define TEMP_SENSOR_FAULT_CONFIRM_COUNT      3u

#ifdef __cplusplus

/**
 * @brief 双电机安全限流器。
 *
 * 一个对象同时管理两路电机，因为全局故障处理和 CAN 超时需要
 * 同时将两路电机命令归零。
 *
 * 该类只保存软件限流状态，不拥有 ADC、PWM 或电机硬件资源。
 */
class SafetyLimiter final {
    public:
        SafetyLimiter();
        ~SafetyLimiter();

        /*
        * 安全状态只能存在一份。复制会产生两份不同的有效电流状态，
        * 因此禁止复制构造和复制赋值。
        */
        SafetyLimiter(const SafetyLimiter&) = delete;
        SafetyLimiter& operator=(const SafetyLimiter&) = delete;

        /**
        * @brief 初始化两路电机的限流状态。
        *
        * 恢复默认最大电流，并将两路有效命令强制归零。
        */
        void init();

        /**
        * @brief 更新一路电机的安全电流命令。
        * @param motor_id 物理电机编号，只允许 0 或 1。
        * @param command_iq 上层请求的 Iq 命令，单位 A。
        * @param command_enabled 非零表示当前命令有效。
        * @return 经过绝对限幅、动态限幅和斜率限制后的 Iq。
        */
        float updateIq(uint8_t motor_id, float command_iq, bool command_enabled);

        /**
        * @brief 设置一路电机当前允许的最大 Iq。
        *
        * limit_a 会被限制在 0 到 MOTOR_COMMAND_IQ_LIMIT_A 之间。
        */
        void setIqLimit(uint8_t motor_id, float limit_a);

        /**
        * @brief 在普通任务上下文中强制一路电机命令归零。
        */
        void forceZero(uint8_t motor_id);

        /**
        * @brief 在中断上下文中强制一路电机命令归零。
        *
        * ISR 中不调用 xTaskGetTickCount()，也不调用非 ISR 安全接口。
        */
        void forceZeroFromIsr(uint8_t motor_id);

        /** @brief 在任务上下文中将全部电机命令归零。 */
        void forceZeroAll();

        /**
        * @brief 计算单步斜率限制结果。
        *
        * 这是一个无状态工具函数，方便后续进行独立单元测试。
        */
        static float updateIqStep(float current, float target, float max_delta);

    
    private:
        /** 将 value 限制在 low 与 high 之间。 */
        static float clamp(float value, float low, float high);

        /** 检查物理电机编号是否合法 */
        static bool isValidMotor(uint8_t motor_id);

        /** 两路电机进过斜率限制后的实际有效命令 */
        float effective_iq_[MOTOR_COMMAND_COUNT];
        
        /** 两路电机当前动态允许的最大绝对电流 */ 
        float iq_limit_[MOTOR_COMMAND_COUNT];

        /** 两路电机上次执行斜率更新时的系统tick */
        TickType_t iq_update_tick_[MOTOR_COMMAND_COUNT];

        /** 标记对应电机是否已经建立有效的更新时间基准 */
        uint8_t iq_tick_valid_[MOTOR_COMMAND_COUNT];
};

/*
 * 以下接口暂时提供给尚未迁移的 C 文件。
 */
extern "C" {
#endif

void SafetyLimit_Init(void);

float MotorCommand_UpdateIq(float current, float target, float max_delta);

float SafetyLimit_UpdateIq(
    uint8_t motor_id,
    float command_iq,
    uint8_t command_enabled);

void SafetyLimit_SetIqLimit(
    uint8_t motor_id,
    float limit_a);

void SafetyLimit_ForceZero(uint8_t motor_id);
void SafetyLimit_ForceZeroFromISR(uint8_t motor_id);
void SafetyLimit_ForceZeroAll(void);



#ifdef __cplusplus
}
#endif

#endif