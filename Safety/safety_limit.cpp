/**
 * @file safety_limit.cpp
 * @brief 双电机安全电流限制器的 C++ 实现。
 *
 * 本次迁移只改变状态的归属和接口组织，不修改限幅数值、
 * 斜率算法、Tick 换算或 ISR 行为。
 */

#include "safety_limit.h"

namespace {
    /*
    * 临时对象注册指针。
    *
    * 尚未迁移的 C 模块通过文件末尾的 C ABI 函数访问 SafetyLimiter。
    * 该指针不拥有对象，不能 delete。
    */
    SafetyLimiter *g_safety_limiter = nullptr;
} // namespace

SafetyLimiter::SafetyLimiter() 
    : effective_iq_{0.0f, 0.0f},
      iq_limit_{
          MOTOR_COMMAND_IQ_LIMIT_A,
          MOTOR_COMMAND_IQ_LIMIT_A},
      iq_update_tick_{0, 0},
      iq_tick_valid_{0u, 0u} {
    /*
     * 构造函数只初始化内存状态，不读取 RTOS tick。
     * TaskManager 在调度器启动前构造该对象。
     */
    g_safety_limiter = this;
}
/**
 * @brief 析构函数。
 */
SafetyLimiter::~SafetyLimiter() {
    /*
     * 正常固件运行期间 main() 不会返回，因此析构函数通常不会执行。
     * 这里只解除 C 兼容注册关系，不访问硬件或 RTOS。
     */
    if (g_safety_limiter == this) {
        g_safety_limiter = nullptr;
    }
}
/**
 * @brief 检查电机 ID 是否有效。
 * @param motor_id 电机 ID。
 * @return 如果 motor_id 小于 MOTOR_COMMAND_COUNT，则返回 true，否则返回 false。
 */
bool SafetyLimiter::isValidMotor(uint8_t motor_id) {
    return motor_id < MOTOR_COMMAND_COUNT;
}
/**
 * @brief 将值限制在指定范围内。
 * @param value 待限制的值。
 * @param low 下限。
 * @param high 上限。
 * @return 如果 value 小于 low，则返回 low；如果 value 大于 high，则返回 high；否则返回 value。
 */
float SafetyLimiter::clamp(float value, float low, float high) {
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}
/**
 * @brief 更新电流步进值。
 * @param current 当前电流值。
 * @param target 目标电流值。
 * @param max_delta 最大允许变化量。
 * @return 更新后的电流值，保证变化量不超过 max_delta。
 */
float SafetyLimiter::updateIqStep(float current, float target, float max_delta) {
    if (target > current + max_delta) {
        return current + max_delta;
    }

    if (target < current - max_delta) {
        return current - max_delta;
    }
    return target;
}
/**
 * @brief 初始化安全限流器。
 *
 * 恢复默认动态限流值，并清除斜率限制器的时间有效标志。
 */
void SafetyLimiter::init() {
    /*
     * 恢复默认动态限流值。
     * forceZeroAll() 同时清除斜率限制器的时间有效标志。
     */
    for (uint8_t motor_id = 0u;
         motor_id < MOTOR_COMMAND_COUNT;
         ++motor_id)
    {
        iq_limit_[motor_id] = MOTOR_COMMAND_IQ_LIMIT_A;
    }

    forceZeroAll();
}
/**
 * @brief 更新电机电流。
 *
 * 根据命令电流和斜率限制器计算实际输出电流。
 *
 * @param motor_id 电机 ID。
 * @param command_iq 命令电流。
 * @param command_enabled 命令是否有效。
 * @return 实际输出电流。
 */
float SafetyLimiter::updateIq(
    uint8_t motor_id,
    float command_iq,
    bool command_enabled)
{
    if (!isValidMotor(motor_id)) {
        return 0.0f; // 无效电机 ID，返回默认值
    }

    const TickType_t now = xTaskGetTickCount();

    if (!command_enabled) {
        /*
         * 命令无效时立即归零，不经过斜率限制。
         *
         * 保持原代码行为：更新时间，但不清除 tick_valid。
         */
        effective_iq_[motor_id] = 0.0f;
        iq_update_tick_[motor_id] = now;

        return 0.0f;
    }

    /*
     * 第一层限制为系统绝对电流上限；
     * 第二层限制为温度等安全模块设置的动态上限。
     */
    command_iq = clamp(command_iq, -MOTOR_COMMAND_IQ_LIMIT_A, MOTOR_COMMAND_IQ_LIMIT_A);

    command_iq = clamp(command_iq, -iq_limit_[motor_id], iq_limit_[motor_id]);

    if (iq_tick_valid_[motor_id] == 0u) {
        /*
         * 第一次收到有效命令时只建立时间基准，不立即跳变输出。
         */
        iq_update_tick_[motor_id] = now;
        iq_tick_valid_[motor_id] = 1u;

        return effective_iq_[motor_id];
    }

    const TickType_t elapsed = now - iq_update_tick_[motor_id];

    iq_update_tick_[motor_id] = now;

    const float max_delta = static_cast<float>(elapsed)
                            * static_cast<float>(portTICK_PERIOD_MS)
                            * 0.001f
                            * MOTOR_COMMAND_IQ_SLEW_A_PER_S;
    effective_iq_[motor_id] = updateIqStep(
        effective_iq_[motor_id],
        command_iq,
        max_delta
    );

    return effective_iq_[motor_id];
}
/**
 * @brief 强制将电机电流归零。
 *
 * 立即将指定电机的实际输出电流置零，并重置斜率限制器的时间有效标志。
 *
 * @param motor_id 电机 ID。
 */
void SafetyLimiter::setIqLimit(uint8_t motor_id, float limit_a) {
    if (!isValidMotor(motor_id)) {
        return;
    }

    iq_limit_[motor_id] = clamp(limit_a, 0.0f, MOTOR_COMMAND_IQ_LIMIT_A);
}
/**
 * @brief 强制将电机电流归零。
 *
 * 立即将指定电机的实际输出电流置零，并重置斜率限制器时间有效标志。
 *
 * @param motor_id 电机 ID。
 */
void SafetyLimiter::forceZero(uint8_t motor_id) {
    if (!isValidMotor(motor_id)) {
        return;
    }
    /*
     * 普通任务上下文允许读取当前 RTOS tick。
     */
    effective_iq_[motor_id] = 0.0f;
    iq_update_tick_[motor_id] = xTaskGetTickCount();
    iq_tick_valid_[motor_id] = 0u;
}
/**
 * 强制电机 ID 的电流为零。
 *
 * @param motor_id 电机 ID。
 */
void SafetyLimiter::forceZeroFromIsr(uint8_t motor_id) {
    if (!isValidMotor(motor_id)) {
        return;
    }
    /*
     * ISR 路径不能调用普通的 xTaskGetTickCount()。
     * 下一次有效命令会重新建立更新时间基准。
     */
    effective_iq_[motor_id]= 0.0f;
    iq_tick_valid_[motor_id] = 0u;
}
/**
 * @brief 强制所有电机电流归零。
 *
 * 遍历所有电机，调用 forceZero() 将其实际输出电流置零，并重置斜率限制器的时间有效标志。
 */
void SafetyLimiter::forceZeroAll() {
    for (uint8_t motor_id = 0u; motor_id < MOTOR_COMMAND_COUNT; ++motor_id) {
        forceZero(motor_id);
    }
}

/*
 * 以下为临时 C ABI 桥接层。
 */

extern "C" void SafetyLimit_Init(void)
{
    if (g_safety_limiter != nullptr)
    {
        g_safety_limiter->init();
    }
}

extern "C" float MotorCommand_UpdateIq(
    float current,
    float target,
    float max_delta)
{
    return SafetyLimiter::updateIqStep(
        current,
        target,
        max_delta);
}

extern "C" float SafetyLimit_UpdateIq(
    uint8_t motor_id,
    float command_iq,
    uint8_t command_enabled)
{
    return (g_safety_limiter != nullptr)
        ? g_safety_limiter->updateIq(
              motor_id,
              command_iq,
              command_enabled != 0u)
        : 0.0f;
}

extern "C" void SafetyLimit_SetIqLimit(
    uint8_t motor_id,
    float limit_a)
{
    if (g_safety_limiter != nullptr)
    {
        g_safety_limiter->setIqLimit(
            motor_id,
            limit_a);
    }
}

extern "C" void SafetyLimit_ForceZero(
    uint8_t motor_id)
{
    if (g_safety_limiter != nullptr)
    {
        g_safety_limiter->forceZero(motor_id);
    }
}

extern "C" void SafetyLimit_ForceZeroFromISR(
    uint8_t motor_id)
{
    if (g_safety_limiter != nullptr)
    {
        g_safety_limiter->forceZeroFromIsr(motor_id);
    }
}

extern "C" void SafetyLimit_ForceZeroAll(void)
{
    if (g_safety_limiter != nullptr)
    {
        g_safety_limiter->forceZeroAll();
    }
}