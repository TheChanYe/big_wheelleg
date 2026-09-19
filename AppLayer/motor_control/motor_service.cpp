/**
 * @file motor_service.cpp
 * @brief 双电机业务服务的 C++ 实现。
 *
 * 本文件将电机业务状态封装进 MotorService 对象，但暂时不改动底层
 * FOC、ADC、CAN 和故障模块的数据结构及控制算法。
 */

#include "motor_service.h"
#include "can_business.h"
#include "safety_limit.h"
#include "motor_fault.h"
#include "drv_fault.h"
#include <math.h>

/*
 * foc.c 仍使用 C 编译。
 *
 * 这里不直接包含 foc.h，因为 foc.h 会继续包含 as5047p.h 和
 * drv8301.h，而这些旧 C 头文件目前仍使用 C++ 关键字 this。
 *
 * 因此本阶段只声明 MotorService 实际调用的几个窄 C 接口。
 */
extern "C" {

/** 初始化指定物理电机的底层 FOC 和硬件资源。 */
int Motor_Init(Motor_Type motor);

/** 执行一次底层级联控制。 */
int CascadeControl_Run(
    Motor_Data *motor,
    Motor_Mode mode,
    float target);

/** 清除电流控制器内部状态。 */
int Motor_Reset_Current_Controller(Motor_Data *motor);

/** 清除速度控制器内部状态。 */
int Motor_Reset_Speed_Controller(Motor_Data *motor);

/** 在故障恢复后重新打开电机输出。 */
int Open_Motor(Motor_Data *motor);

}

/*
 * 这两个历史全局变量仍被以下 C 模块直接访问：
 *
 * - foc.c
 * - motor_adc.c
 * - can_business.c
 * - uart_business.c
 * - drv_fault.c
 *
 * 因此当前阶段不能直接改成 MotorService 的私有成员。
 *
 * 显式使用 C linkage 可以确保上述 C 文件查找到的符号名称仍然是：
 * g_motor1 和 g_motor2，而不是经过 C++ name mangling 的符号。
 */
extern "C" {

Motor_Data g_motor1 = {};
Motor_Data g_motor2 = {};

}

namespace
{

/*
 * 临时对象注册表。
 *
 * 尚未迁移的 C 文件只知道 motor_id，无法直接持有 MotorService 对象。
 * C 兼容函数会通过该表找到 TaskManager 中创建的两个服务对象。
 *
 * 本数组只保存对象地址，不拥有对象，也不能 delete。
 */
MotorService *g_motor_services[2] = {
    nullptr,
    nullptr
};

/**
 * @brief 根据物理电机编号获取已注册的服务对象。
 * @return 有效对象指针，或者在编号无效/对象未创建时返回 nullptr。
 */
MotorService *getMotorService(uint8_t motor_id)
{
    return (motor_id < 2u)
        ? g_motor_services[motor_id]
        : nullptr;
}

} // namespace

MotorService::MotorService(uint8_t motor_id)
    : motor_id_(motor_id),
      motor_(nullptr),
      control_count_(0u),
      control_overrun_count_(0u)
{
    /*
     * 保持历史编号关系：
     * motor_id 0 对应 g_motor1/MOTOR_1/TMR1；
     * motor_id 1 对应 g_motor2/MOTOR_2/TMR8。
     */
    if (motor_id_ == 0u)
    {
        motor_ = &g_motor1;
    }
    else if (motor_id_ == 1u)
    {
        motor_ = &g_motor2;
    }

    /*
     * 注册当前对象，供尚未迁移的 C 模块通过兼容函数访问。
     * 无效编号不会写入数组。
     */
    if (motor_id_ < 2u)
    {
        g_motor_services[motor_id_] = this;
    }
}

MotorService::~MotorService()
{
    /*
     * 正常固件运行过程中 main() 不会返回，所以析构函数一般不会执行。
     * 这里只解除注册关系，不关闭 PWM，也不销毁底层硬件。
     */
    if (motor_id_ < 2u
        && g_motor_services[motor_id_] == this)
    {
        g_motor_services[motor_id_] = nullptr;
    }
}

Motor_Data *MotorService::motor()
{
    return motor_;
}

const Motor_Data *MotorService::motor() const
{
    return motor_;
}

float MotorService::commandDirection() const
{
    /*
     * 左右轮的机械安装方向不同，必须继续保留原来的方向参数。
     */
    return (motor_id_ == 0u)
        ? MOTOR0_COMMAND_DIRECTION
        : MOTOR1_COMMAND_DIRECTION;
}

bool MotorService::currentSenseIsValid(
    const Motor_Data& motor)
{
    /*
     * NaN 或无穷值不能进入控制和故障阈值比较。
     */
    if (!isfinite(motor.current_abc.Ia)
        || !isfinite(motor.current_abc.Ib)
        || !isfinite(motor.current_abc.Ic)
        || !isfinite(motor.control.iq_current_feedback))
    {
        return false;
    }

    /*
     * 根据电流值反推 ADC 输入电压。
     *
     * 每相的有效测量范围取决于校准零偏，不能直接用 3.3V
     * 除以采样电阻来判断。
     */
    const float ia_voltage =
        motor.calib.ia_offset
        - motor.current_abc.Ia * G * Sampling_resistor;

    const float ib_voltage =
        motor.calib.ib_offset
        - motor.current_abc.Ib * G * Sampling_resistor;

    return ia_voltage > CURRENT_SENSE_ADC_LOW_V
        && ia_voltage < CURRENT_SENSE_ADC_HIGH_V
        && ib_voltage > CURRENT_SENSE_ADC_LOW_V
        && ib_voltage < CURRENT_SENSE_ADC_HIGH_V;
}

int MotorService::init()
{
    /*
     * 非法 motor_id 会使 motor_ 保持为空。
     */
    if (motor_ == nullptr)
    {
        return E_PARAM;
    }

    /*
     * 锁存故障存在时不重新初始化硬件。
     * 保持迁移前返回 E_OK 的行为。
     */
    if (MotorFault_MotorHasFault(motor_id_))
    {
        return E_OK;
    }

    const int result = Motor_Init(
        (motor_id_ == 0u) ? MOTOR_1 : MOTOR_2);

    if (result == E_OK)
    {
        /*
         * 只有初始化成功后才能进入 RUN，并通知驱动故障模块
         * 当前物理电机已经准备完成。
         */
        motor_->run_state = RUN;
        DrvFault_MotorReady(motor_id_);
    }

    return result;
}

int MotorService::run()
{
    if (motor_ == nullptr)
    {
        return E_PARAM;
    }

    /*
     * 已锁存故障的电机不再执行控制算法。
     */
    if (MotorFault_MotorHasFault(motor_id_))
    {
        return E_OK;
    }

    int result;

    if (motor_->run_state == RUN)
    {
        if (can_business_get_motor_mode()
            == CAN_MOTOR_MODE_SPEED)
        {
            /*
             * 速度模式：
             * 从 CAN 业务层取得当前电机速度目标，再应用机械方向系数。
             */
            const float speed_target =
                (motor_id_ == 0u)
                    ? can_business_get_motor0_speed_target()
                    : can_business_get_motor1_speed_target();

            result = CascadeControl_Run(
                motor_,
                Speed_loop,
                speed_target * commandDirection());
        }
        else
        {
            /*
             * 电流模式：
             * 先读取 CAN 电流命令，再应用方向和安全限幅。
             */
            const float command_iq =
                (motor_id_ == 0u)
                    ? can_business_get_motor0_iq_output()
                    : can_business_get_motor1_iq_output();

            const float limited_iq =
                SafetyLimit_UpdateIq(
                    motor_id_,
                    command_iq * commandDirection(),
                    can_business_current_command_is_active());

            result = CascadeControl_Run(
                motor_,
                Current_loop,
                limited_iq);
        }
    }
    else if (motor_->run_state == STOP)
    {
        /*
         * STOP 状态仍调用一次零电流控制，保持原有停机输出行为。
         */
        result = CascadeControl_Run(
            motor_,
            Current_loop,
            0.0f);
    }
    else
    {
        /*
         * CALIB、FAULT 等其他状态由对应模块处理。
         */
        return E_OK;
    }

    if (result != E_OK)
    {
        /*
         * 底层控制算法返回错误时，进入对应电机的控制故障。
         */
        MotorFault_Enter(
            motor_id_,
            motor_,
            (motor_id_ == 0u)
                ? MOTOR0_CONTROL_FAULT
                : MOTOR1_CONTROL_FAULT);
    }
    else if (!currentSenseIsValid(*motor_))
    {
        /*
         * 控制执行成功后仍需确认电流采样数据有效。
         */
        MotorFault_Enter(
            motor_id_,
            motor_,
            (motor_id_ == 0u)
                ? MOTOR0_OVERCURRENT
                : MOTOR1_OVERCURRENT);
    }

    return result;
}

int MotorService::clearFault()
{
    if (motor_ == nullptr)
    {
        return E_PARAM;
    }

    /*
     * 没有故障时无需执行恢复流程。
     */
    if (!MotorFault_MotorHasFault(motor_id_))
    {
        return E_OK;
    }

    /*
     * 恢复条件不满足时禁止清除故障。
     */
    if (!MotorFault_CanClear(motor_id_, motor_))
    {
        return E_ERROR;
    }

    /*
     * 清除前先强制电流命令归零，并复位电流环和速度环状态，
     * 防止恢复后沿用故障前的积分值或目标值。
     */
    SafetyLimit_ForceZero(motor_id_);
    Motor_Reset_Current_Controller(motor_);
    Motor_Reset_Speed_Controller(motor_);

    if (!MotorFault_ClearMotor(motor_id_))
    {
        return E_ERROR;
    }

    /*
     * 清除后再次检查，确保没有仍然有效的故障位。
     */
    if (MotorFault_MotorHasFault(motor_id_))
    {
        return E_ERROR;
    }

    if (Open_Motor(motor_) != E_OK)
    {
        return E_ERROR;
    }

    motor_->run_state = RUN;

    return E_OK;
}

void MotorService::recordControlCycle(
    uint32_t notifications)
{
    ++control_count_;

    /*
     * ulTaskNotifyTake(pdTRUE, ...) 会一次返回累计通知数量。
     * 大于 1 表示任务没有逐次处理所有 ADC 通知。
     */
    if (notifications > 1u)
    {
        control_overrun_count_ += notifications - 1u;
    }
}

uint32_t MotorService::controlCount() const
{
    return control_count_;
}

uint32_t MotorService::controlOverrunCount() const
{
    return control_overrun_count_;
}

/*
 * 以下函数是临时 C ABI 桥接层。
 *
 * 新的 C++ 代码应直接调用 MotorService 对象；旧 C 模块暂时通过
 * motor_id 查找对应对象。
 */

extern "C" int MotorService_Init(uint8_t motor_id)
{
    MotorService *service = getMotorService(motor_id);

    return (service != nullptr)
        ? service->init()
        : E_PARAM;
}

extern "C" int MotorService_Run(uint8_t motor_id)
{
    MotorService *service = getMotorService(motor_id);

    return (service != nullptr)
        ? service->run()
        : E_PARAM;
}

extern "C" int MotorService_ClearFault(uint8_t motor_id)
{
    MotorService *service = getMotorService(motor_id);

    return (service != nullptr)
        ? service->clearFault()
        : E_PARAM;
}

extern "C" void MotorService_RecordControlCycle(
    uint8_t motor_id,
    uint32_t notifications)
{
    MotorService *service = getMotorService(motor_id);

    if (service != nullptr)
    {
        service->recordControlCycle(notifications);
    }
}

extern "C" uint32_t MotorService_GetControlCount(
    uint8_t motor_id)
{
    MotorService *service = getMotorService(motor_id);

    return (service != nullptr)
        ? service->controlCount()
        : 0u;
}

extern "C" uint32_t MotorService_GetControlOverrunCount(
    uint8_t motor_id)
{
    MotorService *service = getMotorService(motor_id);

    return (service != nullptr)
        ? service->controlOverrunCount()
        : 0u;
}

extern "C" uint8_t MotorService_CurrentSenseIsValid(
    const Motor_Data *motor)
{
    if (motor == nullptr)
    {
        return 0u;
    }

    return MotorService::currentSenseIsValid(*motor)
        ? 1u
        : 0u;
}