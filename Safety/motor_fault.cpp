/**
 * @file motor_fault.cpp
 * @brief 双电机锁存故障管理器的 C++ 实现。
 *
 * 本次迁移只改变故障状态的归属和接口组织，不改变故障位、
 * 临界区、停机顺序及故障恢复条件。
 */

#include "motor_fault.h"
#include "safety_limit.h"
#include "drv_fault.h"
#include "motor_service.h"

/*
 * 这些电机控制函数仍由 foc.c 实现。
 *
 * 不直接包含 foc.h，避免引入尚未完成 C++ 兼容的旧驱动头文件。
 */
extern "C" {

int Motor_Reset_Current_Controller(Motor_Data *motor);
int Motor_Reset_Speed_Controller(Motor_Data *motor);
int Close_Motor(Motor_Data *motor);
int Get_Mos_Temp(Motor_Data *motor);
int Motor_CheckEncoder(Motor_Data *motor);

}


namespace {

/*
 * 临时对象注册指针。
 *
 * FOC、驱动故障和系统初始化等尚未迁移的模块，通过文件末尾的
 * C ABI 接口访问该对象。该指针不拥有对象。
 */

MotorFaultManager *g_motor_fault_manager = nullptr; 

} // namespace

MotorFaultManager::MotorFaultManager()
    : fault_bits_(0u)
{
    /*
     * 构造函数只初始化内存，不访问电机和 RTOS。
     */
    g_motor_fault_manager = this;
}

MotorFaultManager::~MotorFaultManager()
{
    /*
     * 正常固件运行期间 main() 不会返回。
     * 析构时只解除兼容注册关系，不清除硬件故障。
     */
    if (g_motor_fault_manager == this)
    {
        g_motor_fault_manager = nullptr;
    }
}

void MotorFaultManager::init() {
    /*
     * 该函数只在任务和中断启动前调用，所以保持原来的直接清零行为。
     */
    fault_bits_ = 0u;
}

void MotorFaultManager::setBits(MotorFaultBits bits) {
    taskENTER_CRITICAL();
    fault_bits_ |= bits;
    taskEXIT_CRITICAL();
}

void MotorFaultManager::setBitsFromIsr(MotorFaultBits bits) {
    /*
     * ISR 必须使用 FreeRTOS 专用的中断临界区接口 
     */
    const UBaseType_t mask = 
        taskENTER_CRITICAL_FROM_ISR();
    
    fault_bits_ |= bits;

    taskEXIT_CRITICAL_FROM_ISR(mask);
}

void MotorFaultManager::clearBits(
    MotorFaultBits bits)
{
    taskENTER_CRITICAL();
    fault_bits_ &= ~bits;
    taskEXIT_CRITICAL();
}

MotorFaultBits MotorFaultManager::bits() const
{
    MotorFaultBits snapshot;

    taskENTER_CRITICAL();
    snapshot = fault_bits_;
    taskEXIT_CRITICAL();

    return snapshot;
}

MotorFaultBits MotorFaultManager::faultMask(
    uint8_t motor_id)
{
    /*
     * ADC、校准、通用控制和内部错误属于系统级故障，
     * 任意一路电机查询时都必须包含这些位。
     */
    const MotorFaultBits shared_faults =
        ADC_FAULT
        | CALIBRATION_FAULT
        | CONTROL_FAULT
        | INTERNAL_FAULT;

    if (motor_id == 0u)
    {
        return MOTOR0_ENCODER_FAULT
            | MOTOR0_DRV_FAULT
            | MOTOR0_OVERCURRENT
            | MOTOR0_OVERTEMP
            | MOTOR0_TEMP_SENSOR_FAULT
            | MOTOR0_CONTROL_FAULT
            | shared_faults;
    }

    if (motor_id == 1u)
    {
        return MOTOR1_ENCODER_FAULT
            | MOTOR1_DRV_FAULT
            | MOTOR1_OVERCURRENT
            | MOTOR1_OVERTEMP
            | MOTOR1_TEMP_SENSOR_FAULT
            | MOTOR1_CONTROL_FAULT
            | shared_faults;
    }

    return 0u;
}

MotorFaultBits MotorFaultManager::clearableMask(
    uint8_t motor_id)
{
    /*
     * CAN 清除命令只能清除对应电机的独立故障，
     * 不能清除系统级故障。
     */
    if (motor_id == 0u)
    {
        return MOTOR0_ENCODER_FAULT
            | MOTOR0_DRV_FAULT
            | MOTOR0_OVERCURRENT
            | MOTOR0_OVERTEMP
            | MOTOR0_TEMP_SENSOR_FAULT
            | MOTOR0_CONTROL_FAULT;
    }

    if (motor_id == 1u)
    {
        return MOTOR1_ENCODER_FAULT
            | MOTOR1_DRV_FAULT
            | MOTOR1_OVERCURRENT
            | MOTOR1_OVERTEMP
            | MOTOR1_TEMP_SENSOR_FAULT
            | MOTOR1_CONTROL_FAULT;
    }

    return 0u;
}

void MotorFaultManager::enter(
    uint8_t motor_id,
    Motor_Data *motor,
    MotorFaultBits bits) {
    
    if (motor == nullptr
        || motor_id >= MOTOR_COMMAND_COUNT)
    {
        return;
    }
    
    /*
     * 该顺序属于安全行为，不要调整：
     * 先锁存故障，再清零命令和控制器，最后关闭硬件输出。
     */
    setBits(bits);
    SafetyLimit_ForceZero(motor_id);

    Motor_Reset_Current_Controller(motor);
    Motor_Reset_Speed_Controller(motor);

    motor->run_state = FAULT;

    Close_Motor(motor);
}

void MotorFaultManager::enterFromIsr(
    uint8_t motor_id,
    Motor_Data *motor,
    MotorFaultBits fault_bits) {

    if (motor == nullptr
        || motor_id >= MOTOR_COMMAND_COUNT)
    {
        return;
    }

    /*
    *   ISR 路径不能调用普通任务临界或可能阻塞的关闭流程
    */
    setBitsFromIsr(fault_bits);
    SafetyLimit_ForceZeroFromISR(motor_id);

    Motor_Reset_Current_Controller(motor);
    Motor_Reset_Speed_Controller(motor);

    motor->run_state = FAULT;
}

bool MotorFaultManager::motorHasFault(
    uint8_t motor_id) const {
        const MotorFaultBits mask = 
            faultMask(motor_id);
    
    /*
    *   无效电机编号继续保持原行为： 视为存在故障
    */
    if (mask == 0u)
    {
        return true;
    }

    return (bits() & mask) != 0u;
}

bool MotorFaultManager::clearMotor(
    uint8_t motor_id) {

    const MotorFaultBits mask = 
        clearableMask(motor_id);
    
    if (mask == 0u)
    {
        return false;
    }

    clearBits(mask);

    return true;
}

bool MotorFaultManager::canClear(
    uint8_t motor_id,
    Motor_Data *motor) const
{
    /*
     * 硬件恢复条件：
     * - 电机编号和数据有效；
     * - DRV nFAULT 已恢复；
     * - MOS 温度采样有效；
     * - 温度低于恢复阈值；
     * - 电流采样值有效。
     */
    if (motor == nullptr
        || motor_id >= MOTOR_COMMAND_COUNT
        || DrvFault_IsActive(motor_id) != 0u
        || Get_Mos_Temp(motor) != E_OK
        || motor->mos_temp > MOS_TEMP_RECOVER_C
        || MotorService_CurrentSenseIsValid(motor) == 0u)
    {
        return false;
    }

    /*
     * 系统级故障禁止通过单电机清除命令恢复。
     */
    const MotorFaultBits system_faults =
        ADC_FAULT
        | CALIBRATION_FAULT
        | CONTROL_FAULT
        | INTERNAL_FAULT;

    if ((bits() & system_faults) != 0u)
    {
        return false;
    }

    /*
     * 编码器故障只有在重新读取编码器成功后才能恢复。
     */
    const MotorFaultBits encoder_fault =
        (motor_id == 0u)
            ? MOTOR0_ENCODER_FAULT
            : MOTOR1_ENCODER_FAULT;

    if ((bits() & encoder_fault) != 0u
        && Motor_CheckEncoder(motor) != E_OK)
    {
        return false;
    }

    return true;
}

/*
 * 以下为临时 C ABI 桥接层。
 */

extern "C" void MotorFault_Init(void)
{
    if (g_motor_fault_manager != nullptr)
    {
        g_motor_fault_manager->init();
    }
}

extern "C" void MotorFault_Enter(
    uint8_t motor_id,
    Motor_Data *motor,
    MotorFaultBits bits)
{
    if (g_motor_fault_manager != nullptr)
    {
        g_motor_fault_manager->enter(
            motor_id,
            motor,
            bits);
    }
}

extern "C" void MotorFault_EnterFromISR(
    uint8_t motor_id,
    Motor_Data *motor,
    MotorFaultBits bits)
{
    if (g_motor_fault_manager != nullptr)
    {
        g_motor_fault_manager->enterFromIsr(
            motor_id,
            motor,
            bits);
    }
}

extern "C" MotorFaultBits MotorFault_GetBits(void)
{
    return (g_motor_fault_manager != nullptr)
        ? g_motor_fault_manager->bits()
        : INTERNAL_FAULT;
}

extern "C" uint8_t MotorFault_MotorHasFault(
    uint8_t motor_id)
{
    return (g_motor_fault_manager == nullptr
            || g_motor_fault_manager->motorHasFault(motor_id))
        ? 1u
        : 0u;
}

extern "C" uint8_t MotorFault_ClearMotor(
    uint8_t motor_id)
{
    return (g_motor_fault_manager != nullptr
            && g_motor_fault_manager->clearMotor(motor_id))
        ? 1u
        : 0u;
}

extern "C" uint8_t MotorFault_CanClear(
    uint8_t motor_id,
    Motor_Data *motor)
{
    return (g_motor_fault_manager != nullptr
            && g_motor_fault_manager->canClear(
                motor_id,
                motor))
        ? 1u
        : 0u;
}
