/**
 * 文件用途：双电机业务服务实现。
 * 所属层级：AppLayer/motor_control。
 * 主要职责：保持 FOC 调用、CAN 命令选择和故障切换行为不变。
 */

#include "motor_service.h"
#include "foc.h"
#include "can_business.h"
#include "safety_limit.h"
#include "motor_fault.h"
#include "drv_fault.h"
#include <math.h>

/* 历史命名保持不变：g_motor1=物理 Motor0/左轮/TMR1，g_motor2=物理 Motor1/右轮/TMR8。 */
Motor_Data g_motor1 = {0};
Motor_Data g_motor2 = {0};
static volatile uint32_t g_control_count[2] = {0u};
static volatile uint32_t g_control_overrun_count[2] = {0u};

static Motor_Data *MotorService_GetMotor(uint8_t motor_id)
{
    return (motor_id == 0u) ? &g_motor1 : (motor_id == 1u) ? &g_motor2 : NULL;
}

static float MotorService_CommandDirection(uint8_t motor_id)
{
    return (motor_id == 0u) ? MOTOR0_COMMAND_DIRECTION
                            : MOTOR1_COMMAND_DIRECTION;
}

static uint8_t MotorService_CurrentSenseInvalid(const Motor_Data *motor)
{
    float ia_voltage;
    float ib_voltage;

    if (!isfinite(motor->current_abc.Ia) || !isfinite(motor->current_abc.Ib)
        || !isfinite(motor->current_abc.Ic)
        || !isfinite(motor->control.iq_current_feedback))
        return 1u;

    /* 电流可测量范围由每相零偏决定，不能用 3.3V 直接换算电流。 */
    ia_voltage = motor->calib.ia_offset
        - motor->current_abc.Ia * G * Sampling_resistor;
    ib_voltage = motor->calib.ib_offset
        - motor->current_abc.Ib * G * Sampling_resistor;

    return (ia_voltage <= CURRENT_SENSE_ADC_LOW_V
        || ia_voltage >= CURRENT_SENSE_ADC_HIGH_V
        || ib_voltage <= CURRENT_SENSE_ADC_LOW_V
        || ib_voltage >= CURRENT_SENSE_ADC_HIGH_V) ? 1u : 0u;
}

int MotorService_Init(uint8_t motor_id)
{
    Motor_Data *motor = MotorService_GetMotor(motor_id);
    int ret;

    if (motor == NULL)
        return E_PARAM;

    if (MotorFault_MotorHasFault(motor_id))
        return E_OK;

    ret = Motor_Init((motor_id == 0u) ? MOTOR_1 : MOTOR_2);
    if (ret == E_OK)
    {
        motor->run_state = RUN;
        DrvFault_MotorReady(motor_id);
    }
    return ret;
}

int MotorService_Run(uint8_t motor_id)
{
    Motor_Data *motor = MotorService_GetMotor(motor_id);
    int ret;

    if (motor == NULL)
        return E_PARAM;
    if (MotorFault_MotorHasFault(motor_id))
        return E_OK;

    if (motor->run_state == RUN)
    {
        if (can_business_get_motor_mode() == CAN_MOTOR_MODE_SPEED)
        {
            ret = CascadeControl_Run(motor, Speed_loop,
                ((motor_id == 0u) ? can_business_get_motor0_speed_target()
                                  : can_business_get_motor1_speed_target())
                * MotorService_CommandDirection(motor_id));
        }
        else
        {
            float command_iq = (motor_id == 0u)
                ? can_business_get_motor0_iq_output()
                : can_business_get_motor1_iq_output();

            ret = CascadeControl_Run(motor, Current_loop,
                SafetyLimit_UpdateIq(motor_id, command_iq
                    * MotorService_CommandDirection(motor_id),
                    can_business_current_command_is_active()));
        }
    }
    else if (motor->run_state == STOP)
    {
        ret = CascadeControl_Run(motor, Current_loop, 0.0f);
    }
    else
    {
        return E_OK;
    }

    if (ret != E_OK)
        MotorFault_Enter(motor_id, motor, CONTROL_FAULT);
    else if (MotorService_CurrentSenseInvalid(motor))
        MotorFault_Enter(motor_id, motor, (motor_id == 0u)
            ? MOTOR0_OVERCURRENT : MOTOR1_OVERCURRENT);
    return ret;
}

int MotorService_ClearFault(uint8_t motor_id)
{
    Motor_Data *motor = MotorService_GetMotor(motor_id);

    if (motor == NULL)
        return E_PARAM;
    if (!MotorFault_MotorHasFault(motor_id))
        return E_OK;
    if (DrvFault_IsActive(motor_id))
        return E_ERROR;

    SafetyLimit_ForceZero(motor_id);
    Motor_Reset_Current_Controller(motor);
    Motor_Reset_Speed_Controller(motor);
    if (!MotorFault_ClearMotor(motor_id))
        return E_ERROR;
    if (MotorFault_MotorHasFault(motor_id))
        return E_ERROR;
    if (Open_Motor(motor) != E_OK)
        return E_ERROR;

    motor->run_state = RUN;
    return E_OK;
}

void MotorService_RecordControlCycle(uint8_t motor_id, uint32_t notifications)
{
    if (motor_id >= 2u)
        return;

    g_control_count[motor_id]++;
    if (notifications > 1u)
        g_control_overrun_count[motor_id] += notifications - 1u;
}

uint32_t MotorService_GetControlCount(uint8_t motor_id)
{
    return (motor_id < 2u) ? g_control_count[motor_id] : 0u;
}

uint32_t MotorService_GetControlOverrunCount(uint8_t motor_id)
{
    return (motor_id < 2u) ? g_control_overrun_count[motor_id] : 0u;
}
