/**
 ******************************************************************************
 * @file     motor_fault.c
 * @brief    One-way local motor fault latch and safe-output path.
 ******************************************************************************
 */

#include "motor_fault.h"
#include "foc.h"
#include "safety_limit.h"
#include "drv_fault.h"
#include "motor_service.h"

static volatile MotorFaultBits g_motor_fault_bits = 0u;

void MotorFault_Init(void)
{
    g_motor_fault_bits = 0u;
}

void MotorFault_Enter(uint8_t motor_id, Motor_Data *motor, MotorFaultBits bits)
{
    if (motor == NULL || motor_id >= MOTOR_COMMAND_COUNT)
        return;

    g_motor_fault_bits |= bits;
    SafetyLimit_ForceZero(motor_id);
    Motor_Reset_Current_Controller(motor);
    Motor_Reset_Speed_Controller(motor);
    motor->run_state = FAULT;
    Close_Motor(motor);
}

void MotorFault_EnterFromISR(uint8_t motor_id, Motor_Data *motor,
                             MotorFaultBits bits)
{
    if (motor == NULL || motor_id >= MOTOR_COMMAND_COUNT)
        return;

    g_motor_fault_bits |= bits;
    SafetyLimit_ForceZeroFromISR(motor_id);
    Motor_Reset_Current_Controller(motor);
    Motor_Reset_Speed_Controller(motor);
    motor->run_state = FAULT;
}

MotorFaultBits MotorFault_GetBits(void)
{
    return g_motor_fault_bits;
}

uint8_t MotorFault_MotorHasFault(uint8_t motor_id)
{
    MotorFaultBits mask;

    if (motor_id == 0u)
    {
        mask = MOTOR0_ENCODER_FAULT | MOTOR0_DRV_FAULT | MOTOR0_OVERCURRENT
             | MOTOR0_OVERTEMP | MOTOR0_TEMP_SENSOR_FAULT | ADC_FAULT | CALIBRATION_FAULT | CONTROL_FAULT
             | INTERNAL_FAULT;
    }
    else if (motor_id == 1u)
    {
        mask = MOTOR1_ENCODER_FAULT | MOTOR1_DRV_FAULT | MOTOR1_OVERCURRENT
             | MOTOR1_OVERTEMP | MOTOR1_TEMP_SENSOR_FAULT | ADC_FAULT | CALIBRATION_FAULT | CONTROL_FAULT
             | INTERNAL_FAULT;
    }
    else
    {
        return 1u;
    }

    return (g_motor_fault_bits & mask) ? 1u : 0u;
}

uint8_t MotorFault_ClearMotor(uint8_t motor_id)
{
    MotorFaultBits mask;

    if (motor_id == 0u)
    {
        mask = MOTOR0_ENCODER_FAULT | MOTOR0_DRV_FAULT | MOTOR0_OVERCURRENT
             | MOTOR0_OVERTEMP | MOTOR0_TEMP_SENSOR_FAULT;
    }
    else if (motor_id == 1u)
    {
        mask = MOTOR1_ENCODER_FAULT | MOTOR1_DRV_FAULT | MOTOR1_OVERCURRENT
             | MOTOR1_OVERTEMP | MOTOR1_TEMP_SENSOR_FAULT;
    }
    else
    {
        return 0u;
    }

    g_motor_fault_bits &= ~mask;
    return 1u;
}

uint8_t MotorFault_CanClear(uint8_t motor_id, Motor_Data *motor)
{
    MotorFaultBits motor_mask;

    if (motor == NULL || motor_id >= MOTOR_COMMAND_COUNT
        || DrvFault_IsActive(motor_id)
        || Get_Mos_Temp(motor) != E_OK
        || motor->mos_temp > MOS_TEMP_RECOVER_C
        || !MotorService_CurrentSenseIsValid(motor))
        return 0u;

    if (g_motor_fault_bits & (ADC_FAULT | CALIBRATION_FAULT | INTERNAL_FAULT))
        return 0u;

    motor_mask = (motor_id == 0u) ? MOTOR0_ENCODER_FAULT : MOTOR1_ENCODER_FAULT;
    if ((g_motor_fault_bits & motor_mask) && Motor_CheckEncoder(motor) != E_OK)
        return 0u;

    return 1u;
}
