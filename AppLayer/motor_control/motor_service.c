/**
 * 文件用途：双电机业务服务实现。
 * 所属层级：AppLayer/motor_control。
 * 主要职责：保持 FOC 调用、CAN 命令选择和故障切换行为不变。
 */

#include "motor_service.h"
#include "foc.h"
#include "can_business.h"

/* 历史命名保持不变：g_motor1=物理 Motor0/左轮/TMR1，g_motor2=物理 Motor1/右轮/TMR8。 */
Motor_Data g_motor1 = {0};
Motor_Data g_motor2 = {0};

static Motor_Data *MotorService_GetMotor(uint8_t motor_id)
{
    return (motor_id == 0u) ? &g_motor1 : (motor_id == 1u) ? &g_motor2 : NULL;
}

int MotorService_Init(uint8_t motor_id)
{
    Motor_Data *motor = MotorService_GetMotor(motor_id);
    int ret;

    if (motor == NULL)
        return E_PARAM;

    ret = Motor_Init((motor_id == 0u) ? MOTOR_1 : MOTOR_2);
    if (ret == E_OK)
        motor->run_state = RUN;
    return ret;
}

int MotorService_Run(uint8_t motor_id)
{
    Motor_Data *motor = MotorService_GetMotor(motor_id);
    int ret;

    if (motor == NULL)
        return E_PARAM;

    if (motor->run_state == RUN)
    {
        if (can_business_get_motor_mode() == CAN_MOTOR_MODE_SPEED)
        {
            ret = CascadeControl_Run(motor, Speed_loop,
                (motor_id == 0u) ? can_business_get_motor0_speed_target()
                                  : can_business_get_motor1_speed_target());
        }
        else
        {
            ret = CascadeControl_Run(motor, Current_loop,
                (motor_id == 0u) ? can_business_get_motor0_iq_output()
                                  : can_business_get_motor1_iq_output());
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
        motor->run_state = FAULT;
    return ret;
}
