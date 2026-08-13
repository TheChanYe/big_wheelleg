/**
 ******************************************************************************
 * @file     motor_fault.h
 * @brief    Latched motor safety fault manager.
 ******************************************************************************
 */

#ifndef MOTOR_FAULT_H
#define MOTOR_FAULT_H

#include "foc_cfg.h"

typedef uint32_t MotorFaultBits;

#define MOTOR0_ENCODER_FAULT       (1u << 0)
#define MOTOR1_ENCODER_FAULT       (1u << 1)
#define MOTOR0_DRV_FAULT           (1u << 2)
#define MOTOR1_DRV_FAULT           (1u << 3)
#define MOTOR0_OVERCURRENT         (1u << 4)
#define MOTOR1_OVERCURRENT         (1u << 5)
#define MOTOR0_OVERTEMP            (1u << 6)
#define MOTOR1_OVERTEMP            (1u << 7)
#define VBUS_UNDERVOLTAGE          (1u << 8)
#define VBUS_OVERVOLTAGE           (1u << 9)
#define CAN_TIMEOUT_FAULT          (1u << 10)
#define CAN_SEQUENCE_FAULT         (1u << 11)
#define ADC_FAULT                  (1u << 12)
#define CALIBRATION_FAULT          (1u << 13)
#define CONTROL_FAULT              (1u << 14)
#define INTERNAL_FAULT             (1u << 15)

void MotorFault_Init(void);
void MotorFault_Enter(uint8_t motor_id, Motor_Data *motor, MotorFaultBits bits);
void MotorFault_EnterFromISR(uint8_t motor_id, Motor_Data *motor,
                             MotorFaultBits bits);
MotorFaultBits MotorFault_GetBits(void);
uint8_t MotorFault_MotorHasFault(uint8_t motor_id);
uint8_t MotorFault_ClearMotor(uint8_t motor_id);

#endif /* MOTOR_FAULT_H */
