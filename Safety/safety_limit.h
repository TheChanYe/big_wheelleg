/**
 ******************************************************************************
 * @file     safety_limit.h
 * @brief    Wheel current command limit and slew control.
 ******************************************************************************
 */

#ifndef SAFETY_LIMIT_H
#define SAFETY_LIMIT_H

#include "main.h"

#define MOTOR_COMMAND_COUNT                 2u
#define MOTOR_COMMAND_IQ_LIMIT_A             0.8f
#define MOTOR_COMMAND_IQ_SLEW_A_PER_S        8.0f

/* 初始保护阈值，需结合 NTC 与 PCB 热测试复核。 */
#define MOS_TEMP_WARN_C                      70.0f
#define MOS_TEMP_DERATE_C                    80.0f
#define MOS_TEMP_FAULT_C                     90.0f
#define MOS_TEMP_DERATE_MIN_RATIO            0.25f
#define MOS_TEMP_FAULT_CONFIRM_COUNT          3u
#define MOS_TEMP_RECOVER_C                    75.0f /* 需后续热负载测试复核。 */
#define TEMP_SENSOR_FAULT_CONFIRM_COUNT       3u

void SafetyLimit_Init(void);
float MotorCommand_UpdateIq(float current, float target, float max_delta);
float SafetyLimit_UpdateIq(uint8_t motor_id, float command_iq,
                           uint8_t command_enabled);
void SafetyLimit_SetIqLimit(uint8_t motor_id, float limit_a);
void SafetyLimit_ForceZero(uint8_t motor_id);
void SafetyLimit_ForceZeroFromISR(uint8_t motor_id);
void SafetyLimit_ForceZeroAll(void);

#endif /* SAFETY_LIMIT_H */
