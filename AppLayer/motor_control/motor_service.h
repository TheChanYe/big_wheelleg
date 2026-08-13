/**
 * 文件用途：双电机业务服务接口。
 * 所属层级：AppLayer/motor_control。
 * 主要职责：将电机任务中的初始化和每次 ADC 通知后的控制调用与任务调度分离。
 */

#ifndef MOTOR_SERVICE_H
#define MOTOR_SERVICE_H

#include "main.h"

/** 功能：初始化指定电机并进入原有 RUN 状态；参数：物理电机号；返回值：初始化结果。 */
int MotorService_Init(uint8_t motor_id);

/** 功能：执行指定电机一次原有控制循环；参数：物理电机号；返回值：控制结果。 */
int MotorService_Run(uint8_t motor_id);
int MotorService_ClearFault(uint8_t motor_id);
void MotorService_RecordControlCycle(uint8_t motor_id, uint32_t notifications);
uint32_t MotorService_GetControlCount(uint8_t motor_id);
uint32_t MotorService_GetControlOverrunCount(uint8_t motor_id);

#endif
