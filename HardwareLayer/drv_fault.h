/**
 ******************************************************************************
 * @file     drv_fault.h
 * @brief    DRV8301 nFAULT EXTI handling and deferred diagnostics.
 ******************************************************************************
 */

#ifndef DRV_FAULT_H
#define DRV_FAULT_H

#include "main.h"

int DrvFault_Init(void);
void DrvFault_MotorReady(uint8_t motor_id);
void DrvFault_Process(void);
void DrvFault_NotifyFromISR(uint8_t motor_id);
uint8_t DrvFault_IsActive(uint8_t motor_id);
uint16_t DrvFault_GetStatus1(uint8_t motor_id);
uint16_t DrvFault_GetStatus2(uint8_t motor_id);

#endif /* DRV_FAULT_H */
