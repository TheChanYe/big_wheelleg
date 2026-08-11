#ifndef	_FOC_H
#define	_FOC_H
#include "foc_cfg.h"
#include "motor_adc.h"
#include "motor_timr.h"
#include "as5047p.h"
#include "drv8301.h"
#include "foc_algorithm.h"
#include "my_flash.h"

/*FOC程序*/
int CascadeControl_Run(Motor_Data* motor, Motor_Mode mode, float target);
int Motor_Init(Motor_Type motor);
int Close_Motor(Motor_Data* motor);
int Open_Motor(Motor_Data* motor);
int Get_Mos_Temp(Motor_Data* motor);
int Motor_Update_Speed(Motor_Data *motor);
///*获取电机速度*/
//float GetMotorPreSpeed(void);

#endif

