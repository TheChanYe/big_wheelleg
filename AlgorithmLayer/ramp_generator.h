#ifndef	RAMP_GENERATOR_H
#define	RAMP_GENERATOR_H
#include "main.h"
/*-------------------- 斜坡控制结构体 --------------------*/
typedef struct {
    float gain;          // 斜坡速率每次步进值增加多少
	float dt;//时间常速，每隔多久变化一次
	float error;//误差
	float current;//当前设定值
    TickType_t prev_tick; // 上次更新时间戳
} Ramp_Handle;

/*-------------------- 函数声明 --------------------*/
int Ramp_Init(Ramp_Handle* ramp, float gain);
int Ramp_Execute(Ramp_Handle* ramp, float current,float target, float* p_current);
#endif

