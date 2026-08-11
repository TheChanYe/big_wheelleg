#ifndef	LOWPASS_FILTER_H
#define	LOWPASS_FILTER_H
#include "main.h"
/*-------------------- 滤波器结构体 --------------------*/
typedef struct {
    float Tf;               // 滤波器时间常数（秒）
    TickType_t prev_tick;    // 上次时间戳
    float y_prev;           // 上次输出值
    float alpha;            // 动态计算的滤波系数
} LPF_Handle;

/*-------------------- 函数声明 --------------------*/
int LPF_Init(LPF_Handle* filter, float Tf);
int LPF_Update(LPF_Handle* filter, float input, float* output);
#endif

