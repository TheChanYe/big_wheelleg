#ifndef	PID_H
#define	PID_H
#include "main.h"

/*PID结构体*/
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float error;        // 当前误差
    float integral;     // 积分累积量
    float prev_error;   // 上一次误差（用于微分项）
    float integral_max; // 积分限幅
    float output_max;   // 输出限幅
    float output;   // 输出限幅
} PID_Handle;

// PID参数配置结构体（用于初始化）
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_max; // 积分限幅
    float output_max;   // 输出限幅
} PID_Params;

/*PID初始化*/
int PID_Init(PID_Handle* pid, PID_Params* pid_params);

/*PID*/
int PID_Calc(PID_Handle* pid, float target, float current, float* output);

/*PID复位 — 清零误差、积分、微分历史、输出*/
int PID_Reset(PID_Handle* pid);
#endif

