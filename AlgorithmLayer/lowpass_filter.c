#include "lowpass_filter.h"

#define MODULE_NAME  "lowpass_filter"

#ifdef  MODE_LOG_TAG
#undef  MODE_LOG_TAG
#endif
#define MODE_LOG_TAG          MODULE_NAME

/**
  * @brief  初始化低通滤波器
  * @param  filter  滤波器实例指针
  * @param  Tf      滤波器时间常数（单位：秒）
  */
int LPF_Init(LPF_Handle* filter, float Tf)
{
    if(filter == NULL) 
 	{
		log_error("Parameter error!");	
		return E_PARAM;
	}   
    
    filter->Tf = Tf;
    filter->prev_tick = xTaskGetTickCount(); // 记录初始时间戳
    filter->y_prev = 0.0f;                  // 初始输出归零
    filter->alpha = 0.0f;                   // 首次运行时计算系数
    return E_OK;
}

/**
  * @brief  执行低通滤波计算
  * @param  filter  滤波器实例指针
  * @param  input   输入原始值
  * @retval         滤波后的输出值
  */
int LPF_Update(LPF_Handle* filter, float input, float* output)
{
    if(filter == NULL || output == NULL) 
 	{
		log_error("Parameter error!");	
		return E_PARAM;
	}   
    
    // 获取当前时间戳
    TickType_t current_tick = xTaskGetTickCount();
    
    // 计算时间差（自动处理tick溢出）
    TickType_t tick_diff = current_tick - filter->prev_tick;
    // 检查是否是第一次调用
    if (filter->prev_tick == 0) {
        tick_diff = 0; // 设置初始 tick 差值为 0

    }
	
    float dt = (float)tick_diff * portTICK_PERIOD_MS / 1000.0f;
    
    // 处理异常时间间隔
    if(dt <= 0.0f) {
        dt = 1e-3f;  // 最小时间间隔保护
    } else if(dt > 0.3f) {
        // 长时间未更新时直接使用新值
        filter->y_prev = input;
        filter->prev_tick = current_tick;
        *output = input;
    }
    
    // 动态计算滤波系数
    filter->alpha = filter->Tf / (filter->Tf + dt);
    
    // 执行滤波计算
    *output = filter->alpha * filter->y_prev + (1.0f - filter->alpha) * input;
    
    // 更新滤波器状态
    filter->y_prev = *output;
    filter->prev_tick = current_tick;
    
    return E_OK;
}

