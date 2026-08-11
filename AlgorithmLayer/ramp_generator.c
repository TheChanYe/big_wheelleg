#include "ramp_generator.h"
#define MODULE_NAME  "ramp_generator"

#ifdef  MODE_LOG_TAG
#undef  MODE_LOG_TAG
#endif
#define MODE_LOG_TAG          MODULE_NAME

/**
  * @brief  初始化斜坡控制器
  * @param  ramp  斜坡实例指针
  * @param  gain  最大变化速率（单位/秒）
  */
int Ramp_Init(Ramp_Handle* ramp, float gain)
{
    if(ramp == NULL)
 	{
		log_error("Parameter error!");	
		return E_PARAM;
	}   
    ramp->gain = (gain > 0) ? gain : 1.0f;  // 确保增益有效
	ramp->dt = 2;
    ramp->prev_tick = xTaskGetTickCount();    // 记录初始时间
	return E_OK;
}

/**
  * @brief  执行斜坡计算（直接操作外部当前值）
  * @param  ramp       斜坡实例指针
  * @param  current    当前值（用户输入）
  * @param  target     目标设定值（用户输入）
  * @param  p_current  斜坡输出值（输入输出参数）
  */
int Ramp_Execute(Ramp_Handle* ramp, float current,float target, float* p_current)
{
    // 参数有效性检查
    if(ramp == NULL || p_current == NULL) 
	{
		log_error("Parameter error!");	
		return E_PARAM;
	}
	if(ramp->current != target && ramp->error==0)
	{
		ramp->current = current;
		ramp->error = fabs(target) + fabs(current);
	}
	else
	{
		if(ramp->dt==0)//处理差值
		{
			ramp->dt=2;
			if(ramp->current < target)	
			{		
				ramp->current = ramp->current + ramp->gain;
				ramp->error-=ramp->gain;
				*p_current = ramp->current;
				if(ramp->current>=target)
				{
					ramp->current = target;
					*p_current = target;
					ramp->error=0;
				}
			}
			else if(ramp->current > target)	
			{		
				ramp->current = ramp->current-=ramp->gain;
				ramp->error-=ramp->gain;
				*p_current = ramp->current;
				if(ramp->current<=target)
				{
					ramp->current = target;
					*p_current = target;
					ramp->error=0;
				}
			}
			else//当前设定值与目标值一样直接返回目标值
			{
				*p_current = ramp->current;
			}		
		}
		else
		{
			ramp->dt--;
			*p_current = ramp->current;
		}
	}
	return E_OK;
}

