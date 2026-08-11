#include "pid.h"
#define MODULE_NAME  "pid"

#ifdef  MODE_LOG_TAG
#undef  MODE_LOG_TAG
#endif
#define MODE_LOG_TAG          MODULE_NAME

/*PID参数初始化*/
int PID_Init(PID_Handle* pid, PID_Params* pid_params)
{
	if (pid == NULL || pid_params == NULL)
	{
		log_error("Parameter error!");
		return E_PARAM;
	}
	pid->kp = pid_params->kp;
	pid->ki = pid_params->ki;
	pid->kd = pid_params->kd;
	pid->integral_max = pid_params->integral_max;
	pid->output_max = pid_params->output_max;
	pid->error = 0.0f;
	pid->integral = 0.0f;
	pid->prev_error = 0.0f;

	return E_OK;
}

/* PID计算函数（含抗积分饱和） */
int PID_Calc(PID_Handle* pid, float target, float current, float* output)
{
    if (pid == NULL || output == NULL) {
		log_error("Parameter error!");
        return E_PARAM;
    }

    // 计算误差
    pid->error = target - current;

    // 计算比例项
    const float p_term = pid->kp * pid->error;

    // 计算积分项（自动限幅）
    pid->integral += pid->error * pid->ki;
    pid->integral = fminf(fmaxf(pid->integral, -pid->integral_max), pid->integral_max);

    // 计算微分项（仅当kd非零时启用）
    float d_term = 0.0f;
    if (pid->kd != 0.0f) {
        d_term = pid->kd * (pid->error - pid->prev_error);
        pid->prev_error = pid->error; // 更新历史误差
    }

    // 计算总输出（未限幅）
    const float unclamped_output = p_term + pid->integral + d_term;

    // 输出限幅
    const float clamped_output = fminf(fmaxf(unclamped_output, -pid->output_max), pid->output_max);
	pid->output = clamped_output;
    // 抗积分饱和修正
    if (unclamped_output != clamped_output) {
        // 反向修正积分项
        pid->integral = clamped_output - p_term - d_term;
        pid->integral = fminf(fmaxf(pid->integral, -pid->integral_max), pid->integral_max);
    }

    *output = clamped_output;

	return E_OK;
}

/* PID复位 — 清零所有历史状态，输出归零 */
int PID_Reset(PID_Handle* pid)
{
    if (pid == NULL)
    {
        log_error("PID_Reset: null pointer");
        return E_PARAM;
    }
    pid->error      = 0.0f;
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
    return E_OK;
}
