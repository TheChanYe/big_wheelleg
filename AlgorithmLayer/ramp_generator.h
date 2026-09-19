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

#ifdef __cplusplus
/** 使用原有 Ramp_Handle 作为状态，保持 Motor_Data 布局不变。 */
class RampGenerator final {
public:
    RampGenerator() = delete;
    static int init(Ramp_Handle& ramp, float gain);
    static int execute(Ramp_Handle& ramp, float current, float target,
                       float& output);
};
extern "C" {
#endif

/*-------------------- 函数声明 --------------------*/
int Ramp_Init(Ramp_Handle* ramp, float gain);
int Ramp_Execute(Ramp_Handle* ramp, float current,float target, float* p_current);
#ifdef __cplusplus
}
#endif
#endif

