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

#ifdef __cplusplus
/** 不持有状态；滤波历史仍由原 LPF_Handle 保存，兼容 C 调用方。 */
class LowpassFilter final {
public:
    LowpassFilter() = delete;
    static int init(LPF_Handle& filter, float Tf);
    static int update(LPF_Handle& filter, float input, float& output);
};
extern "C" {
#endif

/*-------------------- 函数声明 --------------------*/
int LPF_Init(LPF_Handle* filter, float Tf);
int LPF_Update(LPF_Handle* filter, float input, float* output);
#ifdef __cplusplus
}
#endif
#endif

