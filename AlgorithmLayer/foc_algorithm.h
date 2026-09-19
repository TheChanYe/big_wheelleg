#ifndef	FOC_ALGORITHM_H
#define	FOC_ALGORITHM_H
#include "foc_cfg.h"

#ifdef __cplusplus
/** FOC 数学与 SVPWM 运算；不持有电机状态，不改变 Motor_Data 布局。 */
class FocAlgorithm final {
public:
    FocAlgorithm() = delete;
    static void clarke(float ia, float ib, float ic, float& alpha, float& beta);
    static void park(float alpha, float beta, float theta, float& d, float& q);
    static void inversePark(float d, float q, float theta, float& alpha, float& beta);
    static void svpwm(Motor_Data& motor, float alpha, float beta, float bus_voltage,
                      uint32_t period);
};
extern "C" {
#endif

/*clark变换*/
void Clark_Transf(float Ia,float Ib,float Ic,float *Ialpha,float *Ibeta);
/*Park变换*/
void Park_Transf(float Ialpha,float Ibeta,float theta,float *Id,float *Iq);
/*Park反变换*/
void RevParkOperate(float vd,float vq,float theta,float *valpha,float *vbeta);
/*SVPWM*/
void SvpwmAlgorithm(Motor_Data *motor, float valpha, float vbeta, float udc, uint32_t tpwm);
#ifdef __cplusplus
}
#endif
#endif

