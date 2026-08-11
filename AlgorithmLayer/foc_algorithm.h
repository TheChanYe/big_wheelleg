#ifndef	FOC_ALGORITHM_H
#define	FOC_ALGORITHM_H
#include "foc_cfg.h"

/*clark变换*/
void Clark_Transf(float Ia,float Ib,float Ic,float *Ialpha,float *Ibeta);
/*Park变换*/
void Park_Transf(float Ialpha,float Ibeta,float theta,float *Id,float *Iq);
/*Park反变换*/
void RevParkOperate(float vd,float vq,float theta,float *valpha,float *vbeta);
/*SVPWM*/
void SvpwmAlgorithm(Motor_Data *motor, float valpha, float vbeta, float udc, uint32_t tpwm);
#endif

