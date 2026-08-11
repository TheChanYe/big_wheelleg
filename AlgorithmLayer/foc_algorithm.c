#include "foc_algorithm.h"
#define MODULE_NAME  "foc_algorithm"

#ifdef  MODE_LOG_TAG
#undef  MODE_LOG_TAG
#endif
#define MODE_LOG_TAG          MODULE_NAME
/**********************************************************************************************************
Clarke变换 输入Ia,Ib得到Ialpha和Ibeta
**********************************************************************************************************/ 
/***************************************
Clark变换 
输入参数：三相电流 Ia Ib Ialpha Ibeta
将输入的三相互差120度电流转换为两相互差90度电流
***************************************/
void Clark_Transf(float Ia,float Ib,float Ic,float *Ialpha,float *Ibeta)
{
	*Ialpha = Ia;
	*Ibeta = (Ia + 2*Ib ) / SQRT3; // (Ib * (sqrt3 / 2.0f) - Ic * (sqrt3 / 2.0f)) * 2.0f / 3.0f;
}
/***************************************
功能：PARK变换
形参：alpha_beta电流、theta值、DQ轴电流
说明：交流变直流
***************************************/
void Park_Transf(float Ialpha,float Ibeta,float theta,float *Id,float *Iq)
{
	*Id = Ialpha * arm_cos_f32(theta) + Ibeta * arm_sin_f32(theta);
	*Iq =	Ibeta * arm_cos_f32(theta)  - Ialpha * arm_sin_f32(theta);
}
/***************************************
功能：反PARK变换
形参：DQ轴电压、theta值、alpha_beta电压
说明：直流变交流
***************************************/
void RevParkOperate(float vd,float vq,float theta,float *valpha,float *vbeta)
{
	*valpha=vd*arm_cos_f32(theta)-vq*arm_sin_f32(theta);
	*vbeta=vd*arm_sin_f32(theta)+vq*arm_cos_f32(theta);
}

// 计算电流采样时间点的函数
static uint16_t CalculateSamplingTime(Motor_Data *motor,uint16_t ccr_ref, uint16_t ccr_other1) {
    uint16_t ccr4;
    uint16_t hDeltaDuty;

    // 如果参考点距离PWM周期结束足够远，则固定采样时刻
    if ((TMR_PR - ccr_ref) > TW_AFTER) {
        ccr4 = TMR_PR - 1;  // 固定采样时刻
    } else {
        // 计算占空比差异
        hDeltaDuty = ccr_ref - ccr_other1;
        if (hDeltaDuty > (TMR_PR - ccr_ref) * 2) {
            // 如果占空比差异较大，采样时刻向前调整
            ccr4 = ccr_ref - TW_BEFORE;
        } 
				else {
            // 否则，采样时刻向后调整
            ccr4 = ccr_ref + TW_BEFORE;
            if (ccr4 >= TMR_PR) {
                // 如果采样时刻超出PWM周期，则反向调整
                motor->pwm4_polarity = PWM1_MODE;
                ccr4 = (2 * TMR_PR) - ccr4 - 1;
            }
        }
    }
    return ccr4;
}
/***************************************
功能：SVPWM计算
形参：alpha_beta电压以及母线电压、定时器周期
说明：根据alpha_beta电压计算三相占空比
***************************************/
// SVPWM计算函数
void SvpwmAlgorithm(Motor_Data *motor, float valpha, float vbeta, float udc, uint32_t tpwm) 
{
    float ta = 0.0f;  // 矢量作用时间A
    float tb = 0.0f;  // 矢量作用时间B
    float value1 = 0.0f, value2 = 0.0f, value3 = 0.0f;  // 三相占空比
    uint8_t sector = 0;  // 扇区号
	
//    // 启动阶段特殊处理
//    const float MIN_VOLTAGE_THRESHOLD = 0.001f;  // 最小电压阈值
//    static bool motor_starting = true;
//    static uint32_t start_counter = 0;
//    const uint32_t START_DURATION = 5;  // 启动持续时间(单位:调用次数)
//    
//    if (motor_starting) {
//        // 启动阶段: 如果输入电压太小,使用固定的启动电压矢量
//        if (fabs(valpha) < MIN_VOLTAGE_THRESHOLD && fabs(vbeta) < MIN_VOLTAGE_THRESHOLD) {
//            // 使用预设的启动电压矢量(例如: 第一扇区)
//            valpha = MIN_VOLTAGE_THRESHOLD;
//            vbeta = MIN_VOLTAGE_THRESHOLD;
//            
//            // 启动计数器递增
//            start_counter++;
//            if (start_counter >= START_DURATION) {
//                motor_starting = false;  // 启动完成
//            }
//        } else {
//            motor_starting = false;  // 输入电压足够大,跳过启动阶段
//        }
//    }		
//		
    /******************** 扇区判断 ***********************/
    if (vbeta > 0) sector = 1;  // 判断是否在第一扇区
    if ((valpha * SQRT3_2 - vbeta / 2.0f) > 0) sector += 2;  // 判断是否在第二扇区
    if ((-valpha * SQRT3_2 - vbeta / 2.0f) > 0) sector += 4;  // 判断是否在第三扇区

    /******************** 矢量时间计算 ********************/
    switch (sector) {
        case 3:  // 第一扇区
            ta = (SQRT3 * tpwm) / udc * (SQRT3_2 * valpha - 0.5f * vbeta);
            tb = (SQRT3 * tpwm) / udc * vbeta;
            break;
        case 1:  // 第二扇区
            ta = -((SQRT3 * tpwm) / udc * (SQRT3_2 * valpha - 0.5f * vbeta));
            tb = -((SQRT3 * tpwm) / udc * (-SQRT3_2 * valpha - 0.5f * vbeta));
            break;
        case 5:  // 第三扇区
            ta = (SQRT3 * tpwm) / udc * vbeta;
            tb = (SQRT3 * tpwm) / udc * (-SQRT3_2 * valpha - 0.5f * vbeta);
            break;
        case 4:  // 第四扇区
            ta = -((SQRT3 * tpwm) / udc * vbeta);
            tb = -((SQRT3 * tpwm) / udc * (SQRT3_2 * valpha - 0.5f * vbeta));
            break;
        case 6:  // 第五扇区
            ta = (SQRT3 * tpwm) / udc * (-SQRT3_2 * valpha - 0.5f * vbeta);
            tb = (SQRT3 * tpwm) / udc * (SQRT3_2 * valpha - 0.5f * vbeta);
            break;
        case 2:  // 第六扇区
            ta = -((SQRT3 * tpwm) / udc * (-SQRT3_2 * valpha - 0.5f * vbeta));
            tb = -((SQRT3 * tpwm) / udc * vbeta);
            break;
    }

    /******************** 占空比计算 ********************/
    float temp = ta + tb;
    if (temp > tpwm) {
        // 如果矢量作用时间超过PWM周期，则按比例缩放
        ta = ta / temp * tpwm;
        tb = tb / temp * tpwm;
    }
    value1 = (tpwm - ta - tb) / 4.0f;  // 第一相占空比
    value2 = value1 + ta / 2.0f;       // 第二相占空比
    value3 = value2 + tb / 2.0f;       // 第三相占空比

    /******************** 计算CCR值及电流采样时间点 ********************/
    uint16_t ccr1, ccr2, ccr3, ccr4;
    switch (sector) {
        case 3: ccr1 = value1; ccr2 = value2; ccr3 = value3; ccr4 = CalculateSamplingTime(motor,ccr2, ccr3); break;
        case 1: ccr1 = value2; ccr2 = value1; ccr3 = value3; ccr4 = CalculateSamplingTime(motor,ccr1, ccr2); break;
        case 5: ccr1 = value3; ccr2 = value1; ccr3 = value2; ccr4 = CalculateSamplingTime(motor,ccr3, ccr1); break;
        case 4: ccr1 = value3; ccr2 = value2; ccr3 = value1; ccr4 = CalculateSamplingTime(motor,ccr3, ccr2); break;
        case 6: ccr1 = value2; ccr2 = value3; ccr3 = value1; ccr4 = CalculateSamplingTime(motor,ccr1, ccr3); break;
        case 2: ccr1 = value1; ccr2 = value3; ccr3 = value2; ccr4 = CalculateSamplingTime(motor,ccr2, ccr1); break;
    }

    /******************** 设置PWM通道4的极性 ********************/
    if (motor->pwm4_polarity == PWM2_MODE) {
        tmr_output_channel_polarity_set(motor->tmr, TMR_SELECT_CHANNEL_4, TMR_POLARITY_ACTIVE_HIGH);  // 设置高极性
    } else {
        tmr_output_channel_polarity_set(motor->tmr, TMR_SELECT_CHANNEL_4, TMR_POLARITY_ACTIVE_LOW);   // 设置低极性
    }

    /******************** 写入CCR值 ********************/
    tmr_channel_value_set(motor->tmr, TMR_SELECT_CHANNEL_1, ccr1);
    tmr_channel_value_set(motor->tmr, TMR_SELECT_CHANNEL_2, ccr2);
    tmr_channel_value_set(motor->tmr, TMR_SELECT_CHANNEL_3, ccr3);
		if(ccr4 > (TMR_PR - 1) )
			ccr4 = TMR_PR - 1;		
    tmr_channel_value_set(motor->tmr, TMR_SELECT_CHANNEL_4,ccr4);

    // 调试信息（可选）
    // log_inform("%d %d %d %d", ccr1, ccr2, ccr3, ccr4);
}
